"""A stand-in for pyserial that replays synthetic LDS-006 traffic.

Installed into sys.modules as "serial" before lidar_server is imported, so the
daemon runs for real — same framer, same counters, same HTTP surface — with no
scanner and no serial port. That also means the suite can run while the service
has the real port open.

The stream can be told to misbehave, which is the point: a framer is only worth
anything if it survives a bad checksum, a stray index and a byte landing in the
middle of a packet.
"""

import threading
import time

IDX_FIRST = 0xA0
IDX_LAST = 0xF9


def packet(index, speed_raw=29946, dist=None, sig=320, corrupt=False):
    """One 22-byte packet.

    dist is four distance WORDS — low 14 bits mm, bit 15 no-return, bit 14
    weak — or None for a ramp of clean returns. Default speed_raw is what the
    real unit reports at ~298 RPM.
    """
    if dist is None:
        base = (index - IDX_FIRST) * 4
        dist = [1000 + (base + i) * 5 for i in range(4)]
    pkt = bytearray([0xFA, index, speed_raw & 0xFF, (speed_raw >> 8) & 0xFF])
    for d in dist:
        pkt += bytes([d & 0xFF, (d >> 8) & 0xFF, sig & 0xFF, (sig >> 8) & 0xFF])
    csum = sum(pkt) & 0xFFFF
    if corrupt:
        csum ^= 0xFFFF
    pkt += bytes([csum & 0xFF, (csum >> 8) & 0xFF])
    return bytes(pkt)


class Serial:
    """Only what lidar_server uses: read, write, flush, close, the buffer reset."""

    # Set before the daemon starts to steer what the fake stream does.
    inject_garbage = False      # a stray byte between packets
    inject_bad_csum = False     # every 10th packet fails its checksum
    inject_bad_index = False    # every 10th packet carries an out-of-range index
    fail_open = None            # an exception instance to raise from Serial()

    def __init__(self, port, baud, timeout=0.2):
        if Serial.fail_open:
            raise Serial.fail_open
        self.port = port
        self.baud = baud
        self.writes = []
        self.lock = threading.Lock()
        self._idx = IDX_FIRST
        self._n = 0

    def reset_input_buffer(self):
        pass

    def flush(self):
        pass

    def close(self):
        pass

    def write(self, data):
        with self.lock:
            self.writes.append(bytes(data))
        return len(data)

    def wrote(self, needle):
        with self.lock:
            return any(needle in w for w in self.writes)

    def read(self, n):
        time.sleep(0.002)          # a real port blocks; a busy loop starves the GIL
        out = bytearray()
        for _ in range(6):
            self._n += 1
            # Different moduli on purpose: a packet that is BOTH corrupted and
            # misindexed never reaches the index check, so the two faults have
            # to be injected on different beats to exercise both counters.
            bad_c = Serial.inject_bad_csum and self._n % 10 == 0
            bad_i = Serial.inject_bad_index and self._n % 13 == 0
            idx = 0xFB if bad_i else self._idx
            if Serial.inject_garbage and self._n % 7 == 0:
                out += b"\xFA\x00"   # looks like a header, is not a packet
            out += packet(idx, corrupt=bad_c)
            if not bad_i:
                self._idx = IDX_FIRST if self._idx >= IDX_LAST else self._idx + 1
        return bytes(out)
