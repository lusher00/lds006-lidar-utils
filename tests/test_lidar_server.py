#!/usr/bin/env python3
"""Off-device tests for the lidar daemon.

Fakes the serial port, runs everything else for real — the framer, the
counters, the HTTP server — and checks what a browser would actually get.

    ./tests/test_lidar_server.py

No scanner needed, and it does not touch /dev/ttyAMA0, so it is safe to run on
the Pi with the service running.
"""

import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

import fake_serial                                    # noqa: E402
sys.modules["serial"] = fake_serial                   # before lidar_server imports it

PORT = int(os.environ.get("LIDAR_TEST_PORT", "8791"))
os.environ["LIDAR_HTTP_PORT"] = str(PORT)
os.environ["LIDAR_AUTOSTART"] = "1"

import lidar_server                                   # noqa: E402

BASE = f"http://127.0.0.1:{PORT}"
failures = []


def check(name, cond, detail=""):
    if cond:
        print(f"  ok    {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return r.status, dict(r.headers), json.loads(r.read() or b"null")


def post(path, obj):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(obj).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read() or b"null")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"null")


# ── the decoder, with no server in the way ──────────────────────────────
def test_decode():
    print("decode")
    s = lidar_server.Scanner()
    good = fake_serial.packet(0xA0, dist=[1000, 1001, 1002, 1003])
    check("checksum accepts a good packet", s.checksum_ok(good))
    bad = fake_serial.packet(0xA0, corrupt=True)
    check("checksum rejects a corrupted one", not s.checksum_ok(bad))

    s.ingest(good, time.monotonic())
    check("index 0xA0 writes angles 0-3",
          s.dist[0:4] == [1000, 1001, 1002, 1003], s.dist[0:4])
    last = fake_serial.packet(0xF9, dist=[7, 8, 9, 10])
    s.ingest(last, time.monotonic())
    check("index 0xF9 writes angles 356-359",
          s.dist[356:360] == [7, 8, 9, 10], s.dist[356:360])

    before = s.c["index_bad"]
    s.ingest(fake_serial.packet(0xFB), time.monotonic())
    check("an out-of-range index is counted, not stored",
          s.c["index_bad"] == before + 1)

    # A bin nobody has written since STALE_S is not a reading any more.
    s.stamp[0] = time.monotonic() - (lidar_server.STALE_S + 0.5)
    check("a stale bin reports -1", s.scan()["dist"][0] == -1)

    # The top two bits are flags, not distance — settled by measurement on
    # the real unit; see the note at the top of lidar_server.py.
    s2 = lidar_server.Scanner()
    s2.ingest(fake_serial.packet(0xA0, dist=[0x8123, 0x4123, 5, 0]), time.monotonic())
    check("bit15 counted as no return", s2.c["invalid"] == 1, s2.c)
    check("a no-return sample stores 0, not 33 m",
          s2.dist[0] == 0, s2.dist[0])
    check("bit14 counted as weak", s2.c["weak"] == 1, s2.c)
    check("a weak sample keeps its distance, masked to 14 bits",
          s2.dist[1] == 0x0123, s2.dist[1])
    check("zero counts both kinds of no-return", s2.c["zero"] == 2, s2.c)


# ── the whole daemon, over HTTP ─────────────────────────────────────────
def test_http():
    print("http")
    threading.Thread(target=lidar_server.SCANNER.run, daemon=True).start()
    threading.Thread(target=lidar_server.rate_thread, daemon=True).start()
    from http.server import ThreadingHTTPServer
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), lidar_server.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    time.sleep(1.6)                                   # one rate window

    code, hdrs, h = get("/health")
    check("/health answers", code == 200)
    check("CORS is open", hdrs.get("Access-Control-Allow-Origin") == "*", hdrs)
    check("the port opened", h["ok"] and not h["serial_error"], h.get("serial_error"))
    check("autostart spun the motor", h["motor"] == "running", h["motor"])
    check("packets are arriving", h["counts"]["packets"] > 100, h["counts"])
    check("rates are populated", h["rates"]["packets"] > 0, h["rates"])
    check("the raw speed field is passed through", h["speed_raw"] == 29946, h["speed_raw"])
    check("the module's own rpm is raw/100",
          abs(h["rpm_reported"] - 299.5) < 0.1, h["rpm_reported"])
    # The fake spins far faster than the real unit; what is being checked is
    # that the number is COUNTED from index wraps, not derived from the field.
    check("rpm is measured from index wraps",
          h["rpm"] > 0 and abs(h["rpm"] - h["rpm_reported"]) > 1, h["rpm"])
    check("spinning is true while wraps keep arriving", h["spinning"] is True, h)

    code, _, s = get("/scan")
    check("/scan answers", code == 200)
    check("360 distances", len(s["dist"]) == 360 and len(s["sig"]) == 360)
    check("every bin is fresh", s["points_fresh"] == 360, s["points_fresh"])
    check("revolutions counted", s["rev"] > 0, s["rev"])
    check("distances are the ramp the fake sends",
          s["dist"][0] == 1000 and s["dist"][10] == 1050, s["dist"][:11])

    code, j = post("/control", {"motor": "stop"})
    check("motor stop accepted", code == 200 and j["ok"] and j["motor"] == "stopped", j)
    check("stoplds$ went down the wire",
          lidar_server.SCANNER.ser.wrote(b"stoplds$"))
    code, j = post("/control", {"motor": "sideways"})
    check("a nonsense motor command is refused", code == 400, j)
    code, j = post("/nope", {})
    check("an unknown path is 404", code == 404, j)

    req = urllib.request.Request(BASE + "/scan", method="OPTIONS")
    with urllib.request.urlopen(req, timeout=5) as r:
        check("preflight is answered", r.status == 204)

    srv.shutdown()


def test_dirty_stream():
    """The framer, against a stream that is not clean."""
    print("dirty stream")
    fake_serial.Serial.inject_garbage = True
    fake_serial.Serial.inject_bad_csum = True
    fake_serial.Serial.inject_bad_index = True
    s = lidar_server.Scanner()
    threading.Thread(target=s.run, daemon=True).start()
    time.sleep(1.2)
    h = s.health()
    c = h["counts"]
    check("good packets still get through", c["packets"] > 100, c)
    check("bad checksums are counted", c["checksum_bad"] > 0, c)
    check("stray indices are counted", c["index_bad"] > 0, c)
    check("the scan still fills", h["points_fresh"] > 300, h["points_fresh"])
    fake_serial.Serial.inject_garbage = False
    fake_serial.Serial.inject_bad_csum = False
    fake_serial.Serial.inject_bad_index = False


def test_no_port():
    """A missing scanner is a reported state, not a crash."""
    print("no serial port")
    fake_serial.Serial.fail_open = OSError("could not open port /dev/nope")
    s = lidar_server.Scanner()
    threading.Thread(target=s.run, daemon=True).start()
    time.sleep(0.5)
    h = s.health()
    check("the daemon stays up", h["uptime_s"] >= 0)
    check("not ok", not h["ok"], h)
    check("the reason is reported", "could not open" in (h["serial_error"] or ""), h)
    fake_serial.Serial.fail_open = None


if __name__ == "__main__":
    test_decode()
    test_http()
    test_dirty_stream()
    test_no_port()
    print()
    if failures:
        print(f"{len(failures)} FAILED: {', '.join(failures)}")
        sys.exit(1)
    print("all checks passed")
