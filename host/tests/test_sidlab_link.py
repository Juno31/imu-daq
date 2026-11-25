"""Host against the SID Lab firmware, without hardware:

  1. server + device on a fake board (tests/fake_sidlab_board.py): connect, idle live frames, Start / Stop,
     SD file download, files written, no stray command letters sent to the board.
  2. DAQ side: frame-coded SYNC pulses and P0.0 packets (as DAQ samples) -> frame times, imu_daq.csv.

    python host/tests/test_sidlab_link.py
"""
import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import device                       # noqa: E402
import server                       # noqa: E402
import sidlab_link as sl            # noqa: E402
from fake_sidlab_board import FakeSidlabSerial, PODS, FRAME_US   # noqa: E402


def wait(cond, secs, what):
    end = time.time() + secs
    while time.time() < end:
        if cond():
            return
        time.sleep(0.05)
    raise AssertionError(f"timeout: {what}")


def test_server_with_fake_board(tmp):
    boards = []

    def fake_serial(*a, **k):
        b = FakeSidlabSerial(sd=True)
        boards.append(b)
        return b
    device.serial.Serial = fake_serial
    server.guess_port = lambda: "FAKE"
    app = server.App(os.path.join(tmp, "data"))
    app.gopro.cfg["enabled"] = False
    app.daq_cfg["enabled"] = False              # no NI DAQ on this PC: IMU-only recording
    wait(lambda: app.state == "idle", 5, "idle after connect")
    assert app.dev.proto == "sidlab", app.dev.proto
    assert app.dev.pod_names == PODS, app.dev.pod_names
    wait(lambda: app.recent_t and len(app.recent_t) > 50, 3, "live frames while idle")
    st = app.status()
    assert len(st["pods"]) == 8 and all(p["ok"] for p in st["pods"]), st["pods"]
    assert abs(int(app.dev.hello["frame_us"]) - FRAME_US) < 300, app.dev.hello
    assert app.session is None and not app.stray_noted, "idle frames must not look like a stray recording"
    wait(lambda: app.fw_status.get("status", "").startswith("Hz:"), 3, "status line")

    r = app.start({"subject": "1", "task": "test", "trial": "1", "date": "261125"})
    assert app.state == "recording"
    time.sleep(2.0)
    summ = app.stop()
    assert app.state == "idle", app.state
    print("summary:", {k: summ.get(k) for k in ("frames_received", "first_frame", "last_frame", "missing_frames",
                                                "duration_s", "rate_hz")})
    print("sd_card:", summ.get("sd_card"))
    assert summ["first_frame"] == 0, summ["first_frame"]
    assert summ["missing_frames"] == 0
    assert 200 < summ["frames_received"] < 300, summ["frames_received"]
    assert summ["sd_card"] and summ["sd_card"]["used"], summ.get("sd_card")
    assert summ["sd_card"]["frames"] == summ["frames_received"]
    d = r["dir"]
    head = open(os.path.join(d, "imu.csv")).readline().strip().split(",")
    assert head[:2] == ["frame", "t_us"] and "S1_age_us" in head and len(head) == 2 + 7 * 8, head
    row = open(os.path.join(d, "imu.csv")).readlines()[1].strip().split(",")
    assert len(row) == len(head) and row[-1] == "3600", row
    assert os.path.exists(os.path.join(d, "imu_sd.csv")) and os.path.exists(os.path.join(d, "imu_live.csv"))
    # live frames come back after Stop
    n0 = app.fseq
    time.sleep(0.5)
    assert app.fseq > n0 + 30, "frames after stop"
    sent = "".join(boards[-1].written)
    print("characters sent to the board:", repr(sent))
    # only "hello" (sent before the board is known; for this firmware e/l/o are no commands and h prints the header)
    # and the one-letter commands h v g p D
    assert sent.startswith("hello\n") and set(sent[6:]) <= set("hvgpD"), f"stray command letters: {sent!r}"
    # second trial: a new SD file, frame 0 again
    app.start({"subject": "1", "task": "test", "trial": "2", "date": "261125"})
    time.sleep(1.0)
    s2 = app.stop()
    assert s2["first_frame"] == 0 and s2["sd_card"]["file"] != summ["sd_card"]["file"], s2.get("sd_card")
    app.shutdown()
    print("server + fake board: OK")


def uart_samples(packets_bytes, starts_s, total_s, fs=sl.DI_FS, baud=sl.BAUD):
    """DAQ samples of the data line carrying each byte string from its start time (idle high)."""
    x = np.ones(int(total_s * fs), np.int8)
    tb = fs / baud
    for b, t0 in zip(packets_bytes, starts_s):
        bits = np.concatenate([np.r_[0, (c >> np.arange(8)) & 1, 1] for c in b])
        i0 = int(t0 * fs)
        for k, v in enumerate(bits):
            x[i0 + int(k * tb):i0 + int((k + 1) * tb)] = v
    return x


def packet(fr, t_us):
    import struct
    b = bytearray([0xA5, 0x5A]) + struct.pack("<IIBB", fr, t_us & 0xFFFFFFFF, 0xFF, 0xFF)
    for k in range(8):
        b += struct.pack("<4h", 16384, 0, 0, 0)
    for k in range(8):
        b += struct.pack("<i", 3600)
    s1 = s2 = 0
    for c in b[2:]:
        s1 = (s1 + c) & 255
        s2 = (s2 + s1) & 255
    return bytes(b + bytes([s1, s2]))


def test_daq_side(tmp):
    period = FRAME_US / 1e6
    frames = np.arange(0, 300)
    t_frame = 0.05 + frames * period * 1.00005          # Teensy clock vs DAQ clock: +50 ppm
    # data line through the chunked decoder, in uneven chunks as the DAQ reader would deliver them
    x = uart_samples([packet(int(f), int(1e6 + f * FRAME_US)) for f in frames], t_frame, t_frame[-1] + 0.02)
    dec, rows, i = sl.Decoder(), [], 0
    rng = np.random.default_rng(3)
    while i < len(x):
        n = int(rng.integers(50_000, 150_000))
        rows += dec.feed(x[i:i + n])
        i += n
    assert len(rows) == len(frames) and dec.bad == 0, (len(rows), dec.bad)
    note = sl.write_packets(os.path.join(tmp, "imu_daq.csv"), rows, PODS)
    assert note["packets"] == 300 and note["missing_frames"] == 0, note
    tab = sl.frame_time_table(rows)
    err = max(abs(tab[f] - t_frame[f]) for f in frames)
    assert err < 20e-6, f"packet frame time error {err * 1e6:.1f} us"
    # frame-coded SYNC at 20 kHz, the first 10 pulses missed (DAQ started late)
    rate = 20000
    pulses = [(int(round(t * rate)), int(sl.sync_width_ms(int(f)) * rate / 1000)) for f, t in zip(frames, t_frame)][10:]
    ft, rep = sl.align_coded(pulses, rate, frames, FRAME_US)
    assert rep["frames_without_pulse"] == 10 and not rep["ok"], rep
    ft, rep = sl.align_coded(pulses, rate, frames[10:], FRAME_US)
    assert rep["ok"], rep
    assert np.nanmax(np.abs(ft[10:300] - t_frame[10:])) < 1.0 / rate, "SYNC frame times"
    print("DAQ side (P0.0 packets, frame-coded SYNC): OK")


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as tmp:
        test_daq_side(tmp)
        test_server_with_fake_board(tmp)
    print("all OK")
