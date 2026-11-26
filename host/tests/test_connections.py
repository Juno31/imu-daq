"""The Connections view's evidence: device.py keeps the SID Lab firmware's per-pod and per-bus lines.

    python host/tests/test_connections.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import device                       # noqa: E402

PODS = ["C7", "T3", "T6", "T9", "T11", "L1", "L3", "S1"]


def feed(dev, *lines):
    for ln in lines:
        assert dev._handle_sidlab(ln, 0.0), ln


def main():
    sent = []
    dev = device.Device(lambda *a: None, lambda *a: None, lambda *a: None, lambda *a: None)
    dev.send_raw = sent.append

    # power-up scan arrives before the column header (board not identified yet): must still be kept
    feed(dev, "# SID Lab Teensy 4.1 spine IMU logger (8 pods, 2 muxes)")
    assert sent == ["h"]
    feed(dev, "# C7: OK (mux A ch0, report every 7 ms)",
         "# T3: mux A did not answer at 0x70",
         "# T6: no BNO086 on mux A ch2",
         "# T9: answered but sent no quaternion after the start command",
         "# T11: OK (mux B ch0, report every 7 ms)")
    cols = "F,frame,t_us," + ",".join(f"{p}_ok,{p}_fresh,{p}_w,{p}_x,{p}_y,{p}_z" for p in PODS) + "," + \
        ",".join(f"{p}_age_us" for p in PODS)
    feed(dev, "# columns: " + cols)
    assert dev.proto == "sidlab"
    d = dev.pod_diag
    assert d["C7"]["state"] == "ok" and d["T3"]["state"] == "no_mux" and d["T6"]["state"] == "no_imu"
    assert d["T9"]["state"] == "no_quat" and d["T11"]["state"] == "ok"
    assert dev.pod_where["C7"] == "A0" and dev.scan_t is not None
    assert dev.bus["B"]["mux"] == "ok"

    # running: a pod drops out and comes back; a bus pauses; I2C counters every 2 s
    feed(dev, "# L1: no answer (5 errors in a row), retrying every 0.2 s")
    assert d["L1"]["state"] == "lost"
    feed(dev, "# L1: answering again")
    assert d["L1"]["state"] == "ok"
    feed(dev, "# bus B: 5 errors in a row, pausing it 1 s")
    assert "pausing" in dev.bus["B"]["paused"]
    hz = " ".join(f"{p} 138.0" for p in PODS)
    feed(dev, f"# Hz: {hz} | I2C err A 3 B 10 | read A 1400 us B 1500 us | bus busy A 80% B 90% | max loop 900 us | late 0")
    feed(dev, f"# Hz: {hz} | I2C err A 3 B 17 | read A 1400 us B 1500 us | bus busy A 80% B 90% | max loop 900 us | late 0")
    assert dev.bus["A"]["err_new"] == 0 and dev.bus["B"]["err_new"] == 7 and dev.bus["B"]["busy_pct"] == 90
    assert dev.hz["C7"] == 138.0

    # 's' status: a pod the scan called OK but now down
    feed(dev, "# S1 ok=0 updates=0 empty_reads=5 fresh_frames=0.0%")
    assert d["S1"]["state"] == "down"
    # names outside the header are ignored
    feed(dev, "# XYZ: no BNO086 on mux A ch3")
    assert "XYZ" not in d
    print("connections parsing: OK")


if __name__ == "__main__":
    main()
