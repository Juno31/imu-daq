"""Pair with one GoPro over Bluetooth and read its status (no recording).
Usage: python pair_test.py   (camera on, in pairing mode the first time)"""
import time
import gopro_ble as g

cams = g.scan(6.0)
print("found:", cams or "none")
for addr, name in cams:
    c = g.BleGoPro(addr, name)
    t = time.time()
    try:
        c.connect()
        print(f"{name}: paired + connected in {time.time() - t:.1f} s")
        c.keep_alive()
        s = c.poll()
        print(f"{name}: battery {s.get('70')}%, SD free {s.get('54', 0) / 1e6:.1f} GB, recording {bool(s.get('10'))}")
    except Exception as e:
        print(f"{name}: FAILED - {e}")
    finally:
        c.close()
