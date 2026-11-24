"""EMG (NI-DAQ) samples of one recording as a CSV next to imu_synced.csv.

    python emg_export.py <recording folder> [<recording folder> ...]

Written automatically when a recording stops; run this by hand for older recordings.
Reads daq_raw.f32 + daq_meta.json and writes emg.csv, one row per DAQ sample:

    sample       DAQ sample index
    daq_time_s   sample / rate: the same clock as daq_time_s in imu_synced.csv
    imu_frame    IMU frame whose SYNC pulse came last at or before this sample (-1 before frame 0,
                 empty when the SYNC check failed). Join on imu_synced.csv "frame" for a merge.
    SYNC_V       the SYNC line (volts)
    EMG01_V ...  each EMG channel in volts at the DAQ input (raw, no filtering)
"""
import json
import os
import sys

import numpy as np


def export(folder, frame_times=None, out_name="emg.csv"):
    """Write <folder>/emg.csv. frame_times: DAQ time (s) of each IMU frame, NaN where missing.
    Returns the path, or None when the recording has no DAQ data."""
    mp = os.path.join(folder, "daq_meta.json")
    if not os.path.isfile(mp):
        return None
    with open(mp, encoding="utf-8") as f:
        meta = json.load(f)
    raw = os.path.join(folder, meta.get("file", "daq_raw.f32"))
    chans = meta.get("channels", [])
    if not chans or not os.path.isfile(raw):
        return None
    x = np.fromfile(raw, dtype=np.float32)
    n = len(x) // len(chans)
    if n == 0:
        return None
    x = x[: n * len(chans)].reshape(n, len(chans))
    rate = float(meta.get("rate_hz", 1.0))

    labels = {c.get("chan"): c.get("name") or c.get("chan") for c in meta.get("channel_labels", [])}
    names, seen = [], set()
    for c in chans:
        nm = labels.get(c, c)
        if nm in seen:
            nm = f"{nm}_{c}"
        seen.add(nm)
        names.append(f"{nm}_V")

    if frame_times is None:
        frame_times = _frame_times_from_imu(folder)
    sample = np.arange(n)
    t = sample / rate

    import pandas as pd
    df = pd.DataFrame({"sample": sample, "daq_time_s": np.round(t, 6)})
    if frame_times is not None and np.isfinite(frame_times).any():
        ft = np.asarray(frame_times, dtype=np.float64)
        fr = np.flatnonzero(np.isfinite(ft))
        pos = np.searchsorted(ft[fr], t + 0.5 / rate, side="right") - 1   # half-sample tolerance
        df["imu_frame"] = np.where(pos >= 0, fr[np.clip(pos, 0, None)], -1)
    for k, nm in enumerate(names):
        df[nm] = x[:, k]
    dst = os.path.join(folder, out_name)
    df.to_csv(dst, index=False, float_format="%.6f")
    return dst


def _frame_times_from_imu(folder):
    """DAQ time of each frame from imu_synced.csv (for re-exporting old recordings)."""
    p = os.path.join(folder, "imu_synced.csv")
    if not os.path.isfile(p):
        return None
    import pandas as pd
    df = pd.read_csv(p, usecols=lambda c: c in ("frame", "daq_time_s"))
    if "daq_time_s" not in df:
        return None
    fr = df["frame"].to_numpy()
    ok = (fr >= 0) & np.isfinite(df["daq_time_s"].to_numpy())
    if not ok.any():
        return None
    ft = np.full(int(fr[ok].max()) + 1, np.nan)
    ft[fr[ok]] = df["daq_time_s"].to_numpy()[ok]
    return ft


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for d in sys.argv[1:]:
        out = export(d)
        print(f"{d}: {out or 'no DAQ data (daq_meta.json / daq_raw.f32 missing)'}")
