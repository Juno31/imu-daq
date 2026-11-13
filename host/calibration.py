"""Static calibration: the participant stands still in the neutral posture for a few seconds.

The server records this like any other trial (trial name "static_cal", with SYNC and EMG on the
DAQ), then this module reads that recording and writes calibration.json:

  per IMU   neutral quaternion (mean, w x y z), how much it moved (max deviation from the mean),
            tilt of the sensor z axis from vertical (from gravity), gyro bias and noise,
            accelerometer noise, magnetic field and magnetometer calibration state
  per EMG   resting RMS (mean removed) and DC offset of every DAQ extra channel

The latest calibration is copied into every following recording folder, and its neutral
quaternions are handed to the Spine view.
"""
import json
import math
import os
import time

import numpy as np

MOVE_LIMIT_DEG = 3.0      # max orientation change during the hold (same limit as the Spine view)
GYRO_LIMIT_DPS = 3.0      # max gyro noise (SD) for "still"
RAD2DEG = 180.0 / math.pi


def quat_mean(q):
    """Average of unit quaternions (Markley): eigenvector of sum(q q^T) with the largest eigenvalue."""
    q = np.asarray(q, dtype=np.float64)
    m = q.T @ q
    w, v = np.linalg.eigh(m)
    out = v[:, int(np.argmax(w))]
    return out if out[0] >= 0 else -out


def quat_angle_deg(a, b):
    """Rotation angle between unit quaternions (rows of a) and b."""
    d = np.abs(np.clip(np.asarray(a) @ np.asarray(b), -1.0, 1.0))
    return 2.0 * np.degrees(np.arccos(d))


def analyze(session_dir, pod_names, subject="", seconds=None):
    import pandas as pd
    df = pd.read_csv(os.path.join(session_dir, "imu.csv"))
    out = {"kind": "static_calibration", "session": os.path.basename(session_dir), "subject": subject,
           "when": time.strftime("%Y-%m-%d %H:%M:%S"), "seconds": seconds, "frames": int(len(df)),
           "limits": {"move_deg": MOVE_LIMIT_DEG, "gyro_sd_dps": GYRO_LIMIT_DPS}, "imus": {}, "emg": {}}
    problems = []
    if len(df) < 20:
        out.update(ok=False, problems=["too few frames recorded"])
        return out
    # drop the first 0.3 s (the participant may still be settling after the button press)
    t = df["t_us"].to_numpy(dtype=np.float64)
    df = df[t >= t[0] + 0.3e6]
    for p in pod_names:
        r = {}
        ok = df[f"{p}_ok"].to_numpy() > 0
        q = df.loc[ok & (df[f"{p}_fresh"].to_numpy() > 0), [f"{p}_w", f"{p}_x", f"{p}_y", f"{p}_z"]].to_numpy()
        if len(q) < 10:
            r["ok"] = False
            problems.append(f"{p}: no orientation data")
            out["imus"][p] = r
            continue
        q = q / np.linalg.norm(q, axis=1, keepdims=True)
        q0 = quat_mean(q)
        dev = quat_angle_deg(q, q0)
        r["q0"] = [round(float(v), 6) for v in q0]
        r["moved_max_deg"] = round(float(dev.max()), 2)
        r["moved_sd_deg"] = round(float(dev.std()), 3)
        still = r["moved_max_deg"] <= MOVE_LIMIT_DEG
        if f"{p}_ax" in df:                                    # firmware 1.2: acc / gyro / mag
            a = df.loc[df[f"{p}_acc_new"] > 0, [f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy()
            g = df.loc[df[f"{p}_gyr_new"] > 0, [f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy()
            mg = df.loc[df[f"{p}_mag_new"] > 0, [f"{p}_mx", f"{p}_my", f"{p}_mz"]].to_numpy()
            if len(a) > 5:
                am = a.mean(axis=0)
                gn = float(np.linalg.norm(am))
                r["gravity_ms2"] = [round(float(v), 4) for v in am]
                r["gravity_norm_ms2"] = round(gn, 3)
                r["tilt_z_from_vertical_deg"] = round(math.degrees(math.acos(max(-1.0, min(1.0, am[2] / gn)))), 2) if gn else None
                r["acc_sd_ms2"] = round(float(a.std(axis=0).max()), 4)
            if len(g) > 5:
                r["gyro_bias_dps"] = [round(float(v) * RAD2DEG, 3) for v in g.mean(axis=0)]
                r["gyro_sd_dps"] = round(float(g.std(axis=0).max()) * RAD2DEG, 3)
                still = still and r["gyro_sd_dps"] <= GYRO_LIMIT_DPS
            if len(mg) > 2:
                mm = mg.mean(axis=0)
                r["mag_ut"] = [round(float(v), 2) for v in mm]
                r["mag_norm_ut"] = round(float(np.linalg.norm(mm)), 2)
            r["mag_accuracy"] = int(df[f"{p}_mag_acc"].iloc[-1]) if f"{p}_mag_acc" in df else None
        r["still"] = bool(still)
        r["ok"] = bool(still)
        if not still:
            why = f"moved {r['moved_max_deg']:.1f} deg" if r["moved_max_deg"] > MOVE_LIMIT_DEG else \
                f"gyro noise {r.get('gyro_sd_dps', 0):.1f} deg/s"
            problems.append(f"{p}: {why}")
        out["imus"][p] = r
    out["emg"] = emg_rest(session_dir)
    out["problems"] = problems
    out["ok"] = not problems
    return out


def emg_rest(session_dir):
    """Resting RMS (mean removed) and offset of every DAQ channel except SYNC, from daq_raw.f32."""
    mp, rp = os.path.join(session_dir, "daq_meta.json"), os.path.join(session_dir, "daq_raw.f32")
    if not (os.path.exists(mp) and os.path.exists(rp)):
        return {}
    try:
        with open(mp, encoding="utf-8") as f:
            meta = json.load(f)
        chans = meta.get("channels", [])
        if len(chans) < 2:
            return {}
        x = np.fromfile(rp, dtype=np.float32).reshape(-1, len(chans))
        skip = int(0.3 * float(meta.get("rate_hz", 1)))          # same settling skip as the IMUs
        x = x[skip:]
        labels = {l.get("chan"): l for l in meta.get("channel_labels", [])}
        res = {}
        blk = max(1, int(0.1 * float(meta.get("rate_hz", 1))))   # 100 ms blocks (same as the live RMS)
        sync = x[:, 0].astype(np.float64)
        pulses = float(sync.std()) > 0.3
        for k, c in enumerate(chans[1:], start=1):
            v = x[:, k].astype(np.float64)
            n = len(v) // blk * blk
            if n == 0:
                continue
            b = v[:n].reshape(-1, blk)
            means = b.mean(axis=1, keepdims=True)
            lab = labels.get(c, {})
            corr = float(np.corrcoef(v, sync)[0, 1]) if pulses and v.std() > 0 else None
            res[lab.get("name") or c] = {
                "sync_corr": None if corr is None else round(corr, 3),
                # True = the input copies SYNC: nothing connected (see daq.FLOAT_CORR)
                "floating": None if corr is None else bool(corr > 0.5),
                "chan": c, "muscle": lab.get("muscle", ""), "side": lab.get("side", ""),
                # resting noise: RMS after removing each 100 ms block's mean (slow drift excluded)
                "rms_mv": round(float(np.sqrt(((b - means) ** 2).mean())) * 1e3, 4),
                "drift_mv": round(float(means.std()) * 1e3, 3),      # slow wander of the baseline
                "offset_mv": round(float(v.mean()) * 1e3, 3)}
        return res
    except Exception as e:
        return {"error": str(e)}


def save(folder, cal, name="calibration.json"):
    if not cal or not folder:
        return
    try:
        with open(os.path.join(folder, name), "w", encoding="utf-8") as f:
            json.dump(cal, f, indent=1)
    except OSError:
        pass


def load_last(data_dir):
    try:
        with open(os.path.join(data_dir, "calibration_last.json"), encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None
