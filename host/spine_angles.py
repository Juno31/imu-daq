"""Spine angles from a recorded session (T7 and T12 IMUs).

    python spine_angles.py <session folder> [--t7 A4B --t12 A4A] [--neutral 0:2]

Uses spine_cal.json in the session folder (written when you calibrate in the Spine view before or
during the recording). Without it, or with --neutral, it calibrates from the recording itself:
neutral = the given time window (s from the first frame, subject standing still), flexion axis =
the largest bend in the recording, so start each trial with ~2 s of standing and one forward bend.

Writes spine_angles.csv: frame, t_s (+ daq_time_s / pc_unix_s if present) and, in degrees,
  rel_flex, rel_lat, rel_axial   T7 relative to T12 (mid-thoracic)
  t12_flex, t12_lat, t12_axial   T12 relative to standing
  t7_flex,  t7_lat,  t7_axial    T7 relative to standing
Signs: flexion + forward, lateral bending + left, axial rotation + left (intrinsic X-Y-Z).
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd


def qmul(a, b):
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], -1)


def qconj(q):
    return q * np.array([1, -1, -1, -1])


def rotvec(q):
    q = np.where(q[..., :1] < 0, -q, q)
    s = np.linalg.norm(q[..., 1:], axis=-1, keepdims=True)
    th = 2 * np.arctan2(s, q[..., :1])
    return np.where(s > 1e-9, q[..., 1:] / np.maximum(s, 1e-12) * th, 0.0)


def qmean(qs):
    qs = qs * np.sign(qs @ qs[0])[:, None]
    m = qs.mean(0)
    return m / np.linalg.norm(m)



def euler_xyz(q):
    w, x, y, z = np.moveaxis(q, -1, 0)
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - w * z); r02 = 2 * (x * z + w * y)
    r12 = 2 * (y * z - w * x); r22 = 1 - 2 * (x * x + y * y)
    return np.degrees(np.stack([np.arctan2(-r12, r22), np.arcsin(np.clip(r02, -1, 1)), np.arctan2(-r01, r00)], -1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--t7"); ap.add_argument("--t12")
    ap.add_argument("--neutral", help="self-calibrate: standing window in s, e.g. 0:2")
    a = ap.parse_args()

    f = os.path.join(a.session, "imu_synced.csv")
    df = pd.read_csv(f if os.path.exists(f) else os.path.join(a.session, "imu.csv"))
    cal = None
    cf = os.path.join(a.session, "spine_cal.json")
    if os.path.exists(cf) and not a.neutral:
        cal = json.load(open(cf, encoding="utf-8"))
    t7 = a.t7 or (cal or {}).get("map", {}).get("T7") or "A4B"
    t12 = a.t12 or (cal or {}).get("map", {}).get("T12") or "A4A"
    Q = {n: df[[f"{n}_w", f"{n}_x", f"{n}_y", f"{n}_z"]].to_numpy(float) for n in (t7, t12)}
    t = (df["t_us"] - df["t_us"].iloc[0]).to_numpy() / 1e6

    if cal and cal.get("q0") and cal.get("B"):
        q0 = {n: np.array(cal["q0"][n]) for n in (t7, t12)}
        B = np.array(cal["B"])
        how = f"spine_cal.json from {cal.get('when')}"
    else:
        lo, hi = (float(x) for x in (a.neutral or "0:2").split(":"))
        sel = (t >= lo) & (t <= hi)
        if sel.sum() < 20:
            sys.exit("neutral window has too few frames")
        q0 = {n: qmean(Q[n][sel]) for n in (t7, t12)}
        rv = sum(rotvec(qmul(Q[n], qconj(q0[n]))) for n in (t7, t12))
        big = (np.linalg.norm(rv, axis=1) > np.radians(10)) & (t > hi)
        if big.sum() < 10:
            sys.exit("no forward bend over 10 deg found to define the flexion axis")
        onset = t[np.argmax(big)]                     # first bend after neutral: use its first 2 s only
        big &= (t >= onset) & (t <= onset + 2.0)
        ax = rv[big].sum(0); X = np.array([ax[0], ax[1], 0]) / np.hypot(ax[0], ax[1])
        yaw = np.arctan2(X[1], X[0])                     # B = heading-only rotation about Z
        B = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        how = f"self-calibrated: neutral {lo}-{hi} s, flexion axis from {big.sum()} bent frames"

    seg = {n: qmul(qmul(qconj(B), qmul(Q[n], qconj(q0[n]))), B) for n in (t7, t12)}
    out = pd.DataFrame({"frame": df["frame"], "t_s": t})
    for c in ("daq_time_s", "pc_unix_s"):
        if c in df:
            out[c] = df[c]
    for pre, q in (("rel", qmul(qconj(seg[t12]), seg[t7])), ("t12", seg[t12]), ("t7", seg[t7])):
        e = euler_xyz(q)
        out[f"{pre}_flex"], out[f"{pre}_lat"], out[f"{pre}_axial"] = e[:, 0], e[:, 1], e[:, 2]
    dst = os.path.join(a.session, "spine_angles.csv")
    out.round(3).to_csv(dst, index=False)
    print(f"T7={t7} T12={t12}; {how}")
    print(f"wrote {dst} ({len(out)} frames)")
    print(out[[c for c in out if c.endswith(("_flex", "_lat", "_axial"))]].describe().loc[["min", "max"]].round(1).to_string())


if __name__ == "__main__":
    main()
