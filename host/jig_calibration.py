"""Six-position jig calibration of the BNO085/BNO086 accelerometer and gyroscope.

Recording (in the GUI, like any trial; no task needed):
  1. Mount the boards in the jig, power on, wait 10-15 min (sensor warm-up).
  2. Put the jig on its bottom face (6 dots down) on a level, still table. Start recording.
  3. Hold still >= 15 s. Lift and turn the jig onto the next face, hold >= 15 s.
     Do all 6 faces (order does not matter). Press M with e.g. "face 3" if you like;
     markers are only listed, the poses are found from the data. Stop.
  One recording calibrates every IMU in it (2 per recording with the current board).

What this script finds, per IMU:
  gyroscope       bias [deg/s] (mean of all still samples), noise, bias spread between poses
  accelerometer   full 6-position fit   C (a_raw - b) = g * u_jig
                    b  bias [m/s^2]
                    C = R K   (polar decomposition)
                    K  scale factors + cross-axis coupling (sensor errors)
                    R  how the sensor axes are rotated against the jig axes (mounting)
                  plus the simple per-axis bias/scale ((a+ + a-)/2, (a+ - a-)/2g) as a check
  between IMUs    the angle between every pair of sensors' axes (from R)

Jig axes: the jig's 6 resting faces define 6 "up" directions, +-x, +-y, +-z. They are named
after the sensor axis closest to each one, so the boards must sit square in the jig (any
mounting error under ~20 deg is fine). All boards in the jig face the same way, so every
IMU gets the same names.

Notes
  - The BNO08x outputs are already "calibrated" by its own dynamic calibration. This finds
    what is left. Keep the firmware's sensor settings the same when you use the results.
  - A still jig cannot reveal gyroscope scale or axis errors (that needs a turntable).
    The gyro shares the chip with the accelerometer, so R also applies to it.
  - Magnetometer is not calibrated here (it needs free rotation, not 6 still poses).

Usage:
  python host/jig_calibration.py <recording folder> [more folders ...]
         [--label A4A=IMU03 --label A4B=IMU07] [--lat 37.56 --alt 40] [--min-hold 8]
Writes jig_calibration.json and jig_calibration.csv into each recording folder, and with
--label also data/imu_calibration/<label>.json (latest result + history for that board).
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

RAD2DEG = 180.0 / math.pi

# still-pose detection
WIN_S = 0.5               # moving window
GYRO_STILL_DPS = 1.5      # |gyro| (window mean) below this
ACC_STILL_MS2 = 0.08      # accelerometer change (window SD of |a|) below this
MIN_HOLD_S = 8.0          # a pose must stay still at least this long
TRIM_S = 1.0              # drop this much at both ends of each still part
MAX_TILT_DEG = 20.0       # a pose's "up" must be within this of a sensor axis

AXES = ["+x", "-x", "+y", "-y", "+z", "-z"]
AXIS_VEC = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0),
            "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}


def local_gravity(lat_deg=37.56, alt_m=40.0):
    """WGS-84 normal gravity with free-air correction [m/s^2]. Default: Seoul (Hanyang Univ.)."""
    s = math.sin(math.radians(lat_deg))
    s2 = math.sin(math.radians(2 * lat_deg))
    return 9.780327 * (1 + 0.0053024 * s * s - 0.0000058 * s2 * s2) - 3.086e-6 * alt_m


# --------------------------------------------------------------------------- reading
def load(rec_dir):
    import pandas as pd
    df = pd.read_csv(os.path.join(rec_dir, "imu.csv"))
    pods = [c[:-3] for c in df.columns if c.endswith("_ax")]
    if not pods:
        raise SystemExit(f"{rec_dir}: imu.csv has no accelerometer columns (firmware older than v1.2?)")
    t = df["t_us"].to_numpy(dtype=np.float64) / 1e6
    markers = []
    ev = os.path.join(rec_dir, "events.csv")
    if os.path.exists(ev):
        with open(ev, newline="") as f:
            for row in csv.DictReader(f):
                if row.get("type") == "marker":
                    markers.append({"frame": row.get("frame"), "text": row.get("detail")})
    meta = {}
    sj = os.path.join(rec_dir, "session.json")
    if os.path.exists(sj):
        with open(sj) as f:
            meta = json.load(f)
    return df, t, pods, markers, meta


def pod_signals(df, p):
    """Accelerometer [m/s^2] and gyroscope [rad/s] per frame, NaN where the sample is not new."""
    a = df[[f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy(dtype=np.float64)
    g = df[[f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(dtype=np.float64)
    a[df[f"{p}_acc_new"].to_numpy() <= 0] = np.nan
    g[df[f"{p}_gyr_new"].to_numpy() <= 0] = np.nan
    return a, g


# --------------------------------------------------------------------------- still poses
def _moving(x, n, fn):
    out = np.full(len(x), np.nan)
    h = n // 2
    for i in range(len(x)):
        w = x[max(0, i - h): i + h + 1]
        w = w[~np.isnan(w)]
        if len(w) >= max(3, n // 3):
            out[i] = fn(w)
    return out


def still_mask(t, signals):
    """Frames where every IMU is still. signals: list of (acc, gyr) arrays."""
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.01
    n = max(5, int(round(WIN_S / dt)))
    still = np.ones(len(t), bool)
    for a, g in signals:
        gn = np.linalg.norm(g, axis=1) * RAD2DEG
        an = np.linalg.norm(a, axis=1)
        still &= _moving(gn, n, np.mean) < GYRO_STILL_DPS
        still &= _moving(an, n, np.std) < ACC_STILL_MS2
    return still


def still_segments(t, still, min_hold=MIN_HOLD_S):
    segs, i, N = [], 0, len(t)
    while i < N:
        if not still[i]:
            i += 1
            continue
        j = i
        while j + 1 < N and still[j + 1]:
            j += 1
        a, b = t[i] + TRIM_S, t[j] - TRIM_S
        if b - a >= min_hold - 2 * TRIM_S:
            segs.append((a, b))
        i = j + 1
    return segs


# --------------------------------------------------------------------------- fits
def nearest_axis(v):
    k = int(np.argmax(np.abs(v)))
    name = ("+" if v[k] > 0 else "-") + "xyz"[k]
    tilt = math.degrees(math.acos(min(1.0, abs(v[k]) / np.linalg.norm(v))))
    return name, tilt


def polar(C):
    """C = R K with R a rotation and K symmetric positive definite."""
    U, S, Vt = np.linalg.svd(C)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    K = R.T @ C
    return R, (K + K.T) / 2


def rot_angles(R):
    """Total angle and small-angle components (about x, y, z) of rotation R [deg]."""
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2
    return ang, [round(float(math.degrees(math.asin(max(-1.0, min(1.0, v))))), 3) for v in w]


def fit_accel(poses, g):
    """poses: {axis_name: mean raw accel (3,)} for the 6 faces. Returns the fit dict."""
    names = [n for n in AXES if n in poses]
    A = np.array([poses[n] for n in names])                     # measured, sensor frame
    U = np.array([AXIS_VEC[n] for n in names], float) * g       # expected, jig frame
    # C a + d = g u  (linear in C and d): 3 equations per pose, 12 unknowns
    X = np.hstack([A, np.ones((len(A), 1))])                    # (n, 4)
    sol, *_ = np.linalg.lstsq(X, U, rcond=None)                 # (4, 3)
    C = sol[:3].T
    d = sol[3]
    b = -np.linalg.solve(C, d)
    R, K = polar(C)
    resid = (A @ C.T + d) - U
    # simple per-axis 6-position values, as a cross-check
    simple = {}
    for k, ax in enumerate("xyz"):
        if f"+{ax}" in poses and f"-{ax}" in poses:
            ap, an = poses[f"+{ax}"][k], poses[f"-{ax}"][k]
            simple[ax] = {"bias_ms2": round(float((ap + an) / 2), 5),
                          "scale": round(float((ap - an) / (2 * g)), 6)}
    ang, comp = rot_angles(R)
    sens = 1.0 / np.diag(K)                                      # sensitivity per axis
    off = K - np.diag(np.diag(K))
    return {
        "bias_ms2": [round(float(v), 5) for v in b],
        "scale": [round(float(v), 6) for v in sens],              # raw = scale * true (per axis)
        "scale_error_pct": [round(float((v - 1) * 100), 3) for v in sens],
        "cross_axis_pct": round(float(np.abs(off).max() * 100), 3),
        "C": np.round(C, 6).tolist(),                              # full correction, sensor -> jig
        "K": np.round(K, 6).tolist(),                              # scale + cross-axis, sensor frame
        "R_sensor_to_jig": np.round(R, 6).tolist(),
        "misalignment_deg": round(ang, 3),
        "misalignment_xyz_deg": comp,
        "residual_rms_ms2": round(float(np.sqrt(np.mean(resid ** 2))), 5),
        "residual_max_ms2": round(float(np.abs(resid).max()), 5),
        "simple_6pos": simple,
    }


# --------------------------------------------------------------------------- main analysis
def analyze(rec_dir, g, labels=None, min_hold=MIN_HOLD_S):
    labels = labels or {}
    df, t, pods, markers, meta = load(rec_dir)
    sig = {p: pod_signals(df, p) for p in pods}
    live = [p for p in pods if np.isfinite(sig[p][0]).any()]
    still = still_mask(t, [sig[p] for p in live])
    segs = still_segments(t, still, min_hold)
    out = {"kind": "jig_calibration", "recording": os.path.basename(os.path.normpath(rec_dir)),
           "when": time.strftime("%Y-%m-%d %H:%M:%S"), "gravity_ms2": round(g, 5),
           "subject_field": meta.get("subject"), "markers": markers,
           "still_segments_s": [[round(a - t[0], 2), round(b - t[0], 2)] for a, b in segs],
           "imus": {}, "pairs": {}, "problems": []}
    if not segs:
        out["problems"].append("no still part found: hold each face >= %.0f s" % min_hold)
        return out

    # name each still part by the reference IMU's "up" axis
    ref = live[0]
    seg_axis = []
    for a, b in segs:
        m = (t >= a) & (t <= b)
        up = np.nanmean(sig[ref][0][m], axis=0)
        name, tilt = nearest_axis(up)
        seg_axis.append((a, b, name, tilt))
    found = sorted({s[2] for s in seg_axis}, key=AXES.index)
    out["poses_found"] = found
    missing = [n for n in AXES if n not in found]
    if missing:
        out["problems"].append("faces missing: " + ", ".join(missing) + " (need all 6)")

    Rs = {}
    for p in live:
        a_all, g_all = sig[p]
        r = {"label": labels.get(p, p), "poses": {}}
        poses, gyro_means, gyro_all = {}, [], []
        for a, b, name, _ in seg_axis:
            m = (t >= a) & (t <= b)
            acc = a_all[m][np.isfinite(a_all[m]).all(axis=1)]
            gyr = g_all[m][np.isfinite(g_all[m]).all(axis=1)]
            if len(acc) < 20:
                continue
            ps = poses.setdefault(name, [])
            ps.append(acc)
            gyro_all.append(gyr)
            gyro_means.append(gyr.mean(axis=0))
        for name in list(poses):
            acc = np.vstack(poses[name])
            mean = acc.mean(axis=0)
            ax, tilt = nearest_axis(mean)
            r["poses"][name] = {"samples": int(len(acc)), "acc_mean_ms2": np.round(mean, 5).tolist(),
                                "acc_sd_ms2": np.round(acc.std(axis=0), 5).tolist(),
                                "norm_ms2": round(float(np.linalg.norm(mean)), 5),
                                "tilt_from_axis_deg": round(tilt, 2)}
            if ax != name:
                out["problems"].append(f"{p}: pose {name} reads up along {ax} (board turned in the jig?)")
            if tilt > MAX_TILT_DEG:
                out["problems"].append(f"{p}: pose {name} is {tilt:.0f} deg off its axis")
            poses[name] = mean
        # gyroscope
        if gyro_all:
            ga = np.vstack(gyro_all)
            gm = np.array(gyro_means)
            r["gyro"] = {"bias_dps": np.round(ga.mean(axis=0) * RAD2DEG, 4).tolist(),
                         "noise_sd_dps": np.round(ga.std(axis=0) * RAD2DEG, 4).tolist(),
                         "bias_spread_between_poses_dps": np.round(np.ptp(gm, axis=0) * RAD2DEG, 4).tolist(),
                         "samples": int(len(ga))}
        # accelerometer
        norms = [round(float(np.linalg.norm(v)), 4) for v in poses.values()]
        r["gravity_norm_raw_ms2"] = {"min": min(norms), "max": max(norms)} if norms else None
        if len(poses) == 6:
            fit = fit_accel(poses, g)
            C, b = np.array(fit["C"]), np.array(fit["bias_ms2"])
            corr = [float(np.linalg.norm(C @ (v - b))) for v in poses.values()]
            r["gravity_norm_corrected_ms2"] = {"min": round(min(corr), 4), "max": round(max(corr), 4)}
            r["accel"] = fit
            Rs[p] = np.array(fit["R_sensor_to_jig"])
        else:
            r["accel"] = None
        out["imus"][p] = r

    # sensor-to-sensor alignment
    names = list(Rs)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            Rij = Rs[names[i]].T @ Rs[names[j]]
            ang, comp = rot_angles(Rij)
            out["pairs"][f"{names[i]}-{names[j]}"] = {"angle_deg": round(ang, 3), "xyz_deg": comp}
    out["ok"] = not out["problems"] and len(Rs) == len(live)
    return out


def apply_calibration(acc, gyr, cal, to_jig=False):
    """Correct raw samples with one IMU's result (cal = out['imus'][pod]).

    acc [m/s^2], gyr [rad/s], arrays (n, 3). Default: corrected in the sensor's own axes
    (bias, scale, cross-axis removed). to_jig=True also rotates into the jig axes."""
    acc = np.asarray(acc, float)
    gyr = np.asarray(gyr, float)
    a = cal["accel"]
    M = np.array(a["C"]) if to_jig else np.array(a["K"])
    acc_c = (acc - np.array(a["bias_ms2"])) @ M.T
    gyr_c = gyr - np.array(cal["gyro"]["bias_dps"]) / RAD2DEG
    if to_jig:
        gyr_c = gyr_c @ np.array(a["R_sensor_to_jig"]).T
    return acc_c, gyr_c


# --------------------------------------------------------------------------- output
def report(out):
    L = [f"Jig calibration: {out['recording']}   g = {out['gravity_ms2']} m/s^2",
         f"  still parts: {len(out['still_segments_s'])}   poses: {', '.join(out.get('poses_found', []))}"]
    for p, r in out["imus"].items():
        L.append(f"  [{p}] {r['label']}")
        if "gyro" in r:
            gy = r["gyro"]
            L.append(f"    gyro bias      {fmt(gy['bias_dps'])} deg/s   noise {fmt(gy['noise_sd_dps'])}"
                     f"   spread {fmt(gy['bias_spread_between_poses_dps'])}")
        a = r.get("accel")
        if a:
            L.append(f"    accel bias     {fmt(a['bias_ms2'])} m/s^2")
            L.append(f"    accel scale    {fmt(a['scale_error_pct'])} %   cross-axis max {a['cross_axis_pct']} %")
            L.append(f"    |g| raw {r['gravity_norm_raw_ms2']['min']}-{r['gravity_norm_raw_ms2']['max']}"
                     f"  ->  corrected {r['gravity_norm_corrected_ms2']['min']}-"
                     f"{r['gravity_norm_corrected_ms2']['max']}")
            L.append(f"    vs jig axes    {a['misalignment_deg']} deg  (about x, y, z: {fmt(a['misalignment_xyz_deg'])})"
                     f"   fit residual {a['residual_rms_ms2']} m/s^2 rms")
    for k, v in out["pairs"].items():
        L.append(f"  {k}: axes differ by {v['angle_deg']} deg (x, y, z: {fmt(v['xyz_deg'])})")
    for m in out["markers"]:
        L.append(f"  marker @frame {m['frame']}: {m['text']}")
    L.append("  RESULT: OK" if out.get("ok") else "  RESULT: CHECK - " + "; ".join(out["problems"]))
    return "\n".join(L)


def fmt(v):
    return "[" + ", ".join(f"{x:+.4f}" for x in v) + "]"


def write(out, rec_dir, library=None):
    with open(os.path.join(rec_dir, "jig_calibration.json"), "w") as f:
        json.dump(out, f, indent=2)
    with open(os.path.join(rec_dir, "jig_calibration.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pod", "label", "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps",
                    "acc_bias_x_ms2", "acc_bias_y_ms2", "acc_bias_z_ms2",
                    "scale_err_x_pct", "scale_err_y_pct", "scale_err_z_pct", "cross_axis_pct",
                    "misalignment_deg", "residual_rms_ms2"])
        for p, r in out["imus"].items():
            a, gy = r.get("accel"), r.get("gyro", {})
            w.writerow([p, r["label"], *gy.get("bias_dps", [""] * 3),
                        *(a["bias_ms2"] if a else [""] * 3), *(a["scale_error_pct"] if a else [""] * 3),
                        a["cross_axis_pct"] if a else "", a["misalignment_deg"] if a else "",
                        a["residual_rms_ms2"] if a else ""])
    if library:
        os.makedirs(library, exist_ok=True)
        for p, r in out["imus"].items():
            if r["label"] == p or not r.get("accel"):
                continue                        # only boards given a --label go to the library
            path = os.path.join(library, f"{r['label']}.json")
            hist = []
            if os.path.exists(path):
                with open(path) as f:
                    hist = json.load(f).get("history", [])
            entry = {"when": out["when"], "recording": out["recording"], "pod": p, "ok": out.get("ok"),
                     "gyro": r.get("gyro"), "accel": r["accel"]}
            with open(path, "w") as f:
                json.dump({"label": r["label"], "latest": entry, "history": hist + [entry]}, f, indent=2)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recordings", nargs="+", help="recording folder(s) containing imu.csv")
    ap.add_argument("--label", action="append", default=[], metavar="POD=NAME",
                    help="name the board in a pod, e.g. A4A=IMU03 (saved to the library)")
    ap.add_argument("--lat", type=float, default=37.56, help="latitude for local gravity (default Seoul)")
    ap.add_argument("--alt", type=float, default=40.0, help="altitude [m] for local gravity")
    ap.add_argument("--min-hold", type=float, default=MIN_HOLD_S, help="shortest still pose [s]")
    ap.add_argument("--library", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                       "..", "data", "imu_calibration"))
    args = ap.parse_args(argv)
    labels = dict(s.split("=", 1) for s in args.label)
    g = local_gravity(args.lat, args.alt)
    status = 0
    for rec in args.recordings:
        out = analyze(rec, g, labels, args.min_hold)
        write(out, rec, os.path.normpath(args.library) if labels else None)
        print(report(out))
        status |= 0 if out.get("ok") else 1
    return status


if __name__ == "__main__":
    sys.exit(main())
