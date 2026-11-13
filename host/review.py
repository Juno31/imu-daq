"""Load a finished recording for the Review page (read-only).

load(folder) -> dict with
  meta      session.json, plus a few summary.json fields
  imu       t (s, on the DAQ clock when available), per-IMU quaternion / acc / gyro arrays
  emg       per-channel min/max envelope (DC removed, mV) and 100 ms RMS, on the same time axis
  events    markers and other events with their time
  cal       neutral quaternions (q0) and body axes (B) for the spine replay, and where they came from

Nothing here writes to the recording folder.
"""
import csv
import json
import os

import numpy as np

import emgnorm

EMG_MAX_BINS = 12000          # envelope points per channel sent to the browser (about 5 ms bins for 1 min)
RMS_WIN_S = 0.1


def _json(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _round(a, nd):
    return np.round(np.asarray(a, dtype=float), nd).tolist()


def _imu(folder):
    path = os.path.join(folder, "imu_synced.csv")
    if not os.path.isfile(path):
        path = os.path.join(folder, "imu.csv")
    if not os.path.isfile(path):
        return None
    with open(path, newline="") as f:
        rd = csv.reader(f)
        head = next(rd)
        rows = [r for r in rd if r]
    if not rows:
        return None
    col = {h: i for i, h in enumerate(head)}
    data = np.array([[float(x) if x not in ("", "nan") else np.nan for x in r] for r in rows], dtype=float)
    names = [h[:-3] for h in head if h.endswith("_ok")]
    t_us = data[:, col["t_us"]]
    t_board = (t_us - t_us[0]) / 1e6
    clock = "board"
    t = t_board
    if "daq_time_s" in col:
        d = data[:, col["daq_time_s"]]
        if np.isfinite(d).sum() > len(d) * 0.5:
            # fill gaps from the board clock (linear fit board -> DAQ)
            ok = np.isfinite(d)
            a, b = np.polyfit(t_board[ok], d[ok], 1)
            t = np.where(ok, d, a * t_board + b)
            clock = "daq"
    out = {"t": _round(t, 5), "frame": data[:, col["frame"]].astype(int).tolist(), "clock": clock, "pods": {},
           "pc0": float(data[0, col["pc_unix_s"]]) if "pc_unix_s" in col else None}
    for n in names:
        p = {"ok": data[:, col[f"{n}_ok"]].astype(int).tolist()}
        if f"{n}_w" in col:
            q = data[:, [col[f"{n}_{c}"] for c in "wxyz"]]
            p["q"] = _round(q, 5)
        for key, cols in (("acc", ("ax", "ay", "az")), ("gyr", ("gx", "gy", "gz")), ("mag", ("mx", "my", "mz"))):
            if all(f"{n}_{c}" in col for c in cols):
                p[key] = _round(data[:, [col[f"{n}_{c}"] for c in cols]], 4)
        out["pods"][n] = p
    return out


def _emg(folder):
    meta = _json(os.path.join(folder, "daq_meta.json"))
    raw = os.path.join(folder, meta.get("file", "daq_raw.f32")) if meta else None
    if not meta or not os.path.isfile(raw):
        return None
    chans = meta.get("channels", [])
    nch = len(chans)
    rate = float(meta.get("rate_hz", 0)) or 1.0
    x = np.fromfile(raw, dtype=np.float32)
    n = len(x) // nch if nch else 0
    if n < 2:
        return None
    x = x[: n * nch].reshape(n, nch)
    labels = {c.get("chan"): c for c in meta.get("channel_labels", [])}
    binsz = max(1, int(np.ceil(n / EMG_MAX_BINS)))
    nb = n // binsz
    win = max(1, int(RMS_WIN_S * rate))
    out = {"rate_hz": rate, "n_samples": n, "bin_s": binsz / rate, "t0": 0.0, "channels": []}
    for i, ch in enumerate(chans):
        lab = labels.get(ch, {})
        name = lab.get("name") or ch
        if name.upper() == "SYNC":
            continue
        v = x[:, i].astype(np.float64)
        v = (v - np.mean(v)) * 1000.0                      # V -> mV, DC removed
        vb = v[: nb * binsz].reshape(nb, binsz)
        mn, mx = vb.min(axis=1), vb.max(axis=1)
        y = emgnorm.bandpass(v, rate)                      # RMS on the 20-450 Hz band (same as the EMG page)
        sq = np.concatenate([[0.0], np.cumsum(y * y)])     # moving RMS (centred window)
        idx = np.arange(nb) * binsz + binsz // 2
        lo, hi = np.clip(idx - win // 2, 0, n), np.clip(idx + win // 2, 0, n)
        rms = np.sqrt((sq[hi] - sq[lo]) / np.maximum(1, hi - lo))
        out["channels"].append({"chan": ch, "name": name, "muscle": lab.get("muscle", ""), "side": lab.get("side", ""),
                                "min": _round(mn, 3), "max": _round(mx, 3), "rms": _round(rms, 3),
                                "rms_total": round(float(np.sqrt(np.mean(y * y))), 3)})
    return out


def _events(folder, imu):
    path = os.path.join(folder, "events.csv")
    if not os.path.isfile(path):
        return []
    fr2t = {}
    if imu:
        fr2t = dict(zip(imu["frame"], imu["t"]))
    out = []
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        for r in csv.DictReader(f):
            try:
                fr = int(r.get("frame") or -1)
            except ValueError:
                fr = -1
            t = fr2t.get(fr)
            if t is None and imu and fr >= 0:           # frame outside the IMU table: nearest
                frames = imu["frame"]
                k = min(range(len(frames)), key=lambda i: abs(frames[i] - fr))
                t = imu["t"][k]
            out.append({"type": r.get("type", ""), "detail": r.get("detail", ""), "frame": fr, "t": t,
                        "pc_time": r.get("pc_time", "")})
    return out


def _cal(folder, data_root):
    sc = _json(os.path.join(folder, "spine_cal.json"))
    if sc and sc.get("q0"):
        return {"q0": sc.get("q0"), "B": sc.get("B"), "map": sc.get("map"), "height_cm": sc.get("height_cm"),
                "source": "spine_cal.json (saved with this recording)"}
    c = _json(os.path.join(folder, "calibration.json"))
    if c and c.get("imus"):
        q0 = {n: v["q0"] for n, v in c["imus"].items() if v.get("q0")}
        # the body axes (forward bend) are only in spine_cal; reuse the last one if it has the same neutral
        last = _json(os.path.join(data_root, "spine_cal_last.json")) or {}
        same = last.get("q0") and all(
            n in last["q0"] and max(abs(a - b) for a, b in zip(last["q0"][n], q)) < 1e-4 for n, q in q0.items())
        return {"q0": q0, "B": last.get("B") if same else None, "map": last.get("map") if same else None,
                "height_cm": last.get("height_cm") if same else None,
                "source": f"calibration.json ({c.get('session', 'static calibration')})"
                          + (" + body axes from the last spine calibration" if same and last.get("B") else "")}
    return None


def load(folder, data_root):
    imu = _imu(folder)
    summ = _json(os.path.join(folder, "summary.json")) or {}
    meta = _json(os.path.join(folder, "session.json")) or {}
    meta.update({k: summ.get(k) for k in ("session", "started", "duration_s", "frames_received", "missing_frames",
                                          "markers", "rate_hz", "aborted")})
    ds = summ.get("daq_sync")
    meta["sync_ok"] = None if ds is None else bool(ds.get("ok"))
    meta["rel"] = os.path.relpath(folder, data_root)
    emg = _emg(folder)
    if imu and emg and imu["clock"] == "board" and imu.get("pc0"):
        # no DAQ time per frame: align the board clock to the DAQ start by the PC clock (about +-5 ms)
        dm = _json(os.path.join(folder, "daq_meta.json")) or {}
        if dm.get("pc_start_s"):
            off = imu["pc0"] - float(dm["pc_start_s"])
            imu["t"] = _round(np.asarray(imu["t"]) + off, 5)
            imu["clock"] = "pc"
    if emg:
        norm, src = emgnorm.for_recording(folder, data_root)
        if norm:
            emg["norm_source"] = src
            emg["norm_saved"] = norm.get("saved", "")
            for ch in emg["channels"]:
                c = (norm.get("channels") or {}).get(ch["name"]) or {}
                if c.get("ref_mv"):
                    ch["ref_mv"], ch["ref_kind"] = c["ref_mv"], c.get("ref_kind", "MVC")
    return {"meta": meta, "imu": imu, "emg": emg, "events": _events(folder, imu), "cal": _cal(folder, data_root)}
