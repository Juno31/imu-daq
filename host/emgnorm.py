"""EMG normalization: baseline offset / noise, MVC and RVC reference values (EMG page).

Each capture is an ordinary short recording (task emg_rest, emg_mvc or emg_rvc) so the raw DAQ data
are kept. This module analyses those recordings and stores the chosen reference values per subject
in <data>/<date>/<subject>/emg_norm.json. Later trials get a copy of that file (server.start).

Processing per channel (SENIAM-style):
  offset     mean of the raw signal at rest (V -> mV)
  filtered   mean removed, band-pass 20-450 Hz (zero-phase FFT filter with cosine edges)
  noise      RMS of the filtered rest signal
  MVC        highest moving RMS (window, default 500 ms) of the filtered signal in an MVC trial
  RVC        RMS of the filtered signal over the middle part (default 60 %) of an RVC hold
"""
import json
import os
import shutil
import time

import numpy as np

BAND = (20.0, 450.0)
KINDS = {"emg_rest": "rest", "emg_mvc": "mvc", "emg_rvc": "rvc"}
FLOAT_CORR = 0.9
_cache = {}


def _json(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def bandpass(x, rate, lo=BAND[0], hi=BAND[1], edge=5.0):
    """Zero-phase band-pass via FFT with raised-cosine transition bands of `edge` Hz."""
    n = len(x)
    if n < 16:
        return x - np.mean(x)
    nfft = 1 << int(np.ceil(np.log2(n)))
    X = np.fft.rfft(x - np.mean(x), nfft)
    f = np.fft.rfftfreq(nfft, 1.0 / rate)
    g = np.zeros_like(f)
    g[(f >= lo + edge) & (f <= hi - edge)] = 1.0
    up = (f > lo - edge) & (f < lo + edge)
    g[up] = 0.5 - 0.5 * np.cos(np.pi * (f[up] - (lo - edge)) / (2 * edge))
    dn = (f > hi - edge) & (f < hi + edge)
    g[dn] = 0.5 + 0.5 * np.cos(np.pi * (f[dn] - (hi - edge)) / (2 * edge))
    return np.fft.irfft(X * g, nfft)[:n]


def moving_rms(y, win):
    sq = np.concatenate([[0.0], np.cumsum(y * y)])
    win = max(1, min(win, len(y)))
    return np.sqrt((sq[win:] - sq[:-win]) / win)


def read_daq(folder):
    meta = _json(os.path.join(folder, "daq_meta.json"))
    if not meta:
        return None, None
    raw = os.path.join(folder, meta.get("file", "daq_raw.f32"))
    if not os.path.isfile(raw):
        return meta, None
    nch = len(meta.get("channels", []))
    x = np.fromfile(raw, dtype=np.float32)
    n = len(x) // nch if nch else 0
    return meta, (x[: n * nch].reshape(n, nch) if n else None)


def analyze(folder, win_s=0.5, hold=0.6, use_filter=True):
    """Per-channel metrics of one recording (cached by file time and parameters)."""
    raw = os.path.join(folder, "daq_raw.f32")
    try:
        key = (folder, os.path.getmtime(raw), win_s, hold, use_filter)
    except OSError:
        return None
    if key in _cache:
        return _cache[key]
    meta, x = read_daq(folder)
    if x is None or len(x) < 100:
        return None
    rate = float(meta.get("rate_hz", 1.0))
    rng = float(meta.get("range_v", 10.0))
    labels = {c.get("chan"): c for c in meta.get("channel_labels", [])}
    chans = meta.get("channels", [])
    sync = None
    for i, ch in enumerate(chans):
        if (labels.get(ch, {}).get("name") or "").upper() == "SYNC":
            sync = x[:, i].astype(np.float64)
    n = len(x)
    win = max(1, int(win_s * rate))
    a, b = int(n * (1 - hold) / 2), int(n * (1 + hold) / 2)
    step = max(1, int(0.02 * rate))                          # 20 ms curve for the page
    out = {"rate_hz": rate, "seconds": round(n / rate, 3), "win_s": win_s, "hold": hold, "filter": use_filter,
           "curve_dt": step / rate, "env_dt": max(1, n // 1500) / rate, "channels": {}}
    for i, ch in enumerate(chans):
        lab = labels.get(ch, {})
        name = lab.get("name") or ch
        if name.upper() == "SYNC":
            continue
        v = x[:, i].astype(np.float64)
        off = float(np.mean(v))
        y = bandpass(v, rate) if use_filter else v - off
        mr = moving_rms(y, win)
        k = int(np.argmax(mr))
        corr = None
        if sync is not None and sync.std() > 0.3 and v.std() > 0:
            corr = float(np.corrcoef(v, sync)[0, 1])
        curve = moving_rms(y, max(1, int(0.1 * rate)))[::step] * 1e3
        eb = max(1, n // 1500)                                    # raw envelope (mean removed) for the plot
        ev = ((v[: n // eb * eb] - off) * 1e3).reshape(-1, eb)
        out["channels"][name] = {
            "chan": ch, "muscle": lab.get("muscle", ""), "side": lab.get("side", ""), "group": lab.get("group", ""),
            "offset_mv": round(off * 1e3, 4),
            "rms_mv": round(float(np.sqrt(np.mean(y * y))) * 1e3, 4),
            "peak_mv": round(float(mr[k]) * 1e3, 4), "peak_t": round((k + win / 2) / rate, 3),
            "hold_mv": round(float(np.sqrt(np.mean(y[a:b] ** 2))) * 1e3, 4),
            "clip_pct": round(100.0 * float(np.mean(np.abs(v) > 0.98 * rng)), 3),
            "floating": None if corr is None else bool(corr > FLOAT_CORR),
            "curve": np.round(curve, 4).tolist(),
            "env_lo": np.round(ev.min(axis=1), 4).tolist(), "env_hi": np.round(ev.max(axis=1), 4).tolist(),
        }
    _cache[key] = out
    return out


def subject_dir(data_root, date, subject):
    from session import date_id, subject_id
    return os.path.join(data_root, date_id(date), subject_id(subject))


def trials(data_root, date, subject, **kw):
    """All EMG calibration recordings of one subject on one day, oldest first, with their metrics."""
    sd = subject_dir(data_root, date, subject)
    out = []
    if not os.path.isdir(sd):
        return sd, out
    for e in sorted(os.scandir(sd), key=lambda e: e.name):
        if not e.is_dir() or not os.path.isfile(os.path.join(e.path, "summary.json")):
            continue
        meta = _json(os.path.join(e.path, "session.json")) or {}
        kind = KINDS.get(meta.get("task", ""))
        if not kind:
            continue
        ec = meta.get("emgcal") or {}
        summ = _json(os.path.join(e.path, "summary.json")) or {}
        out.append({"rel": os.path.relpath(e.path, data_root), "name": e.name, "kind": kind,
                    "trial": meta.get("trial"), "target": ec.get("target", ""), "started": summ.get("started", ""),
                    "aborted": summ.get("aborted"),
                    "metrics": analyze(e.path, **kw)})
    out.sort(key=lambda t: t["started"] or t["name"])
    return sd, out


def load(data_root, date, subject):
    return _json(os.path.join(subject_dir(data_root, date, subject), "emg_norm.json"))


def save(data_root, date, subject, body):
    sd = subject_dir(data_root, date, subject)
    os.makedirs(sd, exist_ok=True)
    path = os.path.join(sd, "emg_norm.json")
    if os.path.isfile(path):                                   # keep the previous version
        os.makedirs(os.path.join(sd, "emg_norm_history"), exist_ok=True)
        shutil.copy2(path, os.path.join(sd, "emg_norm_history", time.strftime("emg_norm_%Y%m%d_%H%M%S.json")))
    body = dict(body, saved=time.strftime("%Y-%m-%d %H:%M:%S"), kind="emg_normalization",
                processing={"band_hz": list(BAND), "note": "offset = rest mean; noise/MVC/RVC on mean-removed, band-passed signal"})
    with open(path, "w", encoding="utf-8") as f:
        json.dump(body, f, indent=1, ensure_ascii=False)
    return path, body


def for_recording(folder, data_root):
    """emg_norm.json copied into a recording, else the subject's current one."""
    n = _json(os.path.join(folder, "emg_norm.json"))
    if n:
        return n, "saved with this recording"
    n = _json(os.path.join(os.path.dirname(folder), "emg_norm.json"))
    return (n, "subject's current EMG normalization") if n else (None, "")
