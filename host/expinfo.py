"""Experiment information kept per test date + subject (data/<date>/<subject>/):

  session_info.json   who ran the session (names + roles), location, time, weather, protocol, notes
  participant.json    participant details (age, sex, height, weight, dominant side, history, consent)
  jig_alignment.json  every IMU's quaternion while all sit in the jig with the same orientation
  setup_check.json    IMU / EMG attachment and data-quality checklists

Each file is copied into every trial recorded afterwards, so a trial folder is self-contained.
data/expinfo_last.json keeps the last session_info so the next session starts pre-filled.
"""
import glob
import json
import os
import time

from session import date_id, subject_id

KINDS = ("session_info", "participant", "jig_alignment", "setup_check")
LAST = "expinfo_last.json"


def subject_dir(root, date, subject):
    return os.path.join(root, date_id(date), subject_id(subject))


def _read(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load(root, date, subject):
    d = subject_dir(root, date, subject)
    out = {k: _read(os.path.join(d, k + ".json")) for k in KINDS}
    out["emg_norm"] = os.path.isfile(os.path.join(d, "emg_norm.json"))
    out["trials"] = sorted(os.path.basename(p) for p in glob.glob(os.path.join(d, "*_trial*"))
                           if os.path.isfile(os.path.join(p, "summary.json")))
    out["last_session_info"] = _read(os.path.join(root, LAST))
    out["dir"] = os.path.relpath(d, root).replace(os.sep, "/")
    return out


def save(root, date, subject, kind, data):
    if kind not in KINDS:
        raise ValueError(f"unknown info kind: {kind}")
    if not isinstance(data, dict):
        raise ValueError("data must be an object")
    d = subject_dir(root, date, subject)
    os.makedirs(d, exist_ok=True)
    data = dict(data, saved=time.strftime("%Y-%m-%d %H:%M:%S"), date=date_id(date), subject=subject_id(subject))
    path = os.path.join(d, kind + ".json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    if kind == "session_info":
        with open(os.path.join(root, LAST), "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    return path, data


def copy_into(trial_dir):
    """Copy the info files of the trial's subject folder into the trial folder."""
    import shutil
    parent = os.path.dirname(trial_dir)
    for k in KINDS:
        src = os.path.join(parent, k + ".json")
        if os.path.isfile(src):
            try:
                shutil.copy2(src, os.path.join(trial_dir, k + ".json"))
            except OSError:
                pass


def latest_trial(root, date, subject, task):
    """Newest finished recording of this task for the date / subject (folder path), or None."""
    d = subject_dir(root, date, subject)
    found = [p for p in glob.glob(os.path.join(d, f"{subject_id(subject)}_{task}_trial*"))
             if os.path.isfile(os.path.join(p, "summary.json"))]
    return max(found, key=os.path.getmtime) if found else None
