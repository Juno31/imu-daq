"""Import GoPro videos copied from the SD cards into the trial folders, named like the trials.

    python video_import.py <folder with the SD card copies> [--data DIR] [--dry-run]

Copy the DCIM folders from the three SD cards anywhere (sub-folders are fine), then run this.
Each video is matched to a trial by:
  - camera: the serial number stored in the MP4 (GPMF 'CASN'). Its last 4 digits are the
    "GoPro 1234" Bluetooth name (USB stores the full serial). If a file has none, a folder name
    containing the 4 digits or cam1/cam2/cam3 is used instead.
  - time: the recording start in the MP4 header vs. the camera start time in the trial's
    video/video.json (the PC sets each camera's clock when it connects).
Matches are copied (never moved) to
    <trial>/video/<trial>_<cam>.mp4          (one file)
    <trial>/video/<trial>_<cam>_ch01.mp4 ... (long takes the camera split into chapters)
and listed in video.json under "imported".
"""
import argparse
import json
import os
import re
import shutil
import struct
import time

MP4_EPOCH = 2082844800          # seconds from 1904-01-01 to 1970-01-01
TOL_S = 30.0                    # camera start vs. file start (camera clock drift, set at connect)
NAME_RE = re.compile(r"^G[XHL](\d\d)(\d{4})\.MP4$", re.I)   # GX + chapter + video number


# ---------------- MP4 reading (standard library only) ----------------
def _boxes(f, start, end):
    pos = start
    while pos + 8 <= end:
        f.seek(pos)
        hdr = f.read(8)
        if len(hdr) < 8:
            return
        size, kind = struct.unpack(">I4s", hdr)
        head = 8
        if size == 1:
            size = struct.unpack(">Q", f.read(8))[0]
            head = 16
        elif size == 0:
            size = end - pos
        if size < head:
            return
        yield kind.decode("latin-1"), pos + head, pos + size
        pos += size


def _gpmf(data):
    """GPMF KLV -> {key: value} for string/int keys, flattening nested containers."""
    out, i = {}, 0
    while i + 8 <= len(data):
        key = data[i:i + 4].decode("latin-1")
        typ, ssize, rep = data[i + 4], data[i + 5], struct.unpack(">H", data[i + 6:i + 8])[0]
        n = ssize * rep
        body = data[i + 8:i + 8 + n]
        if typ == 0:
            out.update(_gpmf(body))
        elif chr(typ) == "c":
            out.setdefault(key, body.split(b"\0")[0].decode("latin-1", "replace").strip())
        i += 8 + ((n + 3) & ~3)
    return out


def mp4_info(path):
    """{'start_naive': unix seconds of the header time, 'duration_s', 'serial', 'model'}."""
    info = {"start_naive": None, "duration_s": None, "serial": "", "model": ""}
    with open(path, "rb") as f:
        end = os.fstat(f.fileno()).st_size
        for kind, a, b in _boxes(f, 0, end):
            if kind != "moov":
                continue
            for k2, a2, b2 in _boxes(f, a, b):
                if k2 == "mvhd":
                    f.seek(a2)
                    ver = f.read(4)[0]
                    if ver == 1:
                        ct, _mt, scale, dur = struct.unpack(">QQIQ", f.read(28))
                    else:
                        ct, _mt, scale, dur = struct.unpack(">IIII", f.read(16))
                    info["start_naive"] = ct - MP4_EPOCH
                    info["duration_s"] = dur / scale if scale else None
                elif k2 == "udta":
                    for k3, a3, b3 in _boxes(f, a2, b2):
                        if k3 == "GPMF" and b3 - a3 < 1 << 20:
                            f.seek(a3)
                            g = _gpmf(f.read(b3 - a3))
                            info["serial"] = g.get("CASN", "")
                            info["model"] = g.get("MINF", "")
    return info


def cam_key(s):
    """'GoPro 9926' / 'C3501325129926' -> '9926'."""
    d = re.sub(r"\D", "", str(s or ""))
    return d[-4:] if len(d) >= 4 else ""


# ---------------- trials ----------------
def find_trials(data_dir):
    """Trials that started cameras: [{dir, name, cams: {cam: {key, t}}}]."""
    out = []
    for root, _dirs, files in os.walk(data_dir):
        if os.path.basename(root) != "video" or "video.json" not in files:
            continue
        try:
            with open(os.path.join(root, "video.json"), encoding="utf-8") as f:
                v = json.load(f)
        except (OSError, ValueError):
            continue
        tdir = os.path.dirname(root)
        cams = {}
        for cam, r in (v.get("start") or {}).items():
            if r.get("ok"):
                t = r.get("t_encoding_unix") or r.get("t_ack_unix") or r.get("t_send_unix")
                stop = (v.get("stop") or {}).get(cam, {})
                cams[cam] = {"key": cam_key(r.get("serial")), "t": t,
                             "dur": (stop.get("t_send_unix") - t) if stop.get("t_send_unix") and t else None}
        if cams:
            out.append({"dir": tdir, "name": os.path.basename(tdir), "cams": cams, "json": v})
    return out


def find_videos(folder):
    """Recordings on the SD copies, chapters grouped: [{key, video_no, chapters: [path...], info}]."""
    groups = {}
    for root, _dirs, files in os.walk(folder):
        for fn in files:
            m = NAME_RE.match(fn)
            if not m:
                continue
            p = os.path.join(root, fn)
            chapter, vno = int(m.group(1)), m.group(2)
            try:
                info = mp4_info(p)
            except Exception as e:
                info = {"error": str(e), "start_naive": None, "serial": ""}
            key = cam_key(info.get("serial")) or _key_from_path(root)
            g = groups.setdefault((key, vno, root), {"key": key, "video_no": vno, "chapters": {}, "info": None,
                                                     "folder": root})
            g["chapters"][chapter] = p
            if chapter == 1 or g["info"] is None:
                g["info"] = info
    out = []
    for g in groups.values():
        g["chapters"] = [g["chapters"][k] for k in sorted(g["chapters"])]
        out.append(g)
    return out


def _key_from_path(path):
    for part in reversed(path.replace("\\", "/").split("/")):
        k = re.search(r"(?<!\d)(\d{4})(?!\d)", part)
        if k:
            return k.group(1)
        c = re.search(r"cam\s*(\d)", part, re.I)
        if c:
            return "cam" + c.group(1)
    return ""


# ---------------- matching ----------------
def match(trials, videos):
    """Pairs (trial, cam, video, offset_s). The header time is tried as local time and as UTC;
    the reading that matches more recordings wins."""
    slots = []
    for t in trials:
        for cam, c in t["cams"].items():
            slots.append((t, cam, c))

    def fits(v, c, cam, shift):
        if v["info"].get("start_naive") is None or not c["t"]:
            return None
        if v["key"] and v["key"] not in (c["key"], cam):
            return None
        d = (v["info"]["start_naive"] + shift) - c["t"]
        return d if abs(d) <= TOL_S else None

    best = None
    local = time.altzone if time.localtime().tm_isdst > 0 else time.timezone
    for shift in (local, 0):                     # header time read as local time, then as UTC
        pairs, used_v, used_s = [], set(), set()
        cands = []
        for vi, v in enumerate(videos):
            for si, (t, cam, c) in enumerate(slots):
                d = fits(v, c, cam, shift)
                if d is not None:
                    cands.append((abs(d), vi, si, d))
        for _ad, vi, si, d in sorted(cands):
            if vi in used_v or si in used_s:
                continue
            used_v.add(vi)
            used_s.add(si)
            t, cam, _c = slots[si]
            pairs.append((t, cam, videos[vi], d))
        if best is None or len(pairs) > len(best[1]):
            best = (shift, pairs)
    return best[1] if best else []


def import_videos(folder, data_dir, dry_run=False, log=print):
    trials = find_trials(data_dir)
    videos = find_videos(folder)
    log(f"{len(videos)} GoPro recording(s) in {folder}; {len(trials)} trial(s) with cameras in {data_dir}")
    pairs = match(trials, videos)
    done = []
    for t, cam, v, d in sorted(pairs, key=lambda x: (x[0]["name"], x[1])):
        vdir = os.path.join(t["dir"], "video")
        n = len(v["chapters"])
        files = []
        for i, src in enumerate(v["chapters"], 1):
            dest = os.path.join(vdir, f"{t['name']}_{cam}" + (f"_ch{i:02d}" if n > 1 else "") + ".mp4")
            files.append({"file": os.path.basename(dest), "source": src, "bytes": os.path.getsize(src)})
            if dry_run:
                continue
            if os.path.isfile(dest) and os.path.getsize(dest) == os.path.getsize(src):
                continue                                           # already imported
            shutil.copy2(src, dest + ".part")
            os.replace(dest + ".part", dest)
        log(f"{t['name']} {cam} <- {', '.join(os.path.basename(c) for c in v['chapters'])} "
            f"(file starts {d:+.1f} s from the camera start)" + (" [dry run]" if dry_run else ""))
        if not dry_run:
            _record(vdir, cam, {"files": files, "file_vs_start_s": round(d, 2), "serial": v["info"].get("serial"),
                                "model": v["info"].get("model"), "imported_unix": time.time()})
        done.append({"trial": t["name"], "cam": cam, "files": [f["file"] for f in files], "offset_s": round(d, 2)})
    used = {id(p[2]) for p in pairs}
    unmatched = [os.path.basename(v["chapters"][0]) for v in videos if id(v) not in used]
    missing = sorted({f"{t['name']} {cam}" for t in trials for cam in t["cams"]} -
                     {f"{p[0]['name']} {p[1]}" for p in pairs})
    if unmatched:
        log(f"not matched to any trial: {', '.join(sorted(unmatched))}")
    return {"imported": done, "unmatched": sorted(unmatched), "trials_without_video": missing}


def _record(vdir, cam, entry):
    p = os.path.join(vdir, "video.json")
    try:
        with open(p, encoding="utf-8") as f:
            v = json.load(f)
    except (OSError, ValueError):
        v = {}
    v.setdefault("imported", {})[cam] = entry
    with open(p, "w", encoding="utf-8") as f:
        json.dump(v, f, indent=2)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("folder", help="folder with the files copied from the SD cards")
    ap.add_argument("--data", default=os.path.join(os.path.dirname(here), "data"), help="recordings folder")
    ap.add_argument("--dry-run", action="store_true", help="show the matches, copy nothing")
    a = ap.parse_args()
    r = import_videos(a.folder, a.data, a.dry_run)
    print(f"imported {len(r['imported'])}, unmatched {len(r['unmatched'])}, "
          f"trials still without video: {len(r['trials_without_video'])}")
