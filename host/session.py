"""One recording = one folder:

    <data>/<YYYYmmdd_HHMMSS>_<label>/
        session.json      settings, firmware/board info, operator notes
        imu.csv           every recorded frame as sent by the board (t_us unwrapped)
        events.csv        start/stop, markers, IMU stalls/restarts, warnings (PC unix time + board time)
        clock_sync.csv    ping round trips used to map board time -> PC time
        daq_raw.f32       NI-DAQ samples (if the DAQ was recorded), see daq_meta.json
        daq_pulses.csv    detected SYNC pulses (rise time, width)
        imu_synced.csv    imu.csv + pc_unix_s (+ daq_time_s when the SYNC check passed)
        summary.json      quality report written at stop
"""
import csv
import json
import os
import re
import time

import numpy as np


def _safe(s):
    s = re.sub(r"[^A-Za-z0-9_\-]+", "_", (s or "").strip())
    return s.strip("_")[:40]


class Session:
    def __init__(self, root, meta, pod_names):
        self.started_unix = time.time()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.started_unix))
        parts = [stamp] + [p for p in (_safe(meta.get("subject")), _safe(meta.get("label"))) if p]
        self.name = "_".join(parts)
        self.dir = os.path.join(root, self.name)
        os.makedirs(self.dir, exist_ok=True)
        self.meta = dict(meta)
        self.pod_names = list(pod_names)
        cols = ["frame", "t_us"]
        for p in self.pod_names:
            cols += [f"{p}_ok", f"{p}_fresh", f"{p}_w", f"{p}_x", f"{p}_y", f"{p}_z"]
        self.columns = cols
        self._imu = open(os.path.join(self.dir, "imu.csv"), "w", newline="")
        self._imu.write(",".join(cols) + "\n")
        self._ev = open(os.path.join(self.dir, "events.csv"), "w", newline="")
        self._evw = csv.writer(self._ev)
        self._evw.writerow(["pc_unix_s", "pc_time", "board_t_us", "frame", "type", "detail"])
        self._pg = open(os.path.join(self.dir, "clock_sync.csv"), "w", newline="")
        self._pgw = csv.writer(self._pg)
        self._pgw.writerow(["token", "pc_send_unix_s", "pc_recv_unix_s", "rtt_ms", "board_t_us"])
        self.ping_samples = []           # (board_us, pc_send_unix, pc_recv_unix)
        self.frames = 0
        self.first_frame = None
        self.last_frame = None
        self.missing = 0
        self.markers = 0
        self.last_flush = time.time()
        self.closed = False
        self.write_meta()

    # ---------------- writing ----------------
    def write_meta(self, extra=None):
        if extra:
            self.meta.update(extra)
        with open(os.path.join(self.dir, "session.json"), "w") as f:
            json.dump(self.meta, f, indent=2)

    def frame(self, num, t_us, vals):
        if self.closed:
            return
        if self.last_frame is not None and num > self.last_frame + 1:
            self.missing += num - self.last_frame - 1
        if self.first_frame is None:
            self.first_frame = num
        self.last_frame = num
        self.frames += 1
        self._imu.write(f"{num},{t_us}," + ",".join(
            (str(int(v)) if (k % 6) < 2 else f"{v:.5f}") for k, v in enumerate(vals)) + "\n")
        now = time.time()
        if now - self.last_flush > 1.0:
            self.flush()
            self.last_flush = now

    def event(self, pc_unix, kind, detail="", board_t_us="", frame=""):
        if self.closed:
            return
        iso = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(pc_unix)) + f".{int((pc_unix % 1) * 1000):03d}"
        self._evw.writerow([f"{pc_unix:.6f}", iso, board_t_us, frame, kind, detail])
        if kind == "marker":
            self.markers += 1

    def ping(self, token, send_unix, recv_unix, board_us):
        if self.closed:
            return
        self._pgw.writerow([token, f"{send_unix:.6f}", f"{recv_unix:.6f}",
                            f"{(recv_unix - send_unix) * 1e3:.3f}", board_us])
        self.ping_samples.append((board_us, send_unix, recv_unix))

    def flush(self):
        for f in (self._imu, self._ev, self._pg):
            try:
                f.flush()
            except Exception:
                pass

    # ---------------- finish ----------------
    def finalize(self, clock_fit, frame_us, stop_info, daq_meta=None, daq_pulses=None, daq_align=None):
        """clock_fit: function(list of (board_s, pc_mid_s, rtt_s)) -> ClockSync."""
        self.flush()
        for f in (self._imu, self._ev, self._pg):
            f.close()
        self.closed = True
        summary = {"session": self.name, "started": time.strftime("%Y-%m-%d %H:%M:%S",
                                                                  time.localtime(self.started_unix)),
                   "frames_received": self.frames, "first_frame": self.first_frame,
                   "last_frame": self.last_frame, "missing_frames": self.missing,
                   "markers": self.markers, "frame_us": frame_us, "board_stop": stop_info}
        frames_sent = int(stop_info.get("frames", self.frames)) if stop_info else self.frames
        summary["frames_sent_by_board"] = frames_sent
        try:
            import pandas as pd
            df = pd.read_csv(os.path.join(self.dir, "imu.csv"))
        except Exception as e:
            summary["error"] = f"could not read imu.csv: {e}"
            df = None
        if df is not None and len(df):
            t = df["t_us"].to_numpy(dtype=np.float64)
            dur = (t[-1] - t[0]) / 1e6 if len(t) > 1 else 0.0
            summary["duration_s"] = round(dur, 3)
            summary["rate_hz"] = round((len(t) - 1) / dur, 3) if dur > 0 else None
            if len(t) > 1:
                dt = np.diff(t) / 1e3
                summary["frame_interval_ms"] = {"mean": round(float(dt.mean()), 4), "sd": round(float(dt.std()), 4),
                                                "min": round(float(dt.min()), 3), "max": round(float(dt.max()), 3),
                                                "over_2x_period": int(np.sum(dt > 2 * frame_us / 1e3))}
            pods = {}
            for p in self.pod_names:
                fr = df[f"{p}_fresh"].to_numpy()
                ok = df[f"{p}_ok"].to_numpy()
                run = best = 0
                for v in fr:
                    run = 0 if v else run + 1
                    best = best if best > run else run
                pods[p] = {"found": bool(ok.max() > 0), "fresh_pct": round(100.0 * fr.mean(), 2),
                           "longest_gap_frames": int(best)}
            summary["pods"] = pods
            # PC time for every frame from the ping fit over this session
            cs = clock_fit([(b / 1e6, (s + r) / 2, r - s) for b, s, r in self.ping_samples]) \
                if self.ping_samples else None
            if cs and cs.fit:
                a, b = cs.fit
                df.insert(2, "pc_unix_s", np.round(a + b * t / 1e6, 6))
                summary["clock_sync"] = cs.stats
            else:
                summary["clock_sync"] = {"error": "no ping samples"}
            # DAQ time per frame (pulse k = frame k)
            if daq_align is not None:
                frame_times, rep = daq_align
                summary["daq_sync"] = rep
                if frame_times is not None and len(frame_times):
                    fr_idx = df["frame"].to_numpy()
                    daq_t = np.full(len(df), np.nan)
                    m = (fr_idx >= 0) & (fr_idx < len(frame_times))
                    daq_t[m] = frame_times[fr_idx[m]]
                    df.insert(3 if "pc_unix_s" in df else 2, "daq_time_s", np.round(daq_t, 6))
                    try:
                        from daq import board_vs_daq
                        rep["board_vs_daq"] = board_vs_daq(t, daq_t)
                    except Exception as e:
                        rep["board_vs_daq"] = {"error": str(e)}
            df.to_csv(os.path.join(self.dir, "imu_synced.csv"), index=False)
        if daq_meta is not None:
            summary["daq"] = daq_meta
        if daq_pulses is not None and daq_meta:
            rate = daq_meta.get("rate_hz") or 1
            with open(os.path.join(self.dir, "daq_pulses.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["pulse", "rise_s", "width_ms"])
                for k, (r, wd) in enumerate(daq_pulses):
                    w.writerow([k, f"{r / rate:.6f}", f"{wd / rate * 1e3:.3f}"])
        with open(os.path.join(self.dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2, default=float)
        return summary
