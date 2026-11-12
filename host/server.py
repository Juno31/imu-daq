"""IMU DAQ — acquisition server + browser GUI.

    conda activate daq
    python server.py              (opens http://127.0.0.1:8765 in the browser)
    python server.py --no-browser --port 8765 --data D:\\recordings

The server owns the serial link, the recordings and the NI-DAQ; the browser page is only a view
with buttons. Closing the browser does not stop a recording; closing this window does (cleanly).
"""
import argparse
import collections
import json
import os
import socket
import sys
import threading
import time
import traceback
import urllib.error
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import daq as daqmod                      # noqa: E402
from clocksync import ClockSync           # noqa: E402
from syncmon import SyncMonitor           # noqa: E402
import channels                           # noqa: E402
from device import Device, guess_port, list_serial_ports   # noqa: E402
from session import Session               # noqa: E402

APP_VERSION = "1.1.0"
STATIC = os.path.join(HERE, "static")
DEFAULT_DATA = os.path.join(os.path.dirname(HERE), "data")


class App:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        os.makedirs(data_dir, exist_ok=True)
        self.lock = threading.RLock()
        self.perf0, self.unix0 = time.perf_counter(), time.time()
        self.dev = Device(self._on_frame, self._on_msg, self._on_pong, self._on_disconnect)
        self.clock = ClockSync()
        self.frames = collections.deque(maxlen=120 * 200)   # (seq, row) row=[t_us, kind, num, *vals]
        self.fseq = 0
        self.logbuf = collections.deque(maxlen=600)          # (seq, dict)
        self.lseq = 0
        self.state = "disconnected"       # disconnected, connecting, idle, starting, recording, stopping
        self.want_port = None             # None = auto-detect
        self.auto_connect = True
        self.connect_at = 0.0
        self.hello_at = None
        self.last_connect_try = 0.0
        self.last_ping = self.last_status = 0.0
        self.ping_n = 0
        self.pending = {}
        self.fw_status, self.fw_pods = {}, {}
        self.pod_live = {}
        self.recent_t = collections.deque(maxlen=101)
        self.session = None
        self.daq = None
        self.syncmon = None               # live SYNC matching view (kept after Stop)
        self.preview = None               # DAQ running while idle (nothing saved): live levels
        self.preview_err = ""
        self.preview_retry = 0.0
        self.preview_lock = threading.RLock()
        self.preview_on = True            # GUI "Preview" toggle
        self.daq_cfg = {"enabled": True, "device": "", "sync_chan": "ai0", "extra_chans": channels.DEFAULT_EXTRA,
                        "rate": 20000, "range": 5.0, "terminal": "DIFF", "require": False}
        self.daq_devices = daqmod.list_devices()
        if self.daq_devices["devices"]:
            self.daq_cfg["device"] = self.daq_devices["devices"][0]["name"]
        self.rec = {}
        self.last_summary = None
        self.start_evt, self.stop_evt = threading.Event(), threading.Event()
        self.stop_info = {}
        self.busy = threading.Lock()      # one start/stop at a time
        self.stray_noted = False
        threading.Thread(target=self._maintain, name="maintain", daemon=True).start()

    # ---------------- helpers ----------------
    def unix(self, perf=None):
        return self.unix0 + ((time.perf_counter() if perf is None else perf) - self.perf0)

    def log(self, level, text, source="host"):
        with self.lock:
            self.lseq += 1
            self.logbuf.append((self.lseq, {"t": round(time.time(), 3), "level": level,
                                            "src": source, "text": text}))
        print(f"[{time.strftime('%H:%M:%S')}] {level.upper():5s} {source}: {text}", flush=True)

    def _pod(self, name):
        if name not in self.pod_live:
            self.pod_live[name] = {"fresh": collections.deque(maxlen=100), "last_fresh": 0.0,
                                   "stalls": 0, "restarts": 0, "q": [1, 0, 0, 0], "ok": 0,
                                   "rec_frames": 0, "rec_fresh": 0}
        return self.pod_live[name]

    # ---------------- device callbacks (reader thread) ----------------
    def _on_frame(self, kind, num, t_us, vals, perf):
        with self.lock:
            self.fseq += 1
            self.frames.append((self.fseq, [t_us, kind, num] + vals))
            self.recent_t.append(t_us)
            rec = kind == "F" and self.session is not None and not self.session.closed
            for k, name in enumerate(self.dev.pod_names):
                p = self._pod(name)
                ok, fresh = int(vals[6 * k]), int(vals[6 * k + 1])
                p["ok"] = ok
                p["fresh"].append(fresh)
                p["q"] = vals[6 * k + 2: 6 * k + 6]
                if fresh:
                    p["last_fresh"] = perf
                if rec:
                    p["rec_frames"] += 1
                    p["rec_fresh"] += fresh
            if rec:
                self.session.frame(num, t_us, vals)
                if self.syncmon is not None:
                    self.syncmon.add_frame(num, t_us)
                self.rec["last_frame"] = num
                self.rec["last_t_us"] = t_us
            elif kind == "F" and self.session is None and not self.stray_noted:
                self.stray_noted = True
                threading.Thread(target=self._stop_stray, daemon=True).start()

    def _stop_stray(self):
        self.log("warn", "board was recording without a session (left over from before); stopping it")
        try:
            self.dev.send("stop")
        except Exception:
            pass

    def _on_msg(self, tag, text, fields, perf):
        unix = self.unix(perf)
        sess = self.session if (self.session and not self.session.closed) else None
        if tag == "HELLO":
            with self.lock:
                self.hello_at = time.time()
                if self.state in ("connecting", "disconnected"):
                    self.state = "idle"
            self.log("info", f"{fields.get('fw', '?')} {fields.get('ver', '?')} on {fields.get('board', '?')}, "
                             f"{1000 / max(1, int(fields.get('report_ms', 10))):.0f} Hz, pods {fields.get('pods', '')}",
                     "board")
            if fields.get("rec") == "1" and self.session is None:
                threading.Thread(target=self._stop_stray, daemon=True).start()
        elif tag == "STATUS":
            self.fw_status = fields
        elif tag == "POD":
            self.fw_pods[fields.get("name", "?")] = fields
        elif tag == "START":
            with self.lock:
                self.rec["board_start_t_us"] = int(fields.get("t_us", 0))
                self.state = "recording"
            if sess:
                sess.event(unix, "start", text, fields.get("t_us", ""), 0)
            self.start_evt.set()
        elif tag == "STOP":
            self.stop_info = fields
            if sess:
                sess.event(unix, "stop", text, fields.get("t_us", ""), fields.get("frames", ""))
            self.stop_evt.set()
        elif tag == "EVENT":
            name, typ = fields.get("pod", "-"), fields.get("type", "?")
            with self.lock:
                if name in self.pod_live:
                    if typ == "stall":
                        self.pod_live[name]["stalls"] += 1
                    elif typ == "restarted":
                        self.pod_live[name]["restarts"] += 1
            level = "info" if typ in ("restarted", "found") else "warn"
            self.log(level, f"{name} {typ}: {text}", "board")
            if sess:
                sess.event(unix, f"board_{typ}", text, "", fields.get("frame", ""))
        elif tag in ("ERR",):
            self.log("error", text, "board")
            if sess:
                sess.event(unix, "board_error", text)
        elif tag == "RATE":
            self.dev.hello.update(fields)
            self.log("info", f"rate set: {text}", "board")
        elif tag == "ACK":
            pass
        elif tag == "#":
            if not text.startswith("columns:"):
                self.log("info", text, "board")
        else:
            self.log("info", f"@{tag} {text}", "board")

    def _on_pong(self, tok, t_us, perf):
        send = self.pending.pop(tok, None)
        if send is None:
            return
        su, ru = self.unix(send), self.unix(perf)
        last = getattr(self, "_last_pong_us", None)
        if last is not None and t_us < last - 1_000_000:
            # the board clock went backwards: it rebooted (or a different board). Old ping samples
            # belong to the previous clock, so start the PC<->board fit again.
            self.clock = ClockSync()
            self.dev.unwrap.reset()
            t_us = self.dev.unwrap(t_us % (1 << 32))
            self.log("warn", "board clock restarted (board reset?) - clock sync restarted", "host")
        self._last_pong_us = t_us
        self.clock.add(t_us, su, ru)
        sess = self.session
        if sess and not sess.closed:
            sess.ping(tok, su, ru, t_us)

    def _on_disconnect(self, reason):
        self.log("error", f"board disconnected ({reason})")
        with self.lock:
            was_rec = self.state in ("recording", "starting")
            self.state = "disconnected"
            self.hello_at = None
        if was_rec:
            threading.Thread(target=self._finish, args=("board disconnected during recording",),
                             daemon=True).start()

    # ---------------- maintenance thread ----------------
    def _maintain(self):
        while True:
            try:
                now = time.time()
                if not self.dev.connected:
                    if self.auto_connect and now - self.last_connect_try > 2.0:
                        self.last_connect_try = now
                        port = self.want_port or guess_port()
                        if port:
                            try:
                                self.state = "connecting"
                                self.connect_at = now
                                self.pending.clear()
                                self.dev.connect(port)
                                self.log("info", f"opened {port}")
                            except Exception as e:
                                self.state = "disconnected"
                                msg = str(e)
                                if "PermissionError" in msg or "Access" in msg or "액세스" in msg:
                                    msg = f"{port} is in use by another program (Arduino Serial Monitor?)"
                                if getattr(self, "_last_conn_err", None) != msg:
                                    self.log("warn", f"cannot open {port}: {msg}")
                                    self._last_conn_err = msg
                else:
                    self._last_conn_err = None
                    if self.hello_at is None and now - self.connect_at > 4.0:
                        self.connect_at = now
                        self.log("warn", "no @HELLO from the board: is the imu_daq firmware uploaded? (retrying)")
                        self.dev.send("hello")
                    if now - self.last_ping >= 1.0:
                        self.last_ping = now
                        self.ping_n += 1
                        tok = str(self.ping_n)
                        self.pending[tok] = time.perf_counter()
                        for k in [k for k, v in self.pending.items() if time.perf_counter() - v > 3]:
                            self.pending.pop(k, None)
                        self.dev.send(f"ping {tok}")
                    if now - self.last_status >= 2.0:
                        self.last_status = now
                        self.dev.send("status")
                self._manage_preview(now)
                d = self.daq
                if d is not None and d.error and not getattr(d, "_error_logged", False):
                    d._error_logged = True
                    self.log("error", d.error, "daq")
                    if self.session:
                        self.session.event(self.unix(), "daq_error", d.error)
            except Exception as e:
                self.log("error", f"maintenance: {e}")
            time.sleep(0.2)

    # ---------------- commands ----------------
    def connect(self, port):
        if self.state in ("recording", "starting", "stopping"):
            raise RuntimeError("stop the recording first")
        self.dev.disconnect()
        self.state = "disconnected"
        self.want_port = port or None
        self.auto_connect = True
        self.last_connect_try = 0.0

    def disconnect(self):
        if self.state in ("recording", "starting", "stopping"):
            raise RuntimeError("stop the recording first")
        self.auto_connect = False
        self.dev.disconnect()
        self.state = "disconnected"
        self.hello_at = None
        self.log("info", "disconnected")

    def send_simple(self, cmd):
        if self.state != "idle":
            raise RuntimeError("only possible while idle")
        self.dev.send(cmd)

    # ---------------- DAQ preview (idle) ----------------
    def _stop_preview(self):
        with self.preview_lock:
            p, self.preview = self.preview, None
        if p is not None:
            p.stop()

    def _manage_preview(self, now):
        with self.preview_lock:
            cfg = self.daq_cfg
            want = (self.preview_on and self.state in ("idle", "disconnected", "connecting") and cfg["enabled"]
                    and cfg["device"] and daqmod.HAVE_NIDAQMX)
            p = self.preview
            if p is not None:
                if p.error:
                    err = p.error.splitlines()[0][:200]
                    if err != self.preview_err:
                        self.log("warn", f"DAQ preview stopped: {err}", "daq")
                    self.preview_err, self.preview_retry = err, now + 3.0
                    self._stop_preview()
                elif not want:
                    self._stop_preview()
                return
            if not want or now < self.preview_retry:
                return
            try:
                r = daqmod.DaqRecorder(None, cfg["device"], cfg["sync_chan"],
                                       [c.strip() for c in str(cfg["extra_chans"]).split(",") if c.strip()],
                                       cfg["rate"], cfg["range"], cfg["terminal"])
                r.start(self.unix)
                self.preview, self.preview_err = r, ""
            except Exception as e:
                try:
                    r.stop()
                except Exception:
                    pass
                err = str(e).splitlines()[0][:200] if str(e) else repr(e)
                if err != self.preview_err:
                    self.log("warn", f"DAQ preview failed: {err}", "daq")
                self.preview_err, self.preview_retry = err, now + 5.0

    def set_preview(self, on):
        self.preview_on = bool(on)
        self.preview_err, self.preview_retry = "", 0.0
        if not self.preview_on:
            self._stop_preview()
        self.log("info", "DAQ preview " + ("on" if self.preview_on else "off"), "user")

    def set_daq(self, cfg):
        if self.state in ("recording", "starting", "stopping"):
            raise RuntimeError("stop the recording first")
        for k in self.daq_cfg:
            if k in cfg:
                self.daq_cfg[k] = cfg[k]
        self.daq_cfg["rate"] = float(self.daq_cfg["rate"])
        self.daq_cfg["range"] = float(self.daq_cfg["range"])
        self._stop_preview()                 # restart the preview with the new settings
        self.preview_err, self.preview_retry = "", 0.0

    def refresh_daq(self):
        self._stop_preview()
        self.preview_err, self.preview_retry = "", 0.0
        self.daq_devices = daqmod.list_devices()
        names = [d["name"] for d in self.daq_devices["devices"]]
        if names and self.daq_cfg["device"] not in names:
            self.daq_cfg["device"] = names[0]
        return self.daq_devices

    def marker(self, text):
        sess = self.session
        frame = self.rec.get("last_frame", "")
        self.log("info", f"marker: {text or '(no text)'}" + (f" at frame {frame}" if sess else " (not recording)"),
                 "user")
        if sess and not sess.closed:
            sess.event(self.unix(), "marker", text or "", self.rec.get("last_t_us", ""), frame)

    def start(self, meta):
        if not self.busy.acquire(blocking=False):
            raise RuntimeError("busy")
        try:
            if self.state != "idle":
                raise RuntimeError(f"cannot start while {self.state}")
            if not self.dev.pod_names:
                raise RuntimeError("board has not reported its IMUs yet")
            with self.preview_lock:
                self.state = "starting"
            self._stop_preview()                 # the recording task needs the DAQ
            hello = dict(self.dev.hello)
            frame_us = int(hello.get("frame_us", 10000))
            meta = dict(meta)
            meta.update({"app_version": APP_VERSION, "firmware": hello, "port": self.dev.port,
                         "pc_start_unix_s": time.time(), "daq_config": dict(self.daq_cfg)})
            sess = Session(self.data_dir, meta, self.dev.pod_names)
            save_spine_cal(sess.dir, getattr(self, "spine_cal", None))
            with self.lock:
                for p in self.pod_live.values():
                    p["rec_frames"] = p["rec_fresh"] = 0
                self.rec = {"t0": time.time(), "frame_us": frame_us}
                self.session = sess
            self.log("info", f"session {sess.name}")
            # DAQ first, so it sees the frame-0 pulse
            self.daq = None
            cfg = self.daq_cfg
            if cfg["enabled"]:
                if not cfg["device"]:
                    msg = "DAQ enabled but no NI-DAQmx device found; recording IMU only"
                    if cfg.get("require"):
                        raise RuntimeError(msg)
                    self.log("warn", msg, "daq")
                    sess.event(self.unix(), "daq_missing", msg)
                else:
                    try:
                        d = daqmod.DaqRecorder(sess.dir, cfg["device"], cfg["sync_chan"],
                                               [c.strip() for c in str(cfg["extra_chans"]).split(",") if c.strip()],
                                               cfg["rate"], cfg["range"], cfg["terminal"])
                        d.start(self.unix)
                        self.daq = d
                        self.log("info", f"DAQ {cfg['device']} recording {', '.join(d.channels)} at "
                                         f"{cfg['rate']:.0f} Hz", "daq")
                        time.sleep(0.3)
                    except Exception as e:
                        msg = f"DAQ start failed: {e}"
                        sess.event(self.unix(), "daq_error", msg)
                        if cfg.get("require"):
                            raise RuntimeError(msg)
                        self.log("error", msg + " — recording IMU only", "daq")
            self.syncmon = SyncMonitor(frame_us, self.daq.rate if self.daq else None)
            self.start_evt.clear()
            self.dev.send("start")
            if not self.start_evt.wait(3.0):
                raise RuntimeError("board did not confirm start (@START)")
            self.log("info", "recording")
            return {"session": sess.name, "dir": sess.dir}
        except Exception as e:
            self.log("error", f"start failed: {e}")
            if self.session and not self.session.closed:
                self._finish(f"start failed: {e}", locked=True)
            elif self.state == "starting":
                self.state = "idle"
            raise
        finally:
            self.busy.release()

    def stop(self):
        if not self.busy.acquire(timeout=5):
            raise RuntimeError("busy")
        try:
            if self.state != "recording":
                raise RuntimeError(f"not recording ({self.state})")
            return self._finish(None, locked=True)
        finally:
            self.busy.release()

    def _finish(self, reason, locked=False):
        """Stop the board and DAQ, write the session files. Returns the summary."""
        sess = self.session
        if sess is None or sess.closed:
            return None
        self.state = "stopping"
        self.stop_evt.clear()
        self.stop_info = {}
        if self.dev.connected:
            try:
                self.dev.send("stop")
                if not self.stop_evt.wait(3.0):
                    self.log("warn", "board did not confirm stop (@STOP)")
            except Exception as e:
                self.log("warn", f"stop command failed: {e}")
        time.sleep(0.4)                     # let the last pulses and lines arrive
        if reason:
            sess.event(self.unix(), "abort", reason)
        daq_meta = pulses = align = None
        if self.daq is not None:
            d, self.daq = self.daq, None
            daq_meta = d.stop()
            if self.syncmon is not None:
                self.syncmon.update(d, final=True)
            pulses = list(d.edges.pulses)
            frames_sent = int(self.stop_info.get("frames", sess.frames))
            align = daqmod.align(pulses, d.rate, frames_sent, self.rec.get("frame_us", 10000))
        if self.syncmon is not None and self.syncmon.active:
            self.syncmon.update(None, final=True)
        summary = sess.finalize(lambda s: ClockSync().fit_samples(s), self.rec.get("frame_us", 10000),
                                self.stop_info, daq_meta, pulses, align)
        summary["dir"] = sess.dir
        if reason:
            summary["aborted"] = reason
        with self.lock:
            self.last_summary = summary
            self.session = None
            self.stray_noted = False
            self.state = "idle" if self.dev.connected and self.hello_at else "disconnected"
        pods = ", ".join(f"{k} {v['fresh_pct']:.1f}%" for k, v in summary.get("pods", {}).items())
        ds = summary.get("daq_sync")
        sync = "" if ds is None else (" | SYNC PASS" if ds.get("ok") else f" | SYNC FAIL: {ds.get('reason')}")
        self.log("info" if not reason else "warn",
                 f"saved {sess.name}: {summary.get('frames_received')} frames, "
                 f"{summary.get('missing_frames')} missing, {summary.get('duration_s', 0)} s, {pods}{sync}")
        return summary

    def shutdown(self):
        if self.session and not self.session.closed:
            self.log("warn", "closing: stopping the recording")
            self._finish("server closed during recording")
        self.auto_connect = False
        self._stop_preview()
        self.dev.disconnect()

    # ---------------- views ----------------
    def frames_since(self, cursor, limit=600):
        out = []
        with self.lock:
            for seq, row in reversed(self.frames):
                if seq <= cursor:
                    break
                out.append(row)
                if len(out) >= limit:
                    break
            last = self.fseq
        out.reverse()
        return last, out

    def log_since(self, cursor):
        with self.lock:
            items = [d for s, d in self.logbuf if s > cursor]
            return self.lseq, items

    def status(self):
        with self.lock:
            t = list(self.recent_t)
            rate = (len(t) - 1) / ((t[-1] - t[0]) / 1e6) if len(t) > 10 and t[-1] > t[0] else 0.0
            now_perf = time.perf_counter()
            pods = []
            for name in self.dev.pod_names:
                p = self._pod(name)
                fr = p["fresh"]
                fw = self.fw_pods.get(name, {})
                pods.append({
                    "name": name, "ok": p["ok"],
                    "addr": next((x.split(":")[1] for x in self.dev.hello.get("pods", "").split(",")
                                  if x.startswith(name + ":")), ""),
                    "fresh_recent": round(100.0 * sum(fr) / len(fr), 1) if fr else 0.0,
                    "silent_s": round(now_perf - p["last_fresh"], 2) if p["last_fresh"] else None,
                    "live": fw.get("live", "1") == "1",
                    "stalls": p["stalls"], "restarts": p["restarts"],
                    "rec_fresh": round(100.0 * p["rec_fresh"] / p["rec_frames"], 1) if p["rec_frames"] else None,
                })
            sess = self.session
            rec = None
            if sess and not sess.closed:
                rec = {"name": sess.name, "dir": sess.dir, "elapsed": round(time.time() - self.rec.get("t0", time.time()), 1),
                       "frames": sess.frames, "missing": sess.missing, "markers": sess.markers,
                       "last_frame": self.rec.get("last_frame")}
            d = self.daq
            sm = self.syncmon
            out = {
                "app_version": APP_VERSION, "state": self.state, "port": self.dev.port if self.dev.connected else None,
                "want_port": self.want_port, "hello": self.dev.hello, "rate_hz": round(rate, 2),
                "board": self.fw_status, "pods": pods, "rec": rec, "clock": self.clock.snapshot(),
                "daq": {"cfg": self.daq_cfg, "devices": self.daq_devices, "live": d.live() if d else None,
                        "preview": self.preview.live() if self.preview else None,
                        "preview_error": self.preview_err, "preview_on": self.preview_on},
                "last_summary": self.last_summary, "data_dir": self.data_dir,
                "bad_lines": self.dev.bad_lines,
            }
        # sync monitor + clock plot (outside the main lock: the frame reader must not wait on it)
        if sm is not None:
            if sm.active and d is not None:
                sm.update(d)
            elif sm.active and self.state == "recording":
                sm.update(None)
            out["sync"] = sm.snapshot()
        else:
            out["sync"] = None
        out["clockplot"] = self.clock.plot_data()
        src = d if d is not None else self.preview
        try:
            out["emg"] = src.emg_view() if src is not None else None
        except Exception:
            out["emg"] = None
        return out

    def sessions(self, limit=30):
        out = []
        try:
            names = sorted(os.listdir(self.data_dir), reverse=True)[:limit]
        except FileNotFoundError:
            names = []
        for n in names:
            p = os.path.join(self.data_dir, n, "summary.json")
            if not os.path.exists(p):
                continue
            try:
                with open(p) as f:
                    s = json.load(f)
                ds = s.get("daq_sync")
                folder = os.path.join(self.data_dir, n)
                files = []
                for e in sorted(os.scandir(folder), key=lambda e: e.name):
                    if e.is_file():
                        files.append({"name": e.name, "bytes": e.stat().st_size})
                meta = {}
                try:
                    with open(os.path.join(folder, "session.json"), encoding="utf-8") as f:
                        meta = json.load(f)
                except Exception:
                    pass
                out.append({"name": n, "dir": folder, "files": files,
                            "bytes": sum(f["bytes"] for f in files), "started": s.get("started"),
                            "subject": meta.get("subject", ""), "trial": meta.get("label", ""),
                            "notes": meta.get("notes", ""),
                            "frames": s.get("frames_received"), "missing": s.get("missing_frames"),
                            "duration_s": s.get("duration_s"),
                            "pods": {k: v.get("fresh_pct") for k, v in s.get("pods", {}).items()},
                            "sync": None if ds is None else bool(ds.get("ok")),
                            "aborted": s.get("aborted")})
            except Exception:
                pass
        return out


# ---------------- HTTP ----------------
APP = None
def save_spine_cal(folder, cal, name="spine_cal.json"):
    """Store the Spine view calibration (neutral quaternions, body axes) next to a recording."""
    if not cal or not folder:
        return
    try:
        with open(os.path.join(folder, name), "w", encoding="utf-8") as f:
            json.dump(cal, f, indent=1)
    except OSError:
        pass


MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".png": "image/png"}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj, default=float).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path in ("/", "/index.html"):
                return self._file("index.html")
            if path in ("/spine", "/spine.html"):
                return self._file("spine.html")
            if path.startswith("/static/"):
                return self._file(path[len("/static/"):])
            if path == "/api/state":
                return self._json(APP.status())
            if path == "/api/ports":
                return self._json({"ports": list_serial_ports(), "guess": guess_port()})
            if path == "/api/daq/devices":
                return self._json(APP.refresh_daq())
            if path == "/api/sessions":
                return self._json({"sessions": APP.sessions()})
            if path == "/api/stream":
                return self._stream()
            self._json({"error": "not found"}, 404)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _file(self, rel):
        full = os.path.normpath(os.path.join(STATIC, rel))
        if not full.startswith(STATIC) or not os.path.isfile(full):
            return self._json({"error": "not found"}, 404)
        with open(full, "rb") as f:
            body = f.read()
        self.send_response(200)
        self.send_header("Content-Type", MIME.get(os.path.splitext(full)[1], "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        fcur = max(0, APP.fseq - 1000)     # start with the last ~10 s
        lcur = max(0, APP.lseq - 200)
        last_status = 0.0
        while True:
            fcur, rows = APP.frames_since(fcur)
            lcur, logs = APP.log_since(lcur)
            msg = {"f": rows}
            if logs:
                msg["log"] = logs
            now = time.time()
            if now - last_status > 0.25:
                msg["s"] = APP.status()
                last_status = now
            if rows or logs or "s" in msg:
                data = json.dumps(msg, separators=(",", ":"), default=float)
                self.wfile.write(f"data: {data}\n\n".encode())
                self.wfile.flush()
            time.sleep(0.04)

    def do_POST(self):
        path = urlparse(self.path).path
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            body = {}
        try:
            if path == "/api/start":
                return self._json({"ok": True, **APP.start(body)})
            if path == "/api/stop":
                if APP.state == "recording":
                    APP.log("info", "stop requested (Stop button / API)", "user")
                return self._json({"ok": True, "summary": APP.stop()})
            if path == "/api/marker":
                APP.marker(str(body.get("text", ""))[:200])
                return self._json({"ok": True})
            if path == "/api/connect":
                APP.connect(body.get("port"))
                return self._json({"ok": True})
            if path == "/api/disconnect":
                APP.disconnect()
                return self._json({"ok": True})
            if path == "/api/restart":
                APP.send_simple("restart")
                return self._json({"ok": True})
            if path == "/api/rate":
                ms = int(body.get("ms", 10))
                APP.send_simple(f"rate {ms}")
                return self._json({"ok": True})
            if path == "/api/preview":
                APP.set_preview(body.get("on", True))
                return self._json({"ok": True, "on": APP.preview_on})
            if path == "/api/daq":
                APP.set_daq(body)
                return self._json({"ok": True, "cfg": APP.daq_cfg})
            if path == "/api/spine_cal":          # calibration from the Spine view
                APP.spine_cal = body
                save_spine_cal(APP.data_dir, body, "spine_cal_last.json")
                sess = APP.session
                if sess and not sess.closed:
                    save_spine_cal(sess.dir, body)
                    sess.event(APP.unix(), "spine_cal", json.dumps(body.get("map")), "", "")
                return self._json({"ok": True})
            if path == "/api/open":
                # open a recording folder, or one file in it, with the Windows default program
                d = body.get("file") or body.get("dir") or APP.data_dir
                full = os.path.abspath(d)
                root = os.path.abspath(APP.data_dir)
                if not (full == root or full.startswith(root + os.sep)):
                    return self._json({"ok": False, "error": "only files in the data folder can be opened"}, 400)
                if body.get("file") and body.get("reveal") and os.path.isfile(full):
                    import subprocess
                    subprocess.Popen(["explorer", "/select,", full])
                elif os.path.exists(full):
                    os.startfile(full)
                else:
                    return self._json({"ok": False, "error": "not found"}, 404)
                return self._json({"ok": True})
            self._json({"ok": False, "error": "not found"}, 404)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        except Exception as e:
            self._json({"ok": False, "error": str(e)}, 400)


class ExclusiveHTTPServer(ThreadingHTTPServer):
    """On Windows, SO_REUSEADDR (Python's default for HTTPServer) lets several processes bind the
    same port. Use an exclusive bind so a second server cannot start silently."""
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def main():
    global APP
    for stream in (sys.stdout, sys.stderr):      # Korean Windows consoles are cp949
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description="IMU DAQ acquisition server + browser GUI")
    ap.add_argument("--port", type=int, default=8765, help="web port (default 8765)")
    ap.add_argument("--data", default=DEFAULT_DATA, help="folder for recordings")
    ap.add_argument("--serial", default=None, help="serial port (default: auto-detect the XIAO)")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()
    # Only one server may own the board: a second copy would fight over the COM port and reset
    # the XIAO mid-recording. If one is already running, just show its page and exit.
    url = f"http://127.0.0.1:{args.port}/"
    try:
        with urllib.request.urlopen(url + "api/state", timeout=1.5) as r:
            if r.status == 200:
                print(f"IMU DAQ is already running at {url} - opening it in the browser.")
                print("(Only one server can use the board. Close the other server window to restart it.)")
                if not args.no_browser:
                    webbrowser.open(url)
                sys.exit(0)
    except (urllib.error.URLError, OSError, ValueError):
        pass
    try:
        srv = ExclusiveHTTPServer(("127.0.0.1", args.port), Handler)
    except OSError as e:
        print(f"web port {args.port} is in use by another program ({e}); try --port {args.port + 1}")
        sys.exit(1)
    APP = App(os.path.abspath(args.data))
    APP.want_port = args.serial
    srv.daemon_threads = True
    url = f"http://127.0.0.1:{srv.server_address[1]}/"
    APP.log("info", f"IMU DAQ {APP_VERSION} at {url} - recordings in {APP.data_dir}")
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever(poll_interval=0.3)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            APP.shutdown()
        except Exception:
            traceback.print_exc()
        srv.server_close()
        print("closed")


if __name__ == "__main__":
    main()
