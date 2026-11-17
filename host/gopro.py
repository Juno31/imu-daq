"""GoPro control over USB (Open GoPro HTTP API), for HERO9 and newer, tested target: HERO13 Black.

Each camera plugged in by USB-C shows up as its own network adapter. The camera is at
172.2X.1YZ.51:8080 (X, Y, Z = last three digits of its serial number) and the PC gets an
address in the same /24, which is how cameras are found without any setup.

On each camera: Preferences > Connections > USB Connection > GoPro Connect (not MTP).

The rig is used by server.py:
    rig.start_all(unix)   - start recording on every enabled camera at once (host times logged)
    rig.stop_all(unix)    - stop them, then find the new files and download them in the background
    rig.snapshot()        - status for the GUI (battery, SD card, encoding, download progress)
"""
import json
import os
import re
import socket
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor

PORT = 8080
USB_NET = re.compile(r"^172\.2\d\.1\d\d\.\d+$")

# Open GoPro status ids (GET /gopro/camera/state -> "status")
ST_BUSY = "8"
ST_ENCODING = "10"
ST_VIDEO_SECS = "13"
ST_SD_STATUS = "33"
ST_SD_FREE_KB = "54"
ST_BATTERY_PCT = "70"


def local_usb_ips():
    """PC-side addresses on GoPro USB networks (172.2X.1YZ.n)."""
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if USB_NET.match(ip):
                ips.add(ip)
    except OSError:
        pass
    return sorted(ips)


def camera_ip_from_serial(serial):
    s = str(serial).strip()[-3:]
    if len(s) != 3 or not s.isdigit():
        raise ValueError(f"bad serial number {serial!r}")
    return f"172.2{s[0]}.1{s[1]}{s[2]}.51"


class GoPro:
    def __init__(self, ip):
        self.ip = ip
        self.base = f"http://{ip}:{PORT}"
        self.info = {}          # model, serial, firmware (from /gopro/camera/info)
        self.state = {}         # last status dict
        self.online = False
        self.error = ""
        self.last_seen = 0.0
        self.usb_ok = False     # wired USB control enabled

    def _get(self, path, timeout=3.0, raw=False):
        with urllib.request.urlopen(self.base + path, timeout=timeout) as r:
            data = r.read()
        if raw:
            return data
        try:
            return json.loads(data or b"{}")
        except ValueError:
            return {}

    # ---- control ----
    def enable_usb(self):
        self._get("/gopro/camera/control/wired_usb?p=1")
        self.usb_ok = True

    def keep_alive(self):
        self._get("/gopro/camera/keep_alive", timeout=2.0)

    def shutter(self, on):
        self._get("/gopro/camera/shutter/" + ("start" if on else "stop"), timeout=3.0)

    def poll(self):
        st = self._get("/gopro/camera/state", timeout=2.0).get("status", {})
        self.state = {k: st.get(k) for k in (ST_BUSY, ST_ENCODING, ST_VIDEO_SECS, ST_SD_STATUS,
                                              ST_SD_FREE_KB, ST_BATTERY_PCT)}
        self.online, self.error, self.last_seen = True, "", time.time()
        return self.state

    def load_info(self):
        try:
            self.info = self._get("/gopro/camera/info", timeout=2.0) or {}
        except Exception:
            self.info = {}

    @property
    def encoding(self):
        return bool(self.state.get(ST_ENCODING))

    @property
    def busy(self):
        return bool(self.state.get(ST_BUSY))

    @property
    def serial(self):
        return str(self.info.get("serial_number") or "")

    # ---- media ----
    def media_files(self):
        """Set of 'DIR/FILE' on the SD card."""
        out = set()
        for d in self._get("/gopro/media/list", timeout=5.0).get("media", []):
            for f in d.get("fs", []):
                out.add(f"{d.get('d')}/{f.get('n')}")
        return out

    def download(self, rel, dest, progress=None):
        """Download one file; progress(bytes_done, bytes_total). Writes to dest.part, then renames."""
        tmp = dest + ".part"
        with urllib.request.urlopen(f"{self.base}/videos/DCIM/{rel}", timeout=30) as r:
            total = int(r.headers.get("Content-Length") or 0)
            done = 0
            with open(tmp, "wb") as fh:
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    fh.write(chunk)
                    done += len(chunk)
                    if progress:
                        progress(done, total)
        os.replace(tmp, dest)
        return done


class GoProRig:
    """All GoPros (USB or Bluetooth): discovery, keep-alive/status polling, start/stop, download (USB only)."""

    def __init__(self, log, cfg=None):
        self.log = log                      # log(level, text, source)
        self.cfg = {"enabled": True, "transport": "usb", "download": True, "require": False,
                    "names": {}, "manual_ips": []}
        self.cfg.update(cfg or {})
        if self.cfg["transport"] not in ("usb", "ble"):
            self.cfg["transport"] = "usb"
        self.ble_names = {}                 # BLE address -> advertised name
        self.cams = {}                      # ip -> GoPro
        self.lock = threading.RLock()
        self.busy = threading.Event()       # start/stop in progress: the poller stays quiet
        self.rec = None                     # current take: {"t_start", "cams": {name: {...}}, "before": {...}}
        self.last_take = None               # last finished take summary (for the GUI)
        self.downloads = {}                 # name -> {"file", "done", "total", "state", "error"}
        self.pool = ThreadPoolExecutor(max_workers=6, thread_name_prefix="gopro")
        self._stop = False
        self._reset = False                 # transport changed: the poller drops all cameras
        threading.Thread(target=self._poller, name="gopro-poll", daemon=True).start()

    # ---------------- discovery / polling ----------------
    def name_of(self, cam):
        names = self.cfg.get("names") or {}
        if cam.serial and cam.serial in names:
            return names[cam.serial]
        with self.lock:
            order = sorted(self.cams)
        return f"cam{order.index(cam.ip) + 1}" if cam.ip in order else cam.ip

    @property
    def ble(self):
        return self.cfg.get("transport") == "ble"

    def _candidate_ips(self):
        if self.ble:
            import gopro_ble
            try:
                for addr, name in gopro_ble.scan():
                    self.ble_names[addr] = name
            except Exception as e:
                if not getattr(self, "_scan_warned", False):
                    self.log("warn", f"Bluetooth scan failed: {_err(e)}", "gopro")
                    self._scan_warned = True
            return set(self.ble_names)       # once seen, keep trying (connected cameras stop advertising)
        ips = {ip.rsplit(".", 1)[0] + ".51" for ip in local_usb_ips()}
        for x in self.cfg.get("manual_ips") or []:
            ips.add(str(x).strip())
        return {ip for ip in ips if ip}

    def _new_cam(self, key):
        if self.ble:
            import gopro_ble
            return gopro_ble.BleGoPro(key, self.ble_names.get(key, ""))
        return GoPro(key)

    def discover(self):
        found = self._candidate_ips()
        with self.lock:
            for ip in found - set(self.cams):
                self.cams[ip] = self._new_cam(ip)
            for ip in set(self.cams) - found:
                if not self.cams[ip].online:
                    del self.cams[ip]
        return found

    def _drop_all(self):
        with self.lock:
            cams, self.cams = list(self.cams.values()), {}
        for c in cams:
            if hasattr(c, "close"):
                c.close()

    def _connect_one(self, cam):
        was = cam.online
        try:
            if not cam.usb_ok:
                cam.enable_usb()
                cam.load_info()
            cam.keep_alive()
            cam.poll()
            if not was:
                model = cam.info.get("model_name", "GoPro")
                label = cam.serial if cam.serial.startswith("GoPro") else f"serial {cam.serial}"
                self.log("info", f"{self.name_of(cam)}: {model} at {cam.ip}"
                                 + (f" ({label})" if cam.serial else ""), "gopro")
        except Exception as e:
            cam.online, cam.usb_ok = False, False
            cam.error = _err(e)
            if was:
                self.log("warn", f"{self.name_of(cam)} at {cam.ip} lost: {cam.error}", "gopro")
            elif self.ble and not getattr(cam, "warned", False) and cam.error != "waiting to retry":
                cam.warned = True             # first failure only: the camera may be off or not paired
                self.log("warn", f"{cam.serial or cam.ip}: Bluetooth connect failed ({cam.error}). "
                                 "Turn the camera on; the first time, put it in pairing mode "
                                 "(Connections > Connect Device > Quik App)", "gopro")

    def _poller(self):
        last_disc = 0.0
        while not self._stop:
            if self._reset:
                self._reset = False
                self._drop_all()
                self.ble_names, last_disc = {}, 0.0
            if self.cfg.get("enabled") and not self.busy.is_set():
                now = time.time()
                if now - last_disc > (15.0 if self.ble else 5.0):
                    self.discover()
                    last_disc = now
                with self.lock:
                    cams = list(self.cams.values())
                list(self.pool.map(self._connect_one, cams))
            time.sleep(2.0)

    def online(self):
        with self.lock:
            return [c for c in self.cams.values() if c.online]

    # ---------------- recording ----------------
    def start_all(self, unix, session_dir):
        """Start every online camera at once. Returns the per-camera timing; raises if 'require' and any fail."""
        if not self.cfg.get("enabled"):
            return None
        self.busy.set()
        try:
            cams = self.online()
            if not cams:
                msg = "no GoPro connected over USB"
                if self.cfg.get("require"):
                    raise RuntimeError(msg)
                self.log("warn", msg + "; recording without video", "gopro")
                return None
            for c in cams:
                if c.encoding:
                    try:
                        c.shutter(False)        # stray recording from the remote: stop it first
                        time.sleep(1.0)
                    except Exception:
                        pass
            before = dict(zip([self.name_of(c) for c in cams],
                              self.pool.map(_safe_media, cams)))

            def go(c):
                r = {"ip": c.ip, "serial": c.serial, "model": c.info.get("model_name", "")}
                r["t_send_unix"] = unix()
                try:
                    c.shutter(True)
                    r["t_ack_unix"] = unix()
                    r["ok"] = True
                except Exception as e:
                    r["t_ack_unix"], r["ok"], r["error"] = unix(), False, _err(e)
                return r

            res = dict(zip([self.name_of(c) for c in cams], self.pool.map(go, cams)))
            # confirm each camera is really encoding (status 10), up to 3 s
            end = time.time() + 3.0
            pending = {n for n, r in res.items() if r["ok"]}
            byname = {self.name_of(c): c for c in cams}
            while pending and time.time() < end:
                for n in list(pending):
                    try:
                        if byname[n].poll() and byname[n].encoding:
                            res[n]["t_encoding_unix"] = unix()
                            pending.discard(n)
                    except Exception:
                        pass
                time.sleep(0.05)
            for n in pending:
                res[n]["ok"], res[n]["error"] = False, "camera did not start encoding"
            bad = [f"{n}: {r.get('error')}" for n, r in res.items() if not r["ok"]]
            if bad:
                msg = "GoPro start failed - " + "; ".join(bad)
                if self.cfg.get("require"):
                    self._stop_cams([byname[n] for n, r in res.items() if r["ok"]])
                    raise RuntimeError(msg)
                self.log("warn", msg, "gopro")
            okn = [n for n, r in res.items() if r["ok"]]
            if okn:
                spread = max(res[n]["t_ack_unix"] for n in okn) - min(res[n]["t_send_unix"] for n in okn)
                self.log("info", f"GoPro recording: {', '.join(okn)} (commands within {spread * 1e3:.0f} ms)", "gopro")
            self.rec = {"dir": session_dir, "cams": res, "before": before}
            self._write(session_dir, {"start": res})
            return res
        finally:
            self.busy.clear()

    def _stop_cams(self, cams):
        for c in cams:
            try:
                c.shutter(False)
            except Exception:
                pass

    def stop_all(self, unix):
        """Stop the cameras of the current take; downloads start in the background."""
        take, self.rec = self.rec, None
        if not take:
            return None
        self.busy.set()
        try:
            with self.lock:
                byname = {self.name_of(c): c for c in self.cams.values()}
            names = [n for n, r in take["cams"].items() if r.get("ok") and n in byname]

            def halt(n):
                c = byname[n]
                r = {"t_send_unix": unix()}
                try:
                    c.shutter(False)
                    r["t_ack_unix"], r["ok"] = unix(), True
                except Exception as e:
                    r["t_ack_unix"], r["ok"], r["error"] = unix(), False, _err(e)
                return r

            res = dict(zip(names, self.pool.map(halt, names)))
            for n, r in res.items():
                if not r["ok"]:
                    self.log("warn", f"{n}: stop failed: {r.get('error')}", "gopro")
            self.log("info", "GoPro stopped: " + ", ".join(n for n, r in res.items() if r["ok"]), "gopro")
            self._write(take["dir"], {"stop": res})
            self.last_take = {"dir": take["dir"], "cams": names}
        finally:
            self.busy.clear()
        if self.ble:
            self._write(take["dir"], {"files": {n: {"note": "Bluetooth: copy the video from the SD card"}
                                                for n in names}})
        elif self.cfg.get("download"):
            for n in names:
                self.pool.submit(self._fetch, n, byname[n], take)
        return res

    def _fetch(self, name, cam, take):
        dl = self.downloads[name] = {"file": "", "done": 0, "total": 0, "state": "waiting", "error": ""}
        try:
            end = time.time() + 20.0             # the camera closes the file after stop
            while time.time() < end:
                try:
                    cam.poll()
                    if not cam.encoding and not cam.busy:
                        break
                except Exception:
                    pass
                time.sleep(0.5)
            new = sorted(cam.media_files() - take["before"].get(name, set()))
            new = [f for f in new if f.upper().endswith(".MP4")]
            if not new:
                raise RuntimeError("no new video found on the SD card")
            vdir = os.path.join(take["dir"], "video")
            os.makedirs(vdir, exist_ok=True)
            files = []
            for rel in new:                       # long takes are split into several chapters
                fname = f"{name}_{rel.split('/')[-1]}"
                dl.update(file=fname, done=0, total=0, state="downloading")
                n = cam.download(rel, os.path.join(vdir, fname),
                                 lambda d, t: dl.update(done=d, total=t))
                files.append({"file": fname, "sd_path": rel, "bytes": n})
            dl["state"] = "done"
            self._write(take["dir"], {"files": {name: files}})
            mb = sum(f["bytes"] for f in files) / 1e6
            self.log("info", f"{name}: downloaded {len(files)} file(s), {mb:.0f} MB", "gopro")
        except Exception as e:
            dl.update(state="error", error=_err(e))
            self._write(take["dir"], {"files": {name: {"error": dl["error"]}}})
            self.log("warn", f"{name}: download failed: {dl['error']} - the video stays on the SD card", "gopro")

    def _write(self, session_dir, update):
        """Merge into <session>/video/video.json."""
        with self.lock:
            vdir = os.path.join(session_dir, "video")
            os.makedirs(vdir, exist_ok=True)
            p = os.path.join(vdir, "video.json")
            try:
                with open(p, encoding="utf-8") as f:
                    cur = json.load(f)
            except (OSError, ValueError):
                cur = {"note": "host times are PC unix seconds (same clock as the IMU/DAQ files); "
                               "the frame-accurate offset comes from the sync LED"}
            for k, v in update.items():
                if isinstance(v, dict) and isinstance(cur.get(k), dict):
                    cur[k].update(v)
                else:
                    cur[k] = v
            with open(p, "w", encoding="utf-8") as f:
                json.dump(cur, f, indent=2)

    # ---------------- settings / view ----------------
    def set_cfg(self, cfg):
        for k in ("enabled", "download", "require"):
            if k in cfg:
                self.cfg[k] = bool(cfg[k])
        if cfg.get("transport") in ("usb", "ble") and cfg["transport"] != self.cfg["transport"]:
            if cfg["transport"] == "ble":
                import gopro_ble
                if not gopro_ble.HAVE_BLEAK:
                    raise RuntimeError(gopro_ble.BLEAK_ERR)
            self.cfg["transport"] = cfg["transport"]
            self._reset = True
        if "names" in cfg and isinstance(cfg["names"], dict):
            self.cfg["names"] = {str(k): str(v).strip()[:20] for k, v in cfg["names"].items() if str(v).strip()}
        if "manual_ips" in cfg:
            self.cfg["manual_ips"] = [str(x).strip() for x in cfg["manual_ips"] if str(x).strip()]
        return self.cfg

    def snapshot(self):
        with self.lock:
            cams = sorted(self.cams.values(), key=lambda c: c.ip)
        out = []
        for c in cams:
            s = c.state
            n = self.name_of(c)
            out.append({"name": n, "ip": c.ip, "serial": c.serial, "model": c.info.get("model_name", ""),
                        "online": c.online, "error": c.error, "encoding": c.encoding,
                        "battery": s.get(ST_BATTERY_PCT), "sd_free_gb": round(s[ST_SD_FREE_KB] / 1e6, 1)
                        if isinstance(s.get(ST_SD_FREE_KB), (int, float)) else None,
                        "sd_status": s.get(ST_SD_STATUS), "video_s": s.get(ST_VIDEO_SECS),
                        "download": self.downloads.get(n)})
        return {"cfg": self.cfg, "cams": out, "recording": bool(self.rec)}

    def shutdown(self):
        self._stop = True
        self._drop_all()


def _safe_media(cam):
    try:
        return cam.media_files()
    except Exception:
        return set()


def _err(e):
    if isinstance(e, urllib.error.HTTPError):
        return f"HTTP {e.code}"
    if isinstance(e, urllib.error.URLError):
        return str(e.reason)
    return str(e) or type(e).__name__


if __name__ == "__main__":
    # quick probe: python gopro.py  -> lists cameras found over USB and their status
    rig = GoProRig(lambda lvl, t, src: print(f"{lvl:5s} {src}: {t}"))
    print("PC addresses on GoPro USB networks:", local_usb_ips() or "none")
    time.sleep(6)
    print(json.dumps(rig.snapshot(), indent=2))
