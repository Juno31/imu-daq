"""Live preview of the GoPros in the GUI (USB only).

Each camera sends its preview (H.265 in MPEG-TS, 1080p30, ~2-3 Mbit/s) by UDP to a port on the PC.
Python receives it (so Windows Firewall only has to allow python.exe) and pipes it into ffmpeg, which
makes small JPEG frames. The browser shows them as an MJPEG stream (<img src="/api/gopro/live/cam1.mjpg">).

The preview keeps running while the camera records (tested on a HERO13 Black), so the view also
works during a trial. It is for framing only: ~0.3-0.5 s late and not time-stamped.
"""
import shutil
import socket
import subprocess
import threading
import time
import urllib.request

BASE_PORT = 8554
FPS = 10
WIDTH = 480


def ffmpeg_exe():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return shutil.which("ffmpeg")


class LiveView:
    """One camera: UDP preview in, latest JPEG frame out."""

    def __init__(self, cam, port, log):
        self.cam, self.port, self.log = cam, port, log
        self.frame, self.seq, self.t_frame = b"", 0, 0.0
        self.error, self.fps = "", 0.0
        self._stop = False
        self.proc = None
        self.sock = None
        self.t_start = 0.0
        threading.Thread(target=self._run, name=f"live-{port}", daemon=True).start()

    # ---- camera side ----
    def _cam_get(self, path):
        with urllib.request.urlopen(f"{self.cam.base}{path}", timeout=3.0) as r:
            r.read()

    def _start_stream(self):
        try:
            self._cam_get("/gopro/camera/stream/stop")
        except Exception:
            pass
        self._cam_get(f"/gopro/camera/stream/start?port={self.port}")
        self.t_start = time.time()

    # ---- threads ----
    def _run(self):
        exe = ffmpeg_exe()
        if not exe:
            self.error = "ffmpeg not found (pip install imageio-ffmpeg)"
            return
        while not self._stop:
            try:
                self._session(exe)
            except Exception as e:
                self.error = str(e) or e.__class__.__name__
            self._close()
            if not self._stop:
                time.sleep(2.0)

    def _session(self, exe):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.proc = subprocess.Popen(
            [exe, "-hide_banner", "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay",
             "-probesize", "500000", "-analyzeduration", "500000", "-f", "mpegts", "-i", "pipe:0",
             "-an", "-vf", f"fps={FPS},scale={WIDTH}:-2", "-c:v", "mjpeg", "-q:v", "6",
             "-f", "image2pipe", "pipe:1"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, creationflags=flags)
        threading.Thread(target=self._read_jpegs, args=(self.proc,), name=f"live-jpg-{self.port}",
                         daemon=True).start()
        self._start_stream()
        last_pkt = time.time()
        while not self._stop:
            try:
                data, _a = self.sock.recvfrom(65536)
            except socket.timeout:
                data = b""
            now = time.time()
            if data:
                last_pkt = now
                self.proc.stdin.write(data)
            elif now - last_pkt > 4.0:          # camera stopped sending: ask again
                raise RuntimeError("no preview from the camera")
            if self.t_frame and now - self.t_frame > 6.0 and now - self.t_start > 6.0:
                raise RuntimeError("preview stalled")
            if self.proc.poll() is not None:
                raise RuntimeError("ffmpeg stopped")

    def _read_jpegs(self, proc):
        buf, n, t0 = bytearray(), 0, time.time()
        out = proc.stdout
        while True:
            chunk = out.read1(65536) if hasattr(out, "read1") else out.read(4096)
            if not chunk:
                return
            buf += chunk
            while True:
                s = buf.find(b"\xff\xd8")
                e = buf.find(b"\xff\xd9", s + 2) if s >= 0 else -1
                if s < 0 or e < 0:
                    if s > 0:
                        del buf[:s]
                    break
                self.frame = bytes(buf[s:e + 2])
                self.seq += 1
                self.t_frame = time.time()
                self.error = ""
                del buf[:e + 2]
                n += 1
                if self.t_frame - t0 >= 2.0:
                    self.fps, n, t0 = round(n / (self.t_frame - t0), 1), 0, self.t_frame

    def _close(self):
        p, self.proc = self.proc, None
        if p is not None:
            try:
                p.stdin.close()
            except Exception:
                pass
            try:
                p.kill()
            except Exception:
                pass
        s, self.sock = self.sock, None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass
        self.fps = 0.0

    def stop(self):
        self._stop = True
        try:
            self._cam_get("/gopro/camera/stream/stop")
        except Exception:
            pass
        self._close()

    def status(self):
        live = bool(self.frame) and time.time() - self.t_frame < 3.0
        return {"on": live, "fps": self.fps if live else 0.0, "error": "" if live else self.error,
                "port": self.port}


class LiveManager:
    """Keeps one LiveView per online USB camera while live view is on."""

    def __init__(self, log):
        self.log = log
        self.views = {}                     # camera ip -> LiveView
        self.names = {}                     # camera name -> ip
        self.lock = threading.Lock()

    def sync(self, want, cams, name_of):
        """cams: online USB cameras (gopro.GoPro). Starts/stops viewers to match."""
        with self.lock:
            target = {c.ip: c for c in cams} if want else {}
            for ip in list(self.views):
                if ip not in target:
                    self.views.pop(ip).stop()
            used = {v.port for v in self.views.values()}
            for ip, c in sorted(target.items()):
                if ip not in self.views:
                    port = next(p for p in range(BASE_PORT, BASE_PORT + 50) if p not in used)
                    used.add(port)
                    self.views[ip] = LiveView(c, port, self.log)
            self.names = {name_of(c): ip for ip, c in target.items()}

    def view(self, name):
        ip = self.names.get(name)
        return self.views.get(ip) if ip else None

    def status(self):
        return {n: self.views[ip].status() for n, ip in self.names.items() if ip in self.views}

    def stop_all(self):
        with self.lock:
            for v in self.views.values():
                v.stop()
            self.views, self.names = {}, {}
