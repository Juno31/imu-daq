"""A stand-in for the SID Lab Teensy firmware (teensy_spine_imu.ino, handover 2026-11-25) on a fake serial port,
for testing the host without hardware. It follows the firmware's USB output and one-letter commands:

  start-up lines, "# columns:" header, F lines (frame, t_us, 8 x ok,fresh,w,x,y,z, 8 x age_us) at 120 Hz,
  "# Hz:" / "# DAQ clock:" status every 2 s, and the commands
  p (stop clock) g (start clock) q / v (frame lines off / on) h (header) s (status) r (restart) D (dump SD file).
  With the SD card: 0.5 s without frames closes the trial file; the next frame opens a new one at frame 0.

Every character the host writes is kept in `written`, so a test can check that no stray command letter
(e.g. the 'r' in "start") reached the board.
"""
import math
import threading
import time

PODS = ["C7", "T3", "T6", "T9", "T11", "L1", "L3", "S1"]
FRAME_US = 8333


class FakeSidlabSerial:
    def __init__(self, sd=True, missing_pod=None):
        self.is_open = True
        self.written = []
        self.sd = sd
        self.missing = missing_pod
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self.running = True                  # frame clock
        self.verbose = True
        self.frame = 0
        self.t0 = time.perf_counter()
        self.trial_open, self.trial_n, self.trial_lines, self.trial_frames = False, 0, [], 0
        self.last_closed, self.last_closed_lines = "", []
        self.last_frame_t = time.perf_counter()
        self._stop = threading.Event()
        self._emit("# SID Lab Teensy 4.1 spine IMU logger (8 pods, 2 muxes)")
        self._scan()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    # ---- pyserial surface used by device.Device ----
    def set_buffer_size(self, **k):
        pass

    def reset_input_buffer(self):
        with self._lock:
            self._buf.clear()

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._buf)

    def read(self, n=1):
        with self._cv:
            if not self._buf:
                self._cv.wait(0.2)
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out

    def write(self, data):
        for ch in data.decode():
            self.written.append(ch)
            self._command(ch)
        return len(data)

    def close(self):
        self.is_open = False
        self._stop.set()

    # ---- firmware behaviour ----
    def _scan(self):
        """startPods(): one line per pod, then the column header (as at power-up and after 'r')."""
        for i, p in enumerate(PODS):
            if p == self.missing:
                self._emit(f"# {p}: no BNO086 on mux {'AB'[i // 4]} ch{i % 4}")
            else:
                self._emit(f"# {p}: OK (mux {'AB'[i // 4]} ch{i % 4}, report every 7 ms)")
        self._emit("# columns: " + self.header())

    def header(self):
        cols = ["F", "frame", "t_us"]
        for p in PODS:
            cols += [f"{p}_ok", f"{p}_fresh", f"{p}_w", f"{p}_x", f"{p}_y", f"{p}_z"]
        cols += [f"{p}_age_us" for p in PODS]
        return ",".join(cols)

    def _emit(self, line):
        with self._cv:
            self._buf += (line + "\n").encode()
            self._cv.notify_all()

    def _command(self, c):
        if c == "p":
            self.running = False
            self._emit("# frame clock stopped")
        elif c == "g":
            self.running = True
            self._emit("# frame clock running")
        elif c == "q":
            self.verbose = False
        elif c == "v":
            self.verbose = True
        elif c == "h":
            self._emit("# columns: " + self.header())
        elif c == "s":
            for p in PODS:
                ok = int(p != self.missing)
                self._emit(f"# {p} ok={ok} updates={self.frame * ok} empty_reads=0 fresh_frames={100.0 * ok:.1f}%")
            self._emit(f"# frames={self.frame} late_resyncs=0 I2C_HZ=400000 REPORT_MS=7 (frame {FRAME_US} us)")
        elif c == "r":
            self._scan()
        elif c == "D":
            if self.trial_open or not self.last_closed:
                self._emit("# SD: nothing to dump (recording, or no file closed yet)")
                return
            body = [self.header()] + self.last_closed_lines
            self._emit(f"# DUMP BEGIN {self.last_closed} {sum(len(x) + 1 for x in body)} bytes")
            for x in body:
                self._emit(x)
            self._emit("# DUMP END")

    def _line(self, fr, t_us):
        parts = ["F", str(fr), str(t_us & 0xFFFFFFFF)]
        ang = 0.3 * math.sin(t_us / 1e6)
        for i, p in enumerate(PODS):
            ok = p != self.missing
            w, z = math.cos(ang / 2 * (i + 1) / 8), math.sin(ang / 2 * (i + 1) / 8)
            parts += [str(int(ok)), str(int(ok))] + ([f"{w:.5f}", "0.00000", "0.00000", f"{z:.5f}"] if ok else
                                                     ["1.00000", "0.00000", "0.00000", "0.00000"])
        parts += [str(3600 if p != self.missing else 0) for p in PODS]
        return ",".join(parts)

    def _run(self):
        next_t = time.perf_counter()
        stat = time.perf_counter()
        while not self._stop.is_set():
            now = time.perf_counter()
            if self.running and now >= next_t:
                next_t += FRAME_US / 1e6
                if self.sd and not self.trial_open:            # first edge after a gap: new SD file at frame 0
                    self.trial_n += 1
                    self.trial_open, self.trial_lines, self.frame = True, [], 0
                    self._emit(f"# SD: recording TRIAL{self.trial_n:03d}.CSV")
                t_us = int((now - self.t0) * 1e6)
                line = self._line(self.frame, t_us)
                if self.verbose:
                    self._emit(line)
                if self.trial_open:
                    self.trial_lines.append(line)
                self.frame += 1
                self.last_frame_t = now
            elif not self.running:
                next_t = now
                if self.trial_open and now - self.last_frame_t > 0.5:
                    self.trial_open = False
                    self.last_closed = f"TRIAL{self.trial_n:03d}.CSV"
                    self.last_closed_lines = self.trial_lines
                    self._emit(f"# SD: {self.last_closed} closed: {len(self.trial_lines)} frames, 0 missed edges, "
                               f"0 buffer overruns, max buffer 1024 bytes")
            if now - stat >= 2.0:
                stat = now
                hz = " ".join(f"{p} {0.0 if p == self.missing else 138.2:.1f}" for p in PODS)
                self._emit(f"# Hz: {hz} | I2C err A 0 B 0 | read A 1472 us B 1580 us | bus busy A 80% B 92% | "
                           f"max loop 900 us | late 0")
                self._emit("# DAQ clock: edges 1234 | missed edges 0 | max frame lag after edge 300 us | "
                           "data packets dropped 0" if self.running else
                           "# DAQ clock: NO EDGES on pin 4 in the last 2 s (DAQ stopped, or check the counter output and wiring)")
            time.sleep(0.0005)
