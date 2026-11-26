"""Serial link to the board firmware. Two protocols are understood:

  "imu_daq"  firmware/imu_daq_teensy and firmware/imu_daq (XIAO): @HELLO / @START / @STOP, text commands.
  "sidlab"   the SID Lab spine-brace firmware (teensy_spine_imu.ino, handover of 2026-11-25):
             frame lines run all the time ("F,frame,t_us, 8 x ok,fresh,w,x,y,z, 8 x age_us"), '#' status lines,
             and one-letter commands (p = stop the frame clock, g = start it, q / v = frame lines off / on,
             h = column header, s = status, r = restart pods, D = send the last SD file).
             The firmware reads commands one character at a time, so a word like "start" would run
             s, t, a, r(estart pods), t. send() therefore only passes the commands listed in SIDLAB_CMDS.

Hands parsed items to callbacks. Runs one reader thread; reads in large chunks so the PC never makes the
board wait.
"""
import re
import threading
import time

import serial
from serial.tools import list_ports

ESPRESSIF_VID = 0x303A
PJRC_VID = 0x16C0          # Teensy
BOARD_VIDS = {PJRC_VID: "Teensy", ESPRESSIF_VID: "XIAO"}
WRAP = 1 << 32

# host command -> SID Lab firmware characters (commands not listed are dropped for that firmware)
SIDLAB_CMDS = {"hello": "h", "header": "h", "clock_stop": "p", "clock_go": "g", "quiet": "q", "verbose": "v",
               "sdump": "D", "restart": "r", "status_full": "s"}


def list_serial_ports():
    out = []
    for p in list_ports.comports():
        out.append({"device": p.device, "description": p.description,
                    "board": BOARD_VIDS.get(p.vid), "espressif": p.vid == ESPRESSIF_VID})
    return out


def guess_port():
    ports = list(list_ports.comports())
    for vid in BOARD_VIDS:                     # a Teensy first, then a XIAO
        found = [p.device for p in ports if p.vid == vid]
        if found:
            return found[0]
    return ports[0].device if len(ports) == 1 else None


def parse_kv(text):
    out = {}
    for tok in text.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


class Unwrapper:
    """Turns the board's 32-bit micros() (wraps every 71.6 min) into a monotonic 64-bit count."""

    def __init__(self):
        self.offset = 0
        self.last = None

    def reset(self):
        self.offset = 0
        self.last = None

    def __call__(self, t):
        full = t + self.offset
        if self.last is not None:
            if full < self.last - (1 << 31):        # wrapped forward
                self.offset += WRAP
                full += WRAP
            elif full > self.last + (1 << 31):     # a slightly older value from before the wrap
                full -= WRAP
        if self.last is None or full > self.last:
            self.last = full
        return full


class Device:
    """Callbacks (all called from the reader thread):
        on_frame(kind, num, t_us, vals, host_t)   kind 'P' or 'F'; vals = floats, 6 per pod
        on_message(tag, text, fields, host_t)     tag '#', 'HELLO', 'START', 'STOP', 'EVENT', ...
        on_pong(token, t_us, host_perf)
        on_disconnect(reason)
    """

    def __init__(self, on_frame, on_message, on_pong, on_disconnect):
        self.on_frame = on_frame
        self.on_message = on_message
        self.on_pong = on_pong
        self.on_disconnect = on_disconnect
        self.on_sample = None   # optional: on_sample(pod_index, report_id, seq, t_us, status, values)
        # SD file download ("sdget"): firmware >= 2.5 sends the file as "D,<line>" between @DUMP BEGIN / END
        self.dump_fh = None
        self.dump_lines = 0
        self.dump_done = threading.Event()
        self.dump_error = None
        self.ser = None
        self.port = None
        self.thread = None
        self.stop_evt = threading.Event()
        self.wlock = threading.Lock()
        self.unwrap = Unwrapper()
        self.hello = {}
        self.pod_names = []
        self.bad_lines = 0
        self.proto = None          # None until the board identifies itself, then "imu_daq" or "sidlab"
        self.recording = False     # sidlab: frame lines count as recorded frames ("F") only while this is set
        self.ages = []             # sidlab: age_us per pod of the last frame
        self.pod_where = {}        # sidlab: pod -> "A0" (mux bus + channel), from the start-up lines
        self.sd = {}               # sidlab: last "# SD:" facts (recording / closed file, frames)
        self.hz = {}               # sidlab: pod -> report rate from the "# Hz:" line
        self.status_line = ""      # sidlab: last "# Hz:" line
        self._dt = []              # sidlab: frame intervals for the frame period estimate
        self._last_t = None
        self.dumping = False
        self.pod_diag = {}         # sidlab: pod -> {"state", "text", "t"} from the firmware's per-pod lines
        self.bus = {}              # sidlab: "A"/"B" -> I2C errors, read time, busy %, pause notice (Hz line)
        self.scan_t = None         # sidlab: time.time() of the last start-up / 'r' pod scan seen
        self.sid_banner = False    # sidlab: start-up banner seen before the board was identified

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port):
        self.disconnect()
        ser = serial.Serial(port, 921600, timeout=0.2, write_timeout=1)
        try:
            ser.set_buffer_size(rx_size=1 << 20, tx_size=1 << 14)   # Windows driver buffers
        except (AttributeError, serial.SerialException):
            pass
        ser.reset_input_buffer()
        self.ser, self.port = ser, port
        self.unwrap.reset()
        self.hello, self.pod_names, self.bad_lines = {}, [], 0
        self.proto, self.recording, self.ages, self.pod_where, self.sd, self.hz = None, False, [], {}, {}, {}
        self._dt, self._last_t, self.dumping, self.status_line = [], None, False, ""
        self.pod_diag, self.bus, self.scan_t, self.sid_banner = {}, {}, None, False
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._reader, name="serial-reader", daemon=True)
        self.thread.start()
        self.send("hello")

    def disconnect(self):
        self.stop_evt.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=1.5)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send(self, cmd):
        """Send a host command. For the SID Lab firmware it is translated (SIDLAB_CMDS) or dropped.
        Before the board has identified itself only "hello" goes out: "ping 1" would stop the SID Lab clock.
        Returns True when something was sent."""
        if not self.connected:
            raise RuntimeError("not connected")
        if self.proto != "imu_daq":
            word = cmd.split()[0] if cmd.split() else ""
            if self.proto is None and word != "hello":
                return False
            if self.proto == "sidlab":
                c = SIDLAB_CMDS.get(word)
                if c is None:
                    return False
                return self.send_raw(c)
        with self.wlock:
            self.ser.write((cmd + "\n").encode())
        return True

    def send_raw(self, chars):
        """Write characters as they are (SID Lab one-letter commands)."""
        if not self.connected:
            raise RuntimeError("not connected")
        with self.wlock:
            self.ser.write(chars.encode())
        return True

    # ---------------- reader ----------------
    def _reader(self):
        buf = b""
        reason = "closed"
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                reason = f"serial error: {e}"
                break
            if not chunk:
                continue
            perf = time.perf_counter()
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").strip()
                if line:
                    try:
                        self._handle(line, perf)
                    except Exception:
                        self.bad_lines += 1
        if not self.stop_evt.is_set():
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.on_disconnect(reason)

    def fetch_file(self, name, path, idle_s=10.0, progress=None):
        """Copy a file from the Teensy SD card to `path` ("sdget"). Returns the number of lines.
        Gives up if no line arrives for idle_s seconds."""
        self.dump_done.clear()
        self.dump_error = None
        self.dump_lines = 0
        fh = open(path, "w", newline="")
        self.dump_fh = fh
        try:
            self.send("sdump" if self.proto == "sidlab" else f"sdget {name}")
            last, last_n, last_report = time.time(), 0, time.time()
            while not self.dump_done.wait(0.5):
                if self.dump_error:
                    break
                if self.dump_lines != last_n:
                    last, last_n = time.time(), self.dump_lines
                elif time.time() - last > idle_s:
                    self.dump_error = f"no data for {idle_s:.0f} s"
                    break
                if progress and time.time() - last_report > 3:
                    progress(self.dump_lines)
                    last_report = time.time()
        finally:
            self.dump_fh = None
            fh.close()
        if self.dump_error:
            raise RuntimeError(f"SD download of {name} failed: {self.dump_error}")
        return self.dump_lines

    def _handle(self, line, perf):
        if self.proto != "imu_daq" and self._handle_sidlab(line, perf):
            return
        c = line[0]
        if c == "D" and len(line) > 1 and line[1] == ",":
            fh = self.dump_fh
            if fh is not None:
                fh.write(line[2:] + "\n")
                self.dump_lines += 1
            return
        if (c == "P" or c == "F") and len(line) > 2 and line[1] == ",":
            parts = line.split(",")
            nv, npod = len(parts) - 3, len(self.pod_names)
            # v1.1: 6 values per IMU; v1.2: 6 per IMU, then 13 per IMU (acc, gyro, mag)
            if (npod and nv not in (6 * npod, 19 * npod)) or (not npod and nv % 6 and nv % 19):
                self.bad_lines += 1
                return
            num = int(parts[1])
            t_us = self.unwrap(int(parts[2]))
            vals = [float(v) for v in parts[3:]]
            self.on_frame(c, num, t_us, vals, perf)
        elif c == "S" and len(line) > 2 and line[1] == ",":
            # firmware >= 2.2: one line per sensor sample while recording
            # S,<pod index>,<report id>,<IMU seq>,<t_us>,<status>,<3 or 4 values>
            parts = line.split(",")
            if len(parts) not in (9, 10) or self.on_sample is None:
                if len(parts) not in (9, 10):
                    self.bad_lines += 1
                return
            self.on_sample(int(parts[1]), int(parts[2]), int(parts[3]), self.unwrap(int(parts[4])),
                           int(parts[5]), [float(v) for v in parts[6:]])
        elif c == "@":
            if self.proto is None:
                self.proto = "imu_daq"
            tag, _, rest = line[1:].partition(" ")
            if tag == "PONG":
                tok, _, t = rest.partition(" ")
                self.on_pong(tok, self.unwrap(int(t)), perf)
                return
            fields = parse_kv(rest)
            if tag == "DUMP" and self.dump_fh is not None:
                if rest.startswith("END"):
                    self.dump_done.set()
                return
            if tag == "ERR" and self.dump_fh is not None and "sd" in rest:
                self.dump_error = rest
                self.dump_done.set()
            if tag == "HELLO":
                self.hello = fields
                self.pod_names = [p.split(":")[0] for p in fields.get("pods", "").split(",") if p]
            for key in ("t_us",):
                if key in fields:
                    try:
                        fields[key] = str(self.unwrap(int(fields[key])))
                    except ValueError:
                        pass
            self.on_message(tag, rest, fields, perf)
        elif c == "#":
            if line.startswith("# columns:") and not self.pod_names:
                cols = line[len("# columns: "):].split(",")
                self.pod_names = [c[:-3] for c in cols[3:] if c.endswith("_ok")]
            self.on_message("#", line[1:].strip(), {}, perf)
        else:
            self.bad_lines += 1

    # ---------------- SID Lab firmware (teensy_spine_imu) ----------------
    SD_REC = re.compile(r"# SD: recording (\S+)")
    SD_CLOSED = re.compile(r"# SD: (\S+) closed: (\d+) frames, (\d+) missed edges, (\d+) buffer overruns")
    POD_OK = re.compile(r"# (\w+): OK \(mux (\w) ch(\d)")
    # per-pod lines of the start-up / 'r' scan and of the running loop -> link state for the Connections view
    POD_DIAG = [
        (re.compile(r"# (\w+): OK \(mux (\w) ch(\d)"), "ok", True),
        (re.compile(r"# (\w+): mux (\w) did not answer"), "no_mux", True),
        (re.compile(r"# (\w+): no BNO086 on mux (\w) ch(\d)"), "no_imu", True),
        (re.compile(r"# (\w+): answered but sent no quaternion"), "no_quat", True),
        (re.compile(r"# (\w+): no answer \((\d+) errors"), "lost", False),
        (re.compile(r"# (\w+): answering again"), "ok", False),
    ]
    POD_STAT = re.compile(r"# (\w+) ok=(\d) updates=(\d+) empty_reads=(\d+) fresh_frames=([\d.]+)%")
    BUS_PAUSE = re.compile(r"# bus (\w): (\d+) errors in a row")
    HZ_BUS = re.compile(r"I2C err A (\d+) B (\d+) \| read A (\d+) us B (\d+) us \| bus busy A (\d+)% B (\d+)%")

    def _sidlab_diag(self, line):
        """Keep the firmware's per-pod and per-bus connection facts (see POD_DIAG)."""
        now = time.time()
        for rx, state, scan in self.POD_DIAG:
            m = rx.match(line)
            if m:
                name = m.group(1)
                if name not in self.pod_names and self.pod_names:
                    return
                if state == "no_mux":
                    bus = m.group(2)
                    self.bus.setdefault(bus, {})["mux"] = "no answer at 0x70"
                    self.bus[bus]["mux_t"] = now
                elif scan:
                    bus = line.split("mux ")[1][0] if "mux " in line else None
                    if bus and state in ("ok", "no_imu", "no_quat"):
                        self.bus.setdefault(bus, {})["mux"] = "ok"
                        self.bus[bus]["mux_t"] = now
                if scan:
                    self.scan_t = now
                self.pod_diag[name] = {"state": state, "text": line[2:].strip(), "t": now}
                return
        m = self.POD_STAT.match(line)
        if m:
            d = self.pod_diag.get(m.group(1))
            if m.group(2) == "0" and (d is None or d["state"] == "ok"):
                self.pod_diag[m.group(1)] = {"state": "down", "text": line[2:].strip(), "t": now}
            return
        m = self.BUS_PAUSE.match(line)
        if m:
            b = self.bus.setdefault(m.group(1), {})
            b["paused"] = line[2:].strip()
            b["paused_t"] = now
            return
        m = self.HZ_BUS.search(line)
        if m:
            v = [int(x) for x in m.groups()]
            for i, bus in enumerate("AB"):
                b = self.bus.setdefault(bus, {})
                prev = b.get("errors")
                b["err_new"] = 0 if prev is None else max(0, v[i] - prev)
                b["errors"], b["read_us"], b["busy_pct"], b["t"] = v[i], v[2 + i], v[4 + i], now

    def _sidlab_hello(self, perf):
        """The board runs the SID Lab firmware: report it like an @HELLO so the server goes idle."""
        self.proto = "sidlab"
        frame_us = int(self.hello.get("frame_us", 8333))
        f = {"fw": "teensy_spine_imu", "board": "Teensy41", "link": "usb", "proto": "sidlab",
             "ver": "(SID Lab)",
             # mux bus + channel: from the start-up lines if seen, else the firmware's fixed order (A ch0-3, B ch0-3)
             "pods": ",".join(f"{p}:{self.pod_where.get(p) or ('AB'[i // 4] + str(i % 4) if i < 8 else '')}"
                              for i, p in enumerate(self.pod_names)),
             "frame_us": str(frame_us), "report_ms": str(max(1, round(frame_us / 1000))), "rec": "0",
             "quat": "game", "data_line": "P0.0"}
        self.hello = f
        self.on_message("HELLO", " ".join(f"{k}={v}" for k, v in f.items()), dict(f), perf)

    def _handle_sidlab(self, line, perf):
        """True when the line was handled as SID Lab firmware output."""
        if self.dumping:                                     # 'D': raw file lines between DUMP BEGIN / END
            if line.startswith("# DUMP END"):
                self.dumping = False
                self.dump_done.set()
            elif self.dump_fh is not None:
                self.dump_fh.write(line + "\n")
                self.dump_lines += 1
            return True
        if line.startswith("#"):
            if line.startswith("# columns:"):
                cols = line[len("# columns: "):].split(",")
                if not any(c.endswith("_age_us") for c in cols):
                    return False                             # an imu_daq header: not ours
                self.pod_names = [c[:-3] for c in cols[3:] if c.endswith("_ok")]
                if self.proto is None:
                    self._sidlab_hello(perf)
                return True
            if self.proto is None:
                if line.startswith("# SID Lab"):
                    self.send_raw("h")                       # ask for the column header (pod names)
                    self.sid_banner = True                   # the pod scan lines follow before the header
                elif not self.sid_banner:
                    return False
            if line.startswith("# DUMP BEGIN"):
                if self.dump_fh is not None:
                    self.dumping = True
                return True
            if self.dump_fh is not None and line.startswith("# SD:") and (
                    "nothing to dump" in line or "cannot open" in line):
                self.dump_error = line[2:]
                self.dump_done.set()
            m = self.POD_OK.match(line)
            if m:
                self.pod_where[m.group(1)] = f"{m.group(2)}{m.group(3)}"
            self._sidlab_diag(line)
            m = self.SD_REC.match(line)
            if m:
                self.sd = {"recording": m.group(1)}
            m = self.SD_CLOSED.match(line)
            if m:
                self.sd = {"closed": m.group(1), "frames": int(m.group(2)), "missed_edges": int(m.group(3)),
                           "overruns": int(m.group(4))}
            if line.startswith("# Hz:"):
                self.status_line = line[2:]
                body = line[5:].split("|")[0].split()
                try:
                    self.hz = {body[i]: float(body[i + 1]) for i in range(0, len(body) - 1, 2)}
                except ValueError:
                    pass
            tag = "SIDSTATUS" if line.startswith(("# Hz:", "# DAQ clock:")) else "#"
            self.on_message(tag, line[1:].strip(), {}, perf)
            return True
        if line.startswith("F,"):
            parts = line.split(",")
            npod = len(self.pod_names)
            if not npod:
                if self.proto is None and len(parts) > 10 and (len(parts) - 3) % 7 == 0:
                    self.send_raw("h")                       # SID Lab frame without a header yet: ask for it
                    return True
                return False
            if len(parts) != 3 + 7 * npod:
                if self.proto is None:
                    return False
                self.bad_lines += 1
                return True
            if self.proto is None:
                self._sidlab_hello(perf)
            num = int(parts[1])
            t_us = self.unwrap(int(parts[2]))
            vals = [float(v) for v in parts[3:3 + 6 * npod]]
            self.ages = [int(v) for v in parts[3 + 6 * npod:]]
            if self._last_t is not None and len(self._dt) < 200 and 0 < t_us - self._last_t < 100000:
                self._dt.append(t_us - self._last_t)
                if len(self._dt) in (20, 200):
                    d = sorted(self._dt)
                    self.hello["frame_us"] = str(int(round(d[len(d) // 2])))
            self._last_t = t_us
            self.on_frame("F" if self.recording else "P", num, t_us, vals + self.ages, perf)
            return True
        return False
