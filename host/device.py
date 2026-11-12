"""Serial link to the imu_daq firmware (XIAO ESP32S3 / ESP32C3).

Parses the firmware's line protocol (see firmware/imu_daq/imu_daq.ino) and hands parsed items to
callbacks. Runs one reader thread; reads in large chunks so the PC never makes the board wait.
"""
import threading
import time

import serial
from serial.tools import list_ports

ESPRESSIF_VID = 0x303A
WRAP = 1 << 32


def list_serial_ports():
    out = []
    for p in list_ports.comports():
        out.append({"device": p.device, "description": p.description,
                    "espressif": p.vid == ESPRESSIF_VID})
    return out


def guess_port():
    ports = list(list_ports.comports())
    esp = [p.device for p in ports if p.vid == ESPRESSIF_VID]
    if len(esp) >= 1:
        return esp[0]
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
        self.ser = None
        self.port = None
        self.thread = None
        self.stop_evt = threading.Event()
        self.wlock = threading.Lock()
        self.unwrap = Unwrapper()
        self.hello = {}
        self.pod_names = []
        self.bad_lines = 0

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
        if not self.connected:
            raise RuntimeError("not connected")
        with self.wlock:
            self.ser.write((cmd + "\n").encode())

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

    def _handle(self, line, perf):
        c = line[0]
        if (c == "P" or c == "F") and len(line) > 2 and line[1] == ",":
            parts = line.split(",")
            if (len(parts) - 3) % 6:
                self.bad_lines += 1
                return
            num = int(parts[1])
            t_us = self.unwrap(int(parts[2]))
            vals = [float(v) for v in parts[3:]]
            self.on_frame(c, num, t_us, vals, perf)
        elif c == "@":
            tag, _, rest = line[1:].partition(" ")
            if tag == "PONG":
                tok, _, t = rest.partition(" ")
                self.on_pong(tok, self.unwrap(int(t)), perf)
                return
            fields = parse_kv(rest)
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
                self.pod_names = [cols[3 + 6 * k].rsplit("_", 1)[0] for k in range((len(cols) - 3) // 6)]
            self.on_message("#", line[1:].strip(), {}, perf)
        else:
            self.bad_lines += 1
