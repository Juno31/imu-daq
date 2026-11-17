"""GoPro control over Bluetooth LE (Open GoPro BLE API), for when the cameras are not on USB.

Bluetooth gives start/stop and status (battery, SD card, encoding) only: no preview and no
download, so the videos are copied from the SD cards afterwards.

First time only, per camera: on the camera, Connections > Connect Device > Quik App (pairing mode),
then let the GUI find it. Windows keeps the pairing, so later the camera only has to be on.

BleGoPro has the same interface as gopro.GoPro (shutter, poll, keep_alive, encoding, busy, serial,
info, online, error, state), so GoProRig drives either one.
"""
import asyncio
import threading
import time

try:
    from bleak import BleakClient, BleakScanner
    HAVE_BLEAK, BLEAK_ERR = True, ""
except Exception as e:                     # bleak missing: the USB transport still works
    HAVE_BLEAK, BLEAK_ERR = False, f"Bluetooth needs the bleak package (pip install bleak): {e}"

GOPRO_SERVICE = "0000fea6-0000-1000-8000-00805f9b34fb"


def _gp(n):
    return f"b5f9{n:04x}-aa8d-11e3-9046-0002a5d5c51b"


CMD, CMD_RSP = _gp(0x0072), _gp(0x0073)
SET, SET_RSP = _gp(0x0074), _gp(0x0075)
QRY, QRY_RSP = _gp(0x0076), _gp(0x0077)

SHUTTER_ON = bytes([0x03, 0x01, 0x01, 0x01])
SHUTTER_OFF = bytes([0x03, 0x01, 0x01, 0x00])
KEEP_ALIVE = bytes([0x03, 0x5B, 0x01, 0x42])        # setting 91 = 66
STATUS_IDS = (8, 10, 13, 33, 54, 70)                 # busy, encoding, video s, SD status, SD free kB, battery %


def status_query():
    return bytes([1 + len(STATUS_IDS), 0x13, *STATUS_IDS])


class Packets:
    """Reassembles Open GoPro BLE responses that arrive in several 20-byte packets."""

    def __init__(self):
        self.buf, self.need = bytearray(), 0

    def feed(self, data):
        """Returns the full message (bytes) once complete, else None."""
        data = bytes(data)
        if not data:
            return None
        h = data[0]
        if h & 0x80:                                 # continuation packet
            self.buf += data[1:]
        else:
            kind = (h >> 5) & 0x03
            if kind == 0:                            # 5-bit length
                self.need, body = h & 0x1F, data[1:]
            elif kind == 1:                          # 13-bit length
                self.need, body = ((h & 0x1F) << 8) | data[1], data[2:]
            else:                                    # 16-bit length
                self.need, body = (data[1] << 8) | data[2], data[3:]
            self.buf = bytearray(body)
        if self.need and len(self.buf) >= self.need:
            msg, self.buf, self.need = bytes(self.buf[:self.need]), bytearray(), 0
            return msg
        return None


def parse_status(msg):
    """[0x13, result, (id, len, value...)...] -> {"10": 1, ...} (keys as strings, like the HTTP API)."""
    if len(msg) < 2 or msg[1] != 0:
        raise RuntimeError(f"status query failed (result {msg[1] if len(msg) > 1 else '?'})")
    out, i = {}, 2
    while i + 2 <= len(msg):
        sid, ln = msg[i], msg[i + 1]
        val = msg[i + 2:i + 2 + ln]
        out[str(sid)] = int.from_bytes(val, "big") if ln else None
        i += 2 + ln
    return out


class BleLoop:
    """One asyncio loop thread for all Bluetooth work (bleak is async, the server is threaded)."""

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, name="gopro-ble", daemon=True).start()

    def run(self, coro, timeout):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)


_LOOP = None


def ble_loop():
    global _LOOP
    if _LOOP is None:
        _LOOP = BleLoop()
    return _LOOP


def scan(timeout=4.0):
    """GoPros advertising nearby: [(address, name)]. Connected cameras stop advertising."""
    if not HAVE_BLEAK:
        raise RuntimeError(BLEAK_ERR)

    async def go():
        found = await BleakScanner.discover(timeout=timeout, return_adv=True, service_uuids=[GOPRO_SERVICE])
        return [(d.address, d.name or a.local_name or d.address) for d, a in found.values()]
    return ble_loop().run(go(), timeout + 5)


class BleGoPro:
    def __init__(self, address, name=""):
        self.ip = address                    # the rig keys cameras by this (shown as the address)
        self.ble_name = name                 # "GoPro 1234" (last 4 digits of the serial)
        self.info = {"model_name": "GoPro (Bluetooth)"}
        self.state, self.online, self.error, self.last_seen = {}, False, "", 0.0
        self.usb_ok = True                   # nothing to enable over Bluetooth
        self.client = None
        self.lock = None                     # asyncio.Lock, made in the loop thread
        self.rx = {}                         # response characteristic -> Packets
        self.waiting = {}                    # response characteristic -> Future
        self.last_try = 0.0

    @property
    def serial(self):
        return self.ble_name                 # used to keep names stable across sessions

    @property
    def encoding(self):
        return bool(self.state.get("10"))

    @property
    def busy(self):
        return bool(self.state.get("8"))

    # ---- async side (runs in the BLE loop) ----
    def _on_rx(self, uuid):
        pk = self.rx.setdefault(uuid, Packets())

        def cb(_char, data):
            msg = pk.feed(data)
            fut = self.waiting.get(uuid)
            if msg is not None and fut is not None and not fut.done():
                fut.set_result(msg)
        return cb

    def _lost(self, _client):
        self.online, self.error = False, "Bluetooth disconnected"

    async def _connect(self):
        if self.lock is None:
            self.lock = asyncio.Lock()
        if self.client is not None and self.client.is_connected:
            return
        old, self.client = self.client, None
        if old is not None:                     # a dropped link: release it before reconnecting
            try:
                await old.disconnect()
            except Exception:
                pass
        # fresh service discovery: a link left over from a killed server can have a stale GATT cache
        c = BleakClient(self.ip, disconnected_callback=self._lost, timeout=15, pair=True,
                        winrt={"use_cached_services": False})
        try:
            await c.connect()
        except (asyncio.TimeoutError, TimeoutError):
            raise RuntimeError("not paired with this PC: put the camera in pairing mode "
                               "(Connections > Connect Device > Quik App)") from None
        try:
            for u in (CMD_RSP, SET_RSP, QRY_RSP):
                self.rx[u] = Packets()
                await c.start_notify(u, self._on_rx(u))
        except Exception:
            try:                                # half-made connection: drop it so the next try starts clean
                await c.disconnect()
            except Exception:
                pass
            raise
        self.client = c

    async def _request(self, char, rsp, data, timeout=4.0):
        async with self.lock:
            fut = asyncio.get_running_loop().create_future()
            self.waiting[rsp] = fut
            try:
                await self.client.write_gatt_char(char, data, response=True)
                return await asyncio.wait_for(fut, timeout)
            finally:
                self.waiting.pop(rsp, None)

    async def _command(self, char, rsp, data):
        msg = await self._request(char, rsp, data)
        if len(msg) >= 2 and msg[1] != 0:
            raise RuntimeError(f"camera refused command 0x{msg[0]:02x} (result {msg[1]})")
        return msg

    def _run(self, coro, timeout=10.0):
        return ble_loop().run(coro, timeout)

    # ---- the same calls as gopro.GoPro ----
    def connect(self):
        if self.client is not None and self.client.is_connected:
            return
        if time.time() - self.last_try < 8.0:     # a camera that is off: do not retry every poll
            raise RuntimeError(self.error or "waiting to retry")
        self.last_try = time.time()
        self._run(self._connect(), 25.0)

    def enable_usb(self):
        self.connect()

    def load_info(self):
        pass

    def keep_alive(self):
        self.connect()
        # right after connecting the camera can refuse this once (result 1): harmless, it is resent every poll
        self._run(self._request(SET, SET_RSP, KEEP_ALIVE))

    def set_clock(self, t=None):
        """Camera clock = PC local time, so the file times on the SD card match the trials."""
        lt = time.localtime(t or time.time())
        y = lt.tm_year
        data = bytes([0x09, 0x0D, 0x07, y >> 8, y & 0xFF, lt.tm_mon, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec])
        self.connect()
        self._run(self._command(CMD, CMD_RSP, data))

    def shutter(self, on):
        self.connect()
        self._run(self._command(CMD, CMD_RSP, SHUTTER_ON if on else SHUTTER_OFF))

    # ---- video settings (resolution, frame rate, lens, HyperSmooth) ----
    def get_settings(self, ids):
        """Current values: {"2": 9, "3": 5, ...}."""
        self.connect()
        msg = self._run(self._request(QRY, QRY_RSP, bytes([1 + len(ids), 0x12, *ids])))
        return parse_status(msg)

    def get_caps(self, ids):
        """Values the camera accepts now (they depend on the other settings): {"3": [0, 1, 5], ...}."""
        self.connect()
        msg = self._run(self._request(QRY, QRY_RSP, bytes([1 + len(ids), 0x32, *ids])))
        if len(msg) < 2 or msg[1] != 0:
            raise RuntimeError(f"capabilities query failed (result {msg[1] if len(msg) > 1 else '?'})")
        out, i = {str(k): [] for k in ids}, 2
        while i + 2 <= len(msg):
            sid, ln = msg[i], msg[i + 1]
            out.setdefault(str(sid), []).append(int.from_bytes(msg[i + 2:i + 2 + ln], "big"))
            i += 2 + ln
        return out

    def set_setting(self, sid, value):
        self.connect()
        msg = self._run(self._request(SET, SET_RSP, bytes([3, sid, 1, value])))
        if len(msg) >= 2 and msg[1] != 0:
            raise RuntimeError(f"camera refused setting {sid} = {value} (result {msg[1]})")

    def video_mode(self):
        """Load the Video preset group, so the shutter records video (not photo / time lapse)."""
        self._run(self._command(CMD, CMD_RSP, bytes([0x04, 0x3E, 0x02, 0x03, 0xE8])))

    def poll(self):
        self.connect()
        self.state = parse_status(self._run(self._request(QRY, QRY_RSP, status_query())))
        self.online, self.error, self.last_seen = True, "", time.time()
        return self.state

    def media_files(self):
        return set()                         # no media access over Bluetooth

    def close(self):
        c, self.client = self.client, None
        if c is not None:
            try:
                self._run(c.disconnect(), 5.0)
            except Exception:
                pass
