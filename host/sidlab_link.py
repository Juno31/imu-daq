"""SID Lab spine-brace firmware (teensy_spine_imu.ino): the IMU data packet on the DAQ digital input and the
frame-coded SYNC pulse. Decoding follows the senior's tools/record_daq.py (handover 2026-11-25).

Data line: Teensy pin 1 (TX1, 230400 baud 8N1) -> DAQ P0.0 (terminal 65), sampled at DI_FS on the same clock
as the analog task (the digital task starts on the analog start trigger, so DI sample i is at i / DI_FS s).
One packet per frame, little-endian:
    A5 5A | frame u32 | t_us u32 | ok bits u8 | fresh bits u8 | n x (w,x,y,z) int16 Q14 | n x age_us int32 | Fletcher-16

SYNC (Teensy pin 2): one pulse per frame. Width: frame % 64 == 0 -> 4 ms block marker; the next 32 frames carry
bit k-1 of the block number (frame // 64): 2.5 ms = 1, 1 ms = 0; all other frames 1 ms.
So every pulse can be given its absolute frame number: frame = block * 64 + position.
"""
import numpy as np

BAUD, DI_FS = 230400, 2_000_000
NPODS = 8
PKT = 12 + NPODS * 12 + 2          # 110 bytes


# ---------------- data line (P0.0) ----------------
def decode_uart(edges, levels, first, fs=DI_FS, baud=BAUD):
    """Line transitions (sample index, new level) -> bytes and the sample index of each start bit (8N1, idle high)."""
    edges, levels = np.asarray(edges, np.int64), np.asarray(levels, np.int8)
    tb = fs / baud
    falls = edges[levels == 0]
    if not len(falls):
        return np.zeros(0, np.uint8), np.zeros(0, np.int64)
    centers = falls[:, None] + ((np.arange(10) + 0.5) * tb).astype(np.int64)   # start, 8 data bits LSB first, stop
    k = np.searchsorted(edges, centers, side="right") - 1
    bits = np.where(k >= 0, levels[np.maximum(k, 0)], first)
    ok = (bits[:, 0] == 0) & (bits[:, 9] == 1)
    vals = (bits[:, 1:9].astype(np.uint16) << np.arange(8, dtype=np.uint16)).sum(1)
    out, starts, busy_until = [], [], -1
    for i in range(len(falls)):                      # a byte's own falling edges are not new start bits
        if falls[i] < busy_until:
            continue
        if ok[i]:
            out.append(vals[i])
            starts.append(falls[i])
            busy_until = falls[i] + 9.5 * tb
    return np.array(out, np.uint8), np.array(starts, np.int64)


def parse_packets(data, starts):
    """Bytes -> (packets as dicts, bad checksums, bytes used: the rest may be an unfinished packet)."""
    rows, bad, i, n = [], 0, 0, len(data)
    while i + PKT <= n:
        if data[i] != 0xA5 or data[i + 1] != 0x5A:
            i += 1
            continue
        p = data[i:i + PKT].tobytes()
        s1 = s2 = 0
        for c in p[2:-2]:
            s1 = (s1 + c) & 255
            s2 = (s2 + s1) & 255
        if (s1, s2) != (p[-2], p[-1]):
            bad += 1
            i += 1
            continue
        fr, t_us = np.frombuffer(p[2:10], "<u4")
        q = np.frombuffer(p[12:12 + 8 * NPODS], "<i2").reshape(NPODS, 4) / 16384.0
        age = np.frombuffer(p[12 + 8 * NPODS:12 + 12 * NPODS], "<i4")
        rows.append(dict(frame=int(fr), t_us=int(t_us), ok=p[10], fresh=p[11], q=q, age=age, start=int(starts[i])))
        i += PKT
    return rows, bad, i


class Decoder:
    """Data-line samples in chunks -> finished packets. Undecoded line activity is carried to the next chunk."""

    def __init__(self):
        self.n = 0
        self.last = None
        self.e = np.zeros(0, np.int64)
        self.l = np.zeros(0, np.int8)
        self.cut, self.bad = 0, 0

    def feed(self, x):
        tb = DI_FS / BAUD
        x = np.asarray(x, np.int8)
        if not len(x):
            return []
        if self.last is None:
            self.last = int(x[0])
        d = np.flatnonzero(np.diff(np.concatenate(([self.last], x))))
        self.e = np.concatenate((self.e, self.n + d))
        self.l = np.concatenate((self.l, x[d]))
        self.n += len(x)
        self.last = int(x[-1])
        data, starts = decode_uart(self.e, self.l, 1)
        done = starts < self.n - 11 * tb                                   # bytes fully received
        data, starts = data[done], starts[done]
        rows, bad, used = parse_packets(data, starts)
        self.bad += bad
        if used < len(data):
            self.cut = starts[used]                                        # unfinished packet: decode it again
        elif len(data):
            self.cut = max(self.cut, int(starts[-1] + 10 * tb))
        if not len(data) and not len(self.e):
            self.cut = self.n
        if not len(data) and len(self.e) and self.e[-1] < self.n - 4 * PKT * 10 * tb:
            self.cut = self.n                                              # old line noise, no bytes: drop it
        keep = self.e >= self.cut
        self.e, self.l = self.e[keep], self.l[keep]
        return rows


def packet_times(rows, fs=DI_FS):
    """(daq_t_s of each packet's first start bit, frame_t_s = the frame time on the DAQ clock fitted from the
    Teensy clock t_us, late_ms = how much later than that the packet went out). Same fit as record_daq.write_imu."""
    t = np.array([r["start"] for r in rows], np.float64) / fs
    if len(rows) < 3:
        return t, t.copy(), np.zeros(len(rows))
    tu = np.unwrap(np.array([r["t_us"] for r in rows], np.int64), period=2 ** 32) / 1e6
    if not np.all(np.diff(tu) > 0):                 # a Teensy reset inside the recording: keep the packet times
        return t, t.copy(), np.zeros(len(rows))
    a, b = np.polyfit(tu, t, 1)
    r = t - (a * tu + b)
    good = r <= np.percentile(r, 20) + 50e-6
    a, b = np.polyfit(tu[good], t[good], 1)
    ft = a * tu + b
    return t, ft, (t - ft) * 1e3


def write_packets(path, rows, pod_names, fs=DI_FS):
    """imu_daq.csv: one row per packet received on the DAQ data line. Returns a summary dict."""
    names = list(pod_names)[:NPODS] + [f"pod{k + 1}" for k in range(len(pod_names), NPODS)]
    cols = ["frame", "daq_t_s", "frame_t_s", "late_ms", "t_us"]
    for p in names:
        cols += [f"{p}_ok", f"{p}_fresh", f"{p}_w", f"{p}_x", f"{p}_y", f"{p}_z"]
    cols += [f"{p}_age_us" for p in names]
    t, ft, late = packet_times(rows, fs)
    with open(path, "w", newline="") as f:
        f.write(",".join(cols) + "\n")
        for r, ts, fts, lt in zip(rows, t, ft, late):
            v = [str(r["frame"]), f"{ts:.6f}", f"{fts:.6f}", f"{lt:.3f}", str(r["t_us"])]
            for k in range(NPODS):
                v += [str(r["ok"] >> k & 1), str(r["fresh"] >> k & 1)] + [f"{x:.5f}" for x in r["q"][k]]
            v += [str(a) for a in r["age"]]
            f.write(",".join(v) + "\n")
    fr = np.array([r["frame"] for r in rows], np.int64)
    gaps = int((np.diff(fr) - 1).clip(0).sum()) if len(fr) > 1 else 0
    return {"packets": len(rows), "first_frame": int(fr[0]) if len(fr) else None,
            "last_frame": int(fr[-1]) if len(fr) else None, "missing_frames": gaps,
            "late_packets": int((late > 0.2).sum()), "late_ms_max": round(float(late.max()), 3) if len(rows) else None}


def frame_time_table(rows, fs=DI_FS):
    """{frame: frame_t_s} from the data-line packets."""
    if not rows:
        return {}
    _, ft, _ = packet_times(rows, fs)
    return {r["frame"]: float(x) for r, x in zip(rows, ft)}


# ---------------- frame-coded SYNC ----------------
def sync_width_ms(f):
    """Firmware syncWidth(): the SYNC pulse width (ms) of frame f."""
    k = f % 64
    return 4.0 if k == 0 else (2.5 if k <= 32 and ((f // 64) >> (k - 1)) & 1 else 1.0)


def pulse_frames(rises_s, widths_ms, period_s):
    """Frame number of every SYNC pulse from the width code (-1 = could not be read).
    Blocks between two markers 64 pulses apart are read directly (as record_daq.sync_frames); pulses the code
    cannot reach (an incomplete block at the end, a block with a lost pulse) get theirs from the spacing to the
    nearest read pulse."""
    rises = np.asarray(rises_s, np.float64)
    w = np.asarray(widths_ms, np.float64)
    n = len(w)
    frames = np.full(n, -1, np.int64)
    if not n:
        return frames
    code = np.where(w > 3.25, 2, np.where(w > 1.75, 1, 0))
    marks = np.flatnonzero(code == 2)
    for i, a in enumerate(marks):
        b = marks[i + 1] if i + 1 < len(marks) else n
        if (i + 1 < len(marks) and b - a != 64) or b - a < 33:
            continue
        block = sum(int(code[a + k] == 1) << (k - 1) for k in range(1, 33))
        frames[a:b] = block * 64 + np.arange(b - a)
    known = np.flatnonzero(frames >= 0)
    if not len(known):
        return frames
    for i in range(known[0] + 1, n):                  # forward from the last known pulse, by spacing
        if frames[i] < 0:
            step = int(round((rises[i] - rises[i - 1]) / period_s))
            if step >= 1 and frames[i - 1] >= 0:
                frames[i] = frames[i - 1] + step
    for i in range(known[0] - 1, -1, -1):             # backward from the first known pulse
        step = int(round((rises[i + 1] - rises[i]) / period_s))
        if step >= 1 and frames[i + 1] - step >= 0:
            frames[i] = frames[i + 1] - step
    return frames


def align_coded(pulses, rate, frames_seen, frame_us):
    """DAQ pulses [(rise_sample, width_samples)] -> (frame_times_s indexed by frame number, report).
    frames_seen: frame numbers received from the board (USB)."""
    rep = {"method": "frame-coded SYNC", "pulses_total": len(pulses)}
    seen = np.asarray(sorted(set(int(f) for f in frames_seen)), np.int64)
    rep["frames"] = int(len(seen))
    if not pulses:
        rep.update(ok=False, reason="no SYNC pulses on the DAQ")
        return None, rep
    rises = np.array([r for r, _ in pulses], np.float64) / rate
    widths = np.array([w for _, w in pulses], np.float64) / rate * 1e3
    fr = pulse_frames(rises, widths, frame_us / 1e6)
    good = fr >= 0
    rep["pulses_with_frame"] = int(good.sum())
    rep["pulses_unread"] = int((~good).sum())
    if not good.any():
        rep.update(ok=False, reason="SYNC frame code not readable (no complete 64-frame block)")
        return None, rep
    top = int(max(fr[good].max(), seen.max() if len(seen) else 0))
    frame_times = np.full(top + 1, np.nan)
    frame_times[fr[good]] = rises[good]
    rep["first_pulse_frame"] = int(fr[good].min())
    rep["last_pulse_frame"] = int(fr[good].max())
    if len(seen):
        miss = int(np.isnan(frame_times[seen]).sum())
        rep["frames_without_pulse"] = miss
        rep["pulses_beyond_last_frame"] = int((fr[good] > seen.max()).sum())
        rep["pulses_before_first_frame"] = int((fr[good] < seen.min()).sum())
        rep["ok"] = bool(miss == 0)
        if miss:
            rep["reason"] = f"{miss} of {len(seen)} received frames have no SYNC pulse"
    else:
        rep["ok"] = True
    iv = np.diff(rises) * 1e3
    if len(iv):
        rep["interval_ms"] = {"mean": float(iv.mean()), "min": float(iv.min()), "max": float(iv.max()),
                              "sd": float(iv.std())}
    return frame_times, rep


def selftest():
    rng = np.random.default_rng(1)
    msg = rng.integers(0, 256, 3 * PKT, dtype=np.uint8)
    pkt = bytearray([0xA5, 0x5A]) + bytes(rng.integers(0, 256, PKT - 4, dtype=np.uint8))
    s1 = s2 = 0
    for c in pkt[2:]:
        s1 = (s1 + c) & 255
        s2 = (s2 + s1) & 255
    pkt += bytes([s1, s2])
    stream = np.concatenate([msg, np.frombuffer(bytes(pkt), np.uint8)])
    tb = DI_FS / BAUD * 1.002
    bits = np.concatenate([np.r_[0, (b >> np.arange(8)) & 1, 1, 1] for b in stream])
    t = 1000 + (np.arange(len(bits)) * tb).astype(np.int64)
    lv = np.concatenate(([1], bits))
    ch = np.flatnonzero(np.diff(lv))
    data, starts = decode_uart(t[ch], bits[ch].astype(np.int8), 1)
    assert np.array_equal(data, stream), "uart decode"
    rows, _, _ = parse_packets(data, starts)
    assert len(rows) >= 1 and rows[-1]["start"] == t[len(msg) * 11], "packet parse"
    x = np.ones(t[-1] + 500, np.int8)
    for i in range(len(bits)):
        x[t[i]:(t[i + 1] if i + 1 < len(t) else len(x))] = bits[i]
    dec, got, i = Decoder(), [], 0
    for n in rng.integers(1000, 30000, 1000):
        got += dec.feed(x[i:i + n])
        i += n
        if i >= len(x):
            break
    assert len(got) == len(rows) and got[-1]["start"] == rows[-1]["start"], "chunked decode"
    # SYNC code: 700 frames from 1000, plus a short last block (read by spacing)
    period = 8.333e-3
    fr = np.arange(1000, 1730)
    rate = 20000
    pulses = [(int((0.004 + k * period) * rate), int(sync_width_ms(f) * rate / 1000)) for k, f in enumerate(fr)]
    ft, rep = align_coded(pulses, rate, fr, 8333)
    assert rep["ok"], rep
    assert abs(ft[1729] - (0.004 + 729 * period)) < 1e-3, "last frames by spacing"
    print("sidlab_link selftest OK")


if __name__ == "__main__":
    selftest()
