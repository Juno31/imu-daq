"""NI-DAQmx recording of the SYNC line (plus optional extra channels) and pulse alignment.

The XIAO sends one SYNC pulse per recorded IMU frame: 3 ms for frame 0, 1 ms for every other
frame. The DAQ records the line continuously; each rising edge after the wide frame-0 pulse is
the DAQ time of one IMU frame (pulse k = frame k, also when a frame line was lost on USB).
"""
import json
import os
import threading
import time

import numpy as np

import channels

try:
    import nidaqmx
    from nidaqmx.constants import AcquisitionType, TerminalConfiguration
    from nidaqmx.stream_readers import AnalogMultiChannelReader
    HAVE_NIDAQMX = True
    _IMPORT_ERROR = ""
except Exception as e:          # pragma: no cover - depends on the PC
    HAVE_NIDAQMX = False
    _IMPORT_ERROR = str(e)

TERMINALS = {"DIFF": "DIFF", "RSE": "RSE", "NRSE": "NRSE", "PSEUDO_DIFF": "PSEUDO_DIFF"}
HI_V, LO_V = 2.0, 1.0
RING_S = 2.0                    # seconds of every channel kept in memory for the live views
# An unconnected (floating) input cannot drive the DAQ's sampling capacitor, so it repeats the channel
# scanned before it: the SYNC pulses show up on it at 30-98 % of their height. A connected, low-impedance
# source (EMG amplifier output) picks up < 0.001 % (measured on the USB-6363). Correlation with SYNC above
# this value therefore means "nothing connected to this input".
FLOAT_CORR = 0.5           # Schmitt thresholds for a 3.3 V logic pulse


def list_devices():
    if not HAVE_NIDAQMX:
        return {"available": False, "error": f"nidaqmx not importable: {_IMPORT_ERROR}", "devices": []}
    try:
        devs = []
        for d in nidaqmx.system.System.local().devices:
            try:
                ai = [c.name.split("/")[-1] for c in d.ai_physical_chans]
            except Exception:
                ai = []
            try:
                max_rate = float(d.ai_max_single_chan_rate)
            except Exception:
                max_rate = None
            devs.append({"name": d.name, "product": d.product_type, "ai": ai,
                         "max_rate": max_rate, "simulated": bool(d.is_simulated)})
        return {"available": True, "error": "", "devices": devs}
    except Exception as e:
        return {"available": False, "error": f"NI-DAQmx error: {e}", "devices": []}


class EdgeDetector:
    """Rising edges and pulse widths with hysteresis, across chunk boundaries."""

    def __init__(self):
        self.state = False
        self.rise_at = None
        self.pulses = []           # (rise_sample, width_samples)

    def feed(self, v, start_index):
        if len(v) == 0:
            return
        hi, lo = v > HI_V, v < LO_V
        # walk only the samples where the state can change
        idx = np.flatnonzero(hi | lo)
        for i in idx:
            if not self.state and hi[i]:
                self.state = True
                self.rise_at = start_index + int(i)
            elif self.state and lo[i]:
                self.state = False
                if self.rise_at is not None:
                    self.pulses.append((self.rise_at, start_index + int(i) - self.rise_at))
                self.rise_at = None


class DaqRecorder:
    def __init__(self, out_dir, device, sync_chan="ai0", extra_chans=(), rate=20000.0,
                 vrange=5.0, terminal="DIFF"):
        self.out_dir = out_dir
        self.device = device
        self.sync_chan = sync_chan
        self.extra_chans = [c for c in extra_chans if c and c != sync_chan]
        self.rate = float(rate)
        self.vrange = float(vrange)
        self.terminal = terminal
        self.task = None
        self.thread = None
        self.stop_evt = threading.Event()
        self.n_samples = 0
        self.edges = EdgeDetector()
        self.error = ""
        self.pc_start = None
        self.last_v = 0.0
        self.fh = None
        # last RING_S seconds of every channel (row 0 = SYNC): live scope and EMG view
        self.ring = np.zeros((len(self.channels), max(100, int(self.rate * RING_S))))
        self.ring_lock = threading.Lock()
        self.ch_stats = []          # per extra channel, last 50 ms: (sd_V, absmax_V)
        self.sync_hi = 0.0          # max SYNC level in the last 50 ms

    @property
    def channels(self):
        return [self.sync_chan] + self.extra_chans

    def start(self, pc_clock):
        """out_dir None = preview only (nothing written)."""
        if self.out_dir:
            os.makedirs(self.out_dir, exist_ok=True)
        term = getattr(TerminalConfiguration, TERMINALS.get(self.terminal, "DIFF"))
        self.task = nidaqmx.Task("imu_daq_sync" if self.out_dir else "imu_daq_preview")
        for ch in self.channels:
            self.task.ai_channels.add_ai_voltage_chan(f"{self.device}/{ch}", terminal_config=term,
                                                      min_val=-self.vrange, max_val=self.vrange)
        self.task.timing.cfg_samp_clk_timing(self.rate, sample_mode=AcquisitionType.CONTINUOUS,
                                             samps_per_chan=int(self.rate * 10))
        self.reader = AnalogMultiChannelReader(self.task.in_stream)
        if self.out_dir:
            self.fh = open(os.path.join(self.out_dir, "daq_raw.f32"), "wb")
        self.task.start()
        self.pc_start = pc_clock()
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._run, name="daq-reader", daemon=True)
        self.thread.start()

    def _run(self):
        n = max(10, int(self.rate / 20))          # 50 ms chunks
        buf = np.zeros((len(self.channels), n), dtype=np.float64)
        while not self.stop_evt.is_set():
            try:
                self.reader.read_many_sample(buf, number_of_samples_per_channel=n, timeout=2.0)
            except Exception as e:
                if not self.stop_evt.is_set():
                    self.error = f"DAQ read error: {e}"
                break
            if self.fh:
                self.fh.write(buf.T.astype(np.float32).tobytes())   # sample-major: s0c0 s0c1 ...
            self.sync_hi = float(buf[0].max())
            if len(buf) > 1:
                self.ch_stats = [(float(r.std()), float(np.abs(r).max())) for r in buf[1:]]
            self.edges.feed(buf[0], self.n_samples)
            with self.ring_lock:
                L = self.ring.shape[1]
                idx = np.arange(self.n_samples, self.n_samples + n) % L
                self.ring[:, idx] = buf
                self.n_samples += n
            self.last_v = float(buf[0, -1])

    def stop(self):
        self.stop_evt.set()
        try:
            if self.task:
                self.task.stop()
        except Exception:
            pass
        if self.thread:
            self.thread.join(timeout=3)
        try:
            if self.task:
                self.task.close()
        except Exception:
            pass
        self.task = None
        if self.fh:
            self.fh.close()
            self.fh = None
        if not self.out_dir:
            return None
        meta = {"device": self.device, "channels": self.channels, "rate_hz": self.rate,
                "range_v": self.vrange, "terminal": self.terminal, "n_samples": self.n_samples,
                "pc_start_s": self.pc_start, "dtype": "float32", "layout": "sample-major (n_samples x n_channels)",
                "file": "daq_raw.f32", "error": self.error,
                "channel_labels": [channels.label(c, self.sync_chan) for c in self.channels]}
        with open(os.path.join(self.out_dir, "daq_meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        return meta

    def recent(self, n):
        """(absolute index of the first sample, last n samples of the SYNC channel)."""
        with self.ring_lock:
            end = self.n_samples
            n = int(min(n, self.ring.shape[1], end))
            idx = np.arange(end - n, end) % self.ring.shape[1]
            return end - n, self.ring[0, idx].copy()

    def emg_view(self, points=300, seconds=RING_S, rms_s=0.1):
        """Per extra channel: min/max envelope of the last `seconds` (points bins), RMS of the last
        `rms_s` (mean removed) and a clipping flag (|v| > 98 % of the range in the window)."""
        if not self.extra_chans:
            return None
        with self.ring_lock:
            end = self.n_samples
            n = int(min(seconds * self.rate, self.ring.shape[1], end))
            idx = np.arange(end - n, end) % self.ring.shape[1]
            allx = self.ring[:, idx].copy()
        sync, x = allx[0], allx[1:]
        pulses = n > 10 and float(sync.std()) > 0.3          # SYNC is pulsing (recording), so we can tell
        out = []
        b = max(1, n // points)
        m = n // b * b
        nr = max(1, int(rms_s * self.rate))
        for c, row in zip(self.extra_chans, x):
            lab = channels.label(c, self.sync_chan)
            if m == 0:
                out.append(dict(lab, lo=[], hi=[], rms_mv=None, clip=False))
                continue
            r = row[n - m:].reshape(-1, b)
            tail = row[-nr:]
            corr = float(np.corrcoef(row, sync)[0, 1]) if pulses and row.std() > 0 else None
            out.append(dict(lab, lo=np.round(r.min(axis=1), 5).tolist(), hi=np.round(r.max(axis=1), 5).tolist(),
                            rms_mv=round(float(tail.std()) * 1e3, 4),
                            clip=bool(np.abs(row).max() > 0.98 * self.vrange),
                            sync_corr=None if corr is None else round(corr, 3),
                            floating=None if corr is None else bool(corr > FLOAT_CORR)))
        return {"seconds": m / self.rate, "range_v": self.vrange, "chans": out}

    def live(self):
        ex = [{"chan": c, "sd_mv": round(sd * 1e3, 3), "max_v": round(mx, 3)}
              for c, (sd, mx) in zip(self.extra_chans, self.ch_stats)]
        return {"samples": self.n_samples, "seconds": self.n_samples / self.rate if self.rate else 0,
                "pulses": len(self.edges.pulses), "last_v": self.last_v, "sync_hi": self.sync_hi,
                "error": self.error, "extra": ex, "range_v": self.vrange,
                "running": bool(self.thread and self.thread.is_alive())}


def align(pulses, rate, n_frames, frame_us):
    """Match DAQ pulses to IMU frames. Returns (frame_times_s or None, report dict)."""
    rep = {"pulses_total": len(pulses), "frames": n_frames}
    if not pulses:
        rep.update(ok=False, reason="no SYNC pulses on the DAQ")
        return None, rep
    widths_ms = np.array([w for _, w in pulses]) / rate * 1e3
    start_k = next((k for k, w in enumerate(widths_ms) if w >= 2.0), None)
    if start_k is None:
        rep.update(ok=False, reason="frame-0 marker pulse (3 ms) not found")
        return None, rep
    rises = np.array([r for r, _ in pulses[start_k:]], dtype=np.float64) / rate
    rep["pulses_after_marker"] = int(len(rises))
    rep["marker_width_ms"] = float(widths_ms[start_k])
    rep["pulses_before_marker"] = int(start_k)
    period = frame_us / 1e6
    # Frame number of each pulse from its spacing to the previous pulse: robust to a missing or
    # extra pulse and to slow drift between the DAQ and board clocks.
    k = np.zeros(len(rises), dtype=np.int64)
    if len(rises) > 1:
        steps = np.rint(np.diff(rises) / period).astype(np.int64)
        k[1:] = np.cumsum(steps)
        iv = np.diff(rises) * 1e3
        rep["interval_ms"] = {"mean": float(iv.mean()), "min": float(iv.min()), "max": float(iv.max()),
                              "sd": float(iv.std())}
        rep["intervals_off_by_more_than_2ms"] = int(np.sum(np.abs(iv - frame_us / 1e3) > 2.0))
        rep["extra_pulses"] = int(np.sum(steps < 1))
    frame_times = np.full(max(n_frames, 0), np.nan)
    valid = (k >= 0) & (k < n_frames)
    frame_times[k[valid]] = rises[valid]
    rep["pulses_beyond_last_frame"] = int(np.sum(k >= n_frames))
    rep["frames_without_pulse"] = int(np.sum(np.isnan(frame_times)))
    rep["count_match"] = bool(len(rises) == n_frames)
    rep["ok"] = bool(rep["frames_without_pulse"] == 0 and rep.get("extra_pulses", 0) == 0
                     and rep["pulses_beyond_last_frame"] == 0)
    if not rep["ok"]:
        rep["reason"] = (f"{len(rises)} pulses after the marker vs {n_frames} frames "
                         f"({rep['frames_without_pulse']} frames without a pulse, "
                         f"{rep.get('extra_pulses', 0) + rep['pulses_beyond_last_frame']} extra pulses)")
    return frame_times, rep


def board_vs_daq(board_t_us, daq_t_s):
    """Fit daq_time = c + d * board_time over frames that have both; residual shows sync quality."""
    m = np.isfinite(daq_t_s)
    if m.sum() < 3:
        return None
    bt = (np.asarray(board_t_us, dtype=np.float64)[m] - board_t_us[m][0]) / 1e6
    d, c = np.polyfit(bt, daq_t_s[m], 1)
    res = daq_t_s[m] - (c + d * bt)
    return {"frames_used": int(m.sum()), "drift_ppm": float((d - 1) * 1e6),
            "residual_ms_sd": float(res.std() * 1e3), "residual_ms_max": float(np.abs(res).max() * 1e3)}
