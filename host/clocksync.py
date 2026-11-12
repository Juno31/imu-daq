"""PC <-> board clock mapping from ping/pong round trips.

Each ping records the PC time it was sent and received; the board answers with its micros().
The board time is taken to correspond to the midpoint of the round trip. Samples with the
smallest round-trip times are the most accurate, so the fit uses the fastest third of recent
samples and fits   pc_time = a + b * board_time   (b - 1 = clock drift).
"""
import threading

import numpy as np


class ClockSync:
    def __init__(self, keep=300):
        self.keep = keep
        self.lock = threading.Lock()
        self.samples = []          # (board_s, pc_mid_s, rtt_s)
        self.fit = None            # (a, b)
        self.stats = {}

    def reset(self):
        with self.lock:
            self.samples.clear()
            self.fit = None
            self.stats = {}

    def add(self, board_us, pc_send_s, pc_recv_s):
        rtt = pc_recv_s - pc_send_s
        with self.lock:
            self.samples.append((board_us / 1e6, (pc_send_s + pc_recv_s) / 2, rtt))
            if len(self.samples) > self.keep:
                del self.samples[: len(self.samples) - self.keep]
            self._refit()

    def _refit(self):
        s = np.array(self.samples)
        if len(s) == 0:
            return
        rtt = s[:, 2]
        if len(s) >= 6:
            good = s[rtt <= np.percentile(rtt, 33)]
        else:
            good = s[rtt <= rtt.min() * 1.5 + 1e-4]
        if len(good) >= 3 and np.ptp(good[:, 0]) > 5.0:
            b, a = np.polyfit(good[:, 0], good[:, 1], 1)
        else:
            k = int(np.argmin(rtt))
            b, a = 1.0, s[k, 1] - s[k, 0]
        resid = good[:, 1] - (a + b * good[:, 0])
        self.fit = (float(a), float(b))
        self.stats = {
            "n": int(len(s)),
            "rtt_ms_median": float(np.median(rtt) * 1e3),
            "rtt_ms_min": float(rtt.min() * 1e3),
            "drift_ppm": float((b - 1.0) * 1e6),
            "resid_ms": float(np.std(resid) * 1e3) if len(resid) > 1 else 0.0,
            "uncertainty_ms": float(np.median(good[:, 2]) * 1e3 / 2),
        }

    def to_pc(self, board_us):
        """PC time (same clock as the samples) for a board micros() value, or None."""
        with self.lock:
            if not self.fit:
                return None
            a, b = self.fit
        return a + b * (board_us / 1e6)

    def plot_data(self, n=90):
        """Recent pings for the GUI: each ping's offset from the fitted line (ms), its round trip,
        and whether it was fast enough to be used in the fit."""
        with self.lock:
            if not self.samples or not self.fit:
                return None
            s = np.array(self.samples[-n:])
            rtt_all = np.array([x[2] for x in self.samples])
            a, b = self.fit
        thr = np.percentile(rtt_all, 33) if len(rtt_all) >= 6 else rtt_all.min() * 1.5 + 1e-4
        off = (s[:, 1] - (a + b * s[:, 0])) * 1e3
        return {"t": np.round(s[:, 0] - s[-1, 0], 2).tolist(), "off_ms": np.round(off, 3).tolist(),
                "rtt_ms": np.round(s[:, 2] * 1e3, 3).tolist(), "used": (s[:, 2] <= thr).tolist()}

    def snapshot(self):
        with self.lock:
            out = dict(self.stats)
            if self.fit:
                out["a"], out["b"] = self.fit
            return out

    def fit_samples(self, samples):
        """Fit on an explicit list of samples (used once per session at stop)."""
        cs = ClockSync(keep=len(samples) + 1)
        cs.samples = list(samples)
        cs._refit()
        return cs
