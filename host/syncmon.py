"""Live view of the SYNC matching while a recording runs.

This does incrementally what daq.align() does once at Stop, so the GUI can show it happening:
  * every DAQ pulse after the wide frame-0 marker is given a frame number from its spacing to the
    previous pulse (pulse k = frame k),
  * each frame line from the board (frame k, board time t_us) is paired with pulse k,
  * a running straight-line fit  daq_time = c + d * board_time  shows the clock drift, and the
    residual of each pair shows how tightly the two clocks agree (limited by the DAQ sample period).
"""
import collections
import math
import threading

import numpy as np

SETTLE_S = 0.6          # a frame without a pulse after this long counts as "no pulse"
SCOPE_PERIODS = 5.0     # scope window, in frame periods
SCOPE_PRE = 0.4         # scope trigger position, in frame periods


class SyncMonitor:
    def __init__(self, frame_us, daq_rate=None):
        self.lock = threading.Lock()
        self.frame_us = int(frame_us)
        self.period = frame_us / 1e6
        self.rate = float(daq_rate) if daq_rate else None
        self.active = True
        # board side
        self.board = []                    # t_us per frame number (None = line not received)
        # DAQ side
        self.daq = []                      # pulse rise time [s] per frame number (nan = no pulse)
        self.seen = 0                      # pulses processed
        self.pre_marker = 0
        self.extra = 0
        self.marker = None                 # (rise_sample, width_ms)
        self.prev_rise_s, self.prev_k = None, -1
        self.rise_to_k = collections.OrderedDict()
        # matching
        self.ptr = 0                       # next frame to settle
        self.codes = bytearray()           # per settled frame: 0 ok, 1 no pulse, 2 no board line
        self.matched = self.no_pulse = self.no_line = 0
        self.origin = None                 # (board_t_us, daq_s) of the first matched pair
        self.n = 0
        self.mx = self.my = self.cxx = self.cxy = 0.0
        self.recent = collections.deque(maxlen=1500)   # (k, x_s, y_s)
        self.scope = None
        self.marker_scope = None

    # ---------- inputs ----------
    def add_frame(self, num, t_us):
        with self.lock:
            if num < len(self.board):
                self.board[num] = t_us
                return
            if num - len(self.board) > 100000:           # nonsense frame number
                return
            self.board.extend([None] * (num - len(self.board)))
            self.board.append(t_us)

    def update(self, rec, final=False):
        """Process new DAQ pulses (rec = daq.DaqRecorder or None) and settle frames."""
        with self.lock:
            if rec is not None and self.rate:
                self._take_pulses(rec)
                if self.active:
                    sc = self._scope(rec)
                    if sc:
                        self.scope = sc
                    if self.marker and self.marker_scope is None:
                        need = self.marker[0] + int((SCOPE_PERIODS - SCOPE_PRE) * self.period * self.rate)
                        if rec.n_samples >= need:
                            self.marker_scope = self._scope(rec, around=self.marker[0]) or False
            last = len(self.board) - 1
            settled = last if final else last - int(SETTLE_S / self.period)
            if self.rate is None:
                settled = -1                                  # nothing to match without a DAQ
            for k in range(self.ptr, settled + 1):
                b = self.board[k]
                d = self.daq[k] if k < len(self.daq) else math.nan
                if b is None:
                    self.no_line += 1
                    self.codes.append(2)
                elif math.isnan(d):
                    self.no_pulse += 1
                    self.codes.append(1)
                else:
                    self.matched += 1
                    self.codes.append(0)
                    self._fit_add(k, b, d)
            self.ptr = max(self.ptr, settled + 1)
            if final:
                self.active = False

    def _take_pulses(self, rec):
        pulses = rec.edges.pulses
        n = len(pulses)
        for i in range(self.seen, n):
            rise, width = pulses[i]
            rs, wms = rise / self.rate, width / self.rate * 1e3
            if self.marker is None:
                if wms < 2.0:
                    self.pre_marker += 1
                    continue
                self.marker = (rise, wms)
                k = 0
            else:
                step = int(round((rs - self.prev_rise_s) / self.period))
                if step < 1:
                    self.extra += 1
                    continue
                k = self.prev_k + step
            self.prev_rise_s, self.prev_k = rs, k
            if k > len(self.daq):
                self.daq.extend([math.nan] * (k - len(self.daq)))
            if k == len(self.daq):
                self.daq.append(rs)
            else:
                self.daq[k] = rs
            self.rise_to_k[rise] = k
            if len(self.rise_to_k) > 600:
                self.rise_to_k.popitem(last=False)
        self.seen = n

    def _fit_add(self, k, b_us, d_s):
        if self.origin is None:
            self.origin = (b_us, d_s)
        x = (b_us - self.origin[0]) / 1e6
        y = d_s - self.origin[1]
        self.n += 1                         # Welford-style running covariance (numerically stable)
        dx = x - self.mx
        self.mx += dx / self.n
        self.my += (y - self.my) / self.n
        self.cxy += dx * (y - self.my)
        self.cxx += dx * (x - self.mx)
        self.recent.append((k, x, y))

    def _line(self):
        if self.n < 2 or self.cxx <= 0:
            return None
        slope = self.cxy / self.cxx
        return self.my - slope * self.mx, slope

    # ---------- scope ----------
    def _scope(self, rec, around=None):
        rate, P = self.rate, self.period
        n_pre, n_win = int(SCOPE_PRE * P * rate), int(SCOPE_PERIODS * P * rate)
        first, v = rec.recent(int(0.5 * rate))
        if len(v) < n_win:
            return None
        end = first + len(v)
        pulses = rec.edges.pulses
        trig = around
        if trig is None:
            for i in range(len(pulses) - 1, max(-1, len(pulses) - 300), -1):
                r = pulses[i][0]
                if r - n_pre < first:
                    break
                if r - n_pre + n_win <= end:
                    trig = r
                    break
        start = (trig - n_pre) if trig is not None else end - n_win
        if start < first or start + n_win > end:
            return None
        seg = v[start - first: start - first + n_win]
        step = 1
        if len(seg) > 1200:                                  # keep the pulse shape: max per bin
            step = int(math.ceil(len(seg) / 1000))
            m = len(seg) // step * step
            seg = seg[:m].reshape(-1, step).max(axis=1)
        edges = []
        for r, w in pulses[-300:]:
            if start <= r < start + n_win:
                edges.append({"t_ms": round((r - start) / rate * 1e3, 3), "w_ms": round(w / rate * 1e3, 3),
                              "k": self.rise_to_k.get(r)})
        return {"dt_ms": 1e3 * step / rate, "v": np.round(seg, 3).tolist(), "edges": edges,
                "triggered": trig is not None, "win_ms": n_win / rate * 1e3}

    # ---------- output ----------
    def snapshot(self):
        with self.lock:
            last_board = len(self.board) - 1
            last_pulse = len(self.daq) - 1
            line = self._line()
            res, sd, mx = [], None, None
            if line and self.recent:
                c, d = line
                pts = list(self.recent)[-1000:]
                step = max(1, -(-len(pts) // 400))
                r_all = np.array([(y - (c + d * x)) * 1e6 for _, x, y in pts])
                sd, mx = float(r_all.std()), float(np.abs(r_all).max())
                x_end = pts[-1][1]
                res = [[round(pts[i][1] - x_end, 3), round(float(r_all[i]), 1)] for i in range(0, len(pts), step)]
            # strip: last 200 frames, 0 ok / 1 no pulse / 2 no line / 3 pending
            n_strip = 200
            lo = max(0, last_board - n_strip + 1)
            strip = []
            for k in range(lo, last_board + 1):
                strip.append(self.codes[k] if k < len(self.codes) else 3)
            return {
                "active": self.active, "has_daq": self.rate is not None,
                "frame_ms": self.period * 1e3, "daq_rate": self.rate,
                "frames": last_board + 1, "pulses": self.seen, "last_pulse_frame": last_pulse,
                "lag_frames": (last_board - last_pulse) if self.marker else None,
                "marker_found": self.marker is not None,
                "marker_width_ms": round(self.marker[1], 2) if self.marker else None,
                "pre_marker": self.pre_marker, "extra": self.extra,
                "matched": self.matched, "no_pulse": self.no_pulse, "no_line": self.no_line,
                "drift_ppm": round((line[1] - 1) * 1e6, 2) if line and self.n > 50 else None,
                "resid_sd_us": round(sd, 1) if sd is not None else None,
                "resid_max_us": round(mx, 1) if mx is not None else None,
                "sample_us": 1e6 / self.rate if self.rate else None,
                "resid": res, "strip": "".join(map(str, strip)), "strip_first": lo,
                "scope": self.scope or None, "marker_scope": self.marker_scope or None,
            }
