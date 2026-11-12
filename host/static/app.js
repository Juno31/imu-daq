"use strict";
// IMU DAQ browser GUI: a view with buttons. All acquisition runs in server.py.

const $ = (id) => document.getElementById(id);
const LS = {
  get(k, d) { try { const v = localStorage.getItem("imudaq." + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("imudaq." + k, JSON.stringify(v)); } catch { } },
};

let S = null;                 // latest server status
let windowSec = LS.get("window", 10);
const pods = new Map();       // name -> pod view
const MAXKEEP = 35;           // seconds of samples kept per pod
let lastLogLevelFilter = "all";

// ---------------- math ----------------
function qmul(a, b) {
  return [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
}
const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
const D = 180 / Math.PI;
function euler(q) {   // ZYX: roll (x), pitch (y), yaw (z), degrees
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * D;
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) * D;
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * D;
  return [roll, pitch, yaw];
}
function rotm(q) {
  const [w, x, y, z] = q;
  return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]];
}
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// ---------------- pod views ----------------
function makePod(name) {
  const node = $("podTpl").content.firstElementChild.cloneNode(true);
  node.querySelector(".podname").textContent = name;
  $("pods").appendChild(node);
  const p = {
    name, node,
    light: node.querySelector(".light"), state: node.querySelector(".podstate"),
    addr: node.querySelector(".addr"), fresh: node.querySelector(".fresh"), stalls: node.querySelector(".stalls"),
    cube: node.querySelector(".cube"), plot: node.querySelector(".plot"),
    roll: node.querySelector(".roll b"), pitch: node.querySelector(".pitch b"), yaw: node.querySelector(".yaw b"),
    quat: node.querySelector(".quat"),
    t: [], e: [], q: [1, 0, 0, 0], ref: LS.get("ref." + name, null), dirty: true,
  };
  pods.set(name, p);
  return p;
}

function podFor(name) { return pods.get(name) || makePod(name); }

function addFrames(rows) {
  if (!S || !S.hello || !S.hello.pods) return;
  const names = S.hello.pods.split(",").map((s) => s.split(":")[0]);
  for (const r of rows) {
    const t = r[0] / 1e6;
    for (let k = 0; k < names.length; k++) {
      const o = 3 + 6 * k;
      if (r.length < o + 6) break;
      const p = podFor(names[k]);
      const q = [r[o + 2], r[o + 3], r[o + 4], r[o + 5]];
      if (p.t.length && t < p.t[p.t.length - 1] - 1) { p.t = []; p.e = []; }   // board restarted
      p.q = q;
      p.t.push(t);
      p.e.push(euler(p.ref ? qmul(qconj(p.ref), q) : q));
      p.dirty = true;
    }
  }
  for (const p of pods.values()) {
    if (!p.t.length) continue;
    const cut = p.t[p.t.length - 1] - MAXKEEP;
    let i = 0;
    while (i < p.t.length && p.t[i] < cut) i++;
    if (i > 200) { p.t.splice(0, i); p.e.splice(0, i); }
  }
}

function zeroAll(on) {
  for (const p of pods.values()) {
    p.ref = on ? p.q.slice() : null;
    LS.set("ref." + p.name, p.ref);
    // recompute history so the plot is consistent
    p.e = [];
  }
  toast(on ? "Display zeroed at the current pose (recorded data is unchanged)" : "Zero reset");
}

// ---------------- drawing ----------------
function fitCanvas(c) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(c.clientWidth * dpr), h = Math.round(c.clientHeight * dpr);
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const ctx = c.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return [ctx, c.clientWidth, c.clientHeight];
}

function drawPlot(p) {
  const [ctx, W, H] = fitCanvas(p.plot);
  ctx.clearRect(0, 0, W, H);
  const padL = 34, padR = 8, padT = 6, padB = 18;
  const pw = W - padL - padR, ph = H - padT - padB;
  const grid = css("--border"), txt = css("--text-3");
  ctx.font = "11px " + css("--mono");
  ctx.fillStyle = txt; ctx.strokeStyle = grid; ctx.lineWidth = 1;
  for (const v of [-180, -90, 0, 90, 180]) {
    const y = padT + ph * (1 - (v + 180) / 360);
    ctx.globalAlpha = v === 0 ? 1 : 0.6;
    ctx.beginPath(); ctx.moveTo(padL, Math.round(y) + 0.5); ctx.lineTo(W - padR, Math.round(y) + 0.5); ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.textAlign = "right"; ctx.textBaseline = "middle"; ctx.fillText(String(v), padL - 6, y);
  }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  const step = windowSec <= 5 ? 1 : windowSec <= 10 ? 2 : 5;
  for (let s = 0; s <= windowSec; s += step) {
    const x = padL + pw * (1 - s / windowSec);
    ctx.fillText(s === 0 ? "now" : `-${s}s`, x, H - padB + 4);
  }
  if (!p.t.length) return;
  if (p.e.length !== p.t.length) {   // after zero: rebuild from quats is not possible -> restart history
    p.t = p.t.slice(-1); p.e = [euler(p.ref ? qmul(qconj(p.ref), p.q) : p.q)];
  }
  const tEnd = p.t[p.t.length - 1], t0 = tEnd - windowSec;
  let i0 = p.t.length - 1;
  while (i0 > 0 && p.t[i0 - 1] >= t0) i0--;
  const cols = [css("--roll"), css("--pitch"), css("--yaw")];
  ctx.save();
  ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
  ctx.lineWidth = 1.6; ctx.lineJoin = "round";
  for (let j = 0; j < 3; j++) {
    ctx.strokeStyle = cols[j];
    ctx.beginPath();
    let prev = null;
    for (let i = i0; i < p.t.length; i++) {
      const x = padL + pw * (1 - (tEnd - p.t[i]) / windowSec);
      const v = p.e[i][j];
      const y = padT + ph * (1 - (v + 180) / 360);
      if (prev === null || Math.abs(v - prev) > 180) ctx.moveTo(x, y); else ctx.lineTo(x, y);   // no wrap lines
      prev = v;
    }
    ctx.stroke();
  }
  ctx.restore();
}

// board-shaped box in sensor axes: x long, y short, z thin
const BOX = (() => {
  const a = 1.0, b = 0.62, c = 0.14, v = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) v.push([sx * a, sy * b, sz * c]);
  // faces as vertex indices (v index = (sx>0)*4 + (sy>0)*2 + (sz>0))
  const F = [[1, 3, 7, 5, "top"], [0, 4, 6, 2, "bottom"], [4, 5, 7, 6, "front"], [0, 2, 3, 1, "back"],
  [2, 6, 7, 3, "side"], [0, 1, 5, 4, "side"]];
  return { v, F };
})();
const VIEW = { az: -35 * Math.PI / 180, el: 28 * Math.PI / 180 };

function project(p, cx, cy, s) {
  const ca = Math.cos(VIEW.az), sa = Math.sin(VIEW.az), ce = Math.cos(VIEW.el), se = Math.sin(VIEW.el);
  const x1 = p[0] * ca - p[1] * sa, y1 = p[0] * sa + p[1] * ca, z1 = p[2];
  const sy = z1 * ce + y1 * se, depth = y1 * ce - z1 * se;
  const k = 1 / (1 + depth * 0.12);
  return [cx + x1 * s * k, cy - sy * s * k, depth];
}

function drawCube(p) {
  const [ctx, W, H] = fitCanvas(p.cube);
  ctx.clearRect(0, 0, W, H);
  const q = p.ref ? qmul(qconj(p.ref), p.q) : p.q;
  const R = rotm(q);
  const rot = (v) => [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
  R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
  R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]];
  const cx = W / 2, cy = H / 2 + 6, s = Math.min(W, H) * 0.34;
  // ground ellipse
  ctx.fillStyle = css("--surface-2");
  ctx.beginPath(); ctx.ellipse(cx, cy + s * 0.95, s * 1.15, s * 0.22, 0, 0, Math.PI * 2); ctx.fill();
  const P = BOX.v.map((v) => project(rot(v), cx, cy, s));
  const light = [0.3, -0.5, 0.8];
  const accent = css("--accent"), side = css("--border-strong"), edge = css("--text-3");
  const faces = BOX.F.map((f) => {
    const n0 = rot([f[4] === "top" ? 0 : 0, 0, 0]);
    const a = BOX.v[f[0]], b = BOX.v[f[1]], c = BOX.v[f[2]];
    const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
    n = rot(n);
    const len = Math.hypot(...n) || 1;
    const shade = Math.max(0.35, (n[0] * light[0] + n[1] * light[1] + n[2] * light[2]) / len * 0.5 + 0.6);
    const depth = (P[f[0]][2] + P[f[1]][2] + P[f[2]][2] + P[f[3]][2]) / 4;
    void n0;
    return { f, shade, depth };
  }).sort((a, b) => b.depth - a.depth);
  for (const { f, shade } of faces) {
    ctx.beginPath();
    for (let i = 0; i < 4; i++) { const pt = P[f[i]]; i ? ctx.lineTo(pt[0], pt[1]) : ctx.moveTo(pt[0], pt[1]); }
    ctx.closePath();
    ctx.fillStyle = f[4] === "top" ? accent : side;
    ctx.globalAlpha = f[4] === "top" ? 0.55 + 0.4 * shade : 0.5 + 0.45 * shade;
    ctx.fill();
    ctx.globalAlpha = 1; ctx.strokeStyle = edge; ctx.lineWidth = 0.8; ctx.stroke();
  }
  // body axes
  const o = project([0, 0, 0], cx, cy, s);
  const ax = [[[1.45, 0, 0], css("--bad"), "x"], [[0, 1.0, 0], css("--ok"), "y"], [[0, 0, 0.75], css("--accent"), "z"]];
  ctx.font = "600 11px " + css("--mono"); ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (const [v, col, lab] of ax) {
    const e = project(rot(v), cx, cy, s);
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(e[0], e[1]); ctx.stroke();
    ctx.fillText(lab, e[0] + (e[0] - o[0]) * 0.12, e[1] + (e[1] - o[1]) * 0.12);
  }
}

// ---------------- sync monitor ----------------
let SM = null, CP = null, smDirty = true, scopeMode = LS.get("scopeMode", "live");

function plotFrame(c, padL = 44, padR = 10, padT = 10, padB = 20) {
  const [ctx, W, H] = fitCanvas(c);
  ctx.clearRect(0, 0, W, H);
  ctx.font = "11px " + css("--mono");
  return { ctx, W, H, padL, padR, padT, padB, pw: W - padL - padR, ph: H - padT - padB };
}
function placeholder(c, text) {
  const f = plotFrame(c);
  f.ctx.fillStyle = css("--text-3"); f.ctx.font = "12.5px " + getComputedStyle(document.body).fontFamily;
  f.ctx.textAlign = "center"; f.ctx.textBaseline = "middle";
  f.ctx.fillText(text, f.W / 2, f.H / 2);
}
function hline(f, y, dash, col, alpha = 1) {
  const { ctx } = f;
  ctx.save(); ctx.strokeStyle = col; ctx.globalAlpha = alpha; ctx.lineWidth = 1; ctx.setLineDash(dash || []);
  ctx.beginPath(); ctx.moveTo(f.padL, Math.round(y) + 0.5); ctx.lineTo(f.W - f.padR, Math.round(y) + 0.5); ctx.stroke();
  ctx.restore();
}
function niceStep(range, target) {
  const raw = range / target, p = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 2.5, 5, 10]) if (m * p >= raw) return m * p;
  return 10 * p;
}

function drawScope() {
  const c = $("cvScope");
  const sm = SM;
  const sc = sm ? (scopeMode === "marker" ? sm.marker_scope : sm.scope) : null;
  if (!sc) {
    let msg = "Shows the SYNC pulses while recording with the DAQ on";
    if (sm && !sm.has_daq) msg = "This recording has no DAQ: no SYNC line to show";
    else if (sm && scopeMode === "marker") msg = sm.active ? "Waiting for the frame-0 marker…" : "Start marker was not captured";
    else if (S && S.state === "recording" && sm) msg = "Waiting for DAQ samples…";
    setText("smScopeHint", "");
    return placeholder(c, msg);
  }
  const f = plotFrame(c, 38, 10, 18, 20), { ctx } = f;
  const vmin = -0.6, vmax = 4.0;
  const Y = (v) => f.padT + f.ph * (1 - (v - vmin) / (vmax - vmin));
  const X = (ms) => f.padL + f.pw * (ms / sc.win_ms);
  // grid + axes
  ctx.fillStyle = css("--text-3"); ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (const v of [0, 1, 2, 3]) { hline(f, Y(v), null, css("--border")); ctx.fillText(v + " V", f.padL - 6, Y(v)); }
  const fr = sm.frame_ms;
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (let t = 0; t <= sc.win_ms + 1e-6; t += fr) {
    const x = X(t + (sc.edges.length ? sc.edges[0].t_ms % fr : 0));
    if (x > f.W - f.padR) break;
    ctx.save(); ctx.strokeStyle = css("--border"); ctx.globalAlpha = 0.6; ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(Math.round(x) + 0.5, f.padT); ctx.lineTo(Math.round(x) + 0.5, f.padT + f.ph); ctx.stroke(); ctx.restore();
  }
  for (let t = 0; t <= sc.win_ms + 1e-6; t += niceStep(sc.win_ms, 5)) ctx.fillText(t.toFixed(0) + " ms", X(t), f.H - f.padB + 5);
  // thresholds
  hline(f, Y(2.0), [5, 4], css("--warn"), 0.9);
  hline(f, Y(1.0), [5, 4], css("--warn"), 0.45);
  ctx.fillStyle = css("--warn"); ctx.textAlign = "right"; ctx.textBaseline = "bottom";
  ctx.fillText("rise 2.0 V", f.W - f.padR - 2, Y(2.0) - 2);
  // trace
  ctx.save(); ctx.beginPath(); ctx.rect(f.padL, f.padT - 4, f.pw, f.ph + 8); ctx.clip();
  ctx.strokeStyle = css("--accent"); ctx.lineWidth = 1.6; ctx.lineJoin = "round";
  ctx.beginPath();
  sc.v.forEach((v, i) => { const x = X(i * sc.dt_ms), y = Y(Math.max(vmin, Math.min(vmax, v))); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();
  // pulse labels
  ctx.textAlign = "center"; ctx.textBaseline = "bottom";
  for (const e of sc.edges) {
    const x = X(e.t_ms), x2 = X(e.t_ms + e.w_ms);
    const marker = e.k === 0 && e.w_ms >= 2;
    const col = e.k === null || e.k === undefined ? css("--text-3") : marker ? css("--warn") : css("--ok");
    ctx.fillStyle = col; ctx.globalAlpha = 0.14; ctx.fillRect(x, Y(3.6), Math.max(2, x2 - x), Y(-0.3) - Y(3.6)); ctx.globalAlpha = 1;
    ctx.strokeStyle = col; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(Math.round(x) + 0.5, Y(3.7)); ctx.lineTo(Math.round(x) + 0.5, Y(-0.4)); ctx.stroke();
    ctx.font = "600 11px " + css("--mono");
    const lab = e.k === null || e.k === undefined ? "?" : marker ? `frame 0 · ${e.w_ms.toFixed(1)} ms` : `#${e.k}`;
    ctx.fillText(lab, Math.min(Math.max(x, f.padL + 30), f.W - f.padR - 30), Y(3.75));
  }
  ctx.restore();
  setText("smScopeHint", scopeMode === "marker" ? "first 5 frames of the recording" : sc.triggered ? "triggered on a pulse" : "free-running");
}

function drawStrip() {
  const c = $("cvStrip");
  const [ctx, W, H] = fitCanvas(c);
  ctx.clearRect(0, 0, W, H);
  const sm = SM;
  if (!sm || !sm.strip || !sm.has_daq) return;
  const n = 200, s = sm.strip, cw = W / n, top = 0, h = H - 14;
  const cols = [css("--ok"), css("--bad"), css("--warn"), css("--border-strong")];
  for (let i = 0; i < s.length; i++) {
    ctx.fillStyle = cols[+s[i]];
    ctx.fillRect(i * cw, top, Math.max(1, cw - (cw > 3 ? 1 : 0)), h);
  }
  ctx.fillStyle = css("--text-3"); ctx.font = "10.5px " + css("--mono"); ctx.textBaseline = "bottom";
  ctx.textAlign = "left"; ctx.fillText(`frame ${sm.strip_first}`, 0, H);
  ctx.textAlign = "right"; ctx.fillText(`frame ${sm.strip_first + s.length - 1}`, Math.min(W, s.length * cw), H);
}

function drawResid() {
  const c = $("cvResid");
  const sm = SM;
  if (!sm || !sm.has_daq || !sm.resid || sm.resid.length < 3) {
    setText("smDaqStat", "");
    return placeholder(c, sm && !sm.has_daq ? "No DAQ in this recording" : "Fills in as frames are matched to pulses");
  }
  const f = plotFrame(c, 50, 10, 10, 20), { ctx } = f;
  const pts = sm.resid, half = sm.sample_us / 2;
  let lim = Math.max(half * 2.2, ...pts.map((p) => Math.abs(p[1]))) * 1.1;
  lim = niceStep(lim, 1) * Math.ceil(lim / niceStep(lim, 1));
  const span = Math.max(1, -pts[0][0]);
  const X = (t) => f.padL + f.pw * (1 + t / span), Y = (v) => f.padT + f.ph * (1 - (v + lim) / (2 * lim));
  ctx.fillStyle = css("--ok"); ctx.globalAlpha = 0.12;
  ctx.fillRect(f.padL, Y(half), f.pw, Y(-half) - Y(half)); ctx.globalAlpha = 1;
  const st = niceStep(2 * lim, 4);
  ctx.fillStyle = css("--text-3"); ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (let v = -Math.floor(lim / st) * st; v <= lim + 1e-9; v += st) {
    hline(f, Y(v), null, css("--border"), v === 0 ? 1 : 0.6);
    ctx.fillText(`${v.toFixed(0)} µs`, f.padL - 6, Y(v));
  }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  const ts = niceStep(span, 5);
  for (let t = 0; t <= span + 1e-9; t += ts) ctx.fillText(t === 0 ? "now" : `-${t.toFixed(t < 1 ? 1 : 0)}s`, X(-t), f.H - f.padB + 5);
  ctx.fillStyle = css("--accent");
  for (const [t, v] of pts) { ctx.beginPath(); ctx.arc(X(t), Y(Math.max(-lim, Math.min(lim, v))), 1.7, 0, Math.PI * 2); ctx.fill(); }
  setText("smDaqStat", `SD ${sm.resid_sd_us} µs · max ${sm.resid_max_us} µs` + (sm.drift_ppm !== null ? ` · drift ${sm.drift_ppm.toFixed(1)} ppm` : ""));
}

function drawClock() {
  const c = $("cvClock");
  const cp = CP;
  if (!cp || cp.t.length < 2) { setText("smPcStat", ""); return placeholder(c, "Waiting for pings from the board…"); }
  const f = plotFrame(c, 50, 10, 10, 20), { ctx } = f;
  const n = cp.t.length;
  let lim = 0;
  for (let i = 0; i < n; i++) lim = Math.max(lim, Math.abs(cp.off_ms[i]) + cp.rtt_ms[i] / 2);
  lim = Math.min(Math.max(lim * 1.1, 0.5), 10);
  const span = Math.max(5, -cp.t[0]);
  const X = (t) => f.padL + f.pw * (1 + t / span), Y = (v) => f.padT + f.ph * (1 - (Math.max(-lim, Math.min(lim, v)) + lim) / (2 * lim));
  const st = niceStep(2 * lim, 4);
  ctx.fillStyle = css("--text-3"); ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (let v = -Math.floor(lim / st) * st; v <= lim + 1e-9; v += st) {
    hline(f, Y(v), null, css("--border"), v === 0 ? 1 : 0.6);
    ctx.fillText(`${+v.toFixed(2)} ms`, f.padL - 6, Y(v));
  }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  const ts = niceStep(span, 5);
  for (let t = 0; t <= span + 1e-9; t += ts) ctx.fillText(t === 0 ? "now" : `-${t.toFixed(0)}s`, X(-t), f.H - f.padB + 5);
  const acc = css("--accent"), dim = css("--text-3");
  for (let i = 0; i < n; i++) {
    const x = X(cp.t[i]), o = cp.off_ms[i], h = cp.rtt_ms[i] / 2, used = cp.used[i];
    ctx.strokeStyle = used ? acc : dim; ctx.globalAlpha = used ? 0.55 : 0.35; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x, Y(o - h)); ctx.lineTo(x, Y(o + h)); ctx.stroke();
    ctx.globalAlpha = 1; ctx.lineWidth = 1.3;
    ctx.beginPath(); ctx.arc(x, Y(o), 2.6, 0, Math.PI * 2);
    if (used) { ctx.fillStyle = acc; ctx.fill(); } else { ctx.strokeStyle = dim; ctx.stroke(); }
  }
  const k = S && S.clock;
  setText("smPcStat", k && k.n ? `RTT ${k.rtt_ms_median.toFixed(2)} ms · ±${k.uncertainty_ms.toFixed(2)} ms` + (k.n >= 6 ? ` · drift ${k.drift_ppm.toFixed(1)} ppm` : "") : "");
}

function applySync(s) {
  SM = s.sync; CP = s.clockplot; smDirty = true;
  const sm = SM, st = s.state;
  const pill = $("smState");
  if (sm && sm.active && st === "recording") { pill.textContent = "LIVE"; pill.className = "pill rec"; }
  else if (sm) { pill.textContent = "LAST RECORDING"; pill.className = "pill"; }
  else { pill.textContent = "WAITING"; pill.className = "pill"; }
  const num = (id, v, cls) => { setText(id, v === null || v === undefined ? "—" : Number(v).toLocaleString()); $(id).className = cls || ""; };
  if (sm && sm.has_daq) {
    num("smFrames", sm.frames); num("smPulses", sm.pulses);
    num("smMatched", sm.matched, sm.matched ? "good" : "");
    num("smNoPulse", sm.no_pulse, sm.no_pulse ? "bad" : "good");
    num("smExtra", sm.extra, sm.extra ? "bad" : "good");
    num("smNoLine", sm.no_line, sm.no_line ? "bad" : "good");
    let lag = "";
    if (!sm.marker_found) lag = sm.frames > 0 ? "frame-0 marker not seen yet" : "";
    else if (sm.active) lag = `DAQ data trails the board by ${Math.max(0, sm.lag_frames)} frames (normal: read in 50 ms blocks)`;
    setText("smLag", lag);
    const good = sm.marker_found && !sm.no_pulse && !sm.extra;
    const sofar = sm.active ? " so far" : "";
    setText("smHint", sm.marker_found ? (good ? `every frame has its pulse${sofar}` : `some frames are missing a pulse${sofar}`) : "");
    $("smHint").className = "foldsum hint " + (sm.marker_found ? (good ? "good" : "bad") : "");
  } else {
    for (const id of ["smFrames", "smPulses", "smMatched", "smNoPulse", "smExtra", "smNoLine"]) num(id, null);
    setText("smLag", ""); setText("smHint", sm && !sm.has_daq ? "last recording had no DAQ" : "start a recording with the DAQ on");
    $("smHint").className = "foldsum hint";
  }
}

// ---------------- EMG view ----------------
let EMG = null, emgDirty = false, emgScale = +LS.get("emgScale", 0), emgKey = "";
const emgViews = new Map();      // chan -> view

function fmtV(v) { return Math.abs(v) >= 1 ? `${+v.toFixed(2)} V` : `${+(v * 1e3).toFixed(v < 0.01 ? 2 : 1)} mV`; }

function buildEmg(chans) {
  const grid = $("emgGrid");
  grid.innerHTML = ""; emgViews.clear();
  for (const ch of chans) {
    const node = document.createElement("div");
    node.className = "emgtr";
    node.innerHTML = `<div class="emghead"><b></b><span class="side"></span><span class="mus"></span>` +
      `<span class="cl hidden">CLIPPING</span><span class="rms">—</span></div><canvas class="emgcv"></canvas>`;
    node.querySelector("b").textContent = ch.name;
    const side = node.querySelector(".side");
    if (ch.side) side.textContent = ch.side; else side.remove();
    node.querySelector(".mus").textContent = ch.muscle || "";
    node.title = `${ch.name} · ${ch.chan}${ch.muscle ? " · " + ch.muscle : ""}${ch.side ? " (" + ch.side + ")" : ""}${ch.group ? " · " + ch.group : ""}`;
    grid.appendChild(node);
    emgViews.set(ch.chan, { node, cv: node.querySelector("canvas"), rms: node.querySelector(".rms"), cl: node.querySelector(".cl"), lim: 0 });
  }
}

function emgEmpty(text) {
  if (emgKey !== "") { emgKey = ""; emgViews.clear(); }
  const grid = $("emgGrid");
  if (!grid.firstElementChild || !grid.firstElementChild.classList.contains("emgempty") || grid.textContent !== text)
    grid.innerHTML = `<div class="emgempty"></div>`, grid.firstElementChild.textContent = text;
}

function applyEmg(s) {
  const daq = s.daq || {}, cfg = daq.cfg || {}, rec = s.state === "recording";
  EMG = s.emg;
  const pill = $("emgState"), btn = $("btnPreview");
  btn.classList.toggle("on", !!daq.preview_on);
  btn.textContent = daq.preview_on ? "Preview on" : "Preview";
  if (rec && EMG) { pill.textContent = "RECORDING"; pill.className = "pill rec"; }
  else if (daq.preview && EMG) { pill.textContent = "PREVIEW"; pill.className = "pill ok"; }
  else { pill.textContent = "OFF"; pill.className = "pill"; }
  if (!EMG || !EMG.chans || !EMG.chans.length) {
    const devs = (daq.devices && daq.devices.devices) || [];
    let msg = "Starting the DAQ…";
    if (!String(cfg.extra_chans || "").trim()) msg = "No EMG channels set (Time sync card → Extra channels).";
    else if (!devs.length) msg = "DAQ not found: EMG needs the NI DAQ.";
    else if (!cfg.enabled) msg = "DAQ recording is off.";
    else if (!rec && !daq.preview_on) msg = "Press Preview to see the electrodes before recording (nothing is saved).";
    else if (!rec && daq.preview_error) msg = "DAQ preview error: " + daq.preview_error;
    emgEmpty(msg);
    setText("emgHint", "");
    return;
  }
  const key = EMG.chans.map((c) => c.chan).join(",");
  if (key !== emgKey) { emgKey = key; buildEmg(EMG.chans); }
  const nclip = EMG.chans.filter((c) => c.clip).length;
  setText("emgHint", `${EMG.chans.length} channels · ±${EMG.range_v} V input` + (nclip ? ` · ${nclip} clipping` : ""));
  $("emgHint").className = "foldsum hint " + (nclip ? "bad" : "");
  for (const ch of EMG.chans) {
    const v = emgViews.get(ch.chan);
    if (!v) continue;
    v.data = ch;
    const r = ch.rms_mv === null ? "—" : `RMS ${ch.rms_mv < 1 ? ch.rms_mv.toFixed(3) : ch.rms_mv < 100 ? ch.rms_mv.toFixed(2) : ch.rms_mv.toFixed(0)} mV`;
    if (v.rms.textContent !== r) v.rms.textContent = r;
    v.cl.classList.toggle("hidden", !ch.clip);
    v.node.classList.toggle("clip", !!ch.clip);
  }
  emgDirty = true;
}

function drawEmg() {
  for (const v of emgViews.values()) {
    const ch = v.data;
    if (!ch) continue;
    const [ctx, W, H] = fitCanvas(v.cv);
    ctx.clearRect(0, 0, W, H);
    const n = ch.hi.length;
    let peak = 0;
    for (let i = 0; i < n; i++) peak = Math.max(peak, Math.abs(ch.hi[i]), Math.abs(ch.lo[i]));
    if (emgScale) v.lim = emgScale;
    else v.lim = Math.max(peak * 1.15, 0.0005, v.lim * 0.92);    // auto: grows at once, shrinks slowly
    const lim = v.lim, padL = 2, pw = W - padL;
    const Y = (x) => H / 2 - (Math.max(-lim, Math.min(lim, x)) / lim) * (H / 2 - 2);
    ctx.strokeStyle = css("--border"); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, Math.round(H / 2) + 0.5); ctx.lineTo(W, Math.round(H / 2) + 0.5); ctx.stroke();
    if (n) {
      ctx.strokeStyle = ch.clip ? css("--bad") : css("--accent");
      ctx.lineWidth = Math.max(1, pw / n);
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const x = padL + (i + 0.5) * pw / n;
        let y1 = Y(ch.hi[i]), y2 = Y(ch.lo[i]);
        if (y2 - y1 < 1) { y1 -= 0.5; y2 += 0.5; }
        ctx.moveTo(x, y1); ctx.lineTo(x, y2);
      }
      ctx.stroke();
    }
    ctx.fillStyle = css("--text-3"); ctx.font = "10.5px " + css("--mono"); ctx.textAlign = "left"; ctx.textBaseline = "top";
    ctx.fillText(`±${fmtV(lim)}`, 4, 2);
  }
}

// ---------------- status tiles (DAQ / IMU / EMG / Sync) ----------------
function tile(id, level, word, detail, live) {
  const el = $(id);
  const cls = "stile " + (level || "") + (live ? " live" : "");
  if (el.className !== cls) el.className = cls;
  const b = el.querySelector("b"), d = el.querySelector(".sd");
  if (b.textContent !== word) b.textContent = word;
  if (d.textContent !== detail) { d.textContent = detail; d.title = detail; }
}

function applyTiles(s) {
  const st = s.state, rec = st === "recording";
  const daq = s.daq || {}, cfg = daq.cfg || {}, devs = (daq.devices && daq.devices.devices) || [];
  const dev = devs.find((x) => x.name === cfg.device);
  const lv = rec ? daq.live : daq.preview;           // DAQ levels: recording task or idle preview
  const daqOk = !!(dev && cfg.enabled && lv && !lv.error);
  // DAQ
  if (daq.devices && !daq.devices.available) tile("stDaq", "bad", "Unavailable", "NI-DAQmx not usable on this PC");
  else if (!devs.length) tile("stDaq", "bad", "Not found", "Check USB + power, then Refresh");
  else if (!cfg.enabled) tile("stDaq", "", "Off", "Recordings are IMU-only");
  else if (lv && lv.error) tile("stDaq", "bad", "Error", lv.error);
  else if (!rec && daq.preview_error) tile("stDaq", "bad", "Error", daq.preview_error);
  else if (lv) tile("stDaq", "ok", rec ? "Recording" : "Connected",
    `${dev ? dev.product : cfg.device} · ${(+cfg.rate / 1000).toFixed(0)} kHz`, rec);
  else tile("stDaq", "warn", "Starting…", `${cfg.device}`);
  // IMU
  const pods = s.pods || [];
  const fus = s.hello && s.hello.frame_us ? +s.hello.frame_us : 10000;
  if (!s.port) tile("stImu", "bad", "Board offline", "Plug in the XIAO (USB)");
  else if (!s.hello || !s.hello.board) tile("stImu", "warn", "Connecting…", s.port);
  else {
    const bad = pods.filter((p) => !p.ok || !p.live || (p.silent_s !== null && p.silent_s > 0.5));
    const weak = pods.filter((p) => p.ok && p.fresh_recent < 90);
    const okN = pods.length - bad.length;
    const hz = `${Math.round(1e6 / fus)} Hz`;
    if (bad.length) tile("stImu", "bad", `${okN} / ${pods.length} OK`,
      bad.map((p) => `${p.name} ${p.ok ? "stalled" : "not found"}`).join(" · "));
    else if (weak.length) tile("stImu", "warn", `${okN} / ${pods.length} OK`,
      weak.map((p) => `${p.name} dropping (${p.fresh_recent.toFixed(0)}%)`).join(" · "));
    else tile("stImu", "ok", `${okN} / ${pods.length} OK`, `${hz} · ${pods.map((p) => p.name).join(", ")}`, rec);
  }
  // EMG = DAQ extra channels
  const extras = String(cfg.extra_chans || "").split(",").map((x) => x.trim()).filter((x) => x && x !== cfg.sync_chan);
  if (!extras.length) tile("stEmg", "", "Not set", "Add DAQ channels, e.g. ai1,ai2");
  else if (!daqOk) tile("stEmg", "", "No DAQ", extras.join(", "));
  else if (!s.emg || !s.emg.chans || !s.emg.chans.length) tile("stEmg", "warn", "Waiting…", `${extras.length} channels`);
  else {
    const ch = s.emg.chans, rng = +s.emg.range_v || +cfg.range || 5;
    const flat = ch.filter((c) => c.rms_mv !== null && c.rms_mv < 0.2), clip = ch.filter((c) => c.clip);
    const names = (a) => a.map((c) => c.name).join(", ");
    const top = Math.max(...ch.map((c) => c.rms_mv || 0));
    if (clip.length) tile("stEmg", "bad", "Clipping", `${names(clip)} over ±${rng} V`, rec);
    else if (flat.length === ch.length) tile("stEmg", "warn", "No signal", "all channels flat: amplifier on?", rec);
    else if (flat.length) tile("stEmg", "warn", `Check ${flat.length} ch`, `flat: ${names(flat)}`, rec);
    else tile("stEmg", "ok", `${ch.length} ch live`, `${ch[0].name}–${ch[ch.length - 1].name} · max RMS ${top.toFixed(2)} mV`, rec);
  }
  // Sync
  const c = s.clock || {}, sm = s.sync, ls = s.last_summary;
  const pc = c.n ? `PC ±${c.uncertainty_ms.toFixed(1)} ms` : "";
  const last = ls && ls.daq_sync ? `last: ${ls.daq_sync.ok ? "PASS" : "FAIL"}` : "";
  if (rec && sm && sm.has_daq) {
    if (!sm.marker_found) tile("stSync", "warn", "Waiting for start pulse", "frame-0 marker not seen yet", true);
    else if (sm.no_pulse || sm.extra) tile("stSync", "bad", "Pulses missing",
      `${sm.no_pulse} frames without pulse · ${sm.extra} extra`, true);
    else tile("stSync", "ok", "Locked", `${sm.matched.toLocaleString()} frames matched` +
      (sm.resid_sd_us !== null ? ` · ±${sm.resid_sd_us} µs` : ""), true);
  } else if (rec) tile("stSync", "warn", "PC clock only", pc || "no DAQ in this recording", true);
  else if (!c.n) tile("stSync", "", "Waiting", "needs the board connected");
  else if (!daqOk) tile("stSync", "warn", "PC clock only", [pc, last].filter(Boolean).join(" · "));
  else if (lv.sync_hi > 1.0) tile("stSync", "warn", "Check SYNC wire", `ai0 not low while idle (${lv.sync_hi.toFixed(1)} V)`);
  else if (ls && ls.daq_sync && !ls.daq_sync.ok) tile("stSync", "bad", "Ready · last FAIL", pc + " · DAQ ready");
  else tile("stSync", "ok", "Ready", [pc, "DAQ ready", last].filter(Boolean).join(" · "));
}

function drawSync() { drawScope(); drawStrip(); drawResid(); drawClock(); }

// ---------------- collapsible sections + settings drawer ----------------
const FOLD_DEFAULT = { imu: true, emg: false, sync: false, log: false };
const isOpen = (name) => { const el = document.querySelector(`.fold[data-fold="${name}"]`); return !!el && el.classList.contains("open"); };

function setFold(el, open) {
  el.classList.toggle("open", open);
  el.querySelector(".foldhead").setAttribute("aria-expanded", String(open));
  LS.set("fold." + el.dataset.fold, open);
  if (open) {                      // canvases had no size while closed: redraw
    smDirty = true; emgDirty = true;
    for (const p of pods.values()) p.dirty = true;
  }
}

function wireFolds() {
  for (const el of document.querySelectorAll(".fold")) {
    setFold(el, !!LS.get("fold." + el.dataset.fold, FOLD_DEFAULT[el.dataset.fold]));
    const head = el.querySelector(".foldhead");
    head.addEventListener("click", (e) => {
      if (e.target.closest(".foldtools")) return;          // buttons inside the header
      setFold(el, !el.classList.contains("open"));
    });
    head.addEventListener("keydown", (e) => {
      if ((e.key === "Enter" || e.key === " ") && e.target === head) { e.preventDefault(); setFold(el, !el.classList.contains("open")); }
    });
  }
}

function openDrawer(open) {
  $("drawer").classList.toggle("open", open);
  $("drawer").setAttribute("aria-hidden", String(!open));
  $("scrim").classList.toggle("hidden", !open);
  if (open) { loadPorts(); $("btnCloseDrawer").focus(); }
}

function goTo(id) {
  if (id === "drawer") return openDrawer(true);
  const el = $(id);
  if (!el) return;
  if (el.classList.contains("fold")) setFold(el, true);
  el.scrollIntoView({ behavior: "smooth", block: "start" });
}

function imuSummary(s) {
  if (!s.port) return "board offline";
  return (s.pods || []).map((pd) => {
    const p = pods.get(pd.name);
    const e = p ? euler(p.ref ? qmul(qconj(p.ref), p.q) : p.q) : null;
    const st = !pd.ok ? "not found" : (!pd.live || (pd.silent_s !== null && pd.silent_s > 0.5)) ? "stalled" : "";
    return st ? `${pd.name} ${st}` : e ? `${pd.name}: roll ${e[0].toFixed(0)}°  pitch ${e[1].toFixed(0)}°  yaw ${e[2].toFixed(0)}°` : pd.name;
  }).join("   ·   ");
}

let lastNum = 0;
function frameLoop(ts) {
  if (smDirty && isOpen("sync")) { smDirty = false; drawSync(); }
  if (emgDirty && isOpen("emg")) { emgDirty = false; drawEmg(); }
  const imuOpen = isOpen("imu");
  for (const p of pods.values()) {
    if (!p.dirty || !imuOpen) continue;
    p.dirty = false;
    drawPlot(p);
    drawCube(p);
  }
  if (ts - lastNum > 100) {
    lastNum = ts;
    for (const p of pods.values()) {
      const e = euler(p.ref ? qmul(qconj(p.ref), p.q) : p.q);
      p.roll.textContent = e[0].toFixed(1) + "°";
      p.pitch.textContent = e[1].toFixed(1) + "°";
      p.yaw.textContent = e[2].toFixed(1) + "°";
      p.quat.textContent = `q = [${p.q.map((v) => v.toFixed(4)).join(", ")}]` + (p.ref ? "  (display zeroed)" : "");
    }
    if (S && !imuOpen) setText("sumImu", imuSummary(S));
    else if (S && imuOpen) setText("sumImu", "");
    tickTimer();
  }
  requestAnimationFrame(frameLoop);
}

// ---------------- status ----------------
const fmtTime = (s) => {
  const m = Math.floor(s / 60), r = s - m * 60;
  return `${String(m).padStart(2, "0")}:${r.toFixed(1).padStart(4, "0")}`;
};
let recT0 = null;
function tickTimer() {
  if (recT0 !== null) $("timer").textContent = fmtTime((performance.now() - recT0) / 1000);
}

function setCls(el, cls, on) { el.classList.toggle(cls, !!on); }
function setText(id, v) { const el = $(id); if (el.textContent !== String(v)) el.textContent = v; }

function applyStatus(s) {
  const prevState = S ? S.state : null;
  S = s;
  const st = s.state;
  // connection chip
  const conn = $("connChip"), dot = $("connDot");
  dot.className = "dot " + (s.port && st !== "connecting" ? "on" : st === "connecting" ? "wait" : "");
  if (s.port && s.hello && s.hello.board) {
    setText("connText", `${s.port} · ${s.hello.board.replace("_", " ")}`);
  } else if (st === "connecting") setText("connText", "Connecting…");
  else setText("connText", "No board");
  conn.className = "chip " + (s.port && s.hello && s.hello.board ? "" : "warn");
  // state badge
  const badge = $("stateBadge");
  const map = { idle: ["READY", "idle"], recording: ["REC", "rec"], starting: ["STARTING", "busy"],
    stopping: ["SAVING", "busy"], connecting: ["CONNECTING", "busy"], disconnected: ["OFFLINE", ""] };
  const [lab, cls] = map[st] || [st.toUpperCase(), ""];
  badge.textContent = lab; badge.className = "badge " + cls;
  document.title = (st === "recording" ? "● REC — " : "") + "IMU DAQ";
  // timer
  if (st === "recording" && s.rec) {
    const t = performance.now() - s.rec.elapsed * 1000;
    if (recT0 === null || Math.abs(recT0 - t) > 500) recT0 = t;
    $("timer").classList.add("rec");
  } else if (recT0 !== null) {
    recT0 = null; $("timer").classList.remove("rec");
  }
  if (st !== "recording" && recT0 === null && prevState !== st) $("timer").textContent = "00:00.0";
  // rate chip
  const fus = s.hello && s.hello.frame_us ? +s.hello.frame_us : 10000;
  const target = 1e6 / fus;
  setText("rateChip", s.port ? `${s.rate_hz.toFixed(1)} Hz` : "— Hz");
  $("rateChip").className = "chip mono " + (!s.port ? "" : Math.abs(s.rate_hz - target) < target * 0.03 ? "good" : "warn");
  // clock chip + time sync card
  const c = s.clock || {};
  if (c.n) {
    setText("clockChip", `clock ±${c.uncertainty_ms.toFixed(2)} ms`);
    $("clockChip").className = "chip mono " + (c.uncertainty_ms < 2 ? "good" : "warn");
    setText("csRtt", `${c.rtt_ms_median.toFixed(2)} ms`);
    setText("csUnc", `±${c.uncertainty_ms.toFixed(2)} ms`);
    setText("csDrift", c.n >= 6 ? `${c.drift_ppm.toFixed(1)} ppm` : "measuring…");
    setText("csRes", `${c.resid_ms.toFixed(3)} ms`);
  } else {
    setText("clockChip", "clock —"); $("clockChip").className = "chip mono";
    for (const id of ["csRtt", "csUnc", "csDrift", "csRes"]) setText(id, "—");
  }
  // DAQ
  applyDaq(s.daq, st);
  // recording card
  const rec = s.rec;
  setText("recFrames", rec ? rec.frames.toLocaleString() : "—");
  setText("recMissing", rec ? rec.missing : "—");
  $("recMissing").className = rec && rec.missing ? "bad" : rec ? "good" : "";
  setText("recMarkers", rec ? rec.markers : "—");
  setText("recName", rec ? rec.name : (s.last_summary ? "last: " + s.last_summary.session : "—"));
  const idle = st === "idle", recording = st === "recording";
  $("btnStart").disabled = !idle;
  $("btnStop").disabled = !recording;
  for (const id of ["portSel", "btnConnect", "btnDisconnect", "rateSel", "btnRestart", "daqEnabled", "daqDevice",
    "daqSync", "daqRate", "daqTerm", "daqRange", "daqExtra", "btnDaqRefresh"])
    $(id).disabled = !(idle || st === "disconnected" || st === "connecting") ||
      (["rateSel", "btnRestart"].includes(id) && !idle);
  // device card
  const h = s.hello || {};
  setText("fwInfo", h.fw ? `${h.fw} ${h.ver}` : "—");
  setText("fwPins", h.sda ? `SDA ${h.sda} · SCL ${h.scl} · SYNC ${h.sync}` : "—");
  const b = s.board || {};
  setText("fwDropped", b.dropped_f !== undefined ? `${b.dropped_f} rec` : "—");
  $("fwDropped").className = +b.dropped_f ? "bad" : "";
  setText("fwResync", b.late_resyncs !== undefined ? b.late_resyncs : "—");
  if (h.report_ms && document.activeElement !== $("rateSel")) $("rateSel").value = h.report_ms;
  // pods
  for (const pd of s.pods) {
    const p = podFor(pd.name);
    p.addr.textContent = pd.addr;
    let cls = "", word = "NOT FOUND";
    if (pd.ok) {
      if (!pd.live || (pd.silent_s !== null && pd.silent_s > 0.5)) { cls = "bad"; word = "STALLED"; }
      else if (pd.fresh_recent < 90) { cls = "warn"; word = "DROPPING"; }
      else { cls = "ok"; word = "OK"; }
    } else if (st === "disconnected") { word = "OFFLINE"; }
    p.light.className = "light " + cls;
    p.state.className = "pill podstate " + cls;
    p.state.textContent = word;
    const fresh = pd.rec_fresh !== null && st === "recording" ? pd.rec_fresh : pd.fresh_recent;
    p.fresh.textContent = pd.ok ? fresh.toFixed(1) + "%" : "—";
    p.stalls.textContent = pd.stalls;
    setCls(p.node, "dim", !pd.ok);
  }
  applySync(s);
  applyTiles(s);
  applyEmg(s);
  // summary banner after a recording
  if (prevState === "stopping" && st !== "stopping" && s.last_summary) { showSummary(s.last_summary); loadSessions(true); }
}

function applyDaq(d, st) {
  if (!d) return;
  const cfg = d.cfg, devs = (d.devices && d.devices.devices) || [];
  const sel = $("daqDevice");
  const want = devs.map((x) => x.name).join(",");
  if (sel.dataset.list !== want) {
    sel.dataset.list = want;
    sel.innerHTML = devs.length ? devs.map((x) => `<option value="${x.name}">${x.name} · ${x.product}${x.simulated ? " (sim)" : ""}</option>`).join("")
      : `<option value="">No device found</option>`;
  }
  if (document.activeElement !== sel) sel.value = cfg.device || "";
  if (!$("daqEnabled").matches(":focus")) $("daqEnabled").checked = !!cfg.enabled;
  for (const [id, key] of [["daqSync", "sync_chan"], ["daqRate", "rate"], ["daqTerm", "terminal"], ["daqRange", "range"], ["daqExtra", "extra_chans"]]) {
    const el = $(id);
    if (document.activeElement !== el) el.value = String(key === "rate" || key === "range" ? +cfg[key] : cfg[key]);
  }
  const note = $("daqNote");
  const chip = $("daqChip");
  if (!d.devices.available) {
    note.textContent = d.devices.error || "NI-DAQmx not available"; note.className = "note bad";
    chip.textContent = "DAQ unavailable"; chip.className = "chip mono bad";
  } else if (!devs.length) {
    note.textContent = "No NI-DAQmx device found. Plug in the DAQ (check NI MAX), then Refresh. Recordings run IMU-only.";
    note.className = "note bad"; chip.textContent = "DAQ not found"; chip.className = "chip mono warn";
  } else if (!cfg.enabled) {
    note.textContent = "DAQ recording off: IMU-only recordings (PC clock sync only)."; note.className = "note";
    chip.textContent = "DAQ off"; chip.className = "chip mono";
  } else {
    const dev = devs.find((x) => x.name === cfg.device);
    note.textContent = dev ? `${dev.product}: SYNC on ${cfg.sync_chan} (${cfg.terminal}), ${(+cfg.rate / 1000).toFixed(0)} kHz. Starts before the board, checked at stop.` : "Select a device.";
    note.className = "note good";
    chip.textContent = `DAQ ${cfg.device}`; chip.className = "chip mono good";
  }
  const live = d.live;
  setText("daqPulses", live ? `${live.pulses.toLocaleString()} / ${S && S.rec ? S.rec.frames.toLocaleString() : "—"}` : "—");
  const lvl = live || d.preview;
  setText("daqLevel", lvl ? `${lvl.last_v.toFixed(2)} V` : "—");
  if (live && live.error) { note.textContent = live.error; note.className = "note bad"; }
}

function showSummary(sm) {
  const b = $("banner");
  const pods = Object.entries(sm.pods || {}).map(([k, v]) => `${k} ${v.fresh_pct.toFixed(1)}%`).join(" · ");
  let cls = "", sync = "";
  if (sm.daq_sync) {
    sync = sm.daq_sync.ok ? ` · SYNC PASS (${sm.daq_sync.pulses_after_marker} pulses = frames)` : ` · SYNC FAIL: ${sm.daq_sync.reason}`;
    if (!sm.daq_sync.ok) cls = "bad";
  }
  if (sm.missing_frames || sm.aborted) cls = cls || "warn";
  b.className = "banner " + cls;
  b.innerHTML = "";
  const span = document.createElement("span");
  span.textContent = `Saved ${sm.session}: ${sm.frames_received} frames, ${sm.missing_frames} missing, ${sm.duration_s ?? 0} s, ` +
    `${sm.rate_hz ?? "—"} Hz · ${pods}${sync}` + (sm.aborted ? ` · ${sm.aborted}` : "");
  const open = document.createElement("button");
  open.className = "btn small ghost"; open.textContent = "Open folder";
  open.onclick = () => api("/api/open", { dir: sm.dir });
  const close = document.createElement("button");
  close.className = "btn small ghost"; close.textContent = "×"; close.style.marginLeft = "0";
  close.onclick = () => b.classList.add("hidden");
  b.append(span, open, close);
}

// ---------------- log + sessions ----------------
function addLog(items) {
  const box = $("log");
  const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 30;
  for (const it of items) {
    const row = document.createElement("div");
    row.className = "e " + it.level;
    const t = new Date(it.t * 1000);
    const ts = t.toTimeString().slice(0, 8);
    row.innerHTML = `<span class="t"></span><span class="s"></span><span class="m"></span>`;
    row.children[0].textContent = ts;
    row.children[1].textContent = it.src;
    row.children[2].textContent = it.text;
    box.appendChild(row);
    if (it.level === "error") toast(it.text, true);
  }
  while (box.children.length > 500) box.removeChild(box.firstChild);
  if (atBottom) box.scrollTop = box.scrollHeight;
  if (items.length) {                       // one-line summary for the closed section
    const it = items[items.length - 1];
    const sum = $("sumLog");
    sum.textContent = `${new Date(it.t * 1000).toTimeString().slice(0, 8)}  ${it.text}`;
    sum.className = "foldsum hint " + (it.level === "error" ? "bad" : "");
  }
}

// ---------------- saved recordings panel ----------------
const FILE_DESC = {
  "imu_synced.csv": "IMU + PC & DAQ time (main file)", "imu.csv": "raw IMU frames", "events.csv": "start/stop, markers, warnings",
  "summary.json": "quality report", "session.json": "subject, trial, settings", "clock_sync.csv": "PC clock pings",
  "daq_raw.f32": "DAQ samples (SYNC + EMG)", "daq_meta.json": "DAQ channels & rate", "daq_pulses.csv": "SYNC pulses",
  "spine_cal.json": "spine calibration",
};
const FILE_ORDER = Object.keys(FILE_DESC);
let SESS = [], openSess = new Set(LS.get("openSess", [])), newestSess = null;

const fmtBytes = (b) => b >= 1 << 30 ? (b / (1 << 30)).toFixed(1) + " GB" : b >= 1 << 20 ? (b / (1 << 20)).toFixed(1) + " MB" : b >= 1024 ? Math.round(b / 1024) + " kB" : b + " B";
function fmtWhen(s) {                     // "20261113_020556_S02_2" -> "11-13 02:05"
  const m = /^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})/.exec(s.name);
  return m ? `${m[2]}-${m[3]} ${m[4]}:${m[5]}` : (s.started || "");
}

async function loadSessions(highlight) {
  try {
    const r = await (await fetch("/api/sessions")).json();
    SESS = r.sessions || [];
    if (highlight && SESS.length) { newestSess = SESS[0].name; openSess.add(newestSess); }
    renderSessions();
  } catch { }
}

function renderSessions() {
  const box = $("sessions"), q = $("filesSearch").value.trim().toLowerCase();
  const list = SESS.filter((s) => !q || `${s.name} ${s.subject} ${s.trial} ${s.notes}`.toLowerCase().includes(q));
  box.innerHTML = "";
  setText("filesCount", SESS.length ? `${SESS.length}` : "");
  const total = SESS.reduce((a, s) => a + (s.bytes || 0), 0);
  setText("filesFoot", SESS.length ? `${fmtBytes(total)} in the latest ${SESS.length} · click a recording for its files, click a file to open it` : "");
  if (!list.length) { box.innerHTML = `<div class="empty">${SESS.length ? "No match." : "No recordings yet."}</div>`; return; }
  for (const s of list) {
    const el = document.createElement("div");
    el.className = "sess2" + (openSess.has(s.name) ? " open" : "") + (s.name === newestSess ? " new" : "");
    let tag = `<span class="pill tag">IMU only</span>`;
    if (s.sync === true) tag = `<span class="pill ok tag">SYNC ✓</span>`;
    else if (s.sync === false) tag = `<span class="pill bad tag">SYNC ✗</span>`;
    if (s.aborted) tag = `<span class="pill warn tag">ABORTED</span>`;
    el.innerHTML = `<div class="sh" title="Show files"><span class="who"></span>${tag}<span class="when"></span><span class="meta"></span></div><div class="fl"></div>`;
    const who = [s.subject, s.trial].filter(Boolean).join(" · ") || s.name;
    el.querySelector(".who").textContent = who;
    el.querySelector(".when").textContent = fmtWhen(s);
    const miss = s.missing ? `${s.missing} missing · ` : "";
    el.querySelector(".meta").textContent = `${s.duration_s ?? 0} s · ${(s.frames ?? 0).toLocaleString()} frames · ${miss}${fmtBytes(s.bytes || 0)}`;
    el.querySelector(".sh").onclick = () => {
      el.classList.toggle("open");
      if (el.classList.contains("open")) openSess.add(s.name); else openSess.delete(s.name);
      LS.set("openSess", [...openSess].slice(-20));
    };
    const fl = el.querySelector(".fl");
    if (s.notes) { const n = document.createElement("div"); n.className = "notes"; n.textContent = s.notes; fl.appendChild(n); }
    const files = (s.files || []).slice().sort((a, b) => {
      const ia = FILE_ORDER.indexOf(a.name), ib = FILE_ORDER.indexOf(b.name);
      return (ia < 0 ? 99 : ia) - (ib < 0 ? 99 : ib) || a.name.localeCompare(b.name);
    });
    for (const f of files) {
      const row = document.createElement("div");
      row.className = "f" + (f.name === "imu_synced.csv" ? " main" : "");
      row.title = `Open ${f.name} (Shift+click: show in Explorer)`;
      row.innerHTML = `<span class="nm"></span><span class="ds"></span><span class="sz"></span>`;
      row.querySelector(".nm").textContent = f.name;
      row.querySelector(".ds").textContent = FILE_DESC[f.name] || "";
      row.querySelector(".sz").textContent = fmtBytes(f.bytes);
      row.onclick = (e) => api("/api/open", { file: s.dir + "\\" + f.name, reveal: e.shiftKey });
      fl.appendChild(row);
    }
    const acts = document.createElement("div");
    acts.className = "acts";
    const ob = document.createElement("button");
    ob.className = "btn small ghost"; ob.textContent = "Open folder";
    ob.onclick = () => api("/api/open", { dir: s.dir || ((S && S.data_dir ? S.data_dir + "\\" : "") + s.name) });
    const cp = document.createElement("button");
    cp.className = "btn small ghost"; cp.textContent = "Copy path";
    cp.onclick = async () => { try { await navigator.clipboard.writeText(s.dir); toast("Path copied"); } catch { toast(s.dir); } };
    acts.append(ob, cp);
    fl.appendChild(acts);
    box.appendChild(el);
  }
}

// ---------------- actions ----------------
let toastTimer = null;
function toast(msg, bad) {
  const t = $("toast");
  t.textContent = msg; t.className = "toast" + (bad ? " bad" : "");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add("hidden"), bad ? 5000 : 2500);
}

async function api(path, body) {
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
    const j = await r.json();
    if (!j.ok) toast(j.error || "failed", true);
    return j;
  } catch (e) { toast("Server not running: double-click “Start IMU DAQ.bat” on the Desktop", true); return { ok: false }; }
}

function saveForm() { for (const id of ["subject", "label", "notes"]) LS.set("f." + id, $(id).value); }

async function startRec() {
  saveForm();
  $("btnStart").disabled = true;
  const j = await api("/api/start", { subject: $("subject").value, label: $("label").value, notes: $("notes").value });
  if (j.ok) { toast("Recording " + j.session); $("banner").classList.add("hidden"); }
}
async function stopRec() {
  $("btnStop").disabled = true;
  await api("/api/stop");
}
async function mark() {
  const txt = $("markText").value.trim();
  const j = await api("/api/marker", { text: txt });
  if (j.ok) { toast(S && S.state === "recording" ? `Marker${txt ? ": " + txt : ""}` : "Marker logged (not recording)"); $("markText").value = ""; }
}

async function loadPorts() {
  try {
    const r = await (await fetch("/api/ports")).json();
    const sel = $("portSel"), cur = S && S.want_port;
    sel.innerHTML = `<option value="">Auto (${r.guess || "none"})</option>` +
      r.ports.map((p) => `<option value="${p.device}">${p.device} · ${p.espressif ? "XIAO" : p.description}</option>`).join("");
    sel.value = cur || "";
  } catch { }
}

function daqCfgFromForm() {
  return { enabled: $("daqEnabled").checked, device: $("daqDevice").value, sync_chan: $("daqSync").value.trim() || "ai0",
    rate: +$("daqRate").value, terminal: $("daqTerm").value, range: +$("daqRange").value, extra_chans: $("daqExtra").value.trim() };
}

function wire() {
  for (const id of ["subject", "label", "notes"]) { $(id).value = LS.get("f." + id, ""); $(id).addEventListener("change", saveForm); }
  $("btnStart").onclick = startRec;
  $("btnStop").onclick = stopRec;
  $("btnMark").onclick = mark;
  $("markText").addEventListener("keydown", (e) => { if (e.key === "Enter") mark(); });
  $("btnConnect").onclick = () => api("/api/connect", { port: $("portSel").value || null });
  $("btnDisconnect").onclick = () => api("/api/disconnect");
  $("portSel").addEventListener("focus", loadPorts);
  $("rateSel").onchange = () => api("/api/rate", { ms: +$("rateSel").value });
  $("btnRestart").onclick = () => api("/api/restart");
  for (const id of ["daqEnabled", "daqDevice", "daqSync", "daqRate", "daqTerm", "daqRange", "daqExtra"])
    $(id).addEventListener("change", () => api("/api/daq", daqCfgFromForm()));
  $("btnDaqRefresh").onclick = async () => {
    const r = await (await fetch("/api/daq/devices")).json();
    toast(r.devices.length ? `Found ${r.devices.map((d) => d.name + " " + d.product).join(", ")}` : (r.error || "No NI-DAQmx device found"), !r.devices.length);
  };
  $("btnZero").onclick = () => zeroAll(true);
  $("btnUnzero").onclick = () => zeroAll(false);
  $("btnOpenData").onclick = () => api("/api/open", {});
  $("btnRefreshFiles").onclick = () => loadSessions();
  $("filesSearch").addEventListener("input", renderSessions);
  for (const b of $("winSeg").querySelectorAll("button")) {
    b.classList.toggle("on", +b.dataset.w === windowSec);
    b.onclick = () => {
      windowSec = +b.dataset.w; LS.set("window", windowSec);
      for (const x of $("winSeg").querySelectorAll("button")) x.classList.toggle("on", x === b);
      for (const p of pods.values()) p.dirty = true;
    };
  }
  for (const b of $("logSeg").querySelectorAll("button")) {
    b.onclick = () => {
      lastLogLevelFilter = b.dataset.l;
      for (const x of $("logSeg").querySelectorAll("button")) x.classList.toggle("on", x === b);
      $("log").classList.toggle("onlywarn", lastLogLevelFilter === "warn");
    };
  }
  document.addEventListener("keydown", onShortcut);
  document.addEventListener("keyup", (e) => {          // stop Space from also "clicking" a focused button
    if (e.code === "Space" && !isTyping(e.target)) e.preventDefault();
  });
  $("keysHelp").onclick = () => showKeys(false);
  $("btnKeys").onclick = () => showKeys(true);
  window.addEventListener("resize", () => { for (const p of pods.values()) p.dirty = true; smDirty = true; });
  $("btnPreview").onclick = () => api("/api/preview", { on: !(S && S.daq && S.daq.preview_on) });
  for (const b of $("emgScale").querySelectorAll("button")) {
    b.classList.toggle("on", +b.dataset.s === emgScale);
    b.onclick = () => {
      emgScale = +b.dataset.s; LS.set("emgScale", emgScale);
      for (const x of $("emgScale").querySelectorAll("button")) x.classList.toggle("on", x === b);
      for (const v of emgViews.values()) v.lim = 0;
      emgDirty = true;
    };
  }
  for (const t of document.querySelectorAll(".stile")) t.onclick = () => goTo(t.dataset.go);
  wireFolds();
  $("btnSettings").onclick = () => openDrawer(!$("drawer").classList.contains("open"));
  $("btnCloseDrawer").onclick = () => openDrawer(false);
  $("scrim").onclick = () => openDrawer(false);
  document.addEventListener("keydown", (e) => { if (e.key === "Escape" && $("drawer").classList.contains("open")) openDrawer(false); });
  for (const b of $("scopeSeg").querySelectorAll("button")) {
    b.classList.toggle("on", b.dataset.m === scopeMode);
    b.onclick = () => {
      scopeMode = b.dataset.m; LS.set("scopeMode", scopeMode);
      for (const x of $("scopeSeg").querySelectorAll("button")) x.classList.toggle("on", x === b);
      smDirty = true;
    };
  }
}

// ---------------- keyboard shortcuts ----------------
const FOLD_KEYS = { "1": "imu", "2": "emg", "3": "sync", "4": "log" };
let lastToggle = 0;

function isTyping(el) {
  const tag = (el && el.tagName || "").toLowerCase();
  return tag === "input" || tag === "textarea" || tag === "select" || !!(el && el.isContentEditable);
}

function toggleRecording() {
  const now = performance.now();
  if (now - lastToggle < 1000) return;                  // ignore an accidental double press
  const st = S && S.state;
  if (st === "idle") { lastToggle = now; startRec(); }
  else if (st === "recording") { lastToggle = now; stopRec(); }
  else toast(st ? `Not now: ${st}` : "Server not connected", true);
}

function showKeys(on) {
  const el = $("keysHelp");
  el.classList.toggle("hidden", on === undefined ? !el.classList.contains("hidden") : !on);
}

function onShortcut(e) {
  if (isTyping(e.target)) {                              // Esc leaves a text field, so keys work again
    if (e.key === "Escape") e.target.blur();
    return;
  }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.code === "Space") {
    e.preventDefault();
    if (e.repeat) return;
    if (document.activeElement && document.activeElement !== document.body) document.activeElement.blur();
    toggleRecording();
    return;
  }
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  if (k === "m") { e.preventDefault(); mark(); }
  else if (k === "z") { e.preventDefault(); zeroAll(true); }
  else if (k === "s") { e.preventDefault(); openDrawer(!$("drawer").classList.contains("open")); }
  else if (FOLD_KEYS[k]) {
    e.preventDefault();
    const el = document.querySelector(`.fold[data-fold="${FOLD_KEYS[k]}"]`);
    if (el) setFold(el, !el.classList.contains("open"));
  } else if (e.key === "?" || k === "h") { e.preventDefault(); showKeys(); }
  else if (e.key === "Escape") showKeys(false);
}

// ---------------- stream ----------------
let serverDown = false;
function connectStream() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    if (serverDown) { serverDown = false; $("banner").classList.add("hidden"); toast("Server connected"); }
    const m = JSON.parse(ev.data);
    if (m.s) applyStatus(m.s);
    if (m.f && m.f.length) addFrames(m.f);
    if (m.log) addLog(m.log);
  };
  es.onerror = () => {
    $("connDot").className = "dot"; setText("connText", "Server offline");
    $("stateBadge").textContent = "NO SERVER"; $("stateBadge").className = "badge";
    if (!serverDown) {
      serverDown = true;
      const b = $("banner");
      b.className = "banner bad";
      b.textContent = "The IMU DAQ server is not running. Double-click “Start IMU DAQ.bat” on the Desktop " +
        "(or check its black window for an error). This page reconnects by itself.";
    }
    for (const id of ["btnStart", "btnStop", "btnMark"]) $(id).disabled = true;
  };
}

wire();
loadPorts();
loadSessions();
connectStream();
requestAnimationFrame(frameLoop);
