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
    headacc: node.querySelector(".headacc"),
    cube: node.querySelector(".cube"), plot: node.querySelector(".plot"),
    roll: node.querySelector(".roll b"), pitch: node.querySelector(".pitch b"), yaw: node.querySelector(".yaw b"),
    quat: node.querySelector(".quat"),
    sens: node.querySelector(".sens"),
    sAcc: node.querySelector(".sacc b"), sGyr: node.querySelector(".sgyr b"), sMag: node.querySelector(".smag b"),
    rAcc: node.querySelector(".sacc small"), rGyr: node.querySelector(".sgyr small"), rMag: node.querySelector(".smag small"),
    t: [], e: [], s: [], q: [1, 0, 0, 0], x: null, magAcc: null, ref: LS.get("ref." + name, null), dirty: true,
    cnt: { t0: null, acc: 0, gyr: 0, mag: 0 }, hz: null,
  };
  pods.set(name, p);
  return p;
}

function podFor(name) { return pods.get(name) || makePod(name); }

function addFrames(rows) {
  if (!S || !S.hello || !S.hello.pods) return;
  const names = S.hello.pods.split(",").map((s) => s.split(":")[0]);
  const n = names.length;
  for (const r of rows) {
    const t = r[0] / 1e6;
    const hasSens = r.length >= 3 + 19 * n;          // firmware 1.2: acc, gyro, mag after the quaternions
    for (let k = 0; k < n; k++) {
      const o = 3 + 6 * k;
      if (r.length < o + 6) break;
      const p = podFor(names[k]);
      const q = [r[o + 2], r[o + 3], r[o + 4], r[o + 5]];
      if (p.t.length && t < p.t[p.t.length - 1] - 1) { p.t = []; p.e = []; p.s = []; p.cnt.t0 = null; }   // board restarted
      p.q = q;
      if (bend && r[o] && bend.samples[names[k]]) bend.samples[names[k]].push(q);
      p.t.push(t);
      p.e.push(euler(p.ref ? qmul(qconj(p.ref), q) : q));
      if (hasSens) {
        const b = 3 + 6 * n + 13 * k;
        const x = r.slice(b + 3, b + 12);            // ax ay az [m/s^2], gx gy gz [rad/s], mx my mz [uT]
        p.x = x; p.magAcc = r[b + 12];
        p.s.push(x);
        const c = p.cnt;                              // live update rate of each sensor (per ~1 s)
        if (c.t0 === null) { c.t0 = t; c.acc = c.gyr = c.mag = 0; }
        c.acc += r[b] ? 1 : 0; c.gyr += r[b + 1] ? 1 : 0; c.mag += r[b + 2] ? 1 : 0;
        if (t - c.t0 >= 1) {
          const d = t - c.t0;
          p.hz = { acc: c.acc / d, gyr: c.gyr / d, mag: c.mag / d };
          c.t0 = t; c.acc = c.gyr = c.mag = 0;
        }
      } else {
        p.s.push(null); p.x = null; p.hz = null;
      }
      p.dirty = true;
    }
  }
  for (const p of pods.values()) {
    if (!p.t.length) continue;
    const cut = p.t[p.t.length - 1] - MAXKEEP;
    let i = 0;
    while (i < p.t.length && p.t[i] < cut) i++;
    if (i > 200) { p.t.splice(0, i); p.e.splice(0, i); p.s.splice(0, i); }
  }
}

// ---------------- plot modes (angles / acc / gyro / mag) ----------------
const RAD2DEG = 180 / Math.PI;
const PLOT_MODES = {
  ang: { label: "Roll / pitch / yaw", unit: "°" },
  acc: { label: "Acceleration (incl. gravity)", unit: "m/s²", off: 0, scale: 1, min: 12 },
  gyr: { label: "Angular velocity", unit: "°/s", off: 3, scale: RAD2DEG, min: 60 },
  mag: { label: "Magnetic field", unit: "µT", off: 6, scale: 1, min: 60 },
};
let plotMode = LS.get("plotMode", "ang");
if (!PLOT_MODES[plotMode]) plotMode = "ang";

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
  if (plotMode !== "ang") return drawSensorPlot(p, PLOT_MODES[plotMode]);
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
    p.t = p.t.slice(-1); p.s = p.s.slice(-1); p.e = [euler(p.ref ? qmul(qconj(p.ref), p.q) : p.q)];
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

// x / y / z of one sensor over the time window, symmetric auto scale
function drawSensorPlot(p, m) {
  const [ctx, W, H] = fitCanvas(p.plot);
  ctx.clearRect(0, 0, W, H);
  const padL = 42, padR = 8, padT = 6, padB = 18;
  const pw = W - padL - padR, ph = H - padT - padB;
  ctx.font = "11px " + css("--mono");
  const n = p.t.length;
  if (!n || !p.s.some((v) => v)) {
    ctx.fillStyle = css("--text-3"); ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(n ? "No sensor data: upload firmware 1.2" : "Waiting for data…", W / 2, H / 2);
    return;
  }
  const tEnd = p.t[n - 1], t0 = tEnd - windowSec;
  let i0 = n - 1;
  while (i0 > 0 && p.t[i0 - 1] >= t0) i0--;
  let peak = 0;
  for (let i = i0; i < n; i++) {
    const v = p.s[i];
    if (v) for (let j = 0; j < 3; j++) peak = Math.max(peak, Math.abs(v[m.off + j] * m.scale));
  }
  const lim0 = Math.max(m.min, peak * 1.1);
  const st = niceStep(2 * lim0, 4), lim = Math.ceil(lim0 / st) * st;
  const Y = (v) => padT + ph * (1 - (v + lim) / (2 * lim));
  ctx.strokeStyle = css("--border"); ctx.fillStyle = css("--text-3"); ctx.lineWidth = 1;
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (let v = -lim; v <= lim + 1e-9; v += st) {
    ctx.globalAlpha = Math.abs(v) < 1e-9 ? 1 : 0.6;
    ctx.beginPath(); ctx.moveTo(padL, Math.round(Y(v)) + 0.5); ctx.lineTo(W - padR, Math.round(Y(v)) + 0.5); ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.fillText(String(+v.toFixed(1)), padL - 6, Y(v));
  }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  const step = windowSec <= 5 ? 1 : windowSec <= 10 ? 2 : 5;
  for (let s = 0; s <= windowSec; s += step) ctx.fillText(s === 0 ? "now" : `-${s}s`, padL + pw * (1 - s / windowSec), H - padB + 4);
  const cols = [css("--roll"), css("--pitch"), css("--yaw")];
  ctx.save();
  ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
  ctx.lineWidth = 1.5; ctx.lineJoin = "round";
  for (let j = 0; j < 3; j++) {
    ctx.strokeStyle = cols[j];
    ctx.beginPath();
    let started = false;
    for (let i = i0; i < n; i++) {
      const v = p.s[i];
      if (!v) { started = false; continue; }
      const x = padL + pw * (1 - (tEnd - p.t[i]) / windowSec), y = Y(v[m.off + j] * m.scale);
      if (started) ctx.lineTo(x, y); else { ctx.moveTo(x, y); started = true; }
    }
    ctx.stroke();
  }
  ctx.restore();
  // legend + unit, top-left inside the plot
  ctx.textAlign = "left"; ctx.textBaseline = "top"; ctx.font = "600 10.5px " + css("--mono");
  let lx = padL + 6;
  for (let j = 0; j < 3; j++) { ctx.fillStyle = cols[j]; ctx.fillText("xyz"[j], lx, padT + 3); lx += 12; }
  ctx.fillStyle = css("--text-3"); ctx.font = "10.5px " + css("--mono");
  ctx.fillText(m.unit, lx + 4, padT + 3);
}

function updateSensorText(p) {
  if (!p.x) { p.sens.classList.add("hidden"); return; }
  p.sens.classList.remove("hidden");
  const x = p.x, mag3 = (a, b, c) => Math.hypot(a, b, c);
  const a = mag3(x[0], x[1], x[2]), g = mag3(x[3], x[4], x[5]) * RAD2DEG, mg = mag3(x[6], x[7], x[8]);
  p.sAcc.textContent = `${a.toFixed(2)} m/s²`;
  p.sGyr.textContent = `${g.toFixed(1)} °/s`;
  p.sMag.textContent = `${mg.toFixed(1)} µT`;
  const hz = (v) => (p.hz ? `${Math.round(v)} Hz` : "— Hz");
  p.rAcc.textContent = hz(p.hz && p.hz.acc);
  p.rGyr.textContent = hz(p.hz && p.hz.gyr);
  const cal = p.magAcc === null || p.magAcc === undefined ? "" : ` · cal ${p.magAcc}/3`;
  p.rMag.textContent = `${hz(p.hz && p.hz.mag)}${cal}`;
  p.sMag.parentElement.classList.toggle("warn", p.magAcc !== null && p.magAcc < 2);
  p.sMag.parentElement.title = p.magAcc !== null && p.magAcc < 2
    ? "Magnetometer not calibrated yet: move the IMU slowly in a figure-8 until cal reaches 2–3" : "Magnetometer";
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
    v.cl.textContent = ch.floating ? "NOT CONNECTED" : "CLIPPING";
    v.cl.title = ch.floating ? `This input copies the SYNC pulses (r = ${ch.sync_corr}): nothing is connected to it` : "";
    v.cl.classList.toggle("hidden", !(ch.clip || ch.floating));
    v.node.classList.toggle("clip", !!ch.clip && !ch.floating);
    v.node.classList.toggle("floating", !!ch.floating);
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
    else if (quatOf(s) === "rv" && pods.some((p) => p.q_status !== null && p.q_status < 2))
      tile("stImu", "warn", "Calibrate mag", "move each IMU in slow figure-8s", rec);
    else tile("stImu", "ok", `${okN} / ${pods.length} OK`, `${hz} · ${QUAT_SHORT[quatOf(s)] || ""}`, rec);
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
    const floating = ch.filter((c) => c.floating);
    if (floating.length === ch.length) tile("stEmg", "bad", "Not connected", "all inputs copy SYNC: connect the EMG amplifier", rec);
    else if (floating.length) tile("stEmg", "bad", `${floating.length} not connected`, names(floating), rec);
    else if (clip.length) tile("stEmg", "bad", "Clipping", `${names(clip)} over ±${rng} V`, rec);
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

// ---------------- steps 1-4: minimizable (like the Live signals sections) ----------------
// Header click (or Enter) minimizes a step to its header plus a one-line summary. Space stays "record".
const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
function stepHead(n) { const el = $("step" + n); return n === 1 ? el : el.querySelector("h2"); }
function setStep(n, min) {
  const el = $("step" + n), head = stepHead(n);
  el.classList.toggle("min", min);
  head.setAttribute("aria-expanded", String(!min));
  LS.set("stepmin." + n, min);
  if (!min) { emgDirty = true; for (const p of pods.values()) p.dirty = true; }
  updateStepSums();
}
function wireSteps() {
  for (const n of [1, 2, 3, 4]) {
    const head = stepHead(n);
    const chev = document.createElement("span");
    chev.className = "chev stepchev"; chev.textContent = "›";
    head.insertBefore(chev, head.firstChild);
    const sum = document.createElement("span");
    sum.className = "stepsum"; sum.id = "stepSum" + n;
    head.appendChild(sum);
    head.classList.add("stephdr");
    head.tabIndex = 0; head.setAttribute("role", "button");
    head.title = "Click to minimize / expand this step";
    head.addEventListener("click", (e) => { if (!e.target.closest("button, a, input, select")) setStep(n, !$("step" + n).classList.contains("min")); });
    head.addEventListener("keydown", (e) => { if (e.key === "Enter" && e.target === head) { e.preventDefault(); setStep(n, !$("step" + n).classList.contains("min")); } });
  }
  for (const n of [1, 2, 3, 4]) setStep(n, !!LS.get("stepmin." + n, false));   // after all headers exist
}
const pillMark = (id) => { const el = $(id); if (!el) return ["", ""];
  const c = el.className; return c.includes(" ok") ? ["✓", "good"] : c.includes("warn") ? ["!", "warn"] : c.includes("bad") ? ["✗", "bad"] : ["✗", "bad"]; };
function updateStepSums() {
  if (![1, 2, 3, 4].every((n) => $("stepSum" + n))) return;
  // 1: device tiles
  $("stepSum1").innerHTML = ["stDaq", "stImu", "stEmg", "stSync"].map((id) => {
    const t = $(id), c = /\bok\b/.test(t.className) ? "good" : /\bbad\b/.test(t.className) ? "bad" : /\bwarn\b/.test(t.className) ? "warn" : "";
    return `<span class="${c}">${esc(t.querySelector("small").textContent)} ${esc(t.querySelector("b").textContent)}</span>`;
  }).join("<i>·</i>");
  // 2: participant
  const subj = $("subject").value.trim(), h = $("heightCm") ? $("heightCm").value : "";
  $("stepSum2").innerHTML = [subj ? `<b>${esc(subj)}</b>` : `<span class="warn">no subject</span>`, esc(dateId($("testDate").value)),
    h ? esc(h) + " cm" : "", esc($("notes").value.trim())].filter(Boolean).join("<i>·</i>");
  // 3: the calibration parts
  const parts = [["Still hold", "calPill"], ["Forward bend", "bendPill"]];
  if ($("emgNormStep") && !$("emgNormStep").classList.contains("hidden")) parts.push(["EMG norm", "emgNormPill"]);
  $("stepSum3").innerHTML = parts.map(([l, id]) => { const [m, c] = pillMark(id); return `<span class="${c}">${l} ${m}</span>`; }).join("<i>·</i>");
  // Setup group: steps 1-3 on one line
  if ($("setupSum")) {
    $("setupSum").innerHTML = [1, 2, 3].map((n) => $("stepSum" + n).innerHTML).join('<i class="sep">|</i>');
    const done = [1, 2, 3].filter((n) => $("step" + n).classList.contains("done")).length;
    $("setupPill").textContent = done === 3 ? "READY" : `${done} / 3 DONE`;
    $("setupPill").className = "pill " + (done === 3 ? "ok" : "warn");
  }
  // 4: next trial
  const sel = $("taskSel"), task = sel.selectedIndex >= 0 && sel.value ? sel.options[sel.selectedIndex].text : "";
  const rec = S && S.state === "recording";
  $("stepSum4").innerHTML = (rec ? `<span class="bad">● REC</span><i>·</i>` : "") +
    (task ? `<b>${esc(task)}</b><i>·</i>trial ${esc($("label").value || "1")}<i>·</i><span class="mono">${esc($("takePath").textContent.replace(/^→\s*/, ""))}</span>`
      : `<span class="warn">no task selected</span>`);
}

// Setup group (steps 1-3): minimize as a whole while recording; one summary line when minimized.
function setSetup(min) {
  $("setupGroup").classList.toggle("min", min);
  $("setupHead").setAttribute("aria-expanded", String(!min));
  LS.set("setupmin", min);
  if (!min) { emgDirty = true; for (const p of pods.values()) p.dirty = true; }
  updateStepSums();
}
function wireSetup() {
  const head = $("setupHead");
  head.addEventListener("click", (e) => { if (!e.target.closest("button, a, input, select")) setSetup(!$("setupGroup").classList.contains("min")); });
  head.addEventListener("keydown", (e) => { if (e.key === "Enter" && e.target === head) { e.preventDefault(); setSetup(!$("setupGroup").classList.contains("min")); } });
  setSetup(!!LS.get("setupmin", false));
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
      updateSensorText(p);
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
  if (recT0 !== null) {
    const sec = (performance.now() - recT0) / 1000;
    $("timer").textContent = fmtTime(sec);
    $("tpTc").textContent = fmtTc(sec);
  }
}
const fmtTc = (s) => {                       // hh:mm:ss.mmm for the transport bar
  const ms = Math.max(0, Math.round(s * 1000)), h = Math.floor(ms / 3600000), m = Math.floor(ms / 60000) % 60;
  const sec = Math.floor(ms / 1000) % 60, r = ms % 1000;
  return `${String(h).padStart(2, "0")}:${String(m).padStart(2, "0")}:${String(sec).padStart(2, "0")}.${String(r).padStart(3, "0")}`;
};

// ---------------- trial naming: <date>/<subNN>/<subNN>_<task>_trialNN ----------------
let TASKS = [];
const todayId = () => { const d = new Date(); return `${String(d.getFullYear()).slice(2)}${String(d.getMonth() + 1).padStart(2, "0")}${String(d.getDate()).padStart(2, "0")}`; };
function dateId(v) { let d = String(v || "").replace(/\D/g, ""); if (d.length === 8) d = d.slice(2); return d.length === 6 ? d : todayId(); }
function subjId(v) {                                   // same rule as the server: 1 / S01 / sub1 -> sub01
  const t = String(v || "").trim(), m = t.match(/^(?:s|sub|subj|subject)?[\s_-]*0*(\d+)$/i);
  if (m) return "sub" + String(+m[1]).padStart(2, "0");
  return t.replace(/[^A-Za-z0-9_\-]+/g, "_").replace(/^_+|_+$/g, "").toLowerCase().slice(0, 40) || "";
}
function takeName() {
  const sub = subjId($("subject").value), task = $("taskSel").value, n = Math.max(1, parseInt($("label").value, 10) || 1);
  return sub && task ? `${sub}_${task}_trial${String(n).padStart(2, "0")}` : "";
}
function takePath() { const sub = subjId($("subject").value), t = takeName(); return t ? `${dateId($("testDate").value)}/${sub}/${t}` : ""; }
function applyTaskInfo() {
  const t = TASKS.find((x) => x.key === $("taskSel").value);
  $("taskInfo").textContent = t ? `${t.method} · 시작 자세: ${t.start}` : "";
  const path = takePath();
  $("takePath").textContent = path ? "→ " + path : "Enter the subject and pick a task";
  $("takePath").classList.toggle("warn", !path);
  if (S) applyTransport(S);
}
let nextTrialTimer = null;
function refreshNextTrial() {                           // first free trial number for this date / subject / task
  clearTimeout(nextTrialTimer);
  nextTrialTimer = setTimeout(async () => {
    const sub = $("subject").value.trim(), task = $("taskSel").value;
    if (!sub || !task) { applyTaskInfo(); return; }
    try {
      const q = new URLSearchParams({ date: dateId($("testDate").value), subject: sub, task });
      const j = await (await fetch("/api/next_trial?" + q)).json();
      if (j.ok && document.activeElement !== $("label")) { $("label").value = j.trial; saveForm(); }
    } catch { }
    applyTaskInfo();
  }, 250);
}
async function loadTasks() {
  try { TASKS = await (await fetch("/static/tasks.json", { cache: "no-store" })).json(); } catch { TASKS = []; }
  const sel = $("taskSel"), want = LS.get("f.task", "");
  sel.innerHTML = `<option value="">— select task —</option>` + TASKS.map((t, i) => `<option value="${t.key}">${i + 1}. ${t.name}</option>`).join("");
  sel.value = TASKS.some((t) => t.key === want) ? want : "";
  refreshNextTrial();
}

// ---------------- transport bar ----------------
function applyTransport(s) {
  const st = s.state, rec = s.rec, recording = st === "recording";
  const mode = $("tpMode");
  const word = { idle: "LIVE", recording: "REC", starting: "STARTING", stopping: "SAVING", connecting: "CONNECTING", disconnected: "OFFLINE" }[st] || String(st).toUpperCase();
  mode.className = "tp-mode " + (recording ? "rec" : st === "idle" ? "idle" : st === "disconnected" ? "" : "busy");
  mode.lastChild.textContent = word;
  $("transport").classList.toggle("rec", recording);
  setText("tpTakeName", rec ? rec.name : (takeName() || "—"));
  if (rec) setText("tpFrames", String(rec.frames).padStart(6, "0"));
  else if (st === "idle" && !recording) setText("tpFrames", "000000");
  if (!recording && recT0 === null) setText("tpTc", "00:00:00.000");
  setText("tpMissing", rec ? rec.missing : "—");
  $("tpMissing").className = "mono" + (rec && rec.missing ? " bad" : "");
  setText("tpMarkers", rec ? rec.markers : "—");
  const btn = $("tpRec");
  btn.classList.toggle("on", recording);
  btn.disabled = !(st === "idle" || recording);
  btn.title = recording ? "Stop recording (Space)" : "Start recording (Space)";
  btn.setAttribute("aria-label", recording ? "Stop recording" : "Start recording");
  $("tpMark").disabled = !(st === "idle" || recording);
  // data: IMU rate · DAQ channels @ rate · approximate bytes written per second
  const parts = [];
  if (s.port && s.rate_hz) parts.push(`IMU ${Math.round(s.rate_hz)} Hz`);
  let bps = 0;
  const cfg = s.daq && s.daq.cfg;
  if (cfg && cfg.enabled && s.daq.devices && (s.daq.devices.devices || []).length) {
    const n = 1 + String(cfg.extra_chans || "").split(",").map((x) => x.trim()).filter((x) => x && x !== cfg.sync_chan).length;
    parts.push(`DAQ ${n} ch @ ${(+cfg.rate / 1000).toFixed(0)} kHz`);
    bps += n * +cfg.rate * 4;
  }
  if (s.rate_hz) bps += s.rate_hz * 60 * (s.pods ? s.pods.length : 1);   // rough CSV size per frame
  if (bps) parts.push(bps >= 1e6 ? `≈${(bps / 1e6).toFixed(1)} MB/s` : `≈${Math.round(bps / 1e3)} KB/s`);
  setText("tpData", parts.join(" · ") || "—");
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
  applyTransport(s);
  const idle = st === "idle", recording = st === "recording";
  $("btnStart").disabled = !idle;
  $("btnStop").disabled = !recording;
  for (const id of ["portSel", "btnConnect", "btnDisconnect", "rateSel", "btnRestart", "daqEnabled", "daqDevice",
    "daqSync", "daqRate", "daqTerm", "daqRange", "daqExtra", "btnDaqRefresh"])
    $(id).disabled = !(idle || st === "disconnected" || st === "connecting") ||
      (["rateSel", "btnRestart"].includes(id) && !idle);
  $("quatSel").disabled = !idle;
  // device card
  const h = s.hello || {};
  setText("fwInfo", h.fw ? `${h.fw} ${h.ver}` : "—");
  setText("fwPins", h.sda ? `SDA ${h.sda} · SCL ${h.scl} · SYNC ${h.sync}` : "—");
  const b = s.board || {};
  setText("fwDropped", b.dropped_f !== undefined ? `${b.dropped_f} rec` : "—");
  $("fwDropped").className = +b.dropped_f ? "bad" : "";
  setText("fwResync", b.late_resyncs !== undefined ? b.late_resyncs : "—");
  if (h.report_ms && document.activeElement !== $("rateSel")) $("rateSel").value = h.report_ms;
  applyQuatMode(s);
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
    const rv = quatOf(s) === "rv" && pd.ok;
    p.headacc.classList.toggle("hidden", !rv);
    if (rv) {
      const cal = pd.q_status, acc = pd.head_acc_deg;
      p.headacc.textContent = (acc !== null && acc !== undefined ? `heading ±${acc.toFixed(1)}°` : "heading —") +
        (cal !== null && cal !== undefined ? ` · mag ${cal}/3` : "");
      p.headacc.className = "pill headacc " + (cal === null || cal === undefined ? "" : cal >= 3 ? "ok" : cal === 2 ? "" : "warn");
    }
    setCls(p.node, "dim", !pd.ok);
  }
  applySync(s);
  applyTiles(s);
  applyEmg(s);
  applyCalib(s);
  applyFlow(s);
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

// ---------------- static calibration (step 3) ----------------
let calSecs = +LS.get("calSecs", 5), calApplied = LS.get("calApplied", ""), calRenderKey = "";

async function runCalibration() {
  if (!S || S.state !== "idle") { toast(S ? `Not now: ${S.state}` : "Server not connected", true); return; }
  saveForm();
  const j = await api("/api/calibrate", { seconds: calSecs, subject: $("subject").value.trim(), date: dateId($("testDate").value), notes: $("notes").value });
  if (j.ok) toast(`Calibration: hold still for ${calSecs} s`);
}

function calRow(cls, name, text, title) {
  const r = document.createElement("div");
  r.className = "r";
  r.innerHTML = `<b></b><span></span>`;
  r.firstChild.className = cls; r.firstChild.textContent = name;
  r.lastChild.textContent = text; r.title = title || text;
  return r;
}

// alert block (errors / warnings): icon + message, kept apart from the result table
function calAlert(cls, text, title) {
  const a = document.createElement("div");
  a.className = "calalert " + cls;
  a.innerHTML = `<i></i><span></span>`;
  a.firstChild.textContent = cls === "bad" ? "✗" : cls === "ok" ? "✓" : "!";
  a.lastChild.textContent = text; a.title = title || text;
  return a;
}

function renderCalResult(box, c, cur, att) {
  box.innerHTML = "";
  const showAtt = att && (!cur || att.session !== cur.session);      // a newer attempt that was not accepted
  if (c.error && !c.running) box.appendChild(calAlert("bad", c.error + " · repeat the calibration"));
  const res = showAtt && !c.error ? att : cur;
  if (!res) {
    if (!c.error) box.appendChild(calRow("", "", "Not calibrated yet. Recordings work without it, but angles have no neutral reference."));
    return;
  }
  // per-IMU table: status · name · moved · tilt · gyro bias · mag cal (columns only when there is data)
  const imus = Object.entries(res.imus || {});
  if (imus.length) {
    const vals = imus.map(([, v]) => v);
    const cols = [
      { h: "Moved", t: "Largest orientation change during the hold (limit 3°)",
        f: (v) => `${v.moved_max_deg.toFixed(1)}°`, bad: (v) => !v.ok && v.moved_max_deg > 3 },
      { h: "Tilt", t: "Sensor z axis vs vertical", on: vals.some((v) => v.tilt_z_from_vertical_deg != null),
        f: (v) => v.tilt_z_from_vertical_deg != null ? `${v.tilt_z_from_vertical_deg.toFixed(0)}°` : "—" },
      { h: "Gyro bias", t: "Mean gyro reading while still", on: vals.some((v) => v.gyro_bias_dps),
        f: (v) => v.gyro_bias_dps ? `${Math.hypot(...v.gyro_bias_dps).toFixed(2)} °/s` : "—" },
      { h: "Mag cal", t: "BNO085 magnetometer calibration (0-3)", on: vals.some((v) => v.mag_accuracy != null),
        f: (v) => v.mag_accuracy != null ? `${v.mag_accuracy}/3` : "—" },
    ].filter((col) => col.on !== false);
    const t = document.createElement("table");
    t.className = "caltab";
    const hr = t.createTHead().insertRow();
    for (const h of ["", "IMU", ...cols.map((col) => col.h)]) { const th = document.createElement("th"); th.textContent = h; hr.appendChild(th); }
    cols.forEach((col, i) => { hr.cells[i + 2].title = col.t; });
    const tb = t.createTBody();
    for (const [name, v] of imus) {
      const tr = tb.insertRow();
      tr.className = v.q0 && v.ok ? "ok" : "bad";
      const ic = tr.insertCell(); ic.className = "ic"; ic.textContent = v.q0 && v.ok ? "✓" : "✗";
      const nm = tr.insertCell(); nm.className = "nm"; nm.textContent = name;
      if (!v.q0) { const td = tr.insertCell(); td.colSpan = cols.length; td.className = "bad"; td.textContent = "no orientation data"; continue; }
      for (const col of cols) { const td = tr.insertCell(); td.textContent = col.f(v); if (col.bad && col.bad(v)) td.className = "bad"; }
    }
    box.appendChild(t);
  }
  const emg = Object.entries(res.emg || {}).filter(([, v]) => v && v.rms_mv != null);
  if (emg.length) {
    const top = emg.reduce((a, b) => (b[1].rms_mv > a[1].rms_mv ? b : a));
    const lo = Math.min(...emg.map(([, v]) => v.rms_mv));
    const high = top[1].rms_mv > 50;      // resting EMG should be well under 50 mV at the DAQ input
    const fl = emg.filter(([, v]) => v.floating).map(([n]) => n);
    box.appendChild(calRow(high ? "warn" : "ok", `${high ? "!" : "✓"} EMG rest`,
      `RMS ${lo.toFixed(2)}–${top[1].rms_mv.toFixed(2)} mV · max ${top[0]}`,
      emg.map(([n, v]) => `${n} ${v.rms_mv.toFixed(3)} mV`).join("\n") +
      (high ? "\nUnconnected DAQ inputs pick up the SYNC pulses while recording (multiplexer crosstalk)." : "")));
    if (fl.length) box.appendChild(calAlert("bad", `EMG not connected: ${fl.length === emg.length ? "all channels" : fl.join(", ")}`,
      "These inputs copy the SYNC pulses, which only happens when nothing is connected to them. Connect the EMG amplifier outputs (or remove unused channels in Settings) and calibrate again."));
    else if (high) box.appendChild(calAlert("warn", "EMG rest RMS too high: electrodes connected?"));
  }
  const m = document.createElement("div");
  m.className = "meta";
  m.textContent = `${(res.when || "").slice(11, 16)} · ${res.subject || "no subject"} · ${res.seconds || ""} s · saved as ${res.session}`;
  m.title = m.textContent;
  box.appendChild(m);
}

// The server holds the master spine calibration. This page keeps a copy in localStorage ("spineCal",
// shared with the Spine view and its embedded copy) and re-loads it whenever the server's revision changes.
let spineRev = null, spineSynced = false;
function writeSpineCal(cal) {
  const txt = JSON.stringify(cal);
  try { if (localStorage.getItem("spineCal") !== txt) localStorage.setItem("spineCal", txt); } catch { }
  const fr = $("spineFrame");                      // the embedded view gets no storage event from its parent
  if (fr && fr.dataset.on === "1" && fr.contentWindow) fr.contentWindow.postMessage({ type: "spinecal", cal }, location.origin);
}
async function syncSpineCal() {
  try {
    const j = await (await fetch("/api/spine_cal", { cache: "no-store" })).json();
    if (j.ok) { if (j.cal) writeSpineCal(j.cal); spineRev = j.rev; }
  } catch { }
  spineSynced = true;
  if (S) { applyCalib(S); applyFlow(S); }
}
async function postSpineCal(sc) {
  try {
    const r = await fetch("/api/spine_cal", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(sc) });
    const j = await r.json();
    if (j.ok && j.cal) { spineRev = j.rev; writeSpineCal(j.cal); if (S) applyFlow(S); }
  } catch { }
}

function applyCalToViews(cur) {
  // IMU display: the neutral pose becomes the zero of the angle readouts (recorded data unchanged)
  for (const p of pods.values()) {
    const v = cur.imus && cur.imus[p.name];
    if (!v || !v.q0) continue;
    p.ref = v.q0.slice(); LS.set("ref." + p.name, p.ref); p.e = [];
  }
  // Spine view: same neutral quaternions (keeps its sensor map and flexion axis)
  try {
    const sc = JSON.parse(localStorage.getItem("spineCal") || "null") || { q0: null, B: null, map: null, when: null };
    const q0 = {};
    for (const [n, v] of Object.entries(cur.imus || {})) if (v.q0) q0[n] = v.q0;
    if (sameQ0(q0, sc.q0)) return;                // already applied (e.g. another browser): keep its forward-bend axis
    sc.q0 = q0;
    sc.B = null; LS.set("bendRes", null);       // a new neutral pose: the flexion axis belongs to the old one, redo the forward bend
    const names = Object.keys(sc.q0);
    if (!sc.map && names.length >= 2) sc.map = { T7: names[0], T12: names[1] };
    sc.when = new Date().toISOString();
    localStorage.setItem("spineCal", JSON.stringify(sc));
    postSpineCal(sc);
    const fr = $("spineFrame");
    if (fr && fr.dataset.on === "1") fr.src = "/spine?embed=1&t=" + Date.now();
  } catch { }
}

function sameQ0(a, b) {
  if (!a || !b) return false;
  const ka = Object.keys(a), kb = Object.keys(b);
  return ka.length > 0 && ka.length === kb.length && ka.every((k) => b[k] && a[k].every((v, i) => Math.abs(v - b[k][i]) < 1e-5));
}

function applyCalib(s) {
  const c = s.calib || {}, cur = c.current, att = c.attempt;
  const subj = $("subject").value.trim();
  $("calProg").classList.toggle("hidden", !c.running);
  if (c.running && $("step3").classList.contains("min")) setStep(3, false);   // show the countdown
  if (c.running && $("setupGroup").classList.contains("min")) setSetup(false);
  if (c.running) {
    const f = c.seconds ? 1 - c.remaining_s / c.seconds : 0;
    $("calBar").style.width = `${Math.max(0, Math.min(100, f * 100)).toFixed(0)}%`;
    setText("calLeft", c.remaining_s > 0 ? `Hold still… ${c.remaining_s.toFixed(1)} s` : "Analysing…");
  }
  $("btnCal").disabled = !!c.running || s.state !== "idle";
  const pill = $("calPill");
  const mismatch = cur && subj && cur.subject && cur.subject !== subj;
  if (c.running) { pill.textContent = "HOLD STILL"; pill.className = "pill warn"; }
  else if (cur && !mismatch) { pill.textContent = "DONE " + (cur.when || "").slice(11, 16); pill.className = "pill ok"; }
  else if (mismatch) { pill.textContent = `FOR ${cur.subject}`; pill.className = "pill warn"; pill.title = "The saved calibration belongs to another subject: calibrate again"; }
  else { pill.textContent = "NOT DONE"; pill.className = "pill"; }
  const key = JSON.stringify([c.running, c.error, cur && cur.session, att && att.session]);
  if (key !== calRenderKey) { calRenderKey = key; renderCalResult($("calRes"), c, cur, att); }
  if (cur && cur.ok && cur.session !== calApplied && pods.size && spineSynced) {
    calApplied = cur.session; LS.set("calApplied", calApplied);
    applyCalToViews(cur);
    if (S && S.calib && S.calib.current === cur) toast("Calibrated: angle display and Spine view use this neutral pose");
  }
}

// ---------------- forward bend (step 3, second part) ----------------
// The static hold gives each sensor's neutral orientation q0. The forward bend gives the body's
// left-right (flexion) axis: the horizontal direction of the summed rotation vectors since neutral.
// Body frame B: X = that axis (left), Z = up (gravity), Y = Z x X (posterior). Same math as the Spine view.
const BEND_SECS = 6;
let bend = null;                                     // {until, samples: {name: [q]}}
function spineCal() { try { return JSON.parse(localStorage.getItem("spineCal") || "null"); } catch { return null; } }
function rotvecQ(q) {
  if (q[0] < 0) q = q.map((v) => -v);
  const s = Math.hypot(q[1], q[2], q[3]);
  if (s < 1e-9) return [0, 0, 0];
  const th = 2 * Math.atan2(s, q[0]);
  return [q[1] / s * th, q[2] / s * th, q[3] / s * th];
}
function qFromAxes(X, Y, Z) {                       // rotation whose matrix columns are X, Y, Z
  const [m00, m10, m20] = X, [m01, m11, m21] = Y, [m02, m12, m22] = Z, tr = m00 + m11 + m22;
  let w, x, y, z;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = s / 4; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = s / 4; y = (m01 + m10) / s; z = (m02 + m20) / s; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = s / 4; z = (m12 + m21) / s; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = s / 4; }
  const n = Math.hypot(w, x, y, z) || 1;
  return [w / n, x / n, y / n, z / n];
}

function bendAllowed() { return !!S && ["idle", "recording"].includes(S.state) && !(S.calib && S.calib.running); }

function runBend() {
  const sc = spineCal();
  if (!bendAllowed()) { toast(S ? `Not now: ${S.state}` : "Server not connected", true); return; }
  if (bend) return;
  if (!sc || !sc.q0 || Object.keys(sc.q0).length < 2) { toast("Do the static calibration first", true); return; }
  const names = sc.map ? [sc.map.T7, sc.map.T12] : Object.keys(sc.q0).slice(0, 2);
  if (!names.every((n) => sc.q0[n] && pods.has(n) && pods.get(n).q)) { toast("No live data from both IMUs", true); return; }
  bend = { until: performance.now() + BEND_SECS * 1000, samples: Object.fromEntries(names.map((n) => [n, []])) };
  if ($("step3").classList.contains("min")) setStep(3, false);
  if ($("setupGroup").classList.contains("min")) setSetup(false);
  $("bendProg").classList.remove("hidden");
  $("btnBend").disabled = true;
  applyBend();
  tickBend();
}

function tickBend() {
  if (!bend) return;
  const left = (bend.until - performance.now()) / 1000;
  $("bendBar").style.width = `${Math.max(0, Math.min(100, (1 - left / BEND_SECS) * 100)).toFixed(0)}%`;
  if (left > 0) { setText("bendLeft", `Bend forward and back… ${left.toFixed(1)} s`); setTimeout(tickBend, 100); return; }
  const b = bend; bend = null;
  $("bendProg").classList.add("hidden");
  finishBend(b);
}

function finishBend(b) {
  const sc = spineCal(), D = 180 / Math.PI;
  let ax = [0, 0, 0], peak = 0, n = 0;
  for (const [name, list] of Object.entries(b.samples)) {
    const q0 = sc.q0[name];
    for (const q of list) {
      n++;
      const r = rotvecQ(qmul(q, qconj(q0)));            // world-frame rotation since neutral
      const deg = Math.hypot(...r) * D;
      peak = Math.max(peak, deg);
      if (deg >= 10) ax = ax.map((v, i) => v + r[i]);   // ignore the standing phase and noise
    }
  }
  const h = Math.hypot(ax[0], ax[1]);
  let res;
  if (n < 40) res = { ok: false, text: "Too few IMU samples: check the IMUs and repeat." };
  else if (peak < 15 || h < 1e-6) res = { ok: false, text: `Only ${peak.toFixed(0)}° of bending seen: bend further (40° or more) and repeat.` };
  else {
    const tilt = Math.atan2(Math.abs(ax[2]), h) * D;
    const X = [ax[0] / h, ax[1] / h, 0], Z = [0, 0, 1], Y = [Z[1] * X[2] - Z[2] * X[1], Z[2] * X[0] - Z[0] * X[2], Z[0] * X[1] - Z[1] * X[0]];
    sc.B = qFromAxes(X, Y, Z);
    sc.when = new Date().toISOString();
    try { localStorage.setItem("spineCal", JSON.stringify(sc)); } catch { }
    postSpineCal(sc);
    const fr = $("spineFrame");
    if (fr && fr.dataset.on === "1") fr.src = "/spine?embed=1&t=" + Date.now();
    res = { ok: true, warn: tilt > 25,
      text: `peak bend ${peak.toFixed(0)}° · twist ${tilt.toFixed(0)}°` + (tilt > 25 ? " · too twisted: repeat straighter" : "") };
    toast("Forward bend saved: spine angles now use the body axes");
  }
  if (!res.ok) toast(res.text, true);
  res.when = new Date().toTimeString().slice(0, 5);
  LS.set("bendRes", res);
  applyBend();
  if (S) applyFlow(S);
}

// leaving the page: a forward bend is captured here, and Stop (Space) only exists on this page
window.addEventListener("beforeunload", (e) => {
  if (bend || (S && S.state === "recording")) { e.preventDefault(); e.returnValue = ""; }
});

function bendDone() { const sc = spineCal(); return !!(sc && sc.q0 && sc.B); }

function applyBend() {
  const done = bendDone(), res = LS.get("bendRes", null), pill = $("bendPill");
  if (bend) { pill.textContent = "BENDING"; pill.className = "pill warn"; }
  else if (done) { pill.textContent = "DONE" + (res && res.ok && res.when ? " " + res.when : ""); pill.className = "pill " + (res && res.warn ? "warn" : "ok"); }
  else { pill.textContent = "NOT DONE"; pill.className = "pill"; }
  const sc = spineCal(), haveNeutral = !!(sc && sc.q0);
  if (!bend) $("btnBend").disabled = !haveNeutral || !bendAllowed();
  const box = $("bendRes"), key = JSON.stringify([done, res, haveNeutral]);
  if (box.dataset.key === key) return;
  box.dataset.key = key; box.innerHTML = "";
  if (!haveNeutral) box.appendChild(calRow("", "", "Do the still hold above first."));
  else if (res && !res.ok) box.appendChild(calAlert("bad", res.text));
  else if (done && res) box.appendChild(res.warn ? calAlert("warn", res.text) : calRow("ok", "✓", res.text));
  else if (done) box.appendChild(calRow("ok", "✓", "flexion axis set (in the Spine view)"));
  else box.appendChild(calAlert("warn", "Needed after every still hold: without it, flexion, side bend and rotation are mixed."));
}

// ---------------- EMG normalization status (step 3c; measured on the /emg page) ----------------
let emgNormKey = "", emgNormRes = null, emgNormBusy = false;
async function checkEmgNorm(force) {
  const subj = $("subject").value.trim(), date = dateId($("testDate").value);
  const key = `${date}|${subj}`;
  if (!force && key === emgNormKey) { showEmgNorm(); return; }
  if (emgNormBusy) { emgNormKey = ""; return; }          // retry on the next status update
  emgNormKey = key; emgNormRes = null;
  if (subj) {
    emgNormBusy = true;
    try {
      const r = await fetch("/api/emgcal/load?" + new URLSearchParams({ date, subject: subj }));
      emgNormRes = r.ok ? { ok: true, norm: (await r.json()).norm || null } : { ok: false };
    } catch { emgNormRes = { ok: false }; }
    emgNormBusy = false;
  }
  showEmgNorm();
}
function showEmgNorm() {
  const step = $("emgNormStep"), pill = $("emgNormPill"), cap = $("emgNormCap"), btn = $("btnEmgNorm");
  const hasEmg = !!(S && S.emg && S.emg.chans && S.emg.chans.length);
  step.classList.toggle("hidden", !hasEmg);
  if (!hasEmg) return;
  const subj = $("subject").value.trim(), n = emgNormRes && emgNormRes.norm;
  const ch = n && n.channels ? Object.values(n.channels) : [];
  const withRef = ch.filter((c) => c.ref_mv), withRest = ch.filter((c) => c.noise_mv != null);
  const total = S.emg.chans.length;
  const done = withRef.length >= total && withRest.length >= total;
  btn.classList.toggle("ghost", done);
  btn.innerHTML = done ? "Open EMG norm page →" : "⚡ Measure EMG rest &amp; MVC / RVC →";
  if (!subj) { pill.textContent = "NOT DONE"; pill.className = "pill"; cap.textContent = "Enter the subject first, then measure the rest baseline and MVC / RVC on the EMG norm page."; }
  else if (emgNormRes && !emgNormRes.ok) { pill.textContent = "UNKNOWN"; pill.className = "pill"; cap.textContent = "Could not check (restart the server once to enable the EMG norm page)."; }
  else if (!n) { pill.textContent = "NOT DONE"; pill.className = "pill warn"; cap.textContent = `No rest baseline or MVC / RVC saved for ${subj} on this test date. Needed to express EMG as %MVC.`; }
  else {
    const kinds = {}; for (const c of withRef) kinds[c.ref_kind] = (kinds[c.ref_kind] || 0) + 1;
    const kt = Object.entries(kinds).map(([k, v]) => `${k} ${v}`).join(", ");
    pill.textContent = done ? "DONE " + (n.saved || "").slice(11, 16) : "PARTIAL";
    pill.className = "pill " + (done ? "ok" : "warn");
    cap.textContent = `Saved ${(n.saved || "").slice(0, 16)} · reference for ${withRef.length}/${total} channels${kt ? " (" + kt + ")" : ""}` +
      (withRest.length < total ? ` · rest baseline for ${withRest.length}/${total}` : "") + (done ? "" : " · finish it on the EMG norm page");
  }
}

// ---------------- experiment flow: which step is next ----------------
let flowPrevState = null;
function applyFlow(s) {
  updateStepSums();
  if (s.spine_cal_rev != null && spineRev != null && s.spine_cal_rev !== spineRev) { spineRev = s.spine_cal_rev; syncSpineCal(); }
  applyBend();
  checkEmgNorm();
  const tiles = ["stDaq", "stImu", "stEmg", "stSync"].map((id) => $(id).className);
  const done = {
    1: tiles.every((c) => !/\b(bad|warn)\b/.test(c)) && !!s.port,
    2: !!$("subject").value.trim(),
    3: !!(s.calib && s.calib.current && (!$("subject").value.trim() || !s.calib.current.subject ||
      s.calib.current.subject === $("subject").value.trim())) && bendDone(),
  };
  const active = !done[1] ? 1 : !done[2] ? 2 : !done[3] ? 3 : 4;
  for (const n of [1, 2, 3, 4, 5]) {
    const el = $("step" + n);
    if (!el) continue;
    el.classList.toggle("done", !!done[n]);
    el.classList.toggle("active", n === active && s.state !== "recording" ? true : n === 4 && s.state === "recording" && !(s.calib && s.calib.running));
  }
  // after a normal trial is saved: next trial number (when the trial name is a number)
  if (flowPrevState === "stopping" && s.state !== "stopping" && s.last_summary &&
      !String(s.last_summary.session || "").includes("static_cal")) {
    refreshNextTrial();
  }
  flowPrevState = s.state;
}

// ---------------- saved recordings panel ----------------
const FILE_DESC = {
  "imu_synced.csv": "IMU + PC & DAQ time (main file)", "imu.csv": "raw IMU frames", "events.csv": "start/stop, markers, warnings",
  "summary.json": "quality report", "session.json": "subject, trial, settings", "clock_sync.csv": "PC clock pings",
  "daq_raw.f32": "DAQ samples (SYNC + EMG)", "daq_meta.json": "DAQ channels & rate", "daq_pulses.csv": "SYNC pulses",
  "spine_cal.json": "spine calibration", "calibration.json": "static calibration used (neutral pose)",
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
  const list = SESS.filter((s) => !q || `${s.rel || s.name} ${s.subject} ${s.trial} ${s.task} ${s.notes}`.toLowerCase().includes(q));
  box.innerHTML = "";
  setText("filesCount", SESS.length ? `${SESS.length}` : "");
  const total = SESS.reduce((a, s) => a + (s.bytes || 0), 0);
  setText("filesFoot", SESS.length ? `${fmtBytes(total)} in the latest ${SESS.length} · click a recording for its files, click a file to open it` : "");
  if (!list.length) { box.innerHTML = `<div class="empty">${SESS.length ? "No match." : "No recordings yet."}</div>`; return; }
  let lastGroup = "";
  for (const s of list) {
    const grp = s.date && s.subject_id ? `${s.date} · ${s.subject_id}` : "older recordings";
    if (grp !== lastGroup) {
      lastGroup = grp;
      const h = document.createElement("div");
      h.className = "sessgroup"; h.textContent = grp;
      box.appendChild(h);
    }
    const el = document.createElement("div");
    el.className = "sess2" + (openSess.has(s.name) ? " open" : "") + (s.name === newestSess ? " new" : "");
    let tag = `<span class="pill tag">IMU only</span>`;
    if (s.sync === true) tag = `<span class="pill ok tag">SYNC ✓</span>`;
    else if (s.sync === false) tag = `<span class="pill bad tag">SYNC ✗</span>`;
    if (s.aborted) tag = `<span class="pill warn tag">ABORTED</span>`;
    el.innerHTML = `<div class="sh" title="Show files"><span class="who"></span>${tag}<span class="when"></span><span class="meta"></span></div><div class="fl"></div>`;
    const tk = TASKS.find((x) => x.key === s.task);
    const who = s.task === "static_cal" ? `Static calibration${s.name.match(/_\d+$/) ? " (" + s.name.match(/_(\d+)$/)[1] + ")" : ""}`
      : s.task ? `${tk ? tk.name : s.task_name || s.task} · trial ${String(s.trial_n).padStart(2, "0")}${s.name.match(/trial\d+_(\d+)$/) ? " (repeat " + s.name.match(/_(\d+)$/)[1] + ")" : ""}`
      : [s.subject, s.trial].filter(Boolean).join(" · ") || s.name;
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
// Alerts: stacked at the bottom right (newest at the bottom), each fades out after its time.
// toast(msg)            info, 4 s
// toast(msg, true)      error, 8 s
// toast(msg, "warn")    warning, 6 s
// Hovering an alert pauses its timer; the same message again bumps a counter instead of stacking.
const TOAST_MS = { info: 4000, warn: 6000, bad: 8000 }, TOAST_MAX = 5;
const TOAST_ICON = { info: "✓", warn: "!", bad: "✕" };
function toast(msg, level) {
  const kind = level === true || level === "bad" || level === "error" ? "bad" : level === "warn" ? "warn" : "info";
  const box = $("toasts");
  const same = [...box.children].find((el) => !el.classList.contains("out") && el.dataset.msg === msg && el.dataset.kind === kind);
  if (same) {                                         // repeat: count it and restart its timer
    const n = (+same.dataset.n || 1) + 1;
    same.dataset.n = n;
    const c = same.querySelector(".tn"); c.textContent = "×" + n; c.hidden = false;
    same._restart();
    return;
  }
  const el = document.createElement("div");
  el.className = "tst " + kind;
  el.dataset.msg = msg; el.dataset.kind = kind;
  el.setAttribute("role", kind === "bad" ? "alert" : "status");
  el.innerHTML = '<span class="ti"></span><span class="tm"></span><span class="tn" hidden></span>' +
    '<button class="tx" title="Dismiss" aria-label="Dismiss">×</button><i class="tbar"></i>';
  el.querySelector(".ti").textContent = TOAST_ICON[kind];
  el.querySelector(".tm").textContent = msg;
  const ms = TOAST_MS[kind], bar = el.querySelector(".tbar");
  let timer = null, left = ms, t0 = 0;
  const close = () => {
    if (el.classList.contains("out")) return;
    clearTimeout(timer); el.classList.add("out");
    setTimeout(() => el.remove(), 220);
  };
  const run = () => {                                  // start / resume the countdown and its bar
    t0 = performance.now(); clearTimeout(timer); timer = setTimeout(close, left);
    bar.style.transition = "none"; bar.style.transform = `scaleX(${left / ms})`;
    void bar.offsetWidth;
    bar.style.transition = `transform ${left}ms linear`; bar.style.transform = "scaleX(0)";
  };
  const pause = () => {
    clearTimeout(timer); left = Math.max(400, left - (performance.now() - t0));
    bar.style.transition = "none"; bar.style.transform = `scaleX(${left / ms})`;
  };
  el._restart = () => { left = ms; run(); };
  el.addEventListener("mouseenter", pause);
  el.addEventListener("mouseleave", run);
  el.querySelector(".tx").onclick = close;
  box.appendChild(el);
  const live = [...box.children].filter((x) => !x.classList.contains("out"));
  for (const old of live.slice(0, Math.max(0, live.length - TOAST_MAX))) old.querySelector(".tx").click();
  run();
}

async function api(path, body) {
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
    const j = await r.json();
    if (!j.ok) toast(j.error || "failed", true);
    return j;
  } catch (e) { toast("Server not running: double-click “Start IMU DAQ.bat” on the Desktop", true); return { ok: false }; }
}

function saveForm() { for (const id of ["subject", "label", "notes", "testDate", "heightCm"]) LS.set("f." + id, $(id).value); LS.set("f.task", $("taskSel").value); }

// participant height -> spine model scale (height / 180 cm); stored with the spine calibration
function heightVal() { const h = parseFloat($("heightCm").value); return h >= 100 && h <= 230 ? h : null; }
function pushHeight() {
  const h = heightVal();
  if ($("heightCm").value !== "" && h == null) { toast("Height must be 100–230 cm", true); return; }
  saveForm();
  let sc;
  try { sc = JSON.parse(localStorage.getItem("spineCal") || "null"); } catch { sc = null; }
  sc = sc || { q0: null, B: null, map: null, when: null };
  if ((sc.height_cm ?? null) === h) return;
  if (h == null) delete sc.height_cm; else sc.height_cm = h;
  try { localStorage.setItem("spineCal", JSON.stringify(sc)); } catch { }
  postSpineCal(sc);
  const fr = $("spineFrame");                 // the embedded view does not get storage events from its own parent
  if (fr && fr.dataset.on === "1" && fr.contentWindow) fr.contentWindow.postMessage({ type: "spineopt", height: h }, location.origin);
}

function wireCalib() {
  $("btnCal").onclick = runCalibration;
  for (const b of $("calSecs").querySelectorAll("button")) {
    b.classList.toggle("on", +b.dataset.s === calSecs);
    b.onclick = () => {
      calSecs = +b.dataset.s; LS.set("calSecs", calSecs);
      for (const x of $("calSecs").querySelectorAll("button")) x.classList.toggle("on", x === b);
    };
  }
  $("subject").addEventListener("input", () => { if (S) { applyCalib(S); applyFlow(S); } });
  // step 3c: re-check after the subject / date changes and when coming back from the EMG norm tab
  for (const id of ["subject", "testDate"]) $(id).addEventListener("change", () => checkEmgNorm());
  window.addEventListener("focus", () => checkEmgNorm(true));
  document.addEventListener("visibilitychange", () => { if (!document.hidden) checkEmgNorm(true); });
  $("btnEmgNorm").addEventListener("click", () => saveForm());      // the EMG page reads the same participant
  setInterval(() => checkEmgNorm(true), 30000);
}

async function startRec() {
  saveForm();
  $("btnStart").disabled = true;
  if (!$("subject").value.trim() || !$("taskSel").value) {
    toast(!$("subject").value.trim() ? "Enter the subject first (step 2)" : "Pick a task first", true);
    $("btnStart").disabled = false; return;
  }
  const t = TASKS.find((x) => x.key === $("taskSel").value) || {};
  const j = await api("/api/start", { subject: $("subject").value.trim(), date: dateId($("testDate").value), task: $("taskSel").value,
    task_name: t.name || "", trial: Math.max(1, parseInt($("label").value, 10) || 1), notes: $("notes").value, height_cm: heightVal() });
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
  { // height: from this form, else from the spine calibration (e.g. typed in the Spine view)
    let sc = null; try { sc = JSON.parse(localStorage.getItem("spineCal") || "null"); } catch { }
    $("heightCm").value = LS.get("f.heightCm", "") || (sc && sc.height_cm) || "";
    $("heightCm").addEventListener("change", pushHeight);
    window.addEventListener("storage", (e) => {
      if (e.key !== "spineCal") return;
      try { const c = JSON.parse(e.newValue || "null"); if (c && document.activeElement !== $("heightCm")) { $("heightCm").value = c.height_cm || ""; saveForm(); } } catch { }
    });
  }
  $("testDate").value = todayId();                      // a new page load starts on today's date (editable)
  $("testDate").addEventListener("change", () => { $("testDate").value = dateId($("testDate").value); saveForm(); refreshNextTrial(); });
  $("subject").addEventListener("change", refreshNextTrial);
  $("taskSel").addEventListener("change", () => { saveForm(); refreshNextTrial(); });
  $("label").addEventListener("input", applyTaskInfo);
  $("subject").addEventListener("input", applyTaskInfo);
  loadTasks();
  $("btnStart").onclick = startRec;
  $("btnStop").onclick = stopRec;
  $("btnBend").onclick = () => { $("btnBend").blur(); runBend(); };
  $("tpRec").onclick = () => { $("tpRec").blur(); toggleRecording(); };
  $("tpMark").onclick = () => { $("tpMark").blur(); mark(); };
  $("tpTake").onclick = () => { const el = $("label"); el.scrollIntoView({ block: "center", behavior: "smooth" }); el.focus(); el.select(); };
  for (const id of ["subject", "label"]) $(id).addEventListener("input", () => { if (S) applyTransport(S); });
  $("btnMark").onclick = mark;
  $("markText").addEventListener("keydown", (e) => { if (e.key === "Enter") mark(); });
  $("btnConnect").onclick = () => api("/api/connect", { port: $("portSel").value || null });
  $("btnDisconnect").onclick = () => api("/api/disconnect");
  $("portSel").addEventListener("focus", loadPorts);
  $("rateSel").onchange = () => api("/api/rate", { ms: +$("rateSel").value });
  $("quatSel").onchange = async () => {
    const j = await api("/api/quatmode", { mode: $("quatSel").value });
    if (j.ok) toast(j.mode === "rv" ? "Rotation vector (with magnetometer): move each IMU in slow figure-8s to calibrate"
      : "Game rotation vector (no magnetometer)");
  };
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
  for (const b of $("plotSeg").querySelectorAll("button")) {
    b.classList.toggle("on", b.dataset.p === plotMode);
    b.onclick = () => {
      plotMode = b.dataset.p; LS.set("plotMode", plotMode);
      for (const x of $("plotSeg").querySelectorAll("button")) x.classList.toggle("on", x === b);
      for (const p of pods.values()) p.dirty = true;
    };
  }
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
  wireSteps();
  wireSetup();
  wireSpine();
  wireCalib();
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

// ---------------- spine panel (left side) ----------------
// The Spine view page in "embed" mode, loaded only when the screen is wide enough to show it,
// so narrow windows do not run a second 3D view and data stream.
function spineCalText() {
  try {
    const c = JSON.parse(localStorage.getItem("spineCal") || "null");
    if (!c || !c.q0) return ["not calibrated", "bad"];
    if (!c.B) return ["half calibrated", "warn"];
    return ["calibrated" + (c.when ? " " + new Date(c.when).toTimeString().slice(0, 5) : ""), "good"];
  } catch { return ["", ""]; }
}
function wireSpine() {
  syncSpineCal();
  const mq = matchMedia("(min-width: 1780px)"), fr = $("spineFrame");
  const apply = () => {
    if (mq.matches && fr.dataset.on !== "1") { fr.dataset.on = "1"; fr.src = "/spine?embed=1"; }
    else if (!mq.matches && fr.dataset.on === "1") { fr.dataset.on = "0"; fr.src = "about:blank"; }
  };
  mq.addEventListener("change", apply);
  apply();
  // model options (same controls as the Spine view's "Model" card), sent to the embedded view
  const opt = (k, d) => { try { const v = localStorage.getItem(k); return v == null ? d : JSON.parse(v); } catch { return d; } };
  const put = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch { } };
  const send = (o) => { if (fr.dataset.on === "1" && fr.contentWindow) fr.contentWindow.postMessage({ type: "spineopt", ...o }, location.origin); };
  const syncTools = () => {
    const view = opt("spineView", "iso");
    for (const b of $("spView").querySelectorAll("button")) b.classList.toggle("on", b.dataset.v === view);
    $("spRibs").checked = opt("showRibs", true); $("spTrail").checked = opt("showTrail", false);
  };
  syncTools();
  for (const b of $("spView").querySelectorAll("button")) b.onclick = () => { put("spineView", b.dataset.v); syncTools(); send({ view: b.dataset.v }); };
  $("spRibs").onchange = () => { put("showRibs", $("spRibs").checked); send({ ribs: $("spRibs").checked }); };
  $("spTrail").onchange = () => { put("showTrail", $("spTrail").checked); send({ trail: $("spTrail").checked }); };
  fr.addEventListener("load", () => send({ view: opt("spineView", "iso") }));
  window.addEventListener("storage", (e) => { if (["spineView", "showRibs", "showTrail"].includes(e.key)) syncTools(); });
  const hint = () => { const [t, c] = spineCalText(); const h = $("spineHint"); h.textContent = t; h.className = "hint " + c; };
  hint();
  window.addEventListener("storage", (e) => { if (e.key === "spineCal") hint(); });   // calibrated in the full view
  setInterval(hint, 5000);
}

// ---------------- orientation mode (game rotation vector / rotation vector) ----------------
const QUAT_SHORT = { game: "game RV", rv: "RV + mag" };
// what the board actually runs; falls back to the requested mode while the board reports nothing
const quatOf = (s) => (s && s.hello && s.hello.quat) || (s && s.quat_mode) || "game";

function applyQuatMode(s) {
  const want = s.quat_mode || "game", have = s.hello && s.hello.quat;
  const sel = $("quatSel");
  if (document.activeElement !== sel) sel.value = want;
  const note = $("quatNote");
  if (s.hello && s.hello.fw && !have) {
    note.textContent = "This firmware only has the game rotation vector: upload imu_daq v1.3 to switch.";
    note.className = "note bad";
  } else if (have && have !== want) {
    note.textContent = "Switching the IMUs…"; note.className = "note";
  } else if (want === "rv") {
    note.textContent = "Absolute heading shared by both IMUs (no relative yaw drift). Keep metal, motors and " +
      "electronics away; calibrate with slow figure-8s until both IMUs show mag 3/3. Saved with each recording.";
    note.className = "note";
  } else {
    note.textContent = "Heading is relative and drifts slowly (affects axial rotation over minutes); not " +
      "affected by metal nearby. Saved with each recording.";
    note.className = "note";
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
  else if (k === "c") { e.preventDefault(); runCalibration(); }
  else if (k === "b") { e.preventDefault(); runBend(); }
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
  const es = openStream();          // one shared /api/stream for all tabs (stream.js)
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
    for (const id of ["btnStart", "btnStop", "btnMark", "tpRec", "tpMark"]) $(id).disabled = true;
    $("tpMode").className = "tp-mode"; $("tpMode").lastChild.textContent = "NO SERVER";
  };
}

wire();
loadPorts();
loadSessions();
connectStream();
requestAnimationFrame(frameLoop);
