// Review page: open a finished recording, plot IMU and EMG on one time axis, replay the spine model.
// Data come from /api/review (host/review.py): IMU times are on the DAQ clock, so EMG and IMU line up.
import { createSpine, segment, eulerXYZ, qmul, qconj } from "/static/spinemodel.js";

const $ = (id) => document.getElementById(id);
const RAD2DEG = 180 / Math.PI;
const LS = {
  get(k, d) { try { const v = localStorage.getItem("rv." + k); return v == null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("rv." + k, JSON.stringify(v)); } catch { } },
};
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

let TASKS = [];
let SESS = [], D = null, cur = 0, full = [0, 1], view = [0, 1], playing = false, speed = 1;
let imuMode = LS.get("imuMode", "ang"), emgMode = LS.get("emgMode", "raw"), emgScale = LS.get("emgScale", "each");
let emgOnly = false, anyway = false, spine = null, spineReady = null, tracks = [], der = null, loadSeq = 0;

// ---------------- toasts ----------------
function toast(text, bad) {
  const el = document.createElement("div");
  el.className = "tst" + (bad ? " bad" : "");
  el.innerHTML = `<span class="ti">${bad ? "!" : "✓"}</span><span class="tm"></span><span></span><button class="tx">×</button>`;
  el.querySelector(".tm").textContent = text;
  el.querySelector(".tx").onclick = () => el.remove();
  $("toasts").appendChild(el);
  setTimeout(() => el.remove(), bad ? 8000 : 4000);
}

// ---------------- recordings list ----------------
async function loadList() {
  try { SESS = (await (await fetch("/api/sessions")).json()).sessions || []; } catch { SESS = []; toast("Server not reachable", true); }
  renderList();
}
function sessTitle(s) {
  if (s.task === "static_cal") return "Static calibration";
  const tk = TASKS.find((x) => x.key === s.task);
  if (s.task) return `${tk ? tk.name : s.task_name || s.task} · trial ${String(s.trial_n).padStart(2, "0")}`;
  return [s.subject, s.trial].filter(Boolean).join(" · ") || s.name;
}
function renderList() {
  const box = $("rvSessions"), q = $("rvSearch").value.trim().toLowerCase(), showCal = $("rvShowCal").checked;
  const list = SESS.filter((s) => (showCal || s.task !== "static_cal") &&
    (!q || `${s.rel || s.name} ${s.subject} ${s.trial} ${s.task} ${s.notes}`.toLowerCase().includes(q)));
  $("rvCount").textContent = SESS.length ? String(list.length) : "";
  box.innerHTML = "";
  if (!list.length) { box.innerHTML = `<div class="empty">${SESS.length ? "No match." : "No recordings yet."}</div>`; return; }
  let grp = "";
  for (const s of list) {
    const g = s.date && s.subject_id ? `${s.date} · ${s.subject_id}` : "older recordings";
    if (g !== grp) { grp = g; const h = document.createElement("div"); h.className = "sessgroup"; h.textContent = g; box.appendChild(h); }
    const el = document.createElement("button");
    el.className = "rv-sess" + (D && D.meta.rel === s.rel ? " on" : "");
    let tag = `<span class="pill tag">IMU only</span>`;
    if (s.sync === true) tag = `<span class="pill ok tag">SYNC ✓</span>`;
    else if (s.sync === false) tag = `<span class="pill bad tag">SYNC ✗</span>`;
    if (s.aborted) tag = `<span class="pill warn tag">ABORTED</span>`;
    el.innerHTML = `<span class="who"></span>${tag}<span class="meta"></span>`;
    el.querySelector(".who").textContent = sessTitle(s);
    el.querySelector(".meta").textContent = `${(s.started || "").slice(11, 16)} · ${s.duration_s ?? 0} s · ${(s.frames ?? 0).toLocaleString()} frames` + (s.missing ? ` · ${s.missing} missing` : "");
    el.title = s.rel || s.name;
    el.onclick = () => openRec(s.rel);
    box.appendChild(el);
  }
}

// ---------------- open a recording ----------------
async function openRec(rel) {
  const my = ++loadSeq;
  pause();
  $("rvTitle").textContent = "Loading " + rel + "…";
  let j;
  try { j = await (await fetch("/api/review?rel=" + encodeURIComponent(rel))).json(); }
  catch (e) { j = { ok: false, error: e.message }; }
  if (my !== loadSeq) return;                 // a newer click won
  if (!j.ok) { toast("Cannot open: " + (j.error || "unknown error"), true); $("rvTitle").textContent = rel; return; }
  D = j; anyway = false;
  LS.set("last", rel);
  history.replaceState(null, "", "/review?rel=" + encodeURIComponent(rel));
  const ts = [];
  if (D.imu && D.imu.t.length) ts.push(D.imu.t[0], D.imu.t[D.imu.t.length - 1]);
  if (D.emg) ts.push(D.emg.t0, D.emg.t0 + D.emg.n_samples / D.emg.rate_hz);
  full = ts.length ? [Math.min(...ts), Math.max(...ts)] : [0, 1];
  view = full.slice(); cur = D.imu && D.imu.t.length ? D.imu.t[0] : full[0];
  $("rvEmpty").classList.add("hidden"); $("rvBody").classList.remove("hidden");
  // EMG normalization captures (rest / MVC / RVC): EMG only, no spine model, angles or IMU
  emgOnly = String(D.meta.task || "").startsWith("emg_");
  $("rvBody").classList.toggle("emgonly", emgOnly);
  renderInfo(); derive(); buildTracks(); renderList();
  update(true);                               // plots do not wait for the 3D model
  await spineReady;
  if (my !== loadSeq || !spine) return;
  spine.setHeight(D.cal && D.cal.height_cm ? D.cal.height_cm : 0);
  updateSpine();
}

function renderInfo() {
  const m = D.meta;
  const title = m.task_name ? `${m.task_name} · trial ${String(m.trial ?? "").padStart(2, "0")}` : (m.session || m.rel);
  $("rvName").textContent = title;
  $("rvRel").textContent = m.rel;
  $("rvTitle").textContent = `${m.subject || ""} · ${title}`;
  const st = [
    ["Subject", m.subject || "—"], ["Started", (m.started || "—").slice(0, 16)], ["Duration", m.duration_s != null ? `${m.duration_s} s` : "—"],
    ["Frames", (m.frames_received ?? 0).toLocaleString()], ["Missing", String(m.missing_frames ?? 0), m.missing_frames ? "bad" : ""],
    ["Sync", m.sync_ok == null ? "IMU only" : m.sync_ok ? "OK" : "FAILED", m.sync_ok === false ? "bad" : m.sync_ok ? "good" : ""],
    ["Markers", String((D.events || []).filter((e) => e.type === "marker").length)],
    ["EMG", D.emg ? `${D.emg.channels.length} ch · ${(D.emg.rate_hz / 1000).toFixed(0)} kHz` : "none"],
  ];
  $("rvStats").innerHTML = "";
  for (const [k, v, c] of st) {
    const d = document.createElement("div");
    d.innerHTML = `<dt></dt><dd></dd>`; d.firstChild.textContent = k; d.lastChild.textContent = v;
    if (c) d.lastChild.className = c;
    $("rvStats").appendChild(d);
  }
  $("rvNotes").textContent = [m.notes ? "Notes: " + m.notes : "", m.aborted ? "This recording was aborted." : ""].filter(Boolean).join(" · ");
  $("rvOpenFolder").onclick = () => fetch("/api/open", { method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ dir: (SESS.find((s) => s.rel === m.rel) || {}).dir }) });
}

// ---------------- derived signals ----------------
function calState() {
  const c = D.cal, names = D.imu ? Object.keys(D.imu.pods) : [];
  const map = c && c.map && names.includes(c.map.T7) && names.includes(c.map.T12) ? c.map : { T7: names[0], T12: names[1] };
  const q0ok = !!(c && c.q0 && map.T7 && map.T12 && c.q0[map.T7] && c.q0[map.T12]);
  const B = c && c.B ? c.B : null;
  return { map, q0ok, B, live: q0ok && (!!B || anyway), names };
}

function derive() {
  der = { pods: {}, rel: null };
  if (!D.imu) return;
  const cs = calState(), n = D.imu.t.length;
  for (const [name, p] of Object.entries(D.imu.pods)) {
    if (!p.q) continue;
    const q0 = cs.q0ok && D.cal.q0[name] ? D.cal.q0[name] : p.q.find((q) => isFinite(q[0]));
    const B = cs.B && cs.q0ok ? cs.B : null;
    const a = [[], [], []];
    for (let i = 0; i < n; i++) {
      const s = segment(p.q[i], q0, B), e = s ? eulerXYZ(s) : [NaN, NaN, NaN];
      a[0].push(e[0]); a[1].push(e[1]); a[2].push(e[2]);
    }
    der.pods[name] = { ang: a, ref: cs.q0ok && D.cal.q0[name] ? (B ? "body axes, from standing" : "sensor axes, from standing") : "from the first frame" };
  }
  if (cs.live) {
    const A = D.imu.pods[cs.map.T7].q, Bq = D.imu.pods[cs.map.T12].q, B = cs.B || [1, 0, 0, 0];
    const r = [[], [], []];
    for (let i = 0; i < n; i++) {
      const a = segment(A[i], D.cal.q0[cs.map.T7], B), b = segment(Bq[i], D.cal.q0[cs.map.T12], B);
      const e = a && b ? eulerXYZ(qmul(qconj(b), a)) : [NaN, NaN, NaN];
      r[0].push(e[0]); r[1].push(e[1]); r[2].push(e[2]);
    }
    der.rel = r;
  }
}

// ---------------- tracks (one canvas per signal row) ----------------
function makeTrack(parent, label, sub, opts) {
  const el = document.createElement("div");
  el.className = "rv-track" + (opts.cls ? " " + opts.cls : "");
  el.innerHTML = `<div class="rv-tl"><b></b><small></small><span class="rv-val mono"></span></div><div class="rv-cvwrap"><canvas></canvas><div class="rv-cur"></div></div>`;
  el.querySelector("b").textContent = label;
  el.querySelector("small").textContent = sub || "";
  parent.appendChild(el);
  const t = { el, cv: el.querySelector("canvas"), curEl: el.querySelector(".rv-cur"), val: el.querySelector(".rv-val"), ...opts };
  tracks.push(t);
  return t;
}

function idxAt(arr, t) {                      // last index with arr[i] <= t
  let lo = 0, hi = arr.length - 1;
  if (!arr.length || t <= arr[0]) return 0;
  if (t >= arr[hi]) return hi;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (arr[m] <= t) lo = m; else hi = m; }
  return lo;
}

const COLORS = ["--roll", "--pitch", "--yaw"];
function seriesTrack(parent, label, sub, getY, unit, fmt, minRange) {
  return makeTrack(parent, label, sub, {
    draw(g, W, H) {
      const t = D.imu.t, i0 = Math.max(0, idxAt(t, view[0]) - 1), i1 = Math.min(t.length - 1, idxAt(t, view[1]) + 1);
      const ys = [0, 1, 2].map(getY);
      let lo = Infinity, hi = -Infinity;
      for (const y of ys) for (let i = i0; i <= i1; i++) { const v = y[i]; if (isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }
      if (!isFinite(lo)) { lo = -1; hi = 1; }
      const mid = (lo + hi) / 2, half = Math.max(minRange / 2, (hi - lo) / 2 * 1.1);
      lo = mid - half; hi = mid + half;
      zeroAndGrid(g, W, H, lo, hi, unit);
      const X = (tt) => ((tt - view[0]) / (view[1] - view[0])) * W, Y = (v) => H - 3 - ((v - lo) / (hi - lo)) * (H - 6);
      ys.forEach((y, k) => {
        g.strokeStyle = css(COLORS[k]); g.lineWidth = 1.4; g.beginPath();
        let pen = false;
        for (let i = i0; i <= i1; i++) {
          const v = y[i]; if (!isFinite(v)) { pen = false; continue; }
          const x = X(t[i]), yy = Y(v);
          if (pen) g.lineTo(x, yy); else { g.moveTo(x, yy); pen = true; }
        }
        g.stroke();
      });
    },
    value(tt) { const i = idxAt(D.imu.t, tt); return [0, 1, 2].map((k) => fmt(getY(k)[i])).join("  "); },
  });
}

function zeroAndGrid(g, W, H, lo, hi, unit) {
  g.strokeStyle = css("--border"); g.lineWidth = 1;
  timeGrid(g, W, H);
  if (lo < 0 && hi > 0) {
    const y = H - 3 - ((0 - lo) / (hi - lo)) * (H - 6);
    g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke();
  }
  g.fillStyle = css("--text-3"); g.font = "10px " + css("--mono");
  g.fillText(`${fmtNum(hi)} ${unit}`, 4, 11); g.fillText(`${fmtNum(lo)}`, 4, H - 4);
}
const fmtNum = (v) => Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2);

function tickStep(span, px) {
  const raw = span / Math.max(1, px / 90);
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  return [1, 2, 5, 10].map((m) => m * p).find((s) => s >= raw) || raw;
}
function timeGrid(g, W, H) {
  const s = tickStep(view[1] - view[0], W);
  g.save(); g.strokeStyle = css("--border"); g.globalAlpha = 0.55; g.setLineDash([2, 4]); g.beginPath();
  for (let t = Math.ceil(view[0] / s) * s; t <= view[1]; t += s) {
    const x = ((t - view[0]) / (view[1] - view[0])) * W; g.moveTo(x, 0); g.lineTo(x, H);
  }
  g.stroke(); g.restore();
  // markers
  for (const e of D.events || []) {
    if (e.type !== "marker" || e.t == null || e.t < view[0] || e.t > view[1]) continue;
    const x = ((e.t - view[0]) / (view[1] - view[0])) * W;
    g.save(); g.strokeStyle = css("--accent"); g.lineWidth = 1.2; g.beginPath(); g.moveTo(x, 0); g.lineTo(x, H); g.stroke(); g.restore();
  }
}

function axisTrack(parent) {
  return makeTrack(parent, "", "", {
    cls: "axis",
    draw(g, W, H) {
      const s = tickStep(view[1] - view[0], W), dec = s < 0.01 ? 3 : s < 0.1 ? 2 : s < 1 ? 1 : 0;
      g.fillStyle = css("--text-3"); g.font = "10.5px " + css("--mono"); g.textAlign = "center";
      for (let t = Math.ceil(view[0] / s) * s; t <= view[1] + 1e-9; t += s) {
        const x = ((t - view[0]) / (view[1] - view[0])) * W;
        g.fillText(t.toFixed(dec) + " s", Math.min(W - 18, Math.max(18, x)), 12);
      }
      for (const e of D.events || []) {
        if (e.type !== "marker" || e.t == null || e.t < view[0] || e.t > view[1]) continue;
        const x = ((e.t - view[0]) / (view[1] - view[0])) * W;
        g.fillStyle = css("--accent"); g.textAlign = "left"; g.fillText("⚑ " + (e.detail || "").slice(0, 24), x + 3, 12);
      }
    },
    value() { return ""; },
  });
}

function emgTrack(parent, ch, gmax) {
  const max = (k) => emgScale === "same" ? gmax[k] : chanMax(ch, k);
  return makeTrack(parent, ch.name, `${ch.side ? ch.side + " " : ""}${ch.muscle}`, {
    cls: "emg",
    draw(g, W, H) {
      const e = D.emg, b = e.bin_s, n = ch.min.length;
      const i0 = Math.max(0, Math.floor((view[0] - e.t0) / b)), i1 = Math.min(n - 1, Math.ceil((view[1] - e.t0) / b));
      const X = (i) => ((e.t0 + (i + 0.5) * b - view[0]) / (view[1] - view[0])) * W;
      timeGrid(g, W, H);
      g.fillStyle = css("--text-3"); g.font = "10px " + css("--mono");
      if (emgMode === "norm" && ch.ref_mv) {           // % of the saved MVC / RVC
        const pct = ch.rms.map((v) => (v / ch.ref_mv) * 100);
        let m = 100; for (let i = i0; i <= i1; i++) if (pct[i] > m) m = pct[i];
        const Y = (v) => H - 2 - (v / m) * (H - 4);
        g.fillText(`${m.toFixed(0)} %${ch.ref_kind || "MVC"}`, 4, 11);
        g.save(); g.strokeStyle = css("--border-strong"); g.setLineDash([3, 3]); g.beginPath(); g.moveTo(0, Y(100)); g.lineTo(W, Y(100)); g.stroke(); g.restore();
        g.strokeStyle = css("--accent"); g.lineWidth = 1.4; g.beginPath();
        for (let i = i0; i <= i1; i++) { const x = X(i), y = Y(pct[i]); if (i === i0) g.moveTo(x, y); else g.lineTo(x, y); }
        g.stroke();
      } else if (emgMode === "rms" || emgMode === "norm") {
        const m = max("rms") || 1, Y = (v) => H - 2 - (v / m) * (H - 4);
        g.fillText(`${fmtNum(m)} mV`, 4, 11);
        g.strokeStyle = css("--accent"); g.lineWidth = 1.4; g.beginPath();
        for (let i = i0; i <= i1; i++) { const x = X(i), y = Y(ch.rms[i]); if (i === i0) g.moveTo(x, y); else g.lineTo(x, y); }
        g.stroke();
      } else {
        const m = max("raw") || 1, Y = (v) => H / 2 - (v / m) * (H / 2 - 2);
        g.fillText(`±${fmtNum(m)} mV`, 4, 11);
        g.strokeStyle = css("--border"); g.beginPath(); g.moveTo(0, H / 2); g.lineTo(W, H / 2); g.stroke();
        g.strokeStyle = css("--text-2"); g.lineWidth = 1; g.beginPath();
        // one vertical stroke per pixel column (min..max of the bins in it)
        let px = -1, lo = 0, hi = 0;
        const flush = () => { if (px >= 0) { g.moveTo(px + 0.5, Y(hi)); g.lineTo(px + 0.5, Y(lo) + 0.5); } };
        for (let i = i0; i <= i1; i++) {
          const x = Math.floor(X(i));
          if (x !== px) { flush(); px = x; lo = ch.min[i]; hi = ch.max[i]; }
          else { lo = Math.min(lo, ch.min[i]); hi = Math.max(hi, ch.max[i]); }
        }
        flush(); g.stroke();
      }
    },
    value(tt) {
      const i = Math.max(0, Math.min(ch.rms.length - 1, Math.floor((tt - D.emg.t0) / D.emg.bin_s)));
      if (emgMode === "norm" && ch.ref_mv) return `${((ch.rms[i] / ch.ref_mv) * 100).toFixed(1)} %${ch.ref_kind || "MVC"}`;
      return `RMS ${ch.rms[i].toFixed(2)} mV` + (emgMode === "norm" ? " · no reference" : "");
    },
  });
}
const _cm = new WeakMap();
function chanMax(ch, k) {                     // robust full-scale: 99.9th percentile of |signal|
  let m = _cm.get(ch); if (!m) { m = {}; _cm.set(ch, m); }
  if (m[k] == null) {
    const v = k === "rms" ? ch.rms.slice() : ch.max.map((x, i) => Math.max(Math.abs(x), Math.abs(ch.min[i])));
    v.sort((a, b) => a - b);
    m[k] = v[Math.min(v.length - 1, Math.floor(v.length * 0.999))] || 1;
  }
  return m[k];
}

function buildTracks() {
  tracks = [];
  for (const id of ["rvSpineTracks", "rvImuTracks", "rvEmgTracks"]) $(id).innerHTML = "";
  // spine angles
  const cs = D.imu ? calState() : null;
  if (emgOnly) { /* hidden for EMG normalization captures */ }
  else if (D.imu && der.rel) {
    seriesTrack($("rvSpineTracks"), "T7–T12", cs.B ? "body axes" : "sensor axes", (k) => der.rel[k], "°", (v) => isFinite(v) ? v.toFixed(1) + "°" : "—", 10);
    axisTrack($("rvSpineTracks"));
  } else {
    $("rvSpineTracks").innerHTML = `<div class="emgempty">Needs the static calibration with a forward bend: see the spine model.</div>`;
  }
  // IMU
  if (emgOnly) { /* hidden */ }
  else if (D.imu) {
    const names = Object.keys(D.imu.pods);
    for (const n of names) {
      const p = D.imu.pods[n], role = cs && cs.map.T7 === n ? "T7" : cs && cs.map.T12 === n ? "T12" : "";
      if (imuMode === "ang" && der.pods[n]) {
        seriesTrack($("rvImuTracks"), n, `${role ? role + " · " : ""}${der.pods[n].ref}`, (k) => der.pods[n].ang[k], "°", (v) => isFinite(v) ? v.toFixed(1) + "°" : "—", 10);
      } else if (imuMode !== "ang" && p[imuMode]) {
        const sc = imuMode === "gyr" ? RAD2DEG : 1, unit = imuMode === "gyr" ? "°/s" : "m/s²";
        const cols = [0, 1, 2].map((k) => p[imuMode].map((r) => r[k] * sc));
        seriesTrack($("rvImuTracks"), n, `${role ? role + " · " : ""}${imuMode === "acc" ? "x y z (incl. gravity)" : "x y z"}`, (k) => cols[k], unit,
          (v) => isFinite(v) ? v.toFixed(imuMode === "gyr" ? 0 : 2) : "—", imuMode === "gyr" ? 30 : 2);
      }
    }
    axisTrack($("rvImuTracks"));
    $("rvImuHint").textContent = imuMode === "ang" ? "flexion / lat. bend / axial rot. (X-Y-Z)" : `${D.imu.t.length.toLocaleString()} frames · ${D.imu.clock === "daq" ? "DAQ clock" : D.imu.clock === "pc" ? "aligned by PC clock" : "board clock"}`;
  } else $("rvImuTracks").innerHTML = `<div class="emgempty">No IMU data in this recording.</div>`;
  // EMG
  if (D.emg && D.emg.channels.length) {
    const gmax = {
      raw: Math.max(...D.emg.channels.map((c) => chanMax(c, "raw"))),
      rms: Math.max(...D.emg.channels.map((c) => chanMax(c, "rms"))),
    };
    for (const ch of D.emg.channels) emgTrack($("rvEmgTracks"), ch, gmax);
    axisTrack($("rvEmgTracks"));
    const hasNorm = D.emg.channels.some((c) => c.ref_mv);
    $("rvEmgNorm").disabled = !hasNorm;
    $("rvEmgNorm").title = hasNorm ? `RMS as % of the MVC / RVC (${D.emg.norm_source}, saved ${D.emg.norm_saved})` : "No EMG normalization for this subject: make one on the EMG norm page";
    $("rvEmgHint").textContent = `DC removed · envelope ${(D.emg.bin_s * 1000).toFixed(1)} ms bins · RMS 100 ms, 20–450 Hz` + (emgMode === "norm" && hasNorm ? " · % of " + D.emg.norm_source : "");
  } else { $("rvEmgTracks").innerHTML = `<div class="emgempty">No EMG (DAQ) data in this recording.</div>`; $("rvEmgHint").textContent = ""; }
  drawAll();
}

// ---------------- drawing ----------------
function drawAll() {
  for (const t of tracks) {
    const c = t.cv, dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
    if (!W || !H) continue;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    t.draw(g, W, H);
  }
  drawOverview();
  placeCursor();
}

function drawOverview() {
  const c = $("rvOvCv"), dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
  if (!W) return;
  if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
  const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const X = (t) => ((t - full[0]) / (full[1] - full[0])) * W;
  // summed EMG RMS as a quick activity trace
  if (D.emg && D.emg.channels.length) {
    const n = D.emg.channels[0].rms.length, step = Math.max(1, Math.floor(n / W)), sum = [];
    for (let i = 0; i < n; i += step) sum.push([i, D.emg.channels.reduce((a, ch) => a + ch.rms[i], 0)]);
    const m = Math.max(...sum.map((s) => s[1])) || 1;
    g.fillStyle = css("--border-strong"); g.beginPath(); g.moveTo(0, H);
    for (const [i, v] of sum) g.lineTo(X(D.emg.t0 + i * D.emg.bin_s), H - (v / m) * (H - 4));
    g.lineTo(W, H); g.fill();
  }
  for (const e of D.events || []) {
    if (e.type !== "marker" || e.t == null) continue;
    g.fillStyle = css("--accent"); g.fillRect(X(e.t) - 1, 0, 2, H);
  }
  const w = $("rvOvWin");
  w.style.left = `${(X(view[0]) / W) * 100}%`; w.style.width = `${((X(view[1]) - X(view[0])) / W) * 100}%`;
  w.classList.toggle("hidden", view[0] <= full[0] && view[1] >= full[1]);
}

function placeCursor() {
  for (const t of tracks) {
    const W = t.cv.clientWidth, x = ((cur - view[0]) / (view[1] - view[0])) * W;
    t.curEl.style.transform = `translateX(${x.toFixed(1)}px)`;
    t.curEl.style.display = cur >= view[0] && cur <= view[1] ? "" : "none";
    t.val.textContent = t.value(cur);
  }
  $("rvOvCur").style.left = `${((cur - full[0]) / (full[1] - full[0])) * 100}%`;
  $("rvTime").textContent = `${(cur - full[0]).toFixed(3)} s`;
  updateSpine();
}

// ---------------- spine replay ----------------
function updateSpine() {
  if (!D || !D.imu || emgOnly) { if (spine) spine.pose(null, null); return; }
  const cs = calState();
  const gate = !cs.live;
  $("rvStage").classList.toggle("uncal", gate);
  $("rvGate").classList.toggle("hidden", !gate);
  $("rvCalSrc").textContent = D.cal ? D.cal.source : "no calibration found";
  if (gate) {
    $("rvGateTitle").textContent = !cs.q0ok ? "No static calibration for this recording" : "No forward-bend axis in this recording";
    $("rvGateText").textContent = !cs.q0ok
      ? "Neither spine_cal.json nor calibration.json has neutral orientations for both IMUs, so the spine cannot be posed."
      : "The neutral posture is known, but the body's left–right axis was not saved: flexion, side bend and rotation would be mixed.";
    $("rvGateAnyway").classList.toggle("hidden", !cs.q0ok);
    if (spine) spine.pose(null, null);
    angleBox(null);
    return;
  }
  const i = idxAt(D.imu.t, cur), B = cs.B || [1, 0, 0, 0];
  const a = segment(D.imu.pods[cs.map.T7].q[i], D.cal.q0[cs.map.T7], B);
  const b = segment(D.imu.pods[cs.map.T12].q[i], D.cal.q0[cs.map.T12], B);
  if (spine) spine.pose(a, b);
  angleBox(a && b ? eulerXYZ(qmul(qconj(b), a)) : null);
}
function angleBox(a) {
  const el = $("rvAngles"), names = [["flex", "Flexion"], ["lat", "Lat. bend"], ["axi", "Axial rot."]];
  if (!el.children.length) el.innerHTML = names.map(([c, n]) => `<div class="a ${c}"><b>—</b><small>${n}</small></div>`).join("");
  [...el.children].forEach((c, k) => { c.firstChild.textContent = a && isFinite(a[k]) ? a[k].toFixed(1) + "°" : "—"; });
}

// ---------------- playback ----------------
let lastTs = 0;
function play() {
  if (!D) return;
  if (cur >= full[1] - 1e-3) cur = full[0];
  playing = true; lastTs = performance.now(); $("rvPlay").textContent = "❚❚"; $("rvPlay").classList.add("on");
  requestAnimationFrame(tick);
}
function pause() { playing = false; $("rvPlay").textContent = "▶"; $("rvPlay").classList.remove("on"); }
function tick(ts) {
  if (!playing) return;
  cur += ((ts - lastTs) / 1000) * speed; lastTs = ts;
  if (cur >= full[1]) { cur = full[1]; pause(); }
  if (cur > view[1] || cur < view[0]) {         // keep the cursor in the window
    const w = view[1] - view[0];
    view = clampView([cur - w * 0.1, cur + w * 0.9]); drawAll();
  } else placeCursor();
  if (playing) requestAnimationFrame(tick);
}
function seek(t) { cur = Math.max(full[0], Math.min(full[1], t)); placeCursor(); drawOverview(); }
function update(redraw) { if (redraw) drawAll(); else placeCursor(); }

function clampView(v) {
  const span = Math.min(full[1] - full[0], Math.max(0.05, v[1] - v[0]));
  let a = v[0];
  a = Math.max(full[0], Math.min(full[1] - span, a));
  return [a, a + span];
}

// ---------------- interaction ----------------
function plotTime(ev, cv) {
  const r = cv.getBoundingClientRect();
  return view[0] + ((ev.clientX - r.left) / r.width) * (view[1] - view[0]);
}
$("rvPlots").addEventListener("wheel", (ev) => {
  const cv = ev.target.closest(".rv-cvwrap")?.querySelector("canvas"); if (!cv || !D) return;
  ev.preventDefault();
  const t = plotTime(ev, cv), f = Math.pow(1.0015, ev.deltaY);
  view = clampView([t - (t - view[0]) * f, t + (view[1] - t) * f]); drawAll();
}, { passive: false });
$("rvPlots").addEventListener("pointerdown", (ev) => {
  const wrap = ev.target.closest(".rv-cvwrap"); if (!wrap || !D || ev.button !== 0) return;
  const cv = wrap.querySelector("canvas"), x0 = ev.clientX, v0 = view.slice(), W = cv.getBoundingClientRect().width;
  let moved = false;
  wrap.setPointerCapture(ev.pointerId);
  const mv = (e) => {
    if (Math.abs(e.clientX - x0) > 3) moved = true;
    if (!moved) return;
    const dt = ((e.clientX - x0) / W) * (v0[1] - v0[0]);
    view = clampView([v0[0] - dt, v0[1] - dt]); drawAll();
  };
  const up = (e) => {
    wrap.removeEventListener("pointermove", mv); wrap.removeEventListener("pointerup", up);
    if (!moved) seek(plotTime(e, cv));
  };
  wrap.addEventListener("pointermove", mv); wrap.addEventListener("pointerup", up);
});
$("rvPlots").addEventListener("dblclick", (ev) => { if (ev.target.closest(".rv-cvwrap") && D) { view = full.slice(); drawAll(); } });
$("rvOverview").addEventListener("pointerdown", (ev) => {
  if (!D) return;
  const el = $("rvOverview"), r = el.getBoundingClientRect();
  const go = (e) => {
    const t = full[0] + ((e.clientX - r.left) / r.width) * (full[1] - full[0]);
    const w = view[1] - view[0];
    if (t < view[0] || t > view[1]) view = clampView([t - w / 2, t + w / 2]);
    seek(t); drawAll();
  };
  go(ev); el.setPointerCapture(ev.pointerId);
  el.onpointermove = (e) => { if (e.buttons) go(e); };
});
$("rvPlay").onclick = () => (playing ? pause() : play());
function seg(id, cur0, onSet) {
  const box = $(id);
  for (const b of box.querySelectorAll("button")) {
    b.classList.toggle("on", b.dataset.v === String(cur0));
    b.onclick = () => { for (const x of box.querySelectorAll("button")) x.classList.toggle("on", x === b); onSet(b.dataset.v); };
  }
}
seg("rvSpeed", 1, (v) => { speed = +v; });
seg("rvImuMode", imuMode, (v) => { imuMode = v; LS.set("imuMode", v); if (D) buildTracks(); });
seg("rvEmgMode", emgMode, (v) => { emgMode = v; LS.set("emgMode", v); if (D) buildTracks(); });
seg("rvEmgScale", emgScale, (v) => { emgScale = v; LS.set("emgScale", v); if (D) buildTracks(); });
seg("rvView", "side", (v) => spine && spine.setView(v));
$("rvGateAnyway").onclick = () => { anyway = true; derive(); buildTracks(); };
$("rvSearch").oninput = renderList;
$("rvShowCal").onchange = renderList;
$("rvRefresh").onclick = loadList;

document.addEventListener("keydown", (e) => {
  if (e.target.closest("input, select, textarea") || !D) return;
  const dt = D.imu && D.imu.t.length > 1 ? (D.imu.t[D.imu.t.length - 1] - D.imu.t[0]) / (D.imu.t.length - 1) : 0.01;
  if (e.code === "Space") { e.preventDefault(); playing ? pause() : play(); }
  else if (e.key === "ArrowRight") { e.preventDefault(); seek(cur + (e.shiftKey ? 1 : dt)); }
  else if (e.key === "ArrowLeft") { e.preventDefault(); seek(cur - (e.shiftKey ? 1 : dt)); }
  else if (e.key === "Home") { seek(full[0]); }
  else if (e.key === "End") { seek(full[1]); }
});
new ResizeObserver(() => D && drawAll()).observe($("rvPlots"));

// ---------------- start ----------------
spineReady = createSpine($("rvThree")).then((s) => { spine = s; $("rvLoad3d").classList.add("hidden"); if (D) updateSpine(); })
  .catch((e) => { $("rvLoad3d").textContent = "3D view failed: " + e.message; });
try { TASKS = await (await fetch("/static/tasks.json", { cache: "no-store" })).json(); } catch { TASKS = []; }
await loadList();
const want = new URLSearchParams(location.search).get("rel") || LS.get("last", "");
if (want && SESS.some((s) => s.rel === want)) openRec(want);
