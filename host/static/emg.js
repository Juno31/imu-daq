// EMG normalization page: rest baseline (offset, noise), MVC / RVC captures, reference values per channel.
// Each capture is a normal short recording (task emg_rest / emg_mvc / emg_rvc) made with /api/start and
// /api/stop, so the raw DAQ data are kept. host/emgnorm.py analyses them; the chosen references are
// saved to <data>/<date>/<subject>/emg_norm.json and copied into every later trial.
"use strict";
const $ = (id) => document.getElementById(id);
const LS = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { } },
};
const KIND = { rest: "Rest", mvc: "MVC", rvc: "RVC" };
const CAP = {
  mvc: "Maximal voluntary contraction against manual or strap resistance: build up over about 1 s, push as hard as possible, hold, then relax. Usually 3 repeats per muscle with 1–2 min rest.",
  rvc: "Reference voluntary contraction: a fixed, repeatable submaximal task (e.g. holding the trunk horizontal, or a set load). Use the same task for every participant.",
};

let S = null, TR = [], NORM = null, REFS = {}, CH = [], running = null;
let opt = { win: LS.get("eg.win", 0.5), hold: LS.get("eg.hold", 0.6), filter: LS.get("eg.filter", true), own: LS.get("eg.own", false) };
let kind = "mvc", secs = 5, restSecs = 5;
const excluded = new Set(LS.get("eg.excluded", []));
const chosen = {};                                   // channel -> "MVC" | "RVC" picked by the user

// ---------------- helpers ----------------
function toast(text, bad) {
  const el = document.createElement("div");
  el.className = "tst" + (bad ? " bad" : "");
  el.innerHTML = `<span class="ti">${bad ? "!" : "✓"}</span><span class="tm"></span><span></span><button class="tx">×</button>`;
  el.querySelector(".tm").textContent = text;
  el.querySelector(".tx").onclick = () => el.remove();
  $("toasts").appendChild(el);
  setTimeout(() => el.remove(), bad ? 8000 : 4000);
}
async function api(path, body) {
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
    const j = await r.json();
    if (!j.ok) toast(j.error || "Request failed", true);
    return j;
  } catch (e) { toast("Server not reachable", true); return { ok: false }; }
}
const fmt = (v, d = 2) => v == null || !isFinite(v) ? "—" : v.toFixed(d);
const who = () => ({ date: $("egDate").value.trim(), subject: $("egSubject").value.trim() });
const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

function seg(id, cur, onSet) {
  const box = $(id);
  for (const b of box.querySelectorAll("button")) {
    b.classList.toggle("on", b.dataset.v === String(cur));
    b.onclick = () => { for (const x of box.querySelectorAll("button")) x.classList.toggle("on", x === b); onSet(b.dataset.v); };
  }
}

// ---------------- participant ----------------
function initWho() {
  $("egDate").value = LS.get("imudaq.f.testDate", "") || "";
  $("egSubject").value = LS.get("imudaq.f.subject", "") || "";
  for (const id of ["egDate", "egSubject"]) $(id).addEventListener("change", () => {
    LS.set("imudaq.f.testDate", $("egDate").value); LS.set("imudaq.f.subject", $("egSubject").value);   // same participant as the main page
    for (const k of Object.keys(chosen)) delete chosen[k];
    loadTrials();
  });
}

// ---------------- trials + analysis ----------------
async function loadTrials() {
  const w = who();
  if (!w.subject) { TR = []; NORM = null; $("egFolder").textContent = "Enter the subject"; $("egFolder").className = "takepath warn"; render(); return; }
  const q = new URLSearchParams({ date: w.date, subject: w.subject, win: opt.win, hold: opt.hold, filter: opt.filter ? 1 : 0 });
  $("egTrialsHint").textContent = "analysing…";
  let j;
  try { j = await (await fetch("/api/emgcal/trials?" + q)).json(); } catch { j = { ok: false, error: "server not reachable" }; }
  if (!j.ok) { toast(j.error || "Cannot load trials", true); $("egTrialsHint").textContent = ""; return; }
  TR = j.trials || []; NORM = j.norm || null;
  $("egFolder").textContent = "→ " + (j.rel || "").replace(/\\/g, "/") + "/"; $("egFolder").className = "takepath mono";
  if (NORM && NORM.channels) for (const [n, c] of Object.entries(NORM.channels)) if (!chosen[n] && c.ref_kind) chosen[n] = c.ref_kind;
  render();
}

function channels() {
  const m = new Map();
  for (const t of TR) for (const [n, c] of Object.entries((t.metrics || {}).channels || {})) if (!m.has(n)) m.set(n, c);
  if (S && S.emg && S.emg.chans) for (const c of S.emg.chans) if (c.name && !m.has(c.name)) m.set(c.name, c);
  return [...m.entries()].map(([name, c]) => ({ name, chan: c.chan, muscle: c.muscle || "", side: c.side || "", group: c.group || "" }));
}

function computeRefs() {
  const inc = TR.filter((t) => !excluded.has(t.rel) && t.metrics);
  const rest = inc.filter((t) => t.kind === "rest").slice(-1)[0] || null;
  const out = {};
  for (const ch of CH) {
    const fits = (t) => !opt.own || !t.target || t.target === "all" || t.target === ch.group;
    const mv = inc.filter((t) => t.kind === "mvc" && fits(t) && t.metrics.channels[ch.name]);
    const rv = inc.filter((t) => t.kind === "rvc" && fits(t) && t.metrics.channels[ch.name]);
    const best = mv.reduce((a, t) => (!a || t.metrics.channels[ch.name].peak_mv > a.metrics.channels[ch.name].peak_mv ? t : a), null);
    const r = rest && rest.metrics.channels[ch.name];
    const mvc = best ? best.metrics.channels[ch.name].peak_mv : null;
    const rvc = rv.length ? rv.reduce((a, t) => a + t.metrics.channels[ch.name].hold_mv, 0) / rv.length : null;
    let k = chosen[ch.name] || (mvc != null ? "MVC" : rvc != null ? "RVC" : null);
    if (k === "MVC" && mvc == null) k = rvc != null ? "RVC" : null;
    if (k === "RVC" && rvc == null) k = mvc != null ? "MVC" : null;
    const ref = k === "MVC" ? mvc : k === "RVC" ? rvc : null;
    const noise = r ? r.rms_mv : null;
    const flags = [];
    const all = inc.filter((t) => t.metrics.channels[ch.name]);
    if (all.some((t) => t.metrics.channels[ch.name].floating)) flags.push(["bad", "not connected (copies SYNC)"]);
    const refTrials = k === "MVC" ? [best] : k === "RVC" ? rv : [];
    if (refTrials.some((t) => t && t.metrics.channels[ch.name].clip_pct > 0.1)) flags.push(["bad", "clipping: lower the amplifier gain"]);
    const snr = ref && noise ? 20 * Math.log10(ref / noise) : null;
    if (snr != null && snr < 20) flags.push(["warn", `low SNR (${snr.toFixed(0)} dB)`]);
    if (!rest) flags.push(["warn", "no rest recording"]);
    if (!ref) flags.push(["warn", "no MVC / RVC yet"]);
    out[ch.name] = {
      chan: ch.chan, muscle: ch.muscle, side: ch.side, group: ch.group,
      offset_mv: r ? r.offset_mv : null, noise_mv: noise, rest_trial: rest ? rest.name : null,
      mvc_mv: mvc, mvc_trial: best ? best.name : null, mvc_t: best ? best.metrics.channels[ch.name].peak_t : null,
      rvc_mv: rvc, rvc_trials: rv.map((t) => t.name), ref_kind: k, ref_mv: ref, snr_db: snr == null ? null : +snr.toFixed(1),
      flags: flags.map((f) => f[1]), _flags: flags,
    };
  }
  return out;
}

// ---------------- rendering ----------------
function render() {
  CH = channels();
  REFS = computeRefs();
  renderTargets(); renderRest(); renderTrials(); renderRefs();
  renderSrc(); buildRows(); sigText(); drawSignals();
}

function renderTargets() {
  const sel = $("egTarget"), cur = sel.value || LS.get("eg.target", "all");
  const groups = [];
  for (const c of CH) if (c.group && !groups.find((g) => g.group === c.group)) groups.push({ group: c.group, muscles: new Set() });
  for (const c of CH) { const g = groups.find((x) => x.group === c.group); if (g) g.muscles.add(c.muscle); }
  const opts = [["all", "All muscles"], ...groups.map((g) => [g.group, `${g.group} (${[...g.muscles].join(", ")})`])];
  const html = opts.map(([v, l]) => `<option value="${esc(v)}">${esc(l)}</option>`).join("");
  if (sel.dataset.html !== html) { sel.innerHTML = html; sel.dataset.html = html; }
  sel.value = opts.some((o) => o[0] === cur) ? cur : "all";
}

function renderRest() {
  const rest = TR.filter((t) => t.kind === "rest" && !excluded.has(t.rel)).slice(-1)[0];
  const pill = $("egRestPill"), box = $("egRestSum");
  if (!rest) { pill.textContent = "NOT DONE"; pill.className = "pill"; box.innerHTML = ""; return; }
  pill.textContent = "DONE " + (rest.started || "").slice(11, 16); pill.className = "pill ok";
  const v = Object.values(rest.metrics ? rest.metrics.channels : {});
  const fl = v.filter((c) => c.floating).length;
  const noise = v.map((c) => c.rms_mv * 1000);
  box.innerHTML = "";
  const line = document.createElement("div");
  line.className = "r";
  line.innerHTML = `<b class="ok">✓</b><span></span>`;
  line.lastChild.textContent = v.length ? `${rest.name} · noise ${Math.min(...noise).toFixed(1)}–${Math.max(...noise).toFixed(1)} µV RMS over ${v.length} channels` : rest.name;
  box.appendChild(line);
  if (fl) {
    const a = document.createElement("div");
    a.className = "calalert bad"; a.innerHTML = `<i>✗</i><span></span>`;
    a.lastChild.textContent = `${fl === v.length ? "All" : fl} EMG channel${fl > 1 ? "s" : ""} not connected: they copy the SYNC pulses. Connect the EMG amplifier outputs.`;
    box.appendChild(a);
  }
}

function renderTrials() {
  const t = $("egTrials");
  const mvcN = TR.filter((x) => x.kind !== "rest" && !excluded.has(x.rel)).length;
  $("egMvcPill").textContent = `${mvcN} TRIAL${mvcN === 1 ? "" : "S"}`; $("egMvcPill").className = "pill" + (mvcN ? " ok" : "");
  $("egTrialsHint").textContent = TR.length ? `${TR.length} recordings · window ${opt.win * 1000} ms${opt.filter ? " · 20–450 Hz" : ""}` : "";
  if (!TR.length) { t.innerHTML = `<tbody><tr><td class="eg-empty">No EMG calibration recordings for this subject yet.</td></tr></tbody>`; return; }
  const used = (name, ch) => { const r = REFS[ch.name]; if (!r) return false; return r.ref_kind === "MVC" ? r.mvc_trial === name : r.ref_kind === "RVC" ? r.rvc_trials.includes(name) : r.rest_trial === name; };
  let h = `<thead><tr><th></th><th>Trial</th><th>Type</th><th>Target</th><th>Time</th>${CH.map((c) => `<th title="${esc(c.muscle)} ${esc(c.side)}">${esc(c.name)}<small>${esc(c.side)}</small></th>`).join("")}</tr></thead><tbody>`;
  for (const tr of TR.slice().reverse()) {
    const off = excluded.has(tr.rel), m = tr.metrics ? tr.metrics.channels : {};
    h += `<tr data-rel="${esc(tr.rel)}" class="${off ? "off" : ""} k-${tr.kind}${tr.rel === src ? " sel" : ""}"><td><input type="checkbox" data-rel="${esc(tr.rel)}" ${off ? "" : "checked"}></td>` +
      `<td class="mono">${esc(tr.name.replace(/^sub\d+_/, ""))}${tr.aborted ? ' <span class="pill warn">ABORTED</span>' : ""}</td>` +
      `<td><span class="pill k">${KIND[tr.kind]}</span></td><td class="tg">${esc(tr.target === "all" || !tr.target ? "All" : tr.target)}</td>` +
      `<td class="mono">${esc((tr.started || "").slice(11, 16))}</td>`;
    for (const c of CH) {
      const v = m[c.name];
      if (!v) { h += `<td class="v">—</td>`; continue; }
      const val = tr.kind === "rest" ? v.rms_mv : tr.kind === "mvc" ? v.peak_mv : v.hold_mv;
      const cls = [v.floating ? "bad" : "", v.clip_pct > 0.1 ? "clip" : "", !off && used(tr.name, c) ? "used" : ""].join(" ");
      h += `<td class="v ${cls}" title="${v.floating ? "not connected · " : ""}clip ${v.clip_pct}%">${!off && used(tr.name, c) ? "★ " : ""}${fmt(val, val < 1 ? 3 : 2)}</td>`;
    }
    h += `</tr>`;
  }
  t.innerHTML = h + `</tbody>`;
  for (const row of t.querySelectorAll("tbody tr[data-rel]")) row.onclick = (e) => { if (!e.target.closest("input")) selectSrc(row.dataset.rel); };
  for (const cb of t.querySelectorAll("input[type=checkbox]")) cb.onchange = () => {
    if (cb.checked) excluded.delete(cb.dataset.rel); else excluded.add(cb.dataset.rel);
    LS.set("eg.excluded", [...excluded].slice(-300)); render();
  };
}

function renderRefs() {
  const t = $("egRefs");
  if (!CH.length) { t.innerHTML = `<tbody><tr><td class="eg-empty">No EMG channels: enable EMG channels in the DAQ settings.</td></tr></tbody>`; updateSaved(); return; }
  let h = `<thead><tr><th>Channel</th><th>Muscle</th><th>Offset<small>mV</small></th><th>Noise<small>µV RMS</small></th><th>MVC<small>mV</small></th><th>RVC<small>mV</small></th><th>Use</th><th>Reference<small>mV</small></th><th>SNR<small>dB</small></th><th>Check</th></tr></thead><tbody>`;
  for (const c of CH) {
    const r = REFS[c.name], bad = r._flags.some((f) => f[0] === "bad"), warn = r._flags.length && !bad;
    h += `<tr><td class="mono"><b>${esc(c.name)}</b></td><td>${esc(c.muscle)} <span class="side">${esc(c.side)}</span></td>` +
      `<td class="v">${fmt(r.offset_mv, 2)}</td><td class="v">${r.noise_mv == null ? "—" : (r.noise_mv * 1000).toFixed(1)}</td>` +
      `<td class="v" title="${esc(r.mvc_trial || "")}">${fmt(r.mvc_mv, 3)}</td><td class="v" title="${esc(r.rvc_trials.join(", "))}">${fmt(r.rvc_mv, 3)}</td>` +
      `<td><select data-ch="${esc(c.name)}" ${r.mvc_mv == null && r.rvc_mv == null ? "disabled" : ""}>` +
      `<option value="MVC" ${r.ref_kind === "MVC" ? "selected" : ""} ${r.mvc_mv == null ? "disabled" : ""}>MVC</option>` +
      `<option value="RVC" ${r.ref_kind === "RVC" ? "selected" : ""} ${r.rvc_mv == null ? "disabled" : ""}>RVC</option></select></td>` +
      `<td class="v ref">${fmt(r.ref_mv, 3)}</td><td class="v ${r.snr_db != null && r.snr_db < 20 ? "warn" : ""}">${r.snr_db == null ? "—" : r.snr_db.toFixed(0)}</td>` +
      `<td class="chk ${bad ? "bad" : warn ? "warn" : "ok"}">${bad || warn ? "" : "✓ OK"}${esc(r.flags.join(" · "))}</td></tr>`;
  }
  t.innerHTML = h + `</tbody>`;
  for (const s of t.querySelectorAll("select")) s.onchange = () => { chosen[s.dataset.ch] = s.value; render(); };
  updateSaved();
}

function clean(refs) {
  const o = {};
  for (const [n, r] of Object.entries(refs)) { const { _flags, ...rest } = r; o[n] = rest; }
  return o;
}
function updateSaved() {
  const pill = $("egSavedPill"), hint = $("egSaveHint");
  const ready = CH.filter((c) => REFS[c.name] && REFS[c.name].ref_mv).length;
  $("egSave").disabled = !ready || !!running;
  if (!NORM) { pill.textContent = "NOT SAVED"; pill.className = "pill"; }
  else {
    const changed = CH.some((c) => {
      const a = REFS[c.name], b = (NORM.channels || {})[c.name];
      return !b || (a.ref_mv || 0) && Math.abs((a.ref_mv - (b.ref_mv || 0)) / a.ref_mv) > 0.005 || a.ref_kind !== b.ref_kind;
    });
    pill.textContent = changed ? "CHANGED" : "SAVED " + (NORM.saved || "").slice(11, 16);
    pill.className = "pill " + (changed ? "warn" : "ok");
  }
  hint.textContent = ready ? `${ready} of ${CH.length} channels have a reference. Saved to the subject folder; later trials get a copy, and Review shows %${"MVC"}.`
    : "Record a rest baseline and at least one MVC or RVC first.";
}

// ---------------- signal view (main UI): live scope or a recorded trial ----------------
let src = "live", scaleMode = LS.get("eg.scale", "each"), show = LS.get("eg.show", "both");
const liveMax = {};                                    // decaying autoscale per channel (mV)
const cssv = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

function renderSrc() {
  if (src !== "live" && !TR.some((t) => t.rel === src)) src = "live";
  const items = [["live", "● Live", "live", "Live signals from the DAQ"], ...TR.slice().reverse().map((t) =>
    [t.rel, `${KIND[t.kind]} ${String(t.trial ?? "").padStart(2, "0")}`, t.kind,
      `${t.name} · ${(t.started || "").slice(11, 16)}${t.target && t.target !== "all" ? " · " + t.target : ""}`])];
  const box = $("egSrc");
  box.innerHTML = items.map(([v, l, k, title]) =>
    `<button class="eg-chip k-${k}${v === src ? " on" : ""}${excluded.has(v) ? " off" : ""}" data-v="${esc(v)}" title="${esc(title)}">${esc(l)}</button>`).join("");
  for (const b of box.children) b.onclick = () => selectSrc(b.dataset.v);
}
function selectSrc(v) {
  if (running && v !== "live") return;
  src = v; renderSrc(); buildRows(); sigText(); drawSignals();
  for (const tr of document.querySelectorAll("#egTrials tbody tr[data-rel]")) tr.classList.toggle("sel", tr.dataset.rel === src);
}

function buildRows() {
  const box = $("egRows"), key = CH.map((c) => c.name).join();
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  if (!CH.length) { box.innerHTML = `<div class="eg-none">No EMG channels. Add EMG channels in the DAQ settings on the Acquisition page.</div>`; return; }
  box.innerHTML = CH.map((c) => `<div class="eg-row" data-ch="${esc(c.name)}">
    <div class="eg-lab"><b>${esc(c.name)}</b><small>${c.side ? `<span class="side">${esc(c.side)}</span> ` : ""}${esc(c.muscle)}</small></div>
    <div class="eg-cv"><canvas></canvas></div>
    <div class="eg-val"><b>—</b><small></small><div class="lvl"><i></i></div></div></div>`).join("");
}

function sizeCanvas(c) {
  const dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
  if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
  const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  return [g, W, H];
}
function envStroke(g, lo, hi, W, Y, color) {           // one vertical stroke per point (min..max)
  g.strokeStyle = color; g.lineWidth = Math.max(1, W / lo.length); g.beginPath();
  for (let i = 0; i < lo.length; i++) { const x = (i + 0.5) / lo.length * W; g.moveTo(x, Y(hi[i])); g.lineTo(x, Y(lo[i]) + 0.5); }
  g.stroke();
}
function setVal(row, big, sub, pct, cls) {
  row.querySelector(".eg-val b").textContent = big;
  row.querySelector(".eg-val small").textContent = sub;
  const lvl = row.querySelector(".lvl");
  lvl.style.visibility = pct == null ? "hidden" : "";
  lvl.firstChild.style.width = Math.min(100, pct || 0) + "%";
  lvl.classList.toggle("over", (pct || 0) > 100);
  row.className = "eg-row" + (cls ? " " + cls : "");
}
const fmtMv = (v) => (v >= 10 ? v.toFixed(0) : v >= 1 ? v.toFixed(1) : v >= 0.1 ? v.toFixed(2) : v.toFixed(3)) + " mV";

function drawSignals() {
  if (!CH.length) { drawAxis(null); return; }
  if (src === "live") drawLive(); else drawTrial(TR.find((t) => t.rel === src));
}

function drawLive() {
  const live = S && S.emg && S.emg.chans ? S.emg.chans : [];
  const sec = S && S.emg ? S.emg.seconds || 2 : 2;
  const data = {};
  for (const ch of CH) {
    const c = live.find((x) => x.name === ch.name);
    if (!c || !c.lo || !c.lo.length) continue;
    let mid = 0; for (let i = 0; i < c.lo.length; i++) mid += (c.lo[i] + c.hi[i]) / 2; mid /= c.lo.length;
    const lo = c.lo.map((v) => (v - mid) * 1e3), hi = c.hi.map((v) => (v - mid) * 1e3);
    let amp = 0; for (let i = 0; i < lo.length; i++) amp = Math.max(amp, -lo[i], hi[i]);
    liveMax[ch.name] = Math.max(amp * 1.1, (liveMax[ch.name] || 0) * 0.97, 0.02);
    data[ch.name] = { c, lo, hi };
  }
  const shared = Math.max(0.02, ...Object.keys(data).map((n) => liveMax[n]));
  for (const row of $("egRows").children) {
    const n = row.dataset.ch, d = data[n], r = REFS[n];
    if (!n) continue;
    const [g, W, H] = sizeCanvas(row.querySelector("canvas"));
    g.strokeStyle = cssv("--border"); g.beginPath(); g.moveTo(0, H / 2); g.lineTo(W, H / 2); g.stroke();
    if (!d) { setVal(row, "—", "no live data", null, "nodata"); continue; }
    const m = scaleMode === "same" ? shared : liveMax[n], Y = (v) => H / 2 - (v / m) * (H / 2 - 3);
    envStroke(g, d.lo, d.hi, W, Y, d.c.clip ? cssv("--bad") : cssv("--accent"));
    g.fillStyle = cssv("--text-3"); g.font = "10px " + cssv("--mono"); g.fillText(`±${fmtMv(m)}`, 4, 11);
    const v = d.c.rms_mv, pct = r && r.ref_mv && v != null ? (v / r.ref_mv) * 100 : null;
    setVal(row, pct != null ? `${pct.toFixed(0)} %` : v != null ? v.toFixed(3) : "—",
      pct != null ? `of ${r.ref_kind} · ${v.toFixed(3)} mV` : "mV RMS (100 ms)", pct,
      [d.c.floating ? "floating" : "", d.c.clip ? "clip" : ""].join(" ").trim());
  }
  drawAxis({ t0: -sec, t1: 0, label: (t) => (t === 0 ? "now" : `${t} s`) });
}

function drawTrial(t) {
  if (!t || !t.metrics) {
    for (const row of $("egRows").children) { if (!row.dataset.ch) continue; sizeCanvas(row.querySelector("canvas")); setVal(row, "—", "no DAQ data", null, "nodata"); }
    drawAxis(null); return;
  }
  const M = t.metrics, dur = M.seconds;
  const amp = (m) => {
    if (!m) return 0.02;
    let a = 0;
    if (show === "both" && m.env_lo) for (let i = 0; i < m.env_lo.length; i++) a = Math.max(a, -m.env_lo[i], m.env_hi[i]);
    for (const v of m.curve) a = Math.max(a, v);
    return Math.max(a * 1.05, 0.02);
  };
  const shared = Math.max(...CH.map((c) => amp(M.channels[c.name])));
  for (const row of $("egRows").children) {
    const n = row.dataset.ch, m = M.channels[n], r = REFS[n];
    if (!n) continue;
    const [g, W, H] = sizeCanvas(row.querySelector("canvas"));
    if (!m) { setVal(row, "—", "not in this trial", null, "nodata"); continue; }
    const top = scaleMode === "same" ? shared : amp(m);
    const X = (sec) => (sec / dur) * W;
    const both = show === "both" && m.env_lo;
    const base = both ? H / 2 : H - 2, half = both ? H / 2 - 3 : H - 5;
    const Y = (v) => base - (v / top) * half;
    // analysed part: MVC window with the highest RMS, or the RVC hold
    let a = null, b = null;
    if (t.kind === "mvc") { a = m.peak_t - M.win_s / 2; b = m.peak_t + M.win_s / 2; }
    else if (t.kind === "rvc") { a = dur * (1 - M.hold) / 2; b = dur * (1 + M.hold) / 2; }
    if (a != null) { g.fillStyle = cssv(t.kind === "mvc" ? "--bad-soft" : "--accent-soft"); g.fillRect(X(a), 0, Math.max(2, X(b) - X(a)), H); }
    g.strokeStyle = cssv("--border"); g.lineWidth = 1; g.beginPath(); g.moveTo(0, base); g.lineTo(W, base); g.stroke();
    if (both) envStroke(g, m.env_lo, m.env_hi, W, Y, m.floating ? cssv("--bad") : cssv("--text-3"));
    // RMS curve (100 ms window, centred)
    g.strokeStyle = cssv("--accent"); g.lineWidth = 1.8; g.beginPath();
    m.curve.forEach((v, i) => { const x = X(i * M.curve_dt + 0.05), y = Y(v); if (i) g.lineTo(x, y); else g.moveTo(x, y); });
    g.stroke();
    // current reference level
    if (r && r.ref_mv && r.ref_mv <= top * 1.02 && t.kind !== "rest") {
      g.save(); g.strokeStyle = cssv("--ok"); g.setLineDash([5, 4]); g.lineWidth = 1.2; g.beginPath();
      g.moveTo(0, Y(r.ref_mv)); g.lineTo(W, Y(r.ref_mv)); g.stroke();
      g.fillStyle = cssv("--ok"); g.font = "10px " + cssv("--mono"); g.textAlign = "right";
      g.fillText(`${r.ref_kind} ${fmtMv(r.ref_mv)}`, W - 4, Math.max(10, Y(r.ref_mv) - 3)); g.restore();
    }
    g.fillStyle = cssv("--text-3"); g.font = "10px " + cssv("--mono"); g.fillText(both ? `±${fmtMv(top)}` : fmtMv(top), 4, 11);
    const val = t.kind === "rest" ? m.rms_mv : t.kind === "mvc" ? m.peak_mv : m.hold_mv;
    const used = r && (t.kind === "mvc" ? r.ref_kind === "MVC" && r.mvc_trial === t.name
      : t.kind === "rvc" ? r.ref_kind === "RVC" && r.rvc_trials.includes(t.name) : r.rest_trial === t.name);
    const pct = t.kind !== "rest" && r && r.ref_mv ? (val / r.ref_mv) * 100 : null;
    setVal(row, t.kind === "rest" ? (val * 1000).toFixed(1) : val.toFixed(3),
      (t.kind === "rest" ? "µV noise RMS" : t.kind === "mvc" ? "mV peak RMS" : "mV hold RMS") +
      (used ? " · ★ reference" : pct != null ? ` · ${pct.toFixed(0)} % of ref` : ""),
      pct, [m.floating ? "floating" : "", m.clip_pct > 0.1 ? "clip" : "", used ? "used" : ""].join(" ").trim());
  }
  drawAxis({ t0: 0, t1: dur, label: (sec) => `${sec} s` });
}

function drawAxis(ax) {
  const [g, W] = sizeCanvas($("egAxis"));
  if (!ax) return;
  const span = ax.t1 - ax.t0, raw = span / Math.max(1, W / 80), p = Math.pow(10, Math.floor(Math.log10(raw)));
  const st = [1, 2, 5, 10].map((k) => k * p).find((k) => k >= raw);
  g.fillStyle = cssv("--text-3"); g.font = "10.5px " + cssv("--mono"); g.textAlign = "center";
  for (let sec = Math.ceil(ax.t0 / st) * st; sec <= ax.t1 + 1e-9; sec += st) {
    const x = ((sec - ax.t0) / span) * W;
    g.fillText(ax.label(Math.round(sec * 100) / 100), Math.min(W - 16, Math.max(16, x)), 12);
  }
}

function sigText() {
  const t = src === "live" ? null : TR.find((x) => x.rel === src);
  $("egSigSub").textContent = !t ? "Live · last 2 s per channel · raw, offset removed · value = RMS of the last 100 ms (unfiltered)"
    : `${t.name} · ${KIND[t.kind]}${t.target && t.target !== "all" ? " · " + t.target : t.kind !== "rest" ? " · all muscles" : ""}` +
      `${t.metrics ? " · " + t.metrics.seconds.toFixed(1) + " s" : ""}${excluded.has(t.rel) ? " · excluded" : ""}`;
  $("egSigCap").textContent = !t ? "During a capture this view stays live. Level bars show % of the reference once one is set."
    : t.kind === "mvc" ? `Grey = raw signal, blue = 100 ms RMS (${opt.filter ? "20–450 Hz" : "unfiltered"}). Red band = the ${opt.win * 1000} ms window with the highest RMS (the MVC value). Green dashed = current reference.`
    : t.kind === "rvc" ? `Grey = raw signal, blue = 100 ms RMS. Blue band = the middle ${opt.hold * 100} % of the hold used for the RVC value. Green dashed = current reference.`
    : "Grey = raw signal, blue = 100 ms RMS. The whole rest recording gives the offset and noise.";
}

// ---------------- recording a capture ----------------
async function capture(k) {
  const w = who();
  if (!w.subject) { toast("Enter the subject first", true); $("egSubject").focus(); return; }
  if (!S || S.state !== "idle") { toast(S ? `Not now: ${S.state}` : "Server not connected", true); return; }
  if (!(S.daq && S.daq.cfg && S.daq.cfg.enabled)) { toast("Recording with the DAQ is off: turn on Record DAQ in Settings on the Acquisition page", true); return; }
  const task = "emg_" + k, dur = k === "rest" ? restSecs : secs, target = k === "rest" ? "" : $("egTarget").value;
  let n = 1;
  try { n = (await (await fetch("/api/next_trial?" + new URLSearchParams({ date: w.date, subject: w.subject, task }))).json()).trial || 1; } catch { }
  const tLabel = target && target !== "all" ? target : k === "rest" ? "" : "all muscles";
  const j = await api("/api/start", { subject: w.subject, date: w.date, task, task_name: `EMG ${KIND[k]}`, trial: n,
    notes: `EMG ${KIND[k]}${tLabel ? " · " + tLabel : ""}`, emgcal: { kind: k, target: target || "", seconds: dur } });
  if (!j.ok) return;
  running = { k, dur, t0: performance.now(), session: j.session };
  selectSrc("live");
  $("egRun").classList.remove("hidden");
  lockUi(true);
  tickRun();
}
function phase(k, t, dur) {
  if (k === "rest") return ["Relax completely · stay still", "rest"];
  if (k === "rvc") return t < 1 ? ["Get into the reference position", "ramp"] : t < dur - 0.5 ? ["Hold steady", "push"] : ["Relax", "rest"];
  return t < 1 ? ["Build up…", "ramp"] : t < dur - 0.5 ? ["PUSH — maximum!", "push"] : ["Relax", "rest"];
}
function tickRun() {
  if (!running) return;
  const t = (performance.now() - running.t0) / 1000, left = running.dur - t;
  const [txt, cls] = phase(running.k, t, running.dur);
  $("egPhase").textContent = txt; $("egPhase").className = "eg-phase " + cls;
  $("egBar").style.width = Math.min(100, (t / running.dur) * 100) + "%";
  $("egLeft").textContent = left > 0 ? `${left.toFixed(1)} s` : "saving…";
  if (left <= 0) { finishRun(); return; }
  setTimeout(tickRun, 50);
}
async function finishRun() {
  if (!running || running.stopping) return;
  running.stopping = true;
  await api("/api/stop", {});
  const t0 = Date.now();                                   // wait until the server is idle again
  while (S && S.state !== "idle" && Date.now() - t0 < 15000) await new Promise((r) => setTimeout(r, 200));
  running = null;
  $("egRun").classList.add("hidden");
  lockUi(false);
  await loadTrials();
  const last = TR.slice().sort((a, b) => (a.started || "").localeCompare(b.started || "")).pop();
  if (last) selectSrc(last.rel);
  toast("Saved: " + (last ? last.name : "recording"));
}
function lockUi(on) {
  for (const id of ["egRestBtn", "egRecBtn", "egSave", "egDate", "egSubject"]) $(id).disabled = on;
}

// ---------------- live status ----------------
function onStatus(s) {
  S = s;
  $("connDot").className = "dot " + (s.port ? "on" : "");
  $("connText").textContent = s.port || "No board";
  const b = $("stateBadge");
  b.textContent = s.state === "recording" ? "REC" : (s.state || "—").toUpperCase();
  b.className = "badge " + (s.state === "recording" ? "rec" : s.state === "idle" ? "idle" : "busy");
  if (!running) {
    const ok = s.state === "idle";
    $("egRestBtn").disabled = !ok; $("egRecBtn").disabled = !ok;
  }
  const ban = $("banner");
  if (s.state === "recording" && !running) { ban.textContent = "A recording is running from the Acquisition page: captures here are disabled until it stops."; ban.className = "banner warn"; }
  else if (!(s.daq && s.daq.cfg && s.daq.cfg.enabled)) { ban.textContent = "Record DAQ is off (Settings on the Acquisition page): EMG cannot be captured."; ban.className = "banner warn"; }
  else ban.className = "banner hidden";
  if (!CH.length && s.emg && s.emg.chans) render();
  if (src === "live") drawSignals();
}
function connect() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => { const m = JSON.parse(ev.data); if (m.s) onStatus(m.s); };
  es.onerror = () => { $("connDot").className = "dot"; $("connText").textContent = "Server offline"; };
}

// ---------------- wiring ----------------
seg("egWin", opt.win, (v) => { opt.win = +v; LS.set("eg.win", opt.win); loadTrials(); });
seg("egHold", opt.hold, (v) => { opt.hold = +v; LS.set("eg.hold", opt.hold); loadTrials(); });
seg("egRestSecs", restSecs, (v) => { restSecs = +v; });
seg("egSecs", secs, (v) => { secs = +v; });
seg("egKind", kind, (v) => {
  kind = v; $("egRecBtn").textContent = "● Record " + KIND[v]; $("egKindCap").textContent = CAP[v];
});
$("egKindCap").textContent = CAP[kind];
$("egFilter").checked = opt.filter; $("egOwnGroup").checked = opt.own;
$("egFilter").onchange = () => { opt.filter = $("egFilter").checked; LS.set("eg.filter", opt.filter); loadTrials(); };
$("egOwnGroup").onchange = () => { opt.own = $("egOwnGroup").checked; LS.set("eg.own", opt.own); render(); };
$("egTarget").onchange = () => LS.set("eg.target", $("egTarget").value);
$("egRestBtn").onclick = () => capture("rest");
$("egRecBtn").onclick = () => capture(kind);
$("egAbort").onclick = () => { if (running) running.dur = (performance.now() - running.t0) / 1000; };
$("egReload").onclick = loadTrials;
seg("egScale", scaleMode, (v) => { scaleMode = v; LS.set("eg.scale", v); drawSignals(); });
seg("egShow", show, (v) => { show = v; LS.set("eg.show", v); drawSignals(); });
new ResizeObserver(() => drawSignals()).observe($("egRows"));
$("egSave").onclick = async () => {
  const w = who();
  const j = await api("/api/emgcal/save", { date: w.date, subject: w.subject, analysis: { win_s: opt.win, hold: opt.hold, band_pass: opt.filter, own_group_only: opt.own },
    channels: clean(REFS) });
  if (j.ok) { NORM = j.norm; toast("Saved " + j.path); render(); }
};
document.addEventListener("keydown", (e) => { if (e.key === "Escape" && running) $("egAbort").click(); });

initWho();
loadTrials();
connect();
