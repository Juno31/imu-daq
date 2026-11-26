"use strict";
// Experiment workflow on the main page: one view per step, chosen by the URL hash.
//   #conn  #session  #participant  #equip-jig  #equip-imu  #equip-emg  #equip-daq  #record
// MVC / RVC (/emg) and Data review (/review) are their own pages; nav.js draws the bar for all of them.
// The live sections (status tiles, IMU, EMG, Sync detail, Event log) exist once; each view has empty
// "slots" and the router moves the sections into the slots of the view on screen.
// Uses app.js globals: $, S, LS, api, toast, pods, setFold, FOLD_DEFAULT, applyParticipant, ...

const ROUTES = ["conn", "session", "participant", "equip-jig", "equip-imu", "equip-emg", "equip-daq", "record"];
const EQUIP = ["equip-jig", "equip-imu", "equip-emg", "equip-daq"];
const EQUIP_NAME = { "equip-jig": "4a · IMU jig", "equip-imu": "4b · IMU attachment",
  "equip-emg": "4c · EMG attachment", "equip-daq": "4d · Time sync" };
const SLOT_EL = {};                   // slot name -> live section element
let ROUTE = null;
let INFO = {};                        // /api/expinfo for the current date + subject
const wesc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const who = () => ({ date: dateId($("testDate").value), subject: $("subject").value.trim() });
const whoKey = () => { const w = who(); return `${w.date}/${subjId(w.subject)}`; };

// ---------------- router ----------------
function showRoute(r, push = true) {
  if (!ROUTES.includes(r)) r = "conn";
  const changed = r !== ROUTE;
  ROUTE = r;
  document.body.dataset.route = r;
  for (const v of document.querySelectorAll(".view")) v.classList.toggle("on", v.dataset.view === r || (v.dataset.view === "equip" && EQUIP.includes(r)));
  for (const s of document.querySelectorAll(".sub")) s.classList.toggle("on", s.dataset.sub === r);
  // live sections into the visible slots, the rest back to the parking place
  const used = new Set();
  for (const slot of document.querySelectorAll(".view.on .slot")) {
    if (slot.closest(".sub") && !slot.closest(".sub").classList.contains("on")) continue;
    const el = SLOT_EL[slot.dataset.slot];
    if (!el || used.has(el)) continue;
    used.add(el);
    if (el.parentElement !== slot) slot.appendChild(el);
  }
  for (const el of Object.values(SLOT_EL)) if (!used.has(el) && el.parentElement !== $("parking")) $("parking").appendChild(el);
  // equipment pages always show their live section open; the recording page keeps the user's choice
  for (const [name, el] of Object.entries(SLOT_EL)) {
    if (!el.classList.contains("fold") || !used.has(el)) continue;
    if (EQUIP.includes(r)) { el.classList.add("open"); el.querySelector(".foldhead").setAttribute("aria-expanded", "true"); }
    else setFold(el, !!LS.get("fold." + el.dataset.fold, FOLD_DEFAULT[el.dataset.fold]));
  }
  smDirty = true; emgDirty = true;
  for (const p of pods.values()) p.dirty = true;
  // equipment: previous / next buttons
  const i = EQUIP.indexOf(r);
  if (i >= 0) {
    const prev = i ? `<a class="btn ghost" href="#${EQUIP[i - 1]}">‹ ${EQUIP_NAME[EQUIP[i - 1]]}</a>` : `<a class="btn ghost" href="#participant">‹ 3 · Participant</a>`;
    const next = i < EQUIP.length - 1 ? `<a class="btn" href="#${EQUIP[i + 1]}">Next: ${EQUIP_NAME[EQUIP[i + 1]]} ›</a>` : `<a class="btn" href="/emg">Next: 5 · MVC / RVC ›</a>`;
    $("equipNext").innerHTML = prev + next;
  }
  if (push && location.hash !== "#" + r) history.replaceState(null, "", "#" + r);
  LS.set("route", r);
  if (window.FlowNav) { FlowNav.current = r; FlowNav.redraw(); }
  if (window.spineApply) spineApply();
  if (changed) window.scrollTo(0, 0);
  renderChecks();
}
window.showRoute = showRoute;

// ---------------- 2 · Session information ----------------
const SI = { start_time: "si_start", end_time: "si_end", protocol: "si_protocol", irb: "si_irb", location: "si_location",
  room: "si_room", weather: "si_wcond", outdoor_temp_c: "si_wtemp", outdoor_rh_pct: "si_whum", indoor_temp_c: "si_itemp",
  indoor_rh_pct: "si_ihum", surface: "si_surface", notes: "si_notes" };
const KEEP = ["protocol", "irb", "location", "room", "surface"];          // carried over from the last session
const hhmm = () => new Date().toTimeString().slice(0, 5);

function teamRow(p = {}) {
  const row = document.createElement("div");
  row.className = "teamrow";
  row.innerHTML = `<input class="tn" placeholder="Name" autocomplete="off"><input class="tr" placeholder="Role" list="siRoles" autocomplete="off">` +
    `<input class="tc" placeholder="Contact (optional)" autocomplete="off"><button class="btn small ghost" type="button" title="Remove">✕</button>`;
  row.querySelector(".tn").value = p.name || "";
  row.querySelector(".tr").value = p.role || "";
  row.querySelector(".tc").value = p.contact || "";
  row.querySelector("button").onclick = () => { row.remove(); siDirty(true); };
  for (const inp of row.querySelectorAll("input")) inp.addEventListener("input", () => siDirty(true));
  $("siTeam").appendChild(row);
}
function collectSession() {
  const d = { test_date: who().date, subject: subjId(who().subject) };
  for (const [k, id] of Object.entries(SI)) d[k] = $(id).value.trim();
  d.team = [...$("siTeam").querySelectorAll(".teamrow")].map((r) => ({ name: r.querySelector(".tn").value.trim(),
    role: r.querySelector(".tr").value.trim(), contact: r.querySelector(".tc").value.trim() })).filter((p) => p.name || p.role);
  return d;
}
function fillSession(d, onlyKeep) {
  for (const [k, id] of Object.entries(SI)) if (!onlyKeep || KEEP.includes(k)) $(id).value = d && d[k] != null ? d[k] : "";
  $("siTeam").innerHTML = "";
  for (const p of (d && d.team) || []) teamRow(p);
  if (!$("siTeam").children.length) for (const role of ["Principal investigator", "GUI operator"]) teamRow({ role });
  if (onlyKeep) for (const id of ["si_start", "si_end", "si_wcond", "si_wtemp", "si_whum", "si_itemp", "si_ihum", "si_notes"]) $(id).value = "";
}
let siIsDirty = false, piIsDirty = false;
function siDirty(on) {
  siIsDirty = on;
  const b = $("btnSessSave");
  b.classList.toggle("dirty", on); b.classList.toggle("applied", !on && !!INFO.session_info);
  b.textContent = on || !INFO.session_info ? "Save session" : "Saved ✓";
  if (on) LS.set("si.draft", { key: whoKey(), d: collectSession() });
}
function piDirty(on) {
  piIsDirty = on;
  const b = $("btnSubjApply");
  b.classList.toggle("dirty", on); b.classList.toggle("applied", !on && !!INFO.participant);
  b.textContent = on || !INFO.participant ? "Save participant" : "Saved ✓";
  if (on) LS.set("pi.draft", { key: whoKey(), d: collectParticipant() });
}
async function saveInfo(kind, data) {
  const w = who();
  if (!w.subject) { toast("Enter the subject ID first (2 · Session)", true); showRoute("session"); $("subject").focus(); return null; }
  const j = await api("/api/expinfo", { date: w.date, subject: w.subject, kind, data });
  if (j.ok) INFO[kind] = j.data;
  return j.ok ? j : null;
}
async function saveSession() {
  if (!$("subject").value.trim()) { toast("Enter the subject ID first", true); $("subject").focus(); return; }
  applyParticipant();                                   // normalizes date + subject, next trial number, calibration state
  const j = await saveInfo("session_info", collectSession());
  if (j) { siDirty(false); LS.set("si.draft", null); toast("Session saved: " + j.path); setText("siState", "Saved " + j.data.saved + " · " + j.path); updateMarks(); }
}

// ---------------- 2 · Participant information ----------------
const PI = { age: "pi_age", sex: "pi_sex", weight_kg: "pi_weight", waist_cm: "pi_waist", dominant_hand: "pi_hand",
  dominant_leg: "pi_leg", group: "pi_group", activity: "pi_activity", low_back_pain: "pi_lbp", pain_nrs: "pi_nrs", history: "pi_history" };
const PI_CK = { consent_signed: "pi_consent", eligibility_checked: "pi_eligible", clothing_ok: "pi_clothing" };
function bmi() {
  const h = parseFloat($("heightCm").value) / 100, w = parseFloat($("pi_weight").value);
  return h > 1 && w > 0 ? +(w / (h * h)).toFixed(1) : null;
}
function collectParticipant() {
  const d = { subject: subjId(who().subject), height_cm: parseFloat($("heightCm").value) || null, bmi: bmi(), notes: $("notes").value.trim() };
  for (const [k, id] of Object.entries(PI)) d[k] = $(id).value.trim();
  for (const [k, id] of Object.entries(PI_CK)) d[k] = $(id).checked;
  return d;
}
function fillParticipant(d) {
  for (const [k, id] of Object.entries(PI)) $(id).value = d && d[k] != null ? d[k] : "";
  for (const [k, id] of Object.entries(PI_CK)) $(id).checked = !!(d && d[k]);
  if (d && d.height_cm && !$("heightCm").value) $("heightCm").value = d.height_cm;
  $("pi_bmi").value = bmi() ?? "";
}
async function saveParticipant() {
  if (!$("subject").value.trim()) { toast("Enter the subject ID first (2 · Session)", true); showRoute("session"); return; }
  applyParticipant();
  const d = collectParticipant();
  if (!d.consent_signed) toast("Informed consent is not ticked", "warn");
  const j = await saveInfo("participant", d);
  if (j) { piDirty(false); LS.set("pi.draft", null); toast("Participant saved: " + j.path); updateMarks(); }
}

// ---------------- load everything saved for this date + subject ----------------
let infoTimer = null, infoKey = "";
function loadInfo(force) {
  clearTimeout(infoTimer);
  infoTimer = setTimeout(async () => {
    const w = who(), key = whoKey();
    $("piWho").textContent = w.subject ? `${subjId(w.subject)} · ${w.date}` : "enter the subject ID on the Session page";
    $("siFolder").textContent = w.subject ? `→ data/${key}/` : "";
    if (!w.subject) { INFO = {}; if (!$("siTeam").children.length) fillSession(null, true); return; }
    if (key === infoKey && !force) return;
    let j;
    try { j = await (await fetch("/api/expinfo?" + new URLSearchParams({ date: w.date, subject: w.subject }))).json(); } catch { return; }
    if (!j.ok) return;
    const fresh = key !== infoKey;
    infoKey = key;
    INFO = j;
    if (fresh || !siIsDirty) {
      const draft = LS.get("si.draft", null);
      if (j.session_info) fillSession(j.session_info);
      else if (draft && draft.key === key) fillSession(draft.d);
      else { fillSession(j.last_session_info, true); if (!$("si_start").value) $("si_start").value = hhmm(); }
      setText("siState", j.session_info ? `Saved ${j.session_info.saved} · ${j.dir}/session_info.json` : "Not saved yet.");
      siDirty(!j.session_info && !!(draft && draft.key === key));
    }
    if (fresh || !piIsDirty) {
      const draft = LS.get("pi.draft", null);
      fillParticipant(j.participant || (draft && draft.key === key ? draft.d : null));
      piDirty(!j.participant && !!(draft && draft.key === key));
    }
    CHK = (j.setup_check && { imu: j.setup_check.imu || {}, emg: j.setup_check.emg || {} }) || { imu: {}, emg: {} };
    chkBuilt = { imu: "", emg: "" };
    if (j.jig_alignment) renderJig(j.jig_alignment); else { $("jigRes").innerHTML = ""; jigPill(null); }
    const st = LS.get("syncTest", null);
    renderSyncResult(st && st.key === key ? st : null);
    renderChecks();
    updateMarks();
  }, 200);
}

// ---------------- 3b · jig: global orientation ----------------
let jigCap = null, jigSecs = +LS.get("jigSecs", 5);
const qn = (q) => { const n = Math.hypot(...q) || 1; return q.map((v) => v / n); };
const angDeg = (a, b) => Math.acos(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))) * D;
const upInSensor = (q) => { const R = rotm(q); return [R[2][0], R[2][1], R[2][2]]; };   // world "up" in sensor axes
function meanQ(list) {
  const r = list[0], acc = [0, 0, 0, 0];
  for (const q of list) { const s = q[0] * r[0] + q[1] * r[1] + q[2] * r[2] + q[3] * r[3] < 0 ? -1 : 1; for (let i = 0; i < 4; i++) acc[i] += s * q[i]; }
  const m = qn(acc);
  return m[0] < 0 ? m.map((v) => -v) : m;
}
function liveNames() { return ((S && S.pods) || []).filter((p) => p.ok && p.live).map((p) => p.name); }
function jigRefOptions() {
  const sel = $("jigRef"), names = ((S && S.pods) || []).map((p) => p.name), key = names.join(",");
  if (sel.dataset.key === key) return;
  const want = sel.value || LS.get("jigRef", "");
  sel.dataset.key = key;
  sel.innerHTML = names.map((n) => `<option>${wesc(n)}</option>`).join("");
  if (names.includes(want)) sel.value = want;
}
function runJig() {
  if (jigCap) return;
  const names = liveNames();
  if (names.length < 1) { toast("No live IMU data", true); return; }
  jigCap = { t0: performance.now(), secs: jigSecs, samples: Object.fromEntries(names.map((n) => [n, []])), gyr: Object.fromEntries(names.map((n) => [n, []])) };
  FRAME_HOOK = (p, ok) => {
    if (!ok || !jigCap || !jigCap.samples[p.name]) return;
    jigCap.samples[p.name].push(p.q);
    if (p.x) jigCap.gyr[p.name].push(p.x.slice(3, 6));
  };
  $("jigProg").classList.remove("hidden");
  $("btnJigCap").disabled = true;
  const tick = () => {
    if (!jigCap) return;
    const f = (performance.now() - jigCap.t0) / 1000 / jigCap.secs;
    $("jigBar").style.width = `${Math.min(100, f * 100).toFixed(0)}%`;
    $("jigLeft").textContent = `Keep the jig still… ${Math.max(0, jigCap.secs * (1 - f)).toFixed(1)} s`;
    if (f >= 1) finishJig(); else setTimeout(tick, 100);
  };
  tick();
}
async function finishJig() {
  const c = jigCap;
  jigCap = null; FRAME_HOOK = null;
  $("jigProg").classList.add("hidden");
  $("btnJigCap").disabled = false;
  const names = Object.keys(c.samples).filter((n) => c.samples[n].length >= 10);
  if (!names.length) { toast("No IMU samples during the capture", true); return; }
  const ref = names.includes($("jigRef").value) ? $("jigRef").value : names[0];
  const out = { when: new Date().toISOString(), seconds: c.secs, reference: ref, quat_mode: quatOf(S),
    note: "q_mean = average quaternion (w, x, y, z) of each IMU while all sit in the jig with the same orientation. " +
      "q_world_offset = q_imu * conj(q_ref): rotation from the reference IMU's world frame to this IMU's; heading_to_ref_deg is its yaw.",
    imus: {} };
  const qr = meanQ(c.samples[ref]);
  for (const n of names) {
    const qs = c.samples[n], q = meanQ(qs);
    const dev = Math.max(...qs.map((x) => { const d = qmul(qconj(q), x); return 2 * Math.acos(Math.min(1, Math.abs(d[0]))) * D; }));
    const g = c.gyr[n].filter(Boolean);
    const grms = g.length ? Math.sqrt(g.reduce((s, v) => s + v[0] * v[0] + v[1] * v[1] + v[2] * v[2], 0) / g.length) * D : null;
    const off = meanQ([qmul(q, qconj(qr))]);
    const rel = qmul(qconj(qr), q);
    const e = euler(q);
    out.imus[n] = { samples: qs.length, q_mean: q.map((v) => +v.toFixed(6)), roll_deg: +e[0].toFixed(2), pitch_deg: +e[1].toFixed(2), yaw_deg: +e[2].toFixed(2),
      still_max_deg: +dev.toFixed(2), gyro_rms_dps: grms == null ? null : +grms.toFixed(3),
      angle_to_ref_deg: +(2 * Math.acos(Math.min(1, Math.abs(rel[0]))) * D).toFixed(2),
      tilt_to_ref_deg: +angDeg(upInSensor(q), upInSensor(qr)).toFixed(2),
      heading_to_ref_deg: +euler(off)[2].toFixed(2), q_world_offset: off.map((v) => +v.toFixed(6)) };
  }
  out.ok = names.every((n) => out.imus[n].still_max_deg < 1.5 && out.imus[n].tilt_to_ref_deg < 3);
  renderJig(out);
  if (!who().subject) { toast("Jig orientation measured, not saved: enter the subject ID (2 · Session) and capture again", "warn"); return; }
  const j = await saveInfo("jig_alignment", out);
  if (j) { toast(`Jig orientation saved (${names.length} IMUs) · ${j.path}`, out.ok ? undefined : "warn"); updateMarks(); }
}
function jigPill(o) {
  const p = $("jigPill");
  p.textContent = !o ? "NOT DONE" : o.ok ? "OK " + new Date(o.when).toTimeString().slice(0, 5) : "CHECK";
  p.className = "pill " + (!o ? "" : o.ok ? "ok" : "warn");
}
function renderJig(o) {
  jigPill(o);
  const lvl = (v, ok, warn) => v == null ? "" : v < ok ? "good" : v < warn ? "warn" : "bad";
  const rows = Object.entries(o.imus || {}).map(([n, r]) => `<tr><td><b>${wesc(n)}</b>${n === o.reference ? ' <span class="hint">ref</span>' : ""}</td>` +
    `<td class="mono">${r.q_mean.map((v) => v.toFixed(4)).join(", ")}</td>` +
    `<td class="mono">${r.roll_deg.toFixed(1)} / ${r.pitch_deg.toFixed(1)} / ${r.yaw_deg.toFixed(1)}</td>` +
    `<td class="mono ${lvl(r.tilt_to_ref_deg, 2, 5)}">${r.tilt_to_ref_deg.toFixed(2)}°</td>` +
    `<td class="mono">${r.heading_to_ref_deg.toFixed(1)}°</td>` +
    `<td class="mono ${lvl(r.still_max_deg, 0.5, 1.5)}">${r.still_max_deg.toFixed(2)}°</td>` +
    `<td class="mono">${r.gyro_rms_dps == null ? "—" : r.gyro_rms_dps.toFixed(2) + " °/s"}</td></tr>`).join("");
  $("jigRes").innerHTML = `<table><thead><tr><th>IMU</th><th>q mean (w, x, y, z)</th><th>roll / pitch / yaw</th>` +
    `<th title="Angle between the two IMUs' gravity directions: should be ~0 in the jig">tilt vs ref</th>` +
    `<th title="Heading offset of this IMU's world frame (game RV: set at IMU start)">heading vs ref</th>` +
    `<th title="Largest deviation from the mean during the capture">moved</th><th>gyro RMS</th></tr></thead><tbody>${rows}</tbody></table>` +
    `<p class="cap">${new Date(o.when).toLocaleString()} · ${wesc(o.quat_mode === "rv" ? "rotation vector (mag)" : "game rotation vector")} · ${o.seconds} s` +
    (o.ok ? "" : ' · <span class="warn">tilt ≥ 3° or the jig moved: check the boards sit flat, capture again</span>') + "</p>";
}

// ---------------- 3b · jig: six-face calibration recording ----------------
let special = null;                    // {task, session, stopAt}
async function startSpecial(task, name, secs) {
  const w = who();
  if (!w.subject) { toast("Enter the subject ID first (2 · Session)", true); return; }
  if (!S || S.state !== "idle") { toast(S ? `Not now: ${S.state}` : "Server not connected", true); return; }
  let trial = 1;
  try { trial = (await (await fetch("/api/next_trial?" + new URLSearchParams({ date: w.date, subject: w.subject, task }))).json()).trial || 1; } catch { }
  const j = await api("/api/start", { subject: w.subject, date: w.date, task, task_name: name, trial, notes: $("notes").value, height_cm: heightVal() });
  if (j.ok) { special = { task, session: j.session, stopAt: secs ? performance.now() + secs * 1000 : null, secs }; toast("Recording " + j.session); $("banner").classList.add("hidden"); }
}
function specialRunning(task) { return !!(special && special.task === task && S && S.state === "recording"); }
async function jigRecToggle() {
  if (specialRunning("jigcal")) { await api("/api/stop"); return; }
  await startSpecial("jigcal", "Six-face jig calibration");
}
async function jigAnalyze() {
  const w = who();
  $("jigCalRes").innerHTML = '<p class="cap">Analysing…</p>';
  const j = await api("/api/jigcal", { date: w.date, subject: w.subject });
  if (!j.ok) { $("jigCalRes").innerHTML = ""; return; }
  const o = j.result, f3 = (a, d = 3) => a ? a.map((v) => (+v).toFixed(d)).join(", ") : "—";
  const rows = Object.entries(o.imus || {}).map(([n, r]) => { const a = r.accel, g = r.gyro || {};
    return `<tr><td><b>${wesc(n)}</b></td><td class="mono">${f3(g.bias_dps)}</td><td class="mono">${a ? f3(a.bias_ms2) : "—"}</td>` +
      `<td class="mono">${a ? f3(a.scale_error_pct, 2) : "—"}</td><td class="mono">${a ? (+a.misalignment_deg).toFixed(2) + "°" : "—"}</td>` +
      `<td class="mono">${a ? (+a.residual_rms_ms2).toFixed(4) : "—"}</td></tr>`; }).join("");
  const pairs = Object.entries(o.pairs || {}).map(([k, v]) => `${wesc(k)} ${(+v.angle_deg).toFixed(2)}°`).join(" · ");
  $("jigCalRes").innerHTML = `<table><thead><tr><th>IMU</th><th>gyro bias °/s</th><th>acc bias m/s²</th><th>scale err %</th><th>axis misalign.</th><th>fit resid.</th></tr></thead>` +
    `<tbody>${rows}</tbody></table><p class="cap"><span class="${o.ok ? "good" : "warn"}">${o.ok ? "✓ complete" : "! " + wesc((o.problems || []).join("; "))}</span>` +
    ` · faces ${wesc((o.poses_found || []).join(" "))}${pairs ? " · sensor-to-sensor: " + pairs : ""} · <span class="mono">${wesc(j.rel)}/jig_calibration.json</span></p>`;
}

// ---------------- 3c / 3d · attachment checklists ----------------
let CHK = { imu: {}, emg: {} }, chkBuilt = { imu: "", emg: "" }, emgRest = null;
function segOf() {
  let m = null; try { m = JSON.parse(localStorage.getItem("spineMap") || "null"); } catch { }
  const out = {};
  for (const [seg, pod] of Object.entries(m || {})) if (pod) out[pod] = seg;
  return out;
}
function ckCell(group, id, field) {
  const on = !!(CHK[group][id] && CHK[group][id][field]);
  return `<td class="ckc"><input type="checkbox" data-g="${group}" data-id="${wesc(id)}" data-f="${field}"${on ? " checked" : ""}></td>`;
}
function onCk(e) {
  const t = e.target;
  if (!t.dataset || !t.dataset.g) return;
  const g = CHK[t.dataset.g], r = g[t.dataset.id] || (g[t.dataset.id] = {});
  r[t.dataset.f] = t.checked;
  chkDirty[t.dataset.g] = true;
  renderChecks(); updateMarks();
}
const chkDirty = { imu: false, emg: false };
function imuRowsState() {
  const seg = segOf();
  return ((S && S.pods) || []).map((pd) => {
    const st = !pd.ok ? ["bad", "NOT FOUND"] : (!pd.live || (pd.silent_s !== null && pd.silent_s > 0.5)) ? ["bad", "STALLED"]
      : pd.fresh_recent < 90 ? ["warn", "DROPPING"] : ["good", "OK"];
    return { pd, seg: seg[pd.name] || "", st };
  });
}
function renderImuCheck() {
  const rows = imuRowsState(), key = rows.map((r) => r.pd.name + ":" + r.seg).join(",");
  if (!rows.length) { $("imuCheck").innerHTML = '<p class="cap">No IMUs reported yet: connect the board.</p>'; chkBuilt.imu = ""; return; }
  if (key !== chkBuilt.imu) {
    chkBuilt.imu = key;
    $("imuCheck").innerHTML = `<table><thead><tr><th>IMU</th><th>Segment</th><th>Attached</th><th>Secured</th><th>Cable free</th>` +
      `<th>Status</th><th>New data</th><th>Stalls</th></tr></thead><tbody>` +
      rows.map((r) => `<tr data-pod="${wesc(r.pd.name)}"><td><b>${wesc(r.pd.name)}</b></td>` +
        `<td>${r.seg ? wesc(r.seg) : '<a href="/spine" class="warn">set in Spine view</a>'}</td>` +
        ckCell("imu", r.pd.name, "attached") + ckCell("imu", r.pd.name, "secured") + ckCell("imu", r.pd.name, "cable") +
        `<td class="st"></td><td class="mono fr"></td><td class="mono sl"></td></tr>`).join("") + "</tbody></table>";
  }
  for (const r of rows) {
    const tr = $("imuCheck").querySelector(`tr[data-pod="${CSS.escape(r.pd.name)}"]`);
    if (!tr) continue;
    const st = tr.querySelector(".st"); st.className = "st " + r.st[0]; st.textContent = r.st[1];
    tr.querySelector(".fr").textContent = r.pd.ok ? r.pd.fresh_recent.toFixed(1) + "%" : "—";
    tr.querySelector(".sl").textContent = r.pd.stalls;
  }
  const ticked = rows.every((r) => ["attached", "secured"].every((f) => CHK.imu[r.pd.name] && CHK.imu[r.pd.name][f]));
  const live = rows.every((r) => r.st[0] === "good");
  const cal = $("step3").classList.contains("done");
  const p = $("imuChkPill");
  p.textContent = ticked && live && cal ? "READY" : !live ? "SIGNAL" : !ticked ? "ATTACH" : "CALIBRATE";
  p.className = "pill " + (ticked && live && cal ? "ok" : live ? "warn" : "bad");
  setText("imuChkState", [ticked ? "" : "tick attached + secured", live ? "" : "not every IMU is OK", cal ? "" : "do the static calibration"].filter(Boolean).join(" · ") || "All IMUs attached, live and calibrated.");
  return ticked && live && cal;
}
function emgState(c) {
  return c.floating ? ["bad", "NOT CONNECTED"] : c.clip ? ["bad", "CLIPPING"] : c.rms_mv !== null && c.rms_mv < 0.2 ? ["warn", "FLAT"] : ["good", "OK"];
}
function renderEmgCheck() {
  const ch = (S && S.emg && S.emg.chans) || [];
  if (!ch.length) {
    chkBuilt.emg = "";
    $("emgCheck").innerHTML = `<p class="cap">No live EMG. ${S && S.daq && !S.daq.preview_on && S.state !== "recording" ? "Press <b>Preview</b> in the EMG section below." : "Check the DAQ (1 · Connections)."}</p>`;
    $("emgChkPill").textContent = "NO SIGNAL"; $("emgChkPill").className = "pill bad";
    return false;
  }
  const key = ch.map((c) => c.chan).join(",");
  if (key !== chkBuilt.emg) {
    chkBuilt.emg = key;
    $("emgCheck").innerHTML = `<table><thead><tr><th>Channel</th><th>Muscle</th><th>Side</th><th>Skin prep</th><th>Electrodes</th><th>Burst seen</th>` +
      `<th>Status</th><th>RMS now</th><th>Rest RMS</th></tr></thead><tbody>` +
      ch.map((c) => `<tr data-ch="${wesc(c.chan)}"><td><b>${wesc(c.name)}</b> <span class="hint mono">${wesc(c.chan)}</span></td>` +
        `<td>${wesc(c.muscle || "")}</td><td>${wesc(c.side || "")}</td>` +
        ckCell("emg", c.chan, "prep") + ckCell("emg", c.chan, "placed") + ckCell("emg", c.chan, "burst") +
        `<td class="st"></td><td class="mono rm"></td><td class="mono rr"></td></tr>`).join("") + "</tbody></table>";
  }
  for (const c of ch) {
    const tr = $("emgCheck").querySelector(`tr[data-ch="${CSS.escape(c.chan)}"]`);
    if (!tr) continue;
    const s = emgState(c), st = tr.querySelector(".st");
    st.className = "st " + s[0]; st.textContent = s[1];
    tr.querySelector(".rm").textContent = c.rms_mv === null ? "—" : c.rms_mv.toFixed(3) + " mV";
    const rr = CHK.emg[c.chan] && CHK.emg[c.chan].rest_rms_mv;
    tr.querySelector(".rr").textContent = rr == null ? "—" : rr.toFixed(3) + " mV";
  }
  const ticked = ch.every((c) => ["prep", "placed"].every((f) => CHK.emg[c.chan] && CHK.emg[c.chan][f]));
  const live = ch.every((c) => emgState(c)[0] !== "bad");
  const p = $("emgChkPill");
  p.textContent = ticked && live ? "READY" : !live ? "SIGNAL" : "ATTACH";
  p.className = "pill " + (ticked && live ? "ok" : live ? "warn" : "bad");
  setText("emgChkState", [ticked ? "" : "tick skin prep + electrodes", live ? "" : "a channel is not connected or clipping"].filter(Boolean).join(" · ") || "All channels attached and live.");
  return ticked && live;
}
function tickEmgRest() {
  if (!emgRest || !S || !S.emg) return;
  for (const c of S.emg.chans || []) if (c.rms_mv !== null) (emgRest.v[c.chan] || (emgRest.v[c.chan] = [])).push(c.rms_mv);
  const left = emgRest.until - performance.now();
  setText("emgRestState", left > 0 ? `participant relaxed… ${(left / 1000).toFixed(1)} s` : "");
  if (left <= 0) {
    for (const [ch, v] of Object.entries(emgRest.v)) { const r = CHK.emg[ch] || (CHK.emg[ch] = {}); r.rest_rms_mv = +(v.reduce((a, b) => a + b, 0) / v.length).toFixed(4); }
    emgRest = null; chkDirty.emg = true; chkBuilt.emg = "";
    setText("emgRestState", "rest RMS measured · save the checklist");
    renderEmgCheck();
  }
}
async function saveChecklist() {
  const j = await saveInfo("setup_check", { imu: CHK.imu, emg: CHK.emg, imu_segments: segOf() });
  if (j) { chkDirty.imu = chkDirty.emg = false; toast("Checklist saved: " + j.path); }
}
function renderChecks() {
  if (ROUTE === "conn") { if (window.connRender) connRender(); if (window.connTestRender) connTestRender(); }
  if (ROUTE === "equip-imu") renderImuCheck();
  if (ROUTE === "equip-emg") renderEmgCheck();
  if (ROUTE === "equip-jig") jigRefOptions();
}

// ---------------- 4d · time sync test ----------------
let syncSecs = +LS.get("syncSecs", 10);
function renderSyncResult(r) {
  const p = $("syncTestPill");
  if (!r) { p.textContent = "NOT RUN"; p.className = "pill"; $("syncTestRes").innerHTML = ""; return; }
  const sm = r.summary || {}, ds = sm.daq_sync;
  p.textContent = r.ok ? "PASS" : "FAIL"; p.className = "pill " + (r.ok ? "ok" : "bad");
  const pods = Object.entries(sm.pods || {}).map(([k, v]) => `${wesc(k)} ${(+v.fresh_pct).toFixed(1)}%`).join(" · ");
  $("syncTestRes").innerHTML = `<table><tbody>` +
    `<tr><th>Result</th><td class="${r.ok ? "good" : "bad"}"><b>${r.ok ? "SYNC PASS" : "FAIL"}</b> ${ds ? (ds.ok ? `· ${ds.pulses_after_marker} pulses = frames` : "· " + wesc(ds.reason)) : "· no DAQ in the recording"}</td></tr>` +
    `<tr><th>Frames</th><td class="mono">${sm.frames_received ?? "—"} received · ${sm.missing_frames ?? "—"} missing · ${sm.duration_s ?? "—"} s · ${sm.rate_hz ?? "—"} Hz</td></tr>` +
    `<tr><th>IMU new data</th><td class="mono">${pods || "—"}</td></tr>` +
    `<tr><th>Recording</th><td class="mono">${wesc(sm.session || "")} · ${new Date(r.when).toLocaleTimeString()}</td></tr></tbody></table>`;
}
function syncTick() {
  if (!special || !special.stopAt || !S || S.state !== "recording") { if (!special || special.task !== "synctest") setText("syncTestLeft", ""); return; }
  const left = (special.stopAt - performance.now()) / 1000;
  setText("syncTestLeft", left > 0 ? `recording… ${left.toFixed(1)} s` : "stopping…");
  if (left <= 0 && !special.stopping) { special.stopping = true; api("/api/stop"); }
}

// ---------------- done marks for the step bar ----------------
let markT = 0;
function updateMarks() {
  if (!window.FlowNav) return;
  const w = who(), have = !!w.subject && infoKey === whoKey();
  FlowNav.mark("session", have && !!INFO.session_info);
  FlowNav.mark("participant", have && !!INFO.participant);
  FlowNav.mark("equip-jig", have && !!INFO.jig_alignment);
  if (S) {
    const imuOk = (() => { const rows = imuRowsState(); return rows.length && rows.every((r) => r.st[0] === "good" && CHK.imu[r.pd.name] && CHK.imu[r.pd.name].attached && CHK.imu[r.pd.name].secured); })();
    FlowNav.mark("equip-imu", have && !!imuOk && $("step3").classList.contains("done"));
    const ch = (S.emg && S.emg.chans) || [];
    FlowNav.mark("equip-emg", have && ch.length > 0 && ch.every((c) => emgState(c)[0] !== "bad" && CHK.emg[c.chan] && CHK.emg[c.chan].prep && CHK.emg[c.chan].placed));
    const st = LS.get("syncTest", null);
    FlowNav.mark("equip-daq", have && !!(st && st.key === whoKey() && st.ok));
  }
  FlowNav.mark("mvc", have && !!INFO.emg_norm);
  FlowNav.mark("record", have && (INFO.trials || []).some((t) => !/_(static_cal|synctest|jigcal|emg_[a-z]+)_trial/.test(t)));
}

// ---------------- status from app.js (every ~250 ms) ----------------
window.onWorkflowStatus = (s, prev) => {
  // special recordings: button text and results
  const jr = specialRunning("jigcal");
  $("btnJigRec").textContent = jr ? "■ Stop jig trial" : "● Record 6-face jig trial";
  $("btnJigRec").classList.toggle("stop", jr);
  $("btnJigRec").disabled = !jr && s.state !== "idle";
  $("btnSyncTest").disabled = s.state !== "idle";
  if (jr && s.rec) setText("jigRecInfo", `recording ${special.session} · ${s.rec.elapsed.toFixed(0)} s · turn the jig to the next face every ≥ 15 s`);
  syncTick();
  if (prev === "stopping" && s.state !== "stopping" && s.last_summary) {
    const sm = s.last_summary;
    if (special && sm.session === special.session) {
      if (special.task === "synctest") {
        const ok = !sm.aborted && !sm.missing_frames && !!(sm.daq_sync && sm.daq_sync.ok);
        const r = { key: whoKey(), ok, when: Date.now(), summary: sm };
        LS.set("syncTest", r);
        renderSyncResult(r);
        setText("syncTestLeft", "");
        toast(ok ? "Sync test PASS" : "Sync test FAIL: see 4d · Time sync", ok ? undefined : true);
      } else if (special.task === "jigcal") {
        setText("jigRecInfo", `saved ${sm.session} · ${sm.duration_s ?? 0} s · press Analyse latest`);
      }
      special = null;
    }
    loadInfo(true);                                   // new trial in the folder
  }
  tickEmgRest();
  renderChecks();
  if (performance.now() - markT > 1000) { markT = performance.now(); updateMarks(); }
};

// ---------------- wiring ----------------
function seg(id, cur, set) {
  for (const b of $(id).querySelectorAll("button")) {
    b.classList.toggle("on", +b.dataset.s === cur);
    b.onclick = () => { set(+b.dataset.s); for (const x of $(id).querySelectorAll("button")) x.classList.toggle("on", x === b); };
  }
}
function wireWorkflow() {
  SLOT_EL.tiles = document.querySelector("#parking .statusrow");
  SLOT_EL.imu = $("foldImu"); SLOT_EL.emg = $("emgCard"); SLOT_EL.sync = $("syncCard"); SLOT_EL.log = $("foldLog");
  FlowNav.mount($("flowNav"), "conn", $("equipSubs"));
  // participant form: own Save buttons (app.js "Apply" text is replaced)
  participantDirty = () => { };                       // eslint-disable-line no-global-assign
  $("btnSessSave").onclick = saveSession;
  $("btnSubjApply").onclick = saveParticipant;
  for (const id of [...Object.values(SI), "testDate", "subject"]) {
    $(id).addEventListener("input", () => siDirty(true));
    $(id).addEventListener("keydown", (e) => { if (e.key === "Enter" && e.target.tagName === "INPUT") { e.preventDefault(); saveSession(); } });
  }
  for (const id of [...Object.values(PI), ...Object.values(PI_CK), "heightCm", "notes"]) {
    $(id).addEventListener("input", () => { piDirty(true); $("pi_bmi").value = bmi() ?? ""; });
    $(id).addEventListener("change", () => piDirty(true));
  }
  for (const id of ["testDate", "subject"]) $(id).addEventListener("change", () => loadInfo());
  $("btnAddPerson").onclick = () => { teamRow(); siDirty(true); $("siTeam").lastElementChild.querySelector("input").focus(); };
  for (const b of document.querySelectorAll("[data-now]")) b.onclick = () => { $(b.dataset.now).value = hhmm(); siDirty(true); };
  // equipment
  seg("jigSecs", jigSecs, (v) => { jigSecs = v; LS.set("jigSecs", v); });
  seg("syncSecs", syncSecs, (v) => { syncSecs = v; LS.set("syncSecs", v); });
  $("jigRef").onchange = () => LS.set("jigRef", $("jigRef").value);
  $("btnJigCap").onclick = runJig;
  $("btnJigRec").onclick = jigRecToggle;
  $("btnJigAnalyze").onclick = jigAnalyze;
  $("btnSyncTest").onclick = () => startSpecial("synctest", "Time sync test", syncSecs);
  $("btnImuChkSave").onclick = saveChecklist;
  $("btnEmgChkSave").onclick = saveChecklist;
  $("btnEmgRest").onclick = () => {
    if (!S || !S.emg || !(S.emg.chans || []).length) { toast("No live EMG: press Preview first", true); return; }
    emgRest = { until: performance.now() + 5000, v: {} };
  };
  document.addEventListener("change", onCk);
  // status tiles: the detail lives on its equipment page
  const TILE_ROUTE = { foldImu: "equip-imu", emgCard: "equip-emg", syncCard: "equip-daq" };
  for (const t of document.querySelectorAll(".stile")) t.onclick = () => {
    const go = t.dataset.go, el = $(go);
    if (go.startsWith("drawer") || (el && el.closest(".view.on"))) return goTo(go);
    if (TILE_ROUTE[go]) showRoute(TILE_ROUTE[go]);
  };
  $("tpTake").onclick = () => { showRoute("record"); const el = $("label"); el.focus(); el.select(); };
  window.addEventListener("hashchange", () => showRoute(location.hash.slice(1), false));
  window.addEventListener("beforeunload", (e) => { if (siIsDirty || piIsDirty) { e.preventDefault(); e.returnValue = ""; } });
  const start = location.hash.slice(1) || LS.get("route", "conn");
  showRoute(ROUTES.includes(start) ? start : "conn");
  loadInfo(true);
}
wireWorkflow();
