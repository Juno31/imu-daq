// Spine view: two IMUs (T7, T12) -> calibrated segment orientations -> 3D spine model.
//
// Conventions
//  * Quaternions are [w, x, y, z]; an IMU quaternion q rotates sensor-frame vectors into the
//    sensor's world frame (Game Rotation Vector: Z up, heading arbitrary).
//  * Body (anatomical) frame, fixed at calibration:  X = left (the forward-flexion axis),
//    Y = posterior, Z = up.  Positive rotation about X = flexion (forward), about Y = lateral
//    bending to the left, about Z = axial rotation to the left.
//  * Segment orientation relative to standing:  R_i = B^-1 (q_i q0_i^-1) B, with q0_i the neutral
//    quaternion and B the body-to-world rotation. Mounting angle of the board therefore does not matter.
//  * Angles: intrinsic X-Y-Z (flexion, then lateral bending, then axial rotation).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { mergeVertices } from "three/addons/utils/BufferGeometryUtils.js";

const $ = (id) => document.getElementById(id);
const D = 180 / Math.PI;

// ---------------- quaternion math ----------------
const qmul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
const qnorm = (q) => { const n = Math.hypot(...q) || 1; return q.map((v) => v / n); };
const qpos = (q) => (q[0] < 0 ? q.map((v) => -v) : q);
function qpow(q, f) {                       // fraction f of the rotation q (slerp from identity)
  q = qpos(q);
  const s = Math.hypot(q[1], q[2], q[3]);
  if (s < 1e-9) return [1, 0, 0, 0];
  const th = 2 * Math.atan2(s, q[0]) * f;
  const k = Math.sin(th / 2) / s;
  return [Math.cos(th / 2), q[1] * k, q[2] * k, q[3] * k];
}
function rotvec(q) {                        // rotation vector (axis * angle, rad)
  q = qpos(q);
  const s = Math.hypot(q[1], q[2], q[3]);
  if (s < 1e-9) return [0, 0, 0];
  const th = 2 * Math.atan2(s, q[0]);
  return [q[1] / s * th, q[2] / s * th, q[3] / s * th];
}
function qmean(list) {                      // sign-aligned average, fine for clustered samples
  const ref = list[0], acc = [0, 0, 0, 0];
  for (const q of list) {
    const d = q[0] * ref[0] + q[1] * ref[1] + q[2] * ref[2] + q[3] * ref[3];
    const s = d < 0 ? -1 : 1;
    for (let i = 0; i < 4; i++) acc[i] += s * q[i];
  }
  return qnorm(acc);
}
function qFromAxes(X, Y, Z) {               // rotation whose matrix columns are X, Y, Z
  const m00 = X[0], m10 = X[1], m20 = X[2], m01 = Y[0], m11 = Y[1], m21 = Y[2], m02 = Z[0], m12 = Z[1], m22 = Z[2];
  const tr = m00 + m11 + m22;
  let w, x, y, z;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = s / 4; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = s / 4; y = (m01 + m10) / s; z = (m02 + m20) / s; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = s / 4; z = (m12 + m21) / s; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = s / 4; }
  return qnorm([w, x, y, z]);
}
function eulerXYZ(q) {                      // intrinsic X-Y-Z -> [flexion, lateral, axial] deg
  const [w, x, y, z] = q;
  const r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
  const r12 = 2 * (y * z - w * x), r22 = 1 - 2 * (x * x + y * y);
  const b = Math.asin(Math.max(-1, Math.min(1, r02)));
  return [Math.atan2(-r12, r22) * D, b * D, Math.atan2(-r01, r00) * D];
}

// ---------------- state ----------------
const H = 10;                                // plot window, s
let status = null, podNames = [];
const live = {};                             // pod name -> latest quaternion
const cal = load("spineCal") || { q0: null, B: null, map: null, when: null };
let capture = null;                          // {kind, until, samples: {name: []}}
const hist = { t: [], a: [] };               // T7-vs-T12 angles

function load(k) { try { return JSON.parse(localStorage.getItem(k)); } catch { return null; } }
function save(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* private mode */ } }

function mapping() { return { T7: $("mapT7").value, T12: $("mapT12").value }; }

function segment(name) {                     // body-frame rotation from standing, or null
  const q = live[name];
  if (!q || !cal.q0 || !cal.q0[name]) return null;
  const g = qmul(q, qconj(cal.q0[name]));    // world-frame rotation since neutral
  const B = cal.B || [1, 0, 0, 0];
  return qnorm(qmul(qmul(qconj(B), g), B));
}

// ---------------- calibration ----------------
function setMsg(text, cls = "") { const m = $("calMsg"); m.textContent = text; m.className = "calmsg " + cls; }

function startCapture(kind, secs) {
  const m = mapping();
  if (!live[m.T7] || !live[m.T12]) { setMsg("No data from both sensors yet. Check the connection on the Acquisition page.", "warn"); return; }
  if (m.T7 === m.T12) { setMsg("T7 and T12 must be different sensors.", "warn"); return; }
  if (kind === "flex" && !cal.q0) { setMsg("Capture neutral first.", "warn"); return; }
  capture = { kind, until: performance.now() + secs * 1000, secs, map: m, samples: { [m.T7]: [], [m.T12]: [] } };
  setMsg(kind === "neutral" ? "Hold still…" : "Bend forward slowly and come back up…", "busy");
}

window.addEventListener("beforeunload", (e) => { if (capture) { e.preventDefault(); e.returnValue = ""; } });

function onCaptureSample() {
  if (!capture) return;
  for (const n of Object.keys(capture.samples)) if (live[n]) capture.samples[n].push(live[n].slice());
  const left = (capture.until - performance.now()) / 1000;
  if (left > 0) { setMsg((capture.kind === "neutral" ? "Hold still… " : "Bend forward and back… ") + left.toFixed(1) + " s", "busy"); return; }
  const c = capture; capture = null;
  if (c.kind === "neutral") finishNeutral(c); else finishFlex(c);
}

function finishNeutral(c) {
  const q0 = {}, spread = [];
  for (const [n, list] of Object.entries(c.samples)) {
    if (list.length < 20) { setMsg("Too few samples; try again.", "warn"); return; }
    q0[n] = qmean(list);
    const worst = Math.max(...list.map((q) => Math.hypot(...rotvec(qmul(q, qconj(q0[n])))) * D));
    spread.push(worst);
  }
  if (Math.max(...spread) > 3) { setMsg(`Subject moved (${Math.max(...spread).toFixed(1)}° during capture). Stand still and repeat.`, "warn"); return; }
  cal.q0 = q0; cal.map = c.map; cal.when = new Date().toISOString();
  // Keep the previous axis only if it belongs to the same sensors; otherwise fall back to world X.
  if (!cal.B) cal.B = null;
  persist();
  setMsg(cal.B ? "Neutral captured. Previous flexion axis kept; redo step 2 if the subject turned." : "Neutral captured. Now do step 2.", cal.B ? "ok" : "");
}

function finishFlex(c) {
  // Sum the world-frame rotation vectors of both segments; their horizontal direction at large
  // angles is the flexion axis. Samples under 10 deg are ignored (noise, standing phase).
  let ax = [0, 0, 0], maxDeg = 0;
  for (const [n, list] of Object.entries(c.samples)) {
    for (const q of list) {
      const r = rotvec(qmul(q, qconj(cal.q0[n])));
      const deg = Math.hypot(...r) * D;
      maxDeg = Math.max(maxDeg, deg);
      if (deg < 10) continue;
      ax = ax.map((v, i) => v + r[i]);
    }
  }
  const h = Math.hypot(ax[0], ax[1]);
  if (maxDeg < 15 || h < 1e-6) { setMsg(`Only ${maxDeg.toFixed(0)}° of bending seen. Bend further (40° or more) and repeat.`, "warn"); return; }
  const tilt = Math.atan2(Math.abs(ax[2]), h) * D;
  const X = [ax[0] / h, ax[1] / h, 0], Z = [0, 0, 1];
  const Y = [Z[1] * X[2] - Z[2] * X[1], Z[2] * X[0] - Z[0] * X[2], Z[0] * X[1] - Z[1] * X[0]];
  cal.B = qFromAxes(X, Y, Z);
  persist();
  setMsg(`Calibrated. Peak bend ${maxDeg.toFixed(0)}°` + (tilt > 25 ? `; the bend included ${tilt.toFixed(0)}° of twist, so repeat it straighter if angles look mixed.` : "."), tilt > 25 ? "warn" : "ok");
}

function persist() {
  save("spineCal", cal);
  updateCalUi();
  fetch("/api/spine_cal", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(cal) })
    .then((r) => r.json()).then((j) => { if (j.ok && j.cal) { spineRev = j.rev; adoptCal(j.cal); } }).catch(() => {});
}

// The server holds the master calibration: load it at start and whenever its revision changes.
let spineRev = null;
function adoptCal(c) {
  for (const k of Object.keys(cal)) if (!(k in c)) delete cal[k];
  Object.assign(cal, c);
  save("spineCal", cal);
  $("heightCm").value = cal.height_cm || "";
  updateCalUi(); applyScale();
}
async function syncCal() {
  try {
    const j = await (await fetch("/api/spine_cal", { cache: "no-store" })).json();
    if (j.ok) { spineRev = j.rev; if (j.cal) adoptCal(j.cal); }
  } catch { /* server offline: keep the local copy */ }
}

// Spine rendering needs both parts of the static calibration: neutral (q0 of the mapped sensors) and
// the forward-bend body axes (B). Until then the model stays in a static pose behind a blurred gate.
function neutralOk() {
  if (!cal.q0) return false;
  const m = mapping(), names = [m.T7, m.T12].filter(Boolean);
  return names.length === 2 ? names.every((n) => cal.q0[n]) : Object.keys(cal.q0).length >= 2;
}
function isCalibrated() { return neutralOk() && !!cal.B; }

function updateGate() {
  const still = neutralOk(), bend = still && !!cal.B, ok = still && bend;
  document.documentElement.classList.toggle("uncal", !ok);
  $("calGate").classList.toggle("hidden", ok);
  if (ok) return;
  $("calGateTitle").textContent = still ? "Forward bend not done" : "Static calibration not done";
  $("calGateText").textContent = still
    ? "The neutral posture is set, but the body's left–right axis is missing: flexion, side bend and rotation would be mixed."
    : "The spine model is shown once the participant's neutral posture and body axes are known.";
  $("cgStill").className = still ? "done" : "todo";
  $("cgBend").className = bend ? "done" : still ? "todo" : "";
  const embed = document.documentElement.classList.contains("embed");
  $("calGateWhere").textContent = embed ? "Step 3 · Static calibration, in the middle column"
    : "Step 3 on the Acquisition page, or Calibration on the left";
}

function updateCalUi() {
  $("stepN").classList.toggle("done", !!cal.q0);
  $("stepF").classList.toggle("done", !!cal.B);
  const chip = $("calChip");
  chip.textContent = cal.q0 && cal.B ? "calibrated " + new Date(cal.when).toLocaleTimeString() : cal.q0 ? "neutral only" : "not calibrated";
  updateGate();
}

// ---------------- UI ----------------
function fillMap() {
  const saved = load("spineMap") || {};
  for (const [id, seg, def] of [["mapT7", "T7", 0], ["mapT12", "T12", 1]]) {
    const sel = $(id), cur = sel.value || saved[seg];
    sel.innerHTML = podNames.map((n) => `<option>${n}</option>`).join("");
    sel.value = podNames.includes(cur) ? cur : podNames[def] || podNames[0] || "";
  }
}
for (const id of ["mapT7", "mapT12"]) $(id).addEventListener("change", () => { save("spineMap", mapping()); updateGate(); });
$("btnNeutral").onclick = () => startCapture("neutral", 3);
$("btnFlex").onclick = () => startCapture("flex", 6);
for (const id of ["showRibs", "showTrail"]) {          // shared with the main page's spine panel
  const v = load(id); if (v != null) $(id).checked = !!v;
  $(id).addEventListener("change", () => save(id, $(id).checked));
}
// options from the main page (embedded copy) and from other tabs
function applyOpts(o) {
  if (o.view && three) setView(o.view);
  if (o.ribs != null) $("showRibs").checked = !!o.ribs;
  if (o.trail != null) $("showTrail").checked = !!o.trail;
  if ("height" in o) {
    if (o.height == null) delete cal.height_cm; else cal.height_cm = o.height;
    $("heightCm").value = cal.height_cm || ""; applyScale();
  }
}
window.addEventListener("message", (e) => {
  if (e.origin !== location.origin || !e.data) return;
  if (e.data.type === "spineopt") applyOpts(e.data);
  else if (e.data.type === "spinecal" && e.data.cal) adoptCal(e.data.cal);   // from the main page
});

// Participant height -> uniform model scale (Gait2392 generic subject is 1.80 m tall)
const MODEL_HEIGHT_CM = 180;
function modelScale() {
  const h = +cal.height_cm;
  return h >= 100 && h <= 230 ? h / MODEL_HEIGHT_CM : 1;
}
function applyScale() {
  const s = modelScale();
  $("scaleV").textContent = "×" + s.toFixed(2);
  if (three) { three.root.scale.setScalar(s); setView(three.view || "iso"); }
}
if (cal.height_cm) $("heightCm").value = cal.height_cm;
$("heightCm").addEventListener("change", () => {
  const h = +$("heightCm").value;
  if ($("heightCm").value === "") delete cal.height_cm;
  else if (h >= 100 && h <= 230) cal.height_cm = h;
  else { $("heightCm").value = cal.height_cm || ""; return; }
  persist(); applyScale();
});
// the embedded copy in the main GUI follows changes made on the Spine page
window.addEventListener("storage", (e) => {
  if (e.key === "showRibs" || e.key === "showTrail") { $(e.key).checked = !!load(e.key); return; }
  if (e.key !== "spineCal") return;
  const c = load("spineCal"); if (!c) return;
  Object.assign(cal, c); if (!("height_cm" in c)) delete cal.height_cm;
  $("heightCm").value = cal.height_cm || ""; updateCalUi(); applyScale();
});

function angleBox(el, a) {
  const names = [["flex", "Flexion", "+fwd"], ["lat", "Lat. bend", "+left"], ["axi", "Axial rot.", "+left"]];
  if (!el.children.length) el.innerHTML = names.map(([c, n, s]) => `<div class="a ${c}"><div class="v">—</div><div class="n">${n} ${s}</div></div>`).join("");
  [...el.children].forEach((c, i) => { c.firstChild.textContent = a ? a[i].toFixed(1) + "°" : "—"; });
}

// ---------------- stream ----------------
function connect() {
  const es = openStream();          // one shared /api/stream for all tabs (stream.js)
  es.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.s) {
      if (m.s.spine_cal_rev != null && spineRev != null && m.s.spine_cal_rev !== spineRev) { spineRev = m.s.spine_cal_rev; syncCal(); }
      status = m.s;
      const pods = status.hello && status.hello.pods ? status.hello.pods.split(",").map((s) => s.split(":")[0]) : [];
      if (pods.join() !== podNames.join()) { podNames = pods; fillMap(); updateGate(); }
      $("connDot").className = "dot " + (status.state && status.state !== "disconnected" ? "ok" : "");
      $("connText").textContent = status.port ? status.port : "No board";
      const b = $("stateBadge"); b.textContent = status.state === "recording" ? "REC" : (status.state || "—").toUpperCase();
      b.className = "badge" + (status.state === "recording" ? " rec" : "");
    }
    if (m.f && m.f.length && podNames.length) {
      for (const r of m.f) {
        for (let k = 0; k < podNames.length; k++) {
          const o = 3 + 6 * k;
          if (r.length < o + 6 || !r[o]) continue;
          live[podNames[k]] = [r[o + 2], r[o + 3], r[o + 4], r[o + 5]];
        }
        onCaptureSample();
        const mp = mapping(), a = segment(mp.T7), b = segment(mp.T12);
        if (a && b) {
          hist.t.push(r[0] / 1e6); hist.a.push(eulerXYZ(qmul(qconj(b), a)));
          const cut = hist.t[hist.t.length - 1] - H;
          while (hist.t.length && (hist.t[0] < cut || hist.t[0] > hist.t[hist.t.length - 1])) { hist.t.shift(); hist.a.shift(); }
        }
      }
    }
  };
  es.onerror = () => { $("connDot").className = "dot"; $("connText").textContent = "Server offline"; };
}

// ---------------- 3D model ----------------
// Bone meshes: OpenSim 4.5 Gait2392 geometry (sacrum, hip bones, L5-C1 vertebrae with discs, ribs,
// skull), prepared by models/build_model.py into static/models/spine_opensim.json. Frame = OpenSim
// pelvis frame (X anterior, Y up, Z right), cm. Each segment rotates about the disc below it.
// Two sensors only (T7, T12), no pelvis sensor: a fixed 70% of T12's rotation is spread over L5-T12,
// the rest is pelvis tilt. Replaced by measured segments once all 7 IMUs are mapped.
const LUMBAR_SHARE = 0.7;
const ORDER = ["pelvis", "L5", "L4", "L3", "L2", "L1", "T12", "T11", "T10", "T9", "T8", "T7", "T6", "T5", "T4", "T3", "T2", "T1"];
const idx = Object.fromEntries(ORDER.map((n, i) => [n, i]));
let three = null;

// body frame (X left, Y posterior, Z up) -> OpenSim frame (X anterior, Y up, Z right)
const toOs = (q) => new THREE.Quaternion(-q[2], q[3], -q[1], q[0]);

async function initThree() {
  const host = $("three");
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
  host.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(32, 1, 1, 2000);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xffffff, 0x6b7280, 1.4));
  const key = new THREE.DirectionalLight(0xffffff, 1.8); key.position.set(80, 120, 60); scene.add(key);
  const rim = new THREE.DirectionalLight(0xbfd4ff, 0.8); rim.position.set(-80, 40, -90); scene.add(rim);
  const grid = new THREE.GridHelper(120, 24, 0x8a93a3, 0x8a93a3);
  grid.material.opacity = 0.18; grid.material.transparent = true; grid.position.y = -12; scene.add(grid);

  $("overlay").textContent = "Loading OpenSim bone meshes…";
  const model = await (await fetch("/static/models/spine_opensim.json")).json();
  const mats = {
    bone: new THREE.MeshStandardMaterial({ color: 0xeee6d3, roughness: 0.55, metalness: 0 }),
    sensorBone: new THREE.MeshStandardMaterial({ color: 0x9db4f0, roughness: 0.5 }),
    disc: new THREE.MeshStandardMaterial({ color: 0x6fb7cf, roughness: 0.35 }),
    ribs: new THREE.MeshStandardMaterial({ color: 0xeee6d3, roughness: 0.6, transparent: true, opacity: 0.7, depthWrite: false }),
    head: new THREE.MeshStandardMaterial({ color: 0xeee6d3, roughness: 0.6, transparent: true, opacity: 0.85 }),
  };
  const sensorMat = new THREE.MeshStandardMaterial({ color: 0xea7a1a, roughness: 0.4, emissive: 0x3a1a00 });

  // node hierarchy: each node sits at its pivot; meshes are offset by -pivot so they stay in place
  const nodes = [], ribs = [], piv = model.pivots;
  let parent = scene, prev = [0, 0, 0];
  const root = new THREE.Group(); root.position.y = 0; scene.add(root); parent = root;
  for (const name of ORDER) {
    const p = piv[name], node = new THREE.Group();
    node.position.set(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]);
    parent.add(node);
    const holder = new THREE.Group(); holder.position.set(-p[0], -p[1], -p[2]); node.add(holder);
    nodes.push({ name, node, holder });
    parent = node; prev = p;
  }
  for (const m of model.meshes) {
    const i = idx[m.segment]; if (i === undefined) continue;
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(m.pos, 3));
    g.setIndex(m.idx);
    const geo = mergeVertices(g, 1e-3); geo.computeVertexNormals();
    const isSensor = m.kind === "bone" && (m.segment === "T7" || m.segment === "T12");
    const mesh = new THREE.Mesh(geo, isSensor ? mats.sensorBone : mats[m.kind] || mats.bone);
    if (m.kind === "ribs") { ribs.push(mesh); mesh.renderOrder = 2; }
    nodes[i].holder.add(mesh);
    if (isSensor) {                 // IMU on the spinous-process tip = most posterior point
      const a = geo.attributes.position; let best = 0;
      for (let k = 1; k < a.count; k++) if (a.getX(k) < a.getX(best)) best = k;
      const s = new THREE.Mesh(new THREE.BoxGeometry(0.5, 2.0, 2.6), sensorMat);
      s.position.set(a.getX(best) - 0.45, a.getY(best), a.getZ(best));
      nodes[i].holder.add(s);
    }
  }
  const trailN = 400, trailPos = new Float32Array(trailN * 3);
  const trailGeo = new THREE.BufferGeometry(); trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xea7a1a })); trail.frustumCulled = false; scene.add(trail);
  three = { renderer, scene, camera, controls, nodes, ribs, trail, trailPos, trailN, trailCount: 0, host, root, view: "iso" };
  three.view = load("spineView") || "iso";
  applyScale();
  new ResizeObserver(resize).observe(host); resize();
  $("overlay").classList.add("hidden");
}

function resize() {
  const { renderer, camera, host } = three;
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h, false); renderer.domElement.style.width = w + "px"; renderer.domElement.style.height = h + "px";
  camera.aspect = w / Math.max(1, h);
  // embedded in a tall, narrow panel: widen the vertical field of view so the model's width still fits
  const narrow = document.documentElement.classList.contains("embed") && camera.aspect < 0.5;
  camera.fov = narrow ? 2 * Math.atan(Math.tan(16 * Math.PI / 180) * 0.5 / camera.aspect) * 180 / Math.PI : 32;
  camera.updateProjectionMatrix();
}

function setView(v) {      // subject faces +X; right side is +Z
  const s = modelScale();                     // keep the scaled model framed the same way
  const T = [-6, 32, 0].map((c) => c * s);
  const p = { side: [-6, 38, 190], front: [190, 38, 0], back: [-200, 38, 0], iso: [-140, 85, 135] }[v].map((c) => c * s);
  three.view = v;
  three.camera.position.set(...p); three.controls.target.set(...T); three.controls.update();
}
document.querySelectorAll("[data-view]").forEach((b) => (b.onclick = () => { if (three) { setView(b.dataset.view); save("spineView", b.dataset.view); } }));

const wp = new THREE.Vector3();
function updateModel() {
  const mp = mapping(), live3d = isCalibrated();       // not calibrated: static neutral model, no angles
  const qT7 = live3d ? segment(mp.T7) : null, qT12 = live3d ? segment(mp.T12) : null;
  const share = LUMBAR_SHARE;
  const J = ORDER.map(() => [1, 0, 0, 0]);       // joint rotation of each segment w.r.t. the one below
  if (qT7 && qT12) {
    const pel = qpow(qT12, 1 - share), lumb = qpow(qmul(qconj(pel), qT12), 1 / (idx.T12 - idx.pelvis));
    J[idx.pelvis] = pel;
    for (let i = idx.L5; i <= idx.T12; i++) J[i] = lumb;
    const thor = qpow(qmul(qconj(qT12), qT7), 1 / (idx.T7 - idx.T12));
    for (let i = idx.T12 + 1; i <= idx.T7; i++) J[i] = thor;
  }
  three.nodes.forEach((nd, i) => nd.node.quaternion.copy(toOs(J[i])));
  for (const r of three.ribs) r.visible = $("showRibs").checked;
  const t = three; t.trail.visible = $("showTrail").checked;
  if (t.trail.visible && qT7) {
    const n7 = t.nodes[idx.T7].node; n7.updateWorldMatrix(true, false);
    wp.set(-6, 3, 0).applyMatrix4(n7.matrixWorld);
    t.trailPos.copyWithin(0, 3); t.trailPos.set([wp.x, wp.y, wp.z], (t.trailN - 1) * 3);
    if (t.trailCount < t.trailN) { for (let k = 0; k < t.trailN - 1; k++) t.trailPos.set([wp.x, wp.y, wp.z], k * 3); t.trailCount = t.trailN; }
    t.trail.geometry.attributes.position.needsUpdate = true;
  } else t.trailCount = 0;
  angleBox($("angRel"), qT7 && qT12 ? eulerXYZ(qmul(qconj(qT12), qT7)) : null);
  angleBox($("angT12"), qT12 ? eulerXYZ(qT12) : null);
  angleBox($("angT7"), qT7 ? eulerXYZ(qT7) : null);
}

// ---------------- plot ----------------
function css(v) { return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }
function drawPlot() {
  const c = $("plot"), dpr = window.devicePixelRatio || 1, W = c.clientWidth, Hh = c.clientHeight;
  if (c.width !== W * dpr) { c.width = W * dpr; c.height = Hh * dpr; }
  const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, Hh);
  if (hist.t.length < 2) { g.fillStyle = css("--text-3"); g.font = "12px sans-serif"; g.fillText(cal.q0 ? "waiting for data…" : "calibrate to see angles", 8, 20); return; }
  const t1 = hist.t[hist.t.length - 1], t0 = t1 - H;
  let lim = 10;
  for (const a of hist.a) for (const v of a) lim = Math.max(lim, Math.abs(v));
  lim = Math.ceil(lim / 10) * 10;
  const y = (v) => Hh / 2 - (v / lim) * (Hh / 2 - 6), x = (t) => ((t - t0) / H) * W;
  g.strokeStyle = css("--border"); g.lineWidth = 1; g.beginPath(); g.moveTo(0, Hh / 2); g.lineTo(W, Hh / 2); g.stroke();
  g.fillStyle = css("--text-3"); g.font = "10px sans-serif"; g.fillText(`±${lim}°`, 4, 11);
  ["--roll", "--pitch", "--yaw"].forEach((col, k) => {
    g.strokeStyle = css(col); g.lineWidth = 1.6; g.beginPath();
    hist.t.forEach((t, i) => (i ? g.lineTo(x(t), y(hist.a[i][k])) : g.moveTo(x(t), y(hist.a[i][k]))));
    g.stroke();
  });
}

function loop() {
  if (three) { updateModel(); three.controls.update(); three.renderer.render(three.scene, three.camera); }
  drawPlot();
  requestAnimationFrame(loop);
}

updateCalUi();
syncCal();
if (cal.q0) setMsg(cal.B ? "Loaded the last calibration. Recalibrate if the sensors were moved or restarted." : "Neutral loaded; do step 2.", cal.B ? "" : "");
initThree().catch((e) => { $("overlay").textContent = "3D view failed: " + e.message; });
connect();
requestAnimationFrame(loop);
