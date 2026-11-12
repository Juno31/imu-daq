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
  fetch("/api/spine_cal", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(cal) }).catch(() => {});
}

function updateCalUi() {
  $("stepN").classList.toggle("done", !!cal.q0);
  $("stepF").classList.toggle("done", !!cal.B);
  const chip = $("calChip");
  chip.textContent = cal.q0 && cal.B ? "calibrated " + new Date(cal.when).toLocaleTimeString() : cal.q0 ? "neutral only" : "not calibrated";
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
for (const id of ["mapT7", "mapT12"]) $(id).addEventListener("change", () => save("spineMap", mapping()));
$("btnNeutral").onclick = () => startCapture("neutral", 3);
$("btnFlex").onclick = () => startCapture("flex", 6);
$("lumbarShare").oninput = () => { $("lumbarShareV").textContent = $("lumbarShare").value + "%"; save("lumbarShare", $("lumbarShare").value); };
{ const v = load("lumbarShare"); if (v != null) { $("lumbarShare").value = v; $("lumbarShareV").textContent = v + "%"; } }

function angleBox(el, a) {
  const names = [["flex", "Flexion", "+fwd"], ["lat", "Lat. bend", "+left"], ["axi", "Axial rot.", "+left"]];
  if (!el.children.length) el.innerHTML = names.map(([c, n, s]) => `<div class="a ${c}"><div class="v">—</div><div class="n">${n} ${s}</div></div>`).join("");
  [...el.children].forEach((c, i) => { c.firstChild.textContent = a ? a[i].toFixed(1) + "°" : "—"; });
}

// ---------------- stream ----------------
function connect() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.s) {
      status = m.s;
      const pods = status.hello && status.hello.pods ? status.hello.pods.split(",").map((s) => s.split(":")[0]) : [];
      if (pods.join() !== podNames.join()) { podNames = pods; fillMap(); }
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
// Vertebrae bottom -> top. h = distance to the next vertebra (cm), n = neutral joint angle to the
// vertebra below (deg, + = flexion); gives ~45 deg lordosis and ~40 deg kyphosis.
const VERT = [
  ["S1", 4.2, 34], ["L5", 3.9, -14], ["L4", 3.8, -12], ["L3", 3.7, -10], ["L2", 3.6, -7], ["L1", 3.4, -4],
  ["T12", 3.0, 2], ["T11", 2.8, 3], ["T10", 2.7, 4], ["T9", 2.6, 4], ["T8", 2.5, 4], ["T7", 2.4, 4],
  ["T6", 2.3, 4], ["T5", 2.2, 4], ["T4", 2.2, 4], ["T3", 2.1, 3], ["T2", 2.1, 2], ["T1", 2.0, 0],
];
const idx = Object.fromEntries(VERT.map((v, i) => [v[0], i]));
let three = null;

function initThree() {
  const host = $("three");
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
  host.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(35, 1, 1, 1000);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 32, 0);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8090a0, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(40, 90, 60); scene.add(sun);
  const grid = new THREE.GridHelper(80, 16, 0x8a93a3, 0x8a93a3);
  grid.material.opacity = 0.25; grid.material.transparent = true; scene.add(grid);

  // root: body frame (X left, Y posterior, Z up) -> three.js (Y up). Subject faces +Z in three.
  const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; root.position.y = 4; scene.add(root);
  const bone = new THREE.MeshStandardMaterial({ color: 0xe9e1cf, roughness: 0.6 });
  const boneThor = new THREE.MeshStandardMaterial({ color: 0xdcd6ea, roughness: 0.6 });
  const live1 = new THREE.MeshStandardMaterial({ color: 0x3867d6, roughness: 0.5 });
  const disc = new THREE.MeshStandardMaterial({ color: 0x7cc4d8, roughness: 0.4, transparent: true, opacity: 0.85 });
  const sensorMat = new THREE.MeshStandardMaterial({ color: 0xea7a1a, roughness: 0.4, emissive: 0x3a1a00 });
  const ribMat = new THREE.MeshStandardMaterial({ color: 0xe9e1cf, roughness: 0.7, transparent: true, opacity: 0.55 });

  const nodes = [], ribs = [];
  let parent = root, prevH = 0;
  for (const [name, h, n] of VERT) {
    const node = new THREE.Group();
    node.position.set(0, 0, prevH);
    parent.add(node);
    const lumbar = name[0] === "L" || name === "S1";
    const r = lumbar ? 2.0 : 1.4 + 0.04 * (12 - Math.min(12, +name.slice(1) || 12));
    const sensorHere = name === "T7" || name === "T12";
    const mat = sensorHere ? live1 : lumbar ? bone : boneThor;
    if (name === "S1") {
      const sac = new THREE.Mesh(new THREE.CylinderGeometry(3.4, 1.4, 9, 20), bone);
      sac.rotation.x = Math.PI / 2; sac.position.set(0, 1.0, -3.6); node.add(sac);
      const pelvis = new THREE.Mesh(new THREE.TorusGeometry(8, 1.1, 12, 40, Math.PI * 1.25), bone);
      pelvis.rotation.z = -Math.PI * 0.125 - Math.PI / 2; pelvis.position.set(0, -4, -2.5); node.add(pelvis);
    }
    const body = new THREE.Mesh(new THREE.CylinderGeometry(r, r * 1.05, h * 0.68, 24), mat);
    body.rotation.x = Math.PI / 2; body.position.set(0, -r * 0.9, h * 0.34); node.add(body);
    if (name !== "T1") {
      const d = new THREE.Mesh(new THREE.CylinderGeometry(r * 0.95, r * 0.95, h * 0.28, 24), disc);
      d.rotation.x = Math.PI / 2; d.position.set(0, -r * 0.9, h * 0.84); node.add(d);
    }
    const spLen = lumbar ? 3.0 : 3.4, spDown = lumbar ? 0.15 : 0.6;
    const sp = new THREE.Mesh(new THREE.BoxGeometry(0.7, spLen, 1.1), mat);
    sp.position.set(0, spLen / 2 + 0.3, h * 0.35); sp.rotation.x = -spDown; node.add(sp);
    const tp = new THREE.Mesh(new THREE.BoxGeometry(lumbar ? 7 : 5.4, 0.8, 0.8), mat);
    tp.position.set(0, 0.6, h * 0.4); node.add(tp);
    if (!lumbar) {
      const k = +name.slice(1);
      const w = 6 + 7 * Math.sin(Math.PI * Math.min(1, (k + 1) / 9)) - (k > 9 ? (k - 9) * 1.2 : 0);
      for (const side of [-1, 1]) {
        // hoop centred in front of the spine; each arc starts at the posterior point (angle +90 deg,
        // at the transverse process) and runs round one side towards the sternum
        const arc = Math.PI * 0.8;
        const rib = new THREE.Mesh(new THREE.TorusGeometry(w, 0.35, 8, 30, arc), ribMat);
        const tilt = new THREE.Group();
        tilt.position.set(0, 0.6, h * 0.4); tilt.rotation.x = 0.35;   // ribs slope downward to the front
        rib.position.set(0, -w, 0); rib.rotation.z = side > 0 ? Math.PI / 2 - arc : Math.PI / 2;
        rib.scale.set(0.95, 0.8, 1);
        tilt.add(rib); node.add(tilt); ribs.push(rib);
      }
    }
    if (sensorHere) {
      const s = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.5, 2.0), sensorMat);
      s.position.set(0, spLen + 0.6, h * 0.35); node.add(s);
    }
    nodes.push({ name, node, n: n / D });
    parent = node; prevH = h;
  }
  // T7 path trail
  const trailN = 400, trailPos = new Float32Array(trailN * 3);
  const trailGeo = new THREE.BufferGeometry(); trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xea7a1a })); trail.frustumCulled = false; scene.add(trail);
  three = { renderer, scene, camera, controls, nodes, ribs, trail, trailPos, trailN, trailCount: 0, host };
  setView("iso");
  new ResizeObserver(resize).observe(host); resize();
  $("overlay").classList.add("hidden");
}

function resize() {
  const { renderer, camera, host } = three;
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h, false); renderer.domElement.style.width = w + "px"; renderer.domElement.style.height = h + "px";
  camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix();
}

function setView(v) {
  const p = { side: [-120, 36, 0], front: [0, 36, 120], back: [0, 36, -120], iso: [-85, 55, 80] }[v];
  three.camera.position.set(...p); three.controls.target.set(0, 32, 0); three.controls.update();
}
document.querySelectorAll("[data-view]").forEach((b) => (b.onclick = () => three && setView(b.dataset.view)));

const tmpQ = new THREE.Quaternion(), neutralQ = new THREE.Quaternion(), axisX = new THREE.Vector3(1, 0, 0), wp = new THREE.Vector3();
function updateModel() {
  const mp = mapping();
  const qT7 = segment(mp.T7), qT12 = segment(mp.T12);
  const share = +$("lumbarShare").value / 100;
  // joint rotations (body frame, child relative to parent), on top of the neutral curve
  const J = VERT.map(() => [1, 0, 0, 0]);
  if (qT7 && qT12) {
    const lumbJoints = idx.T12 - idx.S1;                      // L5..T12 joints (6)
    const pel = qpow(qT12, 1 - share), lumb = qpow(qmul(qconj(pel), qT12), 1 / lumbJoints);
    J[idx.S1] = pel;
    for (let i = idx.S1 + 1; i <= idx.T12; i++) J[i] = lumb;
    const rel = qmul(qconj(qT12), qT7), thor = qpow(rel, 1 / (idx.T7 - idx.T12));
    for (let i = idx.T12 + 1; i <= idx.T7; i++) J[i] = thor;
    // above T7: rigid with T7 (not instrumented)
  }
  three.nodes.forEach((nd, i) => {
    neutralQ.setFromAxisAngle(axisX, nd.n);
    const q = J[i];
    tmpQ.set(q[1], q[2], q[3], q[0]);
    nd.node.quaternion.copy(neutralQ).multiply(tmpQ);
  });
  for (const r of three.ribs) r.visible = $("showRibs").checked;
  // trail
  const t = three; t.trail.visible = $("showTrail").checked;
  if (t.trail.visible && qT7) {
    t.nodes[idx.T7].node.updateWorldMatrix(true, false);
    wp.setFromMatrixPosition(t.nodes[idx.T7].node.matrixWorld);
    t.trailPos.copyWithin(0, 3); t.trailPos.set([wp.x, wp.y, wp.z], (t.trailN - 1) * 3);
    if (t.trailCount < t.trailN) { for (let k = 0; k < t.trailN - 1 - t.trailCount; k++) t.trailPos.set([wp.x, wp.y, wp.z], k * 3); t.trailCount = t.trailN; }
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
if (cal.q0) setMsg(cal.B ? "Loaded the last calibration. Recalibrate if the sensors were moved or restarted." : "Neutral loaded; do step 2.", cal.B ? "" : "");
try { initThree(); } catch (e) { $("overlay").textContent = "3D view failed: " + e.message; }
connect();
requestAnimationFrame(loop);
