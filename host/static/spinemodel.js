// 3D spine model for the Review page: same OpenSim Gait2392 meshes and pose rules as the Spine view
// (spine.js). Pose input: T7 and T12 orientations relative to standing, in the body frame
// (X left, Y posterior, Z up). T12's motion is shared between pelvis and lumbar levels.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { mergeVertices } from "three/addons/utils/BufferGeometryUtils.js";

export const qmul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
export const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
export const qnorm = (q) => { const n = Math.hypot(...q) || 1; return q.map((v) => v / n); };
const qpos = (q) => (q[0] < 0 ? q.map((v) => -v) : q);
export function qpow(q, f) {
  q = qpos(q);
  const s = Math.hypot(q[1], q[2], q[3]);
  if (s < 1e-9) return [1, 0, 0, 0];
  const th = 2 * Math.atan2(s, q[0]) * f, k = Math.sin(th / 2) / s;
  return [Math.cos(th / 2), q[1] * k, q[2] * k, q[3] * k];
}
export function eulerXYZ(q) {                // intrinsic X-Y-Z -> [flexion, lateral, axial] deg
  const D = 180 / Math.PI, [w, x, y, z] = q;
  const r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
  const r12 = 2 * (y * z - w * x), r22 = 1 - 2 * (x * x + y * y);
  return [Math.atan2(-r12, r22) * D, Math.asin(Math.max(-1, Math.min(1, r02))) * D, Math.atan2(-r01, r00) * D];
}
// segment rotation from standing in the body frame: B^-1 (q q0^-1) B
export function segment(q, q0, B) {
  if (!q || !q0 || !isFinite(q[0])) return null;
  const g = qmul(q, qconj(q0)), b = B || [1, 0, 0, 0];
  return qnorm(qmul(qmul(qconj(b), g), b));
}

const ORDER = ["pelvis", "L5", "L4", "L3", "L2", "L1", "T12", "T11", "T10", "T9", "T8", "T7", "T6", "T5", "T4", "T3", "T2", "T1"];
const idx = Object.fromEntries(ORDER.map((n, i) => [n, i]));
const toOs = (q) => new THREE.Quaternion(-q[2], q[3], -q[1], q[0]);   // body frame -> OpenSim frame
const MODEL_HEIGHT_CM = 180;

export async function createSpine(host) {
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

  const model = await (await fetch("/static/models/spine_opensim.json")).json();
  const mats = {
    bone: new THREE.MeshStandardMaterial({ color: 0xeee6d3, roughness: 0.55 }),
    sensorBone: new THREE.MeshStandardMaterial({ color: 0x9db4f0, roughness: 0.5 }),
    disc: new THREE.MeshStandardMaterial({ color: 0x6fb7cf, roughness: 0.35 }),
    ribs: new THREE.MeshStandardMaterial({ color: 0xeee6d3, roughness: 0.6, transparent: true, opacity: 0.7, depthWrite: false }),
    head: new THREE.MeshStandardMaterial({ color: 0xeee6d3, roughness: 0.6, transparent: true, opacity: 0.85 }),
  };
  const sensorMat = new THREE.MeshStandardMaterial({ color: 0xea7a1a, roughness: 0.4, emissive: 0x3a1a00 });
  const nodes = [], ribs = [], piv = model.pivots, root = new THREE.Group();
  scene.add(root);
  let parent = root, prev = [0, 0, 0];
  for (const name of ORDER) {
    const p = piv[name], node = new THREE.Group();
    node.position.set(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]);
    parent.add(node);
    const holder = new THREE.Group(); holder.position.set(-p[0], -p[1], -p[2]); node.add(holder);
    nodes.push({ node, holder });
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
    if (isSensor) {
      const a = geo.attributes.position; let best = 0;
      for (let k = 1; k < a.count; k++) if (a.getX(k) < a.getX(best)) best = k;
      const s = new THREE.Mesh(new THREE.BoxGeometry(0.5, 2.0, 2.6), sensorMat);
      s.position.set(a.getX(best) - 0.45, a.getY(best), a.getZ(best));
      nodes[i].holder.add(s);
    }
  }

  let scale = 1, view = "side", dirty = true;
  function resize() {
    const w = host.clientWidth, h = host.clientHeight;
    renderer.setSize(w, h, false); renderer.domElement.style.width = w + "px"; renderer.domElement.style.height = h + "px";
    camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix(); dirty = true;
  }
  function setView(v) {
    view = v;
    const T = [-6, 32, 0].map((c) => c * scale);
    const p = { side: [-6, 38, 190], front: [190, 38, 0], back: [-200, 38, 0], iso: [-140, 85, 135] }[v].map((c) => c * scale);
    camera.position.set(...p); controls.target.set(...T); controls.update(); dirty = true;
  }
  new ResizeObserver(resize).observe(host); resize(); setView(view);
  controls.addEventListener("change", () => { dirty = true; });

  (function loop() {                         // render only when something changed
    controls.update();
    if (dirty) { renderer.render(scene, camera); dirty = false; }
    requestAnimationFrame(loop);
  })();

  return {
    setView,
    setHeight(cm) { scale = cm >= 100 && cm <= 230 ? cm / MODEL_HEIGHT_CM : 1; root.scale.setScalar(scale); setView(view); },
    setRibs(on) { for (const r of ribs) r.visible = on; dirty = true; },
    pose(qT7, qT12, share = 0.7) {          // null = neutral standing pose
      const J = ORDER.map(() => [1, 0, 0, 0]);
      if (qT7 && qT12) {
        const pel = qpow(qT12, 1 - share), lumb = qpow(qmul(qconj(pel), qT12), 1 / (idx.T12 - idx.pelvis));
        J[idx.pelvis] = pel;
        for (let i = idx.L5; i <= idx.T12; i++) J[i] = lumb;
        const thor = qpow(qmul(qconj(qT12), qT7), 1 / (idx.T7 - idx.T12));
        for (let i = idx.T12 + 1; i <= idx.T7; i++) J[i] = thor;
      }
      nodes.forEach((nd, i) => nd.node.quaternion.copy(toOs(J[i])));
      dirty = true;
    },
  };
}
