"""Build host/static/models/spine_opensim.json from the OpenSim 4.5 Gait2392 torso/pelvis meshes.
Frame: OpenSim pelvis frame (X anterior, Y up, Z right), centimetres. Each group rotates about
its pivot (the intervertebral disc below it; the pelvis about the hip-centre midpoint)."""
import json, numpy as np
from vtpread import read
from comps import comps
G = "C:/OpenSim 4.5/Geometry/"
BACK = np.array([-0.1007, 0.0815, 0.0])          # back joint in pelvis frame
HAT = np.array([0.0205, 0.091, 0.0])             # spine.vtp -> hat (torso) frame
RIBS = np.array([0.0205, 0.3785, 0.0])           # ribs.vtp  -> hat (torso) frame

groups = {}                                      # name -> list of (pts, tris)
def add(g, pts, tri):
    groups.setdefault(g, []).append((pts, tri))

def sub(pts, tri, mask):
    idx = np.flatnonzero(mask); remap = -np.ones(len(pts), int); remap[idx] = np.arange(len(idx))
    keep = mask[tri].all(1)
    return pts[idx], remap[tri[keep]]

# pelvis: sacrum + both hip bones (pelvis frame already)
for m in ("sacrum", "r_pelvis", "l_pelvis"):
    p, t = read(G + m + ".vtp"); add("pelvis", p, t)

# spine: split into vertebrae (>=52 pts) and discs (26 pts)
p, t = read(G + "spine.vtp"); p = p + HAT + BACK
r = comps(p, t)
parts = [(p[r == k].mean(0), k, (r == k).sum()) for k in np.unique(r)]
verts = sorted([x for x in parts if x[2] >= 52], key=lambda x: x[0][1])
discs = sorted([x for x in parts if x[2] < 52], key=lambda x: x[0][1])
names = ["L5", "L4", "L3", "L2", "L1"] + [f"T{i}" for i in range(12, 0, -1)]
pivots = {"pelvis": [0.0, 0.0, 0.0]}
for i, (c, k, n) in enumerate(verts):
    g = names[i] if i < len(names) else "T1"            # C7..C1 ride with T1
    add(g, *sub(p, t, r == k))
    if i < len(names):
        below = [d for d in discs if d[0][1] < c[1]]
        pivots[g] = list(below[-1][0]) if below else list(c)
for c, k, n in discs:                                   # disc moves with the vertebra above it
    above = [j for j, v in enumerate(verts) if v[0][1] > c[1]]
    vi = above[0] if above else len(verts) - 1
    add((names[vi] if vi < len(names) else "T1") + ":disc", *sub(p, t, r == k))

# ribs: 12 pairs + sternum/costal cartilage (largest component)
p, t = read(G + "ribs.vtp"); p = p + RIBS + BACK
r = comps(p, t)
parts = sorted([(p[r == k].mean(0), k, (r == k).sum()) for k in np.unique(r)], key=lambda x: -x[2])
sternum, ribs = parts[0], sorted(parts[1:], key=lambda x: x[0][1])
add("T4:ribs", *sub(p, t, r == sternum[1]))
for i, (c, k, n) in enumerate(ribs):                    # bottom pair first -> T12
    add(f"T{12 - i // 2}:ribs", *sub(p, t, r == k))

# head
for m in ("hat_skull", "hat_jaw"):
    try:
        p, t = read(G + m + ".vtp"); add("T1:head", p + BACK, t)
    except FileNotFoundError:
        pass

order = ["pelvis"] + names
out = {"source": "OpenSim 4.5 Gait2392 geometry (sacrum, r/l_pelvis, spine, ribs, hat_skull, hat_jaw)",
       "frame": "OpenSim pelvis frame: X anterior, Y up, Z right; cm",
       "order": order, "pivots": {k: [round(float(v) * 100, 2) for v in pivots[k]] for k in order}, "meshes": []}
for g, lst in groups.items():
    seg, kind = (g.split(":") + ["bone"])[:2]
    P = []; T = []; off = 0
    for pts, tri in lst:
        P.append(pts); T.append(tri + off); off += len(pts)
    P = np.vstack(P) * 100; T = np.vstack(T)
    out["meshes"].append({"segment": seg, "kind": kind, "name": g,
                          "pos": np.round(P, 2).ravel().tolist(), "idx": T.ravel().astype(int).tolist()})
dst = r"C:\Users\dghsu\Desktop\imu_daq\host\static\models\spine_opensim.json"
import os; os.makedirs(os.path.dirname(dst), exist_ok=True)
json.dump(out, open(dst, "w"), separators=(",", ":"))
print("groups:", sorted(groups)); print("pivots:", out["pivots"]); print(os.path.getsize(dst) // 1024, "KB")
