"""
Parametric 6-position calibration jig for the Adafruit BNO085 breakout (product 4754).

MASTER REFERENCE: ../dimension/adafruit_bno085/4754_BNO085_STEMMA_QT.step
(Adafruit's official model, github.com/adafruit/Adafruit_CAD_Parts). The script
reads it at run time and measures the board from it:
      PCB 25.40 x 22.86 x 1.57 mm, corner radius 2.54 mm, overall height 4.53 mm
      4 mounting holes dia 2.50 at (2.54, 2.54) (22.86, 2.54) (2.54, 20.32) (22.86, 20.32)
      -> hole spacing 20.32 mm (X) x 17.78 mm (Y)
      2 header rows of 6 x dia 1.0 holes at y = 2.54 / 20.32
      STEMMA QT connectors on the two short-X edges; nothing below the PCB
The Eagle-derived pcb_geometry.json in the same folder is used only as a
cross-check, printed every run. The real boards measure 25.6 x 22.7 mm
(MEASURED_X / MEASURED_Y below), which sets the recess size only.
Board coordinates: origin at the board corner nearest header JP1 / the -X STEMMA
connector, X along the 25.4 mm side, Y along the 22.86 mm side.

The jig is a chamfered rectangular block. The top face has one recess per board,
all boards in the same orientation. Each board rests on 4 pillars printed as part
of the jig, one under each mounting hole. In "screw" mode an M2.5 thread-forming
screw goes through the board into each pillar; in "snap" mode each pillar ends in
a split pin with a barb that clips through the hole. The other
five faces are flat reference faces, so the block can rest on any of its 6 faces
for the 6-position test. Die-style pips (1-6 dots) identify the current pose:

    +Z = 1 pip   -Z = 6 pips
    +X = 2 pips  -X = 5 pips
    +Y = 3 pips  -Y = 4 pips      (opposite faces add up to 7, like a die)

The jig frame (origin at the -X,-Y,-Z corner) is shown by the arrows engraved on
the top face. Board X/Y are parallel to jig X/Y. Pockets are numbered row by
row from the arrow corner: S1 at lowest X, lowest Y; then along +X; then +Y.

Datum: the 4 pillar tops and the -X / -Y recess walls. Push each board into the
-X/-Y corner of its recess before tightening the screws.

Usage:
    python imu_jig.py          -> writes imu_jig.stl, coupons and preview PNGs
    python imu_jig.py PLA-CF   -> writes imu_jig_PLA-CF.stl and imu_jig_test_coupon_peg_PLA-CF.stl
    python imu_jig.py BNO086 PLA-CF
                               -> SparkFun BNO086 version: imu_jig_BNO086_PLA-CF.stl and
                                  imu_jig_test_coupon_peg_BNO086_PLA-CF.stl (see the
                                  SparkFun section below for its source and assumptions)
Requires: pip install cadquery manifold3d numpy matplotlib
"""

from pathlib import Path
import math
import struct
import sys

import numpy as np
from manifold3d import Manifold, CrossSection

HERE = Path(__file__).resolve().parent

# Board selection: `python imu_jig.py [BNO085|BNO086] [PLA|PLA-CF]` (any order).
BOARDS = ("BNO085", "BNO086")
_ARGS = [a.upper() for a in sys.argv[1:]] if __name__ == "__main__" else []
BOARD_NAME = next((a for a in _ARGS if a in BOARDS), "BNO085")

# ===================== BOARD (read from ../dimension) =======================
# MASTER reference: Adafruit's STEP model, dimension/adafruit_bno085/
# 4754_BNO085_STEMMA_QT.step. Everything below is measured from it at run time:
#   PCB solid      -> outline, corner radius, thickness
#   PCB holes      -> mounting holes (dia 2-4 mm) and header holes (dia ~1 mm)
#   component solids -> STEMMA QT connectors (the two at the X edges),
#                       BNO085 package (the 5.2 x 3.8 mm part), underside clearance
# pcb_geometry.json (from the Eagle .brd) is only used as a cross-check.
DIM_DIR = HERE.parent / "dimension" / "adafruit_bno085"
STEP_FILE = DIM_DIR / "4754_BNO085_STEMMA_QT.step"


def _load_step_board(path):
    import cadquery as cq
    shape = cq.importers.importStep(str(path)).val()
    solids = shape.Solids()
    pcb = max(solids, key=lambda s: s.BoundingBox().xlen * s.BoundingBox().ylen)
    pb = pcb.BoundingBox()
    x0, y0, z0 = pb.xmin, pb.ymin, pb.zmin

    # corner radius from the arcs of the top-face outline
    top = max((f for f in pcb.Faces() if f.geomType() == "PLANE"), key=lambda f: f.Center().z)
    arcs = [e.radius() for e in top.outerWire().Edges() if e.geomType() == "CIRCLE"]

    # through-holes: cylindrical faces of the PCB solid with a vertical axis
    holes = {}
    for f in pcb.Faces():
        if f.geomType() != "CYLINDER":
            continue
        cyl = f._geomAdaptor().Cylinder()
        if abs(cyl.Axis().Direction().Z()) < 0.99:
            continue
        b, r = f.BoundingBox(), cyl.Radius()
        if (b.xlen < 1.5 * r and b.ylen < 1.5 * r) or b.zlen < pb.zlen - 0.05:
            continue        # skip quarter-circle outline corners and blind features
        loc = cyl.Location()
        key = (round(loc.X() - x0, 3), round(loc.Y() - y0, 3))
        holes[key] = round(2 * cyl.Radius(), 3)
    mount = sorted(k for k, d in holes.items() if 2.0 <= d <= 4.0)
    pins = sorted(k for k, d in holes.items() if 0.8 <= d <= 1.3)
    rows = sorted({y for _, y in pins})
    pin_x = [x for x, _ in pins]

    # components
    others = [s for s in solids if s is not pcb]
    under = min((s.BoundingBox().zmin for s in others), default=z0) - z0
    edge = [s.BoundingBox() for s in others
            if s.BoundingBox().xmin - x0 < 0.5 or pb.xmax - s.BoundingBox().xmax < 0.5]
    stemma_y = sum((b.ymin + b.ymax) / 2 for b in edge) / len(edge) - y0
    chip = min(others, key=lambda s: abs(s.BoundingBox().xlen - 5.2) + abs(s.BoundingBox().ylen - 3.8))
    cb = chip.BoundingBox()

    verts, tris = shape.tessellate(0.02, 0.3)
    v = np.array([p.toTuple() for p in verts]) - [x0, y0, z0]
    mesh = v[np.array(tris)]

    return dict(
        X=pb.xlen, Y=pb.ylen, T=pb.zlen, R=max(arcs) if arcs else 0.0, H=shape.BoundingBox().zmax - z0,
        holes=mount, drill=max(holes[k] for k in mount),
        header_rows_y=rows, header_x=(min(pin_x), max(pin_x)),
        stemma_y=stemma_y, stemma_len=max(b.xlen for b in edge),
        chip=((cb.xmin + cb.xmax) / 2 - x0, (cb.ymin + cb.ymax) / 2 - y0),
        chip_size=(cb.xlen, cb.ylen, cb.zlen),
        underside=under, mesh=mesh,
    )


def _load_brd_json(path):
    """Eagle-derived geometry, used only to cross-check the STEP."""
    import json
    g = json.loads(path.read_text(encoding="utf-8"))
    segs = [{k: float(s[k]) for k in ("x1", "y1", "x2", "y2")} | {"curve": s.get("curve")}
            for s in g["outline"]]
    xs = [s[k] for s in segs for k in ("x1", "x2")]
    ys = [s[k] for s in segs for k in ("y1", "y2")]
    x0, y0 = min(xs), min(ys)
    arcs = [abs(s["x2"] - s["x1"]) for s in segs if s["curve"]]
    holes, drills = [], set()
    for f in g["mounting_features"]:
        e = f["element"]
        for p in f["local_pads"] + f["local_holes"]:
            holes.append((float(e["x"]) + float(p["x"]) - x0, float(e["y"]) + float(p["y"]) - y0))
            drills.add(float(p["drill"]))
    comps = g["component_origins"]
    hdr = [c for c in comps if c["package"].startswith("1X06")]
    jst = [c for c in comps if c["package"].startswith("JST_SH")]
    chip = [c for c in comps if c["package"].startswith("BNO")][0]
    hx = float(hdr[0]["x"]) - x0
    return dict(
        X=max(xs) - x0, Y=max(ys) - y0, R=max(arcs) if arcs else 0.0,
        holes=sorted(holes), drill=max(drills),
        header_rows_y=sorted(float(c["y"]) - y0 for c in hdr),
        header_x=(hx - 2.5 * 2.54, hx + 2.5 * 2.54),    # 6 pins, package origin at centre
        stemma_y=float(jst[0]["y"]) - y0,
        chip=(float(chip["x"]) - x0, float(chip["y"]) - y0),
    )


def cross_check(step, brd, tol=0.02):
    """Compare STEP-derived numbers with the Eagle .brd data. Returns a list of lines."""
    rows = []

    def cmp(name, a, b, t=tol):
        ok = abs(a - b) <= t
        rows.append(f"  {'OK ' if ok else 'DIFF'} {name:<22} STEP {a:8.3f}   BRD {b:8.3f}")

    cmp("outline X", step["X"], brd["X"]); cmp("outline Y", step["Y"], brd["Y"])
    cmp("corner radius", step["R"], brd["R"]); cmp("hole drill", step["drill"], brd["drill"])
    for (sx, sy), (bx, by) in zip(step["holes"], brd["holes"]):
        cmp(f"hole x @({bx:.2f},{by:.2f})", sx, bx); cmp(f"hole y @({bx:.2f},{by:.2f})", sy, by)
    for i, (a, b) in enumerate(zip(step["header_rows_y"], brd["header_rows_y"])):
        cmp(f"header row {i + 1} y", a, b)
    cmp("header first pin x", step["header_x"][0], brd["header_x"][0])
    cmp("header last pin x", step["header_x"][1], brd["header_x"][1])
    cmp("STEMMA QT centre y", step["stemma_y"], brd["stemma_y"], 0.1)
    cmp("chip centre x", step["chip"][0], brd["chip"][0], 0.1)
    cmp("chip centre y", step["chip"][1], brd["chip"][1], 0.1)
    return rows


# ---------------- SparkFun BNO086 (SEN-22857, Qwiic) ----------------------------
# SparkFun publishes no 3D model for this board, so the MASTER reference is their
# Eagle layout: dimension/sparkfun_bno086/pcb_geometry.json (extracted from
# SparkFun_VR_IMU_Breakout_BNO086_QWIIC.brd), checked against dimensions_v10.png:
#   outline 30.48 x 25.40 mm (1.20 x 1.00 in), square corners
#   2 stand-off holes dia 3.302 at (2.54, 22.86) and (27.94, 22.86), 1.00 in apart
#   header pins (dia ~1.0): rows at y = 1.27, 19.05, 24.13, plus one pin at (1.27, 7.62)
#   Qwiic (JST-SH 4-pin RA) connectors at both X edges, centre y = 12.70
#   BNO086 (LGA-28) centre (15.24, 12.70)
# Not in the files (README: "not verified"), so ASSUMED here:
#   PCB thickness 1.6 mm (SparkFun standard), tallest part = Qwiic connector ~2.9 mm.
SF_DIR = HERE.parent / "dimension" / "sparkfun_bno086"
SF_PCB_T = 1.6          # assumed
SF_PART_H = 2.9         # assumed height of the Qwiic connector above the PCB


def _rot(dx, dy, rot):
    """Eagle rotation string (e.g. 'R180', 'MR90') applied to a package-local offset."""
    import math
    rot = rot or "R0"
    mirror = rot.startswith("M")
    a = math.radians(float(rot.lstrip("MSR") or 0))
    if mirror:
        dx = -dx
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def _load_sparkfun_board(path):
    import json
    g = json.loads(path.read_text(encoding="utf-8"))
    xs = [float(s[k]) for s in g["outline"] for k in ("x1", "x2")]
    ys = [float(s[k]) for s in g["outline"] for k in ("y1", "y2")]
    x0, y0 = min(xs), min(ys)
    holes, drills = [], []
    for f in g["mounting_features"]:
        e = f["element"]
        for p in f["local_pads"] + f["local_holes"]:
            holes.append((float(e["x"]) + float(p["x"]) - x0, float(e["y"]) + float(p["y"]) - y0))
            drills.append(float(p["drill"]))
    comps = g["component_origins"]
    pins = []
    for c in comps:
        pk = c["package"]
        if pk.startswith("1X0") and not c.get("rot", "").startswith("M"):
            n = int(pk[2:4])        # 1X08_NO_SILK -> 8 pins, pad k at local (k * 2.54, 0)
            for k in range(n):
                dx, dy = _rot(k * 2.54, 0.0, c.get("rot"))
                pins.append((round(float(c["x"]) + dx - x0, 3), round(float(c["y"]) + dy - y0, 3)))
    rows = {}
    for x, y in pins:
        rows.setdefault(y, []).append(x)
    jst = [c for c in comps if c["package"].startswith("JST04")]
    chip = [c for c in comps if c.get("value", "").startswith("BNO") and c["package"].startswith("LGA")][0]
    X, Y = max(xs) - x0, max(ys) - y0
    return dict(
        X=X, Y=Y, T=SF_PCB_T, R=0.0, H=SF_PCB_T + SF_PART_H,
        holes=sorted(holes), drill=max(drills),
        header_rows=[(y, min(v), max(v)) for y, v in sorted(rows.items())],
        pins=sorted(pins),
        stemma_y=sum(float(c["y"]) for c in jst) / len(jst) - y0,
        chip=(float(chip["x"]) - x0, float(chip["y"]) - y0),
        chip_size=(3.8, 5.2, 1.1),            # BNO086 LGA-28 5.2 x 3.8 mm, placed R90
        underside=0.0, mesh=None,
    )


def _placeholder_mesh(b):
    """Simple stand-in 3D board (PCB + connectors + chip) for the previews."""
    T = b["T"]
    parts = [Manifold.cube([b["X"], b["Y"], T])]
    for x in (0.48, b["X"] - 7.08):                       # Qwiic bodies at both X edges
        parts.append(Manifold.cube([6.6, 6.0, SF_PART_H]).translate([x, b["stemma_y"] - 3, T]))
    cx, cy = b["chip"]
    w, h, t = b["chip_size"]
    parts.append(Manifold.cube([w, h, t]).translate([cx - w / 2, cy - h / 2, T]))
    m = parts[0]
    for p in parts[1:]:
        m = m + p
    for hx, hy in b["holes"]:
        m = m - Manifold.cylinder(T + 1, b["drill"] / 2, b["drill"] / 2, 32).translate([hx, hy, -0.5])
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    return v[np.asarray(mesh.tri_verts)]


if BOARD_NAME == "BNO085":
    B = _load_step_board(STEP_FILE)                  # master reference: Adafruit STEP
    HEADER_ROWS = [(y, *B["header_x"]) for y in B["header_rows_y"]]   # 2 rows, 6.35..19.05
    SUPPORTS = []                                    # 4 mounting holes: no extra rests needed
    PILLAR_D_BOARD = 5.0                             # keep-out ring around each hole is 4 mm
    CONNECTOR = "STEMMA QT"
else:
    B = _load_sparkfun_board(SF_DIR / "pcb_geometry.json")   # master reference: SparkFun .brd
    B["mesh"] = _placeholder_mesh(B)
    HEADER_ROWS = B["header_rows"]
    # Only 2 holes, both on the +Y edge. Two plain rests (no peg) mirrored to the -Y
    # edge give the board 4 support points so it sits flat and cannot tip.
    SUPPORTS = [(hx, B["Y"] - hy) for hx, hy in B["holes"]]  # (2.54, 2.54), (27.94, 2.54)
    PILLAR_D_BOARD = 5.5
    CONNECTOR = "Qwiic"

BOARD_X, BOARD_Y, BOARD_R = B["X"], B["Y"], B["R"]
PCB_T = B["T"]
HOLES = B["holes"]
HOLE_DRILL = B["drill"]
STEMMA_Y = B["stemma_y"]                             # cable connector centre y
CHIP_XY = B["chip"]                                  # IMU package centre
assert B["underside"] >= -1e-6, "parts below the PCB; pillars/floor need rework"

# ===================== JIG PARAMETERS (edit these) ==========================
N_COLS = 3          # boards along X  (3 x 3 keeps the block close to a cube,
N_ROWS = 3          # boards along Y   so it stands steadily on every face)

CLR = 0.15          # recess clearance per side (tune with the test coupon)
RECESS = 1.0        # wall height above the pillar tops; PCB is 1.6 so it sits 0.6 mm proud

# Printed mounting pillars (standoffs), one under each board mounting hole.
# The 4 pillar tops are the datum plane the board rests on.
PILLAR_MODE = "peg"    # "peg"   = solid pin, same diameter as the board hole, friction fit
                       #           (no screws, no flexing parts; press the board on)
                       # "snap"  = split pin with a barb (proved too fragile in testing)
                       # "screw" = pilot hole for an M2.5 thread-forming screw
PEG_D = HOLE_DRILL  # peg mode: pin diameter = board hole (2.50 BNO085, 3.302 BNO086)
PEG_ABOVE = 1.0     # peg mode: how far the pin sticks out above the board top
PEG_CHAMFER = 0.4   # peg mode: lead-in chamfer at the tip
PILLAR_D = PILLAR_D_BOARD  # pillar diameter: 5.0 (BNO085), 5.5 (BNO086)
SUPPORT_D = 4.0     # plain rest posts (boards with fewer than 4 holes)
PILLAR_H = 3.0      # pillar height above the recess floor (space under the board)
PILOT_D = 2.1       # screw mode: pilot hole for M2.5 thread-forming screw
PILOT_DEPTH = 8.0   # screw mode: depth below the pillar top (use M2.5 x 6 screws)
PIN_D = HOLE_DRILL - 0.2   # snap mode: pin diameter (CAD drill 2.5 -> 2.3 mm)
BARB_D = HOLE_DRILL + 0.4  # snap mode: barb diameter that holds the board top
BARB_H = 1.2        # snap mode: barb cone height
SPLIT_W = 0.7       # snap mode: slot that lets the two pin halves flex
SPLIT_DEPTH = 2.0   # snap mode: how far the slot runs down into the pillar

HEADER_SLOTS = True # slots under the header rows, for boards with soldered pins
SLOT_W = 3.2
SLOT_DEPTH = 5.0    # below the recess floor (adds to PILLAR_H; long pins are ~6 mm)

CABLE_W = 8.0       # STEMMA QT plug relief at both X edges
LIFT_R = 3.5        # fingernail lift notch at the +Y edge

GAP_X = 16.0        # space between boards along X (room for STEMMA QT plugs/cables)
GAP_Y = 10.0        # space between boards along Y
MARGIN = 14.0       # outer border (holds the pips and axis arrows)
BODY_Z = 52.0       # block height; + FOOT_H = 60 mm overall (steadier on its sides)
CHAMFER = 1.0

# Corner feet on the top face. The boards, pins and STEMMA QT connectors stick up
# above the top surface, so the jig cannot lie on its top face directly. The 4 foot
# tops form the +Z resting plane instead, and the open sides between the feet let
# the cables run out while the jig is upside down.
FOOT = 12.0         # foot size (square), sits inside the MARGIN corners
FOOT_H = 8.0        # foot height above the top surface

PIP_D = 4.0
PIP_DEPTH = 0.8
SEG = 48
# ===========================================================================

# Measured board size. The real boards are 25.6 x 22.7 mm (matches Adafruit's
# product listing), not the 25.40 x 22.86 CAD outline. The recess is sized to the
# measured board; the CAD hole/connector pattern is kept centred in it.
# Set to None to fall back to the CAD outline.
MEASURED_X = 25.6 if BOARD_NAME == "BNO085" else None   # BNO086: not measured yet,
MEASURED_Y = 22.7 if BOARD_NAME == "BNO085" else None   # uses the CAD outline

FIT_X = MEASURED_X or BOARD_X
FIT_Y = MEASURED_Y or BOARD_Y
OFF_X = (FIT_X - BOARD_X) / 2      # CAD origin offset inside the real board edge
OFF_Y = (FIT_Y - BOARD_Y) / 2
PX = FIT_X + 2 * CLR
PY = FIT_Y + 2 * CLR
ORG = (CLR + OFF_X, CLR + OFF_Y)    # CAD board origin relative to the recess corner


def box(x0, y0, z0, x1, y1, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def zcyl(x, y, z0, z1, r):
    return Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def chamfered_box(sx, sy, sz, c):
    parts = [
        Manifold.cube([sx, sy - 2 * c, sz - 2 * c], center=True),
        Manifold.cube([sx - 2 * c, sy, sz - 2 * c], center=True),
        Manifold.cube([sx - 2 * c, sy - 2 * c, sz], center=True),
    ]
    return Manifold.batch_hull(parts).translate([sx / 2, sy / 2, sz / 2])


def rounded_rect(x0, y0, sx, sy, r, z0, z1):
    r = max(r, 0.01)
    cs = [zcyl(x0 + r, y0 + r, z0, z1, r), zcyl(x0 + sx - r, y0 + r, z0, z1, r),
          zcyl(x0 + r, y0 + sy - r, z0, z1, r), zcyl(x0 + sx - r, y0 + sy - r, z0, z1, r)]
    return Manifold.batch_hull(cs)


def union(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def pocket_positions():
    """Yields (index, x0, y0) of each recess corner (recess includes clearance)."""
    for r in range(N_ROWS):
        for c in range(N_COLS):
            yield (r * N_COLS + c + 1,
                   MARGIN + c * (PX + GAP_X),
                   MARGIN + r * (PY + GAP_Y))


def pillar(x, y, floor, mode):
    """Printed standoff under one mounting hole. Returns (add, cut)."""
    seat = floor + PILLAR_H                      # board bottom rests here
    add = zcyl(x, y, floor - 0.5, seat, PILLAR_D / 2)
    cut = None
    if mode == "screw":
        cut = zcyl(x, y, seat - PILOT_DEPTH, seat + 1, PILOT_D / 2)
    elif mode == "peg":
        tip = seat + PCB_T + PEG_ABOVE
        add += zcyl(x, y, seat - 0.1, tip - PEG_CHAMFER, PEG_D / 2)
        add += Manifold.cylinder(PEG_CHAMFER, PEG_D / 2, PEG_D / 2 - PEG_CHAMFER, SEG).translate(
            [x, y, tip - PEG_CHAMFER])
    else:
        top_pin = seat + PCB_T + 0.1
        add += zcyl(x, y, seat - 0.1, top_pin, PIN_D / 2)
        add += Manifold.cylinder(BARB_H, BARB_D / 2, PIN_D / 2 - 0.35, SEG).translate([x, y, top_pin])
        # split along X so the halves flex toward each other
        cut = box(x - PIN_D, y - SPLIT_W / 2, seat - SPLIT_DEPTH,
                  x + PIN_D, y + SPLIT_W / 2, top_pin + BARB_H + 1)
    return add, cut


def pocket(x0, y0, top, mode=None):
    """One board position. (x0, y0) = recess corner. Returns (cut, add, cut_after)."""
    mode = mode or PILLAR_MODE
    floor = top - RECESS - PILLAR_H
    bx, by = x0 + ORG[0], y0 + ORG[1]    # CAD board origin in jig coordinates
    cuts = [rounded_rect(x0, y0, PX, PY, BOARD_R + CLR, floor, top + 1)]
    adds, cut_after = [], []
    for hx, hy in HOLES:
        a, c = pillar(bx + hx, by + hy, floor, mode)
        adds.append(a)
        if c is not None:
            cut_after.append(c)

    seat = floor + PILLAR_H
    for sx, sy in SUPPORTS:                       # plain rests, same height as the pillars
        adds.append(zcyl(bx + sx, by + sy, floor - 0.5, seat, SUPPORT_D / 2))

    if HEADER_SLOTS:
        # cut after the pillars are added: a slot can end inside a corner pillar,
        # and cutting first would leave a sliver of pillar hanging over the slot
        for hy, hx0, hx1 in HEADER_ROWS:
            cut_after.append(box(bx + hx0 - 1.6, by + hy - SLOT_W / 2, floor - SLOT_DEPTH,
                                 bx + hx1 + 1.6, by + hy + SLOT_W / 2, top + 1))

    # STEMMA QT plug relief on both X edges
    for ex in (x0, x0 + PX):
        cuts.append(box(ex - 3.0, by + STEMMA_Y - CABLE_W / 2, floor,
                        ex + 3.0, by + STEMMA_Y + CABLE_W / 2, top + 1))

    # fingernail lift notch at +Y edge (keeps the -X / -Y datum walls intact)
    cuts.append(zcyl(x0 + PX / 2, y0 + PY, top - RECESS - 1.5, top + 1, LIFT_R))
    return union(cuts), union(adds), cut_after


def arrow(length, width=2.0, head=5.0):
    pts = [(0, -width / 2), (length - head, -width / 2), (length - head, -head / 2),
           (length, 0), (length - head, head / 2), (length - head, width / 2), (0, width / 2)]
    return CrossSection([pts])


def pip_layout(n, s):
    return {
        1: [(0, 0)],
        2: [(-s, -s), (s, s)],
        3: [(-s, -s), (0, 0), (s, s)],
        4: [(-s, -s), (s, -s), (-s, s), (s, s)],
        5: [(-s, -s), (s, -s), (0, 0), (-s, s), (s, s)],
        6: [(-s, -s), (s, -s), (-s, 0), (s, 0), (-s, s), (s, s)],
    }[n]


def face_marks(BX, BY, BZ):
    r = PIP_D / 2
    cyl = Manifold.cylinder(2 * PIP_DEPTH, r, r, SEG, center=True)
    s = min(8.0, BZ / 4)
    out = [cyl.translate([BX - FOOT / 2, BY - FOOT / 2, BZ + FOOT_H])]     # +Z: 1, on a foot
    out += [cyl.translate([BX / 2 + u, BY / 2 + v, 0]) for u, v in pip_layout(6, s)]
    cx = cyl.rotate([0, 90, 0])
    for n, x in ((2, BX), (5, 0)):
        out += [cx.translate([x, BY / 2 + u, BZ / 2 + v]) for u, v in pip_layout(n, s)]
    cy = cyl.rotate([90, 0, 0])
    for n, y in ((3, BY), (4, 0)):
        out += [cy.translate([BX / 2 + u, y, BZ / 2 + v]) for u, v in pip_layout(n, s)]
    # jig-frame arrows (+X, +Y) in the -X,-Y top corner
    a = Manifold.extrude(arrow(MARGIN * 1.6), 2 * PIP_DEPTH).translate([0, 0, -PIP_DEPTH])
    # +X arrow in the -Y margin, +Y arrow in the -X margin, both just past the corner foot
    out += [a.translate([FOOT + 3, MARGIN / 2, BZ]),
            a.rotate([0, 0, 90]).translate([MARGIN / 2, FOOT + 3, BZ])]
    return out


def top_clearance_needed():
    """Highest point of a mounted board above the jig top surface (mm)."""
    pin_tip = {"snap": PCB_T + 0.1 + BARB_H, "peg": PCB_T + PEG_ABOVE}.get(PILLAR_MODE, PCB_T)
    return max(B["H"], pin_tip) - RECESS


def stability_report(BX, BY, BZ):
    """Tip-over angle for each resting face: how far it can tilt before falling."""
    import math
    H = BZ + FOOT_H
    rows = []
    for face, base, height in (("+/-Z (top/bottom)", min(BX, BY), H),
                               ("+/-Y", min(BX, H), BY),
                               ("+/-X", min(BY, H), BX)):
        ang = math.degrees(math.atan((base / 2) / (height / 2)))
        rows.append(f"  resting on {face:<18} narrowest base {base:6.1f} mm, "
                    f"height {height:6.1f} mm -> tips over past {ang:4.1f} deg")
    return rows


def jig_size():
    BX = 2 * MARGIN + N_COLS * PX + (N_COLS - 1) * GAP_X
    BY = 2 * MARGIN + N_ROWS * PY + (N_ROWS - 1) * GAP_Y
    return BX, BY, BODY_Z


def build_jig():
    BX, BY, BZ = jig_size()
    assert RECESS + PILLAR_H + max(PILOT_DEPTH, SLOT_DEPTH if HEADER_SLOTS else 0) + 5 < BZ, \
        "BODY_Z too small"
    body = chamfered_box(BX, BY, BZ, CHAMFER)
    cuts, adds, after = [], [], []
    for _, x0, y0 in pocket_positions():
        c, a, ca = pocket(x0, y0, BZ)
        cuts.append(c); adds.append(a); after += ca
    cuts += face_marks(BX, BY, BZ)
    assert FOOT <= MARGIN, "FOOT must fit inside the MARGIN corner"
    assert FOOT_H >= top_clearance_needed() + 2, "FOOT_H too low for the parts above the top face"
    feet = [chamfered_box(FOOT, FOOT, FOOT_H + 1, CHAMFER).translate([fx, fy, BZ - 1])
            for fx in (0, BX - FOOT) for fy in (0, BY - FOOT)]
    body = body + union(feet)
    jig = body - union(cuts) + union(adds)
    return jig - union(after) if after else jig


def coupon_height():
    return RECESS + PILLAR_H + max(PILOT_DEPTH, SLOT_DEPTH if HEADER_SLOTS else 0) + 3


def build_coupon(mode):
    """One board position in a small block: print first to check the fit and the pillars."""
    m = 8.0
    BX, BY, BZ = PX + 2 * m, PY + 2 * m, coupon_height()
    body = chamfered_box(BX, BY, BZ, 0.6)
    c, a, ca = pocket(m, m, BZ, mode)
    out = body - c + a
    return (out - union(ca) if ca else out), (BX, BY, BZ)


# ----------------------------- output --------------------------------------
def mesh_arrays(m):
    mesh = m.to_mesh()
    return np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts)


def write_stl(m, path):
    v, t = mesh_arrays(m)
    with open(path, "wb") as f:
        f.write(b"BNO085 calibration jig".ljust(80, b" "))
        f.write(struct.pack("<I", len(t)))
        for a, b, c in t:
            p0, p1, p2 = v[a], v[b], v[c]
            n = np.cross(p1 - p0, p2 - p0)
            ln = np.linalg.norm(n)
            f.write(struct.pack("<12fH", *(n / ln if ln else n), *p0, *p1, *p2, 0))
    return len(t)


def render(groups, path, elev, azim, title):
    """groups: list of (triangles Nx3x3, rgb)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(9, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    light = np.array([0.4, -0.5, 0.8]) / np.linalg.norm([0.4, -0.5, 0.8])
    all_t, all_c = [], []
    for tris, rgb in groups:
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        s = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
        all_t.append(tris)
        all_c.append(np.c_[s * rgb[0], s * rgb[1], s * rgb[2], np.ones_like(s)])
    # one collection so faces from all groups are depth-sorted together
    tris_all = np.vstack(all_t)
    ax.add_collection3d(Poly3DCollection(tris_all, facecolors=np.vstack(all_c), edgecolors="none"))
    v = tris_all.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo)
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X (mm)"); ax.set_ylabel("Y (mm)"); ax.set_zlabel("Z (mm)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# Material profiles: `python imu_jig.py PLA-CF` writes imu_jig_PLA-CF.stl etc.
# The geometry is the same for PLA and PLA-CF; a profile can override the
# fit parameters below (e.g. "PEG_D": 2.45) if test prints call for it.
PROFILES = {
    "PLA": {},
    "PLA-CF": {},
}

if __name__ == "__main__":
    MATERIAL = next((a for a in _ARGS if a not in BOARDS), "PLA")
    if MATERIAL not in PROFILES:
        sys.exit(f"Unknown material {MATERIAL!r}; choose from {', '.join(PROFILES)}")
    globals().update(PROFILES[MATERIAL])
    # BNO085 keeps the original file names; other boards are tagged with their name
    TAG = "" if BOARD_NAME == "BNO085" else f"_{BOARD_NAME}"
    SUFFIX = TAG + ("" if MATERIAL == "PLA" else f"_{MATERIAL}")
    print(f"Board: {BOARD_NAME}   Material profile: {MATERIAL}")
    jig = build_jig()
    BX, BY, BZ = jig_size()
    n1 = write_stl(jig, HERE / f"imu_jig{SUFFIX}.stl")
    if BOARD_NAME == "BNO085":
        print(f"Master reference: {STEP_FILE.name}")
    else:
        print(f"Master reference: {SF_DIR.name}/pcb_geometry.json (SparkFun Eagle .brd)")
        print(f"  ASSUMED (not in the files): PCB {PCB_T} mm, tallest part {SF_PART_H} mm")
    print(f"  PCB {BOARD_X:.3f} x {BOARD_Y:.3f} x {PCB_T:.3f} mm, R{BOARD_R:.2f}, overall height "
          f"{B['H']:.2f}; lowest part {B['underside']:+.3f} mm vs PCB bottom (>= 0: underside clear)")
    print(f"  holes: {', '.join(f'({x:.2f}, {y:.2f})' for x, y in HOLES)}  dia {HOLE_DRILL:.3f}"
          f" -> pegs {PEG_D:.3f}; rests: {', '.join(f'({x:.2f}, {y:.2f})' for x, y in SUPPORTS) or 'none'}")
    print(f"  {BOARD_NAME} package centre ({CHIP_XY[0]:.3f}, {CHIP_XY[1]:.3f}); "
          f"{CONNECTOR} connector centre y {STEMMA_Y:.2f}")
    if BOARD_NAME == "BNO085":
        print("Cross-check against the Eagle .brd (pcb_geometry.json):")
        print("\n".join(cross_check(B, _load_brd_json(DIM_DIR / "pcb_geometry.json"))))
    if "pins" in B:      # clearance between header pins and the pillar/rest tops (board underside)
        worst = min((math.dist(p, q) - r, p, q) for q in B["pins"]
                    for p, r in [(h, PILLAR_D / 2) for h in HOLES] + [(s, SUPPORT_D / 2) for s in SUPPORTS])
        print(f"  closest header pin to a pillar/rest edge: {worst[0]:.2f} mm "
              f"(pin pad radius ~0.9; slots trim any overlap)")
    print(f"Jig:    {BX:.1f} x {BY:.1f} x {BZ:.1f} mm, {N_COLS * N_ROWS} boards, "
          f"pillars: {PILLAR_MODE}, volume {jig.volume() / 1000:.0f} cm^3, "
          f"{n1} triangles, genus {jig.genus()}")
    print(f"  overall incl. feet: {BX:.1f} x {BY:.1f} x {BZ + FOOT_H:.1f} mm; parts reach "
          f"{top_clearance_needed():.2f} mm above the top surface, feet {FOOT_H:.1f} mm")
    print("\n".join(stability_report(BX, BY, BZ)))
    coupons = {}
    for mode in ("peg", "screw", "snap"):
        if mode != PILLAR_MODE and SUFFIX:
            continue        # tagged builds only ship the coupon for the pillar type in use
        cm, csz = build_coupon(mode)
        coupons[mode] = (cm, csz)
        n = write_stl(cm, HERE / f"imu_jig_test_coupon_{mode}{SUFFIX}.stl")
        print(f"Coupon ({mode}): {csz[0]:.1f} x {csz[1]:.1f} x {csz[2]:.1f} mm, {n} triangles")
    coupon, csz = coupons[PILLAR_MODE]
    for i, x0, y0 in pocket_positions():
        print(f"  S{i}: {BOARD_NAME} chip centre at X={x0 + ORG[0] + CHIP_XY[0]:.2f}  "
              f"Y={y0 + ORG[1] + CHIP_XY[1]:.2f}  (jig frame, mm)")

    v, t = mesh_arrays(jig)
    grey = (0.62, 0.66, 0.72)
    groups = [(v[t], grey)]
    board = B["mesh"]                    # STEP tessellation (BNO085) or stand-in (BNO086)
    if board is not None:
        for _, x0, y0 in pocket_positions():
            groups.append((board + [x0 + ORG[0], y0 + ORG[1], BZ - RECESS], (0.15, 0.35, 0.75)))
    render(groups, HERE / f"preview_iso{TAG}.png", 35, -60,
           f"{BOARD_NAME} jig with boards (top +Z=1, -Y=4, +X=2)")
    render([(v[t], grey)], HERE / f"preview_bottom{TAG}.png", -30, 125,
           f"{BOARD_NAME} jig - bottom (-Z=6), +Y=3, -X=5")
    cv, ct = mesh_arrays(coupon)
    cg = [(cv[ct], grey)]
    if board is not None:
        cg.append((board + [8 + ORG[0], 8 + ORG[1], csz[2] - RECESS], (0.15, 0.35, 0.75)))
    render(cg, HERE / f"preview_coupon{TAG}.png", 40, -60, f"{BOARD_NAME} single-board test coupon")
    render([(cv[ct], grey)], HERE / f"preview_coupon_empty{TAG}.png", 55, -60,
           f"{BOARD_NAME} test coupon (empty)")
    print("Wrote STLs and previews to", HERE)
