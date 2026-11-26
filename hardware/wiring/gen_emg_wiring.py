"""Generates emg_wiring.html: pin-level EMG wiring for the senior's SID Lab map (SETUP.md, 2026-11-25).
8 MyoWare 2.0 (Link Shield TRS -> TRRS breakout), hub R and hub L, one Cat6 per side, NI USB-6363 RSE AI16-AI23.
EMG7/8 (external oblique, AI22/AI23, terminals 55/58) were added on 2026-11-26 following the senior's pattern.
Run:  python gen_emg_wiring.py
"""
import pathlib

OUT = pathlib.Path(__file__).with_name("emg_wiring.html")


def f(v):
    return f"{v:.1f}".rstrip("0").rstrip(".")


def T(x, y, s, cls="", anchor=None):
    a = f' text-anchor="{anchor}"' if anchor else ""
    return f'<text class="{cls}" x="{f(x)}" y="{f(y)}"{a}>{s}</text>'


def R(x, y, w, h, cls, rx=3):
    return f'<rect class="{cls}" x="{f(x)}" y="{f(y)}" width="{f(w)}" height="{f(h)}" rx="{rx}"/>'


def C(x, y, r, cls):
    return f'<circle class="{cls}" cx="{f(x)}" cy="{f(y)}" r="{f(r)}"/>'


def W(d, cls):
    return f'<path class="w {cls}" d="{d}"/>'


def dot(x, y, cls):
    return C(x, y, 3.6, cls)


# channel: (EMG no., site, AI, DAQ terminal, colour class, Cat6 pin)
SIDE = {
    "R": [(1, "R-LG", "AI16", 33, "e1", 1), (3, "R-MF", "AI18", 39, "e3", 3), (5, "R-IL", "AI20", 49, "e5", 4), (7, "R-EO", "AI22", 55, "e7", 5)],
    "L": [(2, "L-LG", "AI17", 36, "e2", 1), (4, "L-MF", "AI19", 42, "e4", 3), (6, "L-IL", "AI21", 52, "e6", 4), (8, "L-EO", "AI23", 58, "e8", 5)],
}
GND_TERM = {"R": 35, "L": 38}
CAT_GND = 2


def rj45_row(x0, y0, title, used):
    out = [R(x0, y0, 8 * 22 + 6, 30, "pcb", 4), T(x0 - 10, y0 + 20, title, "b", "end")]
    for k in range(8):
        c = used.get(k + 1, "")
        out.append(R(x0 + 4 + 22 * k, y0 + 6, 16, 18, f"lever {c}".strip(), 2))
        out.append(T(x0 + 12 + 22 * k, y0 + 19, str(k + 1), "pin8 inv" if c else "pin8", "middle"))
    return out, (lambda pin: (x0 + 12 + 22 * (pin - 1), y0))


def rj45_col(x0, y0, title, used):
    out = [R(x0, y0, 34, 8 * 22 + 6, "pcb", 4), T(x0 + 17, y0 - 10, title, "b", "middle")]
    for k in range(8):
        c = used.get(k + 1, "")
        out.append(R(x0 + 6, y0 + 4 + 22 * k, 22, 16, f"lever {c}".strip(), 2))
        out.append(T(x0 + 17, y0 + 16 + 22 * k, str(k + 1), "pin8 inv" if c else "pin8", "middle"))
    return out, (lambda pin: (x0 + 34, y0 + 12 + 22 * (pin - 1)))


def stack(sx, y0, n, site, ai, cls):
    """Link Shield on the sensor, TRS cable, TRRS breakout. Returns parts and pad xy for T, R, S."""
    out = [R(sx, y0, 110, 46, "pcb", 4), T(sx + 55, y0 + 19, f"EMG{n} · {site}", f"b t{cls}", "middle"),
           T(sx + 55, y0 + 35, "MyoWare 2 + Link", "pin8", "middle"),
           R(sx + 43, y0 + 46, 24, 9, "part fa", 2),
           f'<path class="trs" d="M{sx + 55} {y0 + 55} V{y0 + 92}"/>',
           R(sx + 15, y0 + 86, 80, 66, "pcb", 4), R(sx + 37, y0 + 92, 36, 20, "part", 3),
           T(sx + 55, y0 + 106, "jack", "pin8", "middle")]
    pads = {}
    for k, (lab, pc) in enumerate((("T", f"f{cls}"), ("R", "f5"), ("S", "fg"))):
        x, y = sx + 31 + 24 * k, y0 + 136
        out.append(C(x, y, 8, f"pad {pc}"))
        out.append(T(x, y + 4, lab, "pl", "middle"))
        pads[lab] = (x, y)
    return out, pads


def hub(x0, y0, side):
    out = [R(x0 - 40, y0 - 34, 580, 334, "zone", 10),
           T(x0 - 30, y0 - 14, f"Hub {side} · {'right' if side == 'R' else 'left'} side of the back · Cat6 {side}", "b")]
    bus5, busg = y0 + 180, y0 + 196
    out.append(W(f"M{x0 + 10} {bus5} H{x0 + 480}", "w5")); out.append(T(x0 + 484, bus5 + 4, "+5 V", "pin8 b t5"))
    out.append(W(f"M{x0 + 10} {busg} H{x0 + 505}", "wg")); out.append(T(x0 + 509, busg + 4, "GND", "pin8 b"))
    used = {CAT_GND: "fg"}
    for (_, _, _, _, cls, pin) in SIDE[side]:
        used[pin] = f"f{cls}"
    rj, pin = rj45_row(x0 + 120, y0 + 250, f"hub RJ45 {side}", used)
    out += rj
    midy = [y0 + 222, y0 + 222, y0 + 230, y0 + 240]
    for k, (n, site, ai, term, cls, cpin) in enumerate(SIDE[side]):
        sx = x0 + k * 130
        p, pd = stack(sx, y0, n, site, ai, cls)
        out += p
        (tx, ty), (rx, ry), (gx, gy) = pd["T"], pd["R"], pd["S"]
        out.append(W(f"M{rx} {ry} V{bus5}", "w5")); out.append(dot(rx, bus5, "f5"))
        out.append(W(f"M{gx} {gy} V{busg}", "wg")); out.append(dot(gx, busg, "fg"))
        px, py = pin(cpin)
        out.append(W(f"M{tx} {ty} V{midy[k]} H{px} V{py + 6}", f"w{cls}"))
    gx, gy = pin(CAT_GND)
    out.append(W(f"M{gx} {busg} V{gy + 6}", "wg")); out.append(dot(gx, busg, "fg"))
    return out, pin, (x0 + 10, bus5), (x0 + 10, busg), (x0 + 302, y0 + 265)


def body_map():
    out = [T(120, 64, "Back view · sites", "b", "middle"), T(40, 92, "L", "b", "middle"), T(200, 92, "R", "b", "middle"),
           '<path class="body" d="M70 96 Q120 82 170 96 L188 120 L178 250 Q172 300 182 340 L190 380 L50 380 L58 340 Q68 300 62 250 L52 120 Z"/>',
           '<path class="spine" d="M120 92 V378"/>']
    for lvl, y in (("L1", 236), ("L2", 262), ("L5", 330)):
        out.append(f'<path class="tick" d="M114 {y} H126"/>'); out.append(T(111, y + 3.5, lvl, "pin8", "end"))
    sites = {"R-LG": (152, 236), "L-LG": (88, 236), "R-IL": (166, 262), "L-IL": (74, 262), "R-MF": (138, 330), "L-MF": (102, 330),
             "R-EO": (174, 200), "L-EO": (66, 200)}
    for side in "RL":
        for (n, site, _, _, cls, _) in SIDE[side]:
            x, y = sites[site]
            out.append(C(x, y, 8, f"pad f{cls}")); out.append(T(x, y + 4, str(n), "pl", "middle"))
    out.append(T(120, 400, "LG = longissimus · MF = multifidus", "pin8", "middle"))
    out.append(T(120, 413, "IL = iliocostalis · EO = ext. oblique", "pin8", "middle"))
    return out


TERMS = [(33, "AI16"), (34, "AI24"), (35, "AI GND"), (36, "AI17"), (37, "AI25"), (38, "AI GND"), (39, "AI18"), (40, "AI26"),
         (41, "AI GND"), (42, "AI19"), (43, "AI27"), (44, "AI GND"), None, (49, "AI20"), (50, "AI28"), (51, "AI GND"),
         (52, "AI21"), (53, "AI29"), (54, "AI GND"), (55, "AI22"), (56, "AI30"), (57, "AI GND"),
         (58, "AI23"), (59, "AI31 · SYNC")]


def build():
    b = body_map()
    HX = 300
    hr, pinR, r5, rg, cabR = hub(HX, 80, "R")
    hl, pinL, l5, lg, cabL = hub(HX, 470, "L")
    b += hr + hl
    # Teensy 5 V and GND feed both hubs
    b.append(C(40, 440, 7.5, "pad f5")); b.append(T(52, 432, "Teensy VIN (5 V)", "pin8 b t5"))
    b.append(C(40, 462, 7.5, "pad fg")); b.append(T(52, 478, "Teensy GND", "pin8 b"))
    b.append(W(f"M40 440 H248 V{r5[1]} H{r5[0]}", "w5")); b.append(W(f"M248 440 V{l5[1]} H{l5[0]}", "w5")); b.append(dot(248, 440, "f5"))
    b.append(W(f"M40 462 H264 V{rg[1]} H{rg[0]}", "wg")); b.append(W(f"M264 462 V{lg[1]} H{lg[0]}", "wg")); b.append(dot(264, 462, "fg"))

    # DAQ terminal strip (top to bottom)
    ty = {}
    y = 90
    for t in TERMS:
        if t is None:
            b.append(T(1318, y + 4, "45–48 …", "pin8"))
        else:
            ty[t[0]] = y
        y += 22
    b.append(R(1270, 50, 210, y - 40, "daq", 6))
    b.append(T(1375, 36, "NI USB-6363 · RSE · ±5 V", "b", "middle"))
    used_t = {}
    for side in "RL":
        for (n, site, ai, term, cls, _) in SIDE[side]:
            used_t[term] = (cls, f"EMG{n} {site}")
        used_t[GND_TERM[side]] = ("g", f"GND hub {side}")
    for t in TERMS:
        if t is None:
            continue
        no, name = t
        yy = ty[no]
        if no in used_t:
            cls, lab = used_t[no]
            b.append(R(1284, yy - 9, 22, 18, f"lever f{cls}", 2)); b.append(C(1295, yy, 4.5, "screw"))
            b.append(T(1314, yy + 4, f"{no} · {name} · {lab}", f"pin b t{cls}"))
        else:
            b.append(R(1284, yy - 9, 22, 18, "lever", 2)); b.append(C(1295, yy, 4.5, "screwo"))
            b.append(T(1314, yy + 4, f"{no} · {name}", "pin8"))

    # DAQ-end adapters and Cat6
    for side, ay, cab, cx in (("R", 60, cabR, 990), ("L", 520, cabL, 1010)):
        used = {CAT_GND: "fg"}
        for (_, _, _, _, cls, pin) in SIDE[side]:
            used[pin] = f"f{cls}"
        a, pr = rj45_col(1060, ay, f"DAQ-end {side}", used)
        b += a
        my = ay + 91
        b.append(f'<path class="cab" d="M{cab[0]} {cab[1]} H{cx} V{my} H1060"/>')
        b.append(T(cab[0] + 20, cab[1] - 10, f"Cat6 {side} · 10 m", "s"))
        # wires: pin order 1 (LG), 2 (GND), 3 (MF), 4 (IL), 5 (EO)
        order = [(1, SIDE[side][0][3], SIDE[side][0][4]), (2, GND_TERM[side], "g"),
                 (3, SIDE[side][1][3], SIDE[side][1][4]), (4, SIDE[side][2][3], SIDE[side][2][4]),
                 (5, SIDE[side][3][3], SIDE[side][3][4])]
        lanes = [1170, 1155, 1140, 1125, 1110] if side == "R" else [1190, 1205, 1220, 1235, 1250]
        for (pin, term, cls), lx in zip(order, lanes):
            px, py = pr(pin)
            b.append(W(f"M{px} {py} H{lx} V{ty[term]} H1284", f"w{cls}"))
    b.append(T(1040, 790, "AI GND = 3rd terminal of each group: check the label on the DAQ", "pin8"))
    b.append(T(30, 840, "TRS cable, Link Shield jack → breakout:  T tip = EMG signal (RAW, ≈ 2.5 V offset)  ·  R ring = +5 V  ·  S sleeve = GND.   "
               "Link Shield: PWR ON, OUTPUT RAW.  Cable Shield: MID + END on the muscle, REF on bone.", "s"))
    body = "\n".join(b)
    return (f'<svg viewBox="0 0 1500 860" role="img" aria-label="Pin-level EMG wiring: eight MyoWare 2 in hub R and hub L, '
            f'TRS tip ring sleeve, Cat6 to NI USB-6363 terminals" xmlns="http://www.w3.org/2000/svg">\n{body}\n</svg>')


CSS = """
:root { --bg:#f6f7f6; --surface:#ffffff; --ink:#17201d; --muted:#5b6662; --line:#d5dcd9; --accent:#0b6870; --accent-soft:#e0eff0;
  --w5:#db7f12; --wg:#27302d; --e1:#12925e; --e2:#0b8ea3; --e3:#cc3f7a; --e4:#8a5a2b; --e5:#6d8b0f; --e6:#4f5bd5; --e7:#b8860b; --e8:#a03fbf; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg:#111615; --surface:#1a201e; --ink:#e5ebe8; --muted:#9ba8a3; --line:#2e3835; --accent:#52b8c1; --accent-soft:#15302f;
  --w5:#f0a24a; --wg:#cfd6d3; --e1:#4cc98d; --e2:#4cc3d6; --e3:#f07aa8; --e4:#c9945e; --e5:#a8c94a; --e6:#8e97f2; --e7:#e0b84a; --e8:#d08ae8; } }
:root[data-theme="dark"] { --bg:#111615; --surface:#1a201e; --ink:#e5ebe8; --muted:#9ba8a3; --line:#2e3835; --accent:#52b8c1; --accent-soft:#15302f;
  --w5:#f0a24a; --wg:#cfd6d3; --e1:#4cc98d; --e2:#4cc3d6; --e3:#f07aa8; --e4:#c9945e; --e5:#a8c94a; --e6:#8e97f2; --e7:#e0b84a; --e8:#d08ae8; }
html, body { margin:0; background:var(--bg); color:var(--ink); font:14px/1.45 "Segoe UI", system-ui, sans-serif; }
main { max-width:1500px; margin:0 auto; padding:16px; }
h1 { font-size:20px; margin:0 0 4px; } p { margin:0 0 8px; color:var(--muted); }
.dia { border:1px solid var(--line); border-radius:8px; background:var(--surface); overflow-x:auto; }
.dia svg { display:block; width:100%; min-width:1100px; height:auto; }
svg text { fill:var(--ink); font-family:"Segoe UI", system-ui, sans-serif; font-size:12px; }
svg text.b { font-weight:600; font-size:13px; } svg text.s { fill:var(--muted); font-size:11.5px; }
svg text.pin { font-family:Consolas, monospace; font-size:11px; } svg text.pin8 { font-family:Consolas, monospace; font-size:10px; fill:var(--muted); }
svg text.pin8.b, svg text.pin.b { font-weight:700; fill:var(--ink); } svg text.inv { fill:var(--surface); font-weight:700; }
svg text.pl { fill:var(--surface); font-weight:700; font-size:11.5px; }
svg text.t5 { fill:var(--w5); } svg text.tg { fill:var(--ink); }
svg text.te1 { fill:var(--e1); } svg text.te2 { fill:var(--e2); } svg text.te3 { fill:var(--e3); }
svg text.te4 { fill:var(--e4); } svg text.te5 { fill:var(--e5); } svg text.te6 { fill:var(--e6); } svg text.te7 { fill:var(--e7); } svg text.te8 { fill:var(--e8); }
.pcb { fill:var(--bg); stroke:var(--ink); stroke-width:1.4; } .part { fill:var(--surface); stroke:var(--muted); stroke-width:1.1; }
.part.fa { fill:var(--accent-soft); stroke:var(--accent); stroke-width:1.5; } .pad { stroke:var(--surface); stroke-width:1.2; }
.zone { fill:none; stroke:var(--line); stroke-width:1.2; stroke-dasharray:6 4; }
.lever { fill:var(--line); stroke:var(--muted); stroke-width:1; } .daq { fill:var(--bg); stroke:var(--ink); stroke-width:1.3; }
.screw { fill:var(--surface); stroke:var(--surface); stroke-width:1; } .screwo { fill:none; stroke:var(--muted); stroke-width:1.2; }
.body { fill:var(--accent-soft); stroke:var(--muted); stroke-width:1.2; } .spine { stroke:var(--muted); stroke-width:1.5; stroke-dasharray:4 3; fill:none; }
.tick { stroke:var(--muted); stroke-width:1.5; }
.f5 { fill:var(--w5); } .fg { fill:var(--wg); } .fe1 { fill:var(--e1); } .fe2 { fill:var(--e2); } .fe3 { fill:var(--e3); }
.fe4 { fill:var(--e4); } .fe5 { fill:var(--e5); } .fe6 { fill:var(--e6); } .fe7 { fill:var(--e7); } .fe8 { fill:var(--e8); }
.lever.f5, .lever.fg, .lever.fe1, .lever.fe2, .lever.fe3, .lever.fe4, .lever.fe5, .lever.fe6, .lever.fe7, .lever.fe8 { stroke:none; }
.w { fill:none; stroke-width:2.4; stroke-linecap:round; stroke-linejoin:round; }
.w5 { stroke:var(--w5); } .wg { stroke:var(--wg); } .we1 { stroke:var(--e1); } .we2 { stroke:var(--e2); } .we3 { stroke:var(--e3); }
.we4 { stroke:var(--e4); } .we5 { stroke:var(--e5); } .we6 { stroke:var(--e6); } .we7 { stroke:var(--e7); } .we8 { stroke:var(--e8); }
.trs { fill:none; stroke:var(--muted); stroke-width:5; stroke-linecap:round; }
.cab { fill:none; stroke:var(--muted); stroke-width:7; stroke-linecap:round; stroke-linejoin:round; }
"""

page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>EMG pin wiring</title><style>{CSS}</style></head><body><main>
<h1>EMG wiring · pin level</h1>
<p>Senior's SID Lab map (SETUP.md, 2026-11-25) + EMG7/8 on AI22/AI23 (2026-11-26): 8 × MyoWare 2, NI USB-6363 single-ended (RSE), ±5 V, 4 kS/s.</p>
<div class="dia">{build()}</div>
</main></body></html>
"""

if __name__ == "__main__":
    OUT.write_text(page, encoding="utf-8")
    print("wrote", OUT, len(page))
