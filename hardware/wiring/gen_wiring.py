"""Generates teensy41_wiring.html: pin-level wiring of the SID Lab spine brace, taken from the senior's
handover (SID_Lab_handover 2026-11-25: PIN_DIAGRAM.svg, SETUP.md section 2, teensy_spine_imu.ino header).
8 BNO086 on 2 Qwiic muxes (bus A = Wire1 16/17, bus B = Wire 19/18), Teensy -> NI USB-6363 (GND 82,
TX1 -> P0.0 65, SYNC -> AI31 59), 8 EMG RSE on AI16-23 (EMG7/8 on AI22/AI23, terminals 55/58, added 2026-11-26).  Run:  python gen_wiring.py
"""
import pathlib

OUT = pathlib.Path(__file__).with_name("teensy41_wiring.html")


def f(v):
    return f"{v:.1f}".rstrip("0").rstrip(".")


def T(x, y, s, cls="", anchor=None):
    a = f' text-anchor="{anchor}"' if anchor else ""
    return f'<text class="{cls}" x="{f(x)}" y="{f(y)}"{a}>{s}</text>'


def R(x, y, w, h, cls, rx=3):
    return f'<rect class="{cls}" x="{f(x)}" y="{f(y)}" width="{f(w)}" height="{f(h)}" rx="{rx}"/>'


def C(x, y, r, cls):
    return f'<circle class="{cls}" cx="{f(x)}" cy="{f(y)}" r="{f(r)}"/>'


def W(pts, cls):
    """Right-angle wire through the given points."""
    d = "M" + " L".join(f"{f(x)} {f(y)}" for x, y in pts)
    return f'<path class="w {cls}" d="{d}"/>'


def svg(w, h, label, body):
    return (f'<svg viewBox="0 0 {w} {h}" role="img" aria-label="{label}" xmlns="http://www.w3.org/2000/svg">\n'
            + "\n".join(body) + "\n</svg>")


# ------------------------------------------------------------------ Teensy 4.1 (top view, USB on the left)
TOP = ["VIN", "GND", "3V3", "23", "22", "21", "20", "19", "18", "17", "16", "15", "14", "13", "GND",
       "41", "40", "39", "38", "37", "36", "35", "34", "33"]
BOT = ["GND", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "3V3",
       "24", "25", "26", "27", "28", "29", "30", "31", "32"]
PITCH = 24


def teensy(x0, y0, used_top, used_bot, title):
    """used_*: {index: colour key}. Returns (svg parts, pin xy function)."""
    b = [R(x0, y0, 24 * PITCH + 16, 128, "pcb", 6), R(x0 - 14, y0 + 40, 38, 48, "part", 3),
         T(x0 + 5, y0 + 69, "USB", "s", "middle"), R(x0 + 24 * PITCH - 120, y0 + 46, 108, 40, "part", 3),
         T(x0 + 24 * PITCH - 66, y0 + 68, "microSD", "s", "middle"),
         T(x0 + 10 * PITCH, y0 + 62, title, "b", "middle"),
         T(x0 + 10 * PITCH, y0 + 78, "3.3 V pins only · never 5 V on any pin", "s", "middle")]

    def xy(row, i):
        return x0 + 18 + i * PITCH, (y0 + 12 if row == "t" else y0 + 116)

    for row, names, used in (("t", TOP, used_top), ("b", BOT, used_bot)):
        for i, n in enumerate(names):
            x, y = xy(row, i)
            c = used.get(i)
            b.append(C(x, y, 7.5 if c else 5, f"pad f{c}" if c else "pad"))
            ly = y + 24 if row == "t" else y - 14
            b.append(T(x, ly, n, f"pin b t{c}" if c else "pin", "middle"))
    return b, xy


# ------------------------------------------------------------------ figure: whole brace
def figure():
    b = []
    TX, TY = 300, 470
    tb, P = teensy(TX, TY, {1: "g", 2: "3", 7: "c", 8: "d", 9: "d", 10: "c", 14: "g"},
                   {0: "g", 2: "t", 3: "s", 5: "x", 14: "3"}, "Teensy 4.1 · top view")
    b += tb
    p = lambda row, i: P(row, i)

    # ---------------- muxes and pods
    def mux(x0, y0, name, pods, imu0):
        out = [R(x0, y0, 470, 70, "pcb", 5), T(x0 + 235, y0 + 40, name, "b", "middle"),
               T(x0 + 235, y0 + 56, "SparkFun Qwiic Mux (TCA9548A) · ch 4–7 free", "s", "middle")]
        socks = []
        for k in range(8):
            sx = x0 + 20 + k * 55
            used = k < 4
            out.append(R(sx - 9, y0 - 6, 18, 12, "part fa" if used else "part", 2))
            out.append(T(sx, y0 + 20, f"ch{k}", "pin8 b" if used else "pin8", "middle"))
            socks.append(sx)
        for k, site in enumerate(pods):
            ix = x0 + 5 + k * 117
            out += [R(ix, 60, 105, 46, "pcb", 3), R(ix + 43, 100, 18, 10, "part fa", 2),
                    T(ix + 52, 78, f"IMU{imu0 + k} · {site}", "b", "middle"),
                    T(ix + 52, 93, "BNO086 · 0x4B", "pin8", "middle")]
            jy = 130 + k * 14
            out.append(f'<path class="qw" d="M{f(socks[k])} {y0 - 6} V{jy} H{ix + 52} V110"/>')
        return out

    MBX, MAX_, MY = 40, 560, 200
    b += mux(MBX, MY, "Mux B · Wire (19/18) · 0x70", ["T11", "L1", "L3", "S1"], 5)
    b += mux(MAX_, MY, "Mux A · Wire1 (16/17) · 0x70", ["C7", "T3", "T6", "T9"], 1)

    # ---------------- Qwiic leads: plug columns, outer pin turns highest (no crossings)
    PY = 400

    def lead(plugx, cols, name, mainx):
        # cols: [(pin_xy, column_x, turn_y, colour)]
        out = [R(plugx - 4, PY, 30, 12, "plug", 2), T(plugx - 10, PY + 10, name, "lab", "end")]
        for (x, y), cx, ty, c in cols:
            out.append(W([(x, y), (x, ty), (cx, ty), (cx, PY + 12)], f"w{c}"))
        out.append(f'<path class="qw" d="M{plugx + 11} {PY} V{(PY + MY + 76) / 2} H{mainx} V{MY + 76}"/>')
        out += [R(mainx - 11, MY + 70, 22, 12, "part fa", 2), T(mainx + 16, MY + 98, "MAIN", "pin8 q")]
        return out

    # lead B: GND (top 1), 3V3 (top 2) from the left; SCL 19 (top 7), SDA 18 (top 8) from the right
    b += lead(400, [(p("t", 1), 400, 440, "g"), (p("t", 2), 407, 456, "3"),
                    (p("t", 7), 414, 456, "c"), (p("t", 8), 421, 440, "d")], "lead B", 411)
    # lead A: SDA1 17 (top 9), SCL1 16 (top 10) from the left; GND (top 14) and 3V3 (bottom 14) from the right
    gx, gy = p("t", 14)
    vx, vy = p("b", 14)
    b += lead(590, [(p("t", 9), 590, 440, "d"), (p("t", 10), 597, 456, "c"), ((gx, gy), 604, 456, "g")], "lead A", 601)
    b.append(W([(vx, vy), (vx, 616), (924, 616), (924, 432), (611, 432), (611, PY + 12)], "w3"))
    b.append(T(930, 520, "3V3 (bottom row)", "pin8 b t3"))
    b.append(T(700, 340, "Qwiic: red 3V3 · black GND · blue SDA · yellow SCL", "s"))

    # ---------------- DAQ
    DX, DY = 150, 700
    b.append(R(DX, DY, 1030, 140, "daq", 6))
    b.append(T(DX + 515, DY + 62, "NI USB-6363 · screw terminals (used ones only)", "b", "middle"))
    b.append(T(DX + 515, DY + 78, "EMG: RSE · ±5 V · 4 kHz · IMU packet on P0.0 · SYNC on AI31", "s", "middle"))

    def term(x, y, num, label, cls, top=True):
        ly = y + 26 if top else y - 18
        return [R(x - 11, y - 9, 22, 18, "lever", 2), C(x, y, 4.5, "screw"),
                T(x, ly, num, f"pin b {cls}".strip(), "middle"),
                T(x, ly + (12 if top else -12), label, "pin8", "middle")]

    # Teensy -> DAQ (top edge): leftmost wire turns highest
    ty = DY + 10
    for (pin, tx, jy, num, lab, c) in ((p("b", 0), 200, 628, "82", "D GND", "g"),
                                      (p("b", 2), 270, 642, "65", "P0.0", "t"),
                                      (p("b", 3), 340, 656, "59", "AI31", "s")):
        b += term(tx, ty, num, lab, f"t{c}")
        b.append(W([pin, (pin[0], jy), (tx, jy), (tx, ty - 9)], f"w{c}"))
    # optional DAQ frame clock -> divider -> pin 4
    p4 = p("b", 5)
    b += term(470, ty, "PFI", "ctr out", "")
    b.append(W([p4, (p4[0], 670), (470, 670), (470, ty - 9)], "wx"))
    b.append(R(452, 676, 36, 10, "res", 2))
    b.append(T(496, 684, "optional · FRAME_CLOCK 1 · 1 kΩ PFI→pin 4, 2 kΩ pin 4→GND", "pin8"))

    # EMG (bottom edge): signal straight up, reference rail under the sources -> AI GND
    by = DY + 130
    emg = [("33", "AI16", "R-LG"), ("36", "AI17", "L-LG"), ("39", "AI18", "R-MF"),
           ("42", "AI19", "L-MF"), ("49", "AI20", "R-IL"), ("52", "AI21", "L-IL"),
           ("55", "AI22", "R-EO"), ("58", "AI23", "L-EO")]
    for k, (num, ai, site) in enumerate(emg):
        x = 500 + k * 76
        b += term(x, by, num, ai, "te", top=False)
        b += [R(x - 14, 890, 60, 40, "part", 3), T(x + 16, 906, f"EMG{k + 1}", "pin8 b", "middle"),
              T(x + 16, 920, site, "pin8", "middle"),
              C(x, 890, 4, "pad fe"), C(x + 28, 930, 4, "pad fg")]
        b.append(W([(x, 890), (x, by + 9)], "we"))
        b.append(W([(x + 28, 930), (x + 28, 948)], "wg"))
    gx_ = 500 + 8 * 76
    b += term(gx_, by, "AI GND", "EMG ref", "", top=False)
    b.append(W([(500 + 28, 948), (gx_, 948), (gx_, by + 9)], "wg"))
    b.append(T(150, 910, "EMG out → terminal; reference → AI GND", "s"))
    b.append(T(150, 926, "twisted pair (signal + GND) for any lead over ~1 m", "s"))
    return svg(1220, 970, "Pin-level wiring of the SID Lab spine brace: Teensy 4.1, two Qwiic muxes, eight BNO086, NI USB-6363 and eight EMG inputs", b)


CSS = """
:root { --bg:#f6f7f6; --surface:#ffffff; --ink:#17201d; --muted:#5b6662; --line:#d5dcd9; --accent:#0b6870; --accent-soft:#e0eff0;
  --w3:#d23a3a; --w5:#db7f12; --wg:#27302d; --wd:#2b67d1; --wc:#b08a00; --we:#17915a; --ws:#8446cf; --wt:#0b8aa0; --wx:#8a9490; --plug:#3b4441; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg:#111615; --surface:#1a201e; --ink:#e5ebe8; --muted:#9ba8a3; --line:#2e3835; --accent:#52b8c1; --accent-soft:#15302f;
  --w3:#f26b6b; --w5:#f0a24a; --wg:#cfd6d3; --wd:#6c9df2; --wc:#e2bf3c; --we:#47c487; --ws:#b487f2; --wt:#3fc3d6; --wx:#7d8884; --plug:#8d9893; } }
:root[data-theme="dark"] { --bg:#111615; --surface:#1a201e; --ink:#e5ebe8; --muted:#9ba8a3; --line:#2e3835; --accent:#52b8c1; --accent-soft:#15302f;
  --w3:#f26b6b; --w5:#f0a24a; --wg:#cfd6d3; --wd:#6c9df2; --wc:#e2bf3c; --we:#47c487; --ws:#b487f2; --wt:#3fc3d6; --wx:#7d8884; --plug:#8d9893; }
html, body { margin:0; background:var(--bg); color:var(--ink); font:14px/1.45 "Segoe UI", system-ui, sans-serif; }
main { max-width:1320px; margin:0 auto; padding:16px; }
h1 { font-size:20px; margin:0 0 4px; } h2 { font-size:15px; margin:18px 0 6px; } p { margin:0 0 6px; color:var(--muted); }
.dia { border:1px solid var(--line); border-radius:8px; background:var(--surface); overflow-x:auto; }
.dia svg { display:block; width:100%; min-width:860px; height:auto; }
.legend { display:flex; flex-wrap:wrap; gap:4px 14px; font-size:12px; color:var(--muted); margin:6px 0; }
.legend i { display:inline-block; width:20px; border-top:3px solid; margin-right:5px; vertical-align:middle; }
svg text { fill:var(--ink); font-family:"Segoe UI", system-ui, sans-serif; font-size:12px; }
svg text.b { font-weight:600; font-size:13px; } svg text.s { fill:var(--muted); font-size:11.5px; }
svg text.pin { font-family:Consolas, monospace; font-size:10.5px; } svg text.pin8 { font-family:Consolas, monospace; font-size:10px; fill:var(--muted); }
svg text.pin8.b, svg text.pin.b { font-weight:700; fill:var(--ink); } svg text.lab { fill:var(--muted); font-style:italic; font-size:11.5px; }
svg text.inv { fill:var(--surface); font-weight:700; } svg text.q { fill:var(--accent); font-weight:600; }
svg text.t3 { fill:var(--w3); } svg text.t5 { fill:var(--w5); } svg text.tg { fill:var(--ink); } svg text.td { fill:var(--wd); }
svg text.tc { fill:var(--wc); } svg text.te { fill:var(--we); } svg text.ts { fill:var(--ws); } svg text.tt { fill:var(--wt); } svg text.tx { fill:var(--muted); }
.pcb { fill:var(--bg); stroke:var(--ink); stroke-width:1.4; } .part { fill:var(--surface); stroke:var(--muted); stroke-width:1.1; }
.part.fa { fill:var(--accent-soft); stroke:var(--accent); stroke-width:1.5; } .pad { fill:var(--surface); stroke:var(--ink); stroke-width:1.2; }
.zone { fill:none; stroke:var(--line); stroke-width:1.2; stroke-dasharray:6 4; }
.wago { fill:var(--surface); stroke:var(--muted); stroke-width:1.3; } .lever { fill:var(--line); stroke:var(--muted); stroke-width:1; }
.daq { fill:var(--bg); stroke:var(--ink); stroke-width:1.3; } .screw { fill:var(--surface); stroke:var(--muted); stroke-width:1; }
.res { fill:var(--surface); stroke:var(--wg); stroke-width:2; } .plug { fill:var(--plug); }
.f3 { fill:var(--w3); } .f5 { fill:var(--w5); } .fg { fill:var(--wg); } .fd { fill:var(--wd); } .fc { fill:var(--wc); } .fe { fill:var(--we); } .fs { fill:var(--ws); } .ft { fill:var(--wt); } .fx { fill:var(--wx); }
.pad.f3 { fill:var(--w3); } .pad.f5 { fill:var(--w5); } .pad.fg { fill:var(--wg); } .pad.fd { fill:var(--wd); } .pad.fc { fill:var(--wc); } .pad.fe { fill:var(--we); } .pad.fs { fill:var(--ws); } .pad.ft { fill:var(--wt); } .pad.fx { fill:var(--wx); }
.lever.f3, .lever.f5, .lever.fg, .lever.fe, .lever.fs { stroke:none; }
.lever.f5 { fill:var(--w5); } .lever.fg { fill:var(--wg); } .lever.fe { fill:var(--we); } .lever.fs { fill:var(--ws); }
.w { fill:none; stroke-width:2.4; stroke-linecap:round; stroke-linejoin:round; }
.w3 { stroke:var(--w3); } .w5 { stroke:var(--w5); } .wg { stroke:var(--wg); } .wd { stroke:var(--wd); } .wc { stroke:var(--wc); } .we { stroke:var(--we); } .ws { stroke:var(--ws); } .wt { stroke:var(--wt); } .wx { stroke:var(--wx); stroke-dasharray:7 5; }
.qw { fill:none; stroke:var(--accent); stroke-width:5; stroke-linecap:round; stroke-linejoin:round; }
.cab { fill:none; stroke:var(--muted); stroke-width:7; stroke-linecap:round; stroke-linejoin:round; }
"""


LEGEND = ('<div class="legend"><span><i style="border-color:var(--w3)"></i>3.3 V</span>'
          '<span><i style="border-color:var(--wg)"></i>GND</span><span><i style="border-color:var(--wd)"></i>SDA</span>'
          '<span><i style="border-color:var(--wc)"></i>SCL</span><span><i style="border-color:var(--wt)"></i>IMU data (TX1 → P0.0)</span>'
          '<span><i style="border-color:var(--ws)"></i>SYNC</span><span><i style="border-color:var(--we)"></i>EMG signal</span>'
          '<span><i style="border-color:var(--wx);border-top-style:dashed"></i>optional</span>'
          '<span><i style="border-color:var(--accent);border-top-width:5px"></i>Qwiic cable</span></div>')

page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>SID Lab spine brace wiring</title><style>{CSS}</style></head><body><main>
<h1>SID Lab spine brace wiring · pin level</h1>
<p>From the senior's handover (2026-11-25): firmware <code>teensy_spine_imu.ino</code>, PIN_DIAGRAM, SETUP.md §2; EMG7/8 on AI22/AI23 added 2026-11-26.</p>
{LEGEND}
<div class="dia">{figure()}</div>
<p>Teensy on USB to the PC (power + status). FRAME_CLOCK 2 (normal): Teensy makes its own 120 Hz frames, pin 4 stays unconnected.</p>
</main></body></html>
"""
OUT.write_text(page, encoding="utf-8")
print("wrote", OUT, len(page))
