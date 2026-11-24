"""Generates teensy41_wiring.html: pin-level wiring of the pilot system (senior's pin map, 2026-11-19).
6 BNO086 on 2 Qwiic muxes (3 per mux, Wire / Wire1), 8 MyoWare 2.0 EMG channels in two thigh hubs,
2 Cat6 cables to the NI USB-6363 (RSE), SYNC through 100 ohm to AI16.  Run:  python gen_wiring.py
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


def W(d, cls):
    return f'<path class="w {cls}" d="{d}"/>'


def dot(x, y, cls):
    return C(x, y, 3.4, cls)


def svg(w, h, label, body):
    return (f'<svg viewBox="0 0 {w} {h}" role="img" aria-label="{label}" xmlns="http://www.w3.org/2000/svg">\n'
            + "\n".join(body) + "\n</svg>")


# ------------------------------------------------------------------ Teensy 4.1 (top view, USB on the left)
TOP = ["VIN", "GND", "3V3", "23", "22", "21", "20", "19", "18", "17", "16", "15", "14", "13", "GND",
       "41", "40", "39", "38", "37", "36", "35", "34", "33"]
BOT = ["GND", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "3V3",
       "24", "25", "26", "27", "28", "29", "30", "31", "32"]


def teensy(x0, y0, used_top, used_bot, title):
    """used_*: {index: colour class}. Returns (svg parts, pin xy function)."""
    p = 24
    b = [R(x0, y0, 24 * p + 16, 128, "pcb", 6), R(x0 - 14, y0 + 40, 38, 48, "part", 3),
         T(x0 + 5, y0 + 69, "USB", "s", "middle"), R(x0 + 24 * p - 120, y0 + 30, 108, 68, "part", 3),
         T(x0 + 24 * p - 66, y0 + 68, "microSD", "s", "middle"), T(x0 + 12 * p + 8, y0 + 70, title, "b", "middle")]

    def xy(row, i):
        return x0 + 18 + i * p, (y0 + 12 if row == "t" else y0 + 116)

    for row, names, used in (("t", TOP, used_top), ("b", BOT, used_bot)):
        for i, n in enumerate(names):
            x, y = xy(row, i)
            c = used.get(i)
            b.append(C(x, y, 7.5 if c else 5, f"pad {c}" if c else "pad"))
            ly = y + 24 if row == "t" else y - 14
            b.append(T(x, ly, n, f"pin b t{c[1:]}" if c else "pin", "middle"))
    return b, xy


def qplug(x, y):
    return R(x, y, 28, 12, "plug", 2)


# ------------------------------------------------------------------ figure 1: Teensy, button, muxes, IMUs
def fig1():
    b = []
    # Teensy at the bottom
    tb, xy = teensy(70, 470, {0: "f5", 1: "fg", 2: "f3", 7: "fc", 8: "fd", 9: "fd", 10: "fc", 14: "fg"},
                    {0: "fg", 3: "fs", 14: "f3"}, "Teensy 4.1 · top view · logic hub")
    b += tb
    P = {k: xy(*v) for k, v in {"VIN": ("t", 0), "GNDv": ("t", 1), "3V3t": ("t", 2), "19": ("t", 7), "18": ("t", 8),
                                 "17": ("t", 9), "16": ("t", 10), "GND13": ("t", 14), "GND0": ("b", 0), "2": ("b", 3),
                                 "3V3b": ("b", 14)}.items()}
    # lead A: GND (next to 0), 3V3 (bottom row), 18, 19 -> plug A -> Qwiic Button
    ax = 236                                          # plug A wire entries: GND, 3V3, SCL, SDA
    b.append(qplug(ax - 3, 384))
    gx, vx, cx, dx = ax, ax + 7, ax + 14, ax + 21
    b.append(W(f"M{f(P['19'][0])} {f(P['19'][1])} V438 H{cx} V396", "wc"))
    b.append(W(f"M{f(P['18'][0])} {f(P['18'][1])} V430 H{dx} V396", "wd"))
    b.append(W(f"M{f(P['GND0'][0])} {f(P['GND0'][1])} V622 H40 V412 H{gx} V396", "wg"))
    b.append(W(f"M{f(P['3V3b'][0])} {f(P['3V3b'][1])} V632 H30 V420 H{vx} V396", "w3"))
    b.append(T(ax - 8, 378, "lead A", "lab", "end"))
    # lead B: GND (between 13 and 41), 3V3 (top row), 17, 16 -> plug B -> long cable B
    bx = 470
    b.append(qplug(bx - 3, 384))
    b.append(W(f"M{f(P['3V3t'][0])} {f(P['3V3t'][1])} V448 H{bx} V396", "w3"))
    b.append(W(f"M{f(P['17'][0])} {f(P['17'][1])} V444 H{bx + 7} V396", "wd"))
    b.append(W(f"M{f(P['16'][0])} {f(P['16'][1])} V440 H{bx + 14} V396", "wc"))
    b.append(W(f"M{f(P['GND13'][0])} {f(P['GND13'][1])} V436 H{bx + 21} V396", "wg"))
    b.append(T(bx + 34, 394, "lead B", "lab"))
    # stubs to figure 2: VIN, GND (next to VIN), pin 2
    b.append(W(f"M{f(P['VIN'][0])} {f(P['VIN'][1])} V404 H30", "w5"))
    b.append(W(f"M{f(P['GNDv'][0])} {f(P['GNDv'][1])} V388 H30", "wg"))
    b.append(T(34, 383, "GND → W2 (hub R)", "pin8"))
    b.append(T(110, 400, "VIN → W1 (hub R)", "pin8 t5"))
    b.append(W(f"M{f(P['2'][0])} {f(P['2'][1])} V650", "ws"))
    b.append(T(P['2'][0] + 8, 646, "SYNC → 100 Ω → RJ45 R pin 5 (figure 2)", "pin8 ts"))
    b.append(T(70, 668, "USB ← power bank (always-on mode)", "s"))

    # Qwiic Button in-line on Wire (logic hub)
    b.append(R(150, 320, 110, 40, "part", 4))
    b.append(T(205, 338, "Qwiic Button", "pin8 b", "middle"))
    b.append(T(205, 352, "0x6F · MARK", "pin8", "middle"))
    b.append(R(144, 330, 12, 20, "part fa", 2)); b.append(R(254, 330, 12, 20, "part fa", 2))
    b.append(f'<path class="qw" d="M{ax + 11} 384 V340 H266"/>'.replace("H266", "H266"))
    # plug A goes into the button's right port; long cable A leaves from its left port
    # muxes and IMUs on the back
    imus = []

    def mux(x0, y0, name, sub):
        out = [R(x0, y0, 120, 150, "pcb", 4), T(x0 + 60, y0 - 26, name, "b", "middle"),
               T(x0 + 60, y0 - 11, sub, "s", "middle"),
               R(x0 + 49, y0 + 132, 22, 14, "part fa", 2), T(x0 + 44, y0 + 143, "MAIN in", "pin8 q", "end"),
               R(x0 + 46, y0 + 60, 28, 40, "part", 2), T(x0 + 60, y0 + 84, "TCA", "pin8", "middle")]
        for k in range(4):
            y = y0 + 22 + k * 30
            used = k < 3
            out.append(R(x0 + 104, y, 14, 18, "part fa" if used else "part", 2))
            out.append(T(x0 + 100, y + 13, f"ch{k}", "pin8 b" if used else "pin8", "end"))
            out.append(R(x0 + 2, y, 14, 18, "part", 2)); out.append(T(x0 + 20, y + 13, f"ch{7 - k}", "pin8"))
        return out

    def imu(x0, y0, name, cable):
        return [R(x0, y0, 120, 40, "pcb", 3), R(x0 + 1, y0 + 10, 12, 20, "part fa", 2), R(x0 + 107, y0 + 10, 12, 20, "part", 2),
                T(x0 + 62, y0 + 18, name, "b", "middle"), T(x0 + 62, y0 + 33, "0x4B · " + cable, "pin8", "middle")]

    for (mx, my, mname, msub, names, ix) in ((110, 90, "Mux A · Wire · 0x70", "taped right of the T7 pod", ["T1", "T7", "T12"], 300),
                                             (560, 90, "Mux B · Wire1 · 0x70", "taped just below the S2 pod", ["L3", "L5", "S2"], 750)):
        b += mux(mx, my, mname, msub)
        for k, n in enumerate(names):
            y = my + 22 + k * 30 + 9
            iy = 40 + k * 70 if False else my - 10 + k * 56
            b += imu(ix, iy, f"IMU · {n}", "50 cm")
            b.append(f'<path class="qw" d="M{mx + 118} {y} H{(mx + 118 + ix) / 2} V{iy + 20} H{ix + 1}"/>')
    # long cable A: button left port -> Mux A main ; long cable B: plug B -> Mux B main
    b.append('<path class="qw" d="M144 340 H120 V300 H170 V238"/>')
    b.append(T(70, 296, "long cable A ≈ 1.2 m", "lab"))
    b.append(f'<path class="qw" d="M{bx + 11} 384 V300 H620 V238"/>')
    b.append(T(bx + 20, 318, "long cable B ≈ 0.8 m", "lab"))
    b.append(T(30, 26, "Back: 2 muxes, 3 IMUs each (ch 0–2), Qwiic 50 cm per IMU", "s"))
    return svg(920, 680, "Teensy 4.1 pin-level wiring to the Qwiic Button, two muxes and six BNO086", b)


# ------------------------------------------------------------------ figure 2: EMG hubs, Cat6, DAQ end
CH = ["LT_T9", "LT_L1", "IL_L2", "MF_L5"]
SIGPIN = [1, 2, 3, 6]


def rj45_row(x0, y0, title, used, title_above=True):
    """Horizontal RJ45 spring adapter, terminals 1-8. used = {pin: class}."""
    out = [R(x0, y0, 8 * 22 + 6, 30, "pcb", 4),
           (T(x0 + 92, y0 - 8, title, "b", "middle") if title_above else T(x0 - 10, y0 + 20, title, "b", "end"))]
    for k in range(8):
        c = used.get(k + 1, "")
        out.append(R(x0 + 4 + 22 * k, y0 + 6, 16, 18, f"lever {c}".strip(), 2))
        out.append(T(x0 + 12 + 22 * k, y0 + 19, str(k + 1), "pin8 inv" if c else "pin8", "middle"))
    return out, (lambda pin: (x0 + 12 + 22 * (pin - 1), y0))


def rj45_col(x0, y0, title, used):
    """Vertical DAQ-end adapter, terminals 1-8 top to bottom. Returns parts and pin -> (x right edge, y)."""
    out = [R(x0, y0, 34, 8 * 22 + 6, "pcb", 4), T(x0 + 17, y0 - 10, title, "b", "middle")]
    for k in range(8):
        c = used.get(k + 1, "")
        out.append(R(x0 + 6, y0 + 4 + 22 * k, 22, 16, f"lever {c}".strip(), 2))
        out.append(T(x0 + 17, y0 + 16 + 22 * k, str(k + 1), "pin8 inv" if c else "pin8", "middle"))
    return out, (lambda pin: (x0 + 34, y0 + 12 + 22 * (pin - 1)))


def trrs(x0, y0, name):
    """Adafruit 5764 TRRS breakout, pads T (Left), R (Right), S (Sleeve) on the bottom edge."""
    out = [R(x0, y0, 76, 62, "pcb", 4), R(x0 + 18, y0 + 6, 40, 26, "part", 3), T(x0 + 38, y0 + 23, "jack", "pin8", "middle"),
           T(x0 + 38, y0 - 8, name, "pin8 b", "middle")]
    pads = {}
    for k, (lab, cls) in enumerate((("T", "fe"), ("R", "f5"), ("S", "fg"))):
        x = x0 + 14 + 24 * k
        out.append(C(x, y0 + 48, 6.5, f"pad {cls}"))
        out.append(T(x, y0 + 74, lab, "pin8 b", "middle"))
        pads[lab] = (x, y0 + 48)
    return out, pads


def wago(x0, y0, name, labels, cls):
    out = [R(x0, y0, 5 * 26 + 8, 30, "wago", 4), T(x0, y0 - 6, name, "pin8 b")]
    for k in range(5):
        c = cls if labels[k] else ""
        out.append(R(x0 + 4 + 26 * k, y0 + 6, 22, 16, f"lever {c}".strip(), 2))
        out.append(T(x0 + 15 + 26 * k, y0 + 44, labels[k] or "–", "pin8", "middle"))
    return out, (lambda k: (x0 + 15 + 26 * (k - 1), y0))


def hub(x0, y0, side, has_wagos):
    """One EMG hub: 4 TRRS breakouts, +5 V and GND buses, hub-side RJ45. Returns parts and anchor points."""
    out = [R(x0 - 44, y0 - 40, 470, 288, "zone", 10),
           T(x0 - 34, y0 - 22, f"EMG hub {side} · {'upper right' if side == 'R' else 'left'} thigh band", "b")]
    pads = []
    for k, ch in enumerate(CH):
        p, pd = trrs(x0 + k * 96, y0, f"{ch} {side}")
        out += p; pads.append(pd)
    bus5, busg = y0 + 100, y0 + 114
    xl, xr = x0 + 4, x0 + 4 * 96 - 8
    out.append(W(f"M{xl} {bus5} H{xr}", "w5")); out.append(T(xl - 6, bus5 + 4, "+5 V", "pin8 b t5", "end"))
    out.append(W(f"M{xl} {busg} H{xr}", "wg")); out.append(T(xl - 6, busg + 4, "GND", "pin8 b", "end"))
    rj, pin = rj45_row(x0 + 100, y0 + 186, f"hub RJ45 {side}",
                       {1: "fe", 2: "fe", 3: "fe", 6: "fe", 4: "fg", 8: "fg", 7: "f5", 5: "fs" if side == "R" else ""},
                       title_above=False)
    out += rj
    for k, pd in enumerate(pads):
        (tx, ty), (rx, ry), (sx, sy) = pd["T"], pd["R"], pd["S"]
        out.append(W(f"M{rx} {ry} V{bus5}", "w5")); out.append(dot(rx, bus5, "f5"))
        out.append(W(f"M{sx} {sy} V{busg}", "wg")); out.append(dot(sx, busg, "fg"))
        px, py = pin(SIGPIN[k])
        midy = y0 + 132 + 10 * (3 - k)
        out.append(W(f"M{tx} {ty} V{midy} H{px} V{py}", "we"))
    return out, pin, (xl, bus5), (xl, busg), (xr, bus5), (xr, busg)


def fig2():
    b = []
    HX, HRY, HLY = 150, 200, 560                     # hub breakouts start x, hub R y, hub L y
    hr, pinR, r5l, rgl, r5r, rgr = hub(HX, HRY, "R", True)
    hl, pinL, l5l, lgl, l5r, lgr = hub(HX, HLY, "L", False)
    b += hr + hl
    # Teensy stubs (from figure 1)
    b.append(C(36, 40, 7.5, "pad fg")); b.append(T(50, 32, "Teensy GND (top row, next to VIN)", "pin8 b"))
    b.append(C(36, 70, 7.5, "pad f5")); b.append(T(50, 62, "Teensy VIN (5 V out)", "pin8 b t5"))
    b.append(C(36, 100, 7.5, "pad fs")); b.append(T(50, 92, "Teensy pin 2 (SYNC)", "pin8 b ts"))
    # W1 (+5 V) and W2 (GND) in hub R
    w1, w1c = wago(650, 220, "W1 · +5 V (hub R)", ["VIN", "bus", "R7", "", ""], "f5")
    w2, w2c = wago(680, 300, "W2 · GND (hub R)", ["bus", "R4", "R8", "", "GND"], "fg")
    b += w1 + w2
    x, y = w1c(1); b.append(W(f"M36 70 H{x} V{y}", "w5"))
    x, y = w2c(5); b.append(W(f"M36 40 H{x} V{y}", "wg"))
    x, y = w1c(2); b.append(W(f"M{r5r[0]} {r5r[1]} H{x} V{y + 30}", "w5"))
    x, y = w2c(1); b.append(W(f"M{rgr[0]} {rgr[1]} H{x} V{y}", "wg"))
    for (wc, k, pinno, cls, yy) in ((w2c, 2, 4, "wg", 430), (w1c, 3, 7, "w5", 438), (w2c, 3, 8, "wg", 446)):
        x, y = wc(k); px, py = pinR(pinno)
        b.append(W(f"M{x} {y + 30} V{yy} H{px} V{py + 30}", cls))
    # SYNC: pin 2 -> 100 ohm -> RJ45 R pin 5
    px, py = pinR(5)
    b.append(W(f"M36 100 H60 V462 H{px - 44}", "ws"))
    b.append(R(px - 44, 456, 28, 12, "res", 2)); b.append(T(px - 30, 484, "100 Ω", "pin8", "middle"))
    b.append(W(f"M{px - 16} 462 H{px} V{py + 30}", "ws"))
    # hub L: buses -> RJ45 L pins 7 (+5 V) and 4, 8 (GND); pin 5 free
    for (start, pinno, cls, yy, xo) in ((l5r, 7, "w5", 798, 34), (lgr, 4, "wg", 790, 22), (lgr, 8, "wg", 806, 46)):
        px, py = pinL(pinno)
        b.append(W(f"M{start[0]} {start[1]} H{start[0] + xo} V{yy} H{px} V{py + 30}", cls))
    px, py = pinL(5); b.append(T(px, py + 74, "pin 5 free (ECG later → AI 17)", "pin8", "middle"))
    # Cat6 cables
    xR, yR = pinR(8)
    b.append(f'<path class="cab" d="M{xR + 18} {yR + 15} H870 V160 H900"/>')
    b.append(T(880, 150, "Cat6 R · 10 m", "s", "end"))
    xL, yL = pinL(8)
    b.append(f'<path class="cab" d="M{xL + 18} {yL + 15} H860 V560 H900"/>')
    b.append(T(880, 550, "Cat6 L · 10 m", "s", "end"))
    # DAQ end
    usedR = {1: "fe", 2: "fe", 3: "fe", 6: "fe", 5: "fs", 4: "fg", 8: "fg", 7: "f5"}
    usedL = {1: "fe", 2: "fe", 3: "fe", 6: "fe", 4: "fg", 8: "fg", 7: "f5"}
    aR, prR = rj45_col(900, 80, "DAQ-end R", usedR)
    aL, prL = rj45_col(900, 480, "DAQ-end L", usedL)
    b += aR + aL
    b.append(R(1120, 50, 200, 800, "daq", 6)); b.append(T(1220, 36, "NI USB-6363 · RSE · ±5 V · ≥ 4 kS/s", "b", "middle"))

    def ni(y, label, cls):
        return [R(1124, y - 9, 22, 18, "lever", 2), C(1135, y, 4.5, "screw"), T(1154, y + 4, label, f"pin b {cls}".strip())]

    for k, ch in enumerate(CH):
        for (pr, base, side) in ((prR, 0, "R"), (prL, 4, "L")):
            x, y = pr(SIGPIN[k])
            b += ni(y, f"AI {base + k} · {ch} {side}", "te")
            b.append(W(f"M{x} {y} H1124", "we"))
    x, y = prR(5); b += ni(y, "AI 16 · SYNC", "ts"); b.append(W(f"M{x} {y} H1124", "ws"))
    x, y = prL(5); b += ni(y, "AI 17 · ECG later", "")
    # W3 GND star and W4 +5 V link at the DAQ end
    w3, w3c = wago(985, 330, "W3 · GND star", ["R4", "R8", "L4", "L8", "AI"], "fg")
    w4, w4c = wago(985, 730, "W4 · +5 V link", ["R7", "L7", "", "", ""], "f5")
    b += w3 + w4
    b += ni(400, "AI GND", "")
    for (pr, pinno, k) in ((prR, 4, 1), (prR, 8, 2)):
        x, y = pr(pinno); cx, cy = w3c(k)
        b.append(W(f"M{x} {y} H{940 + 6 * k} V{cy - 14 + 0 * k} H{cx} V{cy}", "wg"))
    for (pr, pinno, k) in ((prL, 4, 3), (prL, 8, 4)):
        x, y = pr(pinno); cx, cy = w3c(k)
        b.append(W(f"M{x} {y} H{944 + 6 * k} V{cy + 52 + 6 * k} H{cx} V{cy + 30}", "wg"))
    cx, cy = w3c(5); b.append(W(f"M{cx} {cy + 30} V400 H1124", "wg"))
    for (pr, k) in ((prR, 1), (prL, 2)):
        x, y = pr(7); cx, cy = w4c(k)
        b.append(W(f"M{x} {y} H{946 + 4 * k} V{cy - 16 - 6 * k} H{cx} V{cy}", "w5"))
    b.append(T(36, 880, "Each stack: Link Shield (PWR ON, OUTPUT RAW) → sensor → Cable Shield; TRS cable from the Link jack to its TRRS breakout. "
               "Tip = signal, ring = +5 V, sleeve = GND. RAW sits at ≈ 2.5 V; remove the offset in software.", "s"))
    return svg(1340, 900, "EMG wiring at pin level: two thigh hubs with four TRRS breakouts each, Cat6 R and L, DAQ-end WAGOs and NI USB-6363 terminals", b)


CSS = """
:root { --bg:#f6f7f6; --surface:#ffffff; --ink:#17201d; --muted:#5b6662; --line:#d5dcd9; --accent:#0b6870; --accent-soft:#e0eff0;
  --w3:#d23a3a; --w5:#db7f12; --wg:#27302d; --wd:#2b67d1; --wc:#b08a00; --we:#17915a; --ws:#8446cf; --plug:#3b4441; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg:#111615; --surface:#1a201e; --ink:#e5ebe8; --muted:#9ba8a3; --line:#2e3835; --accent:#52b8c1; --accent-soft:#15302f;
  --w3:#f26b6b; --w5:#f0a24a; --wg:#cfd6d3; --wd:#6c9df2; --wc:#e2bf3c; --we:#47c487; --ws:#b487f2; --plug:#8d9893; } }
:root[data-theme="dark"] { --bg:#111615; --surface:#1a201e; --ink:#e5ebe8; --muted:#9ba8a3; --line:#2e3835; --accent:#52b8c1; --accent-soft:#15302f;
  --w3:#f26b6b; --w5:#f0a24a; --wg:#cfd6d3; --wd:#6c9df2; --wc:#e2bf3c; --we:#47c487; --ws:#b487f2; --plug:#8d9893; }
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
svg text.tc { fill:var(--wc); } svg text.te { fill:var(--we); } svg text.ts { fill:var(--ws); }
.pcb { fill:var(--bg); stroke:var(--ink); stroke-width:1.4; } .part { fill:var(--surface); stroke:var(--muted); stroke-width:1.1; }
.part.fa { fill:var(--accent-soft); stroke:var(--accent); stroke-width:1.5; } .pad { fill:var(--surface); stroke:var(--ink); stroke-width:1.2; }
.zone { fill:none; stroke:var(--line); stroke-width:1.2; stroke-dasharray:6 4; }
.wago { fill:var(--surface); stroke:var(--muted); stroke-width:1.3; } .lever { fill:var(--line); stroke:var(--muted); stroke-width:1; }
.daq { fill:var(--bg); stroke:var(--ink); stroke-width:1.3; } .screw { fill:var(--surface); stroke:var(--muted); stroke-width:1; }
.res { fill:var(--surface); stroke:var(--wg); stroke-width:2; } .plug { fill:var(--plug); }
.f3 { fill:var(--w3); } .f5 { fill:var(--w5); } .fg { fill:var(--wg); } .fd { fill:var(--wd); } .fc { fill:var(--wc); } .fe { fill:var(--we); } .fs { fill:var(--ws); }
.pad.f3 { fill:var(--w3); } .pad.f5 { fill:var(--w5); } .pad.fg { fill:var(--wg); } .pad.fd { fill:var(--wd); } .pad.fc { fill:var(--wc); } .pad.fe { fill:var(--we); } .pad.fs { fill:var(--ws); }
.lever.f3, .lever.f5, .lever.fg, .lever.fe, .lever.fs { stroke:none; }
.lever.f5 { fill:var(--w5); } .lever.fg { fill:var(--wg); } .lever.fe { fill:var(--we); } .lever.fs { fill:var(--ws); }
.w { fill:none; stroke-width:2.4; stroke-linecap:round; stroke-linejoin:round; }
.w3 { stroke:var(--w3); } .w5 { stroke:var(--w5); } .wg { stroke:var(--wg); } .wd { stroke:var(--wd); } .wc { stroke:var(--wc); } .we { stroke:var(--we); } .ws { stroke:var(--ws); }
.qw { fill:none; stroke:var(--accent); stroke-width:5; stroke-linecap:round; stroke-linejoin:round; }
.cab { fill:none; stroke:var(--muted); stroke-width:7; stroke-linecap:round; stroke-linejoin:round; }
"""

LEGEND = ('<div class="legend"><span><i style="border-color:var(--w3)"></i>3.3 V</span><span><i style="border-color:var(--w5)"></i>+5 V</span>'
          '<span><i style="border-color:var(--wg)"></i>GND</span><span><i style="border-color:var(--wd)"></i>SDA</span>'
          '<span><i style="border-color:var(--wc)"></i>SCL</span><span><i style="border-color:var(--we)"></i>EMG signal</span>'
          '<span><i style="border-color:var(--ws)"></i>SYNC</span><span><i style="border-color:var(--accent);border-top-width:5px"></i>Qwiic cable</span>'
          '<span><i style="border-color:var(--muted);border-top-width:6px"></i>Cat6</span></div>')

page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pilot system wiring</title><style>{CSS}</style></head><body><main>
<h1>Pilot system wiring · pin level</h1>
<p>6 BNO086 on 2 muxes (3 each) · 8 MyoWare 2.0 in two thigh hubs · NI USB-6363 records EMG and SYNC. Pin map from the senior's pilot schematic (2026-11-19).</p>
{LEGEND}
<h2>1 · Teensy, IMU bus, button</h2>
<div class="dia">{fig1()}</div>
<h2>2 · EMG hubs, Cat6, DAQ end</h2>
<div class="dia">{fig2()}</div>
<p>Laptop on battery. Nothing on the participant is mains-powered, but the GND link to the DAQ remains: follow the lab's electrical-safety rule for the DAQ adapter.</p>
</main></body></html>
"""
OUT.write_text(page, encoding="utf-8")
print("wrote", OUT, len(page))
