"use strict";
// 3a · Connections: live pin-out of the SID Lab spine brace (teensy_spine_imu, handover 2026-11-25).
// Every wire and part of the real wiring is drawn pin by pin and coloured by what the live data says about it:
//   Teensy USB, Teensy -> DAQ (pin 1 data -> P0.0 / 65, pin 2 SYNC -> AI31 / 59, GND -> D GND / 82),
//   Wire (19/18) -> Mux B -> T11 L1 L3 S1, Wire1 (17/16) -> Mux A -> C7 T3 T6 T9, EMG 1-8 -> AI16-23.
// Evidence: frame lines (per pod ok / new data), the firmware's per-pod scan lines ('r') and I2C counters
// ("# Hz:" every 2 s), and the DAQ preview (SYNC pulses, P0.0 packets, EMG floating / clipping / RMS).
// Uses app.js globals: $, S, api, toast, LS; workflow.js calls connRender() on every status tick.
(function () {
  const NS = "http://www.w3.org/2000/svg";
  // ---- Teensy 4.1 seen from the top, USB up: pin rows as printed on the board ----
  const LEFT = ["GND", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "3V3",
    "24", "25", "26", "27", "28", "29", "30", "31", "32"];
  const RIGHT = ["VIN", "GND", "3V3", "23", "22", "21", "20", "19", "18", "17", "16", "15", "14", "13", "GND",
    "41", "40", "39", "38", "37", "36", "35", "34", "33"];
  const PY0 = 150, PITCH = 21, LX = 566, RX = 654, TB = [550, 670];     // pin rows, board x span
  const py = (i) => PY0 + i * PITCH;
  // used pins: side+index -> signal class (colour) and the link it belongs to
  const USED = {
    L0: ["gnd", "gnd"], L2: ["data", "data"], L3: ["sync", "sync"], L5: ["clk", "clk"], L14: ["v33", "busA"],
    R1: ["gnd", "busB"], R2: ["v33", "busB"], R7: ["scl", "busB"], R8: ["sda", "busB"],
    R9: ["sda", "busA"], R10: ["scl", "busA"], R14: ["gnd", "busA"],
  };
  const PODS_B = ["T11", "L1", "L3", "S1"], PODS_A = ["C7", "T3", "T6", "T9"];   // firmware order: A ch0-3, B ch0-3
  const ORDER = [...PODS_A, ...PODS_B];
  const CH_Y = { B: [270, 310, 350, 390], A: [480, 520, 560, 600] };
  const MUX = { B: { y0: 240, y1: 420, label: "Mux B", bus: "Wire · 19 / 18" }, A: { y0: 450, y1: 630, label: "Mux A", bus: "Wire1 · 17 / 16" } };
  const MX = [800, 910], IX = [1010, 1232];
  const DX = [250, 420];                                               // DAQ terminal block
  const DAQ_T = [[0, "82", "D GND", "gnd"], [2, "65", "P0.0", "data"], [3, "59", "AI31", "sync"], [5, "PFI", "ctr out", "clk"]];
  const EMG = [["ai16", 33], ["ai17", 36], ["ai18", 39], ["ai19", 42], ["ai20", 49], ["ai21", 52], ["ai22", 55], ["ai23", 58]];
  const EMG_Y = (k) => 290 + k * 46;
  const W = 1250, H = 700;

  let built = false, svg = null, sel = null, autoScan = false, lastProblems = "";
  const rate = { t: 0, packets: null, pulses: null, pk_s: null, pu_s: null, bad0: null, bad_s: null };
  const ST = {};                      // link / part key -> {s, badge, title, facts[], fix}

  // ---------------- drawing ----------------
  function el(tag, attrs, parent, text) {
    const e = document.createElementNS(NS, tag);
    for (const [k, v] of Object.entries(attrs || {})) e.setAttribute(k, v);
    if (text != null) e.textContent = text;
    if (parent) parent.appendChild(e);
    return e;
  }
  const P = (pts) => "M" + pts.map((p) => p.join(",")).join("L");
  function link(key, sig, pts, opt = {}) {          // one wire: status halo underneath, coloured wire on top
    const g = el("g", { class: "cx-lk", "data-k": key }, svg.querySelector("#cxWires"));
    el("path", { d: P(pts), class: "cx-halo" }, g);
    el("path", { d: P(pts), class: `cx-w w-${sig}${opt.cable ? " cable" : ""}${opt.dash ? " dash" : ""}` }, g);
    return g;
  }
  function part(key, x, y, w, h, cls) {
    const g = el("g", { class: "cx-part " + (cls || ""), "data-k": key }, svg.querySelector("#cxParts"));
    el("rect", { x, y, width: w, height: h, rx: 7, class: "cx-box" }, g);
    return g;
  }
  function txt(parent, x, y, s, cls, anchor) {
    return el("text", { x, y, class: cls || "", "text-anchor": anchor || "start" }, parent, s);
  }

  function build() {
    const host = $("connMap");
    host.innerHTML = "";
    svg = el("svg", { viewBox: `0 0 ${W} ${H}`, class: "cx-svg", role: "img", "aria-label": "Live wiring of the spine brace" }, host);
    el("g", { id: "cxWires" }, svg);
    el("g", { id: "cxParts" }, svg);
    el("g", { id: "cxPins" }, svg);
    const pins = svg.querySelector("#cxPins");

    // PC
    const pc = part("pc", 470, 14, 290, 40, "pc");
    txt(pc, 615, 39, "PC · this GUI", "cx-t", "middle");
    // Teensy
    const tn = part("teensy", TB[0], 112, TB[1] - TB[0], 548, "board");
    el("rect", { x: 592, y: 98, width: 36, height: 18, rx: 3, class: "cx-usb" }, tn);
    txt(tn, TB[0] + 60, 138, "Teensy 4.1", "cx-t", "middle");
    txt(tn, TB[0] + 60, 652, "pin 13 LED blinks = frames", "cx-s", "middle");
    for (const [side, names, x] of [["L", LEFT, LX], ["R", RIGHT, RX]]) {
      names.forEach((n, i) => {
        const u = USED[side + i];
        el("circle", { cx: x, cy: py(i), r: 5, class: u ? `cx-pin used w-${u[0]}` : "cx-pin" }, pins);
        txt(pins, side === "L" ? x + 10 : x - 10, py(i) + 3.5, n, "cx-pl" + (u ? " b" : ""), side === "L" ? "start" : "end");
      });
    }
    link("usb", "usb", [[610, 98], [610, 54]], { cable: true });
    txt(svg.querySelector("#cxWires"), 618, 80, "USB", "cx-s");

    // DAQ
    const dq = part("daq", DX[0], 100, DX[1] - DX[0], 560, "daq");
    txt(dq, 335, 122, "NI USB-6363 · DAQ", "cx-t", "middle");
    txt(dq, 335, 137, "screw terminals", "cx-s", "middle");
    link("daq_usb", "usb", [[335, 100], [335, 34], [470, 34]], { cable: true });
    for (const [i, t, name, sig] of DAQ_T) {
      const y = py(i);
      el("rect", { x: DX[1] - 7, y: y - 5, width: 10, height: 10, rx: 2, class: `cx-term w-${sig}` }, pins);
      txt(dq, DX[1] - 12, y + 3.5, `${t}  ${name}`, "cx-pl b", "end");
    }
    // Teensy -> DAQ wires (straight across: the terminals sit at the pin rows)
    link("gnd", "gnd", [[LX, py(0)], [DX[1], py(0)]]);
    link("data", "data", [[LX, py(2)], [DX[1], py(2)]]);
    link("sync", "sync", [[LX, py(3)], [DX[1], py(3)]]);
    link("clk", "clk", [[DX[1], py(5)], [LX, py(5)]], { dash: true });
    const wl = svg.querySelector("#cxWires");
    txt(wl, 485, py(0) - 5, "common GND", "cx-s", "middle");
    txt(wl, 485, py(2) - 5, "IMU data 230400 Bd", "cx-s", "middle");
    txt(wl, 485, py(3) + 13, "SYNC 1 pulse / frame", "cx-s", "middle");
    txt(wl, 485, py(5) + 13, "optional · pin 4", "cx-s", "middle");

    // EMG -> DAQ AI16..23
    EMG.forEach(([ch, term], k) => {
      const y = EMG_Y(k);
      el("rect", { x: DX[0] - 3, y: y - 5, width: 10, height: 10, rx: 2, class: `cx-term w-e${k}` }, pins);
      txt(dq, DX[0] + 12, y + 3.5, `${term}  ${ch.toUpperCase()}`, "cx-pl b");
      link("emg_" + ch, "e" + k, [[170, y], [DX[0], y]]);
      const g = part("emg_" + ch, 22, y - 19, 148, 38, "sensor");
      el("circle", { cx: 170, cy: y, r: 4, class: `cx-pin used w-e${k}` }, pins);
      txt(g, 32, y - 3, `EMG${k + 1}`, "cx-t");
      txt(g, 32, y + 11, ch, "cx-s mono", "start").classList.add("cx-sub");
      txt(g, 162, y + 4, "—", "cx-badge", "end");
    });
    txt(wl, 96, EMG_Y(0) - 28, "EMG sensors · ref. to AI GND", "cx-s", "middle");

    // muxes, buses, Qwiic cables, IMUs
    for (const m of ["B", "A"]) {
      const M = MUX[m];
      const g = part("mux" + m, MX[0], M.y0, MX[1] - MX[0], M.y1 - M.y0, "mux");
      txt(g, MX[0] + 55, M.y0 + 52, M.label, "cx-t", "middle");
      txt(g, MX[0] + 55, M.y0 + 67, M.bus, "cx-s", "middle");
      txt(g, MX[0] + 55, M.y0 + 82, "0x70", "cx-s mono", "middle");
      txt(g, MX[0] + 55, M.y1 - 34, "—", "cx-badge", "middle");
      CH_Y[m].forEach((y, c) => {
        const name = (m === "A" ? PODS_A : PODS_B)[c];
        el("rect", { x: MX[1] - 6, y: y - 7, width: 12, height: 14, rx: 2, class: "cx-sock" }, pins);
        txt(g, MX[1] - 10, y + 3.5, "ch" + c, "cx-pl", "end");
        link("q_" + name, "qwiic", [[MX[1], y], [IX[0], y]], { cable: true });
        el("rect", { x: IX[0] - 6, y: y - 7, width: 12, height: 14, rx: 2, class: "cx-sock" }, pins);
        const n = ORDER.indexOf(name) + 1;
        const ig = part("imu_" + name, IX[0], y - 16, IX[1] - IX[0], 32, "sensor");
        txt(ig, IX[0] + 12, y + 4, `IMU${n} · ${name}`, "cx-t");
        txt(ig, IX[0] + 98, y + 4, "0x4B", "cx-s mono");
        txt(ig, IX[1] - 10, y + 4, "—", "cx-badge", "end");
      });
    }
    // Mux B pads (top edge: 3V3, GND; left edge: SCL, SDA) and wires from the right pin row
    const pad = (x, y, sig) => el("rect", { x: x - 4, y: y - 4, width: 8, height: 8, rx: 1.5, class: `cx-term w-${sig}` }, pins);
    const mB = MUX.B, mA = MUX.A;
    pad(826, mB.y0, "v33"); pad(846, mB.y0, "gnd"); pad(MX[0], py(7), "scl"); pad(MX[0], py(8), "sda");
    txt(svg.querySelector("#cxPins"), 836, mB.y0 + 16, "3V3 GND", "cx-pl", "middle");
    txt(svg.querySelector("#cxPins"), MX[0] + 8, py(7) + 3.5, "SCL", "cx-pl");
    txt(svg.querySelector("#cxPins"), MX[0] + 8, py(8) + 3.5, "SDA", "cx-pl");
    link("busB", "v33", [[RX, py(2)], [826, py(2)], [826, mB.y0]]);
    link("busB", "gnd", [[RX, py(1)], [846, py(1)], [846, mB.y0]]);
    link("busB", "scl", [[RX, py(7)], [MX[0], py(7)]]);
    link("busB", "sda", [[RX, py(8)], [MX[0], py(8)]]);
    txt(wl, 735, py(7) - 6, "bus B", "cx-s", "middle");
    // Mux A pads (left edge: SDA, SCL; bottom edge: GND, 3V3)
    pad(MX[0], 480, "sda"); pad(MX[0], 500, "scl"); pad(826, mA.y1, "gnd"); pad(846, mA.y1, "v33");
    txt(svg.querySelector("#cxPins"), MX[0] + 8, 483.5, "SDA", "cx-pl");
    txt(svg.querySelector("#cxPins"), MX[0] + 8, 503.5, "SCL", "cx-pl");
    txt(svg.querySelector("#cxPins"), 836, mA.y1 - 10, "GND 3V3", "cx-pl", "middle");
    link("busA", "sda", [[RX, py(9)], [712, py(9)], [712, 480], [MX[0], 480]]);
    link("busA", "scl", [[RX, py(10)], [696, py(10)], [696, 500], [MX[0], 500]]);
    link("busA", "gnd", [[RX, py(14)], [680, py(14)], [680, 650], [826, 650], [826, mA.y1]]);
    link("busA", "v33", [[LX, py(14)], [536, py(14)], [536, 670], [846, 670], [846, mA.y1]]);
    txt(wl, 756, 474, "bus A", "cx-s", "middle");
    txt(wl, IX[0] - 50, CH_Y.B[0] - 14, "Qwiic", "cx-s", "middle");

    svg.addEventListener("click", (e) => {
      const g = e.target.closest("[data-k]");
      select(g ? g.dataset.k : null);
    });
    built = true;
  }

  // ---------------- live state ----------------
  const set = (k, s, badge, title, facts, fix) => { ST[k] = { s, badge, title, facts: facts || [], fix: fix || "" }; };
  const podBus = (pd, i) => (pd.where && pd.where[0]) || (i < 4 ? "A" : "B");
  const podCh = (pd, i) => (pd.where ? +pd.where.slice(1) : i % 4);
  const ago = (t, now) => (t ? Math.max(0, now - t).toFixed(0) + " s ago" : "not seen");

  function compute() {
    for (const k of Object.keys(ST)) delete ST[k];
    const s = S || {};
    const links = s.links || {}, now = links.now || Date.now() / 1000;
    const conn = !!s.port, sid = links.proto === "sidlab", frames = (s.rate_hz || 0) > 1;
    // PC <-> Teensy
    if (!conn) set("usb", "bad", "NO PORT", "Teensy USB", [`server state: ${s.state || "no server"}`],
      "No Teensy COM port. Board in bootloader mode (Teensy Loader shows it, no COM port)? Press Reboot in Teensy Loader. " +
      "Otherwise check the USB cable (data cable, not charge-only) and that no other program (Arduino serial monitor) has the port.");
    else if (!frames) set("usb", "warn", "NO FRAMES", "Teensy USB", [`port ${s.port}`, `state ${s.state}`],
      "Connected but no frame lines: frame clock stopped ('p') or frame lines off ('q'). Reconnect, or press Re-scan.");
    else set("usb", "ok", `${s.rate_hz.toFixed(0)} Hz`, "Teensy USB", [`port ${s.port}`, `${s.rate_hz.toFixed(1)} frames/s`,
      `firmware ${(s.hello && s.hello.fw) || "?"}`]);
    ST.teensy = { ...ST.usb, title: "Teensy 4.1" };
    ST.pc = { s: "ok", badge: "", title: "PC", facts: [`GUI ${s.app_version || ""}`], fix: "" };
    if (conn && !sid && links.proto) set("teensy", "warn", links.proto, "Teensy 4.1", [`firmware protocol ${links.proto}`],
      "This drawing is the SID Lab brace wiring (teensy_spine_imu). The board runs another firmware.");

    // IMUs
    const pods = (s.pods || []);
    const byName = Object.fromEntries(pods.map((pd, i) => [pd.name, [pd, i]]));
    const bus = links.bus || {};
    const busPods = { A: [], B: [] };
    ORDER.forEach((name, j) => {
      const hit = byName[name];
      const pd = hit ? hit[0] : null, i = hit ? hit[1] : j;
      const b = pd ? podBus(pd, i) : (j < 4 ? "A" : "B"), ch = pd ? podCh(pd, i) : j % 4;
      busPods[b].push(pd);
      const where = `mux ${b} ch${ch}`;
      const facts = [where + " · 0x4B"];
      let st, badge, fix = "";
      if (!pd || !conn) { st = "unk"; badge = "—"; facts.push(conn ? "not in the firmware's column header" : "board not connected"); }
      else {
        const d = pd.diag;
        if (pd.hz != null) facts.push(`report rate ${(+pd.hz).toFixed(1)} Hz (firmware, 2 s)`);
        facts.push(`new data in ${pd.fresh_recent.toFixed(0)} % of the last 100 frames`);
        if (pd.silent_s != null) facts.push(`last new quaternion ${pd.silent_s.toFixed(2)} s ago`);
        if (d) facts.push(`firmware: “${d.text}” (${ago(d.t, now)})`);
        if (pd.ok && pd.fresh_recent >= 90 && !(pd.silent_s > 0.5)) { st = "ok"; badge = pd.hz ? `${(+pd.hz).toFixed(0)} Hz` : "OK"; }
        else if (pd.ok && !(pd.silent_s > 0.5)) {
          st = "warn"; badge = `${pd.fresh_recent.toFixed(0)} %`;
          fix = `Data arrive but some frames repeat the old value: I2C errors on bus ${b}? Press the Qwiic plugs on ${where} and on ${name}; keep the cable away from the EMG leads.`;
        } else {
          st = "bad";
          const ds = d && d.state;
          badge = ds === "no_imu" ? "NOT FOUND" : ds === "no_quat" ? "NO DATA" : ds === "lost" ? "LOST" : ds === "no_mux" ? "NO MUX" : pd.ok ? "STALLED" : "NOT FOUND";
          fix = ds === "no_mux" ? `Mux ${b} itself does not answer: see the bus ${b} wires.`
            : ds === "no_quat" ? `${name} answers at 0x4B but sends no quaternion: unplug and replug its Qwiic cable (power-cycles the BNO086), then Re-scan.`
            : ds === "lost" ? `${name} stopped answering while running: loose Qwiic plug on ${where} or at the IMU. The firmware retries every 0.2 s.`
            : `No BNO086 at 0x4B behind ${where}: check the Qwiic cable ${where} → ${name} (both plugs fully in), the cable itself (swap with a neighbour), and the IMU's ADR jumper (must be open = 0x4B). Then Re-scan.`;
          if (!d) fix += " The firmware's reason appears after a Re-scan.";
        }
      }
      set("imu_" + name, st, badge, `IMU${j + 1} · ${name}`, facts, fix);
      // the cable shares the IMU's state except when the mux / bus is the problem
      ST["q_" + name] = { ...ST["imu_" + name], title: `Qwiic cable ${where} → ${name}` };
    });
    for (const b of ["A", "B"]) {
      const bi = bus[b] || {}, ps = busPods[b].filter(Boolean);
      const okN = ps.filter((pd) => pd.ok).length;
      const pins = b === "B" ? "19 SCL / 18 SDA (Wire)" : "16 SCL1 / 17 SDA1 (Wire1)";
      const pw = b === "B" ? "3V3 / GND from the right pin row (VIN side)" : "3V3 from the left pin row, GND from the right row";
      const facts = [`Teensy pins ${pins}`, pw, `${okN} of ${ps.length || 4} IMUs answering`];
      if (bi.errors != null) facts.push(`I2C errors ${bi.errors} total, +${bi.err_new || 0} in the last 2 s · bus busy ${bi.busy_pct} % · read ${bi.read_us} µs`);
      if (bi.mux) facts.push(`mux at last scan: ${bi.mux}`);
      const paused = bi.paused_t && now - bi.paused_t < 6;
      if (paused) facts.push(`firmware: “${bi.paused}”`);
      let st, badge, fix = "";
      if (!conn || !ps.length) { st = "unk"; badge = "—"; }
      else if (okN === 0 || bi.mux === "no answer at 0x70" || paused) {
        st = "bad"; badge = bi.mux === "no answer at 0x70" ? "NO MUX" : "BUS DOWN";
        fix = `No IMU answers on bus ${b}: the fault is between the Teensy and Mux ${b}. Check the 4 wires ${pins}, ${pw}, that SDA/SCL are not swapped, and the mux's address jumpers (0x70). Then Re-scan.`;
      } else if ((bi.err_new || 0) > 0) {
        st = "warn"; badge = `+${bi.err_new} err`;
        fix = `I2C errors are rising on bus ${b}: a contact is marginal. Press the plugs on the Teensy side and on Mux ${b}; shorten or re-route the cable.`;
      } else { st = "ok"; badge = `${okN}/${ps.length}`; }
      set("bus" + b, st, badge, `Bus ${b} · Teensy → Mux ${b}`, facts, fix);
      ST["mux" + b] = { ...ST["bus" + b], title: `Mux ${b} · 0x70` };
      if (st === "bad") for (const pd of ps) if (!pd.ok) {          // mux down: blame the bus, not every cable
        ST["q_" + pd.name] = { ...ST["q_" + pd.name], s: "unk", fix: `Bus ${b} is down: fix that first.` };
        ST["imu_" + pd.name] = { ...ST["imu_" + pd.name], badge: "BUS DOWN", fix: `Bus ${b} is down: fix that first.` };
      }
    }

    // DAQ
    const dq = s.daq || {}, lv = dq.live || dq.preview, devs = dq.devices || [];
    const dname = (dq.cfg && dq.cfg.device) || (devs[0] && (devs[0].name || devs[0])) || "";
    const dfacts = [dname ? `device ${dname}` : "no device chosen"];
    if (dq.preview_error) dfacts.push("error: " + dq.preview_error);
    if (lv && lv.error) dfacts.push("error: " + lv.error);
    if (!devs.length && !lv) set("daq", "bad", "NOT FOUND", "NI USB-6363", dfacts,
      "No NI DAQ found: check its USB cable and power LED, and that NI-DAQmx sees it (NI MAX).");
    else if (dq.preview_error || (lv && lv.error)) set("daq", "bad", "ERROR", "NI USB-6363", dfacts, "The DAQ reported an error: see the message; unplug / replug the DAQ USB, then Preview off / on.");
    else if (!lv) set("daq", "unk", "PREVIEW OFF", "NI USB-6363", dfacts, "Turn the DAQ preview on to check SYNC, the data line and EMG.");
    else set("daq", "ok", lv.running === false ? "IDLE" : "RUNNING", "NI USB-6363", dfacts.concat([`${(lv.seconds || 0).toFixed(0)} s sampled`]));
    ST.daq_usb = { ...ST.daq, title: "DAQ USB" };

    // rates of the DAQ counters (pulses, packets) over >= 1 s
    const tnow = performance.now();
    const dl = lv && lv.data_line;
    if (lv && tnow - rate.t >= 1000) {
      const dt = (tnow - rate.t) / 1000;
      if (rate.pulses != null && lv.pulses >= rate.pulses) rate.pu_s = (lv.pulses - rate.pulses) / dt;
      if (dl && rate.packets != null && dl.packets >= rate.packets) rate.pk_s = (dl.packets - rate.packets) / dt;
      if (dl && rate.bad0 != null && dl.bad >= rate.bad0) rate.bad_s = (dl.bad - rate.bad0) / dt;
      rate.t = tnow; rate.pulses = lv.pulses; rate.packets = dl ? dl.packets : null; rate.bad0 = dl ? dl.bad : null;
    }
    if (!lv) { rate.t = 0; rate.pulses = rate.packets = rate.bad0 = rate.pu_s = rate.pk_s = rate.bad_s = null; }
    const daqUp = !!lv && ST.daq.s === "ok";
    // SYNC: pin 2 -> AI31 (59)
    const sf = ["Teensy pin 2 → DAQ terminal 59 (AI31)"];
    if (lv) sf.push(`${lv.pulses} pulses seen · max level ${(+lv.sync_hi || 0).toFixed(2)} V (last 50 ms)`);
    if (rate.pu_s != null) sf.push(`${rate.pu_s.toFixed(0)} pulses/s`);
    if (!daqUp) set("sync", "unk", "—", "SYNC wire", sf, "Needs the DAQ preview.");
    else if (!frames) set("sync", "unk", "—", "SYNC wire", sf, "No frames from the Teensy, so no SYNC pulses expected.");
    else if (rate.pu_s == null) set("sync", "unk", "…", "SYNC wire", sf, "Measuring…");
    else if (rate.pu_s < 0.5 * (s.rate_hz || 120)) set("sync", "bad", rate.pu_s < 1 ? "NO PULSES" : "FEW PULSES", "SYNC wire", sf,
      "The Teensy makes frames but AI31 sees no (or too few) pulses: check the wire from Teensy pin 2 to terminal 59 (AI31), and the common GND (82). AI31 must be in the DAQ channel list as SYNC.");
    else set("sync", "ok", `${rate.pu_s.toFixed(0)}/s`, "SYNC wire", sf);
    // data line: pin 1 -> P0.0 (65)
    const df = ["Teensy pin 1 (TX1) → DAQ terminal 65 (P0.0), 230400 baud"];
    if (dl) df.push(`${dl.packets} packets · ${dl.bad} bad checksums` + (dl.silent_s != null ? ` · last ${dl.silent_s.toFixed(2)} s ago` : ""));
    if (dl && dl.error) df.push("error: " + dl.error);
    if (rate.pk_s != null) df.push(`${rate.pk_s.toFixed(0)} packets/s`);
    if (!daqUp) set("data", "unk", "—", "Data line", df, "Needs the DAQ preview.");
    else if (!dl) set("data", "unk", "OFF", "Data line", df, "The DAQ data line is not set (Settings → DAQ: data line port0/line0).");
    else if (dl.error) set("data", "bad", "ERROR", "Data line", df, "The DAQ could not read P0.0: see the error.");
    else if (!frames) set("data", "unk", "—", "Data line", df, "No frames from the Teensy, so no packets expected.");
    else if (rate.pk_s == null) set("data", "unk", "…", "Data line", df, "Measuring…");
    else if (rate.pk_s < 0.5 * (s.rate_hz || 120)) set("data", "bad", rate.pk_s < 1 ? "NO PACKETS" : "FEW PACKETS", "Data line", df,
      "The Teensy makes frames but no packets decode on P0.0: check the wire from Teensy pin 1 (TX1, not RX1 = pin 0) to terminal 65 (P0.0), and the common GND (terminal 82).");
    else if ((rate.bad_s || 0) > 0.05 * rate.pk_s) set("data", "warn", "BAD CRC", "Data line", df,
      "Packets arrive but many fail the checksum: the ground is poor or the wire picks up noise. Check GND (82) and keep the data wire short / twisted with GND.");
    else set("data", "ok", `${rate.pk_s.toFixed(0)}/s`, "Data line", df);
    // GND: Teensy GND -> D GND (82): seen through the digital line and SYNC
    const gf = ["Teensy GND (top left) → DAQ terminal 82 (D GND)", "not measured directly: judged from the data line and SYNC"];
    if (ST.data.s === "ok" && ST.sync.s === "ok") set("gnd", "ok", "OK", "Common ground", gf);
    else if (ST.data.s === "warn") set("gnd", "warn", "?", "Common ground", gf, "Bad checksums on P0.0 often mean a poor ground: check terminal 82.");
    else if (ST.data.s === "bad" && ST.sync.s === "bad") set("gnd", "bad", "?", "Common ground", gf,
      "Neither SYNC nor the data line works: the common ground (terminal 82) is the shared suspect, after the DAQ cable itself.");
    else set("gnd", "unk", "—", "Common ground", gf);
    set("clk", "unk", "unused", "Frame clock in (optional)", ["Teensy pin 4 ← DAQ counter (PFI)", "only with FRAME_CLOCK 1; this firmware makes its own 120 Hz frames"]);

    // EMG
    const ch = (s.emg && s.emg.chans) || [];
    EMG.forEach(([c, term], k) => {
      const e = ch.find((x) => x.chan === c);
      const facts = [`DAQ terminal ${term} (${c.toUpperCase()}), single-ended (RSE)`];
      let st, badge, fix = "";
      if (!e) { st = "unk"; badge = "—"; facts.push(daqUp ? "not in the DAQ channel list" : "DAQ preview off"); }
      else {
        facts.unshift(`${e.name} · ${e.group || ""} ${e.muscle || ""}`.trim());
        if (e.rms_mv != null) facts.push(`RMS ${e.rms_mv.toFixed(3)} mV (last 0.1 s)`);
        if (e.sync_corr != null) facts.push(`follows SYNC: r = ${e.sync_corr} (floating inputs copy AI31)`);
        if (e.floating) { st = "bad"; badge = "OPEN"; fix = `Input ${c.toUpperCase()} is floating: nothing drives terminal ${term}. Check the sensor's signal wire into terminal ${term}, its power, and its ground to AI GND.`; }
        else if (e.clip) { st = "bad"; badge = "CLIP"; fix = `${e.name} hits the ±${s.emg.range_v} V range: sensor gain too high, or the reference electrode is off.`; }
        else if (e.rms_mv != null && e.rms_mv < 0.2) { st = "warn"; badge = "FLAT"; fix = `${e.name} is almost flat: sensor off or not powered? (At rest a little noise is normal.)`; }
        else { st = e.floating === null ? "ok" : "ok"; badge = e.rms_mv != null ? `${e.rms_mv.toFixed(2)} mV` : "OK"; }
        if (e.floating === null && st === "ok") facts.push("open-input test needs SYNC pulses on AI31");
      }
      set("emg_" + c, st, badge, `EMG${k + 1} → ${c.toUpperCase()} (${term})`, facts, fix);
    });
  }

  // ---------------- paint ----------------
  function paint() {
    for (const g of svg.querySelectorAll("[data-k]")) {
      const st = ST[g.dataset.k];
      const s = st ? st.s : "unk";
      g.classList.remove("st-ok", "st-warn", "st-bad", "st-unk");
      g.classList.add("st-" + s);
      g.classList.toggle("sel", g.dataset.k === sel);
      const b = g.querySelector(".cx-badge");
      if (b && st && b.textContent !== st.badge) b.textContent = st.badge || "—";
      let t = g.querySelector("title");
      if (!t) t = el("title", {}, g);
      const tt = st ? `${st.title}: ${st.badge || s}${st.fix ? "\n" + st.fix : ""}` : "";
      if (t.textContent !== tt) t.textContent = tt;
    }
    // summary + problem list
    const imu = ORDER.filter((n) => ST["imu_" + n] && ST["imu_" + n].s === "ok").length;
    const emg = EMG.filter(([c]) => ST["emg_" + c] && ST["emg_" + c].s === "ok").length;
    const daq = ["sync", "data", "gnd"].filter((k) => ST[k] && ST[k].s === "ok").length;
    const all = imu === 8 && emg === EMG.length && daq === 3 && ST.usb.s === "ok";
    const pill = $("connPill");
    pill.textContent = all ? "ALL CONNECTED" : `IMU ${imu}/8 · EMG ${emg}/${EMG.length} · DAQ ${daq}/3`;
    pill.className = "pill " + (all ? "ok" : imu + emg + daq ? "warn" : "bad");
    if (window.FlowNav) FlowNav.mark("equip-conn", all);
    const order = ["usb", "busB", "busA", ...ORDER.map((n) => "imu_" + n), "daq", "sync", "data", "gnd", ...EMG.map(([c]) => "emg_" + c)];
    const probs = order.filter((k) => ST[k] && (ST[k].s === "bad" || ST[k].s === "warn") && !/BUS DOWN/.test(ST[k].badge));
    const key = probs.map((k) => k + ST[k].s + ST[k].badge).join("|");
    if (key !== lastProblems) {
      lastProblems = key;
      $("connProblems").innerHTML = probs.length ? probs.map((k) => {
        const st = ST[k];
        return `<li class="cx-prob ${st.s}" data-k="${k}"><b>${esc(st.title)}</b><span class="pill ${st.s === "bad" ? "bad" : "warn"}">${esc(st.badge)}</span>` +
          `<p>${esc(st.fix)}</p></li>`;
      }).join("") : `<li class="cx-none">${S && S.port ? "No problem found on any link." : "Waiting for the board…"}</li>`;
    }
    detail();
    // scan / status line
    const L = (S && S.links) || {};
    const sc = L.scan_t ? `last IMU scan ${ago(L.scan_t, L.now)}` : "no IMU scan seen since connecting";
    const sa = L.status_age_s != null ? ` · firmware status ${L.status_age_s.toFixed(0)} s old` : "";
    setText("connScan", sc + sa);
    const idle = S && S.state === "idle";
    $("btnConnScan").disabled = !idle || L.proto !== "sidlab";
    $("btnConnPreview").textContent = S && S.daq && S.daq.preview_on ? "DAQ preview: on" : "DAQ preview: off";
    // one automatic scan when the page opens on a connected, idle board (the start-up lines were missed)
    if (!autoScan && idle && L.proto === "sidlab" && !L.scan_t) { autoScan = true; scan(true); }
  }
  const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  function detail() {
    const box = $("connDetail");
    const st = sel && ST[sel];
    if (!st) { box.classList.add("hidden"); return; }
    box.classList.remove("hidden");
    const html = `<div class="cx-dh"><b>${esc(st.title)}</b><span class="pill ${st.s === "ok" ? "ok" : st.s === "unk" ? "" : st.s}">${esc(st.badge || st.s)}</span>` +
      `<button class="btn small ghost" id="connDetailX" title="Close">✕</button></div>` +
      `<ul>${st.facts.map((f) => `<li class="mono">${esc(f)}</li>`).join("")}</ul>` + (st.fix ? `<p class="cx-fix">${esc(st.fix)}</p>` : "");
    if (box.dataset.h !== html) { box.dataset.h = html; box.innerHTML = html; $("connDetailX").onclick = () => select(null); }
  }
  function select(k) { sel = k; if (svg) paint(); }
  function highlight(k, on) { if (svg) for (const g of svg.querySelectorAll(`[data-k="${k}"]`)) g.classList.toggle("hl", on); }

  async function scan(auto) {
    const j = await api("/api/restart");
    if (j.ok && !auto) toast("Re-scanning the IMUs: each pod reports OK or why not");
  }

  window.connRender = function () {
    if (!$("connMap")) return;
    if (!built) build();
    compute();
    paint();
  };
  function wire() {
    $("btnConnScan").onclick = () => scan(false);
    $("btnConnStatus").onclick = async () => { const j = await api("/api/podstatus"); if (j.ok) toast("Asked the firmware for its status (see the event log)"); };
    $("btnConnPreview").onclick = () => api("/api/preview", { on: !(S && S.daq && S.daq.preview_on) });
    const pl = $("connProblems");
    pl.addEventListener("mouseover", (e) => { const li = e.target.closest("[data-k]"); if (li) highlight(li.dataset.k, true); });
    pl.addEventListener("mouseout", (e) => { const li = e.target.closest("[data-k]"); if (li) highlight(li.dataset.k, false); });
    pl.addEventListener("click", (e) => { const li = e.target.closest("[data-k]"); if (li) select(li.dataset.k); });
  }
  wire();
})();
