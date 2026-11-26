"use strict";
// 1 · Connection check: one automatic test per device type (Board, DAQ, IMU, EMG). No manual steps.
// Each test measures for 3 s while calling connEval() (connections.js) every 250 ms and grades the
// final live states: ok -> pass, warn -> warn, bad -> fail, unk -> skip. Results are kept in LS "connTest".
// Uses app.js globals: $, S, LS, api, toast.
(function () {
  const TESTS = ["board", "daq", "imu", "emg"];
  const NAME = { board: "Board", daq: "DAQ", imu: "IMUs", emg: "EMG" };
  const GRADE = { ok: "pass", warn: "warn", bad: "fail", unk: "skip" };
  const MEASURE_MS = 3000;
  let running = null;                                  // test key while one runs
  const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const today = () => new Date().toDateString();
  const load = () => LS.get("connTest", {}) || {};

  async function measure(ms = MEASURE_MS, each) {
    const t0 = performance.now();
    let ev = window.connEval();
    while (performance.now() - t0 < ms) {
      await sleep(250);
      ev = window.connEval();
      if (each) each(ev);
    }
    return ev;
  }
  async function waitFor(cond, ms) {
    const t0 = performance.now();
    while (performance.now() - t0 < ms) { if (cond()) return true; await sleep(250); }
    return cond();
  }
  const row = (label, g, badge, fix) => ({ label, g, badge: badge || "", fix: fix || "" });
  const fromST = (ST, k, label) => {
    const st = ST[k];
    if (!st) return row(label, "skip", "—", "No evidence.");
    return row(label || st.title, GRADE[st.s] || "skip", st.badge, st.s === "ok" ? "" : st.fix);
  };

  // ---------------- tests ----------------
  async function testBoard() {
    const rows = [];
    const port = !!(S && S.port);
    rows.push(row("USB port", port ? "pass" : "fail", port ? S.port : "NO PORT",
      port ? "" : "No Teensy COM port: check the USB data cable; if Teensy Loader shows the board, press Reboot."));
    if (!port) return rows;
    const proto = S.links && S.links.proto;
    rows.push(row("Firmware protocol", proto === "sidlab" ? "pass" : "fail", proto || "unknown",
      proto === "sidlab" ? "" : "Flash the SID Lab firmware teensy_spine_imu."));
    const hz = [];
    await measure(MEASURE_MS, () => { if (S && S.rate_hz != null) hz.push(+S.rate_hz); });
    if (!hz.length || Math.max(...hz) < 1) {
      rows.push(row("Frame rate", "fail", "NO FRAMES", "Connected but no frame lines: reconnect the board."));
    } else {
      const mean = hz.reduce((a, b) => a + b, 0) / hz.length;
      const spread = (Math.max(...hz) - Math.min(...hz)) / mean;
      rows.push(row("Frame rate steady", spread < 0.1 ? "pass" : "warn", `${mean.toFixed(0)} Hz ±${(spread * 50).toFixed(1)} %`,
        spread < 0.1 ? "" : "Frame rate varies more than 10 %: USB hub or a busy PC? Use a direct USB port."));
    }
    return rows;
  }

  async function ensureDaq(rows) {
    const dq = (S && S.daq) || {};
    if (!(dq.devices || []).length && !dq.live && !dq.preview) {
      rows.push(row("NI USB-6363", "fail", "NOT FOUND", "No NI DAQ found: check its USB cable and power, and NI MAX."));
      return false;
    }
    rows.push(row("NI USB-6363 found", "pass", (dq.cfg && dq.cfg.device) || ""));
    if (!dq.preview_on && S.state !== "recording") {
      await api("/api/preview", { on: true });
      const on = await waitFor(() => S && S.daq && S.daq.preview_on && (S.daq.live || S.daq.preview), 8000);
      if (!on) { rows.push(row("DAQ preview", "fail", "NO START", "The DAQ preview did not start within 8 s: see the event log.")); return false; }
      rows.push(row("DAQ preview", "pass", "turned on"));
    }
    return true;
  }

  async function testDaq() {
    const rows = [];
    if (!(await ensureDaq(rows))) return rows;
    const ev = await measure();
    rows.push(fromST(ev.ST, "daq", "DAQ running"));
    rows.push(fromST(ev.ST, "sync", "SYNC · pin 2 → AI31 (59)"));
    rows.push(fromST(ev.ST, "data", "Data · pin 1 → P0.0 (65)"));
    rows.push(fromST(ev.ST, "gnd", "Ground → D GND (82)"));
    return rows;
  }

  async function testImu() {
    const rows = [];
    if (!S || !S.port || !((S.rate_hz || 0) > 1)) return [row("IMUs", "skip", "NO FRAMES", "Run the Board test first: no frames from the Teensy.")];
    if (S.state === "recording") return [row("IMUs", "skip", "RECORDING", "Stop the recording first.")];
    // re-scan: every IMU restarts and reports OK or why not (resets the heading: run before the jig step)
    const j = await api("/api/restart");
    rows.push(row("Re-scan IMUs", j && j.ok ? "pass" : "warn", j && j.ok ? "done" : "failed",
      j && j.ok ? "" : "The re-scan command failed; grading the current state."));
    await sleep(4000);
    const ev = await measure();
    rows.push(fromST(ev.ST, "busB", "Bus B · Wire → Mux B"));
    rows.push(fromST(ev.ST, "busA", "Bus A · Wire1 → Mux A"));
    for (const n of ev.ORDER) rows.push(fromST(ev.ST, "imu_" + n));
    return rows;
  }

  async function testEmg() {
    const rows = [];
    if (!(await ensureDaq(rows))) return rows;
    const ev = await measure();
    const ch = (S.emg && S.emg.chans) || [];
    const miss = ev.EMG.filter(([c]) => !ch.some((x) => x.chan === c)).map(([c]) => c.toUpperCase());
    rows.push(row("All EMG channels in the DAQ list", miss.length ? "fail" : "pass", miss.length ? `missing ${miss.join(", ")}` : `${ev.EMG.length}/${ev.EMG.length}`,
      miss.length ? "Add the missing channels in Settings → DAQ (EMG, RSE)." : ""));
    for (const [c] of ev.EMG) rows.push(fromST(ev.ST, "emg_" + c));
    return rows;
  }

  const RUN = { board: testBoard, daq: testDaq, imu: testImu, emg: testEmg };

  function verdict(rows) {
    if (rows.some((r) => r.g === "fail")) return "fail";
    if (rows.some((r) => r.g === "warn")) return "warn";
    if (!rows.length || rows.every((r) => r.g === "skip")) return "skip";
    return rows.some((r) => r.g === "skip") ? "warn" : "pass";
  }

  async function run(k) {
    if (running) { toast("A connection test is already running", true); return false; }
    if (!window.connEval) return false;
    running = k; render();
    let rows;
    try { rows = await RUN[k](); }
    catch (e) { rows = [row("Test error", "fail", "ERROR", String(e && e.message || e))]; }
    const all = load();
    all[k] = { v: verdict(rows), rows, when: Date.now(), day: today() };
    LS.set("connTest", all);
    running = null; render();
    return true;
  }
  async function runAll() {
    for (const k of TESTS) if (!(await run(k))) return;
    const r = load();
    const bad = TESTS.filter((k) => r[k] && r[k].v !== "pass").map((k) => NAME[k]);
    toast(bad.length ? `Connection check: ${bad.join(", ")} not passed` : "Connection check: all passed", bad.length ? true : undefined);
  }

  // ---------------- render ----------------
  const PILL = { pass: ["PASS", "ok"], warn: ["WARN", "warn"], fail: ["FAIL", "bad"], skip: ["SKIPPED", ""] };
  let lastHtml = {};
  function render() {
    const r = load();
    for (const k of TESTS) {
      const card = $("ct_" + k);
      if (!card) continue;
      const res = r[k], pill = card.querySelector(".ct-pill"), btn = card.querySelector(".ct-run");
      btn.disabled = !!running;
      btn.textContent = running === k ? "testing…" : res ? "↻ Test again" : "▶ Test";
      const [pt, pc] = running === k ? ["RUNNING", "warn"] : res ? PILL[res.v] : ["NOT RUN", ""];
      pill.textContent = pt; pill.className = "pill ct-pill " + pc;
      card.querySelector(".ct-when").textContent = res ? `${res.day === today() ? "today" : res.day} ${new Date(res.when).toLocaleTimeString()}` : "";
      const html = res ? res.rows.map((x) => `<li class="ct-${x.g}"><i class="ct-dot"></i><span class="ct-l">${esc(x.label)}</span>` +
        `<span class="ct-b">${esc(x.badge)}</span>${x.fix ? `<p class="ct-fix">${esc(x.fix)}</p>` : ""}</li>`).join("") : "";
      if (lastHtml[k] !== html) { lastHtml[k] = html; card.querySelector(".ct-list").innerHTML = html; }
    }
    const done = TESTS.filter((k) => r[k] && r[k].day === today());
    const passed = done.filter((k) => r[k].v === "pass").length;
    const all = passed === TESTS.length;
    const p = $("connAllPill");
    if (p) {
      p.textContent = running ? `RUNNING · ${NAME[running]}` : all ? "ALL PASSED" : done.length ? `${passed}/${TESTS.length} passed` : "NOT RUN";
      p.className = "pill " + (running ? "warn" : all ? "ok" : done.length ? "bad" : "");
    }
    if ($("btnConnAll")) $("btnConnAll").disabled = !!running;
    if (window.FlowNav) FlowNav.mark("conn", all);
  }
  window.connTestRender = render;

  for (const b of document.querySelectorAll(".ct-run")) b.onclick = () => run(b.dataset.test);
  if ($("btnConnAll")) $("btnConnAll").onclick = runAll;
  render();
})();
