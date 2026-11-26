// Experiment workflow bar, shared by every page: 1 Session · 2 Participant · 3 Equipment · 4 MVC/RVC ·
// 5 Task recording · 6 Data review. Steps 1-3 and 5 are views of the main page (/#route), 4 and 6 are
// their own pages. Each page writes what is done into localStorage "imudaq.flow"; every page shows it.
(function () {
  const STEPS = [
    { key: "session", n: 1, label: "Session", href: "/#session" },
    { key: "participant", n: 2, label: "Participant", href: "/#participant" },
    { key: "equip", n: 3, label: "Equipment", href: "/#equip-conn",
      subs: [["equip-conn", "a", "Connections"], ["equip-jig", "b", "IMU jig"], ["equip-imu", "c", "IMU attach"],
        ["equip-emg", "d", "EMG attach"], ["equip-daq", "e", "DAQ & sync"]] },
    { key: "mvc", n: 4, label: "MVC / RVC", href: "/emg" },
    { key: "record", n: 5, label: "Task recording", href: "/#record" },
    { key: "review", n: 6, label: "Data review", href: "/review" },
  ];
  const read = () => { try { return JSON.parse(localStorage.getItem("imudaq.flow") || "{}") || {}; } catch { return {}; } };

  function stepState(st, done) {
    if (st.subs) {
      const n = st.subs.filter(([k]) => done[k]).length;
      return { done: n === st.subs.length, part: `${n}/${st.subs.length}` };
    }
    return { done: !!done[st.key], part: "" };
  }

  function render(el, current) {
    const done = read();
    const curStep = current.startsWith("equip") ? "equip" : current;
    el.innerHTML = STEPS.map((st) => {
      const s = stepState(st, done);
      const cls = ["fstep", s.done ? "done" : "", st.key === curStep ? "cur" : ""].join(" ").trim();
      return `<a class="${cls}" href="${st.href}" data-key="${st.key}" title="${st.label}${s.part ? " · " + s.part + " done" : ""}">` +
        `<span class="fn">${s.done ? "✓" : st.n}</span><span class="fl">${st.label}</span>` +
        (s.part && !s.done ? `<span class="fp">${s.part}</span>` : "") + `</a>`;
    }).join('<i class="fsep">›</i>');
  }

  // sub-tabs of step 3 (only on the main page)
  function renderSubs(el, current) {
    const done = read();
    el.innerHTML = STEPS[2].subs.map(([k, l, name]) =>
      `<a class="esub${k === current ? " cur" : ""}${done[k] ? " done" : ""}" href="/#${k}">` +
      `<span class="fn">${done[k] ? "✓" : "3" + l}</span>${name}</a>`).join("");
  }

  window.FlowNav = {
    STEPS,
    read,
    mark(key, on) {                       // set one step done / not done
      const d = read();
      if (!!d[key] === !!on) return;
      d[key] = !!on;
      try { localStorage.setItem("imudaq.flow", JSON.stringify(d)); } catch { }
      window.dispatchEvent(new Event("flowchange"));
    },
    mount(el, current, subsEl) {
      const draw = () => { render(el, window.FlowNav.current || current); if (subsEl) renderSubs(subsEl, window.FlowNav.current || current); };
      window.FlowNav.current = current;
      window.FlowNav.redraw = draw;
      draw();
      window.addEventListener("storage", (e) => { if (e.key === "imudaq.flow") draw(); });
      window.addEventListener("flowchange", draw);
    },
  };
})();
