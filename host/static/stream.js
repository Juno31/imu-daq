// openStream(): drop-in for `new EventSource("/api/stream")` that shares ONE connection between all
// IMU DAQ pages in this browser (stream-worker.js). Returns an object with onmessage({data}) /
// onerror() / onopen() handlers like EventSource. Falls back to a direct EventSource where
// SharedWorker is not available.
(function () {
  window.openStream = function () {
    const api = { onmessage: null, onerror: null, onopen: null, close() {} };
    let worker = null;
    try { if (window.SharedWorker) worker = new SharedWorker("/static/stream-worker.js", { name: "imudaq-stream" }); } catch { worker = null; }
    const direct = () => {                            // this page's own connection
      const es = new EventSource("/api/stream");
      es.onmessage = (ev) => api.onmessage && api.onmessage(ev);
      es.onerror = (ev) => api.onerror && api.onerror(ev);
      es.onopen = (ev) => api.onopen && api.onopen(ev);
      api.close = () => es.close();
      api.shared = false;
    };
    if (!worker) { direct(); return api; }
    const port = worker.port;
    let got = false, fellBack = false;
    const bye = () => { try { port.postMessage("bye"); } catch { } };
    // Some browsers (or embedded views) give workers no network access. If the worker cannot reach the
    // server while this page can, use a direct connection instead; if the server is down, keep waiting
    // on the worker (it retries by itself).
    const checkFallback = async () => {
      if (got || fellBack) return;
      try {
        const r = await fetch("/api/state", { cache: "no-store" });
        if (r.ok && !got && !fellBack) { fellBack = true; bye(); port.onmessage = null; direct(); }
      } catch { /* server down: the worker keeps retrying */ }
    };
    port.onmessage = (ev) => {
      const m = ev.data;
      if (m.t === "m") { got = true; api.onmessage && api.onmessage({ data: m.d }); }
      else if (m.t === "err") { api.onerror && api.onerror({}); checkFallback(); }
      else if (m.t === "open") api.onopen && api.onopen({});
    };
    worker.onerror = () => { api.onerror && api.onerror({}); checkFallback(); };
    port.start();
    setTimeout(checkFallback, 4000);                  // nothing at all after 4 s: check as well
    window.addEventListener("pagehide", () => { if (!fellBack) bye(); });
    api.close = () => { if (fellBack) return; bye(); };
    api.shared = true;
    return api;
  };
})();
