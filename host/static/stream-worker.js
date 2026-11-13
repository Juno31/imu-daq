// Shared live-data connection for all IMU DAQ pages in this browser (SharedWorker).
//
// Browsers allow only 6 open HTTP/1.1 connections per host. Every page used to keep its own
// /api/stream (server-sent events) open, and the main page's embedded spine view a second one, so
// three or four tabs used them all up and the next page never finished loading. This worker keeps a
// single /api/stream and forwards each message to every page (see stream.js).
const ports = new Set();
const recent = [];            // [time, data] of the last few seconds, replayed to a page that joins later
const KEEP_MS = 10000;
let es = null;

function send(msg) {
  for (const p of ports) {
    try { p.postMessage(msg); } catch { ports.delete(p); }
  }
}

function connect() {
  es = new EventSource("/api/stream");
  es.onopen = () => send({ t: "open" });
  es.onmessage = (ev) => {
    const now = Date.now();
    recent.push([now, ev.data]);
    while (recent.length && now - recent[0][0] > KEEP_MS) recent.shift();
    send({ t: "m", d: ev.data });
  };
  es.onerror = () => send({ t: "err" });                // EventSource retries by itself
}

onconnect = (e) => {
  const p = e.ports[0];
  ports.add(p);
  p.onmessage = (ev) => {
    if (ev.data !== "bye") return;
    ports.delete(p);
    if (!ports.size && es) { es.close(); es = null; recent.length = 0; }   // last page closed
  };
  p.start();
  if (!es) connect();
  else for (const [, d] of recent) p.postMessage({ t: "m", d });   // catch up: recent frames, log and status
};
