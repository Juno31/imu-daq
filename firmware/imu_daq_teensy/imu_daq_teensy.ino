// =====================================================================
//  imu_daq firmware v2.0 (Teensy 4.1) - SID Lab spine IMU acquisition with DAQ time sync
//  6 x BNO086 on two TCA9548A muxes (3 per mux), two I2C buses read in parallel, one SYNC pulse per frame.
//  Same line protocol as v1.3 (firmware/imu_daq), so host/ works unchanged.
//
//  LIBRARY: teensy4_i2c by Richard Gemmell (non-blocking I2C on both buses), vendored in src/teensy4_i2c:
//           nothing to install.
//           Do NOT include <Wire.h> in this sketch: teensy4_i2c owns the I2C ports.
//           The SparkFun BNO080 library is not used; the small SHTP driver below replaces it.
//  BOARD:   Tools > Board > Teensyduino > Teensy 4.1, USB Type: Serial.
//
//  WIRING (bench guide Stage 1)
//    v2.4 (decided with the senior, 2026-11-19): 6 IMUs, 2 muxes, 2 I2C buses, 3 IMUs per mux.
//    Bus A = LPI2C1 "Wire":  SDA 18,  SCL 19  -> mux A (0x70) ch 0-2 = T1, T7, T12
//    Bus B = LPI2C3 "Wire1": SDA1 17, SCL1 16 -> mux B (0x70) ch 0-2 = L3, L5, S2
//    IMUs at 0x4B (0x4A also accepted). MUXED = false + a third bus = the daisy-chain variant (v2.3).
//    SYNC: pin 2 -> DAQ AI0 (AI0+), GND -> AI8 (AI0-)
//    DAQ CLOCK IN (optional, "clock ext"): DAQ counter output -> pin 4, DAQ GND -> Teensy GND.
//      Pin 4 takes 3.3 V at most: a 5 V TTL counter output needs a divider (e.g. 1k series, 2k to GND).
//
//  TIMING
//    A PIT timer (IntervalTimer) ticks every frame period ("clock pit", default), or each rising edge on
//    pin 4 from the DAQ's counter output is one frame ("clock ext"), so frames follow the DAQ clock. Each tick takes a snapshot of the newest
//    data of every IMU and, while recording, raises SYNC; a second timer lowers it after 1 ms
//    (3 ms for frame 0). The main loop runs one I2C state machine per bus: while bus A reads one IMU,
//    bus B reads another. Each IMU is polled shortly before its next report is due (BNO clock),
//    then every 0.5 ms until it arrives, so empty polls cost little bus time.
//    Frames wait in a 512-frame queue when USB is busy; only an overflowing queue loses lines.
//
//  BEHAVIOUR, SENSORS, OUTPUT LINES, COMMANDS: as v1.3 (see firmware/imu_daq/imu_daq.ino), plus
//    @HELLO  board=Teensy41 link=usb buses=A:18/19,B:17/16 pods=C7:A0,T3:A1,...   (pod:bus+channel)
//    @BUS name=.. busy_avg_ms=.. busy_max_ms=.. over_8ms=.. frames=.. errors=.. timeouts=.. recoveries=..  (in "status")
//    @STOP ... bus_a_avg_ms=.. bus_a_max_ms=.. bus_a_over8=.. (same for b)   I2C load while recording
//    @EVENT pod=.. type=stall reason=nak|silent|mux_nak|bus_timeout ...
//    command "scan": print what answers on each bus and mux channel (not while recording)
//    command "quiet": toggle the idle P lines off/on (to read status in the Serial Monitor)
//    t_us = micros() on the Teensy (wraps every 71.6 min; the host unwraps it).
// =====================================================================
#include <Arduino.h>
#include "src/teensy4_i2c/i2c_driver.h"
#include "src/teensy4_i2c/imx_rt1060/imx_rt1060_i2c_driver.h"
#include "imu_daq_types.h"
#include "sd_log.h"                    // one CSV per recording on the built-in SD card

#define FW_NAME    "imu_daq"
#define FW_VERSION "2.5.3"
const char* BOARD_NAME = "Teensy41";

// ---------------- pins / addresses ----------------
const int     SYNC_PIN = 2;
const int     CLK_PIN  = 4;            // DAQ clock input for "clock ext" (3.3 V max)
const int     LED_PIN  = 13;           // on-board LED, active HIGH
const uint8_t MUX_ADDR = 0x70;
const uint8_t IMU_ADDR = 0x4B;         // SparkFun BNO086 default
const uint8_t IMU_ADDR_ALT = 0x4A;     // ADR jumper closed (SparkFun) or Adafruit BNO085 default
const bool    MUXED = true;            // true: TCA9548A mux per bus (pod ch = channel); false: direct, ch 0/1 = 0x4B/0x4A

// ---------------- settings ----------------
uint32_t i2cHz               = 400000; // "khz 100" / "khz 400" changes it and restarts the IMUs
const uint32_t SYNC_US       = 1000;   // normal SYNC pulse
const uint32_t START_SYNC_US = 3000;   // frame-0 marker pulse
const uint32_t STALL_MS      = 500;    // live IMU silent this long -> restart it
const uint32_t RETRY_MS      = 500;    // between restart attempts of a missing IMU
const uint32_t BOOT_MS       = 100;    // after soft reset, before the first read
const uint32_t BOOT_NAK_MS   = 1000;   // NAKs tolerated this long after a reset
const uint32_t DRAIN_MIN_MS  = 300;    // read start-up messages at least this long
const uint32_t DRAIN_MAX_MS  = 1500;
const uint32_t ID_TIMEOUT_MS = 500;    // product-ID reply
const uint32_t SETUP_MAX_MS  = 3000;   // whole bring-up of one IMU
const uint32_t EN_GAP_MS     = 5;      // between "set feature" commands
const uint32_t OP_TIMEOUT_US = 20000;  // one I2C transfer (a 256-byte read takes ~6 ms)
const uint32_t POLL_RETRY_US = 1000;   // re-poll an IMU whose report has not arrived yet
const uint32_t NEXT_EARLY_US = 1000;   // after a report, poll again this much before the next one is due
const uint32_t START_POLL_US = 2000;   // between reads of an IMU that is still starting up
const uint32_t BUSY_LIMIT_US = 8000;   // "busy" frame threshold in the bus statistics
const uint16_t READ_LEN      = 56;     // bytes per read at start: header + one batch (ts 5 + quat 12 + acc/gyr/mag 10 each = 51)
const uint16_t READ_MIN = 24, READ_MAX = 64, READ_SPARE = 8;   // "readlen 0": read length follows the packet size
uint16_t fixedRead = READ_LEN;         // fixed 56-byte reads beat the adaptive length on a 4-IMU bus (93.7 vs ~99 Hz)
                                       // (each transfer costs extra time: the IMU stretches the clock)

uint16_t reportMs = 10;                // frame period in ms (10 ms = 100 Hz frames)
uint8_t  quatMode = 0;                 // 0 = game rotation vector, 1 = rotation vector (with mag)
const char* quatName() { return quatMode ? "rv" : "game"; }
uint32_t frameUs  = 10000;

#define NPODS 6
#define NBUS  2
#define RING  512                      // queued frames (5 s at 100 Hz)

// ---------------- buses ----------------
Bus buses[NBUS] = {
  {"A", &Master,  18, 19},
  {"B", &Master1, 17, 16},
};

// ---------------- pods ----------------
Pod pods[NPODS] = {
  {"T1", 0, 0}, {"T7", 0, 1}, {"T12", 0, 2},     // mux A (Wire) ch 0-2
  {"L3", 1, 0}, {"L5", 1, 1}, {"S2", 1, 2},      // mux B (Wire1) ch 0-2
};

// Shared between the main loop (writer) and the frame timer ISR (reader).
struct Latest { float q[4]; bool fresh; bool ok; float x[9]; uint8_t nb; uint8_t magAcc; };  // x: acc, gyr, mag
volatile Latest latest[NPODS];

#define NB_ACC 1
#define NB_GYR 2
#define NB_MAG 4
struct Snap { uint32_t num; uint32_t t; bool rec; uint8_t ok, fresh; float q[NPODS][4];
              float x[NPODS][9]; uint8_t nb[NPODS]; uint8_t magAcc[NPODS]; };
DMAMEM Snap ring[RING];                // ~190 KB in RAM2
volatile uint32_t ringHead = 0, ringTail = 0;

// Every sensor sample while recording ("S" lines), so none is lost when two arrive within one frame.
// Written and read by the main loop only (no ISR), so no locking is needed.
struct Sample { uint32_t t; uint8_t pod, id, seq, stat; float v[4]; };
#define SRING 4096                     // ~1.5 s of samples from 7 IMUs
DMAMEM Sample sring[SRING];            // ~96 KB in RAM2
uint32_t sHead = 0, sTail = 0, droppedS = 0;

IntervalTimer tickTimer, pulseTimer;

uint32_t curNum();

// ---------------- frame clock source ----------------
bool extClock = false;                 // false = PIT timer, true = rising edges on CLK_PIN ("clock pit|ext")
const uint32_t EXT_LOST_MS = 2000;     // no edge this long -> "clock_lost" event
volatile uint32_t extEdges = 0, extGlitches = 0, extLastUs = 0, extMinUs = 0xFFFFFFFF, extMaxUs = 0;
volatile uint64_t extSumUs = 0; volatile uint32_t extN = 0;

volatile bool     recording = false, startPending = false, startNoted = false;
volatile uint32_t frame = 0, seq = 0, recStartUs = 0, ticks = 0;
volatile uint32_t droppedF = 0, droppedP = 0;
uint32_t seenTicks = 0;

bool quietP = true;                    // idle P lines off until the host says "hello"; "quiet" toggles
char cmdBuf[64];
int  cmdLen = 0;

// ---------------- timer ISRs ----------------
void pulseEnd() { digitalWriteFast(SYNC_PIN, LOW); pulseTimer.end(); }

void onTick() {
  uint32_t t = micros();
  ticks++;
  if (startPending) {
    startPending = false; recording = true; startNoted = true;
    frame = 0; droppedF = 0; recStartUs = t;
  }
  bool rec = recording;
  uint32_t num = rec ? frame : seq;
  uint32_t next = (ringHead + 1) % RING;
  if (next != ringTail) {
    Snap& s = ring[ringHead];
    s.num = num; s.t = t; s.rec = rec; s.ok = 0; s.fresh = 0;
    for (int i = 0; i < NPODS; i++) {
      for (int k = 0; k < 4; k++) s.q[i][k] = latest[i].q[k];
      for (int k = 0; k < 9; k++) s.x[i][k] = latest[i].x[k];
      s.nb[i] = latest[i].nb; s.magAcc[i] = latest[i].magAcc;
      if (latest[i].ok) s.ok |= 1 << i;
      if (latest[i].fresh) s.fresh |= 1 << i;
      latest[i].fresh = false; latest[i].nb = 0;
    }
    ringHead = next;
  } else {
    for (int i = 0; i < NPODS; i++) { latest[i].fresh = false; latest[i].nb = 0; }
    if (rec) droppedF++; else droppedP++;
  }
  if (rec) {
    digitalWriteFast(SYNC_PIN, HIGH);
    pulseTimer.begin(pulseEnd, frame == 0 ? START_SYNC_US : SYNC_US);
    frame++;
  } else {
    seq++;
  }
}

// One DAQ clock edge = one frame. Edges closer than half a frame are noise and are ignored.
void onExtEdge() {
  uint32_t t = micros();
  if (extEdges && t - extLastUs < frameUs / 2) { extGlitches++; return; }
  if (extEdges) {
    uint32_t d = t - extLastUs;
    if (d < 4 * frameUs) {               // skip the gap after the DAQ was stopped
      extSumUs += d; extN++;
      if (d < extMinUs) extMinUs = d;
      if (d > extMaxUs) extMaxUs = d;
    }
  }
  extLastUs = t; extEdges++;
  onTick();
}

void resetExtStats() {
  noInterrupts();
  extGlitches = 0; extMinUs = 0xFFFFFFFF; extMaxUs = 0; extSumUs = 0; extN = 0;
  interrupts();
}

void startTicks() {
  tickTimer.end();
  detachInterrupt(digitalPinToInterrupt(CLK_PIN));
  if (extClock) {
    pinMode(CLK_PIN, INPUT_PULLDOWN);    // no DAQ connected: stays low, no false frames
    noInterrupts(); extEdges = 0; interrupts();
    resetExtStats();
    attachInterrupt(digitalPinToInterrupt(CLK_PIN), onExtEdge, RISING);
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 32); // same priority as the PIT tick: frame times stay regular
  } else {
    tickTimer.begin(onTick, frameUs);
  }
}

// "clock ext": report when the DAQ clock stops or starts again, and whether its period matches frame_us
void watchClock() {
  static bool lost = false;
  static uint32_t lastCheckMs = 0;
  if (!extClock) { lost = false; return; }
  if (millis() - lastCheckMs < 200) return;
  lastCheckMs = millis();
  noInterrupts();
  uint32_t edges = extEdges, last = extLastUs;
  interrupts();
  bool nowLost = !edges || micros() - last > EXT_LOST_MS * 1000;
  if (nowLost && !lost)
    Serial.printf("@EVENT pod=- type=clock_lost pin=%d no_edges_ms=%lu rec=%d frame=%lu\n", CLK_PIN,
                  (unsigned long)EXT_LOST_MS, recording, (unsigned long)curNum());
  else if (!nowLost && lost)
    Serial.printf("@EVENT pod=- type=clock_ok pin=%d rec=%d frame=%lu\n", CLK_PIN, recording,
                  (unsigned long)curNum());
  lost = nowLost;
}

// ---------------- helpers ----------------
uint16_t sensorOverrideMs = 0;         // "sensor N": IMU report interval in ms; 0 = automatic (= frame period)
uint32_t statsStartMs = 0;             // when the counters in "status" were last reset
uint16_t sensorMs() {
  if (sensorOverrideMs) return sensorOverrideMs;
  return reportMs;                     // 9 ms (90 %) made quat/gyro run at 150-180 Hz and overloaded the buses
}
bool magOn = false;                    // v2.5: off by default (quat + acc + gyro); "mag on": no magnetometer reports (the game rotation vector does not use them)
uint16_t magMs() { return magOn ? (uint16_t)max(18, 2 * (int)sensorMs()) : 0; }   // 0 = report off

bool sendNow(const char* s, int n) {
  if (Serial.availableForWrite() >= n) { Serial.write((const uint8_t*)s, n); return true; }
  return false;
}

void setLatestOk(int i, bool ok) {
  noInterrupts();
  latest[i].ok = ok;
  interrupts();
}

uint32_t curNum() { return recording ? frame : seq; }

void sayHello() {
  Serial.printf("@HELLO fw=%s ver=%s board=%s link=usb sda=18,17 scl=19,16 sync=%d report_ms=%u frame_us=%lu "
                "sensor_ms=%u mag_ms=%u quat=%s sensors=quat,acc,gyr,mag start_sync_us=%lu sync_us=%lu clock=%s clk_pin=%d "
                "samples=1 sd=%d buses=A:18/19,B:17/16 pods=",
                FW_NAME, FW_VERSION, BOARD_NAME, SYNC_PIN, reportMs, (unsigned long)frameUs, sensorMs(), magMs(),
                quatName(), (unsigned long)START_SYNC_US, (unsigned long)SYNC_US, extClock ? "ext" : "pit", CLK_PIN, sdOk);
  for (int i = 0; i < NPODS; i++)
    Serial.printf("%s%s:%s%u", i ? "," : "", pods[i].name, buses[pods[i].bus].name, pods[i].ch);
  Serial.printf(" rec=%d\n", recording);
  static char cols[2400];
  columnsLine(cols, sizeof cols);
  Serial.print(cols);
}

// "# columns: F,frame,t_us,..." line (with \r\n), for HELLO and the top of each SD file.
int columnsLine(char* b, int size) {
  int n = snprintf(b, size, "# columns: F,frame,t_us");
  for (int i = 0; i < NPODS; i++) {
    const char* p = pods[i].name;
    n += snprintf(b + n, size - n, ",%s_ok,%s_fresh,%s_w,%s_x,%s_y,%s_z", p, p, p, p, p, p);
  }
  for (int i = 0; i < NPODS; i++) {
    const char* p = pods[i].name;
    n += snprintf(b + n, size - n, ",%s_acc_new,%s_gyr_new,%s_mag_new,%s_ax,%s_ay,%s_az,%s_gx,%s_gy,%s_gz,%s_mx,%s_my,%s_mz,%s_mag_acc",
                  p, p, p, p, p, p, p, p, p, p, p, p, p);
  }
  n += snprintf(b + n, size - n, "\r\n");
  return n;
}

// ---------------- bus recovery ----------------
// If the Teensy resets (upload, reboot) in the middle of a read, an IMU can keep pulling SDA low while it
// waits for clocks that never come, and then nothing on that bus answers. Clocking SCL by hand until the
// IMU lets go, then sending a STOP, frees the bus (standard I2C recovery). begin() then re-muxes the pins.
// Returns a bit mask of the lines still stuck low afterwards: 1 = SDA, 2 = SCL.
uint8_t resetBus(Bus& b) {
  b.m->end();
  pinMode(b.sda, INPUT_PULLUP);
  pinMode(b.scl, INPUT_PULLUP);
  delayMicroseconds(10);
  if (!digitalReadFast(b.sda) && digitalReadFast(b.scl)) {
    pinMode(b.scl, OUTPUT_OPENDRAIN);
    for (int i = 0; i < 18 && !digitalReadFast(b.sda); i++) {
      digitalWriteFast(b.scl, LOW);  delayMicroseconds(5);
      digitalWriteFast(b.scl, HIGH); delayMicroseconds(5);
    }
    pinMode(b.sda, OUTPUT_OPENDRAIN);            // STOP: SDA low -> high while SCL is high
    digitalWriteFast(b.sda, LOW);  delayMicroseconds(5);
    digitalWriteFast(b.sda, HIGH); delayMicroseconds(5);
    pinMode(b.sda, INPUT_PULLUP);
    pinMode(b.scl, INPUT_PULLUP);
    delayMicroseconds(10);
    b.recoveries++;
  }
  uint8_t stuck = (digitalReadFast(b.sda) ? 0 : 1) | (digitalReadFast(b.scl) ? 0 : 2);
  b.m->begin(i2cHz);
  return stuck;
}

// ---------------- blocking I2C (start-up, scan) ----------------
bool waitBus(Bus& b, uint32_t timeoutUs) {
  uint32_t t0 = micros();
  while (!b.m->finished()) {
    if (micros() - t0 > timeoutUs) { resetBus(b); b.timeouts++; return false; }
  }
  return !b.m->has_error();
}

bool probe(Bus& b, uint8_t addr) {
  static uint8_t dummy = 0;
  b.m->write_async(addr, &dummy, 0, true);
  return waitBus(b, 5000);
}

bool muxSelectNow(Bus& b, int ch) {          // ch < 0: all channels off
  b.tx[0] = ch < 0 ? 0 : (uint8_t)(1 << ch);
  b.m->write_async(MUX_ADDR, b.tx, 1, true);
  bool ok = waitBus(b, 5000);
  b.muxCh = (ok && ch >= 0) ? (uint8_t)ch : 0xFF;
  return ok;
}

// Stop both state machines (after the transfer in progress) so blocking calls can use the buses.
void quiesce() {
  for (Bus& b : buses) {
    if (b.op != OP_NONE) waitBus(b, OP_TIMEOUT_US);
    b.op = OP_NONE; b.cur = -1; b.muxCh = 0xFF;
  }
}

void scanLine(Bus& b, const char* label) {
  char out[160];
  int n = snprintf(out, sizeof out, "# bus %s %s:", b.name, label), found = 0;
  for (uint8_t a = 1; a < 127; a++)
    if (probe(b, a)) { n += snprintf(out + n, sizeof out - n, " 0x%02X", a); found++; }
  if (!found) snprintf(out + n, sizeof out - n, " none");
  Serial.println(out);
}

// Map of both buses. In "ch" lines the mux itself (0x70) always answers; ignore it.
void bootStep(const char* what);
void scanBuses() {
  for (Bus& b : buses) {
    char step[16]; snprintf(step, sizeof step, "scan bus %s", b.name); bootStep(step);
    uint32_t rec0 = b.recoveries;
    uint8_t stuck = resetBus(b);
    if (b.recoveries != rec0)
      Serial.printf("# bus %s: SDA was held low by a device - freed it with clock pulses\n", b.name);
    if (stuck)
      Serial.printf("# bus %s: %s%s still LOW - a device or a short is holding the line; unplug USB for 5 s\n",
                    b.name, (stuck & 1) ? "SDA " : "", (stuck & 2) ? "SCL " : "");
    if (!MUXED) {                                   // expect 0x4A and 0x4B
      scanLine(b, "main");
      continue;
    }
    if (!muxSelectNow(b, -1)) {
      Serial.printf("# bus %s (SDA %d, SCL %d): mux 0x70 not answering - check the Qwiic lead and the mux power LED\n",
                    b.name, b.sda, b.scl);
      scanLine(b, "main");
      continue;
    }
    scanLine(b, "main");
    for (int ch = 0; ch < 8; ch++) {
      bool used = false;
      for (Pod& p : pods) if (p.bus == (&b - buses) && p.ch == ch) used = true;
      if (!used && ch > 3) continue;
      muxSelectNow(b, ch);
      char label[8];
      snprintf(label, sizeof label, "ch%d", ch);
      scanLine(b, label);
    }
    muxSelectNow(b, -1);
  }
}

// "i2ctime": time plain reads on each bus, from the mux (which never holds the clock) and from one IMU.
// The effective clock (eff_khz) near 400 means the bus runs at full speed; much lower on the mux too means
// slow edges (weak pull-ups / cable capacitance); lower only on the IMU means the IMU stretches the clock.
void timeXfer(Bus& b, uint8_t addr, int n, int reps, const char* dev) {
  int ok = 0;
  uint32_t t0 = micros();
  for (int k = 0; k < reps; k++) {
    b.m->read_async(addr, b.rx, n, true);
    if (waitBus(b, OP_TIMEOUT_US)) ok++;
  }
  float us = (micros() - t0) / (float)reps;
  float bits = (n + 1) * 9 + 2;                      // address + data bytes, 9 clocks each, plus START/STOP
  Serial.printf("@I2CTIME bus=%s dev=%s bytes=%d us=%.0f eff_khz=%.0f ok=%d/%d\n", b.name, dev, n, us,
                bits * 1000.0f / us, ok, reps);
}

void i2cTime() {
  quiesce();
  for (Bus& b : buses) {
    if (MUXED && !muxSelectNow(b, -1)) { Serial.printf("@I2CTIME bus=%s mux not answering\n", b.name); continue; }
    if (MUXED) { timeXfer(b, MUX_ADDR, 1, 50, "mux"); timeXfer(b, MUX_ADDR, 32, 50, "mux"); }
    for (Pod& p : pods) {
      if (&buses[p.bus] != &b) continue;
      if (!MUXED || muxSelectNow(b, p.ch)) { timeXfer(b, p.addr, 8, 20, p.name); timeXfer(b, p.addr, 64, 20, p.name); }
      break;                                         // one IMU per bus is enough
    }
    b.muxCh = 0xFF;
  }
}

// ---------------- sensor reports ----------------
// Length of each input report (SH-2 reference manual 6.5); 0 = unknown -> stop parsing the packet.
uint8_t reportLen(uint8_t id) {
  switch (id) {
    case 0xFB: case 0xFA: return 5;
    case 0x01: case 0x02: case 0x03: case 0x04: case 0x06: return 10;
    case 0x08: return 12;
    case 0x05: case 0x09: return 14;
    case 0x07: case 0x0F: return 16;
    default: return 0;
  }
}
inline float qval(const uint8_t* b, int qpoint) { return (int16_t)(b[0] | (b[1] << 8)) * (1.0f / (1 << qpoint)); }

// Keep every sensor report of one input-report packet (cargo without the 4-byte SHTP header).
// Returns how many bytes were used: less than len when the last report is cut off (rest in a continuation).
int parseReports(int i, const uint8_t* d, int len) {
  Pod& p = pods[i];
  int o = 0;
  while (o < len) {
    uint8_t id = d[o], L = reportLen(id);
    if (!L) return len;                         // unknown report: drop the rest of the packet
    if (o + L > len) break;
    const uint8_t* r = d + o;
    bool isQ = (id == 0x08 && !quatMode) || (id == 0x05 && quatMode);
    if (recording && (isQ || id == 0x01 || id == 0x02 || id == 0x03)) {   // queue the sample for an S line
      uint32_t next = (sHead + 1) % SRING;
      if (next == sTail) droppedS++;
      else {
        Sample& s = sring[sHead];
        s.t = micros(); s.pod = i; s.id = id; s.seq = r[1]; s.stat = r[2] & 0x03;
        if (isQ) { s.v[0] = qval(r + 10, 14); s.v[1] = qval(r + 4, 14); s.v[2] = qval(r + 6, 14); s.v[3] = qval(r + 8, 14); }
        else {
          int qp = id == 0x01 ? 8 : id == 0x02 ? 9 : 4;
          s.v[0] = qval(r + 4, qp); s.v[1] = qval(r + 6, qp); s.v[2] = qval(r + 8, qp); s.v[3] = 0;
        }
        sHead = next;
      }
    }
    if (isQ) {
      float x = qval(r + 4, 14), y = qval(r + 6, 14), z = qval(r + 8, 14), w = qval(r + 10, 14);
      noInterrupts();
      latest[i].q[0] = w; latest[i].q[1] = x; latest[i].q[2] = y; latest[i].q[3] = z;
      latest[i].fresh = true;
      interrupts();
      p.qStat = r[2] & 0x03;
      if (id == 0x05) p.headAccRad = qval(r + 12, 12);
      p.updates++;
      p.lastFreshMs = millis();
    } else if (id == 0x01 || id == 0x02 || id == 0x03) {     // acc Q8 m/s^2, gyro Q9 rad/s, mag Q4 uT
      int qp = id == 0x01 ? 8 : id == 0x02 ? 9 : 4, k0 = (id - 1) * 3;
      float a = qval(r + 4, qp), b = qval(r + 6, qp), c = qval(r + 8, qp);
      noInterrupts();
      latest[i].x[k0] = a; latest[i].x[k0 + 1] = b; latest[i].x[k0 + 2] = c;
      latest[i].nb |= id == 0x01 ? NB_ACC : id == 0x02 ? NB_GYR : NB_MAG;
      if (id == 0x03) latest[i].magAcc = r[2] & 0x03;
      interrupts();
      if (id == 0x01) p.nAcc++; else if (id == 0x02) p.nGyr++; else p.nMag++;
    }
    o += L;
  }
  return o;
}

// One input-report packet (cargo d[0..n)). A packet longer than READ_LEN comes in pieces: the next read
// returns a continuation, whose cargo goes on where this one was cut. Join them so no report is lost.
void feedReports(int i, const uint8_t* d, int n, bool cont, bool more) {
  Pod& p = pods[i];
  static uint8_t buf[sizeof(p.tail) + RXMAX];
  const uint8_t* src = d;
  int len = n;
  if (cont) {
    if (!p.contOk) return;                      // the start of this packet was not read: skip it
    memcpy(buf, p.tail, p.tailLen);
    memcpy(buf + p.tailLen, d, n);
    src = buf; len = p.tailLen + n;
    p.conts++;
  }
  p.tailLen = 0; p.contOk = false;
  int left = len - parseReports(i, src, len);
  if (more && left <= (int)sizeof(p.tail)) {
    memcpy(p.tail, src + len - left, left);
    p.tailLen = left; p.contOk = true;
  }
}

// ---------------- pod life cycle ----------------
void podEvent(int i, const char* type, const char* reason) {
  Pod& p = pods[i];
  if (!strcmp(type, "stall"))
    Serial.printf("@EVENT pod=%s type=stall reason=%s silent_ms=%lu rec=%d frame=%lu "
                  "pkts=%lu,%lu,%lu,%lu,%lu,%lu empty=%lu last=%02X%02X%02X%02X:%02X%02X%02X%02X\n", p.name, reason,
                  (unsigned long)(millis() - p.lastFreshMs), recording, (unsigned long)curNum(),
                  (unsigned long)p.pktCh[0], (unsigned long)p.pktCh[1], (unsigned long)p.pktCh[2],
                  (unsigned long)p.pktCh[3], (unsigned long)p.pktCh[4], (unsigned long)p.pktCh[5],
                  (unsigned long)p.empties, p.last[0], p.last[1], p.last[2], p.last[3],
                  p.last[4], p.last[5], p.last[6], p.last[7]);
  else if (reason)
    Serial.printf("@EVENT pod=%s type=%s reason=%s where=%s%u\n", p.name, type, reason, buses[p.bus].name, p.ch);
  else
    Serial.printf("@EVENT pod=%s type=%s addr=0x%02X rec=%d frame=%lu\n", p.name, type, p.addr, recording,
                  (unsigned long)curNum());
}

void podFail(int i, const char* reason) {
  Pod& p = pods[i];
  uint32_t ms = millis();
  if (p.state == P_LIVE || p.reconfig) {
    p.stalls++;
    podEvent(i, "stall", reason);
    p.lastTryMs = ms - RETRY_MS;                 // retry right away
  } else {
    if (!p.failNoted || ms - p.failNoteMs > 5000) {        // repeat every 5 s while it keeps failing
      p.failNoted = true; p.failNoteMs = ms; podEvent(i, "restart_failed", reason);
    }
    p.lastTryMs = ms;
  }
  p.state = P_DOWN; p.reconfig = false;
}

void goLive(int i) {
  Pod& p = pods[i];
  bool quiet = p.reconfig, wasOk = p.ok;
  p.state = P_LIVE; p.reconfig = false; p.failNoted = false; p.ok = true;
  p.lastFreshMs = millis();
  p.nextPollUs = micros();
  memset(p.pktCh, 0, sizeof p.pktCh); p.empties = 0; memset(p.last, 0, sizeof p.last);
  p.reads = p.conts = 0; p.contOk = false; p.tailLen = 0; p.readLen = READ_LEN;
  setLatestOk(i, true);
  if (!quiet) {
    if (wasOk) p.restarts++;
    podEvent(i, wasOk ? "restarted" : "found", nullptr);
  }
}

void reconfigure(int i) {           // re-send the report settings (rate / mode change)
  Pod& p = pods[i];
  if (p.state != P_LIVE) return;
  p.state = P_ENABLE; p.reconfig = true; p.enStep = 0;
  p.tState = millis() - EN_GAP_MS; p.setupStartMs = millis();
}

bool isDue(Pod& p, uint32_t ms, uint32_t us) {
  switch (p.state) {
    case P_LIVE:   return (int32_t)(us - p.nextPollUs) >= 0;
    case P_DOWN:   return ms - p.lastTryMs >= RETRY_MS;
    case P_BOOT:   return ms - p.tState >= BOOT_MS;
    case P_ENABLE: return ms - p.tState >= EN_GAP_MS;
    case P_DRAIN: case P_IDWAIT: return (int32_t)(us - p.nextPollUs) >= 0;
    default:       return true;
  }
}

// ---------------- bus state machine ----------------
void startOp(Bus& b, BusOp op) { b.op = op; b.opStartUs = micros(); }

void opSelect(Bus& b, uint8_t ch) {
  b.tx[0] = (uint8_t)(1 << ch);
  b.m->write_async(MUX_ADDR, b.tx, 1, true);
  startOp(b, OP_SELECT);
}
// One read = SHTP header + cargo in a single transfer. Reading the 4-byte header first and the body right
// after returned all zeros on hardware (the IMU had not re-armed), losing packets, so we no longer split it.
// A packet longer than READ_LEN arrives as continuation packets in the next reads.
void opHdr(Bus& b) {
  Pod& p = pods[b.cur];
  b.want = p.state == P_LIVE && p.readLen ? p.readLen : READ_LEN;
  b.m->read_async(p.addr, b.rx, b.want, true);
  startOp(b, OP_BODY);
}
void opWrite(Bus& b, Pod& p, uint8_t chan, const uint8_t* payload, uint8_t n) {
  uint8_t len = n + 4;
  b.tx[0] = len; b.tx[1] = 0; b.tx[2] = chan; b.tx[3] = p.seq[chan]++;
  memcpy(b.tx + 4, payload, n);
  b.m->write_async(p.addr, b.tx, len, true);
  startOp(b, OP_WRITE);
}

void opSetFeature(Bus& b, Pod& p, uint8_t reportId, uint32_t intervalUs) {
  uint8_t c[17] = {0xFD, reportId, 0, 0, 0,
                   (uint8_t)intervalUs, (uint8_t)(intervalUs >> 8), (uint8_t)(intervalUs >> 16), (uint8_t)(intervalUs >> 24),
                   0, 0, 0, 0, 0, 0, 0, 0};
  opWrite(b, p, 2, c, sizeof c);
}

void endVisit(Bus& b) {
  if (b.cur >= 0) {
    Pod& p = pods[b.cur];
    if (p.state == P_LIVE) {
      uint32_t dt = micros() - b.visitStartUs;
      if (dt > p.maxPollUs) p.maxPollUs = dt;
    }
    b.rr = b.cur + 1;
  }
  b.cur = -1;
}

// First transfer of the pod's next step (mux channel already selected).
void podAction(Bus& b) {
  Pod& p = pods[b.cur];
  switch (p.state) {
    case P_DOWN: {
      static const uint8_t reset[1] = {1};
      p.lastTryMs = millis(); p.setupStartMs = millis();
      memset(p.seq, 0, sizeof p.seq);
      opWrite(b, p, 1, reset, 1);                  // soft reset (executable channel)
      break;
    }
    case P_BOOT:
      p.state = P_DRAIN;                           // tState stays at the reset time
      opHdr(b);
      break;
    case P_IDREQ: {
      static const uint8_t req[2] = {0xF9, 0};     // product ID request
      opWrite(b, p, 2, req, 2);
      break;
    }
    case P_ENABLE: {
      uint32_t s = sensorMs() * 1000UL, mg = magMs() * 1000UL;
      switch (p.enStep) {
        case 0: opSetFeature(b, p, quatMode ? 0x08 : 0x05, 0); break;   // other orientation report off
        case 1: opSetFeature(b, p, quatMode ? 0x05 : 0x08, s); break;
        case 2: opSetFeature(b, p, 0x01, s); break;
        case 3: opSetFeature(b, p, 0x02, s); break;
        default: opSetFeature(b, p, 0x03, mg); break;
      }
      break;
    }
    default:                                       // DRAIN, IDWAIT, LIVE: read a packet
      opHdr(b);
  }
}

void pickAndStart(Bus& b) {
  int bi = &b - buses;
  uint32_t ms = millis(), us = micros();
  for (int k = 0; k < NPODS; k++) {
    int i = (b.rr + k) % NPODS;
    Pod& p = pods[i];
    if (p.bus != bi || !isDue(p, ms, us)) continue;
    b.cur = i; b.pkts = 0; b.visitStartUs = us;
    if (MUXED && b.muxCh != p.ch) opSelect(b, p.ch);
    else podAction(b);
    return;
  }
}

void onNak(Bus& b, const char* reason) {
  Pod& p = pods[b.cur];
  if (MUXED && p.state == P_DOWN) {                       // nothing at this address: try the other one
    bool first = p.addr == IMU_ADDR;
    p.addr = first ? IMU_ADDR_ALT : IMU_ADDR;
    if (first) { p.lastTryMs = millis() - RETRY_MS; endVisit(b); return; }
  }
  bool booting = (p.state == P_BOOT || p.state == P_DRAIN) && millis() - p.tState < BOOT_NAK_MS;
  if (p.state == P_LIVE) b.errors++;
  if (!booting) podFail(b.cur, reason);
  endVisit(b);
}

// A read finished: rx holds the SHTP header and up to READ_LEN-4 bytes of cargo.
void onPacket(Bus& b) {
  Pod& p = pods[b.cur];
  uint16_t raw = (uint16_t)(b.rx[1] << 8) | b.rx[0];
  uint16_t len = raw & 0x7FFF;
  bool cont = raw & 0x8000;                                  // rest of a packet longer than READ_LEN
  uint8_t chan = b.rx[2];
  uint32_t ms = millis();
  if (raw == 0xFFFF) { onNak(b, "bad_header"); return; }     // all ones: nothing driving the bus
  if (p.state == P_LIVE) p.reads++;
  if (len <= 4) {                                            // no data waiting
    if (p.state == P_LIVE) {                                 // polled a little early: try again shortly
      p.nextPollUs = micros() + POLL_RETRY_US;
      p.empties++;
    } else {
      p.nextPollUs = micros() + START_POLL_US;               // starting up: don't hog the bus
      if (p.state == P_DRAIN && ms - p.tState >= DRAIN_MIN_MS) p.state = P_IDREQ;
      else if (p.state == P_IDWAIT && ms - p.tState > ID_TIMEOUT_MS) podFail(b.cur, "no_id");
    }
    endVisit(b);
    return;
  }
  int n = min((int)len, (int)b.want);
  b.pkts++;
  if (p.state == P_LIVE) {
    p.pktCh[chan < 6 ? chan : 5]++;
    memcpy(p.last, b.rx, 8);
    if (chan == 1 && !cont) { podFail(b.cur, "imu_reset"); endVisit(b); return; }   // "reset complete": the IMU rebooted
    bool more = len > b.want;                                // rest of this packet comes in the next read
    if (fixedRead) p.readLen = fixedRead;                    // "readlen N": same length every read (default 56)
    else if (!cont) {                                        // "readlen 0": size the next read to the packets we get:
      uint16_t fit = min((uint16_t)READ_MAX, (uint16_t)(len + READ_SPARE));   // grow at once, shrink slowly
      p.readLen = fit > p.readLen ? fit : (uint16_t)max((int)READ_MIN, p.readLen - 1);
    }
    if (chan == 3 && n > 4) feedReports(b.cur, b.rx + 4, n - 4, cont, more);
    // One read per visit. Re-reading at once mostly returns nothing and costs a full READ_LEN transfer.
    // Come back soon for a continuation or a non-sensor packet, else about one report period later.
    p.nextPollUs = (more || chan != 3) ? micros() + POLL_RETRY_US
                                       : b.visitStartUs + sensorMs() * 1000UL - NEXT_EARLY_US;
  } else if (p.state == P_IDWAIT) {
    if (chan == 2 && !cont && b.rx[4] == 0xF8) { p.state = P_ENABLE; p.enStep = 0; p.tState = ms - EN_GAP_MS; }
  } else if (p.state == P_DRAIN) {
    if (ms - p.tState > DRAIN_MAX_MS) p.state = P_IDREQ;
  }
  if (p.state != P_LIVE) p.nextPollUs = micros() + START_POLL_US;
  endVisit(b);
}

void onWritten(Bus& b) {
  Pod& p = pods[b.cur];
  uint32_t ms = millis();
  if (p.state == P_DOWN)       { p.state = P_BOOT; p.tState = ms; p.nextPollUs = micros(); }
  else if (p.state == P_IDREQ) { p.state = P_IDWAIT; p.tState = ms; p.nextPollUs = micros() + START_POLL_US; }
  else if (p.state == P_ENABLE) {
    p.tState = ms;
    if (++p.enStep >= N_EN_STEPS) goLive(b.cur);
  }
  endVisit(b);
}

void serviceBus(Bus& b) {
  if (b.op != OP_NONE) {
    if (!b.m->finished()) {
      if (micros() - b.opStartUs > OP_TIMEOUT_US) {
        b.timeouts++; b.busyAcc += OP_TIMEOUT_US;
        resetBus(b);
        b.op = OP_NONE; b.muxCh = 0xFF;
        if (b.cur >= 0) { podFail(b.cur, "bus_timeout"); endVisit(b); }
      }
      return;
    }
    b.busyAcc += micros() - b.opStartUs;
    bool err = b.m->has_error();
    BusOp was = b.op;
    b.op = OP_NONE;
    if (was == OP_SELECT) {
      if (err) {
        b.muxCh = 0xFF;
        if (pods[b.cur].state == P_LIVE) b.errors++;
        podFail(b.cur, "mux_nak");
        endVisit(b);
      } else {
        b.muxCh = pods[b.cur].ch;
        podAction(b);
      }
    } else if (err) {
      onNak(b, was == OP_WRITE && pods[b.cur].state == P_DOWN ? "no_ack" : "nak");
    } else if (was == OP_BODY)  onPacket(b);
    else                        onWritten(b);
    if (b.op != OP_NONE) return;
  }
  if (b.cur < 0) pickAndStart(b);
}

// Restart IMUs that went silent, and bring-ups that hang.
void watchdog() {
  uint32_t ms = millis();
  for (int i = 0; i < NPODS; i++) {
    Pod& p = pods[i];
    bool visiting = buses[p.bus].cur == i;
    if (visiting) continue;
    if (p.state == P_LIVE && ms - p.lastFreshMs > STALL_MS) podFail(i, "silent");
    else if (p.state != P_LIVE && p.state != P_DOWN && ms - p.setupStartMs > SETUP_MAX_MS) podFail(i, "setup_timeout");
  }
}

// Bus time per frame (all transfers, including empty polls).
void busStats() {
  uint32_t t = ticks;
  if (t == seenTicks) return;
  uint32_t n = t - seenTicks;
  seenTicks = t;
  for (Bus& b : buses) {
    noInterrupts();
    uint32_t acc = b.busyAcc; b.busyAcc = 0;
    interrupts();
    uint32_t per = acc / n;
    b.nFrames += n; b.busySumUs += acc;
    if (per > b.busyMaxUs) b.busyMaxUs = per;
    if (per > BUSY_LIMIT_US) b.over += n;
  }
}

void resetCounters() {
  statsStartMs = millis();
  for (Pod& p : pods)
    p.framesFresh = p.updates = p.stalls = p.restarts = p.maxPollUs = p.nAcc = p.nGyr = p.nMag = 0;
  for (Bus& b : buses) { b.nFrames = b.busyMaxUs = b.over = b.errors = b.timeouts = 0; b.busySumUs = 0; }
  resetExtStats();
}

void restartAll() {
  quiesce();
  scanBuses();
  uint32_t ms = millis();
  for (int i = 0; i < NPODS; i++) {
    Pod& p = pods[i];
    p.state = P_DOWN; p.ok = p.failNoted = p.reconfig = false;
    p.addr = MUXED || p.ch == 0 ? IMU_ADDR : IMU_ADDR_ALT;
    p.lastTryMs = ms - RETRY_MS; p.lastFreshMs = ms;
    p.headAccRad = -1; p.qStat = 0;
    noInterrupts();
    latest[i].q[0] = 1; latest[i].q[1] = latest[i].q[2] = latest[i].q[3] = 0;
    latest[i].fresh = false; latest[i].ok = false;
    interrupts();
  }
  resetCounters();
}

// ---------------- status ----------------
float busAvgMs(Bus& b) { return b.nFrames ? (float)(b.busySumUs / b.nFrames) / 1000.0f : 0.0f; }

void printStatus() {
  uint32_t qlen = (ringHead + RING - ringTail) % RING;
  Serial.printf("@STATUS rec=%d frames=%lu seq=%lu report_ms=%u frame_us=%lu sensor_ms=%u late_resyncs=0 "
                "dropped_f=%lu dropped_p=%lu queue=%lu slow=0 uptime_ms=%lu i2c_khz=%lu stats_ms=%lu\n",
                recording, (unsigned long)frame, (unsigned long)seq, reportMs, (unsigned long)frameUs,
                sensorMs(), (unsigned long)droppedF, (unsigned long)droppedP, (unsigned long)qlen,
                (unsigned long)millis(), (unsigned long)(i2cHz / 1000), (unsigned long)(millis() - statsStartMs));
  float secs = max(0.001f, (millis() - statsStartMs) / 1000.0f);
  for (Pod& p : pods)                    // rates since the counters were reset (includes start-up time)
    Serial.printf("@HZ pod=%s q_hz=%.1f acc_hz=%.1f gyr_hz=%.1f mag_hz=%.1f\n", p.name, p.updates / secs,
                  p.nAcc / secs, p.nGyr / secs, p.nMag / secs);
  for (Pod& p : pods) {
    Serial.printf("@POD name=%s addr=%s%u i2c=0x%02X ok=%d live=%d state=%u updates=%lu n_acc=%lu n_gyr=%lu n_mag=%lu "
                  "stalls=%lu restarts=%lu max_poll_ms=%.1f quat=%s q_status=%u head_acc_deg=%.1f "
                  "reads=%lu empty=%lu conts=%lu read_len=%u\n",
                  p.name, buses[p.bus].name, p.ch, p.addr, p.ok, p.state == P_LIVE, p.state,
                  (unsigned long)p.updates, (unsigned long)p.nAcc, (unsigned long)p.nGyr, (unsigned long)p.nMag,
                  (unsigned long)p.stalls, (unsigned long)p.restarts, p.maxPollUs / 1000.0f, quatName(), p.qStat,
                  p.headAccRad < 0 ? -1.0f : p.headAccRad * 57.29578f,
                  (unsigned long)p.reads, (unsigned long)p.empties, (unsigned long)p.conts, p.readLen);
  }
  noInterrupts();
  uint32_t edges = extEdges, gl = extGlitches, mn = extMinUs, mx = extMaxUs, n = extN, last = extLastUs;
  uint64_t sum = extSumUs;
  interrupts();
  Serial.printf("@CLOCK src=%s pin=%d edges=%lu period_avg_us=%.1f period_min_us=%lu period_max_us=%lu "
                "glitches=%lu last_edge_ms_ago=%ld\n", extClock ? "ext" : "pit", CLK_PIN, (unsigned long)edges,
                n ? (double)sum / n : 0.0, (unsigned long)(n ? mn : 0), (unsigned long)mx, (unsigned long)gl,
                edges ? (long)((micros() - last) / 1000) : -1L);
  Serial.printf("@SD card=%d armed=%d recording=%d file=%s last=%s lines=%lu overruns=%lu max_buf=%lu max_sync_us=%lu\n",
                sdOk, sdArmed, sdOpen, sdName, sdLast[0] ? sdLast : "-", (unsigned long)sdLines,
                (unsigned long)sdOverruns, (unsigned long)sdMaxUsed, (unsigned long)sdMaxSyncUs);
  for (Bus& b : buses)
    Serial.printf("@BUS name=%s sda=%d scl=%d busy_avg_ms=%.2f busy_max_ms=%.2f over_8ms=%lu frames=%lu "
                  "errors=%lu timeouts=%lu recoveries=%lu\n", b.name, b.sda, b.scl, busAvgMs(b), b.busyMaxUs / 1000.0f,
                  (unsigned long)b.over, (unsigned long)b.nFrames, (unsigned long)b.errors, (unsigned long)b.timeouts,
                  (unsigned long)b.recoveries);
}

// ---------------- frame output ----------------
uint32_t usbSkipF = 0, usbSkipS = 0;   // recorded lines USB could not take (they are on the SD card)
void drainFrames() {
  static char line[1800];
  for (int budget = 0; budget < 32 && ringTail != ringHead; budget++) {
    Snap& s = ring[ringTail];
    int n = snprintf(line, sizeof(line), "%c,%lu,%lu", s.rec ? 'F' : 'P', (unsigned long)s.num, (unsigned long)s.t);
    for (int i = 0; i < NPODS; i++)
      n += snprintf(line + n, sizeof(line) - n, ",%d,%d,%.5f,%.5f,%.5f,%.5f",
                    (s.ok >> i) & 1, (s.fresh >> i) & 1, s.q[i][0], s.q[i][1], s.q[i][2], s.q[i][3]);
    for (int i = 0; i < NPODS; i++) {
      const float* x = s.x[i];
      n += snprintf(line + n, sizeof(line) - n, ",%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%d",
                    s.nb[i] & NB_ACC ? 1 : 0, s.nb[i] & NB_GYR ? 1 : 0, s.nb[i] & NB_MAG ? 1 : 0,
                    x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], s.magAcc[i]);
    }
    n += snprintf(line + n, sizeof(line) - n, "\r\n");
    if (!s.rec && quietP) n = 0;         // idle preview muted: drop the line, keep the queue moving
    if (s.rec) {                         // recording: the SD file gets every line; USB only what fits now
      sdWrite(line, n);
      if (!sendNow(line, n)) usbSkipF++;
    } else if (n && !sendNow(line, n)) return;
    for (int i = 0; i < NPODS; i++) if ((s.fresh >> i) & 1) pods[i].framesFresh++;
    noInterrupts();
    ringTail = (ringTail + 1) % RING;
    interrupts();
  }
}

// S,<pod index>,<report id>,<IMU sequence 0-255>,<t_us when read>,<status 0-3>,<values: quat w x y z | x y z>
// Frames go first; samples only while USB has room, the queue holds ~1.5 s.
void drainSamples() {
  static char line[96];
  for (int budget = 0; budget < 24 && sTail != sHead; budget++) {
    const Sample& s = sring[sTail];
    int n = s.id == 0x01 || s.id == 0x02 || s.id == 0x03
      ? snprintf(line, sizeof line, "S,%u,%u,%u,%lu,%u,%.4f,%.4f,%.4f\r\n", s.pod, s.id, s.seq,
                 (unsigned long)s.t, s.stat, s.v[0], s.v[1], s.v[2])
      : snprintf(line, sizeof line, "S,%u,%u,%u,%lu,%u,%.5f,%.5f,%.5f,%.5f\r\n", s.pod, s.id, s.seq,
                 (unsigned long)s.t, s.stat, s.v[0], s.v[1], s.v[2], s.v[3]);
    sdWrite(line, n);                    // samples exist only while recording: SD always, USB if it fits
    if (!sendNow(line, n)) usbSkipS++;
    sTail = (sTail + 1) % SRING;
  }
}

// ---------------- commands ----------------
void handleCommand(char* c) {
  while (*c == ' ') c++;
  int len = strlen(c);
  while (len && (c[len - 1] == ' ' || c[len - 1] == '\r')) c[--len] = 0;
  if (!len) return;

  if (!strcmp(c, "start")) {
    if (recording || startPending) { Serial.println("@ERR already recording"); return; }
    bool sdRec = sdStart();
    if (sdRec) {                                 // file header: firmware, pods, columns
      static char b[2400];
      int n = snprintf(b, sizeof b, "# %s %s pods=", FW_NAME, FW_VERSION);
      for (int i = 0; i < NPODS; i++)
        n += snprintf(b + n, sizeof b - n, "%s%s:%s%u", i ? "," : "", pods[i].name, buses[pods[i].bus].name, pods[i].ch);
      n += snprintf(b + n, sizeof b - n, " frame_us=%lu sensor_ms=%u mag_ms=%u quat=%s\r\n",
                    (unsigned long)frameUs, sensorMs(), magMs(), quatName());
      sdWrite(b, n);
      n = columnsLine(b, sizeof b);
      sdWrite(b, n);
    }
    usbSkipF = usbSkipS = 0;
    noInterrupts(); startPending = true; interrupts();
    Serial.printf("@ACK start sd_file=%s\n", sdRec ? sdName : "-");
  } else if (!strcmp(c, "stop")) {
    if (!recording && !startPending) { Serial.println("@ERR not recording"); return; }
    noInterrupts();
    bool was = recording;
    recording = false; startPending = false;
    uint32_t n = was ? frame : 0;
    interrupts();
    uint32_t t0 = millis();
    while ((ringTail != ringHead || sTail != sHead) && millis() - t0 < 1500) {
      drainFrames();
      drainSamples();
      sdService();
      for (Bus& b : buses) serviceBus(b);
    }
    bool hadSd = sdOpen;
    sdStop();
    Serial.printf("@STOP frames=%lu t_us=%lu dropped=%lu dropped_s=%lu late_resyncs=0 sd_file=%s sd_lines=%lu "
                  "sd_bytes=%llu sd_overruns=%lu sd_max_buf=%lu sd_max_sync_us=%lu usb_skip_f=%lu usb_skip_s=%lu",
                  (unsigned long)n, (unsigned long)micros(), (unsigned long)droppedF, (unsigned long)droppedS,
                  hadSd ? sdLast : "-", (unsigned long)sdLines, (unsigned long long)sdBytes, (unsigned long)sdOverruns,
                  (unsigned long)sdMaxUsed, (unsigned long)sdMaxSyncUs, (unsigned long)usbSkipF, (unsigned long)usbSkipS);
    for (Bus& b : buses) {                       // bus_a_..., bus_b_..., bus_c_...
      char k = (char)tolower(b.name[0]);
      Serial.printf(" bus_%c_avg_ms=%.2f bus_%c_max_ms=%.2f bus_%c_over8=%lu", k, busAvgMs(b), k,
                    b.busyMaxUs / 1000.0f, k, (unsigned long)b.over);
    }
    Serial.println();
  } else if (!strncmp(c, "sdget ", 6)) {
    if (recording || startPending) { Serial.println("@ERR stop recording before sdget"); return; }
    sdSend(c + 6);
  } else if (!strcmp(c, "sdls")) {
    sdList();
  } else if (!strcmp(c, "sdinit")) {
    if (recording || startPending) { Serial.println("@ERR stop recording before sdinit"); return; }
    sdBegin();
    Serial.printf("@ACK sdinit card=%d file=%s\n", sdOk, sdArmed ? sdName : "-");
  } else if (!strncmp(c, "ping", 4)) {
    const char* tok = c + 4;
    while (*tok == ' ') tok++;
    Serial.printf("@PONG %s %lu\n", *tok ? tok : "0", (unsigned long)micros());
  } else if (!strcmp(c, "hello")) {      // the host app: it wants the live P lines
    quietP = false;
    sayHello();
  } else if (!strcmp(c, "info")) {       // a person in the Serial Monitor: same banner, stays quiet
    sayHello();
  } else if (!strcmp(c, "i2ctime")) {
    if (recording || startPending) { Serial.println("@ERR stop recording before i2ctime"); return; }
    i2cTime();
  } else if (!strncmp(c, "sensor", 6)) {
    if (recording || startPending) { Serial.println("@ERR stop recording before changing sensor"); return; }
    int ms = atoi(c + 6);
    if (ms < 0 || ms > 40) { Serial.println("@ERR sensor must be 0 (auto) to 40 ms"); return; }
    sensorOverrideMs = (uint16_t)ms;
    for (int i = 0; i < NPODS; i++) reconfigure(i);
    resetCounters();
    Serial.printf("@ACK sensor_ms=%u mag_ms=%u\n", sensorMs(), magMs());
  } else if (!strncmp(c, "khz", 3)) {
    if (recording || startPending) { Serial.println("@ERR stop recording before changing khz"); return; }
    int k = atoi(c + 3);
    if (k != 100 && k != 400) { Serial.println("@ERR khz must be 100 or 400"); return; }
    i2cHz = (uint32_t)k * 1000;           // restartAll waits for the current transfers, then resetBus applies it
    Serial.printf("@ACK khz %d\n", k);
    restartAll();
  } else if (!strcmp(c, "status") || !strcmp(c, "s")) {
    printStatus();
  } else if (!strcmp(c, "quiet")) {
    quietP = !quietP;
    Serial.printf("@ACK quiet %s\n", quietP ? "on" : "off");
  } else if (!strcmp(c, "scan")) {
    if (recording || startPending) { Serial.println("@ERR stop recording before scan"); return; }
    Serial.println("@ACK scan");
    restartAll();                         // the scan interrupts every IMU, so bring them all up again
  } else if (!strcmp(c, "restart") || !strcmp(c, "r")) {
    if (recording || startPending) { Serial.println("@ERR stop recording before restart"); return; }
    Serial.println("@ACK restart");
    restartAll();
    sayHello();
  } else if (!strncmp(c, "rate", 4)) {
    if (recording || startPending) { Serial.println("@ERR stop recording before changing rate"); return; }
    int ms = atoi(c + 4);
    if (ms != 5 && ms != 10 && ms != 20 && ms != 40) { Serial.println("@ERR rate must be 5, 10, 20 or 40 ms"); return; }
    reportMs = ms; frameUs = (uint32_t)ms * 1000;
    for (int i = 0; i < NPODS; i++) reconfigure(i);
    startTicks();
    Serial.printf("@RATE report_ms=%u frame_us=%lu sensor_ms=%u mag_ms=%u\n", reportMs, (unsigned long)frameUs,
                  sensorMs(), magMs());
  } else if (!strncmp(c, "mag ", 4)) {
    if (recording || startPending) { Serial.println("@ERR stop recording before changing mag"); return; }
    const char* m = c + 4;
    while (*m == ' ') m++;
    if (!strcmp(m, "on")) magOn = true;
    else if (!strcmp(m, "off")) {
      if (quatMode) { Serial.println("@ERR mode rv needs the magnetometer; send mode game first"); return; }
      magOn = false;
    } else { Serial.println("@ERR mag must be on or off"); return; }
    for (int i = 0; i < NPODS; i++) reconfigure(i);
    resetCounters();
    Serial.printf("@ACK mag=%s mag_ms=%u\n", magOn ? "on" : "off", magMs());
    sayHello();                           // the host keeps the HELLO fields as the recording's firmware info
  } else if (!strncmp(c, "readlen", 7)) {
    if (recording || startPending) { Serial.println("@ERR stop recording before changing readlen"); return; }
    int n = atoi(c + 7);
    if (n != 0 && (n < READ_MIN || n > READ_MAX)) { Serial.println("@ERR readlen must be 0 (adaptive) or 24 to 64"); return; }
    fixedRead = (uint16_t)n;
    for (Pod& p : pods) p.readLen = n ? n : READ_LEN;
    resetCounters();
    Serial.printf("@ACK readlen=%u (%s)\n", fixedRead, fixedRead ? "fixed" : "adaptive");
  } else if (!strncmp(c, "clock", 5)) {
    const char* m = c + 5;
    while (*m == ' ') m++;
    if (*m) {
      if (recording || startPending) { Serial.println("@ERR stop recording before changing clock"); return; }
      if (!strcmp(m, "pit")) extClock = false;
      else if (!strcmp(m, "ext")) extClock = true;
      else { Serial.println("@ERR clock must be pit or ext"); return; }
      startTicks();
    }
    Serial.printf("@ACK clock=%s pin=%d frame_us=%lu\n", extClock ? "ext" : "pit", CLK_PIN, (unsigned long)frameUs);
  } else if (!strncmp(c, "mode", 4)) {
    const char* m = c + 4;
    while (*m == ' ') m++;
    if (*m) {
      if (recording || startPending) { Serial.println("@ERR stop recording before changing mode"); return; }
      if (!strcmp(m, "game")) quatMode = 0;
      else if (!strcmp(m, "rv")) quatMode = 1;
      else { Serial.println("@ERR mode must be game or rv"); return; }
      for (int i = 0; i < NPODS; i++) { pods[i].headAccRad = -1; pods[i].qStat = 0; reconfigure(i); }
    }
    Serial.printf("@MODE quat=%s\n", quatName());
  } else {
    Serial.printf("@ERR unknown command: %s\n", c);
  }
}

void readCommands() {
  static uint32_t lastCharMs = 0;
  // Serial Monitor set to "No Line Ending": run the command once typing stops for 300 ms
  if (cmdLen && !Serial.available() && millis() - lastCharMs > 300) {
    cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); cmdLen = 0;
  }
  while (Serial.available()) {
    char ch = Serial.read();
    lastCharMs = millis();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

// ---------------- setup / loop ----------------
// Boot log: one flushed line per start-up phase, so a freeze shows where it happened.
void bootStep(const char* what) {
  Serial.printf("# boot: %s (t=%lu ms)\n", what, (unsigned long)millis());
  Serial.flush();
}

void setup() {
  Serial.begin(115200);                  // USB: baud ignored
  pinMode(SYNC_PIN, OUTPUT); digitalWriteFast(SYNC_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWriteFast(LED_PIN, HIGH);   // LED stays on until setup() is done
  while (!Serial && millis() < 4000) {}  // give a host up to 4 s to open the port, so the boot log is seen
  bootStep("setup start");
  tickTimer.priority(32);                // above USB and I2C, so frame times stay regular
  for (Bus& b : buses) {
    b.muxCh = 0xFF; b.op = OP_NONE; b.cur = -1; b.rr = 0;
    b.m->begin(i2cHz);
  }
  bootStep("i2c begin done");
  delay(500);                            // BNO086 boot
  Serial.printf("# %s %s on %s\n", FW_NAME, FW_VERSION, BOARD_NAME);
  if (CrashReport) { Serial.println("# previous run crashed:"); Serial.print(CrashReport); }
  bootStep("sd");
  sdBegin();
  bootStep("restartAll");
  restartAll();
  bootStep("restartAll done");
  sayHello();
  startTicks();
  bootStep("setup done");
}

void loop() {
  // 1) both I2C state machines: each starts its next transfer as soon as the last one finishes
  for (Bus& b : buses) serviceBus(b);

  // 2) restart silent IMUs; bus load per frame
  watchdog();
  busStats();
  watchClock();

  // 3) announce a start, then send queued frames
  if (startNoted) {
    noInterrupts();
    startNoted = false;
    uint32_t t = recStartUs;
    interrupts();
    resetCounters();
    droppedS = 0;
    Serial.printf("@START t_us=%lu frame_us=%lu report_ms=%u sensor_ms=%u mag_ms=%u sd_file=%s\n",
                  (unsigned long)t, (unsigned long)frameUs, reportMs, sensorMs(), magMs(), sdOpen ? sdName : "-");
    char b[96];
    int n = snprintf(b, sizeof b, "# start t_us=%lu\r\n", (unsigned long)t);
    sdWrite(b, n);
  }
  drainFrames();
  drainSamples();
  sdService();

  // 4) LED: on while recording, 1 Hz blink when idle
  digitalWriteFast(LED_PIN, (recording || ((millis() / 500) & 1)) ? HIGH : LOW);

  // 5) commands
  readCommands();
}
