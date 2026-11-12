// =====================================================================
//  imu_daq firmware v1.1 — SID Lab IMU acquisition with DAQ time sync
//  Boards: Seeed XIAO ESP32S3 or XIAO ESP32C3 (pins picked from the board selected in the IDE)
//  IMUs:   2 x BNO085/BNO086 on one I2C bus (STEMMA QT / Qwiic chain)
//            pod "A4B" = board at 0x4B, pod "A4A" = board at 0x4A
//            Adafruit BNO085: default 0x4A; DI soldered to 3Vo -> 0x4B
//            SparkFun BNO086: default 0x4B; ADR jumper closed -> 0x4A
//
//  LIBRARY: "SparkFun BNO080 Cortex Based IMU" v1.1.12 (NOT "SparkFun BNO08x": one sensor only)
//  BOARD:   Tools > Board > esp32 > XIAO_ESP32S3 (USB Mode: Hardware CDC and JTAG) or XIAO_ESP32C3
//
//  WIRING (XIAO label)                      S3          C3
//    3V3 -> IMU VIN (never 5V)
//    GND -> IMU GND and DAQ AI 8 (AI0-)
//    D4 SDA                                 GPIO5       GPIO6
//    D5 SCL                                 GPIO6       GPIO7
//    SYNC -> DAQ AI 0 (AI0+)                D1 GPIO2    D2 GPIO4
//
//  TIMING
//    A hardware timer (esp_timer) ticks every frame period. Each tick takes a snapshot of the
//    newest quaternion of every IMU and, while recording, raises SYNC; a one-shot timer lowers it
//    after 1 ms (3 ms for frame 0). Frame spacing and SYNC edges therefore do not depend on the
//    main loop, which only reads the IMUs over I2C and sends queued frames over USB. Frames wait in
//    a 512-frame queue when USB is briefly busy; only an overflowing queue loses lines (counted).
//    The IMUs report ~10 % faster than the frame rate, so (almost) every frame has a new reading.
//
//  BEHAVIOUR
//    IDLE       preview frames "P,..." every tick, no SYNC pulses.
//    RECORDING  after "start": frame counter restarts at 0; "F,..." every tick, one SYNC pulse each.
//               DAQ pulse k (counting from the wide frame-0 pulse) == frame k.
//
//  OUTPUT LINES (native USB serial, baud ignored)
//    P,seq,t_us,<pod: ok,fresh,w,x,y,z>...     preview frame (idle)
//    F,frame,t_us,<pod: ok,fresh,w,x,y,z>...   recorded frame; t_us = time of its SYNC rising edge
//    @HELLO key=value ...                      firmware / board / pins / rate
//    @ACK cmd | @ERR text                      command result
//    @START t_us=.. frame_us=..                recording started (t_us of frame 0)
//    @STOP frames=.. t_us=.. dropped=..        recording stopped; frames = SYNC pulses sent
//    @PONG token t_us                          reply to "ping token" (clock sync)
//    @EVENT pod=.. type=.. ...                 stall / restarted / found / restart_failed / slow
//    @STATUS ... and @POD ...                  reply to "status"
//    @RATE report_ms=.. frame_us=.. sensor_ms=..
//    # ...                                     human-readable comments ("# columns: ..." header)
//    t_us = micros() on the XIAO (wraps every 71.6 min; the host unwraps it).
//
//  COMMANDS (one per line)
//    hello | start | stop | ping <token> | status | restart | rate <5|10|20|40>   (r / s also work)
// =====================================================================
#include <Wire.h>
#include "esp_timer.h"
#include "SparkFun_BNO080_Arduino_Library.h"

#define FW_NAME    "imu_daq"
#define FW_VERSION "1.1.0"

// ---------------- pins ----------------
#if defined(CONFIG_IDF_TARGET_ESP32S3)
const char* BOARD_NAME = "XIAO_ESP32S3";
const int SDA_PIN  = 5;   // D4
const int SCL_PIN  = 6;   // D5
const int SYNC_PIN = 2;   // D1  (D2 = GPIO3 is a strapping pin on the S3)
const int LED_PIN  = 21;  // user LED, active LOW
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
const char* BOARD_NAME = "XIAO_ESP32C3";
const int SDA_PIN  = 6;   // D4
const int SCL_PIN  = 7;   // D5
const int SYNC_PIN = 4;   // D2
const int LED_PIN  = -1;  // no user LED
#else
#error "Select XIAO_ESP32S3 or XIAO_ESP32C3 in Tools > Board"
#endif

// ---------------- settings ----------------
const uint32_t I2C_HZ        = 400000;
const uint32_t I2C_INIT_HZ   = 100000;  // start-up / restart (tolerant of BNO08x boot traffic)
const uint32_t SYNC_US       = 1000;    // normal SYNC pulse
const uint32_t START_SYNC_US = 3000;    // frame-0 marker pulse (fits 200 Hz: 3 ms high + 2 ms low)
const int      INIT_TRIES    = 5;
const uint32_t STALL_MS      = 500;
const uint32_t RETRY_MS      = 1000;
const uint32_t SLOW_US       = 30000;

uint16_t reportMs = 10;                 // frame period in ms (10 ms = 100 Hz frames)
uint32_t frameUs  = 10000;

#define NPODS 2
#define RING  512                       // queued frames (5 s at 100 Hz)

// ---------------- pods ----------------
struct Pod {
  const char* name;
  uint8_t     addr;
  BNO080      imu;
  bool        ok;          // found at least once
  bool        live;        // currently polled
  bool        failNoted;
  uint32_t    updates, lastFreshMs, lastTryMs, stalls, restarts, maxPollUs, framesFresh;
};
Pod pods[NPODS] = { {"A4B", 0x4B}, {"A4A", 0x4A} };

// Shared between the main loop (writer) and the frame timer (reader), guarded by `mux`.
struct Latest { float q[4]; bool fresh; bool ok; };
volatile Latest latest[NPODS];

struct Snap { uint32_t num; uint32_t t; bool rec; uint8_t ok, fresh; float q[NPODS][4]; };
Snap ring[RING];
volatile uint32_t ringHead = 0, ringTail = 0;   // head: timer writes, tail: loop reads

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
esp_timer_handle_t tickTimer = nullptr, pulseTimer = nullptr;

// state touched by the timer (guarded by mux)
volatile bool     recording = false, startPending = false, startNoted = false;
volatile uint32_t frame = 0, seq = 0, recStartUs = 0;
volatile uint32_t droppedF = 0, droppedP = 0;

uint32_t slowEvents = 0;
char     cmdBuf[64];
int      cmdLen = 0;

// ---------------- timer callbacks (esp_timer task) ----------------
void IRAM_ATTR pulseEnd(void*) { gpio_set_level((gpio_num_t)SYNC_PIN, 0); }

void onTick(void*) {
  uint32_t t = micros();
  bool pulse = false;
  uint32_t width = SYNC_US;
  portENTER_CRITICAL(&mux);
  if (startPending) {
    startPending = false; recording = true; startNoted = true;
    frame = 0; droppedF = 0; recStartUs = t;
  }
  bool rec = recording;
  uint32_t num = rec ? frame : seq;
  if (rec) { pulse = true; width = (frame == 0) ? START_SYNC_US : SYNC_US; }
  uint32_t next = (ringHead + 1) % RING;
  if (next != ringTail) {
    Snap& s = ring[ringHead];
    s.num = num; s.t = t; s.rec = rec; s.ok = 0; s.fresh = 0;
    for (int i = 0; i < NPODS; i++) {
      for (int k = 0; k < 4; k++) s.q[i][k] = latest[i].q[k];
      if (latest[i].ok) s.ok |= 1 << i;
      if (latest[i].fresh) s.fresh |= 1 << i;
      latest[i].fresh = false;
    }
    ringHead = next;
  } else {
    for (int i = 0; i < NPODS; i++) latest[i].fresh = false;
    if (rec) droppedF++; else droppedP++;
  }
  if (rec) frame++; else seq++;
  portEXIT_CRITICAL(&mux);
  if (pulse) {
    gpio_set_level((gpio_num_t)SYNC_PIN, 1);
    esp_timer_start_once(pulseTimer, width);
  }
}

void startTicks() {
  esp_timer_stop(tickTimer);
  esp_timer_start_periodic(tickTimer, frameUs);
}

// ---------------- helpers ----------------
// The BNO08x runs on its own clock; reporting ~10 % faster than the frame rate gives (almost)
// every frame a fresh quaternion that is at most one sensor period old.
uint16_t sensorMs() { return reportMs > 1 ? (uint16_t)max(1, reportMs * 9 / 10) : (uint16_t)1; }

bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Send a line only if it fits in the USB buffer right now (never waits for the PC).
bool sendNow(const char* s, int n) {
  if (Serial.availableForWrite() >= n) { Serial.write((const uint8_t*)s, n); return true; }
  return false;
}

void sayHello() {
  Serial.printf("@HELLO fw=%s ver=%s board=%s sda=%d scl=%d sync=%d report_ms=%u frame_us=%lu "
                "sensor_ms=%u start_sync_us=%lu sync_us=%lu clock=esp_timer pods=",
                FW_NAME, FW_VERSION, BOARD_NAME, SDA_PIN, SCL_PIN, SYNC_PIN, reportMs,
                (unsigned long)frameUs, sensorMs(), (unsigned long)START_SYNC_US, (unsigned long)SYNC_US);
  for (int i = 0; i < NPODS; i++) Serial.printf("%s%s:0x%02X", i ? "," : "", pods[i].name, pods[i].addr);
  Serial.printf(" rec=%d\n", recording);
  Serial.print("# columns: F,frame,t_us");
  for (int i = 0; i < NPODS; i++) Serial.printf(",%s_ok,%s_fresh,%s_w,%s_x,%s_y,%s_z",
      pods[i].name, pods[i].name, pods[i].name, pods[i].name, pods[i].name, pods[i].name);
  Serial.println();
}

void scanBus() {
  Serial.print("# I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) if (present(a)) { Serial.printf(" 0x%02X", a); found++; }
  Serial.printf("  (%d device%s)\n", found, found == 1 ? "" : "s");
}

void setLatestOk(int i, bool ok) {
  portENTER_CRITICAL(&mux);
  latest[i].ok = ok;
  portEXIT_CRITICAL(&mux);
}

// One attempt to bring a pod up; returns "" on success, else the reason.
const char* initPod(int i) {
  Pod& p = pods[i];
  if (!present(p.addr)) return "no_ack";
  if (!p.imu.begin(p.addr, Wire)) return "begin_failed";
  delay(50);
  p.imu.enableGameRotationVector(sensorMs());
  return "";
}

void resetPodCounters() {
  for (int i = 0; i < NPODS; i++)
    pods[i].framesFresh = pods[i].updates = pods[i].stalls = pods[i].restarts = pods[i].maxPollUs = 0;
}

void startPods() {
  Wire.setClock(I2C_INIT_HZ);
  scanBus();
  for (int i = 0; i < NPODS; i++) {
    Pod& p = pods[i];
    p.ok = p.live = p.failNoted = false;
    portENTER_CRITICAL(&mux);
    latest[i].q[0] = 1; latest[i].q[1] = latest[i].q[2] = latest[i].q[3] = 0;
    latest[i].fresh = false; latest[i].ok = false;
    portEXIT_CRITICAL(&mux);
    const char* why = "";
    for (int tryNo = 1; tryNo <= INIT_TRIES && !p.ok; tryNo++) {
      if (tryNo > 1) delay(200);
      why = initPod(i);
      if (!*why) {
        p.ok = p.live = true;
        setLatestOk(i, true);
        Serial.printf("# %s: OK at 0x%02X (game rotation vector every %u ms, try %d)\n",
                      p.name, p.addr, sensorMs(), tryNo);
      }
    }
    if (!p.ok) Serial.printf("@EVENT pod=%s type=not_found reason=%s addr=0x%02X\n", p.name, why, p.addr);
    p.lastFreshMs = p.lastTryMs = millis();
  }
  resetPodCounters();
  Wire.setClock(I2C_HZ);
}

uint32_t curNum() { return recording ? frame : seq; }

void printStatus() {
  uint32_t qlen = (ringHead + RING - ringTail) % RING;
  Serial.printf("@STATUS rec=%d frames=%lu seq=%lu report_ms=%u frame_us=%lu sensor_ms=%u late_resyncs=0 "
                "dropped_f=%lu dropped_p=%lu queue=%lu slow=%lu uptime_ms=%lu\n",
                recording, (unsigned long)frame, (unsigned long)seq, reportMs, (unsigned long)frameUs,
                sensorMs(), (unsigned long)droppedF, (unsigned long)droppedP, (unsigned long)qlen,
                (unsigned long)slowEvents, (unsigned long)millis());
  for (int i = 0; i < NPODS; i++) {
    Pod& p = pods[i];
    Serial.printf("@POD name=%s addr=0x%02X ok=%d live=%d updates=%lu stalls=%lu restarts=%lu "
                  "max_poll_ms=%.1f\n", p.name, p.addr, p.ok, p.live, (unsigned long)p.updates,
                  (unsigned long)p.stalls, (unsigned long)p.restarts, p.maxPollUs / 1000.0f);
  }
}

void noteSlow(const char* what, uint32_t us) {
  slowEvents++;
  Serial.printf("@EVENT pod=- type=slow step=%s ms=%.1f rec=%d frame=%lu\n",
                what, us / 1000.0f, recording, (unsigned long)curNum());
}

// ---------------- watchdog: restart silent pods ----------------
void watchdog() {
  uint32_t ms = millis();
  for (int i = 0; i < NPODS; i++) {
    Pod& p = pods[i];
    if (p.live && ms - p.lastFreshMs > STALL_MS) {
      p.live = false; p.stalls++; p.failNoted = false;
      p.lastTryMs = ms - RETRY_MS;
      Serial.printf("@EVENT pod=%s type=stall silent_ms=%lu rec=%d frame=%lu\n", p.name,
                    (unsigned long)(ms - p.lastFreshMs), recording, (unsigned long)curNum());
    }
    if (!p.live && ms - p.lastTryMs >= RETRY_MS) {
      Wire.setClock(I2C_INIT_HZ);
      const char* why = initPod(i);
      Wire.setClock(I2C_HZ);
      if (!*why) {
        bool wasOk = p.ok;
        p.ok = p.live = true; p.failNoted = false;
        setLatestOk(i, true);
        p.lastFreshMs = millis();
        if (wasOk) p.restarts++;
        Serial.printf("@EVENT pod=%s type=%s rec=%d frame=%lu\n", p.name,
                      wasOk ? "restarted" : "found", recording, (unsigned long)curNum());
      } else if (!p.failNoted) {
        p.failNoted = true;
        Serial.printf("@EVENT pod=%s type=restart_failed reason=%s addr=0x%02X\n", p.name, why, p.addr);
      }
      p.lastTryMs = millis();
      return;   // at most one attempt per loop pass
    }
  }
}

// ---------------- frame output ----------------
// Send queued frames while they fit in the USB buffer; keep the rest for the next pass.
void drainFrames() {
  char line[256];
  for (int budget = 0; budget < 32 && ringTail != ringHead; budget++) {
    Snap& s = ring[ringTail];
    int n = snprintf(line, sizeof(line), "%c,%lu,%lu", s.rec ? 'F' : 'P',
                     (unsigned long)s.num, (unsigned long)s.t);
    for (int i = 0; i < NPODS; i++) {
      n += snprintf(line + n, sizeof(line) - n, ",%d,%d,%.5f,%.5f,%.5f,%.5f",
                    (s.ok >> i) & 1, (s.fresh >> i) & 1, s.q[i][0], s.q[i][1], s.q[i][2], s.q[i][3]);
    }
    n += snprintf(line + n, sizeof(line) - n, "\r\n");
    if (!sendNow(line, n)) return;          // USB busy: try again next pass
    for (int i = 0; i < NPODS; i++) if ((s.fresh >> i) & 1) pods[i].framesFresh++;
    portENTER_CRITICAL(&mux);
    ringTail = (ringTail + 1) % RING;
    portEXIT_CRITICAL(&mux);
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
    portENTER_CRITICAL(&mux);
    startPending = true;                      // begins on the next tick
    portEXIT_CRITICAL(&mux);
    Serial.println("@ACK start");
  } else if (!strcmp(c, "stop")) {
    if (!recording && !startPending) { Serial.println("@ERR not recording"); return; }
    portENTER_CRITICAL(&mux);
    bool was = recording;
    recording = false; startPending = false;
    uint32_t n = was ? frame : 0;
    portEXIT_CRITICAL(&mux);
    // send the remaining queued F frames before the @STOP line
    uint32_t t0 = millis();
    while (ringTail != ringHead && millis() - t0 < 500) { drainFrames(); delay(1); }
    Serial.printf("@STOP frames=%lu t_us=%lu dropped=%lu late_resyncs=0\n",
                  (unsigned long)n, (unsigned long)micros(), (unsigned long)droppedF);
  } else if (!strncmp(c, "ping", 4)) {
    const char* tok = c + 4;
    while (*tok == ' ') tok++;
    Serial.printf("@PONG %s %lu\n", *tok ? tok : "0", (unsigned long)micros());
  } else if (!strcmp(c, "hello") || !strcmp(c, "info")) {
    sayHello();
  } else if (!strcmp(c, "status") || !strcmp(c, "s")) {
    printStatus();
  } else if (!strcmp(c, "restart") || !strcmp(c, "r")) {
    if (recording || startPending) { Serial.println("@ERR stop recording before restart"); return; }
    Serial.println("@ACK restart");
    startPods();
    sayHello();
  } else if (!strncmp(c, "rate", 4)) {
    if (recording || startPending) { Serial.println("@ERR stop recording before changing rate"); return; }
    int ms = atoi(c + 4);
    if (ms != 5 && ms != 10 && ms != 20 && ms != 40) { Serial.println("@ERR rate must be 5, 10, 20 or 40 ms"); return; }
    reportMs = ms; frameUs = (uint32_t)ms * 1000;
    for (int i = 0; i < NPODS; i++) if (pods[i].live) pods[i].imu.enableGameRotationVector(sensorMs());
    startTicks();
    Serial.printf("@RATE report_ms=%u frame_us=%lu sensor_ms=%u\n", reportMs, (unsigned long)frameUs, sensorMs());
  } else {
    Serial.printf("@ERR unknown command: %s\n", c);
  }
}

void readCommands() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
#if !ARDUINO_USB_CDC_ON_BOOT || ARDUINO_USB_MODE
  Serial.setTxBufferSize(4096);
#endif
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(10);   // never 0: in core 3.3.x HWCDC, 0 means "wait forever for the PC"
#endif
  Serial.begin(921600);
  pinMode(SYNC_PIN, OUTPUT); digitalWrite(SYNC_PIN, LOW);
  if (LED_PIN >= 0) { pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH); }
  for (int i = 0; i < NPODS; i++) { latest[i].q[0] = 1; latest[i].q[1] = latest[i].q[2] = latest[i].q[3] = 0; }

  esp_timer_create_args_t a = {};
  a.callback = &pulseEnd; a.name = "sync_end";
  esp_timer_create(&a, &pulseTimer);
  a.callback = &onTick; a.name = "frame_tick";
  esp_timer_create(&a, &tickTimer);

  delay(1500);                 // BNO08x boot time
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  Wire.setTimeOut(50);
  Serial.printf("# %s %s on %s\n", FW_NAME, FW_VERSION, BOARD_NAME);
  startPods();
  sayHello();
  startTicks();
}

void loop() {
  // 1) read live pods; publish the newest quaternion for the frame timer
  for (int i = 0; i < NPODS; i++) {
    Pod& p = pods[i];
    if (!p.live) continue;
    uint32_t t0 = micros();
    for (int k = 0; k < 3; k++) {
      uint16_t id = p.imu.getReadings();
      if (id == 0) break;
      if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
        float w = p.imu.getQuatReal(), x = p.imu.getQuatI(), y = p.imu.getQuatJ(), z = p.imu.getQuatK();
        portENTER_CRITICAL(&mux);
        latest[i].q[0] = w; latest[i].q[1] = x; latest[i].q[2] = y; latest[i].q[3] = z;
        latest[i].fresh = true;
        portEXIT_CRITICAL(&mux);
        p.updates++;
        p.lastFreshMs = millis();
      }
    }
    uint32_t dt = micros() - t0;
    if (dt > p.maxPollUs) p.maxPollUs = dt;
    if (dt > SLOW_US) noteSlow(p.name, dt);
  }

  // 2) restart silent pods (frames keep ticking meanwhile)
  uint32_t tw = micros();
  watchdog();
  uint32_t dtw = micros() - tw;
  if (dtw > SLOW_US) noteSlow("watchdog", dtw);

  // 3) announce a start, then send queued frames
  if (startNoted) {
    portENTER_CRITICAL(&mux);
    startNoted = false;
    uint32_t t = recStartUs;
    portEXIT_CRITICAL(&mux);
    resetPodCounters();
    Serial.printf("@START t_us=%lu frame_us=%lu report_ms=%u sensor_ms=%u\n",
                  (unsigned long)t, (unsigned long)frameUs, reportMs, sensorMs());
  }
  drainFrames();

  // 4) LED: on while recording, 1 Hz blink when idle
  if (LED_PIN >= 0) {
    bool on = recording || ((millis() / 500) & 1);
    digitalWrite(LED_PIN, on ? LOW : HIGH);
  }

  // 5) commands
  readCommands();
}
