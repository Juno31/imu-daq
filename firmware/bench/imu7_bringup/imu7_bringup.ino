// Stage 1 - 8-IMU bring-up: a wiring and connection test, not the final firmware.
// Board: Teensy 4.1. Library: "SparkFun BNO080 Cortex Based IMU" (the one imu_daq v1.3 uses).
// Wiring as in the senior's handover (hardware/wiring/teensy41_wiring.html):
// Mux A on Wire1 (SDA1 17, SCL1 16): ch 0-3 = IMU1-IMU4 (C7 T3 T6 T9).
// Mux B on Wire  (SDA 18,  SCL 19): ch 0-3 = IMU5-IMU8 (T11 L1 L3 S1).
// Each IMU reports the game rotation vector every 10 ms. Once a second the sketch prints,
// per IMU: reports per second, degrees turned since start-up, and restart count.
// The buses are read one after the other here; the final firmware reads them in parallel.
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

const uint8_t MUX_ADDR = 0x70;
const uint8_t IMU_ADDR = 0x4B;   // SparkFun BNO086 default (ADR jumper open)

struct Pod {
  const char *name;
  TwoWire *bus;
  uint8_t ch;
  BNO080 imu;
  bool ok;
  uint32_t reports, lastMs, restarts;
  bool haveRef;
  float q0[4];
  float deg;
};

Pod pods[8] = {
  {"IMU1-C7", &Wire1, 0},  {"IMU2-T3", &Wire1, 1}, {"IMU3-T6", &Wire1, 2}, {"IMU4-T9", &Wire1, 3},
  {"IMU5-T11", &Wire, 0},  {"IMU6-L1", &Wire, 1},  {"IMU7-L3", &Wire, 2},  {"IMU8-S1", &Wire, 3}
};

void selectChannel(Pod &p) {
  p.bus->beginTransmission(MUX_ADDR);
  p.bus->write((uint8_t)(1 << p.ch));
  p.bus->endTransmission();
}

bool startPod(Pod &p) {
  selectChannel(p);
  p.bus->setClock(100000);                     // gentle clock while the BNO086 boots
  bool ok = p.imu.begin(IMU_ADDR, *p.bus);
  if (ok) p.imu.enableGameRotationVector(10);  // 10 ms = 100 Hz
  p.bus->setClock(400000);
  p.lastMs = millis();
  p.haveRef = false;
  return ok;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  Wire.begin();
  Wire1.begin();
  for (Pod &p : pods) {
    p.ok = startPod(p);
    Serial.printf("%-9s %s\n", p.name, p.ok ? "found" : "NOT FOUND");
  }
}

uint32_t nextPrint = 1000;

void loop() {
  uint32_t now = millis();
  for (Pod &p : pods) {
    if (!p.ok) {                                 // retry a missing or stalled IMU once a second
      if (now - p.lastMs >= 1000) {
        p.ok = startPod(p);
        p.restarts++;
      }
      continue;
    }
    selectChannel(p);
    while (p.imu.dataAvailable()) {
      p.reports++;
      p.lastMs = millis();
      float q[4] = {p.imu.getQuatReal(), p.imu.getQuatI(), p.imu.getQuatJ(), p.imu.getQuatK()};
      if (!p.haveRef) {
        for (int i = 0; i < 4; i++) p.q0[i] = q[i];
        p.haveRef = true;
      }
      float d = fabsf(q[0] * p.q0[0] + q[1] * p.q0[1] + q[2] * p.q0[2] + q[3] * p.q0[3]);
      p.deg = 2.0f * acosf(fminf(d, 1.0f)) * 57.29578f;
    }
    if (millis() - p.lastMs > 500) {
      p.ok = false;
      Serial.printf("%s stalled\n", p.name);
    }
  }
  if (now >= nextPrint) {
    nextPrint += 1000;
    for (Pod &p : pods) {
      Serial.printf("%s %3lu Hz %5.1f deg r%lu | ", p.name, (unsigned long)p.reports, p.deg,
                    (unsigned long)p.restarts);
      p.reports = 0;
    }
    Serial.println();
  }
}
