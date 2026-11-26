// Stage 1 - I2C map. Board: Teensy 4.1. No libraries needed.
// Scans Wire1 (SDA1 17 / SCL1 16, mux A) and Wire (SDA 18 / SCL 19, mux B),
// wiring as in the senior's handover (hardware/wiring/teensy41_wiring.html),
// then every channel of each mux. Prints the map every 3 s.
// In each "ch" line, ignore 0x70 (the mux itself answers on its main bus)
// and 0x6F (the Qwiic Button, if it sits on mux B's main bus).
#include <Wire.h>

const uint8_t MUX_ADDR = 0x70;

void scan(TwoWire &bus, const char *label) {
  Serial.printf("%s:", label);
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  if (found == 0) Serial.print(" none");
  Serial.println();
}

bool muxSelect(TwoWire &bus, int ch) {   // ch = -1 switches every channel off
  bus.beginTransmission(MUX_ADDR);
  bus.write(ch < 0 ? (uint8_t)0 : (uint8_t)(1 << ch));
  return bus.endTransmission() == 0;
}

void mapBus(TwoWire &bus, const char *name) {
  Serial.printf("\n== %s ==\n", name);
  if (!muxSelect(bus, -1)) {
    Serial.println("  mux 0x70 not answering: check the Qwiic lead and the mux power LED");
    scan(bus, "  main bus");
    return;
  }
  scan(bus, "  main bus (expect 0x70; 0x6F = button)");
  for (int ch = 0; ch < 8; ch++) {
    muxSelect(bus, ch);
    char label[16];
    snprintf(label, sizeof label, "  ch %d", ch);
    scan(bus, label);
  }
  muxSelect(bus, -1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  Wire.begin();
  Wire.setClock(100000);
  Wire1.begin();
  Wire1.setClock(100000);
}

void loop() {
  mapBus(Wire1, "Wire1 (SDA1 17, SCL1 16) - mux A");
  mapBus(Wire, "Wire  (SDA 18, SCL 19) - mux B");
  delay(3000);
}
