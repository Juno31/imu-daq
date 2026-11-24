// Stage 4 - SYNC-only test. Board: Teensy 4.1. Pin 2 -> DAQ AI0, Teensy GND -> AI8.
// One pulse per 10 ms frame: 3 ms for frame 0, 1 ms for every later frame (imu_daq convention).
// Send 's' to start and 'x' to stop over the serial monitor; the sketch prints how many pulses it sent.
#include <IntervalTimer.h>

const int SYNC_PIN = 2;
IntervalTimer frameTimer;
volatile bool running = false;
volatile uint32_t frame = 0;

void tick() {
  if (!running) return;
  digitalWriteFast(SYNC_PIN, HIGH);
  delayMicroseconds(frame == 0 ? 3000 : 1000);
  digitalWriteFast(SYNC_PIN, LOW);
  frame++;
}

void setup() {
  pinMode(SYNC_PIN, OUTPUT);
  digitalWriteFast(SYNC_PIN, LOW);
  Serial.begin(115200);
  frameTimer.begin(tick, 10000);   // 10 000 us = 100 frames/s
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 's') {
    noInterrupts();
    frame = 0;
    running = true;
    interrupts();
    Serial.println("started");
  } else if (c == 'x') {
    running = false;
    Serial.printf("stopped: %lu pulses sent\n", (unsigned long)frame);
  }
}