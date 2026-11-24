// SD card log for imu_daq_teensy: one CSV file per recording on the Teensy 4.1's built-in SD card.
// The same F (frame) and S (sample) lines as on USB. The card is the complete record: while recording,
// a line that USB cannot take is skipped on USB only. The PC fetches the file after the recording ("sdget").
//
// Writing never blocks the main loop: lines go into a RAM ring buffer, and one 512-byte sector is written
// whenever the card is not busy. The next file is created and pre-allocated while idle ("armed"), so
// "start" only flips a flag. Uses SdFat, which comes with Teensyduino (nothing to install).
#pragma once
#include <SdFat.h>
#include <RingBuf.h>

const uint64_t SD_FILE_BYTES = 1024ULL << 20;   // pre-allocated per file: ~1.5 h at ~180 KB/s (6 IMUs, every sample)
const uint32_t SD_SYNC_MS = 1000;               // save the file every second: a power cut loses at most ~1 s
const size_t   SD_RB_BYTES = 128 * 1024;        // ~0.7 s of data: covers the card's occasional write pauses

SdFs   sd;
FsFile sdFile;
RingBuf<FsFile, SD_RB_BYTES> sdRb;
bool     sdOk = false, sdArmed = false, sdOpen = false;
char     sdName[16] = "", sdLast[16] = "";
uint32_t sdOverruns = 0, sdLines = 0, sdMaxUsed = 0, sdLastSyncMs = 0, sdMaxSyncUs = 0;
uint64_t sdBytes = 0;

// Create and pre-allocate the next IMUnnnn.CSV, so the next recording starts without delay.
void sdArm() {
  sdArmed = false;
  if (!sdOk) return;
  for (int n = 1; n < 10000; n++) {
    snprintf(sdName, sizeof sdName, "IMU%04d.CSV", n);
    if (!sd.exists(sdName)) break;
  }
  if (!sdFile.open(sdName, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.printf("# SD: cannot create %s (open failed, sd error 0x%02X)\n", sdName, sd.sdErrorCode());
    return;
  }
  if (!sdFile.preAllocate(SD_FILE_BYTES)) {      // FAT32 needs one contiguous free block; without it, record anyway
    Serial.printf("# SD: %s: could not reserve %lu MB in one piece; recording without a reservation\n",
                  sdName, (unsigned long)(SD_FILE_BYTES >> 20));
  }
  // The reserved space still holds old card data. Blank the first sector, so a file that never records
  // is recognised (and deleted) at the next power-up.
  static uint8_t zeros[512];
  if (sdFile.write(zeros, sizeof zeros) != sizeof zeros || !sdFile.sync() || !sdFile.seekSet(0)) {
    Serial.printf("# SD: cannot prepare %s\n", sdName);
    sdFile.close();
    return;
  }
  sdRb.begin(&sdFile);
  sdArmed = true;
}

// At power-up: delete files that were armed but never recorded; cut a half-written last line
// (power cut during a recording; the file is saved every second).
void sdCleanup() {
  FsFile dir, f;
  char nm[32];
  int removed = 0, cut = 0;
  if (!dir.open("/")) return;
  while (f.openNext(&dir, O_RDWR)) {
    f.getName(nm, sizeof nm);
    if (f.isDir() || strncmp(nm, "IMU", 3)) { f.close(); continue; }
    int c0 = f.read();
    if (c0 != '#') { f.remove(); removed++; continue; }        // never recorded (blank first sector)
    uint64_t size = f.fileSize();
    if (size >= SD_FILE_BYTES) {                              // FAT32 shows the whole reservation: not repaired here
      Serial.printf("# SD: %s was not closed (power cut?); copy it with a card reader and cut it by hand\n", nm);
      f.close(); continue;
    }
    uint64_t end = size;                                      // last complete line
    uint8_t ch = 0;
    while (end > 0) { f.seekSet(end - 1); ch = f.read(); if (ch == '\n') break; end--; }
    if (end < size) { f.truncate(end); cut++; }
    f.close();
  }
  dir.close();
  if (removed || cut) Serial.printf("# SD: removed %d unused file(s), cut %d unfinished line(s)\n", removed, cut);
}

void sdBegin() {
  if (sdOpen) return;                            // never while recording
  if (sdArmed) { sdFile.close(); sdArmed = false; }
  if (sdOk) { sd.end(); sdOk = false; }          // a second begin() on a running card fails (error 0x17)
  for (int k = 0; k < 4 && !sdOk; k++) {         // some cards fail the first start-up (e.g. CMD6): retry, both modes
    if (k) { sd.end(); delay(100); }
    sdOk = sd.begin(SdioConfig(k % 2 ? DMA_SDIO : FIFO_SDIO));
  }
  if (!sdOk) {
    uint8_t ec = sd.sdErrorCode();
    // error code 0 with a failed begin = the card answered but has no FAT32/exFAT file system (e.g. NTFS)
    Serial.printf("# SD: not usable (%s, sd error 0x%02X data 0x%02X); recording goes to USB only. "
                  "\"sdinit\" retries.\n", ec ? "no card, or the card does not answer" : "card found, but not FAT32/exFAT",
                  ec, (unsigned)sd.sdErrorData());
    return;
  }
  Serial.printf("# SD: card OK (%s, %lu MB free)\n", sd.fatType() == FAT_TYPE_EXFAT ? "exFAT" : "FAT32",
                (unsigned long)((uint64_t)sd.freeClusterCount() * sd.bytesPerCluster() >> 20));
  sdCleanup();
  sdArm();
}

// Lines of the current recording. Never blocks: a full buffer drops the line and counts it.
void sdWrite(const char* s, size_t n) {
  if (!sdOpen) return;
  if (sdRb.bytesFree() < n) { sdOverruns++; return; }
  sdRb.write(s, n);
  sdBytes += n; sdLines++;
}

bool sdStart() {                                 // called by "start"
  if (!sdArmed) return false;
  sdArmed = false; sdOpen = true;
  sdOverruns = sdLines = sdMaxUsed = sdMaxSyncUs = 0; sdBytes = 0;
  sdLastSyncMs = millis();
  return true;
}

// Called from loop(): write one sector when the card is free; save the file once a second.
void sdService() {
  if (!sdOpen) return;
  size_t n = sdRb.bytesUsed();
  if (n > sdMaxUsed) sdMaxUsed = n;
  if (n >= 512 && !sdFile.isBusy()) sdRb.writeOut(512);
  else if (millis() - sdLastSyncMs >= SD_SYNC_MS && !sdFile.isBusy()) {
    uint32_t t = micros();
    sdFile.sync();
    sdLastSyncMs = millis();
    if (micros() - t > sdMaxSyncUs) sdMaxSyncUs = micros() - t;
  }
}

void sdStop() {                                  // called by "stop", after the last lines were queued
  if (!sdOpen) return;
  sdRb.sync();
  sdFile.truncate();                             // drop the unused pre-allocated space
  sdFile.close();
  sdOpen = false;
  strcpy(sdLast, sdName);
  sdArm();
}

// "sdget NAME": send a file to the PC, every line prefixed with "D," so it never looks like a live line.
void sdSend(const char* name) {
  FsFile f;
  if (!sdOk || !f.open(name, O_RDONLY)) { Serial.printf("@ERR sd cannot open %s\n", name); return; }
  Serial.printf("@DUMP BEGIN name=%s bytes=%llu\n", name, (unsigned long long)f.fileSize());
  static uint8_t in[4096];
  static char out[8192];
  uint32_t lines = 0;
  bool lineStart = true;
  int n;
  while ((n = f.read(in, sizeof in)) > 0) {
    int o = 0;
    for (int i = 0; i < n; i++) {
      if (lineStart) { out[o++] = 'D'; out[o++] = ','; lineStart = false; }
      out[o++] = (char)in[i];
      if (in[i] == '\n') { lineStart = true; lines++; }
    }
    Serial.write((const uint8_t*)out, o);        // blocking is fine here: not recording
  }
  if (!lineStart) Serial.write("\r\n");
  f.close();
  Serial.printf("@DUMP END name=%s lines=%lu\n", name, (unsigned long)lines);
}

// "sdls": the IMU*.CSV files on the card.
void sdList() {
  if (!sdOk) { Serial.println("@SDLS card=0"); return; }
  FsFile dir, f;
  char nm[32];
  Serial.print("@SDLS card=1 files=");
  bool first = true;
  if (dir.open("/")) {
    while (f.openNext(&dir, O_RDONLY)) {
      f.getName(nm, sizeof nm);
      if (!f.isDir() && !strncmp(nm, "IMU", 3) && strcmp(nm, sdArmed ? sdName : "")) {
        Serial.printf("%s%s:%llu", first ? "" : ",", nm, (unsigned long long)f.fileSize());
        first = false;
      }
      f.close();
    }
    dir.close();
  }
  Serial.println();
}
