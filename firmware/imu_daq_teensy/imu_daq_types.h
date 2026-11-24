// Types for imu_daq_teensy.ino (in a header so the IDE's generated prototypes can use them).
#pragma once
#include <Arduino.h>
#include "src/teensy4_i2c/i2c_driver.h"
#include "src/teensy4_i2c/imx_rt1060/imx_rt1060_i2c_driver.h"

#define RXMAX 256                      // teensy4_i2c reads at most 256 bytes per transfer

enum BusOp : uint8_t { OP_NONE, OP_SELECT, OP_HDR, OP_BODY, OP_WRITE };

struct Bus {
  const char*           name;
  IMX_RT1060_I2CMaster* m;
  int                   sda, scl;
  uint32_t              recoveries;      // times SDA was found stuck low and freed by resetBus()
  uint8_t               muxCh;         // selected mux channel, 0xFF = unknown
  BusOp                 op;
  int                   cur;           // pod being visited, -1 = none
  int                   rr;            // round-robin position
  uint8_t               pkts;          // packets read in this visit
  uint32_t              opStartUs, visitStartUs;
  uint16_t              want;
  uint8_t               tx[24];
  uint8_t               hdr[4];
  uint8_t               rx[RXMAX];
  // load statistics (bus time spent in transfers)
  volatile uint32_t     busyAcc;       // us since the last frame tick
  uint32_t              nFrames, busyMaxUs, over;
  uint64_t              busySumUs;
  uint32_t              errors, timeouts;
};
enum PodState : uint8_t { P_DOWN, P_BOOT, P_DRAIN, P_IDREQ, P_IDWAIT, P_ENABLE, P_LIVE };
const uint8_t N_EN_STEPS = 5;          // other quat off, quat, acc, gyro, mag

struct Pod {
  const char* name;
  uint8_t     bus, ch;
  uint8_t     addr;         // 0x4B (SparkFun default) or 0x4A (ADR closed / Adafruit); found by trying both
  PodState    state;
  bool        ok;           // found at least once
  bool        failNoted;
  bool        reconfig;     // ENABLE because of a rate/mode change (no restart event)
  uint8_t     enStep;
  uint8_t     seq[6];       // SHTP sequence number per channel
  uint32_t    tState, setupStartMs, lastTryMs, lastFreshMs, failNoteMs;
  uint32_t    nextPollUs;
  uint32_t    updates, stalls, restarts, maxPollUs, framesFresh;
  uint32_t    nAcc, nGyr, nMag;
  float       headAccRad;   // rotation vector heading accuracy (rad), -1 = n/a
  uint8_t     qStat;
  // diagnostics since the pod went live (printed with a stall)
  uint32_t    pktCh[6];     // packets per SHTP channel
  uint32_t    empties;      // polls with nothing waiting
  uint8_t     last[8];      // first bytes of the last packet
  uint32_t    reads, conts; // reads while live; continuation packets joined to the one before
  bool        contOk;       // the last input packet was cut by the read length: a continuation follows
  uint8_t     tailLen;      // bytes of a report cut in two, kept until its continuation arrives
  uint8_t     tail[64];
  uint16_t    readLen;      // bytes per read while live (follows the packet size)
};
