# IMU DAQ — IMU acquisition with DAQ time sync

Two BNO085/BNO086 IMUs on a XIAO ESP32S3 (or ESP32C3), recorded from a browser GUI, with every
IMU frame time-locked to an NI DAQ through a SYNC pulse line.

## Install (Windows)

1. **Get the code:** `git clone <repo-url>` or *Code → Download ZIP* on GitHub and unzip.
2. **Python packages:** double-click **`setup.bat`**. It creates the conda env `daq` from
   `environment.yml` (or, without conda, a `.venv` from `requirements.txt`: numpy, pandas, pyserial, nidaqmx).
3. **NI-DAQmx driver** (only for the NI DAQ): install *NI-DAQmx* from ni.com (tested 26.8), or run
   `python -m nidaqmx installdriver` in the env. Check the device appears in NI MAX.
4. **Firmware** (once per board), in Arduino IDE 2:
   - Boards Manager: **esp32 by Espressif** (tested 3.3.5). Board *XIAO_ESP32S3*, USB Mode *Hardware CDC and JTAG*
     (or *XIAO_ESP32C3*).
   - Library Manager: **SparkFun BNO080 Cortex Based IMU** (tested 1.1.12).
   - Open `firmware/imu_daq/imu_daq.ino` and Upload.

   Or with arduino-cli:
   ```
   arduino-cli core install esp32:esp32
   arduino-cli lib install "SparkFun BNO080 Cortex Based IMU"
   arduino-cli compile --upload -p COM4 --fqbn esp32:esp32:XIAO_ESP32S3:USBMode=hwcdc firmware/imu_daq
   ```

## Start

1. Plug in the XIAO (and the NI DAQ if you use it). Close the Arduino Serial Monitor.
2. Double-click **`start.bat`**. The browser opens at http://127.0.0.1:8765.
3. Enter Subject / Trial, press **Start**, press **Stop**. Each recording is a folder in `data\`
   (not uploaded to git).

Keys in the browser: **M** = event marker, **Z** = zero the display (recorded data is never changed).
Closing the browser does not stop a recording; closing the server window does (it saves first).

## Hardware

| XIAO pin | S3 | C3 | to |
|---|---|---|---|
| 3V3 | | | IMU #1 VIN (never 5V) |
| GND | | | IMU #1 GND, DAQ AI 8 (AI0−) |
| D4 SDA | GPIO5 | GPIO6 | IMU #1 SDA |
| D5 SCL | GPIO6 | GPIO7 | IMU #1 SCL |
| SYNC | D1 GPIO2 | D2 GPIO4 | DAQ AI 0 (AI0+) |

IMU #2 is chained from IMU #1 by a STEMMA QT / Qwiic cable. Addresses: one board at **0x4B** (pod
`A4B`), one at **0x4A** (pod `A4A`).
Adafruit BNO085: default 0x4A; DI soldered to 3Vo → 0x4B (read only at power-up).
SparkFun BNO086: default 0x4B; ADR jumper closed → 0x4A.

## Firmware (`firmware/imu_daq/imu_daq.ino`)

Library: **SparkFun BNO080 Cortex Based IMU** v1.1.12. Board: XIAO_ESP32S3 (USB Mode: Hardware CDC
and JTAG) or XIAO_ESP32C3.

- A hardware timer (esp_timer) ticks every frame (100 Hz default; 200/50/25 Hz selectable). Each tick
  snapshots the newest quaternion of each IMU. Measured spacing: 10.000 ms ± 0.002 ms (100 Hz),
  5.000 ms ± 0.003 ms (200 Hz).
- **Idle:** preview frames, no SYNC pulses. **Recording:** frame counter restarts at 0 and every
  frame gets one SYNC pulse: **3 ms for frame 0**, 1 ms for every other frame.
- The IMUs report ~10 % faster than the frame rate, so ~100 % of frames carry a new reading.
- An IMU silent for 0.5 s is restarted automatically (retried every 1 s); frames keep ticking.
- USB output never blocks the timing; frames queue (512) when USB is briefly busy.

## Time sync — how a frame gets its times

| Column (imu_synced.csv) | Meaning | Accuracy |
|---|---|---|
| `t_us` | XIAO clock at the frame's SYNC rising edge | µs (board clock) |
| `daq_time_s` | DAQ time of that frame's SYNC edge (seconds from the DAQ's first sample) | 1 DAQ sample (50 µs at the default 20 kHz) |
| `pc_unix_s` | PC wall-clock time, from ping/pong clock sync (drift-corrected fit) | ~±0.5 ms |

The DAQ is started **before** the board; at Stop the server finds the wide frame-0 pulse, assigns
every later pulse to its frame by spacing (robust to a lost pulse and to clock drift), and reports
**SYNC PASS** only if every frame has exactly one pulse. EMG or other signals recorded as extra DAQ
channels (`Extra channels`, e.g. `ai1,ai2`) share the DAQ clock, so `daq_time_s` aligns them
sample-exactly with the IMU frames.

### Sync monitor (GUI, below the IMU cards)

- **SYNC line at the DAQ:** the last 5 frame periods of `ai0`, triggered on a pulse, each pulse labelled
  with the frame it was matched to. **Start marker** shows the first 5 frames (the 3 ms frame-0 pulse).
- **Pulse ↔ frame matching:** frames from the board vs pulses on the DAQ, live counts of frames without a
  pulse / extra pulses / lost frame lines, and a strip of the last 200 frames (green = matched).
- **Board clock ↔ DAQ clock:** DAQ edge time minus a running straight-line fit, per frame. Expect a
  slow sawtooth inside the ±½-sample band (the DAQ edge is quantized to 1 sample; the two clocks drift a
  few ppm). SD ≈ 14 µs at the default 20 kHz (29 µs at 10 kHz).
- **Board clock ↔ PC clock:** each USB ping with its round trip; filled dots are the fastest third, used
  for the `pc_unix_s` fit. Runs all the time, also when not recording.

## Session folder

| File | Content |
|---|---|
| `session.json` | subject, trial, notes, firmware/board info, DAQ settings |
| `imu.csv` | raw frames: frame, t_us, per IMU ok, fresh, w, x, y, z |
| `imu_synced.csv` | imu.csv + `pc_unix_s` (+ `daq_time_s` when the DAQ was recorded) |
| `events.csv` | start/stop, markers, IMU stalls/restarts, warnings (PC time + frame) |
| `clock_sync.csv` | every ping round trip used for the PC clock fit |
| `daq_raw.f32`, `daq_meta.json` | DAQ samples, float32, sample-major (n_samples × channels) |
| `daq_pulses.csv` | every detected SYNC pulse (rise time, width) |
| `summary.json` | quality report: rate, missing frames, % fresh per IMU, clock fit, SYNC check |

Read the DAQ file in Python:
```python
import json, numpy as np
m = json.load(open("daq_meta.json"))
x = np.fromfile("daq_raw.f32", dtype=np.float32).reshape(-1, len(m["channels"]))
t = np.arange(len(x)) / m["rate_hz"]          # same time base as daq_time_s
```

## Serial protocol (for other tools)

Board → PC: `P,seq,t_us,...` (preview), `F,frame,t_us,...` (recorded), `@HELLO`, `@START`, `@STOP`,
`@PONG`, `@EVENT`, `@STATUS`, `@POD`, `@RATE`, `@ACK`, `@ERR`, `# comments`.
PC → board: `hello`, `start`, `stop`, `ping <token>`, `status`, `restart`, `rate <5|10|20|40>`.
Details at the top of `imu_daq.ino`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| "COMx is in use by another program" | Close the Arduino Serial Monitor / other viewers; the server retries every 2 s |
| "no @HELLO from the board" | Upload `firmware/imu_daq` (older sketches use a different protocol) |
| An IMU shows NOT FOUND | Check its cable; for 0x4B on an Adafruit board check the DI–3Vo solder, then unplug/replug USB |
| DAQ not found | Check the DAQ in NI MAX, then press Refresh in the Time sync card |
| SYNC FAIL | Check AI0+/AI0− wiring and Input = Differential; see `daq_pulses.csv` and `summary.json` |

Command line: `python server.py --help` (web port, data folder, fixed serial port, no browser).
