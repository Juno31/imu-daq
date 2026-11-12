# IMU DAQ — IMU + EMG acquisition with DAQ time sync

Two BNO085/BNO086 IMUs on a XIAO ESP32S3 (or ESP32C3) and 8 EMG channels on an NI USB-6363, recorded
from one browser GUI. Every IMU frame is time-locked to the DAQ by a SYNC pulse wire, so IMU and EMG
share one time base (≈ 14 µs).

```
 XIAO ESP32S3 ── I2C ── BNO085 A4B (0x4B) ── STEMMA QT ── BNO085 A4A (0x4A)
   │ USB (frames)        │ SYNC pulse per frame (D1 → AI0)
   ▼                     ▼
 PC: host/server.py ◄── NI USB-6363 (AI0 = SYNC, AI16–23 = EMG01–08)
   │ HTTP + live stream
   ▼
 Browser GUI  http://127.0.0.1:8765
```

## Install (Windows)

1. **Get the code:** `git clone https://github.com/Juno31/imu-daq.git` (or *Code → Download ZIP*).
2. **Python packages:** double-click **`setup.bat`**. It creates the conda env `daq` from
   `environment.yml`, or without conda a `.venv` from `requirements.txt` (numpy, pandas, pyserial, nidaqmx).
3. **NI-DAQmx driver:** install *NI-DAQmx* from ni.com (tested 26.8). The DAQ must show in **NI MAX**.
   A USB-6363 needs its **power adapter switched on**; USB alone does not power it.
4. **Firmware** (once per board), Arduino IDE 2:
   - Boards Manager: **esp32 by Espressif** (tested 3.3.5). Board *XIAO_ESP32S3*, USB Mode
     *Hardware CDC and JTAG* (or *XIAO_ESP32C3*).
   - Library Manager: **SparkFun BNO080 Cortex Based IMU** (tested 1.1.12).
   - Open `firmware/imu_daq/imu_daq.ino` → Upload.

   Or with arduino-cli:
   ```
   arduino-cli core install esp32:esp32
   arduino-cli lib install "SparkFun BNO080 Cortex Based IMU"
   arduino-cli compile --upload -p COM4 --fqbn esp32:esp32:XIAO_ESP32S3:USBMode=hwcdc firmware/imu_daq
   ```
5. Optional desktop shortcut: right-click `make_shortcut.ps1` → *Run with PowerShell*.

## Use

1. Plug in the XIAO and the DAQ. Close the Arduino Serial Monitor (it blocks the COM port).
2. Double-click **`start.bat`** (or the **IMU DAQ** shortcut). The browser opens the GUI.
   Only one server can run: a second launch just opens the browser. To restart, **close the black server
   window first**.
3. Check the four status lights are green, enter Subject / Trial, press **Space** (or Start), and
   **Space** again to stop. A banner reports frames, missing frames and **SYNC PASS / FAIL**.

Closing the browser does not stop a recording; closing the server window does (it saves first).

### The GUI

| Part | What it shows |
|---|---|
| **Status lights** | DAQ · IMU · EMG · Sync, one word each (green / yellow / red). Click for detail. |
| **Recording bar** | Subject, trial, notes, Start/Stop, marker, frame and missing counters |
| **IMU** | per IMU: 3D board, roll/pitch/yaw, 5/10/30 s plot, % new data, stalls |
| **EMG** | 8 channels in L/R pairs: last 2 s envelope, RMS (100 ms), clipping warning, scale |
| **Sync detail** | SYNC scope with frame numbers, pulse↔frame match strip, board↔DAQ and board↔PC timing |
| **Event log** | start/stop, markers, IMU restarts, warnings |
| **Saved recordings** (right side) | latest recordings with SYNC result and size; click for files, click a file to open it |
| **⚙ Settings** | DAQ device/channels/rate/range, board port, IMU rate, clock statistics |
| **Spine view ↗** | calibrated 3D spine model from the two IMUs (separate tab) |

Sections open and close by clicking their header; the browser remembers the layout.
While not recording the DAQ runs a **preview** (nothing saved) so the DAQ/EMG lights and EMG traces work
before Start; turn it off with **Preview** in the EMG section if another program needs the DAQ.

### Keyboard shortcuts (press `?` in the GUI)

| Key | Action |
|---|---|
| `Space` | Start / stop recording (a second press within 1 s is ignored) |
| `M` | Event marker (uses the marker text) |
| `Z` | Zero the IMU display (recorded data unchanged) |
| `1` `2` `3` `4` | Open / close IMU · EMG · Sync detail · Event log |
| `S` | Settings |
| `Esc` | Leave a text field (keys are off while typing) · close panels |

## Hardware

| XIAO pin | S3 | C3 | to |
|---|---|---|---|
| 3V3 | | | IMU #1 VIN (never 5 V) |
| GND | | | IMU #1 GND, DAQ AI 8 (AI0−) |
| D4 SDA | GPIO5 | GPIO6 | IMU #1 SDA |
| D5 SCL | GPIO6 | GPIO7 | IMU #1 SCL |
| SYNC | D1 GPIO2 | D2 GPIO4 | DAQ AI 0 (AI0+) |

IMU #2 is chained from IMU #1 with a STEMMA QT / Qwiic cable. Addresses: the IMU wired to the XIAO is
**0x4B** (pod `A4B`), the chained one **0x4A** (pod `A4A`).
Adafruit BNO085: default 0x4A; DI soldered to 3Vo → 0x4B (read only at power-up).
SparkFun BNO086: default 0x4B; ADR jumper closed → 0x4A.

**EMG** (differential, Input = Differential): channel `aiN` uses pins AI N (+) and AI N+8 (−).

| Channel | Name | Muscle | Side |
|---|---|---|---|
| ai16 / ai17 | EMG01 / EMG02 | Lumbar erector spinae | L / R |
| ai18 / ai19 | EMG03 / EMG04 | Lumbar multifidus region | L / R |
| ai20 / ai21 | EMG05 / EMG06 | External oblique | L / R |
| ai22 / ai23 | EMG07 / EMG08 | Internal oblique | L / R |

Edit `host/channels.py` to change the map.

## How it works

**Firmware** (`firmware/imu_daq/imu_daq.ino`):
- A hardware timer ticks every frame (100 Hz default; 200/50/25 Hz selectable) and snapshots the newest
  quaternion of each IMU, so I2C work never affects timing (10.000 ± 0.002 ms at 100 Hz).
- **Idle:** preview frames, no SYNC pulses. **Recording:** frame counter restarts at 0 and each frame gets
  one SYNC pulse, **3 ms for frame 0**, 1 ms for all others.
- IMUs report ~10 % faster than the frame rate (≈ 100 % new data); a silent IMU is restarted after 0.5 s.
- USB output never blocks; frames queue (512) if the PC is briefly slow.

**Host** (`host/`):

| File | Role |
|---|---|
| `server.py` | state machine, start/stop sequence, HTTP API + live stream, DAQ preview, single instance |
| `device.py` | serial reader thread, line parser, 64-bit board clock |
| `daq.py` | NI-DAQmx reader thread, raw file, SYNC edge detection, 2 s live buffer, `align()` |
| `syncmon.py` | live pulse↔frame matching for the Sync detail view |
| `clocksync.py` | board↔PC clock fit from 1 Hz ping/pong (fastest third of pings) |
| `session.py` | session folder and files |
| `channels.py` | EMG channel map |
| `spine_angles.py`, `static/spine.*` | Spine view |
| `static/index.html, app.js, app.css` | GUI (plain canvas, no libraries) |

**Start:** session folder → DAQ starts first → board `start` (frame 0 + 3 ms pulse).
**Stop:** board `stop` (queued frames sent) → DAQ stops → pulses matched to frames → files written.

### Time sync

| Column (`imu_synced.csv`) | Meaning | Accuracy |
|---|---|---|
| `t_us` | XIAO clock at the frame's SYNC rising edge | µs (board clock) |
| `daq_time_s` | DAQ time of that edge (s from the DAQ's first sample) | 1 sample: 50 µs at 20 kHz (SD ≈ 14 µs) |
| `pc_unix_s` | PC wall-clock time from the ping fit | ≈ ±0.5 ms |

At Stop the server finds the wide frame-0 pulse and numbers each later pulse by its spacing to the
previous one (a lost pulse leaves one gap, it does not shift later frames). **SYNC PASS** = every frame
has exactly one pulse. EMG is sampled on the same DAQ clock, so `daq_time_s` lines it up with the IMU.

The DAQ default rate is **20 kHz**: edge timing SD ≈ 14 µs (vs 29 µs at 10 kHz), pulses clearly resolved
(1 ms = 20 samples), ample for EMG; 9 channels = 180 kS/s, well within the USB-6363.

## Session folder (`data/<date>_<time>_<subject>_<trial>/`, not in git)

| File | Content |
|---|---|
| `imu_synced.csv` | **main file**: IMU frames + `pc_unix_s` + `daq_time_s` |
| `imu.csv` | raw frames: frame, t_us, per IMU ok, fresh, w, x, y, z |
| `daq_raw.f32`, `daq_meta.json` | DAQ samples float32, sample-major (n × channels); channels, labels, rate |
| `daq_pulses.csv` | every SYNC pulse (rise, width) |
| `events.csv` | start/stop, markers, IMU stalls/restarts, warnings |
| `clock_sync.csv` | every ping used for the PC clock fit |
| `session.json` | subject, trial, notes, firmware, DAQ settings |
| `summary.json` | quality report: rate, missing frames, % new data, clock fit, SYNC check |
| `spine_cal.json` | Spine view calibration (if made) |

Read the DAQ file in Python:
```python
import json, numpy as np
m = json.load(open("daq_meta.json"))
x = np.fromfile("daq_raw.f32", dtype=np.float32).reshape(-1, len(m["channels"]))
t = np.arange(len(x)) / m["rate_hz"]               # same time base as daq_time_s
emg01 = x[:, m["channels"].index("ai16")]          # names in m["channel_labels"]
```

## Serial protocol (for other tools)

Board → PC: `P,seq,t_us,...` (preview), `F,frame,t_us,...` (recorded), `@HELLO`, `@START`, `@STOP`,
`@PONG`, `@EVENT`, `@STATUS`, `@POD`, `@RATE`, `@ACK`, `@ERR`, `# comments`.
PC → board: `hello`, `start`, `stop`, `ping <token>`, `status`, `restart`, `rate <5|10|20|40>`.
Details at the top of `imu_daq.ino`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| "Server not running" banner | Start `start.bat`; check its window for an error |
| Changes not showing after an update | Close the old server window, then start again; F5 in the browser |
| "COMx is in use by another program" | Close the Arduino Serial Monitor / other viewers; the server retries every 2 s |
| "no @HELLO from the board" | Upload `firmware/imu_daq` (older sketches use a different protocol) |
| IMU light red / NOT FOUND | Check the cable; for 0x4B on an Adafruit board check the DI–3Vo solder, then replug USB |
| DAQ light "Not found" | Power adapter + switch on the USB-6363, check NI MAX, then Refresh in Settings |
| EMG "Clipping" | Raise Range to ±10 V or lower the amplifier gain |
| SYNC FAIL | Check AI0+/AI0− wiring and Input = Differential; see `daq_pulses.csv`, `summary.json` |

Note: unconnected EMG inputs still read ~1.5–2 mV RMS, so a flat-signal check cannot detect a missing
electrode; check that the trace rises when the muscle contracts.

Command line: `python host/server.py --help` (web port, data folder, fixed serial port, no browser).
