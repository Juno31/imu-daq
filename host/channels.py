"""DAQ channel map: which NI USB-6363 input records what.

SID Lab spine brace (handover 2026-11-25, see the senior's SETUP.md and config/sensors.json):
  * EMG single-ended (RSE, +-5 V): EMG 1..6 = AI16..AI21 = terminals 33, 36, 39, 42, 49, 52.
  * SYNC from Teensy pin 2 = AI31 (terminal 59). Its pulse width carries the frame number.
  * IMU data packets from Teensy pin 1 (TX1) = P0.0 (terminal 65), read as a digital line.
  * Teensy GND = D GND (terminal 82): required.
Edit this table if the electrode layout changes (sites can also be re-assigned per session).
"""

SYNC_CHAN = "ai31"
TERMINAL = "RSE"
DATA_LINE = "port0/line0"          # P0.0; "" = do not record the IMU data line

EMG_CHANNELS = [
    # (DAQ channel, name, muscle, side, group)
    ("ai16", "EMG1", "Longissimus, L1 (2 fingers lateral)", "R", "R-LG"),
    ("ai17", "EMG2", "Longissimus, L1 (2 fingers lateral)", "L", "L-LG"),
    ("ai18", "EMG3", "Multifidus, L5 (line PSIS - L1)", "R", "R-MF"),
    ("ai19", "EMG4", "Multifidus, L5 (line PSIS - L1)", "L", "L-MF"),
    ("ai20", "EMG5", "Iliocostalis, L2 (1 finger medial to iliac line)", "R", "R-IL"),
    ("ai21", "EMG6", "Iliocostalis, L2 (1 finger medial to iliac line)", "L", "L-IL"),
]

DEFAULT_EXTRA = ",".join(c[0] for c in EMG_CHANNELS)


def label(chan, sync_chan=SYNC_CHAN):
    """Description of one DAQ channel for metadata and the GUI."""
    if chan == sync_chan:
        return {"chan": chan, "name": "SYNC", "muscle": "", "side": "", "group": "IMU frame pulses"}
    for c, name, muscle, side, group in EMG_CHANNELS:
        if c == chan:
            return {"chan": chan, "name": name, "muscle": muscle, "side": side, "group": group}
    return {"chan": chan, "name": chan, "muscle": "", "side": "", "group": ""}
