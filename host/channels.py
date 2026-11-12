"""EMG channel map: which DAQ input records which muscle.

Differential wiring on the USB-6363: channel aiN uses pin AI N (+) and AI N+8 (-),
e.g. ai16 = AI 16 (+) / AI 24 (-). Edit this table if the electrode layout changes.
"""

EMG_CHANNELS = [
    # (DAQ channel, name, muscle, side, group)
    ("ai16", "EMG01", "Lumbar erector spinae", "L", "Lumbar extensor group"),
    ("ai17", "EMG02", "Lumbar erector spinae", "R", "Lumbar extensor group"),
    ("ai18", "EMG03", "Lumbar multifidus region", "L", "Deep lumbar extensor/stabilizer group"),
    ("ai19", "EMG04", "Lumbar multifidus region", "R", "Deep lumbar extensor/stabilizer group"),
    ("ai20", "EMG05", "External oblique", "L", "Abdominal oblique group"),
    ("ai21", "EMG06", "External oblique", "R", "Abdominal oblique group"),
    ("ai22", "EMG07", "Internal oblique", "L", "Internal oblique"),
    ("ai23", "EMG08", "Internal oblique", "R", "Internal oblique"),
]

DEFAULT_EXTRA = ",".join(c[0] for c in EMG_CHANNELS)


def label(chan, sync_chan="ai0"):
    """Description of one DAQ channel for metadata and the GUI."""
    if chan == sync_chan:
        return {"chan": chan, "name": "SYNC", "muscle": "", "side": "", "group": "IMU frame pulses"}
    for c, name, muscle, side, group in EMG_CHANNELS:
        if c == chan:
            return {"chan": chan, "name": name, "muscle": muscle, "side": side, "group": group}
    return {"chan": chan, "name": chan, "muscle": "", "side": "", "group": ""}
