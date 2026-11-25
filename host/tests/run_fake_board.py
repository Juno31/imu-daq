"""Start the GUI server on a fake SID Lab board (no hardware): try the pages, Start / Stop, the spine view.

    python host/tests/run_fake_board.py [--data <folder>] [--port 8766]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import device                       # noqa: E402
import server                       # noqa: E402
from fake_sidlab_board import FakeSidlabSerial   # noqa: E402

device.serial.Serial = lambda *a, **k: FakeSidlabSerial(sd=True)
server.guess_port = lambda: "FAKE"
if "--port" not in sys.argv:
    sys.argv += ["--port", "8766"]
if "--no-browser" not in sys.argv:
    sys.argv.append("--no-browser")
server.main()
