#!/usr/bin/env python3
# board_reboot.py [PORT] -- soft-reboot the ESP32-P4 to a clean VM before an `mpremote run` flight/test.
# This is the isolation boardrun's soft-reset used to give (boardrun is retired -- it was faulty). Sends
# Ctrl-B (leave a wedged raw REPL), Ctrl-C x2 (break a running app to the REPL), Ctrl-D (soft reset ->
# main.py re-runs), then waits for the boot.
#
# NOTE: a soft reset does NOT reset time.ticks_ms, so the recorder/capture uptime keeps climbing across
# runs. flight_video/flight_report make the timeline flight-relative (t=0 at the boosting stage), so a
# large absolute uptime is fine -- see flight_video.load().

import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
link = serial.Serial(port, 115200, timeout=0.3)
link.write(b'\x02')            # Ctrl-B: leave a stuck raw REPL
time.sleep(0.3)
link.write(b'\r\x03\x03')      # Ctrl-C x2: break a running app to the REPL
time.sleep(0.4)
link.write(b'\x04')            # Ctrl-D: soft reset -> re-run main.py
time.sleep(0.5)
link.close()
time.sleep(8)                  # let it boot + bring tasks up
print('rebooted', port)
