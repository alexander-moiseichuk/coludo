#!/usr/bin/env bash
# Integration test for the controller -> Recorder UART link.
#
# Writes a uniquely-marked line over the recorder UART from the ESP32-P4 and confirms it landed
# on the Luckfox recorder (via adb). The UART pins/baud are taken from the board's OWN config
# (config.load(): board.json if present, else config_default) so this works on whatever board
# is currently attached. Targets a DEDICATED file via the `@file@line` routing, NOT the default
# recorder.log (which only commits to disk every 1000 lines).
#
# Needs: the board on USB (mpremote) AND the recorder on adb, with the recorder TX pin wired.
# Env: PORT (default /dev/ttyACM0), REC_FILE (default uart_itest.log).

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${PORT:-/dev/ttyACM0}"
REC_FILE="${REC_FILE:-uart_itest.log}"
REC_PATH="/userdata/recordings/$REC_FILE"

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; N=$'\e[0m'; else G=; R=; N=; fi

command -v mpremote >/dev/null || { echo "${R}error: mpremote not found${N}"; exit 2; }
command -v adb >/dev/null      || { echo "${R}error: adb not found${N}"; exit 2; }
adb get-state >/dev/null 2>&1  || { echo "${R}error: no adb device (recorder not connected?)${N}"; exit 2; }

# deploy modules so the board can read its own config to find the recorder pins
bash "$HERE/deploy_modules.sh" "$PORT" >/dev/null 2>&1 || true

# unique marker so we never match a stale line
marker="coludo-itest-$(date +%s)-${RANDOM}"

# the board picks its own recorder UART pins from config and sends the line
send_py="/tmp/coludo_itest_send.py"
cat > "$send_py" <<PY
import config, time
from machine import UART
cfg, src, errs = config.load()
b = cfg['buses']['uart_recorder']
u = UART(1, baudrate=b['baud'], tx=b['tx'], rx=b['rx'])
u.write('@${REC_FILE}@${marker}\n')
time.sleep_ms(200)
print('sent via %s config: tx=%d rx=%d baud=%d' % (src, b['tx'], b['rx'], b['baud']))
PY

echo "marker: $marker"
run() { mpremote connect "$PORT" run "$send_py"; }
run || { sleep 1; run; } || { echo "${R}FAIL${N}: mpremote send errored"; exit 1; }

sleep 1   # let the recorder receive + append

got="$(adb shell "grep -F '$marker' '$REC_PATH'" 2>/dev/null | tr -d '\r')"
if [ -n "$got" ]; then
    echo "${G}PASS${N}: found in $REC_PATH"
    echo "  $got"
    exit 0
else
    echo "${R}FAIL${N}: marker not found in $REC_PATH"
    echo "  last lines of $REC_PATH:"
    adb shell "tail -3 '$REC_PATH' 2>/dev/null" | sed 's/^/    /'
    exit 1
fi
