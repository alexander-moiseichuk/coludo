#!/usr/bin/env bash
# Integration test for the controller -> Recorder UART link.
#
# Writes a uniquely-marked line over UART from the ESP32-P4 (via mpremote) and confirms it
# landed on the Luckfox recorder (via adb). It targets a DEDICATED file using the recorder's
# `@file@line` routing, NOT the default recorder.log -- recorder.log is only committed to disk
# every 1000 lines, so a single line would not show up there.
#
# Needs: the board on USB (mpremote) AND the recorder on adb, with GPIO20 -> Luckfox RX wired.
# Env: PORT (default /dev/ttyACM0), REC_FILE (default uart_itest.log), TX/RX/BAUD.

set -u
PORT="${PORT:-/dev/ttyACM0}"
REC_FILE="${REC_FILE:-uart_itest.log}"
REC_PATH="/userdata/recordings/$REC_FILE"
TX="${TX:-20}"; RX="${RX:-21}"; BAUD="${BAUD:-921600}"

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; N=$'\e[0m'; else G=; R=; N=; fi

command -v mpremote >/dev/null || { echo "${R}error: mpremote not found${N}"; exit 2; }
command -v adb >/dev/null      || { echo "${R}error: adb not found${N}"; exit 2; }
adb get-state >/dev/null 2>&1  || { echo "${R}error: no adb device (recorder not connected?)${N}"; exit 2; }

# unique marker so we never match a stale line
marker="coludo-itest-$(date +%s)-${RANDOM}"
line="@${REC_FILE}@${marker}"

# settle the board into a clean raw-REPL state first (matches run_tests.sh)
mpremote connect "$PORT" reset >/dev/null 2>&1 || true
sleep 2

echo "UART(tx=$TX,rx=$RX)@${BAUD} -> writing: $line"
write() {
  mpremote connect "$PORT" exec \
    "from machine import UART; import time; u=UART(1, baudrate=$BAUD, tx=$TX, rx=$RX); u.write('${line}\n'); time.sleep_ms(200)"
}
write || { sleep 1; write; } || { echo "${R}FAIL${N}: mpremote write errored"; exit 1; }

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
