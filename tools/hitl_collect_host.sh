#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# tools/hitl_collect_host.sh -- fly ONE HITL scenario on a Rust (src/avionics) board with the physics on the host
# (tools/hitl_host.py, doc/specs/sim-link.md) and collect it, the counterpart of hitl_collect.sh (plan B's on-board
# HITL + the Luckfox): the session(s) pulled off the board's card over Wi-Fi (board_pull.sh, which checks every file
# against the board's own listing and flags an empty stream), the BUILD stamp checked in recorder.log, the flight cut
# (recorder_flight.py) and plotted (recorder_flight_plots.py, when the plotly venv is there).
#
# Usage: hitl_collect_host.sh <board_id> <board_ip> <motor> <scenario> <noise> <wind> <wind_dir> <spike> [outdir]
#                             [glider_g] [inject_hz] [reboot]
#   e.g. hitl_collect_host.sh taster 192.168.102.152 F15 f15_clean 0.0 0.0 0.0 False /tmp/hitl_host/F15
#        hitl_collect_host.sh taster 192.168.102.152 F15 f15_reboot 0.0 0.0 0.0 False /tmp/hitl_host/F15 285 0 True
# CC goes through the running hub (src/control main.py) by <board_id>; the console only panics a reboot scenario.
# Env: PORT (default /dev/ttyACM0), the board's console.
# Prints one verdict line: OK <motor>/<scenario> session=<s> DONE, or FAIL <motor>/<scenario>: <why>.
set -e
PORT=${PORT:-/dev/ttyACM0}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PLOTLY=${PLOTLY:-$HOME/.local/share/pipx/venvs/plotly/bin/python}
board=$1; ip=$2; motor=$3; scen=$4; noise=$5; wind=$6; dir=$7; spike=$8; outdir=${9:-/tmp/hitl_host/$motor}
glider_g=${10:-285}; inject_hz=${11:-0}; reboot=${12:-False}
d="$outdir/$scen"; mkdir -p "$d"; rm -rf "${d:?}"/*
flags=()
[ "$spike" = True ] && flags+=(--spike)
[ "$reboot" = True ] && flags+=(--reboot)
out=$(python3 -u "$ROOT/tools/hitl_host.py" --ip "$ip" --hub "$board" --serial "$PORT" --motor "$motor" \
      --noise "$noise" --wind "$wind" --wind-dir "$dir" --glider-g "$glider_g" --inject-hz "$inject_hz" \
      "${flags[@]}" 2>&1) || true
echo "$out" > "$d/host.log"
fail() { echo "FAIL $motor/$scen: $*"; exit 1; }
ses=$(echo "$out" | grep -oE '^SESSION [0-9A-Za-z_-]+' | awk '{print $2}' | head -1)
[ -n "$ses" ] || fail "no SESSION line (see $d/host.log)"
if [ "$reboot" = True ]; then
  echo "$out" | grep -q '^WARM GATE: True' || fail "session=$ses $(echo "$out" | grep -oE 'WARM GATE: .*' | head -1)"
fi
echo "$out" | grep -qE '^(DONE|TIMEOUT)' || fail "session=$ses: no DONE or TIMEOUT (see $d/host.log)"
# every session the flight wrote: the first, and after a reboot the warm start's
sessions="$ses $(echo "$out" | grep -oE '^REBOOTED [0-9A-Za-z_-]+' | awk '{print $2}')"
for session in $sessions; do
  status=0
  bash "$ROOT/tools/board_pull.sh" "$ip" "$session" "$d/$session" >"$d/$session.pull.log" 2>&1 || status=$?
  [ "$status" = 0 ] || fail "session=$session: board_pull exit $status ($(tail -1 "$d/$session.pull.log"))"
  build=$(grep -ohE 'firmware avionics-[^ |]+' "$d/$session"/recordings/recorder.log 2>/dev/null | head -1)
  [ -n "$build" ] || fail "session=$session: no BUILD stamp in recorder.log"
done
python3 "$ROOT/tools/recorder_flight.py" "$d/$ses/recordings" --session "$ses" -o "$d/flight" >"$d/cut.log" 2>&1 \
  || fail "session=$ses: recorder_flight could not cut the flight ($(tail -1 "$d/cut.log"))"
if [ -x "$PLOTLY" ]; then
  "$PLOTLY" "$ROOT/tools/recorder_flight_plots.py" "$d/flight" >"$d/plots.log" 2>&1 || echo "  (no plots: $d/plots.log)"
fi
echo "OK $motor/$scen session=$ses $build $(echo "$out" | grep -oE '^(DONE|TIMEOUT [0-9]+)' | head -1)"
