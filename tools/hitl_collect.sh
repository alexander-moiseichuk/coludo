#!/bin/bash
# tools/hitl_collect.sh -- fly ONE HITL scenario on the board and collect it: board_reboot (clean VM ->
# fresh recorder session, the isolation boardrun gave us), run the launcher with `mpremote run`, adb-pull
# the Luckfox session, and assemble a capture .txt. Assumes tools/hitl_run.py is on the board (hitl_matrix
# deploys it). The capture timeline is flight-relative downstream, so the climbing soft-reboot uptime is fine.
#
# Usage: hitl_collect.sh <motor> <scenario> <noise> <wind> <wind_dir> <spike> [outdir] [glider_g] [inject_hz] [reboot_s] [no_cc] [attitude_drop_s] [gnss_drop_s]
#   e.g. hitl_collect.sh F15 wind12 0.10 12.0 210.0 False /tmp/hitl/F15
#        hitl_collect.sh F15 f15_full 0.05 0.0 210.0 False /tmp/hitl/mem 300 25   # weight/leak matrix
#   glider_g (default 300) + inject_hz (default 0 = sim_hz) drive the weight + memory-leak captures.
# Env: PORT (default /dev/ttyACM0).
set -e
PORT=${PORT:-/dev/ttyACM0}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
motor=$1; scen=$2; noise=$3; wind=$4; dir=$5; spike=$6; outdir=${7:-/tmp/hitl/$motor}
glider_g=${8:-285}; inject_hz=${9:-0}; reboot_s=${10:-0}; no_cc=${11:-False}; attitude_drop_s=${12:-0}
gnss_drop_s=${13:-0}   # seconds of GNSS blackout in the glide -> exercises the dead-reckoning tier
d="$outdir/$scen"; mkdir -p "$d"; rm -f "$d"/*
printf 'import hitl_run\nhitl_run.fly("%s", %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)\n' \
  "$motor" "$noise" "$wind" "$dir" "$spike" "$glider_g" "$inject_hz" "$reboot_s" "$no_cc" \
  "$attitude_drop_s" "$gnss_drop_s" > /tmp/launch.py
python3 "$ROOT/tools/board_reboot.py" "$PORT" >/dev/null 2>&1 || true   # clean VM -> fresh recorder session
out=$(timeout 190 mpremote connect "$PORT" run /tmp/launch.py 2>&1) || true   # a CDC wedge must not abort (set -e)
ses=$(echo "$out" | grep -oE 'SESSION [0-9_]+' | awk '{print $2}')
[ -z "$ses" ] && { echo "FAIL $motor/$scen: $(echo "$out" | tail -1)"; exit 1; }
# Pull EVERY stream this session wrote -- never a hardcoded list. The old fixed list silently dropped
# any stream added since it was written (flight.csv, the per-servo servo_*.csv, airspeed_sdp810), so a
# board capture was missing data the board had actually recorded and nothing said so.
# list-then-filter: the Luckfox shell does not expand a glob here, and its `ls` emits ANSI colour
# codes + CR, so strip both before matching or every name silently fails to match.
expected=$(adb shell "ls /userdata/recordings/" 2>/dev/null \
           | sed -e "s/\x1b\[[0-9;]*m//g" -e "s/\r//g" | grep "^${ses}_.*\.csv$")
for name in $expected; do
  adb pull "/userdata/recordings/$name" "$d/" >/dev/null 2>&1 || true
done
# VERIFY THE PULL. A capture missing streams still assembles into a file that looks like a whole
# flight, and every downstream tool renders it as one -- this exact class already cost two findings
# (§27.1's hardcoded stream list, §27.8's 5-stream fixture standing in for an 8-stream flight). The
# old check only caught "nothing at all"; a partial pull passed silently.
want=$(echo "$expected" | grep -c . || true)
pulled=$(ls "$d" | wc -l)
if [ "$pulled" -ne "$want" ]; then
  echo "FAIL $motor/$scen: pulled $pulled of $want streams for session $ses -- capture is INCOMPLETE"
  echo "  missing: $(for n in $expected; do [ -f "$d/$n" ] || echo -n "$n "; done)"
  exit 1
fi
[ "$want" -eq 0 ] && { echo "FAIL $motor/$scen: session $ses produced no streams"; exit 1; }
python3 "$ROOT/tools/assemble_capture.py" "$ses" "$d" "$outdir/$scen.txt" >/dev/null
echo "OK $motor/$scen session=$ses $(echo "$out" | grep -oE 'DONE|TIMEOUT [0-9]+' | head -1)"
