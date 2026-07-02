#!/bin/bash
# tools/hitl_collect.sh -- fly ONE HITL scenario on the board and collect it: board_reboot (clean VM ->
# fresh recorder session, the isolation boardrun gave us), run the launcher with `mpremote run`, adb-pull
# the Luckfox session, and assemble a capture .txt. Assumes tools/hitl_run.py is on the board (hitl_matrix
# deploys it). The capture timeline is flight-relative downstream, so the climbing soft-reboot uptime is fine.
#
# Usage: hitl_collect.sh <motor> <scenario> <noise> <wind> <wind_dir> <spike> [outdir] [glider_g] [inject_hz]
#   e.g. hitl_collect.sh F15 wind12 0.10 12.0 210.0 False /tmp/hitl/F15
#        hitl_collect.sh F15 f15_full 0.05 0.0 210.0 False /tmp/hitl/mem 300 25   # weight/leak matrix
#   glider_g (default 300) + inject_hz (default 0 = sim_hz) drive the weight + memory-leak captures.
# Env: PORT (default /dev/ttyACM0).
set -e
PORT=${PORT:-/dev/ttyACM0}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
motor=$1; scen=$2; noise=$3; wind=$4; dir=$5; spike=$6; outdir=${7:-/tmp/hitl/$motor}
glider_g=${8:-300}; inject_hz=${9:-0}
d="$outdir/$scen"; mkdir -p "$d"; rm -f "$d"/*
printf 'import hitl_run\nhitl_run.fly("%s", %s, %s, %s, %s, %s, %s)\n' \
  "$motor" "$noise" "$wind" "$dir" "$spike" "$glider_g" "$inject_hz" > /tmp/launch.py
python3 "$ROOT/tools/board_reboot.py" "$PORT" >/dev/null 2>&1 || true   # clean VM -> fresh recorder session
out=$(timeout 135 mpremote connect "$PORT" run /tmp/launch.py 2>&1) || true   # a CDC wedge must not abort (set -e)
ses=$(echo "$out" | grep -oE 'SESSION [0-9_]+' | awk '{print $2}')
[ -z "$ses" ] && { echo "FAIL $motor/$scen: $(echo "$out" | tail -1)"; exit 1; }
for stream in accel_adxl375 baro_icp10111 imu_bno055 imu_lsm6dso32 gnss laser_agl fins health sequencer power_ina226; do
  adb pull "/userdata/recordings/${ses}_${stream}.csv" "$d/" >/dev/null 2>&1 || true
done
python3 "$ROOT/tools/assemble_capture.py" "$ses" "$d" "$outdir/$scen.txt" >/dev/null
echo "OK $motor/$scen session=$ses $(echo "$out" | grep -oE 'DONE|TIMEOUT [0-9]+' | head -1)"
