#!/bin/bash
# tools/hitl_collect.sh -- fly ONE HITL scenario on the board and collect it: deploy a launcher, run it
# (boardrun runfile soft-resets first, clearing the module cache), adb-pull the Luckfox session, and
# assemble a capture .txt. Assumes tools/hitl_run.py is already on the board (hitl_matrix.sh deploys it).
#
# Usage: hitl_collect.sh <motor> <scenario> <noise> <wind> <wind_dir> <spike> [outdir]
#   e.g. hitl_collect.sh F15 wind12 0.10 12.0 210.0 False /tmp/hitl/F15
# Env: PORT (default /dev/ttyACM0).
set -e
PORT=${PORT:-/dev/ttyACM0}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
motor=$1; scen=$2; noise=$3; wind=$4; dir=$5; spike=$6; outdir=${7:-/tmp/hitl/$motor}
d="$outdir/$scen"; mkdir -p "$d"; rm -f "$d"/*
printf 'import hitl_run\nhitl_run.fly("%s", %s, %s, %s, %s)\n' "$motor" "$noise" "$wind" "$dir" "$spike" > /tmp/launch.py
rshell -p "$PORT" -b 115200 --quiet cp /tmp/launch.py /pyboard/launch.py >/dev/null 2>&1
out=$(timeout 135 python3 "$ROOT/tools/boardrun.py" "$PORT" runfile launch.py 125 2>&1)
ses=$(echo "$out" | grep -oE 'SESSION [0-9_]+' | awk '{print $2}')
[ -z "$ses" ] && { echo "FAIL $motor/$scen: $(echo "$out" | tail -1)"; exit 1; }
for stream in accel_adxl375 baro_icp10111 imu_bno055 gnss laser_agl fins health sequencer; do
  adb pull "/userdata/recordings/${ses}_${stream}.csv" "$d/" >/dev/null 2>&1 || true
done
python3 "$ROOT/tools/assemble_capture.py" "$ses" "$d" "$outdir/$scen.txt" >/dev/null
echo "OK $motor/$scen session=$ses $(echo "$out" | grep -oE 'DONE|TIMEOUT [0-9]+' | head -1)"
