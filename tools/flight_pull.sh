#!/bin/bash
# tools/flight_pull.sh -- pull ONE real-flight recorder session off the Luckfox and render it in one
# command: adb-pull every stream, assemble the interleaved capture, and produce the interactive report
# (HTML) + SVG + KPIs. The FIELD counterpart to hitl_collect.sh -- that one also FLIES a HITL sim;
# here the flight already happened, so this is just the pull + assemble + report chain (plan CC item 11,
# which was hand-run per stream). Needs `adb` to the Luckfox and (for the HTML) the plotly venv.
#
# Usage: flight_pull.sh [session] [outdir]
#   session : recorder session id (default: the LATEST session on the Luckfox)
#   outdir  : output directory (default: /tmp/flights/<session>)
# Env: PAD=lat,lon  ZONE=tl_lat,tl_lon,br_lat,br_lon  (optional -- drawn on the SVG; default HPRC)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REC=/userdata/recordings
PAD=${PAD:-25.514379,-80.391795}
ZONE=${ZONE:-25.514944,-80.392972,25.514583,-80.391111}
PLY=${PLY:-$HOME/.local/share/pipx/venvs/plotly/bin/python}
STREAMS="accel_adxl375 baro_icp10111 baro_bmp280 imu_bno055 imu_lsm6dso32 gnss laser_agl fins health sequencer power_ina226"

command -v adb >/dev/null || { echo "error: adb not found (need the Luckfox recorder)"; exit 2; }

ses=${1:-}
if [ -z "$ses" ]; then                       # default: the newest session (by its health stream)
  ses=$(adb shell "ls -t $REC/*_health.csv 2>/dev/null | head -1" | sed "s|.*/||; s|_health.csv.*||" | tr -d '\r')
  [ -z "$ses" ] && { echo "error: no recorder session found on the Luckfox"; exit 1; }
  echo "latest session: $ses"
fi
out=${2:-/tmp/flights/$ses}
mkdir -p "$out"; rm -f "$out"/*.csv

n=0
for stream in $STREAMS; do
  adb pull "$REC/${ses}_${stream}.csv" "$out/" >/dev/null 2>&1 && n=$((n + 1))
done
[ "$n" -eq 0 ] && { echo "error: session $ses has no streams on the Luckfox"; exit 1; }
echo "pulled $n streams"

cap="$out/$ses.txt"
python3 "$ROOT/tools/assemble_capture.py" "$ses" "$out" "$cap" >/dev/null || { echo "assemble failed"; exit 1; }
echo "assembled $cap"

python3 "$ROOT/tools/flight_svg.py" "$cap" -o "$out/$ses.svg" --title "Coludo flight $ses" \
  --pad "$PAD" --zone "$ZONE" >/dev/null 2>&1 && echo "svg    $out/$ses.svg"
[ -x "$PLY" ] && "$PLY" "$ROOT/tools/flight_report.py" "$cap" -o "$out/$ses.html" --cdn >/dev/null 2>&1 \
  && echo "report $out/$ses.html"
echo "--- KPIs ---"
python3 "$ROOT/tools/flight_kpi.py" "$ses:$cap" 2>/dev/null || true
