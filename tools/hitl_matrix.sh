#!/bin/bash
# tools/hitl_matrix.sh -- run the full noise+wind+corner matrix for one motor on the board, then render.
# Deploys tools/hitl_run.py, flies the 12 TMS-7-basic scenarios (via hitl_collect.sh), assembles captures,
# and renders per-flight SVGs + the 5 plotly HTML reports + compare overlays into <outdir>.
#
# Usage: hitl_matrix.sh <F15|E16> [outdir]
# Env: PORT (default /dev/ttyACM0); PLOTLY_PY (python with plotly for the HTML reports; default python3).
set -e
PORT=${PORT:-/dev/ttyACM0}
PLY=${PLOTLY_PY:-python3}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
motor=$1; outdir=${2:-/tmp/hitl/$motor}
PAD=25.514379,-80.391795
ZONE=25.514944,-80.392972,25.514583,-80.391111
SCENARIOS='noise05 noise10 noise25 noise50 noise100 wind00 wind03 wind06 wind09 wind12 corner_spike corner_stress'

rshell -p "$PORT" -b 115200 --quiet cp "$ROOT/tools/hitl_run.py" /pyboard/hitl_run.py >/dev/null 2>&1
while read -r name noise wind dir spike; do
  [ -z "$name" ] && continue
  bash "$ROOT/tools/hitl_collect.sh" "$motor" "$name" "$noise" "$wind" "$dir" "$spike" "$outdir"
done <<'SCN'
noise05 0.05 0.0 210.0 False
noise10 0.10 0.0 210.0 False
noise25 0.25 0.0 210.0 False
noise50 0.50 0.0 210.0 False
noise100 1.00 0.0 210.0 False
wind00 0.10 0.0 210.0 False
wind03 0.10 3.0 210.0 False
wind06 0.10 6.0 210.0 False
wind09 0.10 9.0 210.0 False
wind12 0.10 12.0 210.0 False
corner_spike 0.10 0.0 210.0 True
corner_stress 0.50 12.0 210.0 True
SCN

for scen in $SCENARIOS; do
  [ -f "$outdir/$scen.txt" ] && python3 "$ROOT/tools/flight_svg.py" "$outdir/$scen.txt" \
    -o "$outdir/report_$scen.svg" --pad $PAD --zone $ZONE >/dev/null 2>&1 || true
done
for scen in corner_spike corner_stress noise05 noise50 wind00; do
  [ -f "$outdir/$scen.txt" ] && "$PLY" "$ROOT/tools/flight_report.py" "$outdir/$scen.txt" \
    -o "$outdir/report_$scen.html" --cdn >/dev/null 2>&1 || true
done
python3 "$ROOT/tools/flight_svg.py" "$outdir"/noise05.txt "$outdir"/noise10.txt "$outdir"/noise25.txt \
  "$outdir"/noise50.txt "$outdir"/noise100.txt --overlay -o "$outdir/compare_noise.svg" \
  --labels '5%,10%,25%,50%,100%' --pad $PAD --zone $ZONE >/dev/null 2>&1 || true
python3 "$ROOT/tools/flight_svg.py" "$outdir"/wind00.txt "$outdir"/wind03.txt "$outdir"/wind06.txt \
  "$outdir"/wind09.txt "$outdir"/wind12.txt --overlay -o "$outdir/compare_wind.svg" \
  --labels 'calm,3,6,9,12 m/s' --pad $PAD --zone $ZONE >/dev/null 2>&1 || true
echo "matrix $motor done -> $outdir"
