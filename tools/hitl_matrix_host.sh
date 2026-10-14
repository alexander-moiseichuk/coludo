#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# tools/hitl_matrix_host.sh -- the 12-scenario HITL matrix for one motor, flown by a Rust board against the host's
# physics (tools/hitl_collect_host.sh per scenario: fly, pull, cut, plot), the counterpart of hitl_matrix.sh (plan B's
# on-board HITL). The scenario table is hitl_matrix.sh's, byte for byte (src/control/test/test_tools.py checks it).
#
# Usage: hitl_matrix_host.sh <board_id> <ip> <E16|F15> <outdir>
# Env: PORT (default /dev/ttyACM0), the board's console; GLIDER_G (default 285, the full build);
#      SCENARIOS (space-separated subset of the 12 below).
# Each scenario lands in <outdir>/<scenario>/ (host.log, the pulled session, flight/ the cut). Prints one line per
# flight and a verdict; exits 1 when any scenario produced no cut flight (a "done" over a partial matrix is the
# silent-success trap: count the artefacts, never trust a success line).
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
board=$1; ip=$2; motor=$3; outdir=$4
[ -n "$outdir" ] || { echo "usage: hitl_matrix_host.sh <board_id> <ip> <E16|F15> <outdir>" >&2; exit 2; }
GLIDER_G=${GLIDER_G:-285}
SCENARIOS=${SCENARIOS:-'noise05 noise10 noise25 noise50 noise100 wind00 wind03 wind06 wind09 wind12 corner_spike corner_stress'}
mkdir -p "$outdir"
while read -r name noise wind dir spike <&3; do
  [ -z "$name" ] && continue
  case " $SCENARIOS " in *" $name "*) ;; *) continue ;; esac
  # a failed flight leaves no cut behind: the previous run's must not be counted as this one's
  rm -rf "${outdir:?}/$name"
  bash "$ROOT/tools/hitl_collect_host.sh" "$board" "$ip" "$motor" "$name" "$noise" "$wind" "$dir" "$spike" \
    "$outdir" "$GLIDER_G" </dev/null || echo "skip $motor/$name (flight failed)"
done 3<<'SCN'
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
wanted=0; flights=0; missing=''
for scen in $SCENARIOS; do
  wanted=$((wanted + 1))
  if [ -f "$outdir/$scen/flight/flight/sequencer.csv" ]; then flights=$((flights + 1)); else missing="$missing $scen"; fi
done
echo "matrix $motor ${GLIDER_G}g -> $outdir (flights: $flights of $wanted)"
if [ "$flights" -ne "$wanted" ]; then
  echo "FAIL: $((wanted - flights)) of $wanted scenario(s) produced no cut flight:$missing" >&2
  exit 1
fi
