#!/bin/bash
# tools/hitl_matrix.sh -- run the full noise+wind+corner matrix for one motor on the board, then render.
# Deploys tools/hitl_run.py, flies the 12 TMS-7-basic scenarios (via hitl_collect.sh), assembles captures,
# and renders per-flight SVGs + the 5 plotly HTML reports + compare overlays into <outdir>.
#
# Usage: hitl_matrix.sh <F15|E16> [outdir]
# Env: PORT (default /dev/ttyACM0); PLOTLY_PY (python with plotly for the HTML reports; default python3);
#      GLIDER_G (glide mass in grams -- TMS-7 v3: 285 = airframe + FULL payload, 235 = + HALF payload);
#      SCENARIOS (space-separated subset of the 12 below, to shorten a matrix).
# The motor and GLIDER_G together are the load/thrust combo: E16 carries 28.5 N.s, F15 49.7 N.s, so
# e16@285 is the worst combo, f15@285 typical, f15@235 the best.
set -e
PORT=${PORT:-/dev/ttyACM0}
# plotly lives in a pipx venv here, not in the system python. Resolve it ONCE rather than letting
# every report fail: the render loop used to swallow the failure with `|| true`, so a full 12-flight
# matrix produced zero HTML reports and still printed "matrix done".
PLY=${PLOTLY_PY:-}
if [ -z "$PLY" ]; then
  for candidate in python3 "$HOME"/.local/share/pipx/venvs/plotly/bin/python; do
    if command -v "$candidate" >/dev/null 2>&1 && "$candidate" -c 'import plotly' 2>/dev/null; then
      PLY=$candidate; break
    fi
  done
fi
[ -z "$PLY" ] && echo "WARNING: no python with plotly found -- HTML reports will be SKIPPED" >&2
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
motor=$1; outdir=${2:-/tmp/hitl/$motor}
GLIDER_G=${GLIDER_G:-285}
PAD=25.514379,-80.391795
ZONE=25.514944,-80.392972,25.514583,-80.391111
SCENARIOS=${SCENARIOS:-'noise05 noise10 noise25 noise50 noise100 wind00 wind03 wind06 wind09 wind12 corner_spike corner_stress'}

# Deploy the runner. NOT muted: this used to be `>/dev/null 2>&1`, and under `set -e` a wedged CDC
# killed the whole matrix here with an empty log and exit 1 -- no flights, no message, nothing to read.
if ! mpremote connect "$PORT" cp "$ROOT/tools/hitl_run.py" : ; then
  # RECOVER, do not just report. This wedge is routine when a matrix starts straight after heavy
  # mpremote use -- a deploy, the board suite, another matrix, or a killed client leaving DTR/RTS
  # asserted (board_unwedge.py's own docstring names that last one). It costs the WHOLE combo: ten
  # flights that never happened, and the operator finds out later from an empty directory. The cure is
  # scripted and takes seconds, so run it rather than printing its name.
  echo "upload failed -- unwedging the board and retrying once" >&2
  python3 "$ROOT/tools/board_unwedge.py" >/dev/null 2>&1 || true
  sleep 5
  if ! mpremote connect "$PORT" cp "$ROOT/tools/hitl_run.py" : ; then
    echo "FATAL: cannot upload hitl_run.py to $PORT -- board wedged? try tools/board_unwedge.py" >&2
    exit 1
  fi
  echo "recovered; continuing" >&2
fi
# The scenario list is fed on FD 3, not stdin, and the flight is given </dev/null.
# Both halves are needed: `mpremote run` inside hitl_collect.sh reads stdin, so on plain stdin it
# swallowed the REST of this heredoc after the first flight -- the matrix then flew exactly one
# scenario and printed "matrix done", a silent 1-of-12 that looks like a full run in the log.
while read -r name noise wind dir spike <&3; do
  [ -z "$name" ] && continue
  # SCENARIOS is the filter as well as the render list, so a shortened matrix does not try to render
  # the flights it never flew. The table below stays complete: a subset is a selection, not an edit.
  case " $SCENARIOS " in *" $name "*) ;; *) continue ;; esac
  # A failed flight must leave NO capture. hitl_collect.sh writes <scen>.txt only on success and clears
  # only its stream dir, so the PREVIOUS run's capture survived here and was rendered and counted as
  # this run's -- two builds mixed into one comparison, with nothing to say so.
  rm -f "$outdir/$name.txt" "$outdir/report_$name.svg" "$outdir/report_$name.html"
  bash "$ROOT/tools/hitl_collect.sh" "$motor" "$name" "$noise" "$wind" "$dir" "$spike" "$outdir" "$GLIDER_G" </dev/null \
    || echo "skip $motor/$name (flight failed)"   # one flaky flight must not abort the matrix
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

# Renders are best-effort -- a failed plot must not discard flights that cost board time -- but a
# failure is REPORTED. Swallowing it with `|| true` is how a matrix came to produce zero HTML reports
# and still look successful.
svg_bad=0
for scen in $SCENARIOS; do
  [ -f "$outdir/$scen.txt" ] || continue
  python3 "$ROOT/tools/flight_svg.py" "$outdir/$scen.txt" \
    -o "$outdir/report_$scen.svg" --pad $PAD --zone $ZONE >/dev/null 2>&1 || { svg_bad=$((svg_bad+1)); }
done
[ "$svg_bad" -gt 0 ] && echo "WARNING: $svg_bad SVG render(s) failed" >&2
html_bad=0
for scen in corner_spike corner_stress noise05 noise50 wind00; do
  [ -f "$outdir/$scen.txt" ] || continue
  [ -z "$PLY" ] && { html_bad=$((html_bad+1)); continue; }
  "$PLY" "$ROOT/tools/flight_report.py" "$outdir/$scen.txt" \
    -o "$outdir/report_$scen.html" --cdn >/dev/null 2>&1 || { html_bad=$((html_bad+1)); }
done
[ "$html_bad" -gt 0 ] && echo "WARNING: $html_bad HTML report(s) failed (PLY=${PLY:-none})" >&2
# The overlays are built from the flights that EXIST, not a hardcoded five. With SCENARIOS narrowing
# the matrix, naming absent captures made flight_svg fail -- and `|| true` swallowed it, so a shortened
# run silently lost its comparison chart. That is the same best-effort-hides-the-failure trap the
# per-scenario renders above were fixed for; an overlay of two curves is still worth drawing.
overlay() {   # overlay <out.svg> <label-for> <scenario>...
  local out=$1 kind=$2; shift 2
  local files=() labels=()
  for scen in "$@"; do
    [ -f "$outdir/$scen.txt" ] || continue
    files+=("$outdir/$scen.txt"); labels+=("${scen#$kind}")
  done
  [ "${#files[@]}" -lt 2 ] && return 0   # one curve is not a comparison
  python3 "$ROOT/tools/flight_svg.py" "${files[@]}" --overlay -o "$out" \
    --labels "$(IFS=,; echo "${labels[*]}")" --pad $PAD --zone $ZONE >/dev/null 2>&1 \
    || echo "WARNING: $(basename "$out") render failed" >&2
}
rm -f "$outdir/compare_noise.svg" "$outdir/compare_wind.svg"   # an overlay too short to redraw must not linger
overlay "$outdir/compare_noise.svg" noise noise05 noise10 noise25 noise50 noise100
overlay "$outdir/compare_wind.svg" wind wind00 wind03 wind06 wind09 wind12
# Count THIS run's scenarios, not every .txt in the directory, and fail when any is missing: "done"
# over a partial matrix is the silent-success trap the rest of this harness exists to prevent.
wanted=0; flights=0; missing=''
for scen in $SCENARIOS; do
  wanted=$((wanted + 1))
  if [ -f "$outdir/$scen.txt" ]; then flights=$((flights + 1)); else missing="$missing $scen"; fi
done
echo "matrix $motor ${GLIDER_G}g done -> $outdir  (flights: $flights of $wanted, svg: $(ls "$outdir"/*.svg 2>/dev/null | wc -l), html: $(ls "$outdir"/*.html 2>/dev/null | wc -l))"
if [ "$flights" -ne "$wanted" ]; then
  echo "FAIL: $((wanted - flights)) of $wanted scenario(s) produced no capture:$missing" >&2
  exit 1
fi
