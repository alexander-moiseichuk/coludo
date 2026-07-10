# sims/ — flight visualisations

Rendered views of a HITL flight, for eyeballing how a run actually went (trajectory, attitude, fins,
memory, CPU, altitude) without reading raw telemetry. All are generated from one recorder capture, so
they are **derived artifacts** — regenerate them any time from a fresh flight (see below).

| file | what it is |
|------|------------|
| `flight.mp4` | follow-cam flythrough: 3D glider (wing deploy, elevon/rudder deflection), ground track + zone, and a live HUD panel — altitude, speed, attitude, **memory (MB), CPU load (%)**, v-speed, g-load, schedule. The pad sits low in frame so the climb has room above. |
| `flight_charts.html` | interactive report (plotly): the 3D trajectory + time-series rows — accel, altitude/elevation, speed, attitude, **fins (commanded)**, **board health (mem/load/temp)**, agl, engine (INA226), gyro rate. Open in a browser; drag to rotate the 3D view. |
| `flight_track.svg` | static top-down ground track + zone (no dependencies). |
| `flight_capture.txt` | the assembled recorder capture the two views are built from (the source data). |

## Regenerate from a new flight

```bash
# 1. fly a HITL scenario on the board (records to the Luckfox)
mpremote connect $PORT cp tools/hitl_run.py :
printf 'import hitl_run\nhitl_run.fly("F15", 0.10, 4.0, 210.0)\n' > /tmp/fly.py
tools/board_reboot.py $PORT && mpremote connect $PORT run /tmp/fly.py

# 2. pull + assemble + charts (needs adb to the Luckfox + the plotly venv)
tools/flight_pull.sh                 # latest session -> /tmp/flights/<session>/{txt,html,svg}

# 3. video (needs PIL + ffmpeg)
python3 tools/flight_video.py sims/flight.mp4 "F15 5% noise, 4 m/s wind" <capture.txt>
```

The heavy rendered files (`*.mp4`, `*.html`, `*.svg`, `*.txt`) are git-ignored — they are large and
per-run. Commit one deliberately only if you want a checked-in reference clip.
