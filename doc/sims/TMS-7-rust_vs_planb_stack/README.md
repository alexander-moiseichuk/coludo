# TMS-7 — the software stack under the same HITL glide: plan B (MicroPython) vs the Rust avionics

**Same board, same motor, same glide: only the software differs.** All three flights are the taster, F15, 285 g,
the HPRC pad, clean air, flown hardware-in-the-loop with plan B's HITL test gains. Plan B's is its own HITL matrix
run r9 (`wind00`, 2026-08-30); the Rust runs are sessions 000684 and 000686 (2026-10-11), collected with
`tools/hitl_collect_host.sh`. The figures come from each stack's own recordings, from BOOSTING to DONE
(`tools/stack_compare.py`).

**One difference in the harness:** plan B's HITL ran the physics on the board itself (`tasks/hitl.py`), stepping at
~8.5 Hz and feeding its sensors at ~19 Hz. The Rust HITL runs the same physics (`sim_model.py`) on the host and sends
the board a sensor frame every 20 ms over UDP. So plan B's load includes its simulator, and its inputs were 2.5x
sparser.

## 1. The figures

| | plan B (r9, 08-30) | Rust run 1 (000684) | Rust run 2 (000686) |
|---|---|---|---|
| GLIDING (s) | 10.2 | 10.1 | 10.1 |
| DONE (s) | 98.4 | 112.0 | 112.1 |
| flight.csv samples (Hz) | 9.0 | 9.5 | 9.5 |
| flight.csv gap median / p99 / max (ms) | 108.3 / 223.4 / 232.5 | 109.9 / 110.1 / 110.3 | 109.9 / 110.1 / 110.1 |
| fin commands logged (Hz) | 18.7 | — | — |
| fin command gap median / p99 / max (ms) | 50.0 / 186.1 / 224.0 | — | — |
| sensor input (Hz) | 18.7 | 49.0 | 48.5 |
| sensor input gap median / p99 / max (ms) | 50.0 / 186.1 / 223.9 | 20.4 / 31.0 / 110.8 | 20.6 / 32.0 / 83.7 |
| die temperature min / median / max (°C) | 33.0 / 34.0 / 34.0 | 35.3 / 35.3 / 36.3 | 35.3 / 35.3 / 36.3 |
| free heap min / median / max (MB) | 6.9 / 25.5 / 32.4 | 23.1 / 23.1 / 23.2 | 23.1 / 23.1 / 23.2 |
| CPU load min / median / max (%) | 0 / 18 / 47 | 26 / 37 / 97 | 24 / 37 / 85 |

([`health_planb_vs_rust.html`](health_planb_vs_rust.html), [SVG](health_planb_vs_rust.svg): the load, the free heap
and the temperature over each flight)

The two Rust runs agree to the tenth of a second at every stage. DONE differs from plan B's r9 run (112 s against 98 s)
because r9 flew plan B's on-board physics at ~8.5 Hz steps. On the same host physics the two control stacks fly the
same glide (TMS-7-rust_hitl_m13: LANDING at 109.0 s on the board, plan B's at 110.1 s).

## 2. How fast it reacts: under 10 ms

The Rust flight law runs on the control thread, core 1, every 10 ms tick, paced by a hardware timer. Each tick reads
the newest inputs, runs the law and writes the fin pulses. Over the whole flight of run 2 (13 283 ticks, the `control`
command at DONE):

| | run 2, the whole flight | run 1, 78 s in |
|---|---|---|
| ticks missed (overruns) | 0 | 0 |
| tick lateness, max | 6 µs | 11 µs |
| ticks past the 200 µs budget | 0 | 0 |
| tick work, max (wake to fins written) | 475 µs | 523 µs |
| tick work, typical | 15–80 µs | 50 µs |

**So a new sensor value reaches the fin command within one tick: at most 10 ms + 0.5 ms, about 5 ms on average.**
The work leaves the core idle 95–99 % of each tick. In flight on the real sensors the IMU is read in the same tick, so
its sample is fresh to within that tick.

Plan B's law was set to a 100 Hz timer too, but what reached it was slower and less regular: its inputs came at
~19 Hz (50 ms gaps, p99 186 ms), its logged fin commands at the same rate with gaps up to 224 ms, and its flight
samples, due every 100 ms, waited up to 232 ms. The MicroPython scheduler and its garbage collector (the heap
swinging between 6.9 and 32.4 MB free; a collection pauses everything for ~67 ms) set its timing.

Both stacks then wait for the servo: the SG90 takes a new pulse width at its next 50 Hz frame (up to 20 ms), then
travels at its own speed (~0.1 s for 60°). From the tick on, that is the same hardware for both.

## 3. CPU, memory, temperature

- **CPU.** The Rust load figure is the busier of the two cores. A 10 s `load` on the taster at rest splits it:
  core 0 at 36.8 %, nearly all the recorder (the ring_eraser 15.0 %, the ring_saver 11.7 %, the ring_writer 7.6 %,
  the uart_saver 2.1 %); core 1, the flight core, at 4.4 % (the i2c0 sensors 2.4 %, the control thread 1.2 %). So the
  Rust flight code costs ~1 % of its core. The 37 % median in flight is core 0's recorder, Wi-Fi and the sim link; its
  spikes (85–97 %) are recorder bursts. Plan B's 18 % is its own scheduler's measure, simulator included: the two
  load columns are not like for like.
- **Memory.** Rust holds a flat 23.1–23.2 MB free through the flight: no garbage collector, nothing allocated per
  tick. Plan B's heap saws between 6.9 and 32.4 MB as MicroPython collects.
- **Temperature.** 35.3–36.3 °C against 33–34 °C, on different days with the ambient unrecorded. The ~2 °C is within
  what the room and the recorder's extra work explain.

## 4. What it points at

- **The ring_eraser polls the flash.** IDF yields during an erase only every 20 ms (one tick), so most of each 4 KB
  sector erase spins core 0: 15 % of it. Erasing 64 KB blocks, or a shorter poll, would give most of it back.
- **The reaction is the tick.** Reacting faster than 10 ms means a faster tick; the work (≤ 0.5 ms) allows up to
  ~1 kHz on core 1 before the work itself matters.

## 5. How to repeat it

```
tools/hitl_collect_host.sh taster 192.168.102.152 F15 f15_clean 0.0 0.0 0.0 False <outdir>   # prints CONTROL at DONE
~/.local/share/pipx/venvs/plotly/bin/python tools/stack_compare.py 'plan B=<capture prefix>' \
    'Rust=<outdir>/f15_clean/<session>/recordings/<session>' -o <figure>
```

`hitl_host.py` now prints the board's `control` line (`CONTROL ...`) before it restores the board, and the hub's
`/api/cmd` carries a reply's named figures (it dropped them before).
