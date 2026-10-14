# TMS-7 — the Rust board's first HITL flights (M12)

**The Rust flight firmware (`src/avionics`) flies hardware-in-the-loop against the same physics plan B
flew against** — `src/glider/sim_model.py`, now on the host (`tools/hitl_host.py`) instead of on the board,
over the UDP sim-link (`doc/specs/sim-link.md`). The board runs its real stage machine on the simulated
sensors, records them under the real sensors' streams, and answers every frame with its stage. M12 flies
**passive**: no servo is ported yet (M13), so the fins stay neutral and the glide runs straight.

All flights: the taster (v1.1 board), F15, clean (noise 0, no wind), 285 g glider (booster + glider at boost),
the HPRC pad. Collected with `tools/hitl_collect_host.sh` (fly, pull the card over Wi-Fi, check every file
against the board's listing, the BUILD stamp, cut, plot).

## 1. The stage machines agree

The Rust board in HITL vs plan B's own control stack (`tools/virtual_flight.py`, CPython) on the same
`sim_model`, both aligned on their BOOSTING entry:

| flight | GLIDING | LANDING | DONE | apogee | peak \|a\| |
|---|---|---|---|---|---|
| Rust taster, HITL, passive — session 000547 | 10.1 s | 116.7 s | 119.8 s | 273.3 m | 3.0 g |
| Rust taster, HITL, passive — session 000554 | 10.1 s | 116.9 s | 119.9 s | 273.3 m | 3.0 g |
| plan B `virtual_flight`, active | 10.0 s | 110.1 s | — | 273.3 m | 3.0 g |

- **Launch and apogee are the same flight**: the same 3.0 g peak, the same 273.3 m apogee, the apogee
  detected within 0.1 s of plan B's. The two Rust runs agree with each other to 0.2 s.
- **The landing differs by design**: plan B's guidance banks into the zone (the loiter on the track
  panel); the passive Rust glide flies straight out at the 30° launch heading for ~1.5 km. Plan B's
  virtual flight stops at touchdown, so it has no DONE.
- **A trap caught on the way**: `virtual_flight` defaults to plan B's 270 g glider and the host to the
  collectors' 285 g. Flown at different masses, the apogee looked 15 m apart and the peak 0.1 g, which
  read as a control difference; at equal mass both are identical.
- The Rust board landed on the **barometric height** (`agl 4.2m`): it crosses `land_agl_m` 5 m while
  the simulated laser, which only reads within 4 m, is still out of range — the same fallback plan B has.

Charts: [`f15_clean_vs_planb.html`](f15_clean_vs_planb.html) (interactive; [SVG](f15_clean_vs_planb.svg)) —
height, the boost's |a|, airspeed, the ground track. The Rust flight on its own, as
`tools/recorder_flight_plots.py` draws any Rust session: [`f15_clean_rust.html`](f15_clean_rust.html)
([launch SVG](f15_clean_rust_launch.svg)).

## 2. A real reboot mid-glide

Plan B simulated its mid-glide reboot inside the board's VM. The Rust board **really reboots**: 1–6 s into
the glide the host panics it through the console (`panic kind=rust confirm`) while the sim flies on.

| | session 000556 (before) | session 000557 (after) |
|---|---|---|
| stages | SETTING → BOOSTING (\|a\| 3.0 g) → GLIDING (apogee 273 m) | GLIDING (restored at 0.87 s uptime) → LANDING (4.2 m) → DONE |
| warm gate | — | `recover gliding, 3s after checkpoint (reset_cause panic)` |

- The crumb restored the stage and the arming; the flight landed on time, as the clean ones did.
- The outage cost **~26 s of frames** (4428 of 5755 answered): the boot, then the Wi-Fi join. A HITL board
  may join again after the launch, since the sim rides the Wi-Fi; a flying board never does (M7).
- HITL simulates no separation switch, so its glide counts as separated for the gate, as plan B's
  harness passed `separated=True` by construction.

## 3. What it took

- The sim-link: 104-byte SENSORS frames at 50 Hz, a 32-byte FINS answer each; **99–100 % answered**
  (5755 of 5755 on 000554). The first bench test lost most frames — sent before the board had joined
  the Wi-Fi; the host now waits for the first answer before it flies.
- The first board build opened its UDP socket before the TCP/IP stack existed: lwIP asserts there, and the
  taster boot-looped (~40 empty sessions) until the socket waited for the stack.
- CC goes through the hub: on the USB console the board's log lines split long replies (`get-config`'s
  9.5 KB), so the first runs could not even read the config. The console only carries the panic.

Not here yet: the active glide (M13), the 12-scenario matrix against plan B's spread (M14).
