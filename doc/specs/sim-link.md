# Sim-link: HITL with the physics on the host

The Rust board flies hardware-in-the-loop against `src/glider/sim_model.py` running on the host, not on the board
as plan B's `tasks/hitl.py` did. The host steps the physics and sends the sensors; the board publishes them on its
databoard as if its parts had read them, flies its real stage machine and control law, and sends its fins and its
stage back. `sim_model.py` stays the one physics implementation (doc/specs/avionics-port.md M12).

Parts:

| part | where | does |
|---|---|---|
| `crates/sim_link` | Rust | the two frames' codec: allocation-free, versioned, CRC-checked |
| `firmware/src/sim_link.rs` | board | the UDP endpoint, only with an enabled `hitl` entry: SENSORS in, FINS out |
| `tools/hitl_host.py` | host | `sim_model.Body` in real time, the scenario, the faults; drives the board over CC |

## Transport

- UDP over the board's Wi-Fi, beside the CC link. The board binds `sim_port` from its `hitl` entry (default
  **1236**; the hub's TCP ports are 1234 and 1235).
- The host sends a **SENSORS** frame per physics publish (plan B's `inject_hz`, 50 Hz by default). The board
  answers every frame it applies with a **FINS** frame to the frame's sender, so the host learns the board's stage,
  arming and fins at the same rate, and the board needs no host address in its config.
- A lost frame is not resent: the next one carries the newer state. The host keeps flying on the last FINS it
  received; the board's databoard windows (each quantity's `timeout_ms`) decide when a quantity the host stopped
  sending goes stale.

## Frames

Little-endian throughout. Every frame starts with the same 8-byte header and ends with a CRC-32 (IEEE 802.3, as
zlib and `binascii.crc32` compute it) over every byte before it.

| offset | size | field |
|---|---|---|
| 0 | 4 | magic `CSL1` (0x43 0x53 0x4C 0x31) |
| 4 | 1 | version: 1 |
| 5 | 1 | kind: 1 SENSORS, 2 FINS |
| 6 | 2 | presence flags (per kind, below) |

A receiver drops a frame of the wrong length, magic, version or kind, a bad CRC, a non-finite float in a present
field, or a sequence number not newer than the last one it applied (UDP may reorder), and counts each reason.
Sequence numbers wrap: `b` is newer than `a` when `(b - a) mod 2^32` is in 1..2^31.

### SENSORS (host -> board), 104 bytes

| offset | size | field | unit | flag bit |
|---|---|---|---|---|
| 8 | 4 | seq | u32, +1 per frame | — |
| 12 | 4 | sim_time_ms | u32, the physics clock | — |
| 16 | 12 | accel | 3 x f32, g, body axes (x forward, y left, z up) | 0 |
| 28 | 12 | rate | 3 x f32, deg/s: roll, pitch, yaw | 1 |
| 40 | 12 | attitude | 3 x f32, degrees: heading [0, 360), roll, pitch | 2 |
| 52 | 12 | mag | 3 x f32, calibrated counts (heading = atan2(-my, mx)) | 3 |
| 64 | 4 | altitude | f32, m MSL | 4 |
| 68 | 4 | elevation | f32, m above the pad | 5 |
| 72 | 4 | agl | f32, m (laser; absent beyond its range) | 6 |
| 76 | 4 | airspeed | f32, m/s (pitot, railed) | 7 |
| 80 | 4 | dynamic_pressure | f32, Pa | 8 |
| 84 | 8 | position | 2 x i32, 1e-7 degree: latitude, longitude | 9 |
| 92 | 4 | speed | f32, m/s over ground | 10 |
| 96 | 4 | course | f32, degrees | 11 |
| 100 | 4 | CRC-32 | over bytes 0..100 | — |

A field whose flag is clear is absent: its bytes are zero and ignored. That is how the host's faults drop a sensor
(plan B's `drop_attitude`, `drop_mag`, `drop_gnss`) and how the laser leaves its range. Every slot has a fixed
place whatever the flags, so a frame is one fixed size.

The units are the Rust databoard's, which differ from plan B's channels: the attitude and the rates are float
degrees where plan B pushed centidegree fixnums, and the dynamic pressure is float Pa where plan B pushed Pa x 100.
The board derives `change` from `elevation` (crates/control `change`, as a barometer does); the separation switch is
not simulated (M12).

### FINS (board -> host), 32 bytes

| offset | size | field | unit | flag bit |
|---|---|---|---|---|
| 8 | 4 | seq | u32, the SENSORS seq this answers | — |
| 12 | 4 | uptime_ms | u32, the board's clock at the answer | — |
| 16 | 1 | stage | u8, 1 SETTING .. 5 DONE (crates/sequencer `Stage`) | — |
| 17 | 1 | armed | 0 or 1 | — |
| 18 | 1 | launched | 0 or 1 | — |
| 19 | 1 | reserved | 0 | — |
| 20 | 6 | fins | 3 x i16, whole degrees, 90 neutral: left elevon, right elevon, yaw | 0 |
| 26 | 2 | reserved | 0 | — |
| 28 | 4 | CRC-32 | over bytes 0..28 | — |

The fins flag is clear while the board drives no fins (M12 flies passive: no servo is ported until M13); the host
then flies the fins at neutral, as plan B's HITL read a missing servo task.

## Who decides what

- **The host** owns the physics and the clock: the realtime accumulator (plan B `hitl.py:323-391`: fixed physics
  sub-steps covering the measured wall time, a stall's catch-up capped at 0.5 s), the pad dwell, the noise and the
  faults, and the boost-or-glide mode, which follows the board's stage from the latest FINS (SETTING and BOOSTING
  fly the 1-DoF boost, GLIDING and LANDING the glide; from DONE the body stops).
- **The board** owns the flight: it publishes each applied frame's quantities at priority 0, stamped with the frame's
  arrival time (the board's clock, never the host's), records them under the real streams' names, and runs its own
  stage machine and control law on them.
- **CC** sets up each scenario over the hub (`set-config`, `reboot`, `arm`), outside the sim-link.

## The mask (avionics-port.md R10)

The board's `hitl` entry masks every sensor whose config name is in `config_hitl._SIM_SENSORS` (`enabled = False`,
as plan B's `config_hitl.mask`, again after the layout), then refuses to start the sim-link if one of them is still
enabled: a real part left publishing beside the sim becomes a silent second source.
