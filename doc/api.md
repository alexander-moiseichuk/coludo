# Coludo API reference

_Generated from module docstrings by `tools/gen_docs.py` — do not edit by hand; run `python3 tools/gen_docs.py` to regenerate._

See [`architecture.md`](architecture.md) for the module dependency graph, class hierarchy, and the annotated `Flight._step()` hot-path call tree (`tools/gen_graph.py`).

# glider firmware (MicroPython) — `src/glider`

## `airspeed.py`

_Tested by `test/test_airspeed.py`._

airspeed.py — hybrid airspeed estimate for the dynamic-pressure fin governor (coludo.md "Fin
authority"). There is NO pitot tube, so:
* accelerometer integration is the BACKBONE (predict) — primary, and the only usable source during
boost and right after separation, when GNSS is jittery under high dynamics;
* a valid, sane GNSS ground speed nudges out the integrator's drift (correct) — a complementary
filter, GNSS as the slow truth, accel as the fast signal.
GNSS is DISTRUSTED by default: rejected without a fix and above a physical ceiling (a 100+ m/s reading
under separation is a glitch), and only ever BLENDED (never a hard replace) so one bad-but-in-range
sample cannot jump the estimate; repeated good fixes pull the drift out. The estimate is biased to
over-read when uncertain — a high airspeed tightens the governor cap, which is the safe direction.

### `class AirspeedEstimator`

Fuse integrated body acceleration (predict) with sanity-gated GNSS ground speed (correct) into one
airspeed estimate (m/s) for the fin governor. Stateless of HOW accel-along-path is derived — the
caller passes it (e.g. |accel| - g during boost), so this stays unit-testable on the host.

- `__init__(ceiling_ms: float=60.0, gnss_gain: float=0.2)` — constructor
- `value() -> float` — The current airspeed estimate (m/s).
- `predict(accel_along: float, dt: float) -> float` — Integrate net acceleration ALONG the flight path (m/s^2; pass it >= 0 / over-read to stay
- `correct(gnss_speed: float, has_fix: bool) -> float` — Blend toward a GNSS ground speed ONLY if trustworthy: a live fix and within the physical

## `cc_client.py`

_Tested by `test/test_cc_client.py`._

cc_client — board side of the Control protocol (specs/cc-protocol.md). Board-first routing:
Control strips the routing board id, so the board receives `command params` and replies
`status params` (no id; only `iam` carries the board id, so Control can learn it on a new
socket). Dispatcher turns a parsed line into a response (pure logic, unit-testable); Client is
the thin networking that reads lines and writes responses.

### `class Dispatcher`

Maps a command to an async handler(msg) -> response line.

- `__init__()` — constructor
- `on(command: str, fn) -> None`
- `handle(line: str) -> str`

### `class Client`

- `__init__(config: dict, dispatcher, log=None, backoff_ms: int=1000)` — constructor
- `run() -> None` — Connect to Control and serve forever, reconnecting with backoff on drop.
- `serve(reader, writer) -> None` — Read commands from Control, dispatch, write responses. Returns on disconnect.

### `create_dispatcher(cfg: dict, controller=None, on_reboot=None, config_path: str='board.config') -> Dispatcher`

Build a Dispatcher with the standard command handlers, wired to the running config, the
Inspector, and (optionally) the Controller. `on_reboot` lets tests intercept the reset. The
handlers are grouped by concern into the _register_* helpers; each closes over one shared
_Context, so this stays a short orchestrator.

## `cc_protocol.py`

_Tested by `test/test_cc_protocol.py`._

CC <-> board line protocol (specs/cc-protocol.md).

One newline-delimited message per line:  <command> <board-id> [params...]
Tokens are whitespace-separated, so there is NO quoting or escaping. A param value is one of:
* bare token    -> a simple value with no spaces (e.g. 3000, taster, 192.168.10.1)
* base64:<data> -> anything else: spaces, quotes, JSON, binary
Both sides know each command's schema, so the parser does not guess types: a bare token is
returned as a str and the receiver converts numerics itself (it knows `ms` is an int). Named
params are key=value; everything else is positional. The command is lowercased; values keep
their case. parse() handles requests and responses (ok/err/pong/iam) alike.

### `encode(v) -> str`

Encode a value into one whitespace-free wire token.

### `decode(tok: str) -> str`

Decode a wire token back to a str (base64-decoded if prefixed, else as-is).

### `parse(line: str) -> _Msg`

Parse a protocol line into a _Msg (works for requests and responses).

### `build(command: str, args=(), named=None) -> str`

Build a protocol line; values are encoded as needed.

## `commons.py`

_Tested by `test/test_commons.py`._

commons.py — small, dependency-free primitives shared across the control-math modules (mixer / pid /
navigation / guidance / governor / sequencer / flight / sg90). The bundle module for the plan.

Layout, one banner per concern: COMPATIBILITY (every MicroPython/CPython shim, in one place) ->
CONSTANTS -> INTEGER MATH (viper) -> FLOAT MATH (native) -> FIN GOVERNOR -> PERSISTENCE ->
WIRE DIAGNOSTICS.

Naming convention:
plain name -- a leaf with no _opt variant at all.
NAME_upy / NAME_opt + `NAME = <winner>`
-- a function with an optimised variant. NAME_upy is the
portable bytecode reference; NAME_opt is the optimised build (viper for ints, native for floats,
future asm). The module binds NAME to whichever the on-board bench FAVOURS -- usually _opt; switch
the one alias line if a measurement changes. Both forms stay public so benchmarks/tests call them
DIRECTLY (no runtime selector). Bound here: clamp_int, wrap180 (@viper, ~2.1-2.8x); between,
magnitude_sq (@native, ~1.2-1.6x); bank_demand -> _upy for now (its @native measured 1.03x -- a
thin wrapper over native between; switch to _opt when a bench shows a gain).

### `clamp_int_upy(low: int, value: int, high: int) -> int`

### `clamp_int_opt(low: int, value: int, high: int) -> int`

### `wrap180_upy(degrees: int) -> int`

### `wrap180_opt(degrees: int) -> int`

### `between_upy(low: float, value: float, high: float) -> float`

Clamp `value` to the inclusive range [low, high]: `low` if below, `high` if above, else `value`.
With low=-x, high=+x it is a symmetric +/-x clamp; either bound may be math.inf for an open side
(between(-inf, v, inf) == v). Float-/inf-valued (so @native, not viper); plain ints pass through
unconverted. Assumes low <= high.

### `between_opt(low: float, value: float, high: float) -> float`

### `magnitude_sq_upy(x: float, y: float, z: float) -> float`

|(x, y, z)|^2 (no sqrt — callers compare against squared thresholds). Pure float -> @native.

### `magnitude_sq_opt(x: float, y: float, z: float) -> float`

### `bank_demand_upy(heading_error: int, gain: float, limit: float) -> float`

Bank-to-turn: the roll angle (deg, right +) to hold for a heading error (deg) -- proportional with
a symmetric hard clamp (gain 0 -> no bank, rudder-only). A banked turn is tight (~v^2/(g*tan(bank)))
where a flat rudder skid is wide and weak, so the glider does not over-RANGE a small zone and the
overshoot loop becomes an altitude-bleeding orbit.

### `bank_demand_opt(heading_error: int, gain: float, limit: float) -> float`

### `fin_deflection_limit(speed_ms: float) -> int`

Max fin deflection in degrees for airspeed `speed_ms` (m/s) -- the dynamic-pressure governor table
lookup (saturates at _FIN_VMAX). Multiply by the config fin_limit_multiplier at the caller.

### `atomic_write_json(path: str, data) -> None`

Persist `data` as JSON to `path` atomically (shared by config.save + mission.save): write a
temp file then rename it over the target, with a remove-then-rename fallback for a VFS (FAT) that
won't rename onto an existing file. os/json are imported lazily so the hot-path importers of commons
do not pull them in.

### `id_classify(read, expected: int) -> str`

Classify a chip WHO_AM_I / device-id byte against the expected value into an operator-readable
wire-level diagnosis. The deeper 'why' a bus driver's diagnose() returns when setup() failed, so
`verify`/`probe` report e.g. 'chip-select not asserting' instead of just 'absent / miswired?'.
`read` is None when the bus read itself failed (no I2C ack / SPI error). Shared by every ID-based
driver (adxl375 / lsm6dso32 / bno055 / bmp280), so it lives here, not in one driver.

## `config.py`

_Tested by `test/test_config.py`._

Board configuration loader / validator — the Phase 0 foundation.

Implements the three-layer model from specs/board-config.md:
config_default.py (firmware default / fallback)
board.config (saved active config, a full snapshot)
in-memory dict (validated, what the Controller builds tasks from)

Runs on MicroPython on the board. Validation here is config-file *integrity* (structure,
types, pin uniqueness, bus refs, reserved pins) — NOT hardware health, which is checked at
runtime and surfaced to the operator (the strict model).

### `validate(cfg) -> list`

Return a list of human-readable error strings (empty list == valid). Config-file *integrity*
only -- structure, types, pin uniqueness, bus refs, reserved pins -- NOT hardware health, which
is checked at runtime and surfaced to the operator (the strict model).

### `config_id(cfg) -> str`

A stable short hash identifying a config snapshot (for the CC iam/config_id).

### `load(path: str='board.config', defaults=None) -> tuple`

Layered load: active board.config if present and valid, else defaults.

Returns (cfg, source, errors). `source` is 'active', 'default', or a fallback reason.
Never raises — a missing/corrupt/invalid active file degrades to defaults so the board is
always reachable.

### `save(cfg, path: str='board.config') -> str`

Validate then atomically persist a full config snapshot. Returns its config_id.

Raises ValueError if invalid (an invalid config is never written).

### `reset(path: str='board.config') -> bool`

Delete the active config so the next load uses defaults. Returns True if removed.

### `bus(cfg, kind, ident) -> dict`

Resolve a bus addressed by `kind` ('uart'/'i2c'/'spi') + `ident` (its id) to its spec dict,
or None. Ids are JSON object keys (always strings), so the int id from a component is normalized
here -- callers pass `device['bus'], device['id']` and never parse a 'type:id' string.

### `device(cfg, name=None, driver=None) -> dict`

Find a sensor/component by `name` and/or implementation. `driver` matches the resolved
implementation -- a component's `driver` (drivers/) or `activity` (tasks/) field. Returns the
dict or None.

## `config_default.py`

Baked-in default board configuration for the WaveShare ESP32-P4-WIFI6 controller.

Human-edited firmware default and the safe fallback when no valid board.config exists (see
specs/board-config.md). Pins come from doc/waveshare_esp32p4_pins.md (validated on hardware by
test/test_pins.py). `default()` returns a FRESH dict each call so callers may mutate it freely.

Topology: buses are grouped by type then id; a sensor/component addresses one by `bus` (the kind,
e.g. 'i2c') + `id` (its int id), so nothing parses a 'type:id' string. `sensors` are data
providers fused by quantity + priority (several may provide the same quantity with different
drivers/priorities); `components` are the consumers/actuators (recorder, ...).

### `default() -> dict`

## `config_hitl.py`

config_hitl.py — a HITL board config derived from config_default (). The real sensor drivers
are turned OFF and the `hitl` task supplies accel/attitude/agl/altitude/elevation/position at priority
0, so the control code reads the simulation. flight is enabled with test gains, the watchdog and the
radios are off (self-contained sim), and separation is off (the boost-timeout drives BOOSTING ->
GLIDING). Servos stay on so the sim can read the commanded fin angles. `default()` returns a fresh
dict -- mutate freely. Run it instead of config_default for a simulation; the flight config is untouched.

### `default(motor: str='F15', noise: float=0.0, spike: bool=False, wind: float=0.0, wind_dir: float=0.0, boost_axis: str='z', glider_g: int=_GLIDER_G, inject_hz: int=0) -> dict`

Build a HITL config. Separation is off here, so boost->glide deploy rides the sequencer's baro
APOGEE detect (mass/motor-independent -- the top of the arc), with config_default's long boost_timeout
as the last-resort fallback; the sim's reduced baro noise keeps the peak-detect clean. `wind`/`wind_dir`
set a steady cross-wind (m/s, toward deg) the glide must crab against. `boost_axis` picks which accel
axis carries the boost |a|. `glider_g` is the glider (glide) mass in grams (default 300, the full build;
150 = the half-weight optimisation target) -- the booster adds to it for the boost phase, then ejects
at separation so the glide runs on `glider_g` alone (a lighter glider -> a longer glide, the worst case
for the GC-off leak). `inject_hz` > 0 sets the sensor publish rate (default 0 -> the sim's sim_hz);
lower it (e.g. 10) to slim the sim's own heap churn so an on-board HITL leak reflects real flight.

## `controller.py`

_Tested by `test/test_controller.py`._

Flight Controller — creates and supervises the tasks described by a validated config, and
tracks the flight stage machine. See specs/coludo.md ('Flight Controller', 'Tasks').

The Controller is the one task created explicitly; it creates the rest from config in a
deterministic order. Task failures are reported, not fatal (the strict/operator-authority
model): a component that fails setup is logged and skipped, and go/no-go stays with the
operator via stats()/validate().

### `class Stage`

The flight stages, self-contained: int ids (cheap to compare/store on MicroPython) and the
`STAGES` id->name mapping (operator-facing names; `in Stage.STAGES` is an O(1) key check). `NAMES`
is the reverse (name->id) so config that names stages by string resolves to an id once.

kept here, in the stage machine's own module. flight/sequencer/hitl/led import it from
controller -- a LIGHT coupling (the module loads fast, no heavy deps pulled just for the enum). It
could move to commons.py as the shared domain enum to drop even that import, but the gain is marginal
versus the cross-file churn; revisit only if importing controller solely for Stage ever bites.


### `class Controller(inspector.Inspectable)`

- `__init__(config: dict, registry: dict=None, log=None)` — constructor
- `directory() -> list` — Names of enabled devices, in creation order (config order).
- `create(name: str) -> task.Task` — Create a task by component name via the registry. A component names its implementation
- `active(name: str=None)` — Return the active task by name (None if absent), or a list of all active tasks if
- `find(names: list[str]) -> list` — Non-blocking: the active tasks for `names`, None for any not up. The fast lookup for
- `query(names: list[str], waiting: bool=True) -> list` — Look up sibling tasks by name from the registry: `gnss, baro = await self.query(['gnss',
- `setup() -> bool` — Create + set up every enabled task in order. Skip (and report) failures. setup() brings a
- `bustune(kind: str, ident, freq: int) -> dict` — Bench frequency-calibration primitive the CC-side sweep drives: retune sensor bus
- `start() -> None` — Launch each task's run() loop as a supervised asyncio task.
- `close(name: str) -> None` — Deactivate a task and clean up its resources.
- `finish() -> None` — Shut down all tasks, in REVERSE bring-up order so a command PRODUCER (e.g. the flight loop,
- `set_stage(stage: int) -> None`
- `stage_name() -> str` — The current flight stage as its operator-facing name.
- `arm() -> None` — Enable actuation. The pre-flight precondition (probe all clean, mission set) is enforced by
- `disarm() -> None`
- `hold(stage_name: str) -> bool` — Operator stage override (ground test): force a stage and pause auto-sequencing. Returns
- `resume() -> None` — Clear the operator hold -> the sequencer drives the stages again.
- `validate() -> bool` — True if every active task is healthy.
- `inspect() -> dict`
- `stats() -> dict`

## `databoard.py`

_Tested by `test/test_databoard.py`._

databoard.py — the shared latest-value store + sensor fusion for hot data (specs/coludo.md "Task
Data-Flow and Message Propagation"). Replaces a two-layer raw/fused store + a polling fusion task
with a registry of Parameter objects whose fused value is computed on read.

Structure.
Databoard   — a registry of Parameter objects. Databoard.parameter(name) gets-or-creates one;
a sensor registers itself as a source via provide() (which returns its channel
handles) and then reports by pushing each channel directly -- the hot write path
is one step, no lookup. value()/read() resolve the winner + primary in one pass.
Parameter    — one fused quantity (e.g. 'altitude') for the consumer. Holds a short LIST of
channels KEPT IN RANK ORDER (lowest = primary first; a list, not a dict, is faster
at this size), plus the shared freshness window derived from its primary tier.
_Channel     — one source's stream: a static rank (priority; lower = preferred) and TWO slots
(the last two readings) -- two slots because the extrapolation here is LINEAR
(needs 2 points); a degree-N model would keep N+1.

Fusion is a pure read-time function, Parameter.value():
1. winner = the lowest-rank channel still fresh. Channels are rank-ordered, so it is just the
FIRST fresh one in the list (same-rank channels are equivalent). Freshness uses ONE shared
window per parameter: the tightest expiry among the rank-0 tier (min() if two share rank 0),
applied to EVERY channel. Return its value.
2. if NO channel is fresh, linearly extrapolate the PRIMARY (channels[0]) two slots to now --
project the trusted source forward rather than hand out a backup that is itself stale.
3. if the primary never pushed (startup), None.
So "rank 0 answers while fresh; a backup takes over only while itself THIS fresh, else rank 0 is
extrapolated" is EMERGENT -- every read re-evaluates freshness against the shared window. A channel
is BORN STALE (t1 a full _DEFAULT_EXPIRE in the past), so an un-pushed channel is never fresh; and
since every window is <= _DEFAULT_EXPIRE, a FRESH channel always has data -- which is why nothing
downstream needs a v1-None check (a source that never produces is simply never fresh, and surfaces
as a missing reading rather than a hidden guard).

The shared window decides WHEN to fall back; offset reconciliation (opt-in, 'reconcile': true on a
provider) decides WHAT the fallback reports. While the primary is fresh, each backup's BIAS against
it is learned (EMA, once per new primary reading -- the rate is set by data, not by reads); on
handover the backup's value is corrected by that offset, so it reads what the primary would --
closing the bias gap between e.g. ICP-10111 and BMP280 rather than jumping across it. Offsets FREEZE
while the primary is stale, and reconciliation is for additive SCALARS only (altitude, pressure) --
never vectors (attitude/accel) or unlike quantities (agl, position). Per-source slots keep
extrapolation within a single source.

Dependencies. A sensor that consumes another's quantity grabs a read handle with parameter(*names)
(get-or-create, so setup order does not matter); a provider gets its write-channels from
provide(source, provides, *want). Both return one handle for one name, a tuple for several.

Telemetry is separate: each sensor writes its own raw SENSOR.csv directly. A global singleton,
Inspectable as `databoard` (fused value/source/age per parameter).

### `class Parameter`

One fused quantity. Holds a rank-ordered channel per source; value() fuses by rank/freshness,
falling back to extrapolation of the primary when none is fresh.

- `__init__(name: str)` — constructor
- `add_source(source: str, rank: int, expire_us: int, reconcile: bool=False) -> _Channel` — Register (or re-register) a source at `rank`; return its channel to push() to directly (no
- `write(value, source: str) -> None` — Report a source's latest reading by name (convenience; sensors push() their channel). The
- `value()` — The fused estimate (offset-reconciled when enabled); None if nothing was ever written.
- `stamp()` — ticks_us of the primary source's latest push, or None if nothing was ever written. For
- `read() -> list` — [value, source, age_ms] of the fused estimate; `source` is None when extrapolated, else the
- `offsets() -> dict` — Learned bias per source (source -> offset) for diagnostics; empty until reconciled.
- `raw(source: str)` — A specific source's latest value (None if absent / unwritten).
- `sources() -> list`

### `class Databoard`

- `parameter(*names)` _(classmethod)_ — Get-or-create read handle(s) for `names` -- the dependency accessor: a consumer grabs
- `provide(source: str, provides: dict, *want)` _(classmethod)_ — Register `source` for the params it provides ({param: {priority, timeout_ms[, reconcile]}})
- `write(name: str, value, source: str) -> None` _(classmethod)_
- `value(name: str)` _(classmethod)_
- `read(name: str) -> tuple` _(classmethod)_
- `raw(name: str, source: str)` _(classmethod)_
- `inspect() -> dict` _(classmethod)_
- `stats() -> dict` _(classmethod)_

## `fixed.py`

_Tested by `test/test_fixed.py`._

fixed.py — fixed-point helpers for the flight hot paths. MicroPython boxes a heap float on EVERY float
operation, and GC is disabled through the airborne phase, so every boxed float leaks toward OOM. The
control path therefore works in scaled integers ("fixnum") and crosses to/from float only at the
isolated sensor boundary.

`fixnum` is `int`, aliased -- a SEMANTIC marker that a value is a scaled fixed-point quantity (×SCALE),
not a plain count and not a float. Annotating with it documents the convention and makes an accidental
float / whole-number mix obvious at the call site; there is no runtime cost.

SCALE is the fractional resolution (100 -> 0.01 unit: centidegrees / centimetres / centi-(m/s)). It is
kept small on purpose: the RV32 small-int ceiling is 2**30, so a product of two scaled quantities is
(val·SCALE)² -- at SCALE 100 a ±180° angle squared is 3.2e8 (safe); at 1000 it is 3.2e10 (a 16-byte
mpz). Start at 100; raise to 1000 only if the accuracy sweep in test_fixed.py shows 0.01 is too coarse.

Convert at the BOUNDARY only -- from_float once on the way in, to_float / to_str once on the way out --
and stay integer in between. There is deliberately NO fixnum mul/div rescale here: a fixed-point rescale
invites float->fixnum->float chains mid-computation, which is exactly what this exists to remove.

### `from_float(value) -> fixnum`

Whole unit (float or int: degrees / metres / m·s⁻¹) -> fixnum. The one boxed-float spot, kept at
the sensor boundary. Truncates toward zero -- the residual is < 1/SCALE (below actuator resolution).

### `to_float(scaled: fixnum) -> float`

fixnum -> whole-unit float. Boxes a float, so use ONLY where a float is genuinely required (trig,
the airspeed integrator) and keep it at the boundary -- never inside a hot loop.

### `to_millis(value: fixnum) -> int`

A fixnum (×SCALE) -> integer MILLI-units (×1000), independent of SCALE -- e.g. at SCALE=100 a
centidegree fixnum becomes millidegrees. For telemetry/logs that fix a milli representation regardless
of the control SCALE. Pure integer rescale (SCALE divides 1000), so no float is boxed.

### `to_str(scaled: fixnum) -> str`

fixnum -> its decimal string ('12.34' at SCALE 100) via INTEGER divmod -- NO float is boxed. For
telemetry / display: a scaled value prints as its true decimal without a float round-trip.

### `clamp(low: fixnum, value: fixnum, high: fixnum) -> fixnum`

Integer clamp to [low, high] (a symmetric ±x clamp with low=-x, high=+x). Routes to commons.
clamp_int -- the `@micropython.viper` integer clamp (~2.1-2.8x the float `between`). Safe here because
fixnum is always a finite int (no math.inf), which is exactly what the fixed-point transition buys:
the whole control-path clamp is now viper-native, not the inf-tolerant @native float path.

### `atan2_cd(y: int, x: int) -> fixnum`

atan2(y, x) as a CENTIDEGREE fixnum, four-quadrant, via integer CORDIC -- NO float boxed. y and x
are a RATIO-FREE integer direction vector: only their ratio sets the angle, and their MAGNITUDE only
trades precision (the CORDIC's right-shifts discard low bits, so bigger inputs keep more). Fed the
control's centi-fixnum scale (accel g via from_float, ~x100) the error is ~0.5 deg typical / 1.8 deg
worst over the glide envelope -- fine for the attitude backup; x1000 would tighten to ~0.16 deg if a
caller ever needs it. Range (-18000, 18000]. CORDIC needs x >= 0, so x < 0 reflects into the right
half-plane and the 180 deg is added back per quadrant.

### `blend_cd(state: fixnum, delta: fixnum, target: fixnum, shift: int, correct: bool) -> fixnum`

One complementary-filter step in centidegrees (viper): `state + delta` (gyro integration), then
optionally a `1/2^shift` pull toward `target` (the accel angle). Pure integer -> zero float boxed;
the attitude backup runs it per axis each control step (tasks/attitude.py).

### `isqrt_upy(n: int) -> int`

Integer floor(sqrt(n)) reference, division-free (bit-by-bit); n < 2**31.

### `isqrt_opt(n: int) -> int`

## `gnss.py`

_Tested by `test/test_gnss.py`._

gnss.py — shared GNSS infrastructure (sibling of i2cbus/spibus/servo). NMEA helpers + a Gnss base
Task: read NMEA over a dedicated UART, parse RMC -> 'position' (lat, lon) and GGA -> 'altitude'
(m MSL) + 'elevation' (m above the GNSS ground zero, a barometer backup). Module-specific sentence
selection + rate is the subclass's _configure(); ATGM336H (CASIC/PCAS) and NEO-6M (u-blox) differ
only there. Talker-agnostic (GP/GN/BD). Best-effort -- lock drops under boost, so the channels go
stale and consumers fall back.

### `checksum_ok(sentence: str) -> bool`

Verify the NMEA `*hh` XOR checksum (over the chars between '$' and '*'); inner loop = _xor_checksum.

### `degrees(value: str, hemisphere: str)`

NMEA ddmm.mmmm + N/S/E/W -> signed decimal degrees (None when the field is empty).

### `nmea(body: str) -> bytes`

Wrap a command body in `$...*hh\r\n` with its XOR checksum (PCAS/PMTK/PUBX config sentences).

### `class Gnss(task.Task)`

Base GNSS driver over a dedicated UART: RMC -> 'position' (lat, lon); GGA -> 'altitude' (m MSL)
+ 'elevation' (m above the GNSS ground zero, a baro backup). Subclasses set the module-specific
sentence selection + rate in _configure().

- `setup() -> bool`
- `run() -> None` — Read NMEA lines forever and parse them; non-ASCII noise and malformed fields are skipped
- `probe() -> str` — On-demand self-test: NMEA is arriving on the UART (the run loop counts lines). A satellite
- `diagnose() -> str` — Deeper analysis when setup() failed: is NMEA arriving on the UART? Open the port and listen
- `inspect() -> dict`

## `governor.py`

_Tested by `test/test_governor.py`._

governor.py — the dynamic-pressure fin governor (specs/coludo.md "Fin authority"), sibling of
pid.py / mixer.py / airspeed.py. Owns the airspeed ESTIMATE (airspeed.AirspeedEstimator: accel
backbone + GNSS corrector), the ADAPTIVE THROTTLE that keeps that float path off the GC-off hot
loop once the glide settles, and the mixer authority cap (commons.fin_deflection_limit ∝ 1/v²,
× the board's fin_limit_multiplier safety dial). Extracted from tasks/flight.py (doc/plan.md
structural roadmap #1) so the throttle policy is unit-testable without a Flight task.

Host-runnable by construction (tools/virtual_flight.py drives the REAL governor): the sensor
dependencies are INJECTED databoard-style handles — `accel.value()` -> (x, y, z) in g or None,
`gnss_speed.read()` -> (m/s, source, age_ms) — never the databoard itself, and nothing here
touches time or the machine.

Why the estimator is throttled at all: the update is a FLOAT path (sqrt magnitude, integrate,
GNSS blend) ~ the biggest GC-off allocator measured (~22 KB/s at 100 Hz). It runs FULL RATE where
the estimate cannot be trusted to pace itself (pre-glide boost/decel, a fresh dive); everywhere
else the DISTANCE-CONSTANT law paces it: update at clamp(speed, floor, ceiling) Hz = one update
per ~1 m of TRAVEL. Probed 1..60 m/s against the previous error-adaptive law (7/04):
* consistency — exactly 1.00 m/check across the whole 5..50 m/s envelope (old: 0.04..1.90 m
with a 9.5x discontinuity at its 20 m/s full-rate trigger);
* safety — the old law's WORST staleness (1.9 m at 19 m/s) sat right below its own trigger;
here staleness self-scales, an overspeed shrinks its own next interval, so the absolute-speed
trigger is gone entirely;
* leak — same class at glide trim (3.1 vs 2.2..5.6 KB/s), 3.3x LESS in a 30 m/s dive (6.7 vs
22.4 KB/s: no more 100 Hz above 20 m/s for granularity nothing needs);
* simplicity — 4 knobs -> 2 (floor/ceiling Hz) and the adaptation state machine becomes the
same precomputed integer-indexed table as commons.fin_deflection_limit.
The estimator integrates the ACCUMULATED dt, so cadence never changes the integral — only how
fresh the fin-authority cap is (the cap persists between updates).

### `class GovernorConfig`

The governor's knobs, resolved from the flight task's config dict ONCE (typed config: one
place for defaults + doc-in-code; the keys keep their board.config names).

- `__init__(config: dict)` — constructor
- `update_interval(speed: float) -> float` — The estimator update interval (s) for `speed` (m/s) — the distance-constant table lookup

### `class Governor`

Cap the mixer's control authority by estimated airspeed (torque ∝ v²): step() each control
slice decides full-rate vs throttled, updates the estimator over the accumulated dt, and writes
the deflection cap into mixer.limit.

- `__init__(config: GovernorConfig, mixer, accel, gnss_speed, fin_limit_multiplier: float=1.0)` — constructor
- `airspeed() -> float` — The current airspeed estimate (m/s) — the boost rod gate and telemetry read it here.
- `cap() -> int` — The dynamic-pressure fin-authority cap (deg) the governor last set on the mixer — the
- `step(dt: float, full_rate_override: bool, pitch: fixnum) -> None` — One control slice: accumulate `dt` (wall seconds since the last step) and update the

## `guidance.py`

_Tested by `test/test_guidance.py`._

guidance.py — the stage-dependent guidance law, sibling of pid.py / mixer.py / navigation.py.
Turns (stage, heading) into the attitude setpoints + heading error the PIDs chase: the boost
rod-vertical hold, bank-to-turn toward the landing zone, the three GPS-degrading heading tiers,
and the low-final-approach centreline tracker. Extracted from tasks/flight.py (doc/plan.md
structural roadmap #1, findings §20 S03) so the control law is unit-testable without a Flight
task; per-stage laws dispatch through a table (S04 — the proven sequencer._detect pattern), so a
new stage is one entry + one method, not a branch in a growing if/elif.

Host-runnable by construction (tools/virtual_flight.py drives the REAL law): dependencies are
INJECTED — the mission (zone/launch_point), the governor (airspeed for the boost rod gate), and
databoard-style handles (`position.read()` -> ((lat, lon), source, age_ms), `agl.value()` -> m or
None). Timing comes in as `now_us` from the caller; only commons.ticks_diff touches ticks.

Results land in the roll_setpoint/pitch_setpoint (centidegree fixnum) + heading_error (int
degrees) INSTANCE SLOTS rather than a returned tuple — decomposed WITHOUT adding a per-step heap
allocation (GC is off in flight).

### `heading_error(target: float, current: float) -> int`

Shortest signed heading error (deg), wrapped to [-180, 180] so 350 -> 10 is +20, not -340.
Integer degrees — sub-degree precision is irrelevant to a servo and lets one modulo replace the
wrap loop (commons.wrap180, viper bundle).

### `class GuidanceConfig`

The guidance knobs, resolved from the flight task's config dict ONCE (typed config: one place
for defaults + doc-in-code; the keys keep their board.config names). `position_window_ms` is the
caller's default tier-1 freshness gate — the GNSS channels' own databoard windows — so it tracks
the GNSS rate instead of a magic number; config sets position_age_max_ms TIGHTER to distrust
GNSS sooner (looser is a no-op: the source is already None past the window).

- `__init__(config: dict, position_window_ms: int)` — constructor

### `class Guidance`

The per-stage control law: setpoint(stage) gates control stages; enter() captures the holds
on entering control; compute() dispatches the stage's law and fills the setpoint slots.

- `__init__(config: GuidanceConfig, mission, governor, position, agl, elevation=None)` — constructor
- `setpoint(stage: int)` — The configured attitude setpoint dict for `stage`, or None when it is not a CONTROL stage
- `reachability(glide_ratio: float, wind_e: float=0.0, wind_n: float=0.0, airspeed: float=0.0)` — Can the glider still glide to the zone from here? The still-air reach = elevation × glide_ratio
- `enter(heading: float, roll: fixnum, pitch: fixnum) -> None` — Entering a control stage (from a non-control one): capture the heading to hold blind and
- `compute(stage: int, setpoint: dict, heading: float, now_us: int) -> bool` — Run `stage`'s law: fill roll_setpoint/pitch_setpoint/heading_error and return True, or

## `i2cbus.py`

_Tested by `test/test_i2cbus.py`._

i2cbus.py — shared, lock-serialized I2C buses. Several sensor drivers sit on one physical bus
(i2c:0 carries the ADXL375, BNO055 and BMP280), so they must not interleave transactions on the
single peripheral: each bus id has ONE machine.I2C plus an asyncio.Lock, and get() hands back the
shared wrapper. The read/write methods are async (they acquire the lock) but the underlying I2C op
is fast and synchronous, so the lock is held only for the transaction. A glider-only module.

### `class Bus`

One physical I2C bus, shared by every device on it; transactions are serialized by a lock.

- `__init__(bus_id: int, spec: dict)` — constructor
- `retune(freq: int) -> None` — Re-init this I2C peripheral at `freq` Hz in place (bench frequency calibration; no reboot).
- `read(addr: int, reg: int, count: int, addrsize: int=8) -> bytes`
- `read_into(addr: int, reg: int, buf, addrsize: int=8) -> None`
- `write(addr: int, reg: int, data: bytes, addrsize: int=8) -> None`
- `writeto(addr: int, data: bytes) -> None` — Raw write (no register) — for command-based devices like the ICP-10111.
- `readfrom(addr: int, count: int) -> bytes` — Raw read (no register) — pairs with writeto() for command-based devices.
- `device(addr: int) -> _Device` — A register window for one address on this bus (matches spibus.Bus.device).
- `scan() -> list`

### `get(bus_id: int, spec: dict) -> Bus`

The shared Bus for `bus_id`, created once from `spec` (scl/sda/freq) and cached thereafter.

## `inspector.py`

_Tested by `test/test_inspector.py`._

Inspector — the registry of Inspectable objects and the operator-facing introspection surface.
Control's inspect/update/stats commands resolve an object by name through the Inspector
(specs/cc-protocol.md). Any object an operator should see or tweak registers itself here.

### `class Inspectable`

Mixin for an operator-inspectable object.

`name` is the registry key; `kind` is a category. inspect() returns a json-able dict of
properties; update(props) applies the supported, changed ones and returns the names actually
changed; stats() returns interesting runtime numbers. The defaults read `_inspect` (readable
property names) and write `_writable` (the subset settable via update()); override any of the
three for computed values.

- `inspect() -> dict`
- `update(props: dict) -> list`
- `stats() -> dict`

### `class Inspector`

- `register(obj) -> None` _(classmethod)_
- `unregister(name: str) -> None` _(classmethod)_
- `names() -> list` _(classmethod)_
- `get(name: str)` _(classmethod)_
- `probe_all() -> dict` _(classmethod)_ — Run probe() on every registered inspectable that implements it; return {name: result} (result
- `inspect(name: str) -> dict` _(classmethod)_
- `update(name: str, props: dict) -> list` _(classmethod)_
- `stats(name: str) -> dict` _(classmethod)_

## `main.py`

_Tested by `test/test_main.py`._

main.py — board bring-up, run on boot. Loads the driver/task packages (so every @task.activity /
@task.driver registers), creates the Mission (launch identity), and hands the config to the Controller,
which builds + supervises the *enabled* tasks. Connectivity (Wi-Fi + the CC link) is just two of
those tasks, so a board with no Wi-Fi (e.g. FireBeetle 2) boots and runs everything else without
CC -- nothing here is hardcoded. Adding a task is dropping a file in drivers/ or tasks/ and
enabling it in the board config.

Telemetry-first: the task loops (recording included) start immediately and keep running; the
Wi-Fi/CC tasks connect in the background when they can. Time sync + live tweaks arrive from Control
over the link (e.g. `update mission {epoch}` sets the RTC); the board itself never asks.

### `bringup(cfg: dict, log=print) -> controller.Controller`

Register every driver/task, create the Mission, and have the Controller build + start the
enabled tasks from the config. Returns the Controller. Network-free itself -- any Wi-Fi/CC work
happens inside the tasks the Controller starts.

### `main() -> None`

## `mission.py`

_Tested by `test/test_mission.py`._

Mission — the per-launch identity the operator sets before a flight: a launch id, the launch
site position (a known origin and a GNSS cold-start seed), and the board clock. Unlike the board
config (hardware; stable across flights, see config.py) the mission changes every launch, so it
lives in its own file, `launch.config`, and is edited live through the Inspector.

Mission is a singleton Inspectable:
inspect mission -> launch id / site / position + the board clock
update mission base64:{"launch_id":"t1"} -> set the launch id for this flight
update mission base64:{"epoch":1750170000} -> set the board RTC (time sync; Unix seconds)
get-config launch / set-config launch -> read / save (merge + persist) launch.config

Position is metres / decimal degrees; it is a known origin now and seeds the GNSS driver later.

### `class Mission(inspector.Inspectable)`

The operator-set launch identity. One per board; registers itself so Control can
`inspect`/`update mission`. Seeded from launch.config at construction.

- `__init__(path: str=LAUNCH_PATH, max_range_m: float=_DEFAULT_MAX_RANGE_M)` — constructor
- `set_time(epoch) -> bool` — Set the board RTC from a Unix epoch (seconds, UTC). Returns True if applied. Rejects a value
- `clock() -> str` — Current board wall-clock as 'YYYY-MM-DDTHH:MM:SS' (from the RTC).
- `epoch() -> int` — Current board clock as a Unix epoch (seconds), for Control to compare against its own.
- `launch_point()` — The launch origin (lat, lon): the operator-set position (CC `update mission` / `assist`) if
- `freeze_launch() -> None` — Pin the live GNSS fix as the persistent launch point (called at arm -- the last moment
- `select_site(fix: tuple)` — CC-less site selection (specs/coludo.md "Field operation without CC"): the nearest known
- `fallback_zone(fix: tuple, bearing_deg: float=0.0, near_m: float=50.0, width_m: float=100.0, depth_m: float=90.0) -> tuple` — The spiral-landing fallback (spec, simplified 7/08): no known site in range after ignition
- `geometry() -> dict` — The landing zone resolved against the launch point: the target (centre) + both gates
- `probe() -> str` — On-demand self-test: a launch position is set (CC or GNSS) and, if a landing zone is set, all
- `inspect() -> dict`
- `update(props: dict) -> list` — Apply launch_id/site/latitude/longitude/altitude (stored, range-checked) and `epoch`
- `persisted() -> dict` — The mission as it is stored in launch.config: the editable launch fields only -- no computed
- `save() -> None` — Persist the stored mission fields to launch.config (atomic temp+rename) so the launch

## `mixer.py`

_Tested by `test/test_mixer.py`._

mixer.py — control-surface mixer (sibling of servo.py / gnss.py). Maps the control axes (roll,
pitch, yaw -- each a deflection command in degrees) to per-fin servo angles for the airframe's
mixing: ELEVONS (the two elerons move together for pitch, differentially for roll) + a RUDDER (the
yaw fin). Per-fin trim (mechanical neutral alignment) and a hard +/- limit on control deflection.
Pure integer math, no hardware -- the flight control task (Phase 3) binds the resolved fin driver
objects once (bind()) and then drives them straight from the mixing loop (actuate()); the
per-driver clamp still guards the physical range. mix() keeps the dict form for tests/host tools.

Signs are config (`surfaces` gains + `trim`), set during bench alignment: if a surface deflects the
wrong way, flip its gain sign; if its neutral is off, set its trim.

### `class Mixer`

Mix (roll, pitch, yaw) deflection commands -> {fin_name: integer angle}:
angle = neutral + trim + clamp(sum(gain * axis), +/- limit).

- `__init__(config: dict=None)` — constructor
- `mix(roll: int=0, pitch: int=0, yaw: int=0) -> dict` — Per-fin integer angle for the given axis deflections (degrees). Returns a SHARED dict REUSED
- `neutralise() -> dict` — The neutral (zero-deflection) angle per fin -- the safe / control-disabled output (shared dict).
- `bind(fins: dict) -> None` — Fuse the resolved fin driver objects ({surface name: object with set_angle(angle)}) into the
- `actuate(roll: int, pitch: int, yaw: int) -> None` — mix() fused with the servo write: clamp each surface's control deflection to +/- limit and

## `navigation.py`

_Tested by `test/test_navigation.py`._

navigation.py — landing-zone navigation geometry (Phase 4 'heading-to-home'), sibling of mixer.py/pid.py.
The mission's landing zone is a lat/lon rectangle, top-left (TL) + bottom-right (BR) corners
(specs/coludo.md). The TARGET is the zone centre; the two GATES are the midpoints of the two SHORTER
sides, so the glider enters along the long axis (the documented "vector to the shortest boundary
entrance"). steer() picks the nearer gate, heads for it until inside the zone, then for the centre.
Equirectangular (flat-earth) math -- "not exact but about", which is plenty at zone scale (<~1 km).

(perf analysis): steer()/bearing()/distance() use float trig and allocate a few small tuples per
call. The concern was GC churn from calling them at the 100 Hz control rate. Resolution: the fix is on
the CALLER side, not here -- guidance._target_heading() caches steer() at GPS cadence (~10 Hz), so
the trig/alloc rate drops ~10x. With the hot-path pressure removed, a zero-allocation rewrite here
would only cost clarity for no measurable gain, so navigation stays simple, pure and correct.

SAFETY: the gates are FIXED to the short sides, and steer() will always vector to one (and turn ~180
back through it on an overshoot) with NO knowledge of what lies beyond any side (trees / launch pad /
people). So the operator must ORIENT the zone -- choose the TL/BR corners in launch.config so the two
short-side entrances point at hazard-free approach corridors and the long sides border the hazards.
Aerodynamics (long run-in, lower crosswind) and safety (clear corridors) only align if it is laid out
that way; the firmware cannot verify it. See specs/coludo.md "Zone orientation -- an operator safety
decision".

### `bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float`

Compass bearing in degrees (0 = north, 90 = east, clockwise) from point 1 to point 2.

### `distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float`

Distance in metres from point 1 to point 2 (equirectangular).

### `zone(corner_tl: tuple, corner_br: tuple) -> tuple`

Resolve the rectangle (top-left, bottom-right corners, each (lat, lon)) -> (target, gate_a,
gate_b): the centre and the midpoints of the two SHORTER sides. A horizontally (longitude)
stretched zone gates on its left/right edges; a vertically (latitude) stretched one on top/bottom.

corner ORDER does not matter. The centre is the average, the spans use abs(), and the gates are
the coordinate EXTREMES (lon_l/lon_r at the centre latitude, or lat_t/lat_b at the centre longitude)
-- so whichever diagonal pair is passed (TL/BR or BL/TR), the two returned gates are the same two
side-midpoints; steer() then picks the nearer. inside() likewise uses min/max. No normalisation needed.

### `inside(position: tuple, corner_tl: tuple, corner_br: tuple) -> bool`

True if position (lat, lon) is within the zone rectangle (corner order-agnostic).

### `steer(position: tuple, corner_tl: tuple, corner_br: tuple) -> tuple`

The heading to fly toward the landing target via the nearer gate: head for the closer short-side
entrance until inside the zone, then for the centre. Returns (bearing_deg, waypoint, leg) with leg
GATE or TARGET. position = (lat, lon).

Stateless + re-evaluated each tick, so the overshoot loop is emergent: if the glider crosses the
zone and exits the far side without landing (still high), the gate it just crossed is now the
nearest one -> it turns back (~180deg) and re-approaches through it. No waypoint memory -- the
spec's 'recalculate to the nearest alternative entry and loop' just happens.

### `cross_track(position: tuple, point: tuple, heading: float) -> float`

Signed perpendicular distance (metres) from `position` to the line through `point` along compass
`heading`; positive = to the RIGHT of the line (looking along the heading).

### `approach(position: tuple, corner_tl: tuple, corner_br: tuple, heading: float, cross_gain: float, intercept_max: float) -> float`

Final-approach guidance: the heading to fly to TRACK the zone's long-axis CENTRELINE (the
strip), used low on final instead of homing to the centre POINT. The glider intercepts the line at up
to `intercept_max` deg (`cross_gain` deg per metre off it), then flies down it -- so a crosswind is
crabbed out and the touchdown holds the narrow strip. Uses the full bank authority (keep it gliding,
not rolling-and-dropping). (This is a banked/crab correction -- a true wing-low SLIP would need a
sideslip-capable airframe model; the residual at strong wind is airframe-bound, not a control gap.)

## `pid.py`

_Tested by `test/test_pid.py`._

pid.py — a minimal fixed-point PID controller for the flight stabilization loop (Phase 3), sibling of
mixer.py. One instance per control axis. Integral anti-windup clamp + output clamp; reset() on
(re)entering a control phase.

INTEGER fixed-point (fixed.fixnum in/out, integer-millisecond dt) so a step allocates NOTHING on the
heap. The flight loop runs with GC DISABLED (sequencer disables it on BOOSTING), so every heap byte
accumulates toward OOM; the old float PID boxed a fresh float on every * + / -- measured 176 B/step,
×3 axes ×100 Hz ≈ 56 KB/s of leak. This version measures 0 B/step (even at a ±180° heading swing, the
worst case for the derivative), leaving only the isolated call-site conversion fixed.from_float(setpoint
- actual) at the sensor boundary. Net saving ≈ 47 KB/s. See findings 17 (memory refactor).

Fixed-point contract (error/output in fixed.fixnum -- degrees × fixed.SCALE; measured alloc-free):
error   fixnum  -- the caller scales at the boundary: fixed.from_float(setpoint - actual)
dt      ms (int)
gains   floats (kp/ki/kd) -- scaled by _KU=100 (0.01 gain resolution) at construction
limits  degrees -- scaled by fixed.SCALE (to the error/output unit) at construction
output  fixnum  -- the caller reduces: output // fixed.SCALE -> integer degrees for the mixer
The two 1000s inside step() are TIME (ms<->s), not the angle scale -- they are independent of SCALE.
Every intermediate product stays < 2**30 (the RV32 small-int ceiling; past it boxes a 16-byte mpz): at
SCALE=100 the worst term kp_k·e = 500·18000 = 9e6 and the derivative swing 36000·1000 = 3.6e7, both far
under it (SCALE=100 keeps ~3x headroom even on a scaled angle², which SCALE=1000 would overflow).

### `class Pid`

error (fixnum, degrees × SCALE) -> control output (fixnum). step(error, dt_ms[, rate]):
kp*e + ki*integral(e) + kd*derivative, each clamped -- all integer, no heap allocation. The
derivative is the measured `rate` (gyro, SCALE-deg/s) when given -- derivative-on-measurement, clean
+ no setpoint kick -- else d(error)/dt (differentiated on the error).

- `__init__(kp: float=0.0, ki: float=0.0, kd: float=0.0, integral_limit: int=_UNBOUNDED_DEG, output_limit: int=_UNBOUNDED_DEG)` — constructor
- `reset() -> None` — Clear the integral + derivative history -- on entering a control phase, so a fresh glide does
- `step(error: fixnum, dt_ms: int, rate: fixnum=None) -> fixnum`

## `recorder.py`

_Tested by `test/test_recorder.py`._

Recorder — the single non-hot data path: telemetry + logs into PSRAM ring buffers, drained to
the Luckfox recorder over UART. See specs/coludo.md ('Task Data-Flow', 'Logging', 'Telemetry',
'Storage Write Constraints').

Recorder is a singleton: any module calls Recorder.log() / Recorder.tlm() globally. Producers
enqueue synchronously (struct.pack_into into a ring -- never slice-assignment, which is
O(buffer length) on this port); the async run() loop drains the rings to the UART via an
asyncio.StreamWriter, telemetry (1st priority) before logs (2nd). Logs are best-effort (dropped
when full); telemetry is important (raises if a record will not fit).

### `class Ring`

Lock-free single-producer / single-consumer byte ring. The writer owns `head`, the reader
owns `tail`; they never touch the same field, so it is safe between an ISR producer and a task
consumer with no locks. Each cell holds <uint16 length><payload>. write() uses pack_into
(cost O(record)) and returns False if there is no room (the record is skipped, never
overwriting unread data). read() returns a bytes copy (stable across an await). Holds
`capacity - 1` records (one cell separates full from empty).

- `__init__(capacity: int=_DEFAULT_CAPACITY, cell_size: int=_DEFAULT_CELL_SIZE)` — constructor
- `write(data: bytes) -> bool`
- `read() -> bytes` — Return the oldest record as bytes (a copy) and advance, or None if empty.
- `count() -> int` — Records currently queued (a stats snapshot).

### `class Recorder`

- `setup(config: dict, uart=None) -> None` _(classmethod)_
- `timestamp() -> int` _(classmethod)_ — Monotonic-ish record timestamp. Currently raw microseconds; the unit may change.
- `session() -> str` _(classmethod)_ — The per-boot file prefix, produced from the RTC the first time it is needed and then
- `log(descriptor: str, message: str) -> bool` _(classmethod)_ — Best-effort log line "<ts> <descriptor> :: <message>" (-> recorder.log). Truncated to
- `cc_logs(duration_ms: int) -> dict` _(classmethod)_ — Poll-model CC log streaming (the `log <ms>` command): the lines buffered since the last call,
- `cc_telemetry(duration_ms: int) -> dict` _(classmethod)_ — Poll-model CC telemetry streaming (the `tlm <ms>` command): the telemetry rows buffered since
- `tlm(filename: str, content: str) -> None` _(classmethod)_ — Important telemetry line "@<session>_<filename>@<content>". Raises if the record will
- `drain() -> int` _(classmethod)_ — Drain queued records to the UART, telemetry first then logs. Returns records drained.
- `run() -> None` _(classmethod)_ — Event-driven drain loop: wait for a producer signal, then drain everything queued, so
- `inspect() -> dict` _(classmethod)_
- `update(props: dict) -> list` _(classmethod)_
- `stats() -> dict` _(classmethod)_
- `report() -> dict` _(classmethod)_

### `class Telemetry`

A typed telemetry stream. Created with a destination file and its data field names; the
first push emits the CSV header (uptime + fields), then each push emits a timestamped row.
All streams in one boot share the Recorder session prefix, so file names are stable.

`decimate_us` rate-limits the stream: push() emits only when at least `decimate_us` microseconds
have passed since the last emitted row (a fast sensor can push every sample and have its telemetry
decimated to a sane rate). `decimate_us=0` (the default) inherits the Recorder GLOBAL rate
(`Recorder.telemetry_decimate_us`, 50 Hz) -- so a stream opts into an individual rate by passing a
non-zero value, else the board-wide `recorder.telemetry_us` prorates it.

- `__init__(filename: str, fields: tuple, decimate_us: int=0)` — constructor
- `push(values) -> None`

## `servo.py`

_Tested by `test/test_servo.py`._

servo.py — shared servo infrastructure, sibling of the bus helpers (i2cbus/spibus). The slew gate
bounds how many fins slew at once (the boost-rail current transient): a process-wide counting
semaphore so `servo_concurrency` (board config) caps total simultaneous slews across every servo
driver. Servo-type-agnostic -- each driver (sg90, future mg90s/mg996r) imports the gate and adds its
own pulse range + slew timing.

### `class Gate`

A tiny FIFO counting semaphore (MicroPython asyncio has no Semaphore, only Lock/Event): at most
`permits` holders at once, the rest queue and are handed a permit in order on release. The
process-wide shared instance lives on the class itself (Gate.slew()/Gate.reset()) -- no module
global.

- `__init__(permits: int)` — constructor
- `acquire() -> None`
- `release() -> None`
- `slew(permits: int) -> 'Gate'` _(classmethod)_ — The process-wide slew gate, created once (the first servo's `permits` wins) and shared by
- `reset() -> None` _(classmethod)_ — Drop the shared gate so the next Gate.slew() rebuilds it -- for tests (clean permit count)

## `sim_model.py`

_Tested by `test/test_sim_model.py`._

sim_model.py — pure flight-dynamics model shared by the on-board HITL task (tasks/hitl.py) and the
host-side virtual-flight tool (tools/virtual_flight.py). PURE: math + random only, no hardware, so it
runs identically on the board (MicroPython) and on the host (CPython) -- the virtual flight and the
HITL sim are then the SAME physics, only the harness around them differs. World frame is ENU metres
from the launch pad; attitude is Euler degrees (roll, pitch, yaw=heading).

### `class Body`

Flight-dynamics state + integrator (PURE -- host-testable). `boost_step()` climbs vertically; at
apogee `begin_glide()` hands over to `glide_step()` (fin-controlled); `sensors()` returns what the
on-board sensors would read.

- `__init__(mass: float, launch: tuple, elevation_m: float, glide_heading: float, glide_mass: float=None)` — constructor
- `boost_step(dt: float, thrust: float, pitch_cmd: float=0.0, roll_cmd: float=0.0) -> None` — Vertical climb (1-DoF: thrust + gravity + drag) PLUS attitude under thrust: a crosswind
- `begin_glide() -> None` — Apogee hand-over: the booster ejects (mass drops to the glider-only glide_mass), then nose down
- `glide_step(dt: float, roll_cmd: float, pitch_cmd: float, yaw_cmd: float) -> None` — Rigid-body glide. Fin deflections (deg from neutral) command roll/pitch; bank turns the
- `position() -> tuple`
- `track() -> float` — Ground-track bearing (deg) -- the direction the glider MOVES over the ground: air velocity
- `ground_speed() -> float` — Horizontal GNSS GROUND speed (m/s) -- the magnitude of the ground velocity, WITH the wind
- `sensors() -> dict` — Clean (pre-noise) sensor readings from the current state.

### `noisy(value, frac: float, lo: float, hi: float)`

Perturb a scalar by +/- frac of its magnitude (uniform), clamped to [lo, hi]. frac 0 -> clean.

## `spibus.py`

spibus.py — shared, lock-serialized SPI buses, mirroring i2cbus. A sensor may move off the shared
I2C bus onto SPI (e.g. the ADXL375, for clean high-rate reads): each bus id gets ONE machine.SPI
plus an asyncio.Lock, and get() hands back the shared wrapper. device(cs) returns a register window
with the SAME read/read_into/write(reg, ...) interface as i2cbus, so a driver is bus-agnostic. The
chip-select is a plain GPIO held low only around each locked transaction (the SPI peripheral does
not own it, so several devices can share one bus). A glider-only module (MicroPython).

### `class Bus`

One physical SPI bus, shared by every device on it; transactions are serialized by a lock.

- `__init__(bus_id: int, spec: dict)` — constructor
- `device(cs: int, mb_bit: int=6) -> _Device` — A register window for one chip-select on this bus (matches i2cbus.Bus.device).
- `retune(freq: int) -> None` — Re-init this SPI peripheral at `freq` Hz in place (bench frequency calibration; no reboot).

### `get(bus_id: int, spec: dict) -> Bus`

The shared Bus for `bus_id`, created once from `spec` (sck/mosi/miso/baud/mode) and cached.

## `task.py`

Task base class and driver registry — the unit the Controller creates and supervises.

Every component/system task follows the common lifecycle from specs/coludo.md:
setup() async; initialize or reset; return True on success
probe() async; ON-DEMAND self-test (the CC `probe` command, never at boot) -> None if healthy,
else an error string. Default None; a sensor reports 'X not found on i2c:0', an actuator
exercises itself (the servo sweeps its range) -- so a mid-flight reboot never sweeps fins.
run() async; the task's main activity loop
notify() subscribe a callback for this task's updates
validate() return True if the task is currently healthy
finish() async; shut down and release resources
A Task is Inspectable: inspect()/update()/stats() expose it to the operator (the Controller
registers each task with the Inspector), so there is no separate report().

A task registers itself with @activity('name') (or its alias @driver('name') for the HAL ones in
drivers/) into ACTIVITIES, the CLASS registry: name -> Task subclass, "what can be built". It is a
module global on purpose -- the decorators fill it at IMPORT time, before any Controller exists, so
it cannot live on a Controller instance (that is why moving it into the Controller would be a mess,
not a tidy-up). The Controller READS it (injected as `registry`, defaulting to ACTIVITIES) to build
a component, and keeps its own INSTANCE directory -- find()/query(), "what is currently running" --
for dependency lookup. Two deliberately separate lookups: class-by-name here, instance-by-name on
the Controller. The driver/activity names share one registry for now; splitting drivers out later.

### `activity(name: str)`

Class decorator: register a Task subclass (a HAL driver or a higher-level activity) under a
name so the Controller can build it from a config component.

### `class Task(inspector.Inspectable)`

- `__init__(name: str, config: dict=None, controller=None)` — constructor
- `note(template: str=None, arg=None) -> None` — De-duplicated best-effort run-loop log + runtime-health flag. Call `note()` (template None) on a
- `setup() -> bool` — Initialize or reset. Override. Return True on success, False otherwise.
- `probe() -> str` — On-demand self-test (the CC `probe` command, NOT run at boot): return None when healthy, or
- `run() -> None` — Main activity loop. Override. Default raises to catch missing overrides (the Controller
- `notify(callback) -> None` — Register callback(task, event) to be invoked on this task's updates.
- `emit(event=None) -> None` — Notify all subscribers of an update.
- `find(names: list[str]) -> list` — Non-blocking sibling lookup via the Controller (None for any not up).
- `query(names: list[str], waiting: bool=True) -> list` — Await sibling tasks by name via the Controller; with `waiting` (default) park until all
- `validate() -> bool` — Return True if the task is currently healthy.
- `finish() -> None` — Shut down and release resources.
- `inspect() -> dict` — Status dict. Subclasses extend it.

## `warmstart.py`

_Tested by `test/test_warmstart.py`._

warmstart.py — in-flight reboot recovery (specs/coludo.md "In-flight reboot & warm start").
A mid-air reset (watchdog, brownout-survivor, crash) must not turn the glider ballistic: the
sequencer drops a tiny BREADCRUMB into NVS at BOOSTING entry (never a VFS file — a filesystem
write locks the scheduler and wears the data flash; esp32.NVS commits to its own partition in
milliseconds) and clears it at DONE. At boot, main.py restores GLIDING when the breadcrumb AND
two physical signals agree — see should_restore() for the gate.

Storage layout: `flight` is a bare i32 flag (cheap to flip on the clear path), the payload is ONE
JSON blob (`crumb`) — full float precision, no per-field key bookkeeping, and a new field is a
dict entry rather than an NVS schema change. The module degrades to no-ops off-board (CPython).

### `save(launch: tuple, zone: tuple, pad_altitude: float, stamp: int) -> bool`

Drop the breadcrumb (called ONCE at BOOSTING entry, on the rod, before GC goes off).
`launch` = (lat, lon) of the live fix; `zone` = ((lat, lon) TL, (lat, lon) BR); `pad_altitude`
= the baro ABSOLUTE altitude at the pad (m — NOT the boot-relative elevation, a rebooted baro
re-zeroes mid-air); `stamp` = RTC epoch seconds. Returns False (and never raises) when NVS is
absent or full — a failed breadcrumb must not block a launch.

### `clear() -> None`

Down the flag (at DONE / after a rejected warm start). The blob stays — the flag alone
decides, so the clear is a single fast i32 write. Never raises.

### `load()`

The breadcrumb dict ({launch: [lat, lon], zone: [[TL], [BR]], pad_altitude, stamp}), or None
when no flight was in progress (flag absent/0) or the blob is missing/torn (-> cold boot).

### `should_restore(crumb, separated: bool, altitude, cause_is_reset: bool, now_s, min_height_m: float=15.0, max_age_s: int=600) -> tuple`

The warm-start gate — ALL must agree (defense in depth; any doubt -> cold boot):
  1. a breadcrumb exists AND carries its `pad_altitude` + `stamp` (a torn/partial JSON blob with a
     missing key REFUSES here rather than crashing the boot -- findings §21.1);
  2. the separation switch reads SEPARATED — the physical latch no software state can fake
     (post-separation it stays LOW for the whole glide; a stack on the pad reads nested);
  3. the baro ABSOLUTE altitude reads at least `min_height_m` above the breadcrumb's pad —
     still clearly in the air (None = baro not up in time -> refuse);
  4. `cause_is_reset` — machine.reset_cause() was WDT/SOFT/HARD. A battery insertion or power
     switch reads PWRON — exactly what a RECOVERY CREW's hands do to a glider that crash-landed
     on a rise above the pad (where gate 3 alone would pass). A mid-air brownout also reads
     PWRON and stays cold: a browning-out battery cannot be trusted to finish the glide;
  5. `age_s` = `now_s` - crumb stamp is positive and under `max_age_s`. The RTC survives soft/WDT
     resets, so the arithmetic holds exactly when a warm start is legitimate (even an unsynced
     RTC — continuity matters, not absolute truth); a power cycle restarts the RTC and breaks
     it -> cold. Age is computed HERE from the crumb's stamp, so a missing stamp refuses cleanly.
Pure function of its inputs (host-testable). Returns (restore, reason).

### `restore(flight, cfg: dict, log=print) -> bool`

Warm start (specs/coludo.md "In-flight reboot & warm start") — was main._restore_flight, moved
here so main.py stays a thin bring-up. A mid-air reset must not turn the glider ballistic: restore
GLIDING when the NVS breadcrumb AND two physical signals agree — the separation latch (read via the
separation DRIVER, not a raw Pin) and the baro absolute altitude clearly above the crumb's pad. Any
doubt -> the crumb is cleared and this is a normal cold boot. Heavy imports stay inside (nothing
beyond the cheap load() runs on a plain boot).

## `wind.py`

_Tested by `test/test_wind.py`._

wind.py — wind estimation from GNSS (plan #6). The MINIMAL method, proven first: the WIND TRIANGLE.

The GNSS reports the ground velocity (course + ground speed); the attitude gives the heading; the
governor gives the airspeed. The air mass the glider flies through is moving, so
wind = ground_velocity - air_velocity = (ground along course) - (airspeed along heading).
An EMA smooths the per-fix estimate. The CROSSWIND component is bias-free; the ALONG-heading component
inherits the governor's airspeed error (an over-read biases head/tailwind) -- acceptable for the rough
uses (reachability margin, approach crab). If the field data shows that bias hurts, the airspeed-free
GPS-only min/max-ground-speed method is the next layer (kept out until a corner case earns it).

Float trig, fed once per GNSS fix (off the hot loop), like navigation -- a telemetry + reachability /
approach input, not a fixnum control quantity.

### `class WindEstimator`

Estimate the wind (east/north m/s) from the GNSS ground velocity vs the air velocity (the wind
triangle, EMA-smoothed). update() once per GNSS fix; speed()/direction()/components() read it.

- `__init__(alpha: float=0.05)` — constructor
- `update(course: float, ground_speed: float, airspeed: float, heading: float) -> None` — One GNSS-fix update. `course`/`heading` deg, `ground_speed`/`airspeed` m/s.
- `components() -> tuple` — (east, north) m/s.
- `speed() -> float`
- `direction() -> float` — Where the wind blows FROM (meteorological convention), degrees. 0 when calm.
- `stats() -> dict` — Diagnostics for the wind soak / telemetry: the method, the estimate, and the raw components.

# glider HAL drivers — `drivers/` — `src/glider/drivers`

## `adxl375.py`

drivers/adxl375.py — ADXL375 ±200 g high-G accelerometer: the boost-phase accel channel. Works over
I2C (shared bus) OR SPI (its own bus, for clean high-rate reads) -- the component's `bus` field
selects, and a shared register-window device (i2cbus/spibus .device()) keeps the driver code
bus-agnostic. @task.driver('adxl375'). setup() probes the device id and configures it; run() writes
the latest (x, y, z) acceleration in g to the databoard 'accel' slot. If the device is absent (no
ack / wrong device id) setup() returns False and the Controller skips it -- the board boots fine
with the sensor unplugged.

Sampling is interrupt-driven when an `int_pin` (INT1) is wired: the chip raises DATA_READY when a
new sample is ready, an IRQ sets a ThreadSafeFlag, and run() awaits it -- so the coroutine sleeps
until there is genuinely fresh data instead of blind-polling. A `fallback_ms` timeout still forces
a sample if interrupts go silent (dead sensor / wiring). With no int_pin it falls back to a plain
`period_ms` poll. Uses the shared locked I2C bus (i2cbus), as it shares i2c:0 with other sensors.

### `class Adxl375(task.Task)`

High-G accel: samples (x, y, z) in g to the databoard 'accel' slot, interrupt-driven.

- `setup() -> bool`
- `sample() -> tuple` — Read and return (x, y, z) acceleration in g (also clears DATA_READY).
- `run() -> None` — Sample on DATA_READY (or every fallback_ms if interrupts go silent); plain poll with no
- `probe() -> str` — On-demand self-test: the device id reads back, then one sample succeeds (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: the bus reads our DEVID and classifies the wire-level
- `inspect() -> dict`

## `atgm336h.py`

drivers/atgm336h.py — ATGM336H GNSS (GPS + BDS, CASIC chip) on a dedicated UART. @task.driver(
'atgm336h'). All NMEA reading/parsing lives in the shared gnss.Gnss base; this driver only adds the
CASIC reconfiguration: RMC at `hz` (position) plus GGA at ~1 Hz (altitude/elevation, a baro backup)
-- both fit 9600 baud (~10 Hz RMC ~700 B/s + ~1 Hz GGA ~70 B/s < 960). PCAS is the CASIC command set;
the PMTK pair is sent too as a fallback for MTK-variant modules (each side ignores the other's
sentences). Graceful: an undefined bus -> setup False (the Controller skips it).

### `class Atgm336h(gnss.Gnss)`

ATGM336H (CASIC): RMC at `hz` for position + GGA at ~1 Hz for altitude/elevation.


## `bluetooth.py`

drivers/bluetooth.py — set the BLE radio to the state declared in config at boot. The component
field `radio` (true/false, default false) says whether Bluetooth should be ON; the driver applies
it -- transparent, so nobody is surprised by an implicit disable. Default false saves power (the
wireless is the external C6 and BLE is unused on the glider). Setup-only @task.driver('bluetooth')
plus update() so the operator can toggle it live (`update bluetooth {"radio": true}`).

### `class Bluetooth(task.Task)`

Apply the configured BLE radio state. Inspectable: `radio` requested, `active` actual.

- `probe() -> str` — On-demand self-test: the BLE radio is in the requested state (or absent on this board ->
- `setup() -> bool`
- `run() -> None` — Setup-only: no run loop. `update()` is the runtime entry point.
- `inspect() -> dict`
- `update(props) -> list`

## `bmp280.py`

drivers/bmp280.py — BMP280 barometric pressure sensor (on the SEN0253) over the shared I2C bus:
the backup altitude channel. @task.driver('bmp280'). setup() probes the chip id, reads the factory
calibration and starts normal-mode conversion; run() reads pressure, applies Bosch compensation
and writes pressure (Pa), temperature (°C), altitude (m AMSL) and elevation (m above the per-sensor
startup ground zero) to the databoard. Graceful: wrong/absent chip id -> setup False -> skipped.

Polled at period_ms (the BMP280 conversion is ~tens of ms, far slower than the IMU). Uses the
shared locked bus (i2cbus) since it shares i2c:0 with the ADXL375 and BNO055.

### `class Bmp280(task.Task)`

Backup baro: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation (m above the
startup ground zero, captured per-sensor so it is offset-free) to the databoard. `update`
{"rezero": true} re-captures ground zero (e.g. after warm-up, just before launch).

- `setup() -> bool`
- `run() -> None`
- `update(props: dict) -> list` — `{"rezero": true}` re-captures ground zero from the latest altitude (sync; operator does
- `probe() -> str` — On-demand self-test: the chip id reads back, then one conversion reads (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: the bus reads the chip id and classifies the fault (no
- `inspect() -> dict`

## `bno055.py`

drivers/bno055.py — BNO055 9-DOF IMU (on the SEN0253) over the shared I2C bus: the attitude
channel. @task.driver('bno055'). In NDOF fusion mode the chip computes absolute orientation
on-chip; run() reads the Euler angles (heading, roll, pitch in degrees) to the databoard
'attitude' slot. Graceful: a wrong/absent chip id -> setup False -> the Controller skips it.

BNO055's INT pin signals motion/threshold events, not a fusion data-ready, so this driver polls at
period_ms (the fusion engine runs at 100 Hz internally); the wired int_pin is reserved for future
event detection (e.g. high-g). Uses the shared locked bus (i2cbus) since it shares i2c:0 with the
ADXL375 and BMP280.

### `class Bno055(task.Task)`

9-DOF: attitude (heading, roll, pitch) deg -> 'attitude', plus the calibrated accelerometer
(g, incl gravity) -> 'accel' as a low-g backup to the ADXL375 (priority 1).

- `setup() -> bool`
- `sample() -> tuple` — Read the block and return a FLAT 6-tuple (run() slices): heading in float degrees (feeds the
- `run() -> None`
- `probe() -> str` — On-demand self-test: the chip id reads back, then one fused sample succeeds (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: the bus reads the chip id and classifies the fault (no
- `inspect() -> dict`

## `icp10111.py`

drivers/icp10111.py — ICP-10111 barometric pressure sensor (TDK ICP-101xx, on the SEN0517) over
the shared I2C bus: the PRIMARY altitude channel (8.5 cm accuracy). @task.driver('icp10111').
Command-based, not register-mapped: setup() verifies the product id and reads the 4 OTP calibration
constants; run() issues a measure command, reads pressure+temperature, applies the TDK polynomial
conversion and writes pressure (Pa), temperature (°C), altitude (m AMSL) and elevation (m above the
per-sensor startup ground zero) to the databoard. Graceful: wrong/absent id -> setup False -> skipped.

Polled at period_ms. Uses the shared locked bus (i2cbus); shares i2c:0 with the other sensors.

### `class Icp10111(task.Task)`

Primary baro: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation (m above the
startup ground zero, captured per-sensor so it is offset-free) to the databoard. `update`
{"rezero": true} re-captures ground zero (e.g. after warm-up, just before launch).

- `setup() -> bool`
- `run() -> None`
- `update(props: dict) -> list` — `{"rezero": true}` re-captures ground zero from the latest altitude (sync; operator does
- `probe() -> str` — On-demand self-test: the run loop is producing pressure. We issue NO I2C here -- the
- `diagnose() -> str` — Deeper analysis when setup() failed: re-issue the product-id command and classify via
- `inspect() -> dict`

## `ina226.py`

drivers/ina226.py — INA226 high-side current / voltage / power monitor over the shared I2C bus:
the battery (or 5 V) supply-line sensor for consumption tracking. @task.driver('ina226'). setup()
verifies the die id, programs the conversion config, and computes + writes the calibration register
from the shunt resistance + the expected max current (the only board-specific numbers); run() polls
the bus voltage (V), current (A) and power (W) to the databoard + telemetry. Graceful: wrong/absent
die id -> setup False -> the Controller skips it.

The INA226 measures the SHUNT VOLTAGE directly (2.5 uV/LSB), so the absolute accuracy comes from the
CAL register, not a precise resistor: Current_LSB = max_current / 2**15, CAL = 0.00512 / (Current_LSB
* shunt_ohms). To trust the watt-hours, calibrate `shunt_ohms` against a KNOWN current once and back
out the effective value -- a 2-wire ohmmeter cannot resolve a 0.01 ohm shunt.

### `class Ina226(task.Task)`

High-side power monitor: bus voltage (mV), current (mA) and power (mW) -- INTEGER milli-units, no
float -- to the databoard + per-sample telemetry. Current/power scale from `shunt_mohms` +
`max_current_ma` (the CAL register). The same driver serves the 5 V USB phase and the LiPo phase --
it reports the INA's own bus voltage, so power is correct as the base rail changes. Graceful: a
wrong/absent die id -> setup False.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: the die id reads back, then one live read (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: the bus reads the die id and classifies the wire-level
- `inspect() -> dict`

## `led.py`

led.py — status LED driver. One GPIO shows the board state at a glance: fast blink when a task is
unhealthy (error), slow blink while setting up / standing by, solid once flying. The pin role
(default 'led_status') comes from the component's `pin` field, resolved against the config `pins`
section. Registered as @task.driver('led') so the Controller creates and supervises it.

### `class LedStatus(task.Task)`

Blink a status pattern on one GPIO derived from the controller's state + health.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: blink the status LED a few times so it is seen to drive, then off.
- `inspect() -> dict`

## `lsm6dso32.py`

drivers/lsm6dso32.py — LSM6DSO32 6-DoF IMU: the primary raw accel + the sole gyro `rate`. ±32 g accel
(covers the 8-12 g boost without clipping, fine 1 g resolution for the airspeed integrator) + ±2000 dps
gyro. @task.driver('lsm6dso32'). setup() checks WHO_AM_I, configures accel/gyro, and provides both the
'accel' (x,y,z in g) and 'rate' (x,y,z in deg/s) databoard slots; run() writes the latest reading. If
the device is absent (wrong WHO_AM_I) setup() returns False and the Controller skips it.

Wired on SPI1 (its own chip-select, shared with the ADXL375) for clean high-rate reads — see
doc/waveshare_esp32p4_pins.md. SPI is 4-wire mode 3; multi-byte reads auto-increment via CTRL3_C.IF_INC
(so the bus device takes mb_bit=None — no address multi-byte bit). I2C (addr 0x6A) also works if the
component sets bus 'i2c'. Sampling is interrupt-driven on INT1 (accel data-ready) when an `int_pin` is
wired, else a plain period_ms poll, mirroring the ADXL375 driver. Gyro + accel sit in contiguous output
registers (0x22..0x2D), so one 12-byte read fetches both.

### `class Lsm6dso32(task.Task)`

6-DoF IMU: samples accel (x,y,z g) -> 'accel' and gyro (x,y,z deg/s) -> 'rate', interrupt-driven.

- `setup() -> bool`
- `sample() -> tuple` — Read and return a FLAT 6-tuple (run() slices it, no concat): accel (ax, ay, az) in float g, then
- `run() -> None` — Sample on INT1 data-ready (or every fallback_ms if interrupts go silent; plain poll with no
- `probe() -> str` — On-demand self-test: WHO_AM_I reads back, then one sample succeeds (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: the bus reads WHO_AM_I and classifies the wire-level
- `inspect() -> dict`

## `neo6mv2.py`

drivers/neo6mv2.py — GY-NEO6MV2 (u-blox NEO-6M) GNSS on a dedicated UART: a drop-in alternative to
the ATGM336H on the SAME UART -- swap the component `driver` to 'neo6mv2' in config (and lower `hz`;
the NEO-6M tops out near 5 Hz). @task.driver('neo6mv2'). NMEA read/parse is the shared gnss.Gnss base;
this driver only adds the u-blox reconfiguration: $PUBX,40 selects RMC (position) + GGA at ~1 Hz
(altitude/elevation) on the UART and silences the rest, then UBX-CFG-RATE sets the measurement
period. Default link is 9600 8N1, like the ATGM. Graceful: an undefined bus -> setup False.

### `class Neo6mv2(gnss.Gnss)`

u-blox NEO-6M: $PUBX,40 selects RMC + ~1 Hz GGA, UBX-CFG-RATE sets the measurement period.


## `separation.py`

drivers/separation.py — stage-separation switch: two adhesive copper pads (one on the glider, one
on the booster) that route 3V3 to a pin while nested (HIGH) and open on separation (LOW). A HAL
input, @task.driver('separation'). An IRQ on either edge wakes run(), which debounces, and on a
confirmed separation during the Boosting stage drives the documented Boosting -> Gliding transition
(the booster ejects the glider at apogee). The event is logged and emitted to subscribers; the
discrete event is NOT a databoard quantity (per specs/coludo.md, events use notify/log).

The pin uses an internal pull-down so an open (separated) circuit reads LOW reliably; while nested
the pads override it HIGH. A separation while not Boosting (e.g. a ground test in Setting) is
logged but does not transition -- the guard keeps go/no-go correct.

this transition calls controller.set_stage() directly, NOT the sequencer's _advance(), so it
does not write a row to sequencer.csv. That is deliberate -- separation is the PRIMARY Boosting ->
Gliding trigger and separation.csv (event + stage, durable) is its authoritative telemetry record;
the sequencer's burnout-timeout is only the fallback, and sequencer.csv records that fallback path.
A post-flight tool reading the BOOSTING->GLIDING reason must consult separation.csv first. (GC policy
is unaffected: gc.disable() already fired on the SETTING->BOOSTING transition.)

### `class Separation(task.Task)`

Detect stage separation (HIGH=nested -> LOW=separated) and trigger Boosting -> Gliding.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: the separation pin reads a valid level (logged nested/separated).
- `diagnose() -> str` — Deeper analysis: read the separation pin -- during a pre-flight check it should be HIGH (the
- `separated() -> bool` — The last debounced latch level as a bool (True = pads open = separated). The AUTHORITATIVE
- `inspect() -> dict`

## `sg90.py`

drivers/sg90.py — SG90 micro fin servo on a PWM pin. @task.driver('sg90'), one instance per fin
(yaw / left eleron / right eleron), each naming its `pin`. 50 Hz frame; the command unit is INTEGER
DEGREES, linearly mapped to a pulse width (min_us..max_us over min_deg..max_deg, integer math) and
CLAMPED to the range so a bad command can never drive the horn past the linkage.

OPEN-LOOP -- NO POSITION FEEDBACK. A 3-wire SG90 (signal / V+ / GND) only RECEIVES a PWM command;
the signal pin is input-only on the servo and there is no wire back, so the board CANNOT read where
the horn actually is. Everything this driver reports (inspect()/telemetry `angle`, `pulse_us`) is
the LAST COMMANDED value it tracks in software -- what we asked for, NOT a measurement. A stalled,
force-held or jammed surface would still read the commanded target. inspect() carries
`feedback: None` to make that explicit. (Real feedback would need a feedback servo, or tapping the
internal pot to an ADC, or a current-sense on the rail.) Separately, this MicroPython-P4 build's PWM
duty_u16()/duty_ns() GETTERS are broken (return a constant), so we cannot even read the commanded
duty back from the peripheral -- the driver only ever WRITES it and remembers what it set.

This class is SG90-specific on purpose. Other servos (MG90S, MG996R, ...) differ in pulse range and
behaviour and would be their own @task.driver -- a new drivers/<type>.py subclassing this or
standalone -- selected by the component's `driver` field. The shared slew gate + degree->pulse math
live here for now; factor them into a servo base when a second type lands.

Two ways to command a fin:
update {"angle": d} -- IMMEDIATE, ungated: the operator override (sync, returns at once).
await move(d) -- GATED + settle-aware: passes through a SHARED slew gate so at most
`servo_concurrency` (board config, default 3 = no limit) fins slew at
once, then awaits the estimated travel so the caller knows it has (open-
loop, no feedback) arrived. The flight control loop uses this.
Both record the command to per-fin telemetry (<name>.csv: angle, pulse_us, done) -- done=0 when a
command is ISSUED, done=1 when a move() has (estimated) COMPLETED. probe() is the on-demand self-
test (CC `probe`, pre-flight -- never at boot, so a reboot never sweeps fins): it sweeps the full
range and returns to neutral, logging each step.

Power: servos run off their own boost rail (per-pin diode protected); the board sources only the
low-current signal on the PWM pin, never the servo supply.

### `class SG90(task.Task)`

One PWM SG90 fin servo, commanded in integer degrees (clamped to [min_deg, max_deg]). OPEN-LOOP
-- reported angle is the last command, never a measurement (see module header; inspect carries
`feedback: None`). `update {"angle": d}` moves it immediately; `await move(d)` moves it through the
shared slew gate; probe() sweeps it on demand.

- `setup() -> bool`
- `run() -> None` — Command-driven: no run loop. `move()` / `update()` are the entry points.
- `probe() -> str` — On-demand self-test (CC `probe`, pre-flight -- never at boot): sweep min -> max -> neutral so
- `move(angle) -> int` — Drive to `angle` (clamped, integer degrees) through the shared slew gate -- at most
- `update(props: dict) -> list` — `{"angle": d}` moves the servo IMMEDIATELY (integer degrees, clamped) -- the operator
- `set_angle(angle) -> int` — The 100 Hz flight-loop hot-path command. AVOIDS update()'s per-step {'angle': ...} dict (
- `finish() -> None` — Release the PWM (stop driving the pin) on shutdown.
- `diagnose() -> str` — Deeper analysis when setup() failed: is the pin PWM-capable? Resolve the pin and try to bring a
- `inspect() -> dict`

## `vl53l4cx.py`

drivers/vl53l4cx.py — VL53L4CX time-of-flight laser ranger (Adafruit 5425) over the shared I2C bus:
the above-ground-level (AGL) channel for the last metres of the glide, where the barometer is
useless. @task.driver('vl53l4cx'). The VL53 family uses 16-BIT register addresses (i2cbus addrsize=
16). This part is the newer 0xEBAA silicon (shared by the VL53L4CD/L4CX), so it uses the VL53L4CD
Ultra-Lite-Driver init -- the older VL53L1X (0xEACC) config does NOT produce ranges on it.

setup(): optional XSHUT reset -> wait for boot -> write the default configuration -> run one VHV
calibration ranging cycle (start/wait/clear/stop, then the VHV config writes) -> start continuous
ranging. run(): wait for data-ready (the GPIO1 interrupt if wired, else a poll), read the distance
and write AGL (m) to the databoard. Single-target distance; the L4CX multi-target extras are unused.
Graceful: no I2C ack -> setup False -> Controller skips it. Shares i2c:0 via the locked i2cbus.

### `class Vl53l4cx(task.Task)`

Laser ToF: writes above-ground-level distance (m) to the databoard 'agl' slot, for the final
low-altitude metres where the barometer cannot resolve height. Interrupt-driven when GPIO1 wired.

- `setup() -> bool`
- `run() -> None` — Sample on data-ready (GPIO1) or every period_ms; write AGL (m) to the databoard. Runs
- `probe() -> str` — On-demand self-test: the model id reads back -- a single locked op, safe alongside the run
- `diagnose() -> str` — Deeper analysis when setup() failed: re-read the 16-bit MODEL_ID high byte (0xEB) and
- `inspect() -> dict`

## `wifi.py`

drivers/wifi.py — Wi-Fi station driver: joins the configured network and keeps it joined, exposing
signal/ip to the operator. HAL (it drives the radio), so @task.driver('wifi'). STA only; SSID / CC
host / TX power come from the `wifi` section of board.config, the password from <ssid>.creds
(gitignored, deploy.sh-pushed).

Optional + telemetry-first + NON-BLOCKING BOOT: setup() never touches the radio (it only reads
config), because bringing the STA link up can block and would stall the serial boot -- so the board
ALWAYS boots and flies, with or without Wi-Fi. The radio comes up lazily in run(), which (re)joins on
an interval ONLY until ignition (after BOOSTING it idles, never competing with the flight loop). A
board with no Wi-Fi just logs once and flies standalone -- no Wi-Fi means no CC, nothing more.

### `class Wifi(task.Task)`

Join + maintain the STA link; Inspectable as `wifi`.

- `setup() -> bool` — NON-BLOCKING: only read config; the radio is brought up lazily in run(). Bringing the
- `run() -> None` — (Re)join every `retry_ms` -- but ONLY on the ground. From BOOSTING through LANDING the
- `connect(timeout_ms: int=15000) -> bool` — Join the configured network. Returns True once connected, False on timeout/error.
- `isconnected() -> bool`
- `ifconfig() -> tuple`
- `ip() -> str`
- `rssi() -> int`
- `set_tx_power(dbm: int) -> bool` — Adjust the TX power (operator signal-level tuning). Returns True on success.
- `diagnose() -> str` — Dump the Wi-Fi link state to the console AND the recorder log, and return the one-line summary.
- `inspect() -> dict`
- `update(props: dict) -> list`
- `stats() -> dict`

# glider subsystem tasks — `tasks/` — `src/glider/tasks`

## `attitude.py`

tasks/attitude.py — attitude REDUNDANCY: a complementary-filter backup for the BNO055 (plan item 4;
coludo.md "Sensors Fusion/Backup"). The BNO055 is the sole fused-attitude source; losing it mid-flight
would leave the flight loop with stale/absent attitude -> neutral fins -> ballistic. This task derives
(heading, roll, pitch) from the LSM6DSO32 gyro `rate` + accel gravity vector and PROVIDES it on the
databoard at PRIORITY 1, so the existing timeout-handoff fusion swaps to it automatically the moment the
BNO055 (priority 0) stops -- no change to flight.py.

@task.activity('attitude'). Two regimes, checked each cycle by the fused-attitude SOURCE:
* BNO055 alive (it is the fused source): MIRROR it -- copy roll/pitch (already fixnum cd) + heading,
staying warm and FRESH so the handoff is seamless, no atan2/accel math (the BNO055 is trusted).
* BNO055 lost (source is us / extrapolated): FREE-RUN -- integrate the gyro rate (integer) and, when
|accel| ~ 1 g (a trustworthy gravity vector), pull roll/pitch toward the accel angle via the integer
CORDIC fixed.atan2_cd (throttled -- drift correction is slow). Heading is gyro-z only (it drifts: the
LSM6DSO32 has no magnetometer) -- roll/pitch stay solid (gravity-referenced), so the glider holds
wings-level + pitch; nav heading degrades gracefully. Integer/fixnum throughout; the only boxed float
is the heading value the channel format requires (nav consumes heading as float degrees).

Mounting (the gyro-D-term convention, HITL-validated): gx->roll, gy->pitch, gz->yaw; accel roll =
atan2(ay, az), pitch = atan2(-ax, |ay,az|). Field calibration flips a sign like the mixer gains.

### `class Attitude(task.Task)`

Complementary-filter attitude backup (heading, roll, pitch) at priority 1 behind the BNO055.

- `setup() -> bool`
- `run() -> None` — Mirror the primary while it is the fused source; free-run the complementary filter when it is
- `probe() -> str` — On-demand self-test: the gyro `rate` is present (the backup's core input). A dead gyro means
- `inspect() -> dict`
- `stats() -> dict`

## `board_health.py`

tasks/board_health.py — board vitals task: samples temperature, free memory and CPU load every
period, pushes a telemetry row (health.csv) and exposes the latest to the operator. Registered as
@task.activity('health') so the Controller creates and supervises it.

CPU load (an integer percent 0..100) is estimated WITHOUT busy-spinning: a probe task sleeps a fixed
period (probe_ms) and measures how LATE it actually wakes. asyncio.sleep_ms only resumes once the
event loop is free, so time other tasks spend running delays the wake-up -- the overshoot beyond the
nominal sleep is the time the CPU was busy with other work:
load% = round(100 * (elapsed - probe_ms) / elapsed).
Sleeping rather than spinning on sleep_ms(0) lets the core actually idle between probes (FreeRTOS idle
/ WFI) -- much lower idle power draw (the old spin pinned the CPU at ~100%). No calibration baseline
is needed (it is absolute). test_board_health drives a CPU hog and asserts the load rises with it.

### `class BoardHealth(task.Task)`

Periodic vitals -> telemetry (health.csv) + `inspect health`.

- `setup() -> bool`
- `temperature() -> float`
- `mem_free() -> int`
- `sample() -> dict`
- `oom_s()` — Predicted seconds to memory exhaustion from the current decay slope, or None while the
- `land_s()` — Predicted seconds until the glider sinks to the rescue floor (rescue_agl_m), from the
- `run() -> None` — Push a vitals row at startup, then every period_ms. A probe task tracks CPU load. Runs
- `probe() -> str` — On-demand self-test: free memory reads positive (a basic board-vitals sanity); the
- `inspect() -> dict`
- `stats() -> dict`

## `cc_link.py`

tasks/cc_link.py — the Control link task: once Wi-Fi is up it dials the CC hub and serves the
command dispatcher, reconnecting with backoff. @task.activity('cc'). Telemetry-first: with no Wi-Fi
up it simply waits, so the board flies fine without CC. The hub address is the configured `cc_host`,
or -- when unset -- the `.1` of whatever subnet the board joins (the Control hub by convention), so
a board reaches its hub on any network. An empty `cc_host` ('') disables CC entirely (standalone).
The dispatcher is wired to this board's config + Controller.

### `class ControlLink(task.Task)`

Serve the CC protocol to the hub when the link is available; never fatal. With no `cc_host`
configured the board dials the `.1` of whatever subnet it joins (the Control hub by convention);
an empty `cc_host` ('') disables CC and the board flies standalone.

- `setup() -> bool`
- `run() -> None` — Park until the Wi-Fi dependency is up, then dial CC and serve until the link drops; retry.
- `probe() -> str` — On-demand self-test: the CC hub address resolves (explicit or derived) and the Wi-Fi

## `field.py`

tasks/field.py — the CC-less field agent (specs/coludo.md "Field operation without CC").
@task.activity('field'), DISABLED by default. On the pad (SETTING) it makes at most two decisions:
1. SITE BY GPS — on the first fresh fix, the mission adopts the nearest launch.config site
within max_range_m; none in range -> the synthesized spiral-landing fallback zone offset
from the fix at the configured clear-sector bearing.
2. AUTO-ARM (opt-in) — arm once the board has sat STATIONARY with a live fix for the whole
auto_arm_dwell_s. The long dwell makes a bench/carry arm unlikely, and the flight loop's
control-stage gating still holds the fins neutral on the ground either way.
Each decision fires once, then the task idles; the operator/CC can still override everything live.

### `class Field(task.Task)`

Site-by-GPS + optional auto-arm, so a board can fly with no Control hub present.

- `setup() -> bool`
- `run() -> None`

## `flight.py`

tasks/flight.py — Phase 3 stabilization loop. @task.activity('flight'). At `schedule_hz` it runs the
control PIPELINE: dt -> airspeed Governor (fin-authority cap, adaptively throttled) -> control-stage
gate -> attitude -> Guidance (per-stage setpoints + heading) -> PID per axis -> mixer actuate. The
control LAW lives in guidance.py and the airspeed/authority POLICY in governor.py (doc/plan.md
structural roadmap #1) — this task is the orchestration: databoard reads, arming/degraded gates,
scheduling, and the PID->mixer->servo drive. Per-stage behaviour, the GPS-degrading heading tiers,
boost hold and final approach are guidance.py's; the adaptive estimator throttle is governor.py's.
Degraded: stale/absent attitude -> neutral. Disarmed / non-control stage -> neutral.

Scheduling: schedule_hz > 0 -> a machine.Timer ticks the step, so the control law gets a regular slice
independent of what other asyncio tasks are doing (deterministic, e.g. while the laser hammers I2C in
landing). schedule_hz == 0 -> a plain asyncio loop at period_ms (reconfigure/debug; subject to the ~10 ms
asyncio floor). Default 100 Hz timer = ~1 m per control step at 100 m/s. Gains default to 0 and the
task is disabled by default -- it cannot move a surface until enabled + tuned on the airframe.

### `class Flight(task.Task)`

Attitude-hold stabilization: GLIDING-gated, timer- or asyncio-scheduled, fail-safe to neutral.

- `setup() -> bool`
- `run() -> None`
- `finish() -> None`
- `progress() -> tuple` — (controlling, steps, stage, updated_us) -- the public control-loop heartbeat, so the watchdog
- `vitals() -> dict` — The live flight-panel readout (CC dashboard): the governor's airspeed estimate + the
- `inspect() -> dict`

## `hitl.py`

tasks/hitl.py — Hardware-In-The-Loop flight simulator (Phase-5). @task.activity('hitl').

Closes the control loop ON THE BOARD without changing any production code: it reads the commanded fin
angles from the cached servo tasks, steps a flight-dynamics model (sim_model.Body), and PROVIDES the
resulting sensor quantities on the databoard at priority 0 -- so sequencer.py / flight.py / pid /
mixer / navigation read it and cannot tell it is simulated. The full chain runs closed-loop: sim
sensors -> sequencer (stage machine) -> flight (PID -> mixer -> fins) -> back into the model. Use with
config_hitl (real sensors off, this on, flight + sequencer enabled, watchdog off). The physics live in
sim_model.py (pure, shared with the host-side tools/virtual_flight.py -- same model, both worlds).

Fidelity: BOOST adds attitude under thrust -- a crosswind weathercocks the stack and the boost
stage's guarded fins fight to hold it vertical, on top of the vertical 1-DoF that drives launch detect +
apogee; the GLIDE is a rigid body with roll/pitch/yaw state driven by the elevon/rudder deflections the
flight loop commands (that is where the rest of control happens). Aero is simplified and the
coefficients are deliberately tunable -- the point is a stable, closed loop that exercises the control
code, not aerodynamic truth. Outputs are perturbed by a noise level N and optional 2x spikes
to study sensor-quality degradation (e.g. the laser dropping out beyond its range).

The simulated sensors are ALSO recorded as telemetry under the SAME csv names/fields as the real
drivers (accel_adxl375 / imu_bno055 / baro_icp10111 / gnss / laser_agl + a combined fins), so an
on-board HITL run produces a COMPLETE, renderable capture on the Luckfox (flight_report/flight_svg),
not just health/sequencer/servo. The records are decimated so the recorder link keeps up.

### `class Hitl(task.Task)`

The HITL simulator task: drive the model from the commanded fins and publish simulated sensors.

- `setup() -> bool`
- `run() -> None`
- `inspect() -> dict`

## `recorder.py`

tasks/recorder.py — the Recorder's task adapter. The data path itself is the top-level `recorder`
singleton (used directly by every module via recorder.Recorder.log/tlm); this thin @task.activity
plugs it into the Controller's task graph so the `recorder` component (its bus selects the UART)
is created and supervised like any other task. No 'uart_sink' abstraction -- the Recorder is it.

### `class RecorderTask(task.Task)`

Owns the Recorder's setup + drain loop and surfaces it to the operator; everything else
keeps logging/telemetering through the global recorder.Recorder.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: the Recorder rings are up and a probe log line writes through them.
- `inspect() -> dict`
- `stats() -> dict`
- `update(props) -> list`

## `sequencer.py`

tasks/sequencer.py — Phase 3 flight-stage automation. @task.activity('sequencer'). Watches the
databoard and drives the guarded, forward-only stage machine that the control loop gates on:
SETTING -> BOOSTING : |accel| over launch_g sustained launch_ms (motor ignition), OR the baro climbing
past launch_alt_m off the pad (an independent, threshold-robust backup)
BOOSTING -> GLIDING : the separation switch (drivers/separation.py) is primary; else the baro APOGEE
detect (peak - apogee_drop_m, at the top of the arc, mass/motor-independent); burnout timeout last
GLIDING -> LANDING : agl below land_agl_m (the laser sees the ground; elevation is the fallback)
LANDING -> done : |accel| ~1 g (stationary) sustained ground_ms (on the ground)
Each transition fires once (the stage check + reset-on-change is the guard), logs the reason and a
sequencer.csv telemetry marker. Thresholds are config; launch_g/launch_ms is exactly what the
E16/F15 passive flights tune. One control-independent tick, so it runs on the passive flights too
(stages logged, no actuation -- the flight task stays disabled).

### `class Sequencer(task.Task)`

Drive the flight-stage machine from sensor signals (forward-only, guarded, logged).

- `setup() -> bool`
- `finish() -> None`
- `run() -> None`

## `watchdog.py`

tasks/watchdog.py — Phase 3 watchdog + heartbeat supervisor. @task.activity('watchdog'). Two layers:
1. a hardware machine.WDT fed every period -> a TOTAL event-loop wedge (any task stuck below the
await level, a hung I2C bus) stops the feed and the board hard-resets. The backstop.
2. a heartbeat check of the CONTROL LOOP: while the flight task is in a control stage it must keep
ticking (its step counter advances). A stalled control loop (live scheduler, dead control) ->
reset, since a soft restart cannot preempt a wedged native call and the HW (PWM, the I2C bus,
sensors mid-transaction) needs a clean reset to be trustworthy.
Recovery is a full machine.reset() (fast on the P4; boot re-centres the fins) -- a soft event-loop
restart is unreliable here. The flight loop already fail-safes to neutral on stale attitude (degraded
mode), so that is NOT a watchdog trigger. Disabled by default -- a live WDT also resets the board when
you drop the running firmware to the REPL for bench work; enable it for flight.

### `class Watchdog(task.Task)`

Feed a hardware WDT (wedge backstop) + supervise the control loop (stall -> full reset).

- `setup() -> bool`
- `kick() -> None` — Out-of-band feed for a caller about to LEGITIMATELY block the loop -- the memory
- `run() -> None`

# control (CPython) — `src/control`

## `board.py`

_Tested by `test/test_board.py`._

board.py — one connected Coludo board as seen by the hub: lockstep request/response over its
socket (specs/cc-protocol.md). The per-board lock makes every exchange strictly sequential, so the
heartbeat and operator traffic to one board can never overlap. CPython 3.12, stdlib asyncio only.

### `class Board`

One connected board: lockstep request/response over its socket.

- `__init__(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, log=None)` — constructor
- `peer() -> str` _(property)_
- `exchange(line: str, timeout: float=EXCHANGE_TIMEOUT_S, quiet: bool=False) -> cc._Msg` — Send a ready board-facing line and return its parsed reply (None if disconnected).
- `properties() -> dict` — The Control-side snapshot of this board: identity + the cached config/inspect/stats/health
- `command(command: str, *args, timeout=EXCHANGE_TIMEOUT_S, quiet=False) -> cc._Msg` — Build `command args...` and exchange it. Returns the parsed reply or None.
- `identify() -> str`
- `inspect(name: str) -> dict`
- `close() -> None`

## `gps.py`

_Tested by `test/test_gps.py`._

gps.py — host-side GPS assist for the Control hub (finding #10).

The flight board carries its own GNSS (ATGM336H); a GPS plugged into the Control host (e.g.
/dev/ttyUSB0) is an ASSIST, not the source of truth. Two jobs:
1. tell the operator when a usable fix is available — the ideal launch condition is a 3D fix
with 4+ satellites (so the board's own cold start has a good almanac/position seed);
2. hand a launch position to the board (operator `assist <board>` -> `update mission` +
`set-config launch`, persisted in the board's launch.config) when the on-board GPS has no fix yet.

Pure NMEA parsing (GGA position/sats, GSA 2D/3D mode) is split from the serial transport so it is
unit-tested without hardware (test_gps.py); the Linux serial open + read loop is exercised by
itest_gps.py against a real receiver. CPython 3.12, stdlib asyncio only — no pyserial.

### `class Fix`

The latest GNSS fix, accumulated from GGA (position/altitude/satellites) and GSA (2D/3D).

- `__init__()` — constructor
- `fix_3d() -> bool` _(property)_
- `has_position() -> bool` _(property)_
- `usable() -> bool` _(property)_ — The ideal launch condition: a 3D fix with enough satellites and an actual position.

### `class Gps`

Host GPS reader: feed NMEA lines, expose the latest fix + a launch position for board assist.

- `__init__(log=print)` — constructor
- `feed(line: str) -> bool` — Parse one NMEA sentence into the running fix. Returns False for non-NMEA, a bad checksum,
- `status() -> dict` — Operator-facing fix snapshot: is it a usable 3D fix, how many satellites, where.
- `position()` — The host position as a mission dict (latitude/longitude[/altitude]) when the fix is
- `run(reader: asyncio.StreamReader) -> None` — Feed every line from an NMEA stream until it ends (the read loop, transport-agnostic).
- `serve(device: str, baud: int=9600) -> None` — Open the serial GPS and feed it forever (the wired host-assist path). A device that cannot be

### `open_serial(device: str, baud: int=9600) -> asyncio.StreamReader`

Open a Linux serial tty as an asyncio StreamReader: raw 8N1 at `baud`, stdlib only (termios +
connect_read_pipe). Hardware path — covered by itest_gps.py, not the host unit tests.

## `main.py`

main.py — CLI entry point for the Control hub. Run it headless on a LAN box (it binds 0.0.0.0 by
default) and telnet / browse to it from another workstation, instead of opening a browser locally.

python3 main.py [--host H] [--port N] [--operator-port N] [--web-port N]   (--help for all)

### `main() -> None`

## `server.py`

_Tested by `test/test_server.py`._

server.py — the Control hub: a board listener (1234) + per-board heartbeat + a telnet operator
console (1235), plus the web bridge (web.py, 8080). Boards dial in, Control learns each id via
whoami/iam and owns every exchange. An operator line whose first token is a board id or `all`
routes to that board (id stripped, the rest forwarded verbatim) and the reply is tagged
`from <board> ...`; any other first token is a Control command from the drop-in commands/ registry.
CPython 3.12, stdlib asyncio only. cc_protocol.py is shared with the firmware (symlinked).

### `class Server`

The hub: a board listener + per-board heartbeat + an operator console. `on_board` is an
optional async hook invoked once, right after a board identifies (used by integration tests).

- `__init__(host: str='0.0.0.0', port: int=1234, operator_port: int=1235, web_port: int=8080, on_board=None, log=print, heartbeat_s: float=HEARTBEAT_S, gps=None)` — constructor
- `board_rows() -> list` — The registry as json-able rows — shared by the `list` operator command and the web
- `cc_status() -> dict` — The Control host's own status for the dashboard header: the wall clock and the host GPS
- `start_stream(client, interval_ms, kind='log') -> None` — (Re)start streaming a board's `kind` ('log'|'tlm') at `interval_ms`, replacing any running
- `stop_stream(board_id) -> None` — Stop streaming a board and tell it to stop collecting (a final `<kind> 0` drain), so it does
- `serve_forever() -> None` — Accept board connections on `port` (board-facing listener).
- `serve_operators() -> None` — Accept operator connections on `operator_port` (telnet-friendly console).
- `run() -> None` — Run the board listener, operator console, and web bridge until cancelled.

## `web.py`

Web bridge — the browser face of the Control hub (specs/cc-protocol.md "Browser bridge").

A minimal HTTP/1.1 + SSE server on 8080 over the same stdlib asyncio loop as the board listener
and operator console (no extra dependency, no framework). Plain HTTP: the LAN is trusted and
encryption is out of scope (cc-protocol.md "Transport & ports"). Routes:
GET  /             -> the one-page dashboard (static/index.html)
GET  /api/boards   -> hub.board_rows() as JSON (same data as the `list` command)
POST /api/cmd      -> {board, command, params} -> run it on the board, reply as JSON
POST /api/op       -> {line} -> run an operator-console line (calibrate, ...) -> {lines}
GET  /events       -> Server-Sent Events: the board list pushed every heartbeat (live table)

### `class Web`

The HTTP/SSE server. Holds the hub for the board registry + routing; one per hub.

- `__init__(hub, host: str='0.0.0.0', port: int=8080, log=print)` — constructor
- `serve() -> None`

# control operator commands — `commands/` — `src/control/commands`

## `assist.py`

`assist <board>` — push the host GPS position to a board's mission (sync the launch site), then
persist it to the board's launch.config. Only sends a usable 3D fix; defaults to the selected
board. Requires a GPS attached to the Control host (main.py --gps-device).

### `assist_command(hub, tokens, session) -> list`

## `cache.py`

`cache <board>` — the Control-side cached properties for a board (config / inspect / stats /
health), last-known values without touching the board. Defaults to the session's selected board.

### `cache_command(hub, tokens, session) -> list`

## `calibrate.py`

`calibrate <board> <i2c|spi> <id> [margin-steps]` -- find a sensor bus's max stable frequency.

Drives the board's `bustune` primitive (retune-in-place + per-device health) UP a frequency ladder,
stopping at the first step any device fails. Reports the ceiling (highest all-healthy step), the
LIMITING device (first to drop out -> the one to rewire / move off the shared bus), and a `chosen`
freq backed off `margin` ladder steps for headroom (default 1 -- your MAX-1 rule). Restores the bus
to its configured freq afterwards; it does NOT persist. To apply, the operator runs the printed
`set-config board ... + reboot` (the immutable-config activation path). The sweep lives here on CC,
the board only executes one retune-and-test step at a time.

### `calibrate_command(hub, tokens, session) -> list`

## `gps.py`

`gps` — the host GPS fix status (3D + satellites), so the operator knows when the launch site has a
usable position. `gps <board>` also fetches that board's on-board GNSS (`inspect gnss`) and shows it
beside the host fix, to check what the on-board receiver delivers against the USB reference before
trusting it / using `assist`. Requires a GPS attached to the Control host (main.py --gps-device).

### `gps_command(hub, tokens, session) -> list`

## `help.py`

`help` — list operator commands, or `help <command>` for one.

### `help_command(hub, tokens, session) -> list`

## `list.py`

`list` — the connected boards and their last-known status.

### `list_command(hub, tokens, session) -> list`

## `select.py`

`select <board>` — set this session's sticky target; a later bare command routes to it.

### `select_command(hub, tokens, session) -> list`

## `who.py`

`who` — show this session's currently selected board.

### `who_command(hub, tokens, session) -> list`
