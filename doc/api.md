# Coludo API reference

_Generated from module docstrings by `tools/gen_docs.py` — do not edit by hand; run `python3 tools/gen_docs.py` to regenerate._

See [`architecture.md`](architecture.md) for the module dependency graph, class hierarchy, and the annotated `Flight._step()` hot-path call tree (`tools/gen_graph.py`).

# glider firmware (MicroPython) — `src/glider`

## `airspeed.py`

_Tested by `test/test_airspeed.py`._

Hybrid airspeed estimate for the dynamic-pressure fin governor (coludo.md "Fin authority"). Three
sources, in order of trust:
  * a PITOT/static differential-pressure reading (measure) -- a DIRECT airspeed measurement (sdp810.py
    -> governor), the preferred source WHEN IN-BAND: it measures the air, not the ground, so no wind
    offset and no attitude gate. Saturates past its full scale (boost / a steep dive), where the caller
    falls back to the accel backbone rather than trust an under-read;
  * accelerometer integration is the BACKBONE (predict) -- always running, and the only usable source
    during boost and right after separation, when the pitot rails and GNSS is jittery under high dynamics;
  * a valid, sane GNSS ground speed nudges out the integrator's drift (correct) -- a complementary
    filter used when the pitot is absent/stale, GNSS as the slow truth, accel as the fast signal.
GNSS is DISTRUSTED by default: rejected without a fix and above a physical ceiling (a 100+ m/s reading
under separation is a glitch), and only ever BLENDED (never a hard replace) so one bad-but-in-range
sample cannot jump the estimate; repeated good fixes pull the drift out. The estimate is biased to
over-read when uncertain -- a high airspeed tightens the governor cap, which is the safe direction.

CONFIDENCE: a freshly-constructed estimator (cold boot, or a MID-AIR RESET that re-runs setup) reads 0
before anything charges it -- and a 0 airspeed would open the governor's fin cap to full 45deg at high
dynamic pressure (the unsafe direction). `confident()` reports whether a TRUSTED speed exists yet: it
flips True once the accel integrator charges a clearly-airborne speed (boost / a building dive) or the
first sane GNSS fix anchors it (a direct set, not a slow blend). Until then the governor caps
conservatively rather than off the un-charged 0. `confident` is a latch -- once trusted, it stays so.

WARM START: the conservative cap keeps a recovered glider SAFE, but it is a blunt instrument -- pinned
at `airspeed_unconfident_ms` (30 m/s) it also starves fin authority at the moment a glider that just
rebooted mid-air most needs it. `seed()` closes that window from the warm-start crumb, so the recovery
order is pitot -> saved -> GNSS: the crumb is available IMMEDIATELY at restore (no fix wait), the accel
backbone integrates on from it, and the pitot -- a direct measurement -- overrides on its first in-band
read. GNSS is last precisely because a fix can take tens of seconds to reacquire after a reset.

### `class AirspeedEstimator`

Fuse integrated body acceleration with sanity-gated GNSS ground speed into one airspeed estimate.

The airspeed estimate (m/s) feeds the fin governor. Stateless of HOW accel-along-path is derived --
the caller passes it (e.g. |accel| - g during boost), so this stays unit-testable on the host.

- `__init__(ceiling_ms: float=60.0, gnss_gain: float=0.2)` — constructor
- `value() -> float` — The current airspeed estimate (m/s).
- `confident() -> bool` — Whether the estimate is trustworthy yet (see module header).
- `seed(airspeed: float) -> float` — Restore a PERSISTED airspeed (the warm-start crumb) as the starting estimate.
- `predict(accel_along: float, dt: float) -> float` — Integrate net acceleration ALONG the flight path over `dt` seconds -- the backbone.
- `correct(gnss_speed: float, has_fix: bool) -> float` — Blend toward a GNSS ground speed ONLY if trustworthy: a live fix and within the physical ceiling.
- `measure(airspeed: float, gain: float) -> float` — Fold a DIRECT airspeed measurement (the pitot) -- the PREFERRED source when it is available.

## `cc_client.py`

_Tested by `test/test_cc_client.py`._

Board side of the Control protocol (doc/specs/cc-protocol.md). Board-first routing: Control strips the
routing board id, so the board receives `command params` and replies `status params` (no id; only
`iam` carries the board id, so Control can learn it on a new socket). Dispatcher turns a parsed line
into a response (pure logic, unit-testable); Client is the thin networking that reads lines and
writes responses.

### `class Dispatcher`

Command name -> async handler(msg) registry that turns a request line into a response line.

Pure routing with no networking (Client owns the socket), so the command logic stays
unit-testable off a live connection. Register handlers with on(); dispatch a line with handle().

- `__init__()` — constructor
- `on(command: str, fn) -> None`
- `handle(line: str) -> str`

### `class Client`

The thin CC networking: dial Control, read request lines, write the dispatcher's responses.

- `__init__(config: dict, dispatcher, log=None, backoff_ms: int=1000)` — constructor
- `run() -> None` — Connect to Control and serve forever, reconnecting with backoff on drop.
- `serve(reader, writer) -> None` — Read commands from Control, dispatch, write responses. Returns on disconnect.

### `create_dispatcher(cfg: dict, controller=None, on_reboot=None, config_path: str='board.config') -> Dispatcher`

Build a Dispatcher with the standard command handlers.

Wires the handlers to the running config, the Inspector, and (optionally) the Controller.
`on_reboot` lets tests intercept the reset. The handlers are grouped by concern into the
_register_* helpers; each closes over one shared _Context, so this stays a short orchestrator.

Args:
    cfg - the running board config.
    controller - the flight Controller, or None (unit tests / recorder-only nodes).
    on_reboot - a reset hook the reboot handler calls, or None to fall back to machine.reset().
    config_path - where set-config board / reset-config persist the board config.

Returns:
    The wired Dispatcher.

## `cc_protocol.py`

_Tested by `test/test_cc_protocol.py`._

CC <-> board line protocol (doc/specs/cc-protocol.md).

One newline-delimited message per line:  <command> <board-id> [params...]. Tokens are
whitespace-separated, so there is NO quoting or escaping. A param value is one of:
  * bare token    -> a simple value with no spaces (e.g. 3000, taster, 192.168.10.1)
  * base64:<data> -> anything else: spaces, quotes, JSON, binary
Both sides know each command's schema, so the parser does not guess types: a bare token is returned
as a str and the receiver converts numerics itself (it knows `ms` is an int). Named params are
key=value; everything else is positional. The command is lowercased; values keep their case. parse()
handles requests and responses (ok/err/pong/iam) alike.

### `encode(v) -> str`

Encode a value into one whitespace-free wire token.

Args:
    v - the value to encode (bool / int / str / other via str()).

Returns:
    The wire token: bare when already safe, else 'base64:'-prefixed.

### `decode(tok: str) -> str`

Decode a wire token back to a str.

Args:
    tok - the wire token.

Returns:
    The decoded string (base64-decoded when 'base64:'-prefixed, else the token as-is).

### `parse(line: str) -> _Msg`

Parse a protocol line into a _Msg (works for requests and responses).

Args:
    line - the raw newline-stripped protocol line.

Returns:
    A _Msg; its command is None for an empty line.

### `build(command: str, args=(), named=None) -> str`

Build a protocol line from a command and its params.

Args:
    command - the command (or response) keyword.
    args - positional param values, encoded as needed.
    named - key -> value params, emitted as key=value (encoded), or None.

Returns:
    The assembled single-line protocol string.

## `commons.py`

_Tested by `test/test_commons.py`._

Small, dependency-free primitives shared across the control-math modules (mixer / pid / navigation /
guidance / governor / sequencer / flight / sg90). The bundle module for the plan.

Layout, one section per concern: COMPATIBILITY (every MicroPython/CPython shim, in one place) ->
CONSTANTS -> INTEGER MATH (viper) -> FLOAT MATH (native) -> FIN GOVERNOR -> PERSISTENCE ->
WIRE DIAGNOSTICS.

Naming convention: a plain name is a leaf with no _opt variant at all. A NAME_upy / NAME_opt pair
plus `NAME = <winner>` is a function with an optimised variant -- NAME_upy is the portable bytecode
reference; NAME_opt is the optimised build (viper for ints, native for floats, future asm). The
module binds NAME to whichever the on-board bench FAVOURS -- usually _opt; switch the one alias line
if a measurement changes. Both forms stay public so benchmarks/tests call them DIRECTLY (no runtime
selector). Bound here: clamp_int, wrap180 (@viper, ~2.1-2.8x); between, magnitude_sq (@native,
~1.2-1.6x); bank_demand -> _upy for now (its @native measured 1.03x -- a thin wrapper over native
between; switch to _opt when a bench shows a gain).

### `clamp_int_upy(low: int, value: int, high: int) -> int`

### `clamp_int_opt(low: int, value: int, high: int) -> int`

### `wrap180_upy(degrees: int) -> int`

### `wrap180_opt(degrees: int) -> int`

### `between_upy(low: float, value: float, high: float) -> float`

Clamp `value` to the inclusive range [low, high]: `low` if below, `high` if above, else `value`.

With low=-x, high=+x it is a symmetric +/-x clamp; either bound may be math.inf for an open side
(between(-inf, v, inf) == v). Float-/inf-valued (so @native, not viper); plain ints pass through
unconverted. Assumes low <= high.

Args:
    low - the lower bound (may be -math.inf for an open lower side).
    value - the value to clamp.
    high - the upper bound (may be math.inf for an open upper side).

Returns:
    `value` clamped to [low, high].

### `between_opt(low: float, value: float, high: float) -> float`

### `magnitude_sq_upy(x: float, y: float, z: float) -> float`

|(x, y, z)|^2 (no sqrt -- callers compare against squared thresholds). Pure float -> @native.

### `magnitude_sq_opt(x: float, y: float, z: float) -> float`

### `bank_demand_upy(heading_error: int, gain: float, limit: float) -> float`

Bank-to-turn: the roll angle to hold for a heading error, proportional with a symmetric hard clamp.

A banked turn is tight (~v^2/(g*tan(bank))) where a flat rudder skid is wide and weak, so the glider
does not over-RANGE a small zone and the overshoot loop becomes an altitude-bleeding orbit.

Args:
    heading_error - the heading error to null (deg).
    gain - roll degrees commanded per degree of error (0 -> no bank, rudder-only).
    limit - the symmetric hard clamp on the returned roll (deg).

Returns:
    The roll angle (deg, right +), clamped to [-limit, limit].

### `bank_demand_opt(heading_error: int, gain: float, limit: float) -> float`

### `fin_deflection_limit(speed_ms: float) -> int`

Max fin deflection in degrees for a given airspeed -- the dynamic-pressure governor table lookup.

Saturates at _FIN_VMAX. Multiply by the config fins.limit_multiplier at the caller (the safety dial
is not baked into the table).

Args:
    speed_ms - the airspeed (m/s).

Returns:
    The max fin deflection (deg from neutral) for that airspeed.

### `atomic_write_json(path: str, data) -> None`

Persist `data` as JSON to `path` atomically (shared by config.save + mission.save).

Write a temp file then rename it over the target, with a remove-then-rename fallback for a VFS
(FAT) that won't rename onto an existing file.

Args:
    path - the destination file path.
    data - the JSON-serialisable object to persist.

Returns:
    None. Writes `path` atomically as a side effect (via a `.tmp` sibling then rename).

### `id_classify(read, expected: int) -> str`

Classify a chip WHO_AM_I / device-id byte into an operator-readable wire-level diagnosis.

The deeper 'why' a bus driver's diagnose() returns when setup() failed, so `verify`/`probe` report
e.g. 'chip-select not asserting' instead of just 'absent / miswired?'. Shared by every ID-based
driver (adxl375 / lsm6dso32 / bno055 / bmp280), so it lives here, not in one driver.

Args:
    read - the id byte read from the device, or None when the bus read itself failed (no I2C ack /
        SPI error).
    expected - the device's documented id byte.

Returns:
    A human-readable diagnosis string: 'ok' when read == expected, else the most likely wiring/
    power fault inferred from read (None / 0x00 / 0xFF / a wrong non-zero id).

### `apogee_step(elevation, now_ms: int, peak, since_ms, drop_m, dwell_ms: int) -> tuple`

One step of APOGEE detection: track the baro peak, report when it has fallen off it.

Pure and stateless -- the caller owns `peak` and `since_ms` -- because this logic exists in two
worlds and used to be written twice. `tasks/sequencer.py` runs it on the board; the host sim in
`tools/virtual_flight.py` ran a hand-maintained MIRROR whose own comment records that it "had
DRIFTED to timeout-only deploy, and a low arc could be back underground by the timeout". Stage
timing sets the separation altitude, so a drifted mirror moves the host's apogee away from the
board's -- which is exactly the divergence measured between them. One implementation, no mirror.

The caller keeps the arming window (the motor's pressure wave corrupts the in-airframe baro during
burn) and the burnout-timeout fallback: those need the caller's own clock and stage entry.

Args:
    elevation - the current baro height above the pad, or None while not armed / no reading.
    now_ms - the caller's monotonic clock in milliseconds.
    peak - the highest elevation seen so far, or None before the first reading.
    since_ms - when the fall below the peak began, or None if not currently below it.
    drop_m - how far below the peak counts as descending (same units as elevation).
    dwell_ms - how long that fall must be sustained before it is apogee rather than a dip.

Returns:
    (peak, since_ms, fired) -- the updated state, and True exactly when apogee is confirmed.

### `class Waiter`

An IRQ-kicked wake with a sliced fallback -- the cheap replacement for asyncio.wait_for_ms.

MEASURED on the board (test/diag_alloc_hotloop.py): `asyncio.wait_for_ms(ThreadSafeFlag.wait(), t)`
allocates **560 B per call**, against 96 B for a bare flag wait and **48 B for one
asyncio.sleep_ms**. On the ADXL375, whose IRQ fires at its 100 Hz ODR, that wrapper alone was
~56 KB/s. It is not the waiting that costs, it is asking to be woken with a deadline attached.
Converting the three interrupt-driven drivers to this took the board's leak from **331 KB/s
(OOM ~96 s) to 191 KB/s (OOM ~167 s)**.

So: sleep in slices and check a counter between them. At a 100 Hz ODR the kick has already landed
when the first slice ends, so the normal path costs ONE sleep_ms -- 48 B instead of 560. With a
dead interrupt the loop runs out its slices and the caller samples anyway, which is the same
fallback it always had. The price is up to `slice_ms` of wake latency, nothing for a sensor whose
data changes every 10 ms.

A COUNTER rather than a bool, because it costs the same and holds more: a count above one means
interrupts arrived faster than they were consumed, so a sampling overrun is recoverable evidence
rather than a silently dropped sample. No separate miss counter -- that is the same fact stored
twice.

THE COUNTER IS INTERNAL. Callers use kick(), take() and wait(); nothing outside reads `kicks`,
because a driver that tests it directly has to remember to clear it, and one that forgets goes on
sampling forever on a stale edge. take() is the only non-blocking read, and it clears as it
reports -- which is exactly the form the polling fallbacks need, and what a ThreadSafeFlag cannot
give them (it offers only a blocking wait(), which is why those drivers used to carry a second
mark beside it). If an overrun count is ever wanted outside, add a method for it rather than
reaching in.

- `__init__(slice_ms: int=10)` — constructor
- `kick(_unused_pin=None) -> None` — ISR entry: one small-int increment. No branch, no allocation, nothing to get wrong.
- `take() -> int` — Non-blocking: HOW MANY kicks arrived since the last check. Clears the count.
- `wait(timeout_ms: int) -> bool` — Sleep in slices until a kick lands or `timeout_ms` elapses.

## `config.py`

_Tested by `test/test_config.py`._

Board configuration loader / validator -- the foundational config layer the rest of the firmware
builds its tasks from.

Implements the three-layer model from doc/specs/board-config.md:
  config_default.py -- firmware default / fallback
  board.config      -- saved active config, a full snapshot
  in-memory dict    -- validated, what the Controller builds tasks from

Runs on MicroPython on the board. Validation here is config-file *integrity* (structure, types, pin
uniqueness, bus refs, reserved pins) -- NOT hardware health, which is checked at runtime and surfaced
to the operator (the strict model).

### `validate(cfg) -> list`

Validate a config for FILE INTEGRITY, threading the accumulators through the section helpers.

Config-file integrity only -- structure, types, pin uniqueness, bus refs, reserved pins -- NOT
hardware health, which is checked at runtime and surfaced to the operator (the strict model).

Args:
    cfg - the config object to validate.

Returns:
    A list of human-readable error strings; the empty list means valid. A non-dict `cfg` returns
    a single 'config is not an object' error.

### `config_id(cfg) -> str`

A stable short hash identifying a config snapshot (for the CC iam / config_id).

Args:
    cfg - the config object to hash.

Returns:
    A 12-hex-char id: the SHA-256 prefix when hashlib is available, else an 8-hex FNV-1a fallback.

### `load(path: str='board.config', defaults=None) -> tuple`

Layered load: the active board.config if present and valid, else the defaults.

Never raises -- a missing / corrupt / invalid active file degrades to the defaults so the board
is always reachable.

Args:
    path - the active config file path (default 'board.config').
    defaults - the fallback config; None builds the firmware default via config_default.

Returns:
    (cfg, source, errors). `source` is 'active' (the file was loaded), 'default' (no file), or a
    'default(fallback: ...)' reason (the file was bad JSON or failed validation); `errors` is the
    validation error list for whatever config was chosen.

### `schema_version(cfg) -> str`

The config-schema version a config was produced from ('' when it predates versioning).

Args:
    cfg - a config object (or None).

Returns:
    The 'version' string, or '' when absent.

### `outdated(cfg, defaults=None)`

Compare a config's schema version against the firmware's; the pair when they differ, else None.

A mismatch does NOT change what runs (see load) -- it means the saved file was produced from a
different config tree, so it may lack devices or defaults this firmware knows about. The operator
re-saves to adopt them.

Args:
    cfg - the config to check (typically the loaded/active one).
    defaults - the firmware default to compare against; None builds it.

Returns:
    (saved_version, firmware_version) when they differ, else None.

### `save(cfg, path: str='board.config') -> str`

Validate then atomically persist a full config snapshot.

Args:
    cfg - the full config snapshot to persist.
    path - the destination path (default 'board.config').

Returns:
    The config_id of the persisted snapshot.

Raises:
    ValueError - if `cfg` is invalid (an invalid config is never written).

### `reset(path: str='board.config') -> bool`

Delete the active config so the next load falls back to the defaults.

Args:
    path - the active config file path (default 'board.config').

Returns:
    True if the file was removed, False if it did not exist.

### `bus(cfg, kind, ident) -> dict`

Resolve a bus addressed by kind + id to its spec dict.

Ids are JSON object keys (always strings), so the int id from a component is normalised here --
callers pass `device['bus'], device['id']` and never parse a 'type:id' string.

Args:
    cfg - the config to look in.
    kind - the bus kind ('uart' / 'i2c' / 'spi').
    ident - the bus id (int or string; coerced to string for the lookup).

Returns:
    The bus spec dict, or None if no such bus is defined.

### `device(cfg, name=None, driver=None) -> dict`

Find a sensor / component by name and/or implementation.

`driver` matches the resolved implementation -- a device's `driver` (drivers/) or `activity`
(tasks/) field. Both filters are optional; with neither, the first device is returned.

Args:
    cfg - the config to search.
    name - the device name to match, or None to match any name.
    driver - the implementation to match, or None to match any.

Returns:
    The first matching device dict (sensors searched before components), or None.

## `config_default.py`

Baked-in default board configuration for the WaveShare ESP32-P4-WIFI6 controller.

Human-edited firmware default and the safe fallback when no valid board.config exists (see
doc/specs/board-config.md). Pins come from doc/waveshare_esp32p4_pins.md (validated on hardware by
test/test_pins.py). `default()` returns a FRESH dict each call so callers may mutate it freely.

Topology: buses are grouped by type then id; a sensor/component addresses one by `bus` (the kind,
e.g. 'i2c') + `id` (its int id), so nothing parses a 'type:id' string. `sensors` are data providers
fused by quantity + priority (several may provide the same quantity with different drivers /
priorities); `components` are the consumers / actuators (recorder, ...).

### `default() -> dict`

## `config_hitl.py`

A HITL board config derived from config_default(). The real sensor drivers are turned OFF and the
`hitl` task supplies accel / attitude / agl / altitude / elevation / position at priority 0, so the
control code reads the simulation. flight is enabled with test gains, the watchdog and the radios are
off (self-contained sim), and separation is off (the boost-timeout drives BOOSTING -> GLIDING).
Servos stay on so the sim can read the commanded fin angles. `default()` returns a fresh dict --
mutate freely. Run it instead of config_default for a simulation; the flight config is untouched.

### `default(motor: str='F15', noise: float=0.0, spike: bool=False, wind: float=0.0, wind_dir: float=0.0, boost_axis: str='z', glider_g: int=_GLIDER_G, inject_hz: int=0, gnss_drift: float=0.0, gnss_drift_dir: float=0.0, pad_dwell_s: float=0.0) -> dict`

Build a HITL config from config_default(), the real sensors off and the `hitl` sim task added.

Separation is off here, so the boost->glide deploy rides the sequencer's baro APOGEE detect
(mass / motor-independent -- the top of the arc), with config_default's long boost_timeout as the
last-resort fallback; the sim's reduced baro noise keeps the peak-detect clean. The booster adds to
the glide mass for the boost phase then ejects at separation, so the glide runs on `glider_g` alone
-- a lighter glider glides LONGER, the worst case for the GC-off leak.

Args:
    motor - the booster motor ('E16' / 'F15'); its mass sets the boost-phase liftoff mass.
    noise - sensor-noise level fed to the sim (0.0 = clean).
    spike - inject accel spikes when True (a robustness stressor).
    wind - steady cross-wind speed (m/s) the glide must crab against.
    wind_dir - the wind's toward-bearing (deg).
    boost_axis - which accel axis ('x' / 'y' / 'z') carries the boost |a|.
    glider_g - the glider (glide) mass in grams (default _GLIDER_G, the full build; pass the
        light-build mass for the half-weight optimisation target).
    inject_hz - the sensor publish rate; 0 -> the sim's sim_hz. Lower it (e.g. 10) to slim the
        sim's own heap churn so an on-board HITL leak reflects real flight.
    gnss_drift - steady GNSS ground-velocity drift (m/s) the pad-drift calibration must measure out.
    gnss_drift_dir - the drift's bearing (deg).
    pad_dwell_s - seconds held stationary on the pad before launch (lets the drift calibration
        gather samples).

Returns:
    A fresh HITL config dict (the real sensors disabled, flight + the `hitl` task enabled).

## `controller.py`

_Tested by `test/test_controller.py`._

Flight Controller -- creates and supervises the tasks described by a validated config, and tracks the
flight stage machine. See doc/specs/coludo.md ('Flight Controller', 'Tasks').

The Controller is the one task created explicitly; it creates the rest from config in a deterministic
order. Task failures are reported, not fatal (the strict/operator-authority model): a component that
fails setup is logged and skipped, and go/no-go stays with the operator via stats()/validate().

### `class Stage`

The flight stages: int ids and their operator-facing names, self-contained.

Int ids (cheap to compare/store on MicroPython) plus the `STAGES` id->name mapping (operator-facing
names; `in Stage.STAGES` is an O(1) key check). `NAMES` is the reverse (name->id) so config that
names stages by string resolves to an id once.

NULL = 0 is a SENTINEL, not a flight stage: it is what an unwritten NVS checkpoint reads back, so
a warm-start knows "no checkpoint saved". The live stages start at SETTING = 1, so any non-zero
saved id IS a real stage to recover into. NULL is deliberately kept OUT of STAGES/NAMES -- it is
never set_stage()-able and never has an operator name.

Kept here, in the stage machine's own module. flight/sequencer/hitl/led import it from controller
-- a LIGHT coupling (the module loads fast, no heavy deps pulled just for the enum). It could move
to commons.py as the shared domain enum to drop even that import, but the gain is marginal versus
the cross-file churn; revisit only if importing controller solely for Stage ever bites.

- `active(stage: int) -> bool` _(staticmethod)_ — True during the powered boost (BOOSTING only) -- the one active-thrust stage.
- `passive(stage: int) -> bool` _(staticmethod)_ — True while gliding unpowered, post-separation (GLIDING..LANDING) -- the boost stack is gone.
- `airborne(stage: int) -> bool` _(staticmethod)_ — True while off the ground -- the union of the active boost and the passive glide.

### `class Controller(inspector.Inspectable)`

- `__init__(config: dict, registry: dict=None, log=None)` — constructor
- `directory() -> list` — Names of enabled devices, in creation order (config order).
- `create(name: str) -> task.Task` — Create a task by component name via the registry.
- `active(name: str=None)` — The active task by name, or all active tasks.
- `find(names: list[str]) -> list` — The active tasks for `names`, without blocking.
- `query(names: list[str], waiting: bool=True) -> list` — Look up sibling tasks by name from the registry, optionally waiting until they are all up.
- `setup() -> bool` — Create + set up every enabled task in order, skipping (and reporting) failures.
- `bustune(kind: str, ident, freq: int) -> dict` — Retune a sensor bus in place and report which of its devices stay healthy.
- `start() -> None` — Launch each task's run() loop as a supervised asyncio task.
- `close(name: str) -> None` — Deactivate a task and clean up its resources.
- `finish() -> None` — Shut down all tasks, in REVERSE bring-up order.
- `set_stage(stage: int) -> None`
- `stage_name() -> str` — The current flight stage as its operator-facing name.
- `arm() -> None` — Enable actuation.
- `disarm() -> None`
- `hold(stage_name: str) -> bool` — Operator stage override (ground test): force a stage and pause auto-sequencing.
- `resume() -> None` — Clear the operator hold -> the sequencer drives the stages again.
- `validate() -> bool` — True if every active task is healthy.
- `inspect() -> dict`
- `stats() -> dict`

## `databoard.py`

_Tested by `test/test_databoard.py`._

The shared latest-value store + sensor fusion for hot data (doc/specs/coludo.md "Task Data-Flow and
Message Propagation"). Replaces a two-layer raw/fused store + a polling fusion task with a registry
of Parameter objects whose fused value is computed on read.

Structure.
  Databoard   -- a registry of Parameter objects. Databoard.parameter(name) gets-or-creates one;
                 a sensor registers itself as a source via provide() (which returns its channel
                 handles) and then reports by pushing each channel directly -- the hot write path
                 is one step, no lookup. value()/read() resolve the winner + primary in one pass.
  Parameter   -- one fused quantity (e.g. 'altitude') for the consumer. Holds a short LIST of
                 channels KEPT IN RANK ORDER (lowest = primary first; a list, not a dict, is faster
                 at this size), plus the shared freshness window derived from its primary tier.
  _Channel    -- one source's stream: a static rank (priority; lower = preferred) and TWO slots
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

One fused quantity.

Holds a rank-ordered channel per source; value() fuses by rank and freshness, falling back to
extrapolation of the primary when none is fresh.

- `__init__(name: str)` — constructor
- `add_source(source: str, rank: int, expire_us: int, reconcile: bool=False) -> _Channel` — Register (or re-register) a source at `rank`; return its channel to push() to directly.
- `write(value, source: str) -> None` — Report a source's latest reading by name (a convenience; sensors push() their channel).
- `value()` — The fused estimate (offset-reconciled when enabled); None if nothing was ever written.
- `stamp()` — ticks_us of the primary source's latest push, or None if nothing was ever written.
- `read() -> list` — [value, source, age_ms] of the fused estimate.
- `offsets() -> dict` — Learned bias per source (source -> offset) for diagnostics; empty until reconciled.
- `raw(source: str)` — A specific source's latest value (None if absent / unwritten).
- `sources() -> list`

### `class Databoard`

The global registry of Parameter objects; Inspectable as `databoard` (fused value per name).

- `parameter(*names)` _(classmethod)_ — Get-or-create read handle(s) for `names` -- the dependency accessor.
- `provide(source: str, provides: dict, *want)` _(classmethod)_ — Register `source` for the params it provides and hand back its write-channel(s).
- `write(name: str, value, source: str) -> None` _(classmethod)_
- `value(name: str)` _(classmethod)_
- `read(name: str) -> tuple` _(classmethod)_
- `raw(name: str, source: str)` _(classmethod)_
- `inspect() -> dict` _(classmethod)_
- `stats() -> dict` _(classmethod)_

## `fixed.py`

_Tested by `test/test_fixed.py`._

Fixed-point helpers for the flight hot paths. MicroPython boxes a heap float on EVERY float operation,
and GC is disabled through the airborne phase, so every boxed float leaks toward OOM. The control path
therefore works in scaled integers ('fixnum') and crosses to/from float only at the isolated sensor
boundary.

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

Whole unit (float or int: degrees / metres / m·s⁻¹) -> fixnum.

The one boxed-float spot, kept at the sensor boundary. Truncates toward zero -- the residual is
< 1/SCALE (below actuator resolution).

Args:
    value - the whole-unit quantity (degrees / metres / m·s⁻¹), float or int.

Returns:
    The value as a fixnum (the whole unit scaled by SCALE).

### `to_float(scaled: fixnum) -> float`

fixnum -> whole-unit float.

Boxes a float, so use ONLY where a float is genuinely required (trig, the airspeed integrator) and
keep it at the boundary -- never inside a hot loop.

Args:
    scaled - the fixnum to convert.

Returns:
    The whole-unit float value (scaled / SCALE).

### `to_millis(value: fixnum) -> int`

A fixnum (×SCALE) -> integer MILLI-units (×1000), independent of SCALE.

For telemetry/logs that fix a milli representation regardless of the control SCALE -- e.g. at
SCALE=100 a centidegree fixnum becomes millidegrees. Pure integer rescale (SCALE divides 1000), so
no float is boxed.

Args:
    value - the fixnum to rescale.

Returns:
    The value in integer milli-units (×1000).

### `to_str(scaled: fixnum) -> str`

fixnum -> its decimal string ('12.34' at SCALE 100) via INTEGER divmod -- NO float is boxed.

For telemetry / display: a scaled value prints as its true decimal without a float round-trip.

Args:
    scaled - the fixnum to format.

Returns:
    The decimal string (e.g. '12.34' at SCALE 100), sign preserved.

### `clamp(low: fixnum, value: fixnum, high: fixnum) -> fixnum`

Integer clamp to [low, high] (a symmetric ±x clamp with low=-x, high=+x).

Routes to commons.clamp_int -- the `@micropython.viper` integer clamp (~2.1-2.8x the float
`between`). Safe here because fixnum is always a finite int (no math.inf), which is exactly what the
fixed-point transition buys: the whole control-path clamp is now viper-native, not the inf-tolerant
@native float path.

Args:
    low - the lower bound (fixnum).
    value - the value to clamp (fixnum).
    high - the upper bound (fixnum).

Returns:
    value clamped to [low, high] (fixnum).

### `atan2_cd(y: int, x: int) -> fixnum`

atan2(y, x) as a CENTIDEGREE fixnum, four-quadrant, via integer CORDIC -- NO float boxed.

y and x are a RATIO-FREE integer direction vector: only their ratio sets the angle, and their
MAGNITUDE only trades precision (the CORDIC's right-shifts discard low bits, so bigger inputs keep
more). Fed the control's centi-fixnum scale (accel g via from_float, ~x100) the error is ~0.5 deg
typical / 1.8 deg worst over the glide envelope -- fine for the attitude backup; x1000 would tighten
to ~0.16 deg if a caller ever needs it. CORDIC needs x >= 0, so x < 0 reflects into the right
half-plane and the 180 deg is added back per quadrant.

The old magnitude floor is GONE -- the core pre-normalises (see the _ATAN_CD note), so the error
is a flat ~0.05 deg at every magnitude instead of exploding as the vector shrinks. Measured on
the board (test/diag_fixed_nav.py), worst error by magnitude: 1 -> 0.02 deg, 3 -> 0.025,
100 -> 0.048, 1000 -> 0.040. atan2_cd(1, 1) is now exactly 4500; the axes land within 0.02 deg.

What a caller still cannot get back is precision its INPUTS already threw away: two components
quantised to a coarse integer no longer carry the true ratio (a 10 cm vector at centimetre scale
holds ~10 % per component, so ~4 deg -- but only ~7 mm of position). Feed it the largest-magnitude
form of the vector available, and prefer raw sensor counts over pre-scaled values.

Args:
    y - the direction vector's y component (integer, any magnitude).
    x - the direction vector's x component (integer, any magnitude).

Returns:
    The angle in centidegrees, range (-18000, 18000]; 0 for the undefined (0, 0) input.

### `blend_cd(state: fixnum, delta: fixnum, target: fixnum, shift: int, correct: bool) -> fixnum`

One complementary-filter step in centidegrees (viper).

`state + delta` (gyro integration), then optionally a `1/2^shift` pull toward `target` (the accel
angle). Pure integer -> zero float boxed; the attitude backup runs it per axis each control step
(tasks/attitude.py).

Args:
    state - the current filter state (centidegree fixnum).
    delta - the gyro-integration increment (centidegree fixnum).
    target - the accel angle to pull toward (centidegree fixnum).
    shift - the pull is 1/2^shift of the way toward target.
    correct - True to apply the pull toward target; False to only advance.

Returns:
    The updated state (centidegree fixnum).

### `isqrt_upy(n: int) -> int`

Integer floor(sqrt(n)) reference, division-free (bit-by-bit).

Args:
    n - the value to root; must be < 2**31.

Returns:
    floor(sqrt(n)); 0 for n <= 0.

### `isqrt_opt(n: int) -> int`

## `gnss.py`

_Tested by `test/test_gnss.py`._

Shared GNSS infrastructure (sibling of i2cbus/spibus/servo). NMEA helpers + a Gnss base Task: read
NMEA over a dedicated UART, parse RMC -> 'position' (lat, lon) and GGA -> 'altitude' (m MSL) +
'elevation' (m above the GNSS ground zero, a barometer backup). Module-specific sentence selection +
rate is the subclass's _configure(); ATGM336H (CASIC/PCAS) and NEO-6M (u-blox) differ only there.
Talker-agnostic (GP/GN/BD). Best-effort -- lock drops under boost, so the channels go stale and
consumers fall back.

### `checksum_ok(sentence: str) -> bool`

Verify the NMEA `*hh` XOR checksum (over the chars between '$' and '*').

The inner XOR loop is _xor_checksum.

Args:
    sentence - the full NMEA sentence including the '$' and the '*hh' suffix.

Returns:
    True when the computed checksum matches the sentence's; False on a missing '*' or a bad /
    absent hex suffix.

### `degrees(value: str, hemisphere: str)`

Convert an NMEA ddmm.mmmm value + hemisphere to signed decimal degrees.

Args:
    value - the ddmm.mmmm field (empty -> None).
    hemisphere - 'N'/'S'/'E'/'W' (S and W give a negative result).

Returns:
    The signed decimal degrees, or None when the field is empty.

### `nmea(body: str) -> bytes`

Wrap a command body in `$...*hh\r\n` with its XOR checksum.

For building PCAS/PMTK/PUBX config sentences.

Args:
    body - the sentence body between '$' and '*' (no delimiters).

Returns:
    The full sentence as bytes, ready to write to the UART.

### `class Gnss(task.Task)`

Base GNSS driver over a dedicated UART.

RMC -> 'position' (lat, lon); GGA -> 'altitude' (m MSL) + 'elevation' (m above the GNSS ground
zero, a baro backup). Subclasses set the module-specific sentence selection + rate in
_configure().

- `setup() -> bool`
- `run() -> None` — Read NMEA lines forever and parse them.
- `probe() -> str` — On-demand self-test: NMEA is arriving on the UART.
- `diagnose() -> str` — Deeper analysis when setup() failed: is NMEA arriving on the UART?
- `inspect() -> dict`

## `governor.py`

_Tested by `test/test_governor.py`._

The dynamic-pressure fin governor (doc/specs/coludo.md "Fin authority"), sibling of pid.py / mixer.py /
airspeed.py. Owns the airspeed ESTIMATE (airspeed.AirspeedEstimator: the PITOT direct source when
in-band, else the accel backbone + GNSS corrector), the ADAPTIVE THROTTLE that keeps that float path
off the GC-off hot loop once the glide settles, and the mixer authority cap (commons.fin_deflection_limit
∝ 1/v², × the board's fin_limit_multiplier safety dial). Extracted from tasks/flight.py so the throttle
policy is unit-testable without a Flight task.

Host-runnable by construction (tools/virtual_flight.py drives the REAL governor): the sensor
dependencies are INJECTED databoard-style handles -- `accel.value()` -> (x, y, z) in g or None,
`gnss_speed.read()` -> (m/s, source, age_ms), `pitot.read()` -> (airspeed m/s, source, age_ms) from the
SDP810 (the DIRECT source, sdp810.py did the sqrt) -- never the databoard itself, and nothing here
touches time or the machine.

Why the estimator is throttled at all: the update is a FLOAT path (sqrt magnitude, integrate, GNSS
blend) ~ the biggest GC-off allocator measured (~22 KB/s at 100 Hz). It runs FULL RATE where the
estimate cannot be trusted to pace itself (pre-glide boost/decel, a fresh dive); everywhere else the
DISTANCE-CONSTANT law paces it: update at clamp(speed, floor, ceiling) Hz = one update per ~1 m of
TRAVEL. Probed 1..60 m/s against the previous error-adaptive law (7/04):
  * consistency -- exactly 1.00 m/check across the whole 5..50 m/s envelope (old: 0.04..1.90 m with a
    9.5x discontinuity at its 20 m/s full-rate trigger);
  * safety -- the old law's WORST staleness (1.9 m at 19 m/s) sat right below its own trigger; here
    staleness self-scales, an overspeed shrinks its own next interval, so the absolute-speed trigger
    is gone entirely;
  * leak -- same class at glide trim (3.1 vs 2.2..5.6 KB/s), 3.3x LESS in a 30 m/s dive (6.7 vs 22.4
    KB/s: no more 100 Hz above 20 m/s for granularity nothing needs);
  * simplicity -- 4 knobs -> 2 (floor/ceiling Hz) and the adaptation state machine becomes the same
    precomputed integer-indexed table as commons.fin_deflection_limit.
The estimator integrates the ACCUMULATED dt, so cadence never changes the integral -- only how fresh
the fin-authority cap is (the cap persists between updates).

### `class GovernorConfig`

The governor's knobs, resolved from the flight task's config dict ONCE.

Typed config: one place for defaults + doc-in-code; the keys keep their board.config names.

- `__init__(config: dict)` — constructor
- `update_interval(speed: float) -> float` — The estimator update interval (s) for a given speed -- the distance-constant table lookup.

### `class Governor`

Cap the mixer's control authority by estimated airspeed (torque ∝ v²).

step() each control slice decides full-rate vs throttled, updates the estimator over the
accumulated dt, and writes the deflection cap into mixer.limit.

- `__init__(config: GovernorConfig, mixer, accel, gnss_speed, pitot, fin_limit_multiplier: float=1.0)` — constructor
- `airspeed() -> float` — The current airspeed estimate (m/s) -- the boost rod gate and telemetry read it here.
- `cap() -> int` — The dynamic-pressure fin-authority cap (deg) the governor last set on the mixer -- the
- `seed_airspeed(airspeed: float) -> None` — Restore the airspeed persisted by the warm-start crumb, and cap off it AT ONCE.
- `step(dt: float, full_rate_override: bool, pitch: fixnum) -> None` — One control slice: accumulate `dt` and update the estimator + fin cap when due.

## `guidance.py`

_Tested by `test/test_guidance.py`._

The stage-dependent guidance law, sibling of pid.py / mixer.py / navigation.py. Turns (stage,
heading) into the attitude setpoints + heading error the PIDs chase: the boost rod-vertical hold,
bank-to-turn toward the landing zone, the three GPS-degrading heading tiers, and the
low-final-approach centreline tracker. Extracted from tasks/flight.py so the control law is
unit-testable without a Flight task; per-stage laws dispatch through a table (the proven
sequencer._detect pattern), so a new stage is one entry + one method, not a branch in a growing
if/elif.

Host-runnable by construction (tools/virtual_flight.py drives the REAL law): dependencies are
INJECTED -- the mission (zone/launch_point), the governor (airspeed for the boost rod gate), and
databoard-style handles (`position.read()` -> ((lat, lon), source, age_ms), `agl.value()` -> m or
None). Timing comes in as `now_us` from the caller; only commons.ticks_diff touches ticks.

Results land in the roll_setpoint/pitch_setpoint (centidegree fixnum) + heading_error (int degrees)
INSTANCE SLOTS rather than a returned tuple -- decomposed WITHOUT adding a per-step heap allocation
(GC is off in flight).

### `class Heading`

The endgame HOLDING pattern, self-contained like controller.Stage.

Int ids (cheap to compare/store on MicroPython) + the `PATTERNS` id->name mapping and `NAMES`
reverse. Config names the pattern by string; resolve() turns 'auto'/'o'/'ov'/'oo'/'o-o'
(case-insensitive; 'o-o' aliases 'oo') into an id once. AUTO defers to the Mission
(mission.endgame_heading), which picks by the zone's long/short aspect k: k < OVAL_ASPECT -> 'o'
(single circle), OVAL_ASPECT <= k < OO_ASPECT -> 'ov' (centreline oval), k >= OO_ASPECT -> 'oo'.

- `resolve(name) -> int` _(classmethod)_ — A config string -> the pattern id.

### `heading_error(target: float, current: float) -> int`

Shortest signed heading error (deg), wrapped to [-180, 180] so 350 -> 10 is +20, not -340.

Integer degrees -- sub-degree precision is irrelevant to a servo and lets one modulo replace the
wrap loop (commons.wrap180, viper bundle).

Args:
    target - the desired heading (degrees).
    current - the current heading (degrees).

Returns:
    The signed error target - current, wrapped to [-180, 180] (integer degrees).

### `class GuidanceConfig`

The guidance knobs, resolved from the flight task's config dict ONCE.

Typed config: one place for defaults + doc-in-code; the keys keep their board.config names.
`position_window_ms` is the caller's default tier-1 freshness gate -- the GNSS channels' own
databoard windows -- so it tracks the GNSS rate instead of a magic number; config sets
position_age_max_ms TIGHTER to distrust GNSS sooner (looser is a no-op: the source is already None
past the window).

- `__init__(config: dict, position_window_ms: int)` — constructor

### `class Guidance`

The per-stage control law.

setpoint(stage) gates control stages; enter() captures the holds on entering control; compute()
dispatches the stage's law and fills the setpoint slots.

- `__init__(config: GuidanceConfig, mission, governor, position, agl, elevation=None)` — constructor
- `setpoint(stage: int)` — The configured attitude setpoint dict for a stage, or None when it is not a CONTROL stage.
- `reachability(glide_ratio: float, wind_e: float=0.0, wind_n: float=0.0, airspeed: float=0.0)` — Can the glider still glide to the zone from here?
- `min_turn_radius(bank_deg: float) -> float` — The tightest coordinated turn the airframe can HOLD at `bank_deg` and its LIVE airspeed.
- `endgame_bank() -> float` — The steepest bank the ENDGAME spiral may hold at the LIVE airspeed, stall-margin bounded.
- `landing_turn_radius()` — The endgame turn-radius floor at the bank the endgame will ACTUALLY hold -- the precision bound
- `enter(heading: float, roll: fixnum, pitch: fixnum) -> None` — Entering a control stage (from a non-control one): capture the holds and reset the nav cache.
- `compute(stage: int, setpoint: dict, heading: float, now_us: int) -> bool` — Run a stage's law: fill the setpoint slots and report whether the fins may actuate.

## `i2cbus.py`

_Tested by `test/test_i2cbus.py`._

Shared, lock-serialized I2C buses. Several sensor drivers sit on one physical bus (i2c:0 carries the
ADXL375, BNO055 and BMP280), so they must not interleave transactions on the single peripheral: each
bus id has ONE machine.I2C plus an asyncio.Lock, and get() hands back the shared wrapper. The
read/write methods are async (they acquire the lock) but the underlying I2C op is fast and
synchronous, so the lock is held only for the transaction. A glider-only module.

### `class Bus`

One physical I2C bus, shared by every device on it.

NO PER-OPERATION LOCK, deliberately. Every operation below is ONE synchronous machine.I2C call
with no `await` inside it, and MicroPython's asyncio is cooperative and single-threaded: a section
that never yields cannot be interrupted, so the event loop already serialises them. The lock that
used to wrap each call therefore protected nothing -- and it was not free: MEASURED on the board,
`async with self._lock` cost **288 B of the 320 B** an `await bus.read_into()` allocated, against
0 B for the underlying `readfrom_mem_into`. With eight drivers sampling at 10-50 Hz that was the
single largest GC-off allocator on the board, far above telemetry.

It also never gave what a lock is usually for: it was released BETWEEN calls, so a multi-step
sequence (icp10111's measure-command, sleep, read) already interleaves with other drivers today
and works, because an I2C device holds its own state per address. Anything that genuinely needs a
sequence to be atomic across awaits should say so explicitly with `async with bus.transaction():`
-- which is honest about the cost, rather than paying it on every single-shot read.

- `__init__(bus_id: int, spec: dict)` — constructor
- `transaction()` — Hold the bus across a MULTI-STEP sequence that must not interleave (the explicit escape hatch).
- `retune(freq: int) -> None` — Re-init this I2C peripheral at `freq` Hz in place (bench frequency calibration; no reboot).
- `read(addr: int, reg: int, count: int, addrsize: int=8) -> bytes`
- `read_chip_id(addr: int, reg: int, addrsize: int=8) -> int` — Read a device's one-byte identity register (WHO_AM_I / CHIP_ID).
- `read_into(addr: int, reg: int, buf, addrsize: int=8) -> None`
- `write(addr: int, reg: int, data: bytes, addrsize: int=8) -> None`
- `writeto(addr: int, data: bytes) -> None` — Raw write (no register) -- for command-based devices like the ICP-10111.
- `readfrom(addr: int, count: int) -> bytes` — Raw read (no register) -- pairs with writeto() for command-based devices.
- `device(addr: int) -> _Device` — A register window for one address on this bus (matches spibus.Bus.device).
- `scan() -> list`

### `get(bus_id: int, spec: dict) -> Bus`

The shared Bus for `bus_id`, created once from `spec` (scl/sda/freq) and cached thereafter.

### `bind(board: dict, device: dict, default_addr: int) -> tuple`

Resolve a device's config block to the (bus, address) pair it talks over.

Six drivers opened setup() with the identical four lines -- read `id`, resolve the spec through
config.bus(), fetch the shared bus, pull `addr`. That preamble is BUS knowledge (which config keys
name a bus, what they default to), not driver knowledge, so it belongs here rather than on the
generic Task base: a task-level helper would need an i2c and an spi variant, and would drag both
bus modules into the import graph of every task that has no bus at all.

Returns the pair rather than a _Device window because these drivers pass the address per transfer
(self._bus.read(self._addr, ...)) -- they share the bus wrapper's recovery and claim paths.

Args:
    board - the whole board config (holds the `buses` section).
    device - the component's own config block ('bus', 'id', 'addr').
    default_addr - the datasheet address when the block does not name one.

Returns:
    (bus, addr), or (None, None) when the config declares no such bus -- the caller's setup()
    returns False on that and the Controller reports the device as not connected.

## `inspector.py`

_Tested by `test/test_inspector.py`._

Inspector -- the registry of Inspectable objects and the operator-facing introspection surface.
Control's inspect/update/stats commands resolve an object by name through the Inspector
(doc/specs/cc-protocol.md). Any object an operator should see or tweak registers itself here.

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
- `probe_all() -> dict` _(classmethod)_ — Run probe() on every registered inspectable that implements it.
- `calibration_all() -> dict` _(classmethod)_ — Calibration requirement + state for every registered device that declares one.
- `inspect(name: str) -> dict` _(classmethod)_
- `update(name: str, props: dict) -> list` _(classmethod)_
- `stats(name: str) -> dict` _(classmethod)_

## `main.py`

_Tested by `test/test_main.py`._

Board bring-up, run on boot. Loads the driver/task packages (so every @task.activity / @task.driver
registers), creates the Mission (launch identity), and hands the config to the Controller, which
builds + supervises the *enabled* tasks. Connectivity (Wi-Fi + the CC link) is just two of those
tasks, so a board with no Wi-Fi (e.g. FireBeetle 2) boots and runs everything else without CC --
nothing here is hardcoded. Adding a task is dropping a file in drivers/ or tasks/ and enabling it in
the board config.

Telemetry-first: the task loops (recording included) start immediately and keep running; the Wi-Fi/CC
tasks connect in the background when they can. Time sync + live tweaks arrive from Control over the
link (e.g. `update mission {epoch}` sets the RTC); the board itself never asks.

### `bringup(cfg: dict, log=print) -> controller.Controller`

Register every driver/task, create the Mission, and start the enabled tasks from the config.

Network-free itself -- any Wi-Fi/CC work happens inside the tasks the Controller starts.

Args:
    cfg - the validated board config the Controller builds its tasks from.
    log - line logger for bring-up progress (defaults to print).

Returns:
    The Controller, with each enabled component's task created and its run loop launched.

### `main() -> None`

## `mission.py`

_Tested by `test/test_mission.py`._

Mission -- the per-launch identity the operator sets before a flight: a launch id, the launch site
position (a known origin and a GNSS cold-start seed), and the board clock. Unlike the board config
(hardware; stable across flights, see config.py) the mission changes every launch, so it lives in its
own file, `launch.config`, and is edited live through the Inspector.

Mission is a singleton Inspectable:
  inspect mission -> launch id / site / position + the board clock
  update mission base64:{"launch_id":"t1"} -> set the launch id for this flight
  update mission base64:{"epoch":1750170000} -> set the board RTC (time sync; Unix seconds)
  get-config launch / set-config launch -> read / save (merge + persist) launch.config

Position is metres / decimal degrees; it is a known origin now and seeds the GNSS driver later.

### `class Mission(inspector.Inspectable)`

The operator-set launch identity.

One per board; registers itself so Control can `inspect`/`update mission`. Seeded from
launch.config at construction.

- `__init__(path: str=LAUNCH_PATH, max_range_m: float=_DEFAULT_MAX_RANGE_M)` — constructor
- `set_time(epoch) -> bool` — Set the board RTC from a Unix epoch (seconds, UTC).
- `clock() -> str` — Current board wall-clock as 'YYYY-MM-DDTHH:MM:SS' (from the RTC).
- `epoch() -> int` — Current board clock as a Unix epoch (seconds), for Control to compare against its own.
- `launch_point()` — The launch origin (lat, lon).
- `freeze_launch() -> None` — Pin the live GNSS fix as the persistent launch point.
- `select_site(fix: tuple)` — CC-less site selection (doc/specs/coludo.md "Field operation without CC").
- `fallback_zone(fix: tuple, bearing_deg: float=0.0, near_m: float=50.0, width_m: float=100.0, depth_m: float=90.0) -> tuple` — The spiral-landing fallback: synthesize and ADOPT a GENEROUS box the spiral just lands INSIDE.
- `zone_points() -> tuple` — (target, gate_a, gate_b) for the current landing zone, memoized by zone identity.
- `zone_aspect() -> float` — The zone's long/short side ratio (>= 1), memoized alongside zone_points.
- `endgame_heading() -> int` — The endgame holding pattern (a guidance.Heading id) the AUTO setting resolves to.
- `geometry() -> dict` — The landing zone resolved against the launch point.
- `probe() -> str` — On-demand self-test: a launch position is set and the zone (if any) is in range.
- `inspect() -> dict`
- `update(props: dict) -> list` — Apply the editable mission fields from an update.
- `persisted() -> dict` — The mission as it is stored in launch.config: the editable launch fields only.
- `save() -> None` — Persist the stored mission fields to launch.config (atomic temp+rename).

## `mixer.py`

_Tested by `test/test_mixer.py`._

Control-surface mixer (sibling of servo.py / gnss.py). Maps the control axes (roll, pitch, yaw --
each a deflection command in degrees) to per-fin servo angles for the airframe's mixing: ELEVONS (the
two elerons move together for pitch, differentially for roll) + a RUDDER (the yaw fin). A hard +/-
limit on control deflection. Pure integer math, no hardware -- the flight control task (Phase 3) binds
the resolved fin driver objects once (bind()) and then drives them straight from the mixing loop
(actuate()); the per-driver clamp still guards the physical range. mix() keeps the dict form for
tests/host tools.

Signs are config (`surfaces` gains), set during bench alignment: if a surface deflects the wrong way,
flip its gain sign. MECHANICAL NEUTRAL (each fin's true zero) is NOT here -- it lives in the servo
driver (sg90 `trim`, per fin), so it applies to boot / failsafe / control alike; the mixer commands a
COMMON neutral and the driver offsets each fin to its centre.

### `class Mixer`

Mix (roll, pitch, yaw) deflection commands -> {fin_name: integer angle}.

angle = neutral + clamp(sum(gain * axis), +/- limit).  Each fin's mechanical zero is the servo
driver's `trim`, applied downstream -- the mixer only commands the COMMON neutral.

- `__init__(config: dict=None)` — constructor
- `mix(roll: int=0, pitch: int=0, yaw: int=0) -> dict` — Per-fin integer angle for the given axis deflections (degrees).
- `neutralise() -> dict` — The neutral (zero-deflection) angle per fin -- the safe / control-disabled output (shared dict).
- `bind(fins: dict) -> None` — Fuse the resolved fin driver objects into the surface table.
- `angles() -> dict` — {surface name: the angle currently commanded to its fin}.
- `actuate(roll: int, pitch: int, yaw: int) -> None` — mix() fused with the servo write: clamp and set_angle() each bound fin in one loop.

## `navigation.py`

_Tested by `test/test_navigation.py`._

Landing-zone navigation geometry ('heading-to-home'), sibling of mixer.py/pid.py. The mission's
landing zone is a lat/lon rectangle, top-left (TL) + bottom-right (BR) corners (doc/specs/coludo.md).
The TARGET is the zone centre; the two GATES are the midpoints of the two SHORTER sides, so the
glider enters along the long axis (the documented "vector to the shortest boundary entrance").
steer() picks the nearer gate, heads for it until inside the zone, then for the centre.
Equirectangular (flat-earth) math -- "not exact but about", which is plenty at zone scale (<~1 km).

Perf: steer()/bearing()/distance() use float trig and allocate a few small tuples per call --
measured GC-off (n=4000): distance ~170 B, bearing ~256, steer ~518, approach ~982. The primary
throttle is on the CALLER side -- guidance._target_heading() caches the result at GPS cadence
(~10 Hz), so the rate drops ~10x -- but at that rate the nav path is still ~5 KB/s of the ~15 KB/s
glide leak, so "no measurable gain" (the old note) was wrong. The lever is REDUNDANT float work,
not fixnum (lat/lon need ~1e-5 deg; a SCALE-100 fixnum degree is ~1 km -- far too coarse): a caller
that needs both range AND bearing to a point now calls range_bearing() (one offset(), not two).
navigation stays pure float; the geometry is computed once. (Next lever, if needed: cache the
per-flight-constant zone() geometry.)

SAFETY: the gates are FIXED to the short sides, and steer() will always vector to one (and turn ~180
back through it on an overshoot) with NO knowledge of what lies beyond any side (trees / launch pad /
people). So the operator must ORIENT the zone -- choose the TL/BR corners in launch.config so the two
short-side entrances point at hazard-free approach corridors and the long sides border the hazards.
Aerodynamics (long run-in, lower crosswind) and safety (clear corridors) only align if it is laid out
that way; the firmware cannot verify it. See doc/specs/coludo.md "Zone orientation -- an operator safety
decision".

### `offset(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple`

Metre (east, north) offset from point 1 to point 2 (equirectangular).

The longitude delta is wrapped to [-180, 180] so a span crossing the anti-meridian (+/-180 deg)
does not flip the vector -- the same wrap the heading-error math uses (Coludo flies nowhere near
+/-180, but it is a free correctness guard). This is the ONE geographic primitive: distance() /
bearing() / range_bearing() all derive from it, so a caller needing BOTH range and bearing to the
same point computes the float-trig ONCE (via range_bearing) instead of twice (see the header note).

Args:
    lat1, lon1 - the FROM point (decimal degrees).
    lat2, lon2 - the TO point (decimal degrees).

Returns:
    (east, north) offset in metres.

### `compass(east: float, north: float) -> float`

Convert an (east, north) metre offset to a compass bearing.

Args:
    east - eastward component (metres).
    north - northward component (metres).

Returns:
    Bearing in degrees: 0 = north, 90 = east, clockwise, wrapped to [0, 360).

### `bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float`

Compass bearing from point 1 to point 2.

Args:
    lat1, lon1 - the FROM point (decimal degrees).
    lat2, lon2 - the TO point (decimal degrees).

Returns:
    Bearing in degrees (0 = north, 90 = east, clockwise).

### `distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float`

Straight-line distance from point 1 to point 2 (equirectangular).

Args:
    lat1, lon1 - the FROM point (decimal degrees).
    lat2, lon2 - the TO point (decimal degrees).

Returns:
    Distance in metres.

### `range_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple`

Distance AND bearing from point 1 to point 2 in one pass.

The fused primitive for a caller that needs both (loiter: orbit radius + tangent heading;
reachability: range + wind projection) -- one offset()/float-trig instead of the distance() +
bearing() pair that recomputed it twice.

Args:
    lat1, lon1 - the FROM point (decimal degrees).
    lat2, lon2 - the TO point (decimal degrees).

Returns:
    (distance_metres, bearing_degrees).

### `zone(corner_tl: tuple, corner_br: tuple) -> tuple`

Resolve a zone rectangle to its target centre + the two short-side gates.

A horizontally (longitude) stretched zone gates on its left/right edges; a vertically (latitude)
stretched one on top/bottom. Corner ORDER does not matter: the centre is the average, the spans use
abs(), and the gates are the coordinate EXTREMES -- so whichever diagonal pair is passed (TL/BR or
BL/TR) the two returned gates are the same side-midpoints (steer() then picks the nearer). No
normalisation needed.

Args:
    corner_tl - one corner (lat, lon), decimal degrees.
    corner_br - the diagonally-opposite corner (lat, lon).

Returns:
    (target, gate_a, gate_b): the centre (lat, lon) and the two shorter-side midpoints.

### `zone_aspect(corner_tl: tuple, corner_br: tuple) -> float`

The zone rectangle's long/short side ratio.

The endgame picks its pattern from it: a SQUARISH zone (ratio <= the config threshold) orbits a
single circle ('o'); an ELONGATED strip flies two lobes along the long axis ('oo').

Args:
    corner_tl - one corner (lat, lon), decimal degrees.
    corner_br - the diagonally-opposite corner (lat, lon).

Returns:
    The ratio (>= 1); 1.0 when a side is degenerate.

### `inside(position: tuple, corner_tl: tuple, corner_br: tuple) -> bool`

Whether a position is within the zone rectangle (corner order-agnostic).

Args:
    position - the point (lat, lon), decimal degrees.
    corner_tl, corner_br - the two diagonally-opposite corners (lat, lon).

Returns:
    True if position is inside the rectangle, else False.

### `steer(position: tuple, corner_tl: tuple, corner_br: tuple) -> tuple`

The heading to fly toward the landing target via the nearer gate.

Head for the closer short-side entrance until inside the zone, then for the centre. Stateless +
re-evaluated each tick, so the overshoot loop is emergent: if the glider crosses the zone and exits
the far side still high, the gate it just crossed is now the nearest -> it turns back (~180deg) and
re-approaches through it. No waypoint memory -- the spec's "recalculate to the nearest alternative
entry and loop" just happens.

Args:
    position - the glider (lat, lon), decimal degrees.
    corner_tl, corner_br - the zone's diagonally-opposite corners (lat, lon).

Returns:
    (bearing_degrees, waypoint, leg), with leg GATE (outside the zone) or TARGET (inside).

### `steer_to(position: tuple, corner_tl: tuple, corner_br: tuple, target: tuple, gate_a: tuple, gate_b: tuple) -> tuple`

steer() with the zone geometry (target + the two gates) ALREADY resolved.

Lets a caller that steers every nav tick resolve the per-flight-constant zone() ONCE
(mission.zone_points) instead of paying it here each call. inside() still takes the corners -- a
cheap min/max, no trig/alloc.

Args:
    position - the glider (lat, lon), decimal degrees.
    corner_tl, corner_br - the zone corners (for the cheap inside() check).
    target, gate_a, gate_b - the pre-resolved zone geometry (from zone()).

Returns:
    (bearing_degrees, waypoint, leg), with leg GATE or TARGET.

### `cross_track(position: tuple, point: tuple, heading: float) -> float`

Signed perpendicular distance from a position to a line.

Args:
    position - the point to measure (lat, lon), decimal degrees.
    point - a point ON the line (lat, lon).
    heading - the line's compass direction (degrees).

Returns:
    Perpendicular distance in metres; positive = to the RIGHT of the line, looking along heading.

### `approach(position: tuple, corner_tl: tuple, corner_br: tuple, heading: float, cross_gain: float, intercept_max: float) -> float`

Final-approach heading that TRACKS the zone's long-axis centreline (the strip).

Used low on final instead of homing to the centre POINT: the glider intercepts the line at up to
intercept_max deg (cross_gain deg per metre off it), then flies down it, so a crosswind is crabbed
out and the touchdown holds the narrow strip. (A banked crab, not a wing-low slip -- that would need
a sideslip-capable airframe model; the residual at strong wind is airframe-bound, not a control gap.)

Args:
    position - the glider (lat, lon), decimal degrees.
    corner_tl, corner_br - the zone's diagonally-opposite corners (lat, lon).
    heading - the glider's current heading (degrees), used to pick the along-strip direction.
    cross_gain - intercept degrees commanded per metre off the centreline.
    intercept_max - the cap on the intercept angle (degrees).

Returns:
    The heading to fly (degrees).

### `approach_to(position: tuple, target: tuple, gate_a: tuple, gate_b: tuple, heading: float, cross_gain: float, intercept_max: float) -> float`

approach() with the zone geometry ALREADY resolved (see steer_to()).

The caller reuses one mission.zone_points() resolve across the whole nav tick instead of each nav
call recomputing zone().

Args:
    position - the glider (lat, lon), decimal degrees.
    target, gate_a, gate_b - the pre-resolved zone geometry (from zone()).
    heading - the glider's current heading (degrees).
    cross_gain - intercept degrees commanded per metre off the centreline.
    intercept_max - the cap on the intercept angle (degrees).

Returns:
    The heading to fly (degrees).

## `pid.py`

_Tested by `test/test_pid.py`._

A minimal fixed-point PID controller for the flight stabilization loop (Phase 3), sibling of mixer.py.
One instance per control axis. Anti-windup is TWO mechanisms: the integral/output clamps bound the
magnitude (and track the governor's live fin cap via set_limit()), and BACK-CALCULATION bleeds the
integral by whatever demand the saturated fin could not fly. reset() on (re)entering a control phase.

INTEGER fixed-point (fixed.fixnum in/out, integer-millisecond dt) so a step allocates NOTHING on the
heap. The flight loop runs with GC DISABLED (sequencer disables it on BOOSTING), so every heap byte
accumulates toward OOM; the old float PID boxed a fresh float on every * + / -- measured 176 B/step,
×3 axes ×100 Hz ≈ 56 KB/s of leak. This version measures 0 B/step (even at a ±180° heading swing, the
worst case for the derivative), leaving only the isolated call-site conversion
fixed.from_float(setpoint - actual) at the sensor boundary. Net saving ≈ 47 KB/s (from the
memory-refactor work).

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

A minimal fixed-point PID controller for one control axis: error (fixnum) -> control output (fixnum).

step(error, dt_ms[, rate]) is kp*e + ki*integral(e) + kd*derivative, each clamped -- all integer,
no heap allocation. Error and output are fixnums (degrees × SCALE). The derivative is the measured
`rate` (gyro, SCALE-deg/s) when given -- derivative-on-measurement, clean + no setpoint kick --
else d(error)/dt (differentiated on the error).

- `__init__(kp: float=0.0, ki: float=0.0, kd: float=0.0, integral_limit: int=_UNBOUNDED_DEG, output_limit: int=_UNBOUNDED_DEG, anti_windup_shift: int=_ANTI_WINDUP_SHIFT)` — constructor
- `reset() -> None` — Clear the integral + derivative history.
- `set_limit(limit_deg: int) -> None` — Retune the output clamp + anti-windup integral clamp to a live authority limit (whole degrees).
- `step(error: fixnum, dt_ms: int, rate: fixnum=None) -> fixnum`

## `recorder.py`

_Tested by `test/test_recorder.py`._

The single non-hot data path: telemetry + logs into PSRAM ring buffers, drained to the Luckfox
recorder over UART. See doc/specs/coludo.md ('Task Data-Flow', 'Logging', 'Telemetry', 'Storage Write
Constraints').

Recorder is a singleton: any module calls Recorder.log() / Recorder.tlm() globally. Producers enqueue
synchronously (struct.pack_into into a ring -- never slice-assignment, which is O(buffer length) on
this port); the async run() loop drains the rings to the UART via an asyncio.StreamWriter, telemetry
(first priority) before logs. Logs are best-effort (dropped when full); telemetry is important (raises
if a record will not fit).

That trade continues on the RECORDER side, deliberately -- logging is what is spent to make telemetry
trustworthy, so the two channels have different guarantees end to end:
  * TELEMETRY is committed PER LINE, so a row that reached the link is on disk. That is what makes a
    capture survive a crash mid-flight, and why anything that must be evidence belongs in tlm().
  * LOGS are buffered and flushed roughly every 1000 telemetry messages, so a SHORT session can end
    with log lines that were never written out. A log line is a convenience, never evidence -- do not
    reason about a flight from one, and do not put a value there that a capture needs.
  * SETUP-TIME messages reach NEITHER: drivers set up before the recorder task, so the log ring is
    empty and the line is discarded. Use print() there -- the only channel that early (measured: an
    sdp810 setup line never reached recorder.log, while print() shows on the console at boot).

### `class Ring`

Lock-free single-producer / single-consumer byte ring.

The writer owns `head`, the reader owns `tail`; they never touch the same field, so it is safe
between an ISR producer and a task consumer with no locks. Each cell holds <uint16 length>
<payload>. write() uses pack_into (cost O(record)) and returns False if there is no room (the
record is skipped, never overwriting unread data). read() returns a bytes copy (stable across an
await). Holds `capacity - 1` records (one cell separates full from empty).

- `__init__(capacity: int=_DEFAULT_CAPACITY, cell_size: int=_DEFAULT_CELL_SIZE)` — constructor
- `write(data: bytes) -> bool`
- `read() -> bytes` — Return the oldest record as bytes (a copy) and advance, or None if empty.
- `count() -> int` — Records currently queued (a stats snapshot).

### `class Recorder`

The global telemetry + log singleton: enqueue synchronously, drain to the Luckfox UART async.

- `setup(config: dict, uart=None) -> None` _(classmethod)_
- `timestamp() -> int` _(classmethod)_ — Monotonic-ish record timestamp. Currently raw microseconds; the unit may change.
- `session() -> str` _(classmethod)_ — The per-boot file prefix.
- `log(descriptor: str, message: str) -> bool` _(classmethod)_ — Best-effort log line "<ts> <descriptor> :: <message>" (-> recorder.log).
- `cc_logs(duration_ms: int) -> dict` _(classmethod)_ — Poll-model CC log streaming (the `log <ms>` command).
- `cc_telemetry(duration_ms: int) -> dict` _(classmethod)_ — Poll-model CC telemetry streaming (the `tlm <ms>` command).
- `tlm(filename: str, content: str) -> None` _(classmethod)_ — Queue an important telemetry line "@<session>_<filename>@<content>".
- `tlm_raw(data: bytes) -> None` _(classmethod)_ — Queue an ALREADY-ENCODED telemetry line (the hot path Telemetry.push uses).
- `drain() -> int` _(classmethod)_ — Drain queued records to the UART, telemetry first then logs. Returns records drained.
- `run() -> None` _(classmethod)_ — Event-driven drain loop: wait for a producer signal, then drain everything queued.
- `inspect() -> dict` _(classmethod)_
- `update(props: dict) -> list` _(classmethod)_
- `stats() -> dict` _(classmethod)_
- `report() -> dict` _(classmethod)_

### `class Telemetry`

A typed telemetry stream.

Created with a destination file and its data field names; the first push emits the CSV header
(uptime + fields), then each push emits a timestamped row. All streams in one boot share the
Recorder session prefix, so file names are stable.

`decimate_us` rate-limits the stream: push() emits only when at least `decimate_us` microseconds
have passed since the last emitted row (a fast sensor can push every sample and have its telemetry
decimated to a sane rate). `decimate_us=0` (the default) inherits the Recorder GLOBAL rate
(`Recorder.telemetry_decimate_us`, 50 Hz) -- so a stream opts into an individual rate by passing a
non-zero value, else the board-wide `recorder.telemetry_us` prorates it.

- `__init__(filename: str, fields: tuple, decimate_us: int=0)` — constructor
- `due(now: int) -> bool` — Whether the decimation window has elapsed -- so a HOT-PATH producer can skip building its row.
- `push(values) -> None`

## `servo.py`

_Tested by `test/test_servo.py`._

Shared servo infrastructure, sibling of the bus helpers (i2cbus/spibus). The slew gate bounds how many
fins slew at once (the boost-rail current transient): a process-wide counting semaphore so
`fins.concurrency` (board config) caps total simultaneous slews across every servo driver.
Servo-type-agnostic -- each driver (sg90, future mg90s/mg996r) imports the gate and adds its own pulse
range + slew timing.

### `class Gate`

A tiny FIFO counting semaphore (MicroPython asyncio has no Semaphore, only Lock/Event).

At most `permits` holders at once, the rest queue and are handed a permit in order on release. The
process-wide shared instance lives on the class itself (Gate.slew()/Gate.reset()) -- no module
global.

- `__init__(permits: int)` — constructor
- `acquire() -> None`
- `release() -> None`
- `slew(permits: int) -> 'Gate'` _(classmethod)_ — The process-wide slew gate, created once and shared by every servo driver.
- `reset() -> None` _(classmethod)_ — Drop the shared gate so the next Gate.slew() rebuilds it.

## `sim_model.py`

_Tested by `test/test_sim_model.py`._

Pure flight-dynamics model shared by the on-board HITL task (tasks/hitl.py) and the host-side
virtual-flight tool (tools/virtual_flight.py). PURE: math + random only, no hardware, so it runs
identically on the board (MicroPython) and on the host (CPython) -- the virtual flight and the HITL
sim are then the SAME physics, only the harness around them differs. World frame is ENU metres from
the launch pad; attitude is Euler degrees (roll, pitch, yaw=heading).

### `class Faults`

Sensor-fault injection for robustness runs (findings §27.20).

The firmware is full of degradation paths -- databoard priority fallback, the unconfident airspeed
cap floor, the GNSS jump/steep gates, pitot saturation, warm start -- and those are exactly the paths
that run when a flight is already going badly. Only a roll spike and a manual BNO055 drop were ever
exercised, so the rest were untested. This injects the rest at the PUBLISH boundary (where a harness
hands a reading to the control stack), which is where a real sensor failure appears.

Spec is a comma-separated list of `channel` or `channel@seconds` (when it starts, default 0):

    'baro'              baro dead from t=0
    'gnss@30,pitot@45'  GNSS drops at 30 s, the pitot rails at 45 s

Modes per channel: a DEAD channel returns None (the driver stopped / the databoard aged it out); the
pitot instead RAILS to its full-scale reading, because a saturated differential-pressure sensor
under-reads airspeed rather than going silent -- the more dangerous failure, since a low airspeed
LOOSENS the fin cap.

- `__init__(spec: str='')` — constructor
- `active(channel: str, t: float) -> bool` — Whether `channel` is faulted at time `t` (seconds since launch).
- `apply(channel: str, value, t: float)` — The reading a faulted channel should publish; `value` unchanged when healthy.

### `class Body`

Flight-dynamics state + integrator (PURE -- host-testable).

`boost_step()` climbs vertically; at apogee `begin_glide()` hands over to `glide_step()`
(fin-controlled); `sensors()` returns what the on-board sensors would read.

- `__init__(mass: float, launch: tuple, elevation_m: float, glide_heading: float, glide_mass: float=None)` — constructor
- `boost_step(dt: float, thrust: float, pitch_cmd: float=0.0, roll_cmd: float=0.0) -> None` — Vertical climb (1-DoF: thrust + gravity + drag) PLUS attitude under thrust.
- `begin_glide() -> None` — Apogee hand-over: the booster ejects and the glider noses down into the trim glide.
- `glide_step(dt: float, roll_cmd: float, pitch_cmd: float, yaw_cmd: float) -> None` — Rigid-body glide step under fin control.
- `position() -> tuple`
- `track() -> float` — Ground-track bearing (deg) -- the direction the glider MOVES over the ground.
- `ground_speed() -> float` — Horizontal GNSS GROUND speed (m/s) -- the magnitude of the ground velocity, WITH the wind.
- `sensors() -> dict` — Clean (pre-noise) sensor readings from the current state.

### `noisy(value, frac: float, lo: float, hi: float, reference: float=None)`

Perturb a scalar by +/- frac of a REFERENCE magnitude (uniform), clamped to [lo, hi].

The reference defaults to `abs(value) + 1` -- noise proportional to the reading, which is right for
a quantity whose error genuinely scales with it (speed, dynamic pressure, acceleration).

It is WRONG for a CIRCULAR quantity, and that mattered. A compass heading lives on 0..360, so
magnitude-proportional noise made "5 %" mean +-18 deg near 350 and +-0.5 deg near 10 -- as if north
were more certain than south. A real BNO055 fused heading is ~+-1-2 deg wherever it points. Every
noise>0 study was therefore exercising an absurdly hostile heading signal: measured on the host,
switching heading to an absolute +-1 deg cut fin travel 22791 -> 16687 deg (-27 %) while touchdown
moved 118.3 -> 118.4 m. So it never distorted the ACCURACY results, only the fin-activity and
servo-power ones -- which is exactly what the numbers were being used for.

Pass `reference` for such channels (see HEADING_NOISE_REF) to get an absolute error band instead.

Args:
    value - the clean scalar to perturb.
    frac - the noise fraction (0 -> returned clean).
    lo - the lower clamp bound.
    hi - the upper clamp bound.
    reference - noise scale to use instead of abs(value) + 1 (for circular / absolute-error channels).

Returns:
    The perturbed, clamped value; the clean value clamped when frac is 0.

## `spibus.py`

Shared, lock-serialized SPI buses, mirroring i2cbus. A sensor may move off the shared I2C bus onto SPI
(e.g. the ADXL375, for clean high-rate reads): each bus id gets ONE machine.SPI plus an asyncio.Lock,
and get() hands back the shared wrapper. device(cs) returns a register window with the SAME
read/read_into/write(reg, ...) interface as i2cbus, so a driver is bus-agnostic. The chip-select is a
plain GPIO held low only around each locked transaction (the SPI peripheral does not own it, so several
devices can share one bus). A glider-only module (MicroPython).

### `class Bus`

One physical SPI bus, shared by every device on it; transactions are serialized by a lock.

- `__init__(bus_id: int, spec: dict)` — constructor
- `device(cs: int, mb_bit: int=6) -> _Device` — A register window for one chip-select on this bus (matches i2cbus.Bus.device).
- `retune(freq: int) -> None` — Re-init this SPI peripheral at `freq` Hz in place (bench frequency calibration; no reboot).

### `get(bus_id: int, spec: dict) -> Bus`

The shared Bus for `bus_id`, created once from `spec` (sck/mosi/miso/baud/mode) and cached.

### `bind(board: dict, device: dict)`

Resolve a device's config block to the SPI bus it talks over -- i2cbus.bind's twin.

The two dual-bus drivers (adxl375, lsm6dso32) pick their family at runtime, so each carried its own
copy of the same `config.bus()` preamble to resolve EITHER family. With both modules exposing
bind(), that preamble is gone from both and each driver's transport helper is just the dispatch.

Returns the BUS ALONE, not i2cbus.bind's (bus, addr) pair, and the difference is real rather than an
oversight: an I2C address is a plain integer sitting in the device block, but a chip-select is a
board PIN, resolved through the pin map by Task._pin_gpio. Pin mapping is the task's job, so the
caller passes the resolved GPIO to bus.device() itself.

Args:
    board - the whole board config (holds the `buses` section).
    device - the component's own config block ('bus', 'id').

Returns:
    The shared Bus, or None when the config declares no such bus. Default id is 1: spi:1 is the
    only SPI bus the board declares (i2cbus.bind defaults to 0 for the same reason).

## `task.py`

_Tested by `test/test_task.py`._

Task base class and driver registry -- the unit the Controller creates and supervises.

Every component/system task follows the common lifecycle from doc/specs/coludo.md:
  setup() async; initialize or reset; return True on success
  probe() async; ON-DEMAND self-test (the CC `probe` command, never at boot) -> None if healthy,
      else an error string. Default None; a sensor reports 'X not found on i2c:0', an actuator
      exercises itself (the servo sweeps its range) -- so a mid-flight reboot never sweeps fins.
  run() async; the task's main activity loop
  notify() subscribe a callback for this task's updates
  validate() return True if the task is currently healthy
  finish() async; shut down and release resources
A Task is Inspectable: inspect()/update()/stats() expose it to the operator (the Controller registers
each task with the Inspector), so there is no separate report().

A task registers itself with @activity('name') (or its alias @driver('name') for the HAL ones in
drivers/) into ACTIVITIES, the CLASS registry: name -> Task subclass, "what can be built". It is a
module global on purpose -- the decorators fill it at IMPORT time, before any Controller exists, so it
cannot live on a Controller instance (that is why moving it into the Controller would be a mess, not a
tidy-up). The Controller READS it (injected as `registry`, defaulting to ACTIVITIES) to build a
component, and keeps its own INSTANCE directory -- find()/query(), "what is currently running" -- for
dependency lookup. Two deliberately separate lookups: class-by-name here, instance-by-name on the
Controller. The driver/activity names share one registry for now; splitting drivers out later.

### `activity(name: str)`

Class decorator: register a Task subclass under a name.

Registers a HAL driver or a higher-level activity so the Controller can build it from a config
component.

Args:
    name - the registry key a config component names to build this class.

Returns:
    The class decorator, which registers the class and returns it unchanged.

### `class Task(inspector.Inspectable)`

- `__init__(name: str, config: dict=None, controller=None)` — constructor
- `event(message: str) -> None` — A DURABLE diagnostic record -- for the handful of facts a flight must not be able to lose.
- `note(template: str=None, arg=None) -> None` — De-duplicated best-effort run-loop log + runtime-health flag.
- `setup() -> bool` — Initialize or reset. Override. Return True on success, False otherwise.
- `claim() -> None` — Wait until no other caller owns this device's MULTI-STEP conversation, then take it.
- `unclaim() -> None` — Release the claim taken by claim(); call it from a `finally`, never a happy path only.
- `strike(failed: bool, limit: int) -> bool` — Count a run of CONSECUTIVE failures; True exactly ONCE, when the run reaches `limit`.
- `calibration() -> str` — What the OPERATOR must do to make this device flight-ready -- or '' when there is nothing.
- `calibrate() -> str` — Start / enforce this device's calibration (the CC `calibrate <device>` command).
- `probe() -> str` — On-demand self-test (the CC `probe` command, NOT run at boot).
- `run() -> None` — The task's main activity loop.
- `notify(callback) -> None` — Register callback(task, event) to be invoked on this task's updates.
- `emit(event=None) -> None` — Notify all subscribers of an update.
- `find(names: list[str]) -> list` — Non-blocking sibling lookup via the Controller (None for any not up).
- `query(names: list[str], waiting: bool=True) -> list` — Await sibling tasks by name via the Controller.
- `validate() -> bool` — Return True if the task is currently healthy.
- `finish() -> None` — Shut down and release resources.
- `inspect() -> dict` — Status dict. Subclasses extend it.

## `warmstart.py`

_Tested by `test/test_warmstart.py`._

In-flight reboot recovery (doc/specs/coludo.md "In-flight reboot & warm start"). A mid-air reset
(watchdog, brownout-survivor, crash) must not turn the glider ballistic: the Checkpoint task keeps a
tiny CRUMB in NVS (never a VFS file -- a filesystem write locks the scheduler and wears the data
flash; esp32.NVS commits to its own partition in milliseconds) carrying the live flight state. At
boot, main.py restores the SAVED stage when the crumb and the per-stage physical signals agree -- see
should_restore() for the gate -- and the normal detectors (separation / apogee / landing) re-evaluate
from there.

The Checkpoint task writes the crumb every `checkpoint_s` while airborne (BOOSTING/GLIDING/LANDING;
floored at 1 s) and once on entering each stage, but ONLY for an ARMED flight (a disarmed passive
flight must never warm-start into an armed stage); it always writes the telemetry/log timeline.
Recovery is stage-aware: SETTING recovers as a plain cold boot, DONE stays DONE (a landed glider never
re-enters the flight sequence), and the passive glide stages must show the separation latch.

Storage layout: the `stage` i32 IS the flag (Stage.NULL = 0 -> cold, non-zero -> recover it -- no
separate key), the payload is ONE JSON blob (`crumb`) -- full float precision, no per-field key
bookkeeping, and a new field is a dict entry rather than an NVS schema change. The module degrades to
no-ops off-board (CPython).

### `save(crumb: dict) -> bool`

Commit the checkpoint crumb to NVS: the blob, then the stage i32 LAST.

The Checkpoint task builds the crumb (the live flight state + the BOOSTING-captured identity);
this is the raw write. There is no separate flag -- the `stage` i32 IS the flag via the Stage.NULL
= 0 sentinel (0 -> cold, non-zero -> recover it), and it is written last so a torn write can never
leave a live stage pointing at a half-written blob.

Args:
    crumb - the checkpoint dict: stage / armed / altitude / speed / ticks_ms / stamp, plus the
        launch / zone / pad_altitude frozen at BOOSTING.

Returns:
    True when committed; False (never raises) when NVS is absent or full -- a failed checkpoint
    must never block the flight.

### `clear() -> None`

Reset the stage i32 to Stage.NULL = 0 (after a rejected warm start).

The blob stays -- the stage i32 alone decides, so the clear is a single fast write. Cold-boots the
next start unambiguously (a fresh power-on already re-writes SETTING; this makes a rejected crumb
stop pointing at a stale flight).

Returns:
    None. Zeroes the NVS `stage`; never raises (a failed clear is swallowed).

### `load()`

Read back the last checkpoint crumb (the stage i32 gates it, then the blob fills in).

Returns:
    The crumb dict (the blob fields + `stage` from the authoritative i32), or None when no flight
    was checkpointed (stage absent / Stage.NULL) or the blob is missing/torn (-> cold boot).

### `should_restore(crumb, separated: bool, cause_is_reset: bool, now_s) -> tuple`

The warm-start gate: a legitimate mid-flight reset to recover the crumb's stage into?

The periodic checkpoint keeps the crumb's `stage` fresh (re-stamped every second aloft, so it is
at most ~1 s stale at a reset), so the STAGE itself is trustworthy -- the gate only has to confirm
this is a genuine RECENT reset of a real flight, not that the altitude independently agrees (a
height cross-check was an atavism of the old single-breadcrumb design and is gone):

  * universal (every stage): a valid crumb (carries stage + stamp); `cause_is_reset` -- WDT/SOFT/
    HARD, never a power-on (a battery insertion / power switch is a human -- a fresh flight or a
    recovery crew -> cold); the crumb age in 0.._MAX_AGE_S (the RTC survives soft/WDT resets so the
    continuity holds; a power cycle restarts it and breaks the arithmetic -> cold).
  * passive stages (GLIDING/LANDING, the unpowered post-separation glide): the separation switch
    reads SEPARATED -- the physical latch no software can fake, so a landed-then-nested glider can
    never recover into a glide. BOOSTING is the active boost (pre-separation, latch nested), so
    reset + age carry it; SETTING/DONE are on the ground and need neither (SETTING recovers as a
    plain cold boot, DONE just stays DONE).

Pure function of its inputs (host-testable).

Args:
    crumb - the crumb dict from load(), or None.
    separated - the separation driver's latch reading (True = separated).
    cause_is_reset - True when machine.reset_cause() was WDT/SOFT/HARD (not a power-on).
    now_s - the current time (RTC epoch seconds).

Returns:
    (restore, reason): restore True with the passing reason when the gate agrees, else False with
    the first failing reason.

### `restore(flight, cfg: dict, log=print) -> bool`

Warm start (doc/specs/coludo.md "In-flight reboot & warm start") -- was main._restore_flight, moved
here so main.py stays a thin bring-up.

A mid-air reset must not turn the glider ballistic: restore GLIDING when the NVS breadcrumb AND two
physical signals agree -- the separation latch (read via the separation DRIVER, not a raw Pin) and
the baro absolute altitude clearly above the crumb's pad. Any doubt -> the crumb is cleared and
this is a normal cold boot.

Args:
    flight - the controller to move into GLIDING on a passed gate.
    cfg - the board config (the checkpoint component's warm_start toggle + the sensor list for the baros).
    log - the log sink (defaults to print).

Returns:
    True when a warm start was applied (-> gliding, armed, GC off); False on a normal cold boot or
    a rejected gate.

### `class Checkpoint(task.Task)`

Periodic + on-stage-change flight-state checkpoint to NVS -- the warm-start source.

Every `checkpoint_s` while AIRBORNE (BOOSTING/GLIDING/LANDING; floored at 1 s) and once on
entering each stage, write the live state to the NVS crumb -- but ONLY for an ARMED flight (a
disarmed passive flight must never warm-start into an armed stage). The telemetry/log timeline is
written on every checkpoint regardless. SETTING and DONE checkpoint once on entry and never
periodically -- nothing moves during the long pad dwell / post-landing wait, so re-writing every
period would only wear the flash.

- `setup() -> bool`
- `run() -> None` — Checkpoint on every stage change + every period_ms while airborne; forever.

## `wind.py`

_Tested by `test/test_wind.py`._

Wind estimation from GNSS. The MINIMAL method, proven first: the WIND TRIANGLE.

The GNSS reports the ground velocity (course + ground speed); the attitude gives the heading; the
governor gives the airspeed. The air mass the glider flies through is moving, so
  wind = ground_velocity - air_velocity = (ground along course) - (airspeed along heading).
An EMA smooths the per-fix estimate. The CROSSWIND component is bias-free; the ALONG-heading component
inherits the governor's airspeed error (an over-read biases head/tailwind) -- acceptable for the rough
uses (reachability margin, approach crab). If the field data shows that bias hurts, the airspeed-free
GPS-only min/max-ground-speed method is the next layer (kept out until a corner case earns it).

The estimator OWNS its config subtree (the board `wind` block) and is Inspectable: it publishes and
accepts live tweaks of triangle_alpha + the physical envelope (calm_speed..max_speed) through
inspect()/update(), rather than the caller wiring individual params in. Float trig, fed once per GNSS
fix (off the hot loop) via observe() -- a telemetry + reachability/approach input, not a control fixnum.

### `class WindEstimator(inspector.Inspectable)`

Estimate the wind (east/north m/s) from the GNSS ground velocity vs the air velocity.

The wind triangle, EMA-smoothed. observe() folds in one GNSS fix; speed()/direction()/components()
read the estimate; inspect()/update() publish + retune the config subtree.

- `__init__(config: dict=None)` — constructor
- `observe(course: float, ground_speed: float, airspeed: float, heading: float) -> None` — Fold in one GNSS fix (the wind triangle for this sample, EMA-blended into the estimate).
- `components() -> tuple` — (east, north) m/s -- floored to calm (0, 0) below the envelope's calm_speed.
- `speed() -> float`
- `direction() -> float` — Where the wind blows FROM (meteorological convention), degrees. 0 when calm.
- `inspect() -> dict` — Operator-facing: the tunable config + the current estimate.
- `stats() -> dict` — Diagnostics for the wind soak / telemetry: the method, the estimate, and the raw components.

# glider HAL drivers — `drivers/` — `src/glider/drivers`

## `adxl375.py`

ADXL375 ±200 g high-G accelerometer: the boost-phase accel channel. Works over I2C (shared bus) OR SPI
(its own bus, for clean high-rate reads) -- the component's `bus` field selects, and a shared
register-window device (i2cbus/spibus .device()) keeps the driver code bus-agnostic.
@task.driver('adxl375'). setup() probes the device id and configures it; run() writes the latest
(x, y, z) acceleration in g to the databoard 'accel' slot. If the device is absent (no ack / wrong
device id) setup() returns False and the Controller skips it -- the board boots fine with the sensor
unplugged.

Sampling is interrupt-driven when an `int_pin` (INT1) is wired: the chip raises DATA_READY when a new
sample is ready, an IRQ sets a plain bool, and run() waits on it in slices -- so the coroutine sleeps until
there is genuinely fresh data instead of blind-polling. A `fallback_ms` timeout still forces a sample if
interrupts go silent (dead sensor / wiring). With no int_pin it falls back to a plain `period_ms` poll.
Uses the shared locked I2C bus (i2cbus), as it shares i2c:0 with other sensors.

### `class Adxl375(task.Task)`

High-G accel: samples (x, y, z) in g to the databoard 'accel' slot, interrupt-driven.

- `setup() -> bool`
- `sample() -> tuple` — Read one acceleration sample from the device.
- `run() -> None` — Sample on DATA_READY (or every fallback_ms if interrupts go silent); plain poll with no INT wired.
- `probe() -> str` — On-demand self-test: the device id reads back, then one sample succeeds (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: classify the wire-level fault.
- `inspect() -> dict`

## `atgm336h.py`

ATGM336H GNSS (GPS + BDS, CASIC chip) on a dedicated UART. @task.driver('atgm336h'). All NMEA
reading/parsing lives in the shared gnss.Gnss base; this driver only adds the CASIC reconfiguration:
RMC at `hz` (position) plus GGA at ~1 Hz (altitude/elevation, a baro backup) -- both fit 9600 baud
(~10 Hz RMC ~700 B/s + ~1 Hz GGA ~70 B/s < 960). PCAS is the CASIC command set; the PMTK pair is sent
too as a fallback for MTK-variant modules (each side ignores the other's sentences). Graceful: an
undefined bus -> setup False (the Controller skips it).

### `class Atgm336h(gnss.Gnss)`

ATGM336H (CASIC): RMC at `hz` for position + GGA at ~1 Hz for altitude/elevation.


## `bluetooth.py`

Set the BLE radio to the state declared in config at boot. The component field `radio` (true/false,
default false) says whether Bluetooth should be ON; the driver applies it -- transparent, so nobody is
surprised by an implicit disable. Default false saves power (the wireless is the external C6 and BLE is
unused on the glider). Setup-only @task.driver('bluetooth') plus update() so the operator can toggle it
live (`update bluetooth {"radio": true}`).

### `class Bluetooth(task.Task)`

Apply the configured BLE radio state. Inspectable: `radio` requested, `active` actual.

- `probe() -> str` — On-demand self-test: the BLE radio is in the requested state (or absent on this board ->
- `setup() -> bool`
- `run() -> None` — Setup-only: no run loop. `update()` is the runtime entry point.
- `inspect() -> dict`
- `update(props) -> list`

## `bmp280.py`

BMP280 barometric pressure sensor (on the SEN0253) over the shared I2C bus: the backup altitude
channel. @task.driver('bmp280'). setup() probes the chip id, reads the factory calibration and starts
normal-mode conversion; run() reads pressure, applies Bosch compensation and writes pressure (Pa),
temperature (°C), altitude (m AMSL) and elevation (m above the per-sensor startup ground zero) to the
databoard. Graceful: wrong/absent chip id -> setup False -> skipped.

Polled at period_ms (the BMP280 conversion is ~tens of ms, far slower than the IMU). Uses the shared
locked bus (i2cbus) since it shares i2c:0 with the ADXL375 and BNO055.

### `class Bmp280(task.Task)`

Backup baro to the databoard: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation.

Elevation is metres above the startup ground zero, captured per-sensor so it is offset-free.
`update {"rezero": true}` re-captures ground zero (e.g. after warm-up, just before launch).

- `setup() -> bool`
- `run() -> None`
- `update(props: dict) -> list` — Apply an operator property change: re-zero or directly set the ground reference.
- `probe() -> str` — On-demand self-test: the chip id reads back, then one conversion reads (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: classify the wire-level fault.
- `inspect() -> dict`

## `bno055.py`

BNO055 9-DOF IMU (on the SEN0253) over the shared I2C bus: the attitude channel.
@task.driver('bno055'). In NDOF fusion mode the chip computes absolute orientation on-chip; run() reads
the Euler angles (heading, roll, pitch in degrees) to the databoard 'attitude' slot. Graceful: a
wrong/absent chip id -> setup False -> the Controller skips it.

BNO055's INT pin signals motion/threshold events, not a fusion data-ready, so this driver polls at
period_ms (the fusion engine runs at 100 Hz internally); the wired int_pin is reserved for future event
detection (e.g. high-g). Uses the shared locked bus (i2cbus) since it shares i2c:0 with the ADXL375 and
BMP280.

### `class Bno055(task.Task)`

9-DOF IMU to the databoard: fused attitude and a calibrated low-g accelerometer.

NDOF fusion attitude (heading, roll, pitch in degrees) -> 'attitude', plus the calibrated
accelerometer (g, including gravity) -> 'accel' as a low-g backup to the ADXL375 (priority 1).

- `setup() -> bool`
- `sample() -> tuple` — Read the ACC..EUL block and return a FLAT 6-tuple (run() slices it).
- `calibrated() -> bool` — True once the MAGNETOMETER is calibrated -- the axis that needs the operator's figure-8.
- `calibration() -> str` — The figure-8 instruction while NDOF is unconverged, with the live reading folded in; '' once done.
- `run() -> None`
- `probe() -> str` — On-demand self-test: the chip id reads back, then one fused sample succeeds (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: classify the wire-level fault.
- `inspect() -> dict`

## `icp10111.py`

ICP-10111 barometric pressure sensor (TDK ICP-101xx, on the SEN0517) over the shared I2C bus: the
PRIMARY altitude channel (8.5 cm accuracy). @task.driver('icp10111'). Command-based, not
register-mapped: setup() verifies the product id and reads the 4 OTP calibration constants; run() issues
a measure command, reads pressure+temperature, applies the TDK polynomial conversion and writes pressure
(Pa), temperature (°C), altitude (m AMSL) and elevation (m above the per-sensor startup ground zero) to
the databoard. Graceful: wrong/absent id -> setup False -> skipped.

Polled at period_ms. Uses the shared locked bus (i2cbus); shares i2c:0 with the other sensors.

### `class Icp10111(task.Task)`

Primary baro to the databoard: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation.

Elevation is metres above the startup ground zero, captured per-sensor so it is offset-free.
`update {"rezero": true}` re-captures ground zero (e.g. after warm-up, just before launch).

- `setup() -> bool`
- `calibration() -> str` — The ground-reference instruction; '' once a ground zero is held.
- `calibrate() -> str` — Re-capture ground zero from the live reading -- do it with the glider ON THE PAD.
- `run() -> None`
- `update(props: dict) -> list` — Apply an operator property change: re-zero or directly set the ground reference.
- `probe() -> str` — On-demand self-test: confirm the run loop is producing pressure.
- `diagnose() -> str` — Deeper analysis when setup() failed: re-issue the product-id command and classify the fault.
- `inspect() -> dict`

## `ina226.py`

INA226 high-side current / voltage / power monitor over the shared I2C bus: the battery (or 5 V)
supply-line sensor for consumption tracking. @task.driver('ina226'). setup() verifies the die id,
programs the conversion config, and computes + writes the calibration register from the shunt
resistance + the expected max current (the only board-specific numbers); run() polls the bus voltage
(V), current (A) and power (W) to the databoard + telemetry. Graceful: wrong/absent die id -> setup
False -> the Controller skips it.

The INA226 measures the SHUNT VOLTAGE directly (2.5 uV/LSB), so the absolute accuracy comes from the
CAL register, not a precise resistor: Current_LSB = max_current / 2**15, CAL = 0.00512 / (Current_LSB *
shunt_ohms). To trust the watt-hours, calibrate 'shunt_ohms' against a KNOWN current once and back out
the effective value -- a 2-wire ohmmeter cannot resolve a 0.01 ohm shunt.

### `class Ina226(task.Task)`

High-side power monitor: bus voltage (mV), current (mA) and power (mW) as INTEGER milli-units.

No float -- pushed to the databoard + per-sample telemetry. Current/power scale from 'shunt_mohms'
+ 'max_current_ma' (the CAL register). The same driver serves the 5 V USB phase and the LiPo phase
-- it reports the INA's own bus voltage, so power is correct as the base rail changes. Graceful: a
wrong/absent die id -> setup False.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: the die id reads back, then one live read (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: classify the wire-level fault behind an absent monitor.
- `inspect() -> dict`

## `led.py`

Status LED driver. One GPIO shows the board state at a glance: fast blink when a task is unhealthy
(error), slow blink while setting up / standing by, solid once flying. The pin role (default
'led_status') comes from the component's `pin` field, resolved against the config `pins` section.
Registered as @task.driver('led') so the Controller creates and supervises it.

### `class LedStatus(task.Task)`

Blink a status pattern on one GPIO derived from the controller's state + health.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: blink the status LED a few times so it is seen to drive, then off.
- `inspect() -> dict`

## `lsm6dso32.py`

LSM6DSO32 6-DoF IMU: the primary raw accel + the sole gyro 'rate'. A +/-32 g accel range (covers the
8-12 g boost without clipping, fine 1 g resolution for the airspeed integrator) plus a +/-2000 dps
gyro. @task.driver('lsm6dso32'). setup() checks WHO_AM_I, configures accel/gyro, and provides both the
'accel' (x,y,z in g) and 'rate' (x,y,z in deg/s) databoard slots; run() writes the latest reading. If
the device is absent (wrong WHO_AM_I) setup() returns False and the Controller skips it.

Wired on SPI1 (its own chip-select, shared with the ADXL375) for clean high-rate reads -- see
doc/waveshare_esp32p4_pins.md. SPI is 4-wire mode 3; multi-byte reads auto-increment via CTRL3_C.IF_INC
(so the bus device takes mb_bit=None -- no address multi-byte bit). I2C (addr 0x6A) also works if the
component sets bus 'i2c'. Sampling is interrupt-driven on INT1 (accel data-ready) when an 'int_pin' is
wired, else a plain period_ms poll, mirroring the ADXL375 driver. Gyro + accel sit in contiguous output
registers (0x22..0x2D), so one 12-byte read fetches both.

### `class Lsm6dso32(task.Task)`

6-DoF IMU: samples accel (x,y,z g) -> 'accel' and gyro (x,y,z deg/s) -> 'rate', interrupt-driven.

- `setup() -> bool`
- `sample() -> tuple` — Read one accel + gyro sample as a flat 6-tuple.
- `run() -> None` — The sampling loop: publish the latest accel + gyro to the databoard, forever.
- `probe() -> str` — On-demand self-test: WHO_AM_I reads back, then one sample succeeds (each step logged).
- `diagnose() -> str` — Deeper analysis when setup() failed: classify the wire-level fault behind an absent IMU.
- `inspect() -> dict`

## `mg90s.py`

MG90S metal-gear positional fin servo. @task.driver('mg90s'). Electrically IDENTICAL to the SG90 (same
50 Hz frame, ~500..2500 us pulse -> angle, open-loop, no feedback), so this is a THIN SG90 subclass --
the angle->pulse math, per-fin `trim`, the shared slew gate, update()/move() and the probe() self-test
are all inherited. What differs is MECHANICAL: metal gears give higher holding torque, so aerodynamic
load cannot back-drive the horn (an SG90's plastic train lets wind slide the fin off its commanded
angle). Preferred on the yaw fin (the rudder sees the most steady wind pressure).

TRAVEL is per instance via the component's min_deg/max_deg, whose SPAN must equal the servo's
mechanical travel so one command-degree maps to one degree of rotation. The 180deg variant uses the
default (0..180) and drops in exactly where an SG90 was. The 360deg-travel MG90S wants its 360deg
CENTRED on the mixer neutral (90): `min_deg: -90, max_deg: 270` -> command 90 = servo mid = 1500 us =
fin centre (so boot / probe / failsafe all sit centred), and +/-45 command = +/-45deg of fin. A fin
only uses neutral +/- the mixer limit, so the spare range is unused and the ~0.18deg pulse step is
still plenty. MIXED fleets are fine -- each fin's `driver` is independent (e.g. mg90s yaw + sg90
elevons); the mixer commands angles by name and is servo-type-blind.

Slew is a touch quicker (metal gear) and the probe draw window is shifted up (more current). Both are
datasheet-approximate TYPE defaults -- tune per built rig via the component's slew / engine_*_mw config.

### `class MG90S(SG90)`

MG90S metal-gear fin servo -- the SG90 protocol + logic with higher holding torque.

Center the 360deg-travel instance on the mixer neutral with `min_deg: -90, max_deg: 270`; the
180deg variant and the default match the SG90 elevons. Everything else (trim, clamp, slew gate,
probe, open-loop reporting) is SG90's.


## `neo6mv2.py`

GY-NEO6MV2 (u-blox NEO-6M) GNSS on a dedicated UART: a drop-in alternative to the ATGM336H on the SAME
UART -- swap the component `driver` to 'neo6mv2' in config (and lower `hz`; the NEO-6M tops out near
5 Hz). @task.driver('neo6mv2'). NMEA read/parse is the shared gnss.Gnss base; this driver only adds the
u-blox reconfiguration: $PUBX,40 selects RMC (position) + GGA at ~1 Hz (altitude/elevation) on the UART
and silences the rest, then UBX-CFG-RATE sets the measurement period. Default link is 9600 8N1, like the
ATGM. Graceful: an undefined bus -> setup False.

### `class Neo6mv2(gnss.Gnss)`

u-blox NEO-6M: $PUBX,40 selects RMC + ~1 Hz GGA, UBX-CFG-RATE sets the measurement period.


## `sdp810.py`

SDP810-500Pa differential-pressure sensor (Sensirion SDP8xx, thermal flow-through) over the shared I2C
bus: the pitot/static AIRSPEED channel. @task.driver('sdp810'). Command-based, not register-mapped:
setup() clears any prior continuous mode, starts continuous measurement (differential-pressure temp-comp,
average-till-read) and validates one CRC-checked frame; run() reads the 9-byte frame each period, scales
the tared dynamic pressure (a `fixed` fixnum, Pa × SCALE) and derives airspeed once, publishing both to
the databoard. Graceful: nothing acks / a corrupt frame -> setup False -> skipped.

Bench-verified: 0x25 on i2c:0 (SDA 7 / SCL 8), scale factor 60 (Pa = raw/60), zero ~0.02 Pa. Tube
polarity (blow-verified): P+ = pitot (total), P- = interior static. The interior-static PRESSURE bias is
tared out by `zero_offset_pa` (a pad tare, `update {"zero": true}`); the position-span error folds into
`air_density`, the single q->v knob (a GNSS-vs-q calm pass trims it).

INTEGER internals, ONE float: the raw scaling and the pad-tared dynamic pressure stay a `fixed` fixnum
(a small int, so the store never boxes). Airspeed = sqrt(2q/rho) is the ONE float, computed ONCE per read
and reused for the airspeed channel (-> the governor's estimator, airspeed.py) AND the telemetry row -- no
second conversion anywhere. The governor consumes the ready airspeed; it does no sqrt of its own.

Polled at period_ms in continuous mode (no per-sample write). Uses the shared locked bus (i2cbus);
shares i2c:0 with the other forward sensors.

### `class Sdp810(task.Task)`

Pitot/static airspeed + dynamic pressure to the databoard.

`dynamic_pressure` is a `fixed` fixnum (Pa × SCALE, signed); `airspeed` is m/s, v = sqrt(2q/rho),
derived once per read and shared with telemetry. The pad-tared zero cancels the interior-static
PRESSURE bias; `air_density` is the single pressure->speed knob (it absorbs the position-span error,
trimmed on a GNSS-vs-q calm pass). The saturation guard lives with the consumer (the governor drops
back to the accel backbone when the pitot rails), so this driver just reports what it reads.

- `setup() -> bool`
- `calibration() -> str` — The still-air tare instruction; '' once a zero offset has been captured.
- `calibrate() -> str` — Capture the current still-air reading as the zero offset -- the board can do this itself.
- `run() -> None`
- `update(props: dict) -> list` — Apply an operator property change: pad-tare the zero, set the zero offset, or set the density.
- `probe() -> str` — On-demand self-test: confirm the run loop is producing readings.
- `diagnose() -> str` — Deeper analysis when setup() failed: read one frame and classify the fault at the wire level.
- `inspect() -> dict`

## `separation.py`

Stage-separation switch: two adhesive copper pads (one on the glider, one on the booster) that route
3V3 to a pin while nested (HIGH) and open on separation (LOW). A HAL input, @task.driver('separation').
An IRQ on either edge wakes run(), which debounces, and on a confirmed separation during the Boosting
stage drives the documented Boosting -> Gliding transition (the booster ejects the glider at apogee).
The event is logged and emitted to subscribers; the discrete event is NOT a databoard quantity (per
doc/specs/coludo.md, events use notify/log).

The pin uses an internal pull-down so an open (separated) circuit reads LOW reliably; while nested the
pads override it HIGH. A separation while not Boosting (e.g. a ground test in Setting) is logged but
does not transition -- the guard keeps go/no-go correct.

This transition calls controller.set_stage() directly, NOT the sequencer's _advance(), so it does not
write a row to sequencer.csv. That is deliberate -- separation is the PRIMARY Boosting -> Gliding
trigger and separation.csv (event + stage, durable) is its authoritative telemetry record; the
sequencer's burnout-timeout is only the fallback, and sequencer.csv records that fallback path. A
post-flight tool reading the BOOSTING->GLIDING reason must consult separation.csv first. (GC policy is
unaffected: gc.disable() already fired on the SETTING->BOOSTING transition.)

### `class Separation(task.Task)`

Detect stage separation (HIGH=nested -> LOW=separated) and trigger Boosting -> Gliding.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: the separation pin reads a valid level (logged nested/separated).
- `diagnose() -> str` — Deeper analysis: read the separation pin directly.
- `separated() -> bool` — The last debounced latch level as a bool (True = pads open = separated).
- `inspect() -> dict`

## `sg90.py`

SG90 micro fin servo on a PWM pin. @task.driver('sg90'), one instance per fin (yaw / left eleron /
right eleron), each naming its 'pin'. 50 Hz frame; the command unit is INTEGER DEGREES, linearly mapped
to a pulse width (min_us..max_us over min_deg..max_deg, integer math) and CLAMPED to the range so a bad
command can never drive the horn past the linkage.

OPEN-LOOP -- NO POSITION FEEDBACK. A 3-wire SG90 (signal / V+ / GND) only RECEIVES a PWM command; the
signal pin is input-only on the servo and there is no wire back, so the board CANNOT read where the horn
actually is. Everything this driver reports (inspect()/telemetry 'angle', 'pulse_us') is the LAST
COMMANDED value it tracks in software -- what we asked for, NOT a measurement. A stalled, force-held or
jammed surface would still read the commanded target. inspect() carries 'feedback: None' to make that
explicit. (Real feedback would need a feedback servo, or tapping the internal pot to an ADC, or a
current-sense on the rail.) Separately, this MicroPython-P4 build's PWM duty_u16()/duty_ns() GETTERS are
broken (return a constant), so we cannot even read the commanded duty back from the peripheral -- the
driver only ever WRITES it and remembers what it set.

This class is SG90-specific on purpose. Other servos (MG90S, MG996R, ...) differ in pulse range and
behaviour and would be their own @task.driver -- a new drivers/<type>.py subclassing this or standalone
-- selected by the component's 'driver' field. The shared slew gate + degree->pulse math live here for
now; factor them into a servo base when a second type lands.

Two ways to command a fin:
  update {"angle": d} -- IMMEDIATE, ungated: the operator override (sync, returns at once).
  await move(d) -- GATED + settle-aware: passes through a SHARED slew gate so at most fins.concurrency
    (board config, default 3 = no limit) fins slew at once, then awaits the estimated travel so the
    caller knows it has (open-loop, no feedback) arrived. The flight control loop uses this.
Both record the command to per-fin telemetry (<name>.csv: angle, pulse_us, done) -- done=0 when a
command is ISSUED, done=1 when a move() has (estimated) COMPLETED. probe() is the on-demand self-test
(CC 'probe', pre-flight -- never at boot, so a reboot never sweeps fins): it sweeps the full range and
returns to neutral, logging each step.

Power: servos run off their own boost rail (per-pin diode protected); the board sources only the
low-current signal on the PWM pin, never the servo supply.

### `class SG90(task.Task)`

One PWM SG90 fin servo, commanded in integer degrees (clamped to [min_deg, max_deg]).

Each fin has a per-engine `trim` (degrees; config or live via update {"trim": d}): its MECHANICAL
zero offset, added to every command before the pulse map so boot / failsafe / control all land on
this fin's true centre -- physical install is never exact, so each engine is zeroed individually.

OPEN-LOOP -- reported angle is the last command, never a measurement (see module header; inspect
carries 'feedback: None'). update {"angle": d} moves it immediately; await move(d) moves it through
the shared slew gate; probe() sweeps it on demand.

- `travel_ms(degrees: int, settle: bool=True) -> int` — How long a move of `degrees` occupies this servo (ms) -- the ONE place that converts angle to time.
- `setup() -> bool`
- `run() -> None` — Command-driven: no run loop. move() / update() are the entry points.
- `probe() -> str` — On-demand self-test (CC 'probe', pre-flight -- never at boot): sweep min -> max -> neutral.
- `move(angle) -> int` — Drive to angle (clamped, integer degrees) through the shared slew gate.
- `update(props: dict) -> list` — Operator overrides, IMMEDIATE and ungated. {"angle": d} moves the servo (integer degrees,
- `set_angle(angle) -> int` — The 100 Hz flight-loop hot-path command (compare-and-set, no per-step dict).
- `settle() -> None` — Apply a held reversal once the horn has arrived (call from the same loop as set_angle).
- `finish() -> None` — Release the PWM (stop driving the pin) on shutdown.
- `diagnose() -> str` — Deeper analysis when setup() failed: is the pin PWM-capable?
- `inspect() -> dict`

## `vl53l4cx.py`

VL53L4CX time-of-flight laser ranger (Adafruit 5425) over the shared I2C bus: the above-ground-level
(AGL) channel for the last metres of the glide, where the barometer is useless.
@task.driver('vl53l4cx'). The VL53 family uses 16-BIT register addresses (i2cbus addrsize=16). This
part is the newer 0xEBAA silicon (shared by the VL53L4CD/L4CX), so it uses the VL53L4CD Ultra-Lite-
Driver init -- the older VL53L1X (0xEACC) config does NOT produce ranges on it.

setup(): optional XSHUT reset -> wait for boot -> write the default configuration -> run one VHV
calibration ranging cycle (start/wait/clear/stop, then the VHV config writes) -> start continuous
ranging. run(): wait for data-ready (the GPIO1 interrupt if wired, else a poll), read the distance and
write AGL (m) to the databoard. Single-target distance; the L4CX multi-target extras are unused.
Graceful: no I2C ack -> setup False -> Controller skips it. Shares i2c:0 via the locked i2cbus.

### `class Vl53l4cx(task.Task)`

Laser ToF: writes above-ground-level distance (m) to the databoard 'agl' slot.

For the final low-altitude metres where the barometer cannot resolve height. Interrupt-driven when
GPIO1 is wired.

- `setup() -> bool`
- `run() -> None` — The sampling loop: write AGL (m) to the databoard, forever.
- `probe() -> str` — On-demand self-test: the model id reads back.
- `diagnose() -> str` — Deeper analysis when setup() failed: classify the wire-level fault behind an absent ranger.
- `inspect() -> dict`

## `wifi.py`

Wi-Fi station driver: joins the configured network and keeps it joined, exposing signal/ip to the
operator. HAL (it drives the radio), so @task.driver('wifi'). STA only; SSID / CC host / TX power come
from the 'wifi' section of board.config, the password from <ssid>.creds (gitignored, deploy.sh-pushed).

Optional + telemetry-first + NON-BLOCKING BOOT: setup() never touches the radio (it only reads config),
because bringing the STA link up can block and would stall the serial boot -- so the board ALWAYS boots
and flies, with or without Wi-Fi. The radio comes up lazily in run(), which (re)joins on an interval
ONLY until ignition (after BOOSTING it idles, never competing with the flight loop). A board with no
Wi-Fi just logs once and flies standalone -- no Wi-Fi means no CC, nothing more.

### `class Wifi(task.Task)`

Join + maintain the STA link; Inspectable as 'wifi'.

- `setup() -> bool` — NON-BLOCKING: only read config; the radio is brought up lazily in run().
- `run() -> None` — (Re)join every retry_ms -- but ONLY on the ground.
- `connect(timeout_ms: int=15000) -> bool` — Join the configured network.
- `isconnected() -> bool`
- `ifconfig() -> tuple`
- `ip() -> str`
- `rssi() -> int`
- `set_tx_power(dbm: int) -> bool` — Adjust the TX power (operator signal-level tuning).
- `diagnose() -> str` — Dump the Wi-Fi link state to the console AND the recorder log; return the one-line summary.
- `inspect() -> dict`
- `update(props: dict) -> list`
- `stats() -> dict`

# glider subsystem tasks — `tasks/` — `src/glider/tasks`

## `attitude.py`

Attitude REDUNDANCY: a complementary-filter backup for the BNO055 (coludo.md "Sensors Fusion/Backup").
The BNO055 is the sole fused-attitude source; losing it mid-flight would leave the flight loop with
stale/absent attitude -> neutral fins -> ballistic. This task derives (heading, roll, pitch) from the
LSM6DSO32 gyro `rate` + accel gravity vector and PROVIDES it on the databoard at PRIORITY 1, so the
existing timeout-handoff fusion swaps to it automatically the moment the BNO055 (priority 0) stops -- no
change to flight.py.

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
- `run() -> None` — Mirror the primary while it is the fused source; free-run the filter when it is lost.
- `probe() -> str` — On-demand self-test: the gyro `rate` is present (the backup's core input).
- `inspect() -> dict`
- `stats() -> dict`

## `board_health.py`

Board vitals task: samples temperature, free memory and CPU load every period, pushes a telemetry row
(health.csv) and exposes the latest to the operator. Registered as @task.activity('health') so the
Controller creates and supervises it.

CPU load (an integer percent 0..100) is estimated WITHOUT busy-spinning: a probe task sleeps a fixed
period (probe_ms) and measures how LATE it actually wakes. asyncio.sleep_ms only resumes once the event
loop is free, so time other tasks spend running delays the wake-up -- the overshoot beyond the nominal
sleep is the time the CPU was busy with other work:
  load% = round(100 * (elapsed - probe_ms) / elapsed).
Sleeping rather than spinning on sleep_ms(0) lets the core actually idle between probes (FreeRTOS idle /
WFI) -- much lower idle power draw (the old spin pinned the CPU at ~100%). No calibration baseline is
needed (it is absolute). test_board_health drives a CPU hog and asserts the load rises with it.

### `class BoardHealth(task.Task)`

Periodic vitals -> telemetry (health.csv) + `inspect health`.

- `setup() -> bool`
- `temperature() -> float`
- `mem_free() -> int`
- `sample() -> dict`
- `oom_s()` — Predicted seconds to memory exhaustion from the current decay slope.
- `land_s()` — Predicted seconds until the glider sinks to the ground, from the elevation-decay slope.
- `run() -> None` — Push a vitals row at startup, then every period_ms.
- `probe() -> str` — On-demand self-test: free memory reads positive (a basic board-vitals sanity).
- `inspect() -> dict`
- `stats() -> dict`

## `cc_link.py`

The Control link task: once Wi-Fi is up it dials the CC hub and serves the command dispatcher,
reconnecting with backoff. @task.activity('cc'). Telemetry-first: with no Wi-Fi up it simply waits, so
the board flies fine without CC. The hub address is the configured `cc_host`, or -- when unset -- the
`.1` of whatever subnet the board joins (the Control hub by convention), so a board reaches its hub on
any network. An empty `cc_host` ('') disables CC entirely (standalone). The dispatcher is wired to this
board's config + Controller.

### `class ControlLink(task.Task)`

Serve the CC protocol to the hub when the link is available; never fatal.

With no `cc_host` configured the board dials the `.1` of whatever subnet it joins (the Control hub
by convention); an empty `cc_host` ('') disables CC and the board flies standalone.

- `setup() -> bool`
- `inspect() -> dict`
- `run() -> None` — Park until the Wi-Fi dependency is up, then dial CC and serve until the link drops; retry.
- `probe() -> str` — On-demand self-test: the CC hub address resolves (explicit or derived) and the Wi-Fi

## `field.py`

The CC-less field agent (doc/specs/coludo.md "Field operation without CC"). @task.activity('field'),
DISABLED by default. On the pad (SETTING) it makes at most two decisions:
  1. SITE BY GPS -- on the first fresh fix, the mission adopts the nearest launch.config site within
     max_range_m; none in range -> the synthesized spiral-landing fallback zone offset from the fix
     at the configured clear-sector bearing.
  2. AUTO-ARM (opt-in) -- arm once the board has sat STATIONARY with a live fix for the whole
     auto_arm_dwell_s. The long dwell makes a bench/carry arm unlikely, and the flight loop's
     control-stage gating still holds the fins neutral on the ground either way.
Each decision fires once, then the task idles; the operator/CC can still override everything live.

### `class Field(task.Task)`

Site-by-GPS + optional auto-arm, so a board can fly with no Control hub present.

- `setup() -> bool`
- `run() -> None`

## `flight.py`

Phase 3 stabilization loop. @task.activity('flight'). At `schedule_hz` it runs the control PIPELINE:
dt -> airspeed Governor (fin-authority cap, adaptively throttled) -> control-stage gate -> attitude ->
Guidance (per-stage setpoints + heading) -> PID per axis -> mixer actuate. The control LAW lives in
guidance.py and the airspeed/authority POLICY in governor.py -- this task is the orchestration: databoard
reads, arming/degraded gates, scheduling, and the PID->mixer->servo drive. Per-stage behaviour, the
GPS-degrading heading tiers, boost hold and final approach are guidance.py's; the adaptive estimator
throttle is governor.py's. Degraded: stale/absent attitude -> neutral. Disarmed / non-control stage ->
neutral.

Scheduling: schedule_hz > 0 -> a machine.Timer ticks the step, so the control law gets a regular slice
independent of what other asyncio tasks are doing (deterministic, e.g. while the laser hammers I2C in
landing). schedule_hz == 0 -> a plain asyncio loop at period_ms (reconfigure/debug; subject to the
~10 ms asyncio floor). Default 100 Hz timer = ~1 m per control step at 100 m/s. Gains default to 0 and
the task is disabled by default -- it cannot move a surface until enabled + tuned on the airframe.

### `class Flight(task.Task)`

Attitude-hold stabilization: GLIDING-gated, timer- or asyncio-scheduled, fail-safe to neutral.

- `setup() -> bool`
- `run() -> None` — finally covers BOTH exits -- a crash out of _tick (uncaught exception in a control stage) and an
- `finish() -> None`
- `progress() -> tuple` — The public control-loop heartbeat, so a supervisor need not read private attributes.
- `airspeed() -> float` — The fused airspeed estimate (m/s) -- what the checkpoint persists for a warm start.
- `seed_airspeed(airspeed: float) -> None` — Restore the airspeed a warm-start crumb carried (warmstart.py calls this after the gate passes).
- `vitals() -> dict` — The live flight-panel readout (CC dashboard).
- `inspect() -> dict`

## `gnss_calib.py`

GNSS consistent-drift calibration on the pad. @task.activity('gnss_calib').

A STATIONARY GNSS position walks slowly (changing satellite geometry, ionospheric delay, multipath).
Over the ~60 s a rocket sits on the pad the walk is roughly a constant DRIFT VELOCITY, and the almanac
barely changes through the ~60 s flight -- so the drift measured on the pad predicts the drift in the
air. A fixed antenna is NOT carried by the wind, so on the pad the receiver's whole reported ground
velocity IS its drift: we average it through SETTING and FREEZE it at launch. The flight loop then
subtracts it from the GNSS ground velocity before the wind triangle -- otherwise the drift folds
STRAIGHT into the wind estimate (wind = ground_velocity - airspeed*heading), reading as phantom wind.

Position-nav is deliberately NOT corrected: the drift over a 60 s flight is a few metres, inside the
~20 m turn-radius landing floor (doc/specs/coludo.md), so it would not move the touchdown -- the win is a
clean wind estimate. Slow loop (the drift is slow); Inspectable -> the operator sees the frozen drift.

### `class GnssCalib(task.Task)`

Average the reported ground velocity while stationary on the pad (SETTING) -> the GNSS drift, and
freeze it at launch (BOOSTING).

drift() hands it to the flight loop to de-bias the wind.

- `setup() -> bool`
- `run() -> None`
- `drift() -> tuple` — (east, north) m/s frozen drift velocity; (0, 0) until launch freezes it (or too few samples).
- `inspect() -> dict`

## `hitl.py`

Hardware-In-The-Loop flight simulator (Phase-5). @task.activity('hitl').

Closes the control loop ON THE BOARD without changing any production code: it reads the commanded fin
angles from the cached servo tasks, steps a flight-dynamics model (sim_model.Body), and PROVIDES the
resulting sensor quantities on the databoard at priority 0 -- so sequencer.py / flight.py / pid / mixer /
navigation read it and cannot tell it is simulated. The full chain runs closed-loop: sim sensors ->
sequencer (stage machine) -> flight (PID -> mixer -> fins) -> back into the model. Use with config_hitl
(real sensors off, this on, flight + sequencer enabled, watchdog off). The physics live in sim_model.py
(pure, shared with the host-side tools/virtual_flight.py -- same model, both worlds).

Fidelity: BOOST adds attitude under thrust -- a crosswind weathercocks the stack and the boost stage's
guarded fins fight to hold it vertical, on top of the vertical 1-DoF that drives launch detect + apogee;
the GLIDE is a rigid body with roll/pitch/yaw state driven by the elevon/rudder deflections the flight
loop commands (that is where the rest of control happens). Aero is simplified and the coefficients are
deliberately tunable -- the point is a stable, closed loop that exercises the control code, not
aerodynamic truth. Outputs are perturbed by a noise level N and optional 2x spikes to study
sensor-quality degradation (e.g. the laser dropping out beyond its range).

The simulated sensors are ALSO recorded as telemetry under the SAME csv names/fields as the real drivers
(accel_adxl375 / imu_bno055 / baro_icp10111 / gnss / laser_agl + a combined fins), so an on-board HITL
run produces a COMPLETE, renderable capture on the Luckfox (flight_report/flight_svg), not just
health/sequencer/servo. The records are decimated so the recorder link keeps up.

### `class Hitl(task.Task)`

The HITL simulator task: drive the model from the commanded fins and publish simulated sensors.

- `setup() -> bool`
- `run() -> None` — FIXED-TIMESTEP ACCUMULATOR. The sim must track the WALL clock, because the sequencer's stage
- `inspect() -> dict`

## `recorder.py`

The Recorder's task adapter. The data path itself is the top-level `recorder` singleton (used directly
by every module via recorder.Recorder.log/tlm); this thin @task.activity plugs it into the Controller's
task graph so the `recorder` component (its bus selects the UART) is created and supervised like any
other task. No 'uart_sink' abstraction -- the Recorder is it.

### `class RecorderTask(task.Task)`

Owns the Recorder's setup + drain loop and surfaces it to the operator.

Everything else keeps logging/telemetering through the global recorder.Recorder.

- `setup() -> bool`
- `run() -> None`
- `probe() -> str` — On-demand self-test: the Recorder rings are up and a probe log line writes through them.
- `inspect() -> dict`
- `stats() -> dict`
- `update(props) -> list`

## `sequencer.py`

Phase 3 flight-stage automation. @task.activity('sequencer'). Watches the databoard and drives the
guarded, forward-only stage machine that the control loop gates on:
SETTING -> BOOSTING : |accel| over launch_g sustained launch_ms (motor ignition), OR the baro climbing
                      past launch_alt_m off the pad (an independent, threshold-robust backup)
BOOSTING -> GLIDING : the separation switch (drivers/separation.py) is primary; else the baro APOGEE
                      detect (peak - apogee_drop_m, at the top of the arc, mass/motor-independent);
                      burnout timeout last
GLIDING -> LANDING : agl below land_agl_m (the laser sees the ground; elevation is the fallback)
LANDING -> done : |accel| ~1 g (stationary) sustained ground_ms (on the ground)
Each transition fires once (the stage check + reset-on-change is the guard), logs the reason and a
sequencer.csv telemetry marker. Thresholds are config; launch_g/launch_ms is exactly what the E16/F15
passive flights tune. One control-independent tick, so it runs on the passive flights too (stages
logged, no actuation -- the flight task stays disabled).

### `class Sequencer(task.Task)`

Drive the flight-stage machine from sensor signals (forward-only, guarded, logged).

- `setup() -> bool`
- `finish() -> None`
- `run() -> None`

## `watchdog.py`

Watchdog + heartbeat supervisor. @task.activity('watchdog'). Two layers:
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
- `kick() -> None` — Out-of-band feed for a caller about to LEGITIMATELY block the loop.
- `run() -> None`

# control (CPython) — `src/control`

## `board.py`

_Tested by `test/test_board.py`._

One connected Coludo board as seen by the hub: lockstep request/response over its socket
(doc/specs/cc-protocol.md). The per-board lock makes every exchange strictly sequential, so the
heartbeat and operator traffic to one board can never overlap. CPython 3.12, stdlib asyncio only.

### `class Board`

One connected board: lockstep request/response over its socket.

- `__init__(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, log=None)` — constructor
- `peer() -> str` _(property)_
- `exchange(line: str, timeout: float=EXCHANGE_TIMEOUT_S, quiet: bool=False) -> cc._Msg` — Send a ready board-facing line and return its parsed reply.
- `properties() -> dict` — The Control-side snapshot of this board: identity + the cached config/inspect/stats/health.
- `command(command: str, *args, timeout=EXCHANGE_TIMEOUT_S, quiet=False) -> cc._Msg` — Build `command args...` and exchange it.
- `identify() -> str`
- `inspect(name: str) -> dict`
- `close() -> None`

## `gps.py`

_Tested by `test/test_gps.py`._

Host-side GPS assist for the Control hub.

The flight board carries its own GNSS (ATGM336H); a GPS plugged into the Control host (e.g.
/dev/ttyUSB0) is an ASSIST, not the source of truth. Two jobs:
  1. tell the operator when a usable fix is available -- the ideal launch condition is a 3D fix
     with 4+ satellites (so the board's own cold start has a good almanac/position seed);
  2. hand a launch position to the board (operator `assist <board>` -> `update mission` +
     `set-config launch`, persisted in the board's launch.config) when the on-board GPS has no fix yet.

Pure NMEA parsing (GGA position/sats, GSA 2D/3D mode) is split from the serial transport so it is
unit-tested without hardware (test_gps.py); the Linux serial open + read loop is exercised by
itest_gps.py against a real receiver. CPython 3.12, stdlib asyncio only -- no pyserial.

### `class Fix`

The latest GNSS fix, accumulated from GGA (position/altitude/satellites) and GSA (2D/3D).

- `__init__()` — constructor
- `fix_3d() -> bool` _(property)_
- `has_position() -> bool` _(property)_
- `usable() -> bool` _(property)_ — The ideal launch condition: a 3D fix with enough satellites and an actual position.

### `class Gps`

Host GPS reader: feed NMEA lines, expose the latest fix + a launch position for board assist.

- `__init__(log=print)` — constructor
- `feed(line: str) -> bool` — Parse one NMEA sentence into the running fix.
- `status() -> dict` — Operator-facing fix snapshot: is it a usable 3D fix, how many satellites, where.
- `position()` — The host position as a mission dict, when the fix is usable.
- `run(reader: asyncio.StreamReader) -> None` — Feed every line from an NMEA stream until it ends (the read loop, transport-agnostic).
- `serve(device: str, baud: int=9600) -> None` — Open the serial GPS and feed it forever (the wired host-assist path).

### `open_serial(device: str, baud: int=9600) -> asyncio.StreamReader`

Open a Linux serial tty as an asyncio StreamReader: raw 8N1 at `baud`.

Stdlib only (termios + connect_read_pipe). Hardware path -- covered by itest_gps.py, not the host
unit tests.

Args:
    device - the serial device path.
    baud - the serial baud rate (default 9600).

Returns:
    An asyncio.StreamReader over the opened tty.

Raises:
    FileNotFoundError - the device cannot be opened.

## `main.py`

CLI entry point for the Control hub. Run it headless on a LAN box (it binds 0.0.0.0 by default) and
telnet / browse to it from another workstation, instead of opening a browser locally.

  python3 main.py [--host H] [--port N] [--operator-port N] [--web-port N]   (--help for all)

### `main() -> None`

## `server.py`

_Tested by `test/test_server.py`._

The Control hub: a board listener (1234) + per-board heartbeat + a telnet operator console (1235),
plus the web bridge (web.py, 8080). Boards dial in, Control learns each id via whoami/iam and owns
every exchange. An operator line whose first token is a board id or `all` routes to that board (id
stripped, the rest forwarded verbatim) and the reply is tagged `from <board> ...`; any other first
token is a Control command from the drop-in commands/ registry. CPython 3.12, stdlib asyncio only.
cc_protocol.py is shared with the firmware (symlinked).

### `class Server`

The hub: a board listener + per-board heartbeat + an operator console.

`on_board` is an optional async hook invoked once, right after a board identifies (used by
integration tests).

- `absent() -> list` — Gliders the roster knows that are not connected right now, with what to do about it.
- `__init__(host: str='0.0.0.0', port: int=1234, operator_port: int=1235, web_port: int=8080, on_board=None, log=print, heartbeat_s: float=HEARTBEAT_S, gps=None, roster_path: str=None)` — constructor
- `board_rows() -> list` — The registry as json-able rows.
- `cc_status() -> dict` — The Control host's own status for the dashboard header: the wall clock and the host GPS.
- `start_stream(client, interval_ms, kind='log') -> None` — (Re)start streaming a board's `kind` ('log'|'tlm') at `interval_ms`, replacing any running one.
- `stop_stream(board_id) -> None` — Stop streaming a board and tell it to stop collecting (a final `<kind> 0` drain).
- `serve_forever() -> None` — Accept board connections on `port` (board-facing listener).
- `serve_operators() -> None` — Accept operator connections on `operator_port` (telnet-friendly console).
- `run() -> None` — Run the board listener, operator console, and web bridge until cancelled.

## `web.py`

_Tested by `test/test_web.py`._

Web bridge -- the browser face of the Control hub (doc/specs/cc-protocol.md "Browser bridge").

A minimal HTTP/1.1 + SSE server on 8080 over the same stdlib asyncio loop as the board listener
and operator console (no extra dependency, no framework). Plain HTTP: the LAN is trusted and
encryption is out of scope (cc-protocol.md "Transport & ports"). Routes:
  GET  /             -> the one-page dashboard (static/index.html)
  GET  /api/boards   -> hub.board_rows() as JSON (same data as the `list` command)
  GET  /api/absent   -> known-but-disconnected gliders from the roster, each with a reboot hint
  POST /api/cmd      -> {board, command, params} -> run it on the board, reply as JSON
  POST /api/op       -> {line} -> run an operator-console line (calibrate, ...) -> {lines}
  GET  /events       -> Server-Sent Events: the board list pushed every heartbeat (live table)

### `class Web`

The HTTP/SSE server. Holds the hub for the board registry + routing; one per hub.

- `__init__(hub, host: str='0.0.0.0', port: int=8080, log=print)` — constructor
- `serve() -> None`

# control operator commands — `commands/` — `src/control/commands`

## `assist.py`

`assist <board>` -- push the host GPS position to a board's mission (sync the launch site), then
persist it to the board's launch.config. Only sends a usable 3D fix; defaults to the selected
board. Requires a GPS attached to the Control host (main.py --gps-device).

### `assist_command(hub, tokens, session) -> list`

## `bustune.py`

`bustune <board> <i2c|spi> <id> [margin-steps]` -- find a sensor bus's max stable frequency.

NAMED FOR THE PRIMITIVE IT DRIVES. It was `calibrate`, which collided with the board's own
`calibrate` command (sensor calibration -- a pitot tare, an IMU figure-8): the same verb meant
"tune a bus" at the operator console and "calibrate a sensor" on the dashboard, decided only by which
surface you typed it on. Nothing about the behaviour changed with the rename.

Drives the board's `bustune` primitive (retune-in-place + per-device health) UP a frequency ladder,
stopping at the first step any device fails. Reports the ceiling (highest all-healthy step), the
LIMITING device (first to drop out -> the one to rewire / move off the shared bus), and a `chosen`
freq backed off `margin` ladder steps for headroom (default 1 -- the MAX-1 rule). Restores the bus
to its configured freq afterwards; it does NOT persist. To apply, the operator runs the printed
`set-config board ... + reboot` (the immutable-config activation path). The sweep lives here on CC,
the board only executes one retune-and-test step at a time.

### `bustune_command(hub, tokens, session) -> list`

## `cache.py`

`cache <board>` -- the Control-side cached properties for a board (config / inspect / stats /
health), last-known values without touching the board. Defaults to the session's selected board.

### `cache_command(hub, tokens, session) -> list`

## `gps.py`

`gps` -- the host GPS fix status (3D + satellites), so the operator knows when the launch site has a
usable position. `gps <board>` also fetches that board's on-board GNSS (`inspect gnss`) and shows it
beside the host fix, to check what the on-board receiver delivers against the USB reference before
trusting it / using `assist`. Requires a GPS attached to the Control host (main.py --gps-device).

### `gps_command(hub, tokens, session) -> list`

## `help.py`

`help` -- list operator commands, or `help <command>` for one.

### `help_command(hub, tokens, session) -> list`

## `list.py`

`list` -- the connected boards and their last-known status.

### `list_command(hub, tokens, session) -> list`

## `select.py`

`select <board>` -- set this session's sticky target; a later bare command routes to it.

### `select_command(hub, tokens, session) -> list`

## `who.py`

`who` -- show this session's currently selected board.

### `who_command(hub, tokens, session) -> list`
