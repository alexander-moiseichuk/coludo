# Coludo API reference

_Generated from module docstrings by `tools/gen_docs.py` — do not edit by hand; run `python3 tools/gen_docs.py` to regenerate._

# glider firmware (MicroPython) — `src/glider`

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

### `create_dispatcher(cfg: dict, controller=None, on_reboot=None, config_path: str='board.json') -> Dispatcher`

Build a Dispatcher with the standard command handlers, wired to the running config, the
Inspector, and (optionally) the Controller. `on_reboot` lets tests intercept the reset.

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

## `config.py`

_Tested by `test/test_config.py`._

Board configuration loader / validator — the Phase 0 foundation.

Implements the three-layer model from specs/board-config.md:
config_default.py  (firmware default / fallback)
board.json         (saved active config, a full snapshot)
in-memory dict     (validated, what the Controller builds tasks from)

Runs on MicroPython on the board. Validation here is config-file *integrity* (structure,
types, pin uniqueness, bus refs, reserved pins) — NOT hardware health, which is checked at
runtime and surfaced to the operator (the strict model).

### `validate(cfg) -> list`

Return a list of human-readable error strings (empty list == valid).

### `config_id(cfg) -> str`

A stable short hash identifying a config snapshot (for the CC iam/config_id).

### `load(path: str='board.json', defaults=None) -> tuple`

Layered load: active board.json if present and valid, else defaults.

Returns (cfg, source, errors). `source` is 'active', 'default', or a fallback reason.
Never raises — a missing/corrupt/invalid active file degrades to defaults so the board is
always reachable.

### `save(cfg, path: str='board.json') -> str`

Validate then atomically persist a full config snapshot. Returns its config_id.

Raises ValueError if invalid (an invalid config is never written).

### `reset(path: str='board.json') -> bool`

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

Human-edited firmware default and the safe fallback when no valid board.json exists (see
specs/board-config.md). Pins come from doc/waveshare_esp32p4_pins.md (validated on hardware by
test/test_pins.py). `default()` returns a FRESH dict each call so callers may mutate it freely.

Topology: buses are grouped by type then id; a sensor/component addresses one by `bus` (the kind,
e.g. 'i2c') + `id` (its int id), so nothing parses a 'type:id' string. `sensors` are data
providers fused by quantity + priority (several may provide the same quantity with different
drivers/priorities); `components` are the consumers/actuators (recorder, ...).

### `default() -> dict`

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
`STAGES` id->name mapping (operator-facing names; `in Stage.STAGES` is an O(1) key check).


### `class Controller(inspector.Inspectable)`

- `__init__(config: dict, registry: dict=None, log=None)` — constructor
- `directory() -> list` — Names of enabled devices, in creation order (config order).
- `create(name: str) -> task.Task` — Create a task by component name via the registry. A component names its implementation
- `active(name: str=None)` — Return the active task by name (None if absent), or a list of all active tasks if
- `find(names: list[str]) -> list` — Non-blocking: the active tasks for `names`, None for any not up. The fast lookup for
- `query(names: list[str], waiting: bool=True) -> list` — Look up sibling tasks by name from the registry: `gnss, baro = await self.query(['gnss',
- `setup() -> bool` — Create + set up every enabled task in order. Skip (and report) failures. setup() brings a
- `start() -> None` — Launch each task's run() loop as a supervised asyncio task.
- `close(name: str) -> None` — Deactivate a task and clean up its resources.
- `finish() -> None` — Shut down all tasks.
- `set_stage(stage: int) -> None`
- `stage_name() -> str` — The current flight stage as its operator-facing name.
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
- `read() -> tuple` — (value, source, age_ms) of the fused estimate; `source` is None when extrapolated. A
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
inspect mission                              -> launch id / site / position + the board clock
update mission base64:{"launch_id":"t1"}     -> set the launch id for this flight
update mission base64:{"epoch":1750170000}   -> set the board RTC (time sync; Unix seconds)
save-mission                                 -> persist the live mission back to launch.config

Position is metres / decimal degrees; it is a known origin now and seeds the GNSS driver later.

### `class Mission(inspector.Inspectable)`

The operator-set launch identity. One per board; registers itself so Control can
`inspect`/`update mission`. Seeded from launch.config at construction.

- `__init__(path: str=LAUNCH_PATH)` — constructor
- `set_time(epoch) -> bool` — Set the board RTC from a Unix epoch (seconds, UTC). Returns True if applied.
- `clock() -> str` — Current board wall-clock as 'YYYY-MM-DDTHH:MM:SS' (from the RTC).
- `epoch() -> int` — Current board clock as a Unix epoch (seconds), for Control to compare against its own.
- `inspect() -> dict`
- `update(props: dict) -> list` — Apply launch_id/site/latitude/longitude/altitude (stored, range-checked) and `epoch`
- `save() -> None` — Persist the stored mission fields to launch.config (atomic temp+rename) so the launch

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

`decimate_us` rate-limits the stream: with it 0 every push() emits; with it set, push() emits
only when at least `decimate_us` microseconds have passed since the last emitted row (a fast
sensor can push every sample and have its telemetry decimated to a sane rate).

- `__init__(filename: str, fields: tuple, decimate_us: int=0)` — constructor
- `push(values) -> None`

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

### `get(bus_id: int, spec: dict) -> Bus`

The shared Bus for `bus_id`, created once from `spec` (sck/mosi/miso/baud/mode) and cached.

## `task.py`

Task base class and driver registry — the unit the Controller creates and supervises.

Every component/system task follows the common lifecycle from specs/coludo.md:
setup()    async; initialize or reset; return True on success
probe()    async; ON-DEMAND self-test (the CC `probe` command, never at boot) -> None if healthy,
else an error string. Default None; a sensor reports 'X not found on i2c:0', an actuator
exercises itself (the servo sweeps its range) -- so a mid-flight reboot never sweeps fins.
run()      async; the task's main activity loop
notify()   subscribe a callback for this task's updates
validate() return True if the task is currently healthy
finish()   async; shut down and release resources
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
- `setup() -> bool` — Initialize or reset. Override. Return True on success, False otherwise.
- `probe() -> str` — On-demand self-test (the CC `probe` command, NOT run at boot): return None when healthy, or
- `run() -> None` — Main activity loop. Override. Default returns immediately.
- `notify(callback) -> None` — Register callback(task, event) to be invoked on this task's updates.
- `emit(event=None) -> None` — Notify all subscribers of an update.
- `find(names: list[str]) -> list` — Non-blocking sibling lookup via the Controller (None for any not up).
- `query(names: list[str], waiting: bool=True) -> list` — Await sibling tasks by name via the Controller; with `waiting` (default) park until all
- `validate() -> bool` — Return True if the task is currently healthy.
- `finish() -> None` — Shut down and release resources.
- `inspect() -> dict` — Status dict. Subclasses extend it.

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
- `inspect() -> dict`

## `atgm336h.py`

drivers/atgm336h.py — ATGM336H GNSS (GPS + BDS) over UART: the position channel. @task.driver(
'atgm336h'). At setup it reconfigures the module to RMC-only at the configured rate (default 10 Hz)
via PMTK commands -- RMC alone (~70 B) fits 10 Hz inside 9600 baud (~960 B/s), so no baud switch is
needed. run() reads NMEA lines asynchronously (asyncio.StreamReader), parses RMC for the fix and
writes (latitude, longitude) to the databoard 'position' slot; a GGA sentence, if the module still
emits one, supplies altitude (a deep fallback to the baro). Lock is lost easily under high-g, so
position is best-effort and the consumers fall back when it goes stale.

NMEA is talker-agnostic (GP/GN/BD). The UART is dedicated (uart:2), not a shared bus, so the driver
owns the peripheral. Graceful: an undefined bus -> setup False -> the Controller skips it.

### `class Atgm336h(task.Task)`

GNSS: reconfigures to RMC-only at `hz`, then writes (latitude, longitude) to 'position' (and
altitude to 'altitude' if a GGA is seen). Best-effort -- lock can drop under boost.

- `setup() -> bool`
- `run() -> None` — Read NMEA lines forever and parse them; non-ASCII noise lines and malformed fields are
- `inspect() -> dict`

## `bluetooth.py`

drivers/bluetooth.py — set the BLE radio to the state declared in config at boot. The component
field `radio` (true/false, default false) says whether Bluetooth should be ON; the driver applies
it -- transparent, so nobody is surprised by an implicit disable. Default false saves power (the
wireless is the external C6 and BLE is unused on the glider). Setup-only @task.driver('bluetooth')
plus update() so the operator can toggle it live (`update bluetooth {"radio": true}`).

### `class Bluetooth(task.Task)`

Apply the configured BLE radio state. Inspectable: `radio` requested, `active` actual.

- `setup() -> bool`
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
- `sample() -> tuple` — Read the block and return (attitude (heading, roll, pitch) deg, accel (x, y, z) g).
- `run() -> None`
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
- `inspect() -> dict`

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

### `class Separation(task.Task)`

Detect stage separation (HIGH=nested -> LOW=separated) and trigger Boosting -> Gliding.

- `setup() -> bool`
- `run() -> None`
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
update {"angle": d}  -- IMMEDIATE, ungated: the operator override (sync, returns at once).
await move(d)        -- GATED + settle-aware: passes through a SHARED slew gate so at most
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
- `probe() -> str` — On-demand self-test (CC `probe`, pre-flight -- never at boot): sweep min -> max -> neutral so
- `move(angle) -> int` — Drive to `angle` (clamped, integer degrees) through the shared slew gate -- at most
- `update(props: dict) -> list` — `{"angle": d}` moves the servo IMMEDIATELY (integer degrees, clamped) -- the operator
- `finish() -> None` — Release the PWM (stop driving the pin) on shutdown.
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
- `inspect() -> dict`

## `wifi.py`

drivers/wifi.py — Wi-Fi station driver: joins the configured network and keeps it joined, exposing
signal/ip to the operator. HAL (it drives the radio), so @task.driver('wifi'). STA only; SSID / CC
host / TX power come from the `wifi` section of board.json, the password from <ssid>.creds
(gitignored, deploy.sh-pushed).

Optional + telemetry-first: `network` is imported in setup() so the module still loads on a board
with no Wi-Fi (e.g. the FireBeetle 2); setup() then returns False, the Controller skips the task,
and the board runs everything else without CC. run() is a maintain loop -- a failed join is never
fatal, it just retries.

### `class Wifi(task.Task)`

Join + maintain the STA link; Inspectable as `wifi`.

- `setup() -> bool`
- `run() -> None` — Keep the link up: (re)join whenever disconnected. Never fatal -- the board flies without
- `connect(timeout_ms: int=15000) -> bool` — Join the configured network. Returns True once connected, False on timeout/error.
- `isconnected() -> bool`
- `ifconfig() -> tuple`
- `ip() -> str`
- `rssi() -> int`
- `set_tx_power(dbm: int) -> bool` — Adjust the TX power (operator signal-level tuning). Returns True on success.
- `inspect() -> dict`
- `update(props: dict) -> list`
- `stats() -> dict`

# glider subsystem tasks — `tasks/` — `src/glider/tasks`

## `board_health.py`

tasks/board_health.py — board vitals task: samples temperature, free memory and CPU load every
period, pushes a telemetry row (health.csv) and exposes the latest to the operator. Registered as
@task.activity('health') so the Controller creates and supervises it.

CPU load (an integer percent 0..100) is estimated from a low-priority idle task that increments a
counter and yields (sleep_ms(0)) in a tight loop. Each period we measure the idle counter's RATE
(counts/ms); the highest rate ever observed (`_max_rate`) is taken as the fully-idle baseline, and
load% = round(100 * (1 - rate / _max_rate)).
So load is RELATIVE to the busiest-idle moment seen: it self-calibrates as the board gets idle
time (the baseline only rises), but a board that is never truly idle reads relative to its
least-busy moment. test_board_health drives a CPU hog and asserts the load rises with real load.

### `class BoardHealth(task.Task)`

Periodic vitals -> telemetry (health.csv) + `inspect health`.

- `setup() -> bool`
- `temperature() -> float`
- `mem_free() -> int`
- `sample() -> dict`
- `run() -> None` — Push a vitals row at startup, then every period_ms estimate load and push again. Runs
- `inspect() -> dict`
- `stats() -> dict`

## `cc_link.py`

tasks/cc_link.py — the Control link task: once Wi-Fi is up it dials the CC hub and serves the
command dispatcher, reconnecting with backoff. @task.activity('cc'). Optional + telemetry-first:
with no `cc_host` configured setup() skips it; with no Wi-Fi up it simply waits, so the board
flies fine without CC. The dispatcher is wired to this board's config + Controller (cc_client.py).

### `class ControlLink(task.Task)`

Serve the CC protocol to the hub when the link is available; never fatal.

- `setup() -> bool`
- `run() -> None` — Park until the Wi-Fi dependency is up, then dial CC and serve until the link drops; retry.

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
- `inspect() -> dict`
- `stats() -> dict`
- `update(props) -> list`

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
- `exchange(line: str, timeout: float=EXCHANGE_TIMEOUT_S) -> cc._Msg` — Send a ready board-facing line and return its parsed reply (None if disconnected).
- `properties() -> dict` — The Control-side snapshot of this board: identity + the cached config/inspect/stats/health
- `command(command: str, *args, timeout=EXCHANGE_TIMEOUT_S) -> cc._Msg` — Build `command args...` and exchange it. Returns the parsed reply or None.
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
`save-mission`, persisted in the board's launch.config) when the on-board GPS has no fix yet.

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
- `serve(device: str, baud: int=9600) -> None` — Open the serial GPS and feed it forever (the wired host-assist path).

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
- `board_rows() -> list` — The registry as json-able rows (id, online, last-known stage/config_id) — shared by the
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

## `gps.py`

`gps` — the host GPS fix status (3D + satellites), so the operator knows when the launch site has
a usable position. Requires a GPS attached to the Control host (main.py --gps-device).

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
