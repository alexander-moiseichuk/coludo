# Coludo API reference

_Generated from module docstrings by `tools/gen_docs.py` — do not edit by hand; run `python3 tools/gen_docs.py` to regenerate._

# glider firmware (MicroPython) — `src/glider`

## `board_health.py`

_Tested by `test/test_board_health.py`._

BoardHealth — periodic device vitals (temperature, free memory, CPU load) pushed to telemetry
and exposed to the operator via the Inspector (findings.txt #10). CPU load is estimated from a
low-priority idle task: the fewer times it runs in a period (vs the most it has ever run), the
busier the board.

### `class BoardHealth(inspector.Inspectable)`

- `__init__(period_ms: int=1000)` — constructor
- `temperature()`
- `mem_free() -> int`
- `sample() -> dict`
- `run() -> None` — Sample vitals every period_ms, estimate load, and push a telemetry row. Runs forever.
- `inspect() -> dict`
- `stats() -> dict`

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
- `on(command, fn)`
- `handle(line)`

### `class Client`

- `__init__(config, dispatcher, log=None, backoff_ms=1000)` — constructor
- `run()` — Connect to Control and serve forever, reconnecting with backoff on drop.
- `serve(reader, writer)` — Read commands from Control, dispatch, write responses. Returns on disconnect.

### `create_dispatcher(cfg, controller=None, on_reboot=None, fw='0.1', config_path='board.json')`

Build a Dispatcher with the standard command handlers, wired to the running config, the
Inspector, and (optionally) the Controller. `on_reboot` lets tests intercept the reset.

## `cc_protocol.py`

_Tested by `test/test_cc_protocol.py`._

CC <-> board line protocol (specs/cc-protocol.md).

One newline-delimited message per line:  <command> <board-id> [params...]
Tokens are whitespace-separated, so there is NO quoting or escaping. A param value is one of:
* bare token    -> a simple value with no spaces (e.g. 3000, glider1, 192.168.10.1)
* base64:<data> -> anything else: spaces, quotes, JSON, binary
Both sides know each command's schema, so the parser does not guess types: a bare token is
returned as a str and the receiver converts numerics itself (it knows `ms` is an int). Named
params are key=value; everything else is positional. The command is lowercased; values keep
their case. parse() handles requests and responses (ok/err/pong/iam) alike.

### `encode(v)`

Encode a value into one whitespace-free wire token.

### `decode(tok)`

Decode a wire token back to a str (base64-decoded if prefixed, else as-is).

### `parse(line)`

Parse a protocol line into a _Msg (works for requests and responses).

### `build(command, args=(), named=None)`

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

### `validate(cfg)`

Return a list of human-readable error strings (empty list == valid).

### `config_id(cfg)`

A stable short hash identifying a config snapshot (for the CC iam/config_id).

### `load(path='board.json', defaults=None)`

Layered load: active board.json if present and valid, else defaults.

Returns (cfg, source, errors). `source` is 'active', 'default', or a fallback reason.
Never raises — a missing/corrupt/invalid active file degrades to defaults so the board is
always reachable.

### `save(cfg, path='board.json')`

Validate then atomically persist a full config snapshot. Returns its config_id.

Raises ValueError if invalid (an invalid config is never written).

### `reset(path='board.json')`

Delete the active config so the next load uses defaults. Returns True if removed.

### `bus(cfg, ref)`

Resolve a bus reference 'type:id' (e.g. 'uart:1', 'i2c:0') to its spec dict, or None.

### `device(cfg, name=None, driver=None)`

Find a sensor/component by name or driver. Returns the dict or None.

## `config_default.py`

Baked-in default board configuration for the WaveShare ESP32-P4-WIFI6 controller.

Human-edited firmware default and the safe fallback when no valid board.json exists (see
specs/board-config.md). Pins come from doc/waveshare_esp32p4_pins.md (validated on hardware by
test/test_pins.py). `default()` returns a FRESH dict each call so callers may mutate it freely.

Topology: buses are grouped by type then id (referenced as 'uart:1', 'i2c:0', ...). `sensors`
are data providers fused by quantity + priority (several may provide the same quantity with
different drivers/priorities); `components` are the consumers/actuators (recorder, ...).

### `default()`

## `controller.py`

_Tested by `test/test_controller.py`._

Flight Controller — creates and supervises the tasks described by a validated config, and
tracks the flight state machine. See specs/coludo.md ('Flight Controller', 'Tasks').

The Controller is the one task created explicitly; it creates the rest from config in a
deterministic order. Task failures are reported, not fatal (the strict/operator-authority
model): a component that fails setup is logged and skipped, and go/no-go stays with the
operator via report()/validate().

### `class Controller(inspector.Inspectable)`

- `__init__(config, registry=None, log=None)` — constructor
- `directory()` — Names of enabled devices, in creation order (config order).
- `create(name)` — Create a task by component name via the driver registry. Returns task or None.
- `active(name=None)` — Return the active task by name, or a list of all active tasks if name is None.
- `setup()` — Create + set up every enabled task in order. Skip (and report) failures.
- `start()` — Launch each task's run() loop as a supervised asyncio task.
- `close(name)` — Deactivate a task and clean up its resources.
- `finish()` — Shut down all tasks.
- `set_state(state)`
- `validate()` — True if every active task is healthy.
- `inspect()`
- `stats()`

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

- `__init__(filename: str, fields: tuple)` — constructor
- `push(values) -> None`

## `task.py`

Task base class and driver registry — the unit the Controller creates and supervises.

Every component/system task follows the common lifecycle from specs/coludo.md:
setup()    async; initialize or reset; return True on success
run()      async; the task's main activity loop
notify()   subscribe a callback for this task's updates
validate() return True if the task is currently healthy
finish()   async; shut down and release resources
A Task is Inspectable: inspect()/update()/stats() expose it to the operator (the Controller
registers each task with the Inspector), so there is no separate report().

A driver registers itself with @driver('name'); the Controller maps a component's 'driver'
field to the class via DRIVERS.

### `driver(name)`

Class decorator: register a Task subclass under a driver name.

### `class Task(inspector.Inspectable)`

- `__init__(name, config=None, controller=None)` — constructor
- `setup()` — Initialize or reset. Override. Return True on success, False otherwise.
- `run()` — Main activity loop. Override. Default returns immediately.
- `notify(callback)` — Register callback(task, event) to be invoked on this task's updates.
- `emit(event=None)` — Notify all subscribers of an update.
- `validate()` — Return True if the task is currently healthy.
- `finish()` — Shut down and release resources.
- `inspect()` — Status dict. Subclasses extend it.

## `wifi.py`

_Tested by `test/test_wifi.py`._

Wi-Fi station — joins the Control Center's network as a client (specs/board-config.md,
cc-protocol.md). STA only; the board never hosts an AP. SSID, CC host/port and the tunable TX
power come from the `wifi` section of board.json; the password comes from <ssid>.creds (pushed
by deploy.sh, never committed) so it is not in the repo.

### `class Wifi(inspector.Inspectable)`

- `__init__(config: dict, log=None)` — constructor
- `connect(timeout_ms: int=15000) -> bool` — Join the configured network. Returns True once connected, False on timeout.
- `isconnected() -> bool`
- `ifconfig()`
- `ip() -> str`
- `rssi()`
- `set_tx_power(dbm: int) -> bool` — Adjust the TX power (operator signal-level tuning). Returns True on success.
- `inspect() -> dict`
- `update(props: dict) -> list`
- `stats() -> dict`

# control (CPython) — `src/control`

## `control.py`

_Tested by `test/test_control.py`._

Control — host-side ground station for the Coludo boards (specs/cc-protocol.md). Board-first:
boards dial in, Control learns each board's id via whoami/iam, and drives commands over the
board socket (which sees `command params`, no id; only `iam` carries the id). CPython 3.12,
stdlib asyncio only. cc_protocol.py is shared with the firmware (symlinked).

### `class Board`

One connected board: lockstep request/response over its socket.

- `__init__(reader: asyncio.StreamReader, writer: asyncio.StreamWriter)` — constructor
- `peer() -> str` _(property)_
- `command(command: str, *args, timeout=5.0)` — Send `command args...` and return its parsed response. The per-board lock makes calls
- `identify() -> str`
- `inspect(name: str) -> dict`
- `close() -> None`

### `class Server`

- `__init__(host: str='0.0.0.0', port: int=1234, on_board=None, log=print)` — constructor
- `serve_forever() -> None`
