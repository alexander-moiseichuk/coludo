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
* bare token    -> a simple value with no spaces (e.g. 3000, glider1, 192.168.10.1)
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

### `bus(cfg, ref) -> dict`

Resolve a bus reference 'type:id' (e.g. 'uart:1', 'i2c:0') to its spec dict, or None.

### `device(cfg, name=None, driver=None) -> dict`

Find a sensor/component by `name` and/or implementation. `driver` matches the resolved
implementation -- a component's `driver` (drivers/) or `activity` (tasks/) field. Returns the
dict or None.

## `config_default.py`

Baked-in default board configuration for the WaveShare ESP32-P4-WIFI6 controller.

Human-edited firmware default and the safe fallback when no valid board.json exists (see
specs/board-config.md). Pins come from doc/waveshare_esp32p4_pins.md (validated on hardware by
test/test_pins.py). `default()` returns a FRESH dict each call so callers may mutate it freely.

Topology: buses are grouped by type then id (referenced as 'uart:1', 'i2c:0', ...). `sensors`
are data providers fused by quantity + priority (several may provide the same quantity with
different drivers/priorities); `components` are the consumers/actuators (recorder, ...).

### `default() -> dict`

## `controller.py`

_Tested by `test/test_controller.py`._

Flight Controller — creates and supervises the tasks described by a validated config, and
tracks the flight stage machine. See specs/coludo.md ('Flight Controller', 'Tasks').

The Controller is the one task created explicitly; it creates the rest from config in a
deterministic order. Task failures are reported, not fatal (the strict/operator-authority
model): a component that fails setup is logged and skipped, and go/no-go stays with the
operator via stats()/validate().

### `class Controller(inspector.Inspectable)`

- `__init__(config: dict, registry: dict=None, log=None)` — constructor
- `directory() -> list` — Names of enabled devices, in creation order (config order).
- `create(name: str) -> task.Task` — Create a task by component name via the registry. A component names its implementation
- `active(name: str=None)` — Return the active task by name (None if absent), or a list of all active tasks if
- `find(names: list[str]) -> list` — Non-blocking: the active tasks for `names`, None for any not up. The fast lookup for
- `query(names: list[str], waiting: bool=True) -> list` — Look up sibling tasks by name from the registry: `gnss, baro = await self.query(['gnss',
- `setup() -> bool` — Create + set up every enabled task in order. Skip (and report) failures.
- `start() -> None` — Launch each task's run() loop as a supervised asyncio task.
- `close(name: str) -> None` — Deactivate a task and clean up its resources.
- `finish() -> None` — Shut down all tasks.
- `set_stage(stage: int) -> None`
- `stage_name() -> str` — The current flight stage as its operator-facing name.
- `validate() -> bool` — True if every active task is healthy.
- `inspect() -> dict`
- `stats() -> dict`

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

A task registers itself with @activity('name') (or its alias @driver('name') for the HAL ones in
drivers/); the Controller maps a component's 'driver' field to the class via ACTIVITIES. The two
names share one registry for now -- splitting drivers out is a later concern if it is needed.

### `activity(name: str)`

Class decorator: register a Task subclass (a HAL driver or a higher-level activity) under a
name so the Controller can build it from a config component.

### `class Task(inspector.Inspectable)`

- `__init__(name: str, config: dict=None, controller=None)` — constructor
- `setup() -> bool` — Initialize or reset. Override. Return True on success, False otherwise.
- `run() -> None` — Main activity loop. Override. Default returns immediately.
- `notify(callback) -> None` — Register callback(task, event) to be invoked on this task's updates.
- `emit(event=None) -> None` — Notify all subscribers of an update.
- `find(names: list[str]) -> list` — Non-blocking sibling lookup via the Controller (None for any not up).
- `query(names: list[str], waiting: bool=True) -> list` — Await sibling tasks by name via the Controller; with `waiting` (default) park until all
- `validate() -> bool` — Return True if the task is currently healthy.
- `finish() -> None` — Shut down and release resources.
- `inspect() -> dict` — Status dict. Subclasses extend it.

# glider HAL drivers — `drivers/` — `src/glider/drivers`

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
period, pushes a telemetry row, and exposes the latest to the operator. CPU load is estimated from
a low-priority idle task: the fewer times it runs in a period (vs the most it ever has), the busier
the board. Registered as @task.activity('health') so the Controller creates and supervises it.

### `class BoardHealth(task.Task)`

Periodic vitals -> telemetry (health.csv) + `inspect health`.

- `setup() -> bool`
- `temperature() -> float`
- `mem_free() -> int`
- `sample() -> dict`
- `run() -> None` — Sample vitals every period_ms, estimate load, and push a telemetry row. Runs forever.
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

- `__init__(reader: asyncio.StreamReader, writer: asyncio.StreamWriter)` — constructor
- `peer() -> str` _(property)_
- `exchange(line: str, timeout: float=EXCHANGE_TIMEOUT_S) -> cc._Msg` — Send a ready board-facing line and return its parsed reply (None if disconnected).
- `command(command: str, *args, timeout=EXCHANGE_TIMEOUT_S) -> cc._Msg` — Build `command args...` and exchange it. Returns the parsed reply or None.
- `identify() -> str`
- `inspect(name: str) -> dict`
- `close() -> None`

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

- `__init__(host: str='0.0.0.0', port: int=1234, operator_port: int=1235, web_port: int=8080, on_board=None, log=print, heartbeat_s: float=HEARTBEAT_S)` — constructor
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
