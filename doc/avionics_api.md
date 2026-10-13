# Coludo avionics API reference

_Generated from the Rust sources' doc comments by `tools/gen_avionics_docs.py` — do not edit by hand; run `python3 tools/gen_avionics_docs.py` to regenerate._

The Rust flight firmware (`src/avionics`, [specs/avionics-port.md](specs/avionics-port.md)): one chapter per crate, a crate after the crates it uses. Plan B (MicroPython) and the control hub are in [`glider_api.md`](glider_api.md).

# `board_config` — `src/avionics/crates/board_config`

_Uses: no workspace crate. External: serde_json, sha2._

_Integration tests: `tests/golden.rs`, `tests/views.rs`._

## `board_config`

_Tested by its own `mod tests`._

The board config (src/glider config.py): the document a board is built from, and its rules. The hardware
map (buses, pins, sensors, components), the board identity and the Wi-Fi, stable across flights, unlike
the per-launch mission. This crate is the document with no storage: the firmware's config store feeds
[`load`] the stored text and persists [`write()`]'s.

* [`canon`] and [`config_id`]: the identity CC announces, sha256 over Python's repr of the document.
* [`validate`]: config.validate, the same messages in the same order, plus one rule of the port's own: a
  device's `debounce_ms` is at most [`LONGEST_DEBOUNCE_MS`].
* [`load`]: which document a boot runs, and the source text whoami reports.
* [`write()`]: the canonical pretty text (`json.dumps(indent=1, sort_keys=True)` plus a newline).
* A typed view of the few keys the firmware reads so far: [`board_id`], [`mcu`], [`wifi`],
  [`max_range_m`], [`camera`]; and for the device bring-up (M5) the buses ([`i2c_bus`], [`spi_bus`]), the
  device [`directory`] with each entry's [`pin`]s and [`provides`], and the [`setup_retries`]; and for the
  supervisor (M8) the [`checkpoint`] cadence, the [`warm_start`] toggle and the [`watchdog`] settings.
* [`python`]: Python's `==`, `%s` and truthiness over config values, for the ports that read a config as
  plan B does.

Python semantics are CPython 3.12's, the golden oracle (src/avionics/golden), including the float repr
and the str repr's escapes. A document is a `serde_json::Value`, whose numbers are f64 or integers: the
f64s are kept for the hash and the bytes, and never computed with (D18).

Where serde_json reads a text differently from CPython's json (both refuse everything else alike):
 * `-0` is the float -0.0 here and the int 0 there. So its repr (and the id) differs, `-0.0` vs `0`, and
   validate() reads it as a float: it fails the checks that want an int and take a zero (a pin, a
   device's bus `id` and `addr`, a `provides` priority or timeout, `recorder.telemetry_ms`), so load()
   falls back to the defaults where plan B runs the document; and an enabled watchdog's
   `wdt_timeout_ms` of `-0` goes unchecked here, where plan B refuses it below the floor;
 * an integer past the u64 / i64 range is a float here and an exact int there, so validate() reads it
   as a float too (and the repr prints it as one);
 * `NaN`, `Infinity`, a lone surrogate escape and a number past f64 (`1e400`) parse there and are not
   JSON here (load() falls back for the text; plan B would have loaded it);
 * nesting: serde_json refuses a document with arrays and objects nested 128 levels deep or more (its
   recursion limit, which keeps a hostile text from overflowing the stack), and CPython's json reads one
   until its own recursion limit (about 1000). load() falls back for the text, where plan B runs it.

No writer the project has emits any of these.

### `pub struct Loaded`

What load() chose for this boot.

### `pub fn canon(value: &Value) -> String`

Plan B's `_canon`: the text config_id hashes, Python's repr of the document with sorted keys.

A dict prints `{` + `repr(key):canon(value)` joined by `,` + `}`, its keys sorted (by code point, sorted
here explicitly, whichever map serde_json was built with); a list prints `[a,b]`; anything else is its
Python repr: a str with Python's quote choice and escapes, an int in decimal, a float as Python's repr
(`1e-05`, `1e+16`, `5.0`), True, False and None.

### `pub fn config_id(document: &Value) -> String`

Plan B's `config_id`: the first 12 hex digits of sha256(canon(document)) over its UTF-8 bytes, with
`board.firmware_version` left out.

load() restamps the firmware version from the running build, so keeping it would make the id a property
of the build: a byte-identical config would announce another id after every deploy, and the id
`set-config` returns would not match the one whoami then announces.

### `pub fn load(stored: Option<&str>, defaults: &Value) -> Loaded`

Plan B's `load`: the stored config if it is valid JSON and valid, else the defaults. Never fails: a bad
stored config degrades to the defaults, and the source says why.

* `stored` None (nothing stored): the defaults, source `default`, and the defaults' own messages.
* Text that is not JSON: the defaults, `default(fallback: board.config is not valid JSON)` and that one
  message.
* A document validate() refuses: the defaults, `default(fallback: invalid board.config)` and its messages.
* Otherwise the stored document wins as it is, but for `board.firmware_version`, which is restamped from
  the defaults' (the running build; 'dev' when they name none) whenever both board sections are objects:
  it describes the running binary, not configuration. The source is `active`, or names the schema
  versions when they differ (`outdated`).

## `board_config::printable`

_Tested by its own `mod tests`._

Which non-ASCII characters CPython 3.12's `repr()` copies as they are. Python calls a character printable
unless its Unicode category is Cc, Cf, Cs, Co, Cn (unassigned), Zl, Zp or Zs (the space aside), and
`repr()` escapes the rest, so config_id hashes depend on the exact set: unassigned code points included,
which makes it a property of CPython's Unicode database (15.0.0 in 3.12), not of Rust's. (Rust's own
`escape_debug` table differs: a newer Unicode, and it also escapes default-ignorable characters such as the
variation selectors, which Python prints.)

[`NON_PRINTABLE`] was generated once from CPython 3.12's `str.isprintable()` over U+0080..U+10FFFF:
```text
python3.12 -c "edges, inside = [], False
for code in range(0x80, 0x110000):
    if chr(code).isprintable() == inside: edges.append(code); inside = not inside
print(edges + [0x110000] * inside)"
```
golden/config_id.json carries strings made of every edge's neighbours, repr'd by CPython, and
tests/golden.rs holds `canon` to them, so a table that drifts from CPython fails there.

## `board_config::python`

_Tested by its own `mod tests`._

Python's value semantics over JSON values, the ones plan B's config.py leans on: `repr()` (the config_id
hash and every `%r` in validate's messages), `%s` formatting, truthiness (`x or y`, `if x`) and `==`; and
`isinstance(x, int)`, the test plan B's readers of pins and counts apply (the typed view's M5 readers).

A value is what CPython's json.loads makes of a document: an object is a dict, an array a list, a string a
str, an integer an int, any other number a float, and true/false/null are True/False/None. The oracle is
CPython 3.12, not MicroPython: plan B's board prints non-ASCII text raw and floats in single precision, and
the port plan (§6, `config_id` floats) fixes the CPython meaning as the contract.

Public for the other ports of code that reads the config the same way (crates/layout: layout.py compares,
formats and tests config values as config.py does): [`equals`], [`formatted`] and [`truthy`].

### `pub fn formatted(value: &Value) -> String`

Python's `'%s' % value`: the text itself for a string, the repr of anything else (a float's str is its
repr in Python 3).

### `pub fn truthy(value: &Value) -> bool`

Python's truthiness, as `x or y` and `if x` read a value: None, False, a zero, and an empty str, list or
dict are false; everything else is true.

### `pub fn equals(left: &Value, right: &Value) -> bool`

Python's `left == right` between two JSON values: numbers (and bools) by value across int and float, str
by text, list by items in order, dict by the same keys with equal values; different kinds are unequal.

## `board_config::validate`

_Tested by its own `mod tests`._

`validate`: config.validate and every helper it calls, ported line by line, with the same messages in
the same order. It checks FILE INTEGRITY only (structure, types, pin uniqueness, bus references, reserved
pins, the watchdog floor), never hardware health, which the running board reports.

One rule is the port's own, so a document breaking it is the one place the messages differ from plan B's: a
device's `debounce_ms` is an int of at most [`LONGEST_DEBOUNCE_MS`] (`Validation::debounce`). The drivers read
their config blocks infallibly, with plan B's default for a missing or malformed key, so a value that is
well-formed but absurd is refused here, where the operator sees why: at `set-config`, and at boot, where load()
then runs the defaults.

As in plan B, one pass threads three accumulators through a helper per section ([`Validation`]): the
messages, the GPIO owners (uniqueness and reserved pins) and the buses the `buses` section defines (which
devices then address).

Order: plan B walks each dict in its document order, this port in the map's order. serde_json's default
map is sorted, so the two agree on every document whose keys are sorted, which is every file plan B or
the tools write (`write`). Where a document's keys are not sorted, the messages are the same set, but two
messages may swap places, and of two claims on one GPIO the other one may be named first.

### `pub fn validate(document: &Value) -> Vec<String>`

Validate a config document for FILE INTEGRITY: config.validate, with the same messages in the same order.

Returns the messages; empty means valid. A document that is not an object gets the single message
`config is not an object`. Never panics: every value of every kind is a message or nothing, which is also
why plan B's `validation raised` fallback in load() has no counterpart here (no JSON document reaches it).

### Constants

- `const LONGEST_DEBOUNCE_MS: u64 = 1000` — The longest `debounce_ms` a device may ask for, in milliseconds: a port rule, plan B checks none.

## `board_config::view`

_Tested by its own `mod tests`._

The typed, read-only view of a config document: only what the firmware consumes so far: the board id and
MCU, the Wi-Fi networks and the CC hub, the field's range limit (M2), and the camera. Each reader follows the plan B
code that reads the same keys, and is meant for a document [`crate::validate`] accepts (what load()
returns); for anything else it answers the nearest safe value, as each one says.

For the device bring-up (M5): the I2C and SPI buses ([`i2c_bus`], [`spi_bus`]), plan B's device directory
([`directory`]), and per entry its pins ([`pin`]) and its databoard sources ([`provides`]), plus the setup
attempts ([`setup_retries`]). They are meant for the post-layout document (crates/layout rewrites it in place
first), as plan B's Controller reads the config layout.resolve has rewritten.

For the supervisor (M8): the checkpoint's cadence ([`checkpoint`]), the warm start's toggle ([`warm_start`]) and
the watchdog's settings ([`watchdog`]). Unlike the readers above, the checkpoint and the watchdog refuse a key
plan B could not run with, so the firmware fails the entry at bring-up and arm refuses, as plan B's arm refused
a task whose setup failed or whose run loop crashed on one.

Where plan B would carry a value of the wrong type into the radio or the socket and fail there (a text
`retry_ms`, a numeric password, a host that is not text), the view reads it as the default or as off; each
such reading is named on its function. Every reader borrows the document and changes nothing.

### `pub struct Wifi`

The board's Wi-Fi as plan B's wifi driver and CC link set it up.

### `pub struct Network`

One network the board may join.

Its Debug never prints the password, only whether there is one: a Wi-Fi view ends up in log lines, panic
and assert messages, which reach the console, the recorder and CC's log stream.

### `pub enum Hub`

Where the CC link dials (tasks/cc_link.py and cc_client.Client).

### `pub struct Camera`

The flight camera on the board's CSI connector: an OV5647. Its SCCB is the board's i2c:0, fixed by the
wiring, so the entry names no bus.

### `pub enum CameraClock`

What the camera entry says about its oscillator.

### `pub struct I2cSpecification`

An I2C bus, as plan B's i2cbus.Bus builds it from its spec (i2cbus.py:91-96).

### `pub struct SpiSpecification`

An SPI bus, as plan B's spibus.Bus builds it from its spec (spibus.py:131-147).

### `pub struct UartSpecification`

A UART bus as the GNSS opens it (M11).

### `pub enum Section`

Which device list of the config an entry sits in.

### `pub struct Entry<'d>`

One device entry of the config, as plan B's Controller reads it to bring the device up.

The bus fields are the entry's own, with none of the defaults a driver applies: plan B's i2cbus.bind reads an
absent `id` as 0 and an absent `addr` as the driver's datasheet address (i2cbus.py:297-301), spibus.bind an
absent `id` as 1 (spibus.py:217), and each an absent `bus` as its own kind. An explicit null is not absent there
(bind() then finds no bus, or hands the driver no address and its setup fails), nor is an `addr` this view reads
as None: [`Entry::fields`] tells them apart.

### `pub struct Provide<'d>`

One quantity a device feeds the databoard: an entry of its `provides`, as plan B's `Databoard.provide` hands it
to `Parameter.add_source` (databoard.py:457-483, 227).

### `pub struct Watchdog`

The supervisor watchdog's settings, as plan B's watchdog task reads its entry (tasks/watchdog.py:34-36).

### `pub fn board_id(document: &Value) -> String`

The board id (`board.id`), as the hub and CC address the board. Empty when the document has none, where
plan B would raise (never for a document validate() accepts).

### `pub fn mcu(document: &Value) -> String`

The MCU (`board.mcu`). Empty when the document has none (never for a document validate() accepts).

### `pub fn wifi(document: &Value) -> Wifi`

The Wi-Fi view, following plan B's drivers/wifi.py setup() for the networks and the policy, and
tasks/cc_link.py with cc_client.Client for the hub.

Networks: `wifi.networks` when it is a non-empty list, else `[wifi.ssid]` when that is non-empty, else
none. A bare string entry is `{ssid}`. Each entry starts from the section's defaults (policy 'auto',
enabled, the section's `retry_ms` or 10000 ms, the section's `password` or empty) and its own
keys override them. Note: the entry defaults for `policy` and `enabled` are fixed, NOT the section's: plan
B's setup() never reads a top-level `enabled`, and the top-level `policy` only decides `policy_enabled`.
Dropped: an entry whose ssid is empty (or not truthy), whose `enabled` is not truthy, or whose policy is
'disabled'.

Readings where plan B could not join or dial at all (validate() lets these through):
 * an entry that is neither text nor an object, or a non-list `networks`, is skipped (validate refuses
   them); an ssid that is not text drops the entry (plan B's connect() raises on it);
 * `retry_ms`: a number is the least whole number of milliseconds it allows (Python compares the integer
   tick difference with it), held within 0..=u32::MAX; a bool is 0 or 1; anything else is
   10000 ms, the default (plan B's comparison raises TypeError at the second attempt);
 * a password that is not text is empty (MicroPython's connect() takes None as no password, and raises on
   other types);
 * `cc_host`: a falsy non-text value (false, 0, [], {}) dials the gateway, as plan B's `host or ...` does;
   a truthy one is [`Hub::Off`] (plan B dials it and never connects);
 * `cc_port`: an integer in 0..=65535, or a float equal to one (the typed view's rule for integer knobs);
   anything else is 1234, the default (plan B never connects with it);
 * a `wifi` section that is present but not an object (null, which validate accepts, or another kind):
   plan B's wifi and CC setups both raise on it, so neither runs: the radio is untouched (`policy_enabled`
   false, no networks) and [`Hub::Off`].

### `pub fn max_range_m(document: &Value) -> Option<f32>`

The airframe's glide range, in metres: `max_range_m` on the first sensor or component named 'field'
(main.py `config.device(cfg, name='field')`, which the Mission takes as its range limit).

None when the config does not say (no field device, or no `max_range_m`): the caller then uses the
Mission's default, as plan B does (`field_cfg.get('max_range_m', 200)`). Also None for a value that is not
a number (null, text, a bool), which plan B would carry into the Mission's range comparisons and fail on.

### `pub fn schedule_hz(document: &Value) -> u32`

The control thread's tick rate, Hz: `schedule_hz` on the first entry whose implementation is the `flight`
activity, enabled or not (plan B's flight.py:75 `config.get('schedule_hz', 100)`).

One clock paces the whole control thread in the port (the IMU, separation, the sequencer; the flight law from
M13), where plan B's timer paced the flight task alone and its `schedule_hz` 0 ran that task on asyncio at
`period_ms`. So 0, a value past 1 kHz, anything that is not Python's int, and an absent flight entry all read as
[`DEFAULT_SCHEDULE_HZ`].

### `pub fn camera(document: &Value) -> Option<Camera>`

The board's camera: the first sensor or component whose implementation is 'ov5647', its `driver` or else its
`activity` (plan B's `config.device(cfg, driver=...)` order and match), when it is enabled (`enabled` absent
reads as enabled).

None when the config declares no camera, or a disabled one: the board then records no video and leaves the
encoder's internal RAM unreserved. Plan B never had a camera of its own (the Luckfox recorded video), so this
entry is a schema addition: validate() checks it as any device, and older files, which have none, stay valid.
Plan B itself has no 'ov5647' implementation: it counts the entry as a failed device and refuses to arm.

### `pub fn i2c_bus(document: &Value, id: u8) -> Option<I2cSpecification>`

The I2C bus `id`, as plan B builds it: `config.bus(cfg, 'i2c', id)` (config.py:681-696) handed to i2cbus.Bus,
which constructs `I2C(id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=spec.get('freq', 400000))`
(i2cbus.py:91-96).

None when the config defines no such bus: `buses.i2c.<id>` absent, null or not an object, which plan B's bind()
reports as no bus, so its devices are not connected. Also None for a spec that lacks `scl` or `sda` (plan B's
constructor raises KeyError, losing the bus and every device on it; validate() refuses it) or that names no GPIO
for one of them: a pin reads as [`pin`] reads a pins-map value, where plan B's `Pin()` raises.

The clock is `freq` when it is a whole number of Hz in 1..=u32::MAX: an int, or a float equal to one (the typed
view's rule for integer knobs). Absent or anything else, 400 kHz: validate() checks no clock, and plan B hands any
other value to MicroPython's constructor, which refuses null, text and a float (losing the bus) and has no
meaningful clock for a bool, zero or a negative.

### `pub fn spi_bus(document: &Value, id: u8) -> Option<SpiSpecification>`

The SPI bus `id`, as plan B builds it: `config.bus(cfg, 'spi', id)` (config.py:681-696) handed to spibus.Bus,
which constructs `SPI(id, baudrate=spec.get('baud', 5_000_000), polarity=mode >> 1, phase=mode & 1, sck=...,
mosi=..., miso=...)` with `mode = spec.get('mode', 3)` (spibus.py:134-139).

None as for [`i2c_bus`]: no such bus, or a spec that lacks `sck`, `mosi` or `miso` or names no GPIO for one.
The clock reads as [`i2c_bus`] reads it, with 5 MHz for the default.

The mode is the one of 0..=3 that `mode` equals under Python's `==`, which are exactly the values validate()
admits (config.py:194-197): an int, a bool (True is mode 1, as plan B's shifts read it) or a whole float (plan B's
`>>` raises on a float, losing the bus; the view reads the mode it names). Absent or anything else, 3.

### `pub fn hitl_enabled(document: &Value) -> bool`

Whether `document` runs HITL: an enabled component whose implementation is `hitl` (doc/specs/sim-link.md).

### `pub fn mask_simulated(document: &mut Value) -> Vec<String>`

Plan B's `config_hitl.mask`: with HITL enabled, every [`SIM_SENSORS`] sensor that is enabled is disabled (`enabled`
false), by its name; nothing changes without HITL. Run after the layout, which re-enables the parts a revision
fits (plan B's tools/hitl_run.py:47).

Returns the names it disabled, in the config's order.

### `pub fn uart_bus(document: &Value, id: u8) -> Option<UartSpecification>`

The UART bus `id`, as plan B opens it: `UART(id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])`
(gnss.py setup). None when the config defines no such bus (plan B's setup returns False), or for a spec that lacks
`tx`, `rx` or a whole `baud` in 1..=u32::MAX, or names no GPIO for a line (plan B's constructor raises).

### `pub fn uart_transmitter(document: &Value, id: u8) -> Option<(u8, u32)>`

A transmit-only UART `buses.uart.<id>` (the recorder's link to its receiver, M19: plan B's UART1, `tx` 20 at
921600 baud): its TX GPIO and baud rate, an `rx` ignored. None when the bus, its `tx` or a whole `baud` is missing.

### `pub fn directory(document: &Value) -> Vec<Entry<'_>>`

Plan B's `Controller.directory()` (controller.py:96-103): every enabled, named device entry, `sensors` first and
then `components`, each in its config order, the order plan B creates and sets the devices up in.

Enabled as plan B reads `device.get('enabled', True)`: absent, or truthy. Named: a non-empty text `name`; plan B
would list a truthy name that is not text too, which validate() refuses. Skipped where plan B raises: an item that
is not an object, and a `sensors` or `components` that is not a list.

A part layout disabled is not here, as in plan B: crates/layout writes `enabled` false into the document this
reads (layout.py:430-453 `_fit`, for the parts layout.py:291-321 `fitted` rules out).

### `pub fn entries(document: &Value) -> Vec<Entry<'_>>`

Every named device entry, enabled or not, sensors first then components, each in its config order (the
[`directory`] before its `enabled` filter): what a part could be once the layout has fitted it. The recorder
registers its streams from these before the layout runs, so a part the layout enables (a fitted SEN0253) still has
one.

### `pub fn pin(document: &Value, entry: &Entry, key: &str) -> Option<u8>`

Plan B's `Task._pin_gpio(field)` (task.py:144-166): the GPIO an entry's `key` names through the board's `pins`
map. The entry gives a pin NAME (`"cs_pin": "lsm6dso32_cs"`), and the map its number.

None, exactly as plan B, when the feature is absent or wired off: the entry has no `key`, the map has no such
name, or the map gives null or a negative number. The number is Python's int, as plan B tests it
(`isinstance(gpio, int) and gpio >= 0`): a bool is GPIO 0 or 1 and a float none, both refused by validate().
Also None where plan B would fail: a `pins` that is not an object, a `key` whose value is not text (the map's
keys are text, so plan B finds nothing for a number and raises on a list) and a GPIO past 255, which no MCU the
config names has.

Plan B's default pin name for an entry that omits `key` (`_pin_gpio('pin', 'separation_switch')`) is not here: no
M5 part uses one.

### `pub fn provides<'d>(entry: &Entry<'d>) -> Vec<Provide<'d>>`

The quantities `entry` provides, sorted by quantity: plan B's `provide(name, self.config.get('provides', {}))`
(databoard.py:457-483), each one `add_source(source, spec.get('priority', 0), spec.get('timeout_ms', 0) * 1000,
spec.get('reconcile', False))`. Sorted as controller.py:151-162 `_quantities` lists them; plan B registered them
in the document's order, which orders only one source's own quantities, never two sources of one quantity.

Empty when the entry provides nothing: no `provides`, or one that is null or not an object (validate() admits
null, and plan B's drivers raise on it in setup). A quantity whose spec is not an object is left out (plan B
raises on it; validate() refuses it).

`priority` and `timeout_ms` are Python's int (a bool is 0 or 1); anything else reads as 0, the absent reading
(validate() admits only an int). A priority past i32 saturates, so two such ranks tie (the configs rank 0..=3).
A timeout past u32::MAX saturates.

A negative timeout reads as 0, the default second (validate() admits it). Plan B keeps it (`expire_us or
_DEFAULT_EXPIRE_US`, databoard.py:254), and on a primary-tier source it becomes the quantity's SHARED window
(databoard.py:255-259), under which `fresh()`'s `0 <= elapsed <= window_us` (databoard.py:117) never holds: every
source of the quantity reads stale, not just this one. On a source below the primary tier plan B never consults
it either.

### `pub fn setup_retries(document: &Value) -> u32`

How many times the bring-up tries a device's setup before it calls the device failed: plan B's `max(1,
board.get('setup_retries', 1))` (controller.py:368). Despite the key's name, a count of ATTEMPTS: 1 is one try and
no retry.

Python's int (a bool is 0 or 1, which `max` lifts to 1); 1 when absent, and also for anything else, and for a
`board` that is not an object: plan B's bring-up raises on text (`max(1, '3')`), null, and a float
(`range(1, 3.0 + 1)`). A count past u32::MAX saturates.

### `pub fn checkpoint(document: &Value) -> Option<Result<f32, String>>`

The airborne checkpoint cadence, in seconds: `checkpoint_s` on the first enabled entry whose implementation is the
`checkpoint` activity (plan B runs a Checkpoint task per such entry; this reads the first), read as plan B's setup
reads it, `max(1, self.config.get('checkpoint_s', 1))` (warmstart.py:311).

Absent is 1 s, and so is a bool (Python's `max(1, True)` is the int 1). A number is kept as f32 and floored at
1 s, a negative one included. Anything else (null, text, a list, an object) is refused, as plan B's `max` raised
TypeError on it in setup, so its checkpoint failed and arm refused.

Returns: None when no enabled entry runs the checkpoint; else the cadence in seconds, at least 1, or
`checkpoint_s: not a number`.

### `pub fn warm_start(document: &Value) -> bool`

Whether a crumb may warm-start the board: `warm_start` on the first sensor or component NAMED `checkpoint`,
enabled or not, by Python's truthiness, and true when there is no such entry or it has no `warm_start`. Plan B's
restore reads `(config.device(cfg, name='checkpoint') or {}).get('warm_start', True)` (warmstart.py:274).

Found by its name, not its implementation, as plan B finds it. A present `warm_start` that is falsy (false, null,
0, '', an empty list or object) turns the warm start off.

### `pub fn watchdog(document: &Value) -> Option<Result<Watchdog, String>>`

The supervisor watchdog's settings: the first enabled entry whose implementation is the `watchdog` activity, each
key absent read as plan B's default (5000, 200 and 500 ms, tasks/watchdog.py:34-36).

The keys must be what plan B's watchdog runs with, or it is refused at bring-up, where plan B's own refusal came
later but with the same outcome, arm refused:
 * `wdt_timeout_ms` is a JSON integer of at least 4000 ms, the boot floor validate() holds an enabled watchdog
   to, and at most i32::MAX, the longest the task watchdog keeps whole (`LONGEST_TIMEOUT_MS`). A float is
   refused even with no fraction: plan B's `WDT(timeout=)` takes only an int, so its run loop crashed on one,
   which put the task in the failures arm refuses on. So `100.0` and `3999.0`, which validate() does not hold to
   the floor (it checks an int timeout only), never reach the task watchdog;
 * `period_ms` is a JSON integer from 1 to 2^29 - 1 (`LONGEST_PERIOD_MS`): plan B's `sleep_ms` crashed its run
   loop on a float or a longer one too. 0 or below is refused, where plan B would spin;
 * `stall_ms` is any number above zero as an f32, a fraction included, as plan B multiplies it by 1000 and
   compares.

Stricter than plan B: a bool for any key (Python's int there), a text `stall_ms` (plan B failed only at its
first stall check, in flight), a `stall_ms` at or below zero, and a `period_ms` at or below 0.

Returns: None when no enabled entry runs the watchdog (the task watchdog keeps IDF's warn-only default); else
the settings, or the first key refused, in the order above, as `<key>: <why>`.

### Constants

- `const DEFAULT_SCHEDULE_HZ: u32 = 100` — The control tick's rate when the config gives none: plan B's flight default (flight.py:75), and the 100 Hz the IMUs were read at (their `period_ms` 10).
- `const SIM_SENSORS: [&str; 12] = [` — The sensors a HITL run's simulator publishes over, by config NAME (plan B `config_hitl._SIM_SENSORS`; they are entry names, not drivers): a real part left beside the simulator would be a silent second source (avionics-port.md R10).

## `board_config::writer`

_Tested by its own `mod tests`._

`write`: the canonical pretty text a board config is stored as. It is CPython's
`json.dumps(document, indent=1, sort_keys=True) + '\n'`, the writer of the `launches/<date>/<BOARD>/*.config`
files (tools/make_telemetry_config.py), and it reproduces every one of them byte for byte.

It is NOT what plan B's own save() writes: commons.atomic_write_json writes `json.dumps(document)`, one
line in the document's key order with `, ` and `: ` separators and no newline. Both are the same JSON to
every reader (load() and config_id() see the same value), so the board files are the format kept: one key
per line, sorted, so a diff shows what moved.

The text is pure ASCII, as json.dumps's default `ensure_ascii` makes it: every non-ASCII character and DEL
become `\uhhhh` escapes (two, a surrogate pair, past U+FFFF). Numbers print as Python prints them (a float
is its repr: `1e-05`, `5.0`).

### `pub fn write(document: &Value) -> String`

The canonical text of a config document: `json.dumps(document, indent=1, sort_keys=True) + '\n'`.

Keys are sorted explicitly, so the text is the same whichever map serde_json was built with.

# `cc_protocol` — `src/avionics/crates/cc_protocol`

_Uses: no workspace crate. External: base64._

_Integration tests: `tests/conformance.rs`, `tests/vectors.rs`._

## `cc_protocol`

CC <-> board line protocol (doc/specs/cc-protocol.md), wire-identical to src/glider/cc_protocol.py.

One newline-delimited message per line: `<command> [params...]`. Tokens are whitespace-separated, so
there is NO quoting or escaping. A param value is one of:
 * a bare token -> a simple value with no spaces (e.g. `3000`, `taster`, `192.168.10.1`)
 * `base64:<data>` -> anything else: spaces, quotes, JSON, binary

Both sides know each command's schema, so the parser does not guess types: a bare token stays a string
and the receiver converts numerics itself. Named params are `key=value`; everything else is positional.
The command is lowercased; values keep their case. [`parse`] handles requests and responses
(ok/err/pong/iam) alike.

### `pub struct Message`

A parsed protocol message (request or response).

A board receives `command params` (Control has stripped the routing board id), so `args` are the
positional params and `named` the `key=value` ones.

### `pub enum Error`

Why a line or token would not decode.

### `pub fn encode(text: &str) -> String`

Encode a value into one whitespace-free wire token: bare when already safe, else `base64:`-prefixed.

Integers and booleans go through their `to_string()` / `"true"` forms first, exactly as the Python side
does, and are always safe.

### `pub fn decode(token: &str) -> Result<String, Error>`

Decode a wire token back to a string (base64-decoded when `base64:`-prefixed, else the token as-is).

### `pub fn parse(line: &str) -> Result<Message, Error>`

Parse a protocol line (request or response); the command is None for an empty line.

### `pub fn build(command: &str, args: &[&str], named: &[(&str, &str)]) -> String`

Build a protocol line from a command, its positional values and its `key=value` params.

# `databoard` — `src/avionics/crates/databoard`

_Uses: no workspace crate. External: none._

_Integration tests: `tests/concurrent.rs`, `tests/golden.rs`._

## `databoard`

_Tested by its own `mod tests`._

The shared latest-value store for hot data: the port of src/glider databoard.py (doc/specs/coludo.md "Task
Data-Flow and Message Propagation"; doc/specs/avionics-port.md §4.1 and M5).

The structure is plan B's:
 * [`Parameter`] -- one fused quantity (`altitude`, `accel`, ...) for its consumers. It holds one [`Channel`]
   per source, KEPT IN RANK ORDER (the lowest rank, the primary, first), and ONE freshness window shared by every
   channel: the tightest timeout of the lowest-rank tier.
 * [`Channel`] -- one source's stream: its latest value and when it was pushed, behind a seqlock, so a reader on
   any thread sees a whole reading or none, and the writer never waits for a reader.
 * [`Writer`] -- the one handle a source pushes through: plan B's `provide()` handed the driver its channel, and
   the hot write path is one step, with no lookup.

Fusion is a pure read: [`Parameter::read`] answers the FIRST fresh channel in rank order (same-rank channels are
equivalent, and the one registered first wins), with its source and its age. "Rank 0 answers while fresh; a
backup takes over only while itself THIS fresh" is emergent: every read judges freshness again, against the
one shared window, so a backup that declared 5 s is still used only while it is as fresh as the primary must be.

Offset reconciliation (opt-in: `reconcile: true` on any provider of the parameter) decides WHAT a fallback
reports. While the primary is fresh, each fresh backup's BIAS against it is learned (an EMA, one look per
[`LEARN_PERIOD_US`]); on handover the backup reads offset-corrected, so it reports what the primary would have --
closing the bias gap between, say, the ICP-10111 and the BMP280 instead of jumping across it. Offsets freeze while
the primary is stale. It is for additive SCALARS only (altitude, pressure), never vectors or unlike quantities: a
[`Sample`] that has no [`Sample::difference`] never learns one.

What is deliberately different from plan B (avionics-port.md §4.1, §6):
 * NO EXTRAPOLATION. Plan B's `value()` (and `read()` with source None) projected the primary's last two readings
   forward when nothing was fresh. A production consumer (the sequencer's `_detect_landing`, reading
   `agl.value()`) took a stale laser height extrapolated past the landing threshold at apogee, and the flight
   ended there. Here a parameter with no fresh source has NO reading: [`Parameter::read`] answers None, and no
   other accessor hands out a number nobody vouches for. A channel keeps one reading, not two: the second existed
   only to extrapolate.
 * Reconcile learning runs on the WRITER side, at plan B's flown cadence. Plan B learned inside `read()`, once
   per new primary reading AT A READ, so on the board its rate was its readers': warmstart polls `altitude` every
   500 ms on the pad and checkpoints it every second aloft, and `pressure` is read only by the CC inspect. That
   is one look every 0.5-1 s, an EMA time constant (alpha 0.1) of about 5-10 s. Readers here run on other threads
   and must not write, so the primary's own push learns (the same EMA, the same rules) and a reader only applies
   the offset -- but a push is every primary reading, 10 Hz from the ICP (§4.3), which would shrink the time
   constant to about 1 s and let the offset follow a boost transient (the nose baro read +83 m at speed on
   2026-10-03) instead of the slow bias it exists to close. So the push learns at most once per
   [`LEARN_PERIOD_US`], warmstart's pad poll: the time constant plan B flew with. The golden fixture
   (golden/build_databoard.py) reads plan B at exactly that cadence.
 * Born stale is a state, not a timestamp. Plan B backdated a new channel by 1 s + 1 us, which is stale only under
   windows up to 1 s: a GNSS-only tier at 2000 ms read FRESH, with no value, for its first second. A channel here
   that never pushed has no reading under any window, and [`Parameter::stamp`] answers None for it (plan B
   answered the backdated tick).
 * Time is an argument (microseconds of uptime, u64, with no 2**30 wrap) and nothing reads a clock.
 * No allocation, no lock, no 64-bit atomic (the P4 core has none): `no_std`, a fixed number of sources per
   parameter ([`MAXIMUM_SOURCES`]), and the 64-bit push time travels inside the seqlock as two words.
 * The registry (plan B's `Databoard` class: get-or-create by name, `provide()` over a config `provides` map,
   `write()` that creates a source on first use, the inspect payload) is not here. The firmware's topology is
   static: controller.rs builds it once at bring-up from the config, each parameter with its own value type, and
   freezes it (§4.5). [`Parameter::add_source`] is what `provide()` did for one quantity.

### `pub trait Sample: Copy`

A value a channel can carry: packed into [`SAMPLE_WORDS`] 32-bit words, so the seqlock moves it with plain
atomic word stores and a reader can never see half of one.

Reconciliation needs [`Sample::difference`] and [`Sample::shifted`]; their defaults (no difference, no shift) mark
a value that is never reconciled, as plan B's `TypeError` on a tuple did.

- `fn to_words(self) -> [u32; SAMPLE_WORDS]` — The value as words; the unused words are zero.
- `fn from_words(words: [u32; SAMPLE_WORDS]) -> Self` — The value [`Sample::to_words`] packed.
- `fn difference(self, _other: Self) -> Option<f32>` — `self - other` when the value is an additive scalar a bias can be learned for, else None (the default).
- `fn shifted(self, _offset: f32) -> Self` — The value corrected by a learned offset (`self + offset`); the default leaves it as it is, since a value with no difference never has an offset learned.

### `pub struct Channel<T>`

One source's stream within a [`Parameter`]: its latest value and push time, behind a seqlock.

The seqlock, all of it atomic words (so no read is ever a data race, only a retry): `sequence` is
`NEVER_PUSHED` (0) until the first push, odd while a push is in progress and even otherwise; a reader takes the
words between two equal even loads of it. Exactly one [`Writer`] pushes at a time, which the claim enforces.

Beside the reading, the bias learned against the primary while this channel is a backup ([`Parameter`]'s
reconciliation); only the primary's writer stores it.

- `fn latest(&self) -> Option<T>` — The latest value, whatever its age; None before the first push, or, for one read, while a push on the same core was preempted mid-write (see `READ_ATTEMPTS`) (plan B `_Channel.value()`).
- `fn recent(&self, window_us: u32, now_us: u64) -> Option<T>` — The latest value if it was pushed within `window_us` of `now_us`, else None (plan B `_Channel.recent`); a push preempted mid-write on the same core reads as not recent for that read (see `READ_ATTEMPTS`).
- `fn pushed_at(&self) -> Option<u64>` — When the latest value was pushed, microseconds of uptime; None before the first push, or, for one read, while a push on the same core was preempted mid-write (see `READ_ATTEMPTS`). So one None is not proof that the channel never pushed: a probe that declares a sensor dead on it must see None again after yielding.
- `fn fresh(&self, now_us: u64, window_us: u32) -> bool` — Whether this channel pushed within `window_us` of `now_us` (plan B `_Channel.fresh`, lower bound included).

### `pub struct Reading<T>`

One fused reading: the winner's value (offset-corrected when it is a reconciled backup), the source that pushed
it, and how old it is. There is no "extrapolated" form: no fresh source is no reading.

### `pub struct TooManySources`

A parameter already holds [`MAXIMUM_SOURCES`] sources and a new one cannot register.

### `pub struct Parameter<T>`

One fused quantity: up to [`MAXIMUM_SOURCES`] channels in rank order and the window they share (plan B
`Parameter`).

Sources register through `&mut self` at bring-up ([`Parameter::add_source`]); pushes and reads take `&self`, so a
parameter shared between threads (a `&'static` once the topology is frozen) cannot change its sources while
anyone holds a [`Writer`].

- `const fn new(name: &'static str) -> Self` — An empty parameter: no sources, the default window. `const`, so a fixed topology can live in a static.
- `fn name(&self) -> &'static str` — The quantity's name.
- `fn add_source(&mut self, source: &'static str, priority: i32, timeout_ms: u32, reconcile: bool) -> Result<(), TooManySources>` — Register (or re-register) `source` at rank `priority`, with its config `timeout_ms` (0 is the default second) and its `reconcile` flag: plan B's `Databoard.provide` for this quantity, through `Parameter.add_source`.
- `fn writer(&self, source: &str) -> Option<Writer<'_, T>>` — The handle `source` pushes through; None when it never registered or another writer holds it already.
- `fn channel(&self, source: &str) -> Option<&Channel<T>>` — One source's channel, to read it alone (plan B `raw(source)` and a driver's look at its own stream); None for a source that never registered.
- `fn read(&self, now_us: u64) -> Option<Reading<T>>` — The fused reading at `now_us`: the first channel in rank order that is fresh within the shared window, with its value, source and age; None when no source is fresh (or none registered).
- `fn fresh_below(&self, rank: i32, now_us: u64) -> bool` — Whether a source ranked BELOW `rank` (a larger number) is fresh at `now_us` (plan B `fresh_below`).
- `fn rank(&self, source: &str) -> Option<i32>` — The rank `source` registered at, or None when it never did (plan B `rank`).
- `fn stamp(&self) -> Option<u64>` — When the primary last pushed, microseconds of uptime; None with no source, before the primary's first push, or, for one read, while a push on the same core was preempted mid-write (see `READ_ATTEMPTS`) (plan B `stamp`).
- `fn offsets(&self) -> impl Iterator<Item = (&'static str, f32)> + '_` — The learned bias per source, in rank order, for diagnostics; empty until something was reconciled (plan B `offsets`).
- `fn sources(&self) -> impl Iterator<Item = &'static str> + '_` — The registered sources in rank order, the primary first (plan B kept its channels in this order; its `sources()` sorted them by name, which the caller can do).
- `fn window_us(&self) -> u32` — The shared freshness window in microseconds.

### `pub struct Writer<'parameter, T: Sample>`

The handle one source pushes its readings through; it also reads its own channel back (it derefs to the
[`Channel`]: [`Channel::recent`], [`Channel::latest`]).

Exactly one exists per channel at a time ([`Parameter::writer`]), and `push` takes `&mut self`, so the seqlock
always has a single writer. Dropping it releases the channel.

- `fn push(&mut self, value: T, now_us: u64)` — Record a reading taken at `now_us`. When this source is the parameter's primary and the parameter reconciles, the push also learns each fresh backup's bias against it, at most once per [`LEARN_PERIOD_US`] (the writer-side learning of the crate notes).
- `fn source(&self) -> &'static str` — The source's name, as registered.
- `fn rank(&self) -> i32` — The source's rank (plan B's `channel.rank`): what a provider passes to [`Parameter::fresh_below`].

### Constants

- `const DEFAULT_EXPIRE_US: u32 = 1_000_000` — The default freshness window, and the one a timeout of 0 means: about twice the slowest sensor's period (plan B `_DEFAULT_EXPIRE_US`).
- `const OFFSET_ALPHA: f32 = 0.1` — The EMA weight of one new bias sample when a backup's offset against the fresh primary is learned (plan B `_OFFSET_ALPHA`): slow enough that one noisy pair barely moves it, quick enough to follow a baro's drift.
- `const LEARN_PERIOD_US: u32 = 500_000` — The shortest time between two looks at the backups' bias: the primary's push learns only when this long has passed since the last push that learned (see the crate notes on the writer side).
- `const MAXIMUM_SOURCES: usize = 8` — How many sources one parameter can hold. The most any parameter has in the default config is four (accel: LSM6DSO32, BMI323, ADXL375, BNO055; altitude: three baros and the GNSS), plus the HITL sim beside them: five. Eight leaves room for an experimental board without growing a channel set the firmware keeps per parameter.
- `const SAMPLE_WORDS: usize = 4` — How many 32-bit words a channel stores a value in: four, a quaternion's floats, the widest value a sensor here publishes. Every [`Sample`] packs into exactly this many.

# `recorder_wire` — `src/avionics/crates/recorder_wire`

_Uses: no workspace crate. External: none._

_Integration tests: `tests/golden.rs`._

## `recorder_wire`

The recorder's line wire codec (doc/specs/recorder-wire.md; doc/specs/avionics-port.md "Storage design"),
the wire parts of src/glider recorder.py and of the host reference tools/recorder_wire.py:
 * [`crc32`], zlib's CRC-32;
 * the integrity wrapper: [`wrap`]/[`wrap_into`] write a line, [`verify`], [`unwrap`], [`wrapped`] and
   [`classify`] read one back;
 * [`split`], recorderd's demux of a byte stream into routed lines, whose filing rule the ring_saver follows;
 * the line text before it is wrapped: a telemetry [`header`] and [`row`] of [`Cell`]s, a [`log_line`],
   the session.csv [`SessionRow`] and [`SESSION_INDEX_HEADER`], the session prefix ([`session_label`],
   [`session_prefix`], [`is_label`], [`routing_session`]), the text-cell rule [`cell`], and the Recorder's
   [`status_message`].

The flash ring's binary format (the block header and the chunks) is flash_ring's own; it takes [`crc32`] from
here, so one table serves the ring and the lines the saver prints from it.

Uptimes are u64 microseconds with no 2^30 wrap; a wrapper's U is the uptime mod 2^32, which is what plan B
and the host reference compute for any uptime text. The module docs list where the text deliberately
differs from plan B's. golden/recorder_wire.json pins all of it to plan B and the host reference.

`no_std` with `alloc`: it builds for the ring_saver on the board and for the host tools alike.

## `recorder_wire::crc`

_Tested by its own `mod tests`._

CRC-32 as zlib and Python's `binascii.crc32` compute it (IEEE 802.3, the reflected polynomial 0xEDB88320):
the wrapper's OPEN tag, and flash_ring's block and chunk checksums. A 256-entry table built at compile time
(1 KiB of read-only data) instead of a crate: the wire needs this one function and nothing else from a CRC
library.

### `pub fn crc32(previous: u32, bytes: &[u8]) -> u32`

The CRC-32 of `bytes`, continuing from the CRC `previous` of the bytes before them, exactly as
`binascii.crc32(bytes, previous)` and zlib's `crc32(previous, bytes)`: start a fresh CRC with 0, and
`crc32(crc32(0, a), b) == crc32(0, a ++ b)`. Continuing is what lets OPEN cover `<routing>@<payload>`
without first copying the pieces into one buffer.

## `recorder_wire::demux`

_Tested by its own `mod tests`._

recorderd's demux (src/camera/recorded/recorderd.cpp `processing` and `processBuffer`): how a raw byte
stream becomes lines filed by routing. The Luckfox daemon is frozen, and the ring_saver files the lines it
prints from a flash block by the same rule (doc/specs/avionics-port.md "The ring_saver"), so a session pulled
from either is the same set of files.

The rule, byte for byte:
 * every byte below 0x0A is dropped as it arrives. recorderd compares a `char`, which is unsigned on the
   Luckfox's ARM, so bytes from 0x80 up pass;
 * a `\n` ends the line, and the line keeps it;
 * a line starting with `@` is routed when another `@` follows at index 4 or later: the routing is the bytes
   between, so it is at least 3 bytes long and an `@` among its first three is kept in it. The payload is
   everything after that `@`, its `\n` included;
 * any other line goes whole to recorder.log, its leading `@` included.

Not ported: recorderd's line buffer is 256 KiB, and a longer line wedges it for good (its loop stops
consuming bytes). No line comes near it: a flash chunk is at most 4032 bytes, and a LOG message at most
1 KiB.

### `pub fn split(stream: &[u8], mut each: impl FnMut(Option<&[u8]>, &[u8])) -> &[u8]`

Demux `stream` as recorderd does, calling `each(routing, payload)` for every complete line in order.

`routing` is None for a line recorderd writes to recorder.log, whose payload is then the whole line;
`payload` always ends with the line's `\n`, so it is exactly the bytes recorderd appends to the file. A line
that lost a byte below 0x0A is handed over without it (in a copy; any other line is borrowed from
`stream`).

Returns the bytes after the last `\n`, as they came: an unterminated line, which recorderd holds until the
next `\n` arrives. The saver's lines never leave one (each ends with its `\n`); a capture cut short does.

## `recorder_wire::text`

_Tested by its own `mod tests`._

The text of a recorder line before it is wrapped (src/glider recorder.py; doc/specs/avionics-port.md "Row,
log and index text"): a telemetry stream's header and rows, a log line, the session.csv index rows, the
session prefix and the Recorder's status message.

Every function appends to the caller's buffer or returns the text, and none adds the line end: the wrapper
closes the line ([`crate::wrap`]).

Deliberate differences from plan B, each one a text the host tools already read:
 * an F32 cell is Rust's `Display`: the shortest text that reads back as the same f32, never an exponent.
   Plan B printed `str(float)`, which on the board (MicroPython 1.29, single precision) is a round-trip text
   of up to 9 digits, not always the shortest (`0.982831936` where the shortest is `0.98283195`), with an
   exponent below 1e-4 (`1e-05`), `.0` on an integral value (`1.0` where this writes `1`), `nan` (`NaN`
   here) and `-0.0` (`-0` here). Compare floats numerically, never as text;
 * an E7 cell (a 1e-7 degree integer) and session.csv's `cc_lat`/`cc_lon` print 7 decimals from the integer
   (`25.5000000`), where plan B printed its float degrees (`25.5`);
 * a Text cell has `;` and control bytes made spaces, and a log line its control bytes. Plan B wrote them as
   they came: a `;` split the cell, a `\n` split the line (both halves then failed their checks), and a byte
   below 0x0A was dropped by recorderd, failing the line's CRC;
 * session.csv's `utc_offset` is emptied outside -720..=840 here as well, where plan B left that to
   mission.py, and the date comes from a Unix epoch (the Rust clock), not the 2000-based MicroPython RTC;
 * a `recorder.session` value that is not text names no session: plan B took `str(value)`, so `true` became
   the label `True`;
 * the status message lists `session`, `tlm`, `log` in that order, as plan B's dict does on CPython, then any
   members plan B did not have (the flash ring's); on the board MicroPython printed its own dict order (`tlm`,
   `log`, `session`). The readers search for `'session': '<s>'`, which both carry.

### `pub enum Cell<'a>`

One value of a telemetry row (doc/specs/avionics-port.md "Producer side": a ring record's cell tags).

### `pub enum SessionLabel`

What a `recorder.session` config value names.

### `pub struct SessionRow<'a>`

One session.csv row (recorder.py `Recorder.index_session`): this boot tied to the wall clock, written at the
`boot`, at every successful time set and at the `anchor`.

- `fn append_to(&self, line: &mut Vec<u8>)` — Append the row's text, the cells of [`SESSION_INDEX_HEADER`] in order: [`row`] of [`SessionRow::cells`], so the text the board's ring_saver prints from a session.csv record and this text are one and the same.
- `fn cells<'b>(&'b self, date: &'b mut String) -> [Cell<'b>; SESSION_INDEX_CELLS] where 'a: 'b,` — The row's cells after its uptime, as a session.csv ROW record carries them (doc/specs/avionics-port.md "Producer side": a session.csv row is a ROW to `session.csv` with Text and integer cells, never text made on the flight path). [`row`] prints them behind the record's uptime as [`SessionRow::append_to`] writes the row: a boot nothing counted, an unset clock's utc and utc_offset, and a missing CC position are empty Text cells, and the free text goes through [`cell`]. The config id is a Text cell as it comes (it is hex, so the Text cell's rule, a `;` or a control byte made a space, never changes it).

### `pub struct Counters`

A ring's figures in the Recorder's status message.

### `pub fn header(line: &mut Vec<u8>, fields: &[&str])`

Append a telemetry stream's CSV header, `uptime;<field>;...` (recorder.py `Telemetry`). As plan B, a stream
with no fields gets `uptime;`.

### `pub fn row(line: &mut Vec<u8>, uptime_us: u64, cells: &[Cell])`

Append a telemetry row, `<uptime_us>;<cell>;...` (recorder.py `Telemetry.push`). As plan B, a row with no
cells is `<uptime_us>;`.

### `pub fn log_line(line: &mut Vec<u8>, uptime_us: u64, descriptor: &str, message: &str, limit: usize)`

Append a recorder.log line, `<uptime_us> <descriptor> :: <message>` (recorder.py `Recorder.log`), with
control bytes made spaces.

`limit` is the longest line in bytes counting the `\n` the wrapper adds: a longer text is cut to
`limit - 1` bytes, wherever that falls (inside a UTF-8 character too), as plan B cut its encoded line to the
ring cell and put the `\n` back. A cut that dropped the line end would merge the line with the next one on
the wire, a telemetry row included.

### `pub fn cell(text: &str) -> &str`

A free-text session.csv cell (recorder.py `_cell`): `text` when it is printable ASCII (0x20-0x7E) without
`;` and at most 32 bytes, else empty. A `;` would split the cell, a control byte the line or its CRC, and a
long cell would crowd the row.

### `pub fn is_label(text: &str) -> bool`

Whether a `recorder.session` label is usable (recorder.py `_is_label`): ASCII letters, digits and `-`, at
least one letter, at most 32 bytes.

The letter keeps a label from reading as a boot id, the missing `_` keeps `<session>_<stream>.csv`
splittable at its first `_`, and the cap keeps the routing it heads short.

### `pub fn session_label(configured: &str) -> SessionLabel`

The label a `recorder.session` text names, as plan B reads it: stripped, its spaces made `-`
(`catapult run3` is `catapult-run3`), then kept only when [`is_label`] passes.

### `pub fn session_prefix(label: &SessionLabel, boot_id: Option<u32>, epoch: i64, random: u32) -> String`

This boot's file prefix (recorder.py `Recorder.session`; doc/specs/recorder-wire.md "Session prefix"): the
first of a valid label, the boot id as `%06u`, or the legacy `YYYYMMDD_HHMMSS_<6 digits>` when nothing
counted the boot.

Only the legacy form reads `epoch` (the clock in Unix seconds) and `random` (any random number, such as
esp_random(), mapped onto the six-digit suffixes): its random part keeps apart boots that start within the
same second of an unset clock.

### `pub fn routing_session(routing: &str) -> Option<(&str, &str)>`

The session a line's `routing` names and the rest after the session's `_` (`000671_imu.csv` is `000671` and
`imu.csv`): a boot id or a label ([`is_label`]: no `_` of its own) up to the first `_`, the legacy clock prefix
`YYYYMMDD_HHMMSS_<6 digits>` ([`session_prefix`]) whole. None for a routing that names no session (the bare
`session.csv`).

### `pub fn status_message(session: &str, telemetry: &Counters, log: &Counters, members: &[(&str, &str)]) -> String`

The Recorder's status message, logged as `Recorder :: <message>` every `stats_ms` (recorder.py `report`):
`{'session': '<s>', 'tlm': {...}, 'log': {...}}` in Python dict repr, which recorder_flight searches for
`'session': '<s>'`.

`members` follow `log` in their order, each `(key, value)` written `, '<key>': <value>`, with `value` already
in Python repr: the ring's `'ring': {...}` (erased, unsaved and lost blocks, the positions) and the ring_saver's
`'saver': {...}` (doc/specs/avionics-port.md M4), whose figures their owners format. Plan B's message has
none, so `&[]` writes it exactly.

`session` and each key are written between single quotes as they are: a prefix is letters, digits, `-` and
`_`, and a key is an identifier, never a quote or backslash, which Python's repr would escape.

### Constants

- `const SESSION_INDEX: &str = "session.csv"` — The shared session index's routing: bare, with no session prefix, so every boot appends to one file.
- `const SESSION_INDEX_HEADER: &str = "uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon"` — The session index's CSV header (recorder.py `_SESSION_INDEX_HEADER`).
- `const SESSION_INDEX_CELLS: usize = 10` — The cells of a session.csv row after its uptime: the columns of [`SESSION_INDEX_HEADER`] past `uptime`.

## `recorder_wire::wrapper`

_Tested by its own `mod tests`._

The integrity wrapper every recorder line carries (doc/specs/recorder-wire.md, "Integrity wrapper"):

```text
[@<routing>@]{OPEN};<payload>;<CLOSE>\n
OPEN  = crc32 of the bytes '[<routing>@]<payload>'     8 lowercase hex digits
CLOSE = (!OPEN ^ U) mod 2^32                           U = the payload's leading decimal, else 0
```

[`wrap`] writes a line the way plan B's drain (`recorder._wrap`) and the host reference
(`tools/recorder_wire.py` `wrap`) both do; [`verify`], [`unwrap`], [`wrapped`] and [`classify`] read one
back as the host reference does (its `split` is [`classify`] here, since [`crate::split`] is recorderd's
demux). Everything works on bytes: the host reference decodes a line with `surrogateescape`, whose
round trip is exactly these bytes, and every index it compares at is an ASCII character's.

The offline salvage of the host reference (`salvage`, `pieces`, `recover`, `fit`) is not ported: the board
never reads a wrapped line back. The ring_saver prints every line itself from a record of a chunk whose CRC
checked (doc/specs/avionics-port.md "The ring_saver"), and a cut tears a chunk whole, failing its CRC, not
the UART's lost bytes those rules repair.

### `pub struct Wrapper<'a>`

The three parts of a wrapped body `{OPEN};<payload>;<CLOSE>`, read but not verified.

### `pub enum Status`

What a line read back proves.

### `pub struct Line<'a>`

One line read back: its status, its routing and its payload.

### `pub fn leading_number(payload: &[u8]) -> u32`

U: the payload's leading unsigned decimal (a row's uptime, a log line's microseconds), mod 2^32; 0 when
the payload does not start with a digit (a CSV header).

The digits wrap as they are read, so any length works, a u64 uptime and longer included: the same value
as the host reference's `int(digits) & 0xFFFFFFFF` and plan B's 32-bit viper register. A Rust uptime has
no 2^30 wrap of its own, so U is simply the uptime mod 2^32.

### `pub fn checks(routing: Option<&[u8]>, payload: &[u8]) -> (u32, u32)`

(OPEN, CLOSE) for `payload` under `routing`: what a wrapped line must carry to verify.

`routing` is None for a log line, whose OPEN covers the payload alone. A line that had `@<routing>@` has a
routing even when it is empty: OPEN then covers `@` + payload.

### `pub fn wrap(routing: Option<&str>, payload: &[u8]) -> Vec<u8>`

The wire line for `payload` under `routing` (None for a log line), with its `\n`.

`payload` is the record without a line end. It must not hold a `\n` (that splits the line) nor a byte below
0x0A (recorderd and [`crate::split`] drop those, so the line fails its check): the row and log text of this crate
never does.

### `pub fn wrap_into(line: &mut Vec<u8>, routing: Option<&str>, payload: &[u8])`

[`wrap`], appended to `line`: the ring_saver's batch buffer for a file takes one line after another without a
copy.

### `pub fn unwrap(body: &[u8]) -> Option<Wrapper<'_>>`

The parts of a wrapped body `{xxxxxxxx};<payload>;<xxxxxxxx>`, or None when `body` does not have that shape
(the host reference's `unwrap`). Nothing is verified here.

### `pub fn verify<'a>(routing: Option<&[u8]>, body: &'a [u8]) -> Option<&'a [u8]>`

The payload of a wrapped body when both checks pass under `routing` (None for a log line), else None.

### `pub fn wrapped(body: &[u8]) -> bool`

Whether `body` carries a wrapper at EITHER end: the `{xxxxxxxx};` head at its first `{`, or the
`;<xxxxxxxx>` tail (the host reference's `wrapped`).

A line that lost its leading `@` keeps both ends, and a truncated one keeps the head: neither may pass as a
legacy line from before the wrapper. A tag counts when the host's `int(tag, 16)` reads it
(`host_reads_as_hex`), not only when [`unwrap`] does: a head damaged to `{cb34fe6 };` on a line that also
lost its tail's `>` is BAD on both sides, never LEGACY here.

### `pub fn classify(line: &[u8]) -> Line<'_>`

Read one line in any of the three shapes a capture holds: wrapped and good, wrapped and damaged, or legacy
(the host reference's `split`).

`line` may carry its line end: every trailing `\r` and `\n` is dropped first. A line starting with `@`
routes up to the next `@` wherever it is, an empty routing included (recorderd's own rule differs for
routings under 3 bytes, which the board never writes: [`crate::split`]).

### Constants

- `const WRAPPER_BYTES: usize = 22` — The bytes the wrapper adds around a payload, `{xxxxxxxx};` and `;<xxxxxxxx>`: an empty payload's whole body.

# `sequencer` — `src/avionics/crates/sequencer`

_Uses: no workspace crate. External: serde_json._

_Integration tests: `tests/golden.rs`, `tests/multisource.rs`._

## `sequencer`

The flight stages, the flight word and the stage machine, one module per plan B module it ports:
- [`controller`]: the flight stages and the flight word (src/glider controller.py `Stage`, and the Controller's
  stage, hold and arming with the rules `hold`, `resume`, `arm` and `disarm` change them by), its one writer, what
  each commit does to the recorder's flight flag, and the stages a change may run in;
- [`slot`]: the request slot an operator request reaches the flight word's writer through (new in Rust: plan B's
  one asyncio loop called the Controller directly);
- [`commons`]: the detector primitives, the leaky dwell and the apogee tracker (src/glider commons.py,
  `dwell_step`/`apogee_step`);
- [`sequencer`]: the stage machine (src/glider tasks/sequencer.py), a pure `step` the control thread runs every
  `period_ms` (20 Hz by default) on every bound source of each quantity, its settings, read from the board config,
  and the binding of the registered sources to its slots;
- [`thrust`]: the thrust gate (new in Rust): what one accelerometer has seen of the boost since BOOSTING entry, for
  the separation switch (crates/drivers re-exports it) and for the stage machine's apogee;
- [`video`]: the camera by stage, the rules the firmware's video thread follows from the flight word (new in Rust:
  plan B records no video).

The machine allocates nothing and reads no clock. The crate is `std` for `f32::sqrt`, which the reason text's
|accel| takes and which `core` lacks on stable Rust, for serde_json, which reads the settings, for the binding's
list of ignored sources, made once at bring-up, and for the requesting thread's wait on its slot (a clock and a
sleep, never on the control thread).

## `sequencer::commons`

_Tested by its own `mod tests`._

The stage machine's detector primitives, from src/glider commons.py: the leaky dwell ([`dwell_step`]), the
apogee tracker ([`apogee_step`]) and the squared magnitude the accel thresholds compare
([`magnitude_squared`]). The flight-math half of commons.py (the fin table, `altitude_m`) is in crates/control,
module `commons`.

Pure and stateless: the caller owns the state ([`Dwell`], [`Apogee`]) and hands it back on every call. Plan B
made them so because each detector lived in two worlds -- the board's sequencer and the host sim
(`tools/virtual_flight.py`) -- and the sim's hand-kept mirror had drifted: its apogee had decayed to a
timeout-only deploy, and its launch dwell was the copy the 100 % noise study had really been grading. One
implementation per detector, no mirror; the golden fixture commons.json holds this port to plan B's.

Time is the caller's monotonic clock in milliseconds, a `u32`, and every interval is the signed wrapping
difference [`elapsed_ms`]: MicroPython's `ticks_diff`, at a 2^32 period instead of 2^30. A signed interval is
what plan B compares, so a wrap of the clock costs nothing, and a clock that steps backwards (the plan B tests
feed one) never satisfies a [`reached`] threshold -- the timeouts, the arming window, the contiguous dwells.
The leaky dwell is the exception: there a backwards step turns the step's sign, draining an active dwell but
FEEDING an inactive one, which can complete it ([`dwell_step`]).

f32 throughout (decision D18). The decisions compare metres and g against thresholds of whole metres and
tenths of a g, and an f32 holds an elevation to 3e-5 m at 500 m.

### `pub struct Dwell`

The state of one leaky dwell (plan B's `(credit_ms, last_ms)` pair).

### `pub struct Apogee`

The state of the apogee tracker (plan B's `(peak, since_ms, smooth)` triple).

### `pub fn elapsed_ms(now_ms: u32, then_ms: u32) -> i32`

The signed milliseconds from `then_ms` to `now_ms` on a wrapping `u32` clock: negative when `now_ms` is the
earlier one.

MicroPython's `ticks_diff` at a 2^32 period. Plan B chose `ticks_diff` over a raw subtraction because a bench
board left in SETTING across the 2^30 wrap would otherwise see a huge positive step and satisfy any dwell at
once -- a false launch from arithmetic, on the detector that starts the flight.

### `pub fn reached(now_ms: u32, since_ms: u32, threshold_ms: u32) -> bool`

Whether `elapsed_ms(now_ms, since_ms)` has reached `threshold_ms`: the comparison every dwell, arming window
and timeout of the stage machine makes. Widened to i64 so a negative interval (a clock stepping backwards)
compares below every threshold instead of wrapping into a large one.

### `pub fn magnitude_squared([x, y, z]: [f32; 3]) -> f32`

|(x, y, z)|^2: the accel magnitude squared, in g^2 (plan B `commons.magnitude_sq`).

Squared so the thresholds compare without a root; only the rare transition reason takes one.

### `pub fn dwell_step(active: bool, now_ms: u32, dwell: Dwell, threshold_ms: u32) -> (Dwell, bool)`

One step of a LEAKY dwell: the condition must hold `threshold_ms` of NET time, and a sample that misses only
drains the credit (plan B `commons.dwell_step`).

The plain "sustained since" dwell it replaced reset its start on any single sample that missed. That is fine
for a clean signal and useless for a noisy one: at 100 % accel noise the magnitude crosses the launch
threshold in both directions every few samples, so an unbroken run of `launch_ms` never happens, and
MEASURED (doc/sims/TMS-7-noise_tolerance) the glider never left the pad at all.

Draining rather than resetting keeps the tolerance without changing the UNIT. That distinction cost plan B a
revert: a leaky count of samples also worked, but it silently made `launch_ms` a sample count, so the
trigger's timing drifted with the tick rate. Time in, time out: a `threshold_ms` of 100 still means 100 ms,
and a lone spike still drains to nothing and resets.

The corners are plan B's too: the first active sample starts the clock at zero credit (a dwell never fires
on its first sample, even with a zero threshold); a credit that drains to zero or below ends the dwell, so
a repeated instant with no credit ends it as well; and a clock stepping backwards turns the step's sign, so
it drains an active dwell and feeds an inactive one.

Returns the new state and whether the dwell is satisfied.

### `pub fn apogee_step(elevation: Option<f32>, now_ms: u32, apogee: Apogee, drop_m: f32, dwell_ms: u32) -> (Apogee, bool)`

One step of APOGEE detection: track the smoothed baro peak and report when it has fallen `drop_m` below it
for `dwell_ms` (plan B `commons.apogee_step`).

SMOOTH BEFORE COMPARING, because the peak is a running MAXIMUM and a maximum has no noise immunity at all:
one high sample is latched forever, and every later reading is judged against that spike instead of the
trajectory. MEASURED at 100 % baro noise: apogee fired 2.8 s early, separating at 168 m instead of 268 m and
cutting the flight from 48.8 s to 32.8 s. The drop band rejects noise on the descending side; the smoothing
protects the peak itself. A first-order IIR (weight 1/[`APOGEE_SMOOTH`]) needs one scalar of state where a
sample window needs a ring buffer.

No reading (`elevation` None: not armed yet, or no fresh source) changes nothing -- an open dwell stays open
through a baro dropout, and fires on the first fresh sample still below the line once `dwell_ms` has passed.
The caller keeps the arming window and the burnout fallback, which need its own clock and stage entry.

A NaN elevation poisons the smoother for good, as in plan B: every comparison with it is false, so the
tracker never fires again and the burnout timeout is the way out.

Returns the new state and true exactly when apogee is confirmed.

### Constants

- `const APOGEE_SMOOTH: f32 = 4.0` — The IIR weight divisor of the apogee tracker: the smoothed elevation moves a quarter of the way to each new sample (plan B `commons.APOGEE_SMOOTH`).

## `sequencer::controller`

_Tested by its own `mod tests`._

The flight stages and the flight word, from src/glider controller.py: the stages (`Stage`: their ids, their
operator-facing names, which of them are on the ground and which airborne, and in which a command that changes the
board may run: plan B's cc_client `_in_flight`, narrowed), and the Controller's flight state
(`stage`, `manual`, `armed`) with the rules its `hold`, `resume`, `arm` and `disarm` change it by.

**The flight word.** Plan B kept that state in three Controller attributes, set by five writers on one asyncio
loop. Rust packs it, with `launched`, into ONE `AtomicU32` ([`FlightWord`]): a reader's single load is a
consistent snapshot (a stage never pairs with another moment's hold or arming), and one writer ([`FlightWriter`],
not Clone, handed out once) commits every change: the stage machine's advances, the separation's, and the
operator's requests, which reach it through a [`crate::slot::RequestSlot`]. That writer lives on the control
thread, so a request is applied between two ticks and never races a detector.

The rest of plan B's Controller (bring-up, the device directory) is the firmware's.

### `pub enum Stage`

One flight stage. The ids are plan B's, so a warm-start crumb reads the same on both firmwares; 0 is the
NVS "no checkpoint" sentinel and never a live stage.

- `fn name(self) -> &'static str` — The operator-facing name (`stage`, `health`, `whoami`, and the stage column of sequencer.csv).
- `fn from_name(name: &str) -> Option<Stage>` — The stage an operator names, None for a name that is not a stage.
- `fn from_id(id: u8) -> Option<Stage>` — The stage with this id, None for 0 (the sentinel) or an id past the last stage.
- `fn ground(self) -> bool` — Nothing is flying: the stages active commands may run in (src/glider cc_client `_GROUND`).
- `fn airborne(self) -> bool` — Off the ground: BOOSTING through LANDING (plan B `Stage.airborne`, the union of the powered boost and the passive glide). The stage machine's RSO backstop bounds exactly these: it once tested "not DONE" instead, which let a stale timer force DONE from SETTING on a board sitting on the pad.
- `fn change_refused(self, rezero: bool) -> Option<ChangeRefused>` — Why a command that changes the board must not run in this stage, None when it may.

### `pub struct ChangeRefused(Stage)`

A change refused in a stage ([`Stage::change_refused`]): the stage as a value, so the thread that refuses makes no
text. [`fmt::Display`] writes `refused -- stage is <stage> (changes only in setting)`.

### `pub struct Flight`

The board's flight state as one value: what the flight word packs.

Plan B's Controller `stage`, `manual` and `armed`, plus `launched`, which plan B did not need: Rust keys the
link's reconnects on it.

- `const fn pack(self) -> u32` — The flight word: the stage id in bits 0-2, then `manual` (bit 3), `armed` (bit 4) and `launched` (bit 5).
- `fn unpack(word: u32) -> Option<Flight>` — The flight a word packs, None when it is no flight word: a stage id that is no stage (0, the warm start's "no checkpoint" sentinel, or past DONE), or a bit set past `launched`.
- `fn apply(self, request: Request) -> Result<Flight, StageProblem>` — The flight after an operator request, by plan B's Controller rules (controller.py `hold`, `resume`, `arm`, `disarm`), or why an arm is refused.
- `fn stage_problem(self) -> Option<StageProblem>` — Why this board is not on the ground under automatic sequencing, None when it is (plan B cc_client `_stage_problem`): `arm` refuses it and `verify` reports it, both under the key `stage`.
- `fn reconnect_allowed(self) -> bool` — Whether the link may rejoin, hunt or dial: not once the stage machine has detected the launch.

### `pub enum Request`

An operator request the control thread applies to the flight word ([`Flight::apply`]), posted through a
[`crate::slot::RequestSlot`].

### `pub enum StageProblem`

Why a board is not on the ground under automatic sequencing ([`Flight::stage_problem`]). [`fmt::Display`]
writes plan B's text exactly.

### `pub struct FlightWord`

The flight word: the one atomic every thread reads the [`Flight`] from, written only through its
[`FlightWriter`]. Release stores and Acquire loads: a reader that sees a commit sees what came before it.

- `const fn new() -> FlightWord` — A word at [`Flight::GROUND`], as every boot starts, its writer not yet handed out. `const`, so the firmware's word is a static. Any other flight is committed through the writer, a warm start's restore included.
- `fn as_ptr(&self) -> *const u32` — The word's address, for a C reader that loads it atomically (the video recorder's segment cuts, which stop once the board leaves SETTING): [`Flight::pack`]'s bits. Valid as long as the word, a static on the board.
- `fn load(&self) -> Flight` — The flight now: one load, so its stage, hold, arming and launch are of one moment.
- `fn writer(&self) -> Option<FlightWriter<'_>>` — The word's one writer: Some the first time, None ever after (even once that writer is dropped), so no second thread can ever commit.

### `pub struct FlightWriter<'word>`

The flight word's one writer: the control thread's, so the stage machine, the separation and the operator's
requests commit in one order, between two ticks. Not Clone, and [`FlightWord::writer`] hands out only one.

- `fn flight(&self) -> Flight` — The flight as last committed: the word's value.
- `fn advance(&mut self, transition: Transition) -> Commit` — Commit the stage machine's own advance ([`crate::Step::advance`]): the stage moves, and the launch detector's SETTING -> BOOSTING, the only advance into BOOSTING, sets `launched`.
- `fn separate(&mut self) -> Option<Commit>` — Commit a confirmed separation: BOOSTING moves to GLIDING, held or not. Plan B's driver advances a held BOOSTING too, and the hold stays (drivers/separation.py `_apply`).
- `fn apply(&mut self, request: Request) -> Result<Commit, StageProblem>` — Apply an operator request by [`Flight::apply`]'s rules.
- `fn restore(&mut self, stage: Stage, armed: bool) -> Option<Commit>` — Commit a warm start's restored flight, before any thread runs: the crumb's airborne stage and arming, no hold, and `launched`, since the board was in the air when it reset, so the link stays down (plan B's warmstart `set_stage`, then `arm()` when the crumb was armed). The stage machine then sees the stage as moved from outside, and re-bases its backstop on it.

### `pub struct Commit`

One committed change of the flight word.

- `fn flight_flag(&self) -> Option<FlightFlag>` — How this commit moves the recorder's flight flag, None when it stays: the flag follows `airborne()` of each committed stage, an operator hold's too (a bench hold makes flight blocks, which costs nothing).

### `pub enum FlightFlag`

A change of the recorder's flight flag ([`Commit::flight_flag`]).

## `sequencer::sequencer`

_Tested by its own `mod tests`._

The flight-stage machine, from src/glider tasks/sequencer.py (`Sequencer._tick` and its detectors): the
guarded, forward-only stage automation the control loop gates on, deciding from every source of each quantity.

- SETTING -> BOOSTING: |accel| over `launch_g` for `launch_ms` of net time (motor ignition), OR the baro
  climbing past `launch_alt_m` for as long (an independent, threshold-robust backup), having risen
  `launch_alt_m` within `launch_alt_window_ms` above a level held for `launch_ms` (the rate gate, not plan B's:
  weather drift is no launch, nor one low read). Never the GNSS elevation.
- BOOSTING -> GLIDING: the separation switch is primary, and it moves the stage from OUTSIDE (the firmware's
  separation debounce, plan B drivers/separation.py) -- the machine sees it as an external move. Else the baro
  APOGEE (`apogee_drop_m` below the smoothed peak, blind for `apogee_arm_ms` after BOOSTING entry); the
  burnout timeout last.
- GLIDING -> LANDING: the height below `land_agl_m` for `land_ms`: the laser when it is fresh, else the
  elevation.
- LANDING -> DONE: |accel| inside 1 +/- `still_g` for `ground_ms` (stopped on the ground), or the landing
  timeout.
- Any airborne stage -> DONE: the RSO flight timeout, `flight_timeout_ms` after BOOSTING entry.

**Every source decides in its turn (the operator's, not plan B's).** Plan B decided on one fused reading per
quantity. Here every source the databoard registers for a quantity holds a fixed slot, bound by rank at bring-up
([`Binding`]): four accelerometers, two barometers plus the GNSS (by name, never a voter), one laser. A slot keeps
only the history that compares its source with itself (a barometer's climb and apogee tracker, an accelerometer's
thrust gate and its run at rest), so the offset between two barometers never reads as a rise or a drop, and no
reconcile offset is ever applied. A source is LIVE at a step when it is fresh in its quantity's shared window and
its value is finite: plan B's `read()`, which never consults health.

Each detector keeps ONE dwell, plan B's own, stepped every step with the condition of its DECIDER: the
highest-ranked live source of the class (for the launch; for the apogee the top live barometer, else the GNSS; for
the landing the laser, else the top live barometer, else the GNSS). A lower source never decides while a higher one
is live, and after a handover the dwell carries on only while the new decider's own condition holds too. With no
decider the dwell sees no reading, as in plan B. One exception, for the apogee: a source back from a gap the flight
went on through without it JOINS afresh ([`Tracker`]): its smoothing restarts, it sits out `launch_ms`, and it
decides only while no barometer with an unbroken record is live, since its own record may lack the top.

With one source per class this is plan B step for step (golden sequencer_trace.json) but for three things: the
physics vetoes below (pinned in the fixture where they change a plan B event); a NaN reading, which is no reading
here and poisoned plan B's apogee tracker; and a GNSS standing in for the barometer, which plan B's one apogee
tracker smoothed into the barometer's series and the port keeps apart, the GNSS joining on its return (no fixture
event moves).

On a step where the decider's condition holds, a VETO makes it an inactive sample (a leaky dwell drains, a
contiguous one restarts): a held event must earn its whole dwell again unvetoed, and a peer that misses one step
cannot release it.
- The PEER veto: at least half of the live voting sources, the decider among them, are contrary ([`Detector`] says
  when a peer is). A missing reading is never contrary; the GNSS never counts and never votes.
- The PHYSICS vetoes, across classes, each on EVERY live accelerometer (none live, no veto), so one faulty but live
  part can never block a launch path alone: the altitude launch is held while every live accelerometer has read
  inside the still band for the whole `launch_alt_window_ms` (a boost leaves the band within a step, a coast passes
  through it in about 0.4 s; a step without a reading keeps an accelerometer's run, so a dropout never releases the
  veto); the apogee is held while every live accelerometer's thrust gate ([`crate::thrust`], on its own +x) still
  sees the boost.

The backstops (burnout, flight and landing timeouts) run on machine clocks from stage entry and are never vetoed.
[`Step`] reports where each veto episode begins and ends, for vetoes.csv.

[`StageMachine::step`] is `_tick`, pure: the caller passes the time, the stage and the hold the flight word
holds, and every source's fresh reading, and gets back what to record and where to go. It reads no clock and no
channel, allocates nothing, and the caller commits the advance through the flight word's writer
([`crate::FlightWriter::advance`]) before the next step (plan B's `_advance` set the Controller's stage on the
spot).

[`Settings::from_config`] reads the sequencer's board-config block at bring-up and refuses a value it cannot run
as written (text, a fractional or negative ms, or one longer than its clock reaches): the board then fails the
entry, so `arm` refuses. Plan B crashed a detector in flight on text, and flew a fractional or negative ms.

**Freshness is the whole point.** Plan B's `Databoard.value()` answered a stale channel by extrapolating its
last two samples without bound, and production consumers used it: the laser only reaches about 4 m, so for
all but the last seconds of a flight the agl channel is stale, and its projection across a whole boost is a
noise-signed number -- `landing; agl -9.6m` fired 0.38 s after `gliding; apogee 274m`, ending the flight at
apogee. Every detector here takes a reading only when its source vouches for it (plan B's `read()` with a
non-None source); the Rust databoard has no extrapolation at all, so a stale channel is simply None.

Plan B's GC policy (compact and disable at BOOSTING, re-enable at DONE) is not ported: Rust has no
collector. Neither is its sequencer.csv push, which is the caller's: [`Step`] carries the rows.

### `pub struct Settings`

The sequencer's board-config keys, each field named after its key (the contract): the step period and the
stage machine's thresholds, read by [`Settings::from_config`] with plan B's `Sequencer.setup()` defaults for a
missing one ([`Settings::default`]), and the altitude launch's rate gate, which plan B does not have.
`disable_gc_flight` is not ported.

- `fn from_config(config: &Value) -> Result<Settings, SettingsError>` — Read the sequencer's board-config block: each key it carries over its [`Settings::default`].

### `pub struct SettingsError`

A sequencer key [`Settings::from_config`] refused. [`fmt::Display`] writes `<key>: <value> <why>`, the text
the board fails the entry with.

### `pub enum SettingsProblem`

What is wrong with a refused sequencer key.

### `pub enum Source`

Who vouches for a fresh fused reading. Plan B's `read()` names the device; the one-source form asks one question of
it: is this the GNSS?

### `pub struct Reading<T>`

One fresh fused channel value, as plan B's `read()` answers it with a non-None source.

### `pub struct Readings`

The fused read: one reading per quantity, each None when no source is fresh -- the one-source form the control
thread steps the machine with until it assembles [`SourceReadings`] from its [`Binding`]. It converts as plan B's
fused read would sit in the slots: the accel in the first accelerometer slot, the elevation in the first barometer
slot (the GNSS slot when the GNSS vouches for it), the agl in the laser slot. Stepped so, the machine is plan B's
but for the fusion rule's own (see the module notes): the physics vetoes read the fused accel, so a board lying
still holds an altitude launch, and only a bench copy with no accelerometer entry launches by altitude hands-free.

### `pub struct SourceReadings`

Every bound source's fresh reading at one step, by its slot in the [`Binding`]: None where the source is not fresh
in its quantity's shared window, or no source holds the slot.

None is not a low reading and not a still one: a detector whose decider has no reading does not advance (and the
contiguous dwells restart). A value that is not finite (a NaN, an infinity, in any axis of a vector) is no reading
either: the machine checks.

### `pub struct Binding`

Which slots the registered sources hold: bound once at bring-up, in rank order per quantity (the databoard's
`sources()`), so a slot always belongs to the same source and one source's history never passes to another.

- `fn new(accelerations: impl IntoIterator<Item = &'static str>, elevations: impl IntoIterator<Item = &'static str>, heights_above_ground: impl IntoIterator<Item = &'static str>) -> Binding` — Bind each quantity's registered sources, given in rank order, to its class's slots: the first `accel` sources to the accelerometers, the first `elevation` sources but [`GNSS_SOURCE`] to the barometers and the GNSS to its own slot, the first `agl` source to the laser. The rest are [`Binding::ignored`].
- `fn ignored(&self) -> &[Ignored]` — The sources past their class's slots, for the bring-up log and `inspect`.
- `fn source(&self, decider: Decider) -> Option<&'static str>` — The source holding `decider`'s slot; None for an empty slot.
- `fn readings(&self, acceleration: impl Fn(&'static str) -> Option<[f32; 3]>, elevation: impl Fn(&'static str) -> Option<f32>, height_above_ground: impl Fn(&'static str) -> Option<f32>) -> SourceReadings` — One step's readings, by slot: each bound source's reading through the reader of its quantity, called with its device name -- the firmware reads that source's channel, fresh in the quantity's shared window or None. An ignored source is never read.

### `pub struct Ignored`

A source past its class's slots: the detectors never read it.

### `pub enum Decider`

The slot a detector's decider holds.

### `pub enum Detector`

A detector a veto can hold, and when a live peer of its decider is contrary. The landing has no veto: the drift
between two barometers is unmeasured, and the pitot's vetoes come with the pitot.

- `fn name(self) -> &'static str` — The detector's name, vetoes.csv's `detector` cell.

### `pub enum Physics`

The physics veto that held a detector (see the module notes).

### `pub struct Veto`

One vetoed step of a detector: who decided, which vetoes held it, the vote, and the decider's value.

- `fn kinds(&self) -> &'static str` — The vetoes that held it, vetoes.csv's `kinds` cell: `peer`, `at_rest` or `thrust`, two joined by `+`.

### `pub enum VetoEdge`

Where a veto episode -- a detector vetoed on consecutive steps -- begins and ends: the two rows vetoes.csv records.

- `fn name(&self) -> &'static str` — `begin` or `end`: vetoes.csv's `edge` cell.
- `fn veto(&self) -> &Veto` — The vetoed step the edge carries.

### `pub enum Reason`

Why a stage changed: the reason column of sequencer.csv. [`fmt::Display`] writes plan B's text exactly,
since post-flight tools read it.

### `pub struct Transition`

One stage change and why: one sequencer.csv row (`stage`, `reason`).

- `fn external(&self) -> bool` — Whether the move came from outside the stage machine (the separation switch, an operator, a warm start).

### `pub struct Step`

What one step reports: up to two sequencer.csv rows, in the order plan B pushes them, and the veto episodes that
begin or end on it.

Two rows, because `_tick` first notices a move made from outside since the last tick and only then runs the
detectors, and both can record on one tick: an operator releases a hold that outlasted the flight timeout,
and the backstop fires on the very tick the held stage is first seen.

- `fn transitions(&self) -> impl Iterator<Item = Transition>` — The rows to record, in order.
- `fn vetoes(&self) -> impl Iterator<Item = VetoEdge> + '_` — The veto episodes that begin or end on this step, in [`Detector`] order: vetoes.csv's rows.

### `pub struct StageMachine`

The stage machine: plan B's `Sequencer` state, one dwell per detector, and each slot's own history, stepped by
[`StageMachine::step`].

- `fn new(settings: Settings) -> StageMachine` — A machine that has seen nothing yet: its first step records the stage it finds as external (plan B starts with no stage seen, so even a cold boot's SETTING opens sequencer.csv as `setting,external`). The one allocation it ever makes is here: the barometers' climb histories, boxed.
- `fn settings(&self) -> &Settings` — The thresholds this machine runs.
- `fn step(&mut self, now_ms: u32, stage: Stage, readings: impl Into<SourceReadings>, manual: bool) -> Step` — One stage-machine step: plan B's `Sequencer._tick`, on every source.

### Constants

- `const LONGEST_LAUNCH_ALTITUDE_WINDOW_MS: u32 = 2000` — The longest `launch_alt_window_ms`, twice the operator's 1 s: the machine keeps each barometer slot's elevation history for it in a fixed array of 81 slots, 25 ms apart, 12 bytes each (972 B, two of them, 1944 B), boxed when the machine is built so they live on the heap and not on the control thread's stack; stepping allocates nothing. A sluggish 1.5 g boost is 10 m up 2.0 s after ignition: a 2 s window passes it then, the default 1 s about 0.5 s later.
- `const GNSS_SOURCE: &str = "gnss"` — The device name the GNSS publishes `elevation` under (plan B tasks/sequencer.py `_GNSS_SOURCE`): [`Binding`] gives it the elevation class's GNSS slot, which never starts a flight and never votes.
- `const ACCELEROMETER_SLOTS: usize = 4` — The accel class's voting slots: the LSM6DSO32 and the BMI323 on a SEN0697 board, and room for the ADXL375 and the BNO055 once they are ported.
- `const BAROMETER_SLOTS: usize = 2` — The elevation class's voting slots: the barometers (the ICP-10111 and the BMP581; the BMP280 on TMS-7C). The GNSS has a slot of its own besides them, by name.

## `sequencer::slot`

_Tested by its own `mod tests`._

The request slot: how a thread asks the control thread to apply an operator [`Request`] (`stage`, `arm`,
`disarm`) to the flight word, with no lock, no allocation and no kernel call on the control thread's side.

Plan B needed none: its one asyncio loop called the Controller directly. In Rust the flight word has one writer,
the control thread's ([`FlightWriter`]), so a requesting thread (the CC executor) posts its request into its own
slot and polls for the answer, and the control thread serves the slot once a tick ([`RequestSlot::serve`]).
Nothing notifies the control thread: its task notification counts the timer's ticks.

One `AtomicU32` per requesting thread: `sequence` (bits 16-31) | `phase` (14-15) | `kind` (8-13) | `value`
(0-7). A request goes idle -> posted (the requester) -> taken (the control thread claims it) -> done (the answer
in `value`). A requester that gives up withdraws its request, posted -> idle, and one compare-and-swap decides
that race with the claim: a withdrawal that wins means the request was never applied; one that loses means it
was taken, and its answer follows within microseconds. The sequence tells a request's answer from an earlier
one's.

Every operation is SeqCst: a request costs a handful of them, so the cost is nil, and no ordering has to be
argued.

The requesting thread's whole exchange is [`RequestSlot::request`]: post, poll, and withdraw at the deadline.

### `pub type Answer = Result<Flight, StageProblem>`

What the control thread answers a request: the flight it committed, or why it refused an arm.

### `pub struct Ticket`

A posted request's receipt: what its answer and its withdrawal are matched by.

### `pub enum Unanswered`

Why [`RequestSlot::request`] brought no answer back. [`fmt::Display`] says whether the request was applied.

### `pub struct RequestSlot`

One requesting thread's slot (see the module notes): that thread posts, polls and withdraws; the control
thread serves. A second requesting thread gets a slot of its own.

- `const fn new() -> RequestSlot` — An idle slot. `const`, so the firmware's slot is a static.
- `fn post(&self, request: Request) -> Option<Ticket>` — Post `request` (the requesting thread). Its sequence is the last one's plus one, wrapping.
- `fn answer(&self, ticket: Ticket) -> Option<Answer>` — The answer to `ticket`'s request (the requesting thread, polling).
- `fn withdraw(&self, ticket: Ticket) -> bool` — Withdraw `ticket`'s request before the control thread takes it (the requesting thread, at its deadline).
- `fn request(&self, request: Request, deadline: Duration, poll: Duration) -> Result<Answer, Unanswered>` — Post `request` and wait for its answer (the requesting thread): poll every `poll`, and at `deadline` withdraw the request if the control thread has not taken it yet. One it took is applied or refused, and its answer follows within microseconds: it gets `deadline` once more before it counts as lost.
- `fn serve(&self, writer: &mut FlightWriter<'_>) -> Option<Result<Commit, StageProblem>>` — Serve the waiting request, if any (the control thread, once a tick): claim it, apply it through `writer`, and leave the answer in the slot.

## `sequencer::thrust`

_Tested by its own `mod tests`._

The thrust gate (the operator's, new in Rust: plan B has none): what one accelerometer has seen of the boost since
BOOSTING entry. It reads the glider's own axial specific force, a_x on the body frame's +x (the nose; +y left, +z
up). Thrust is SEEN when a_x holds above the sequencer's `launch_g` for its `launch_ms` of net time (the launch
detector's leaky dwell, [`crate::commons::dwell_step`]), and has ENDED when a_x then holds below 0 g for 20 ms of
net time: the drag deceleration the glider feels the moment nothing pushes it, by burnout or by an early
aerodynamic separation of a destabilized stack. Both start afresh at each BOOSTING entry. A missing a_x drains a
dwell like a sample that misses. A wrong axis or sign, no IMU, or a bench hold never sees thrust.

Two readers, each with its own gates:
- the separation switch (crates/drivers, `separation`, which re-exports this module's types): one gate on the
  fused `accel`, stepped every control tick, whose verdict holds a separation in BOOSTING pending until the boost
  has ended, so vibration under thrust can never deploy;
- the stage machine ([`crate::StageMachine`]): one gate per accelerometer, each on its own +x, stepped every
  machine step, whose verdicts hold the apogee while every live accelerometer still sees the boost.

### `pub enum Thrust`

What a thrust gate has seen of the boost since BOOSTING entry: `control`'s `thrust`.

- `fn name(self) -> &'static str` — The name `control` shows: `none`, `seen` or `ended`.

### `pub struct ThrustGate`

One accelerometer's thrust gate (see the module notes): stepped with its fresh a_x, open once the boost was seen
and then ended, both afresh at each BOOSTING entry.

- `fn new(settings: &Settings) -> ThrustGate` — A shut gate on the sequencer's `launch_g` and `launch_ms`: `settings` are the sequencer entry's, or [`Settings::default`] (plan B's) for a board without one.
- `fn step(&mut self, now_ms: u32, stage: Stage, axial_g: Option<f32>)` — One step at `now_ms` in `stage`, with `axial_g`, the glider's a_x in g from a fresh `accel` (None when it is not fresh). A BOOSTING entry starts the gate afresh, shut. In BOOSTING the sample feeds the dwell of what the gate waits for: a_x above `launch_g` for the thrust, then below 0 g for its end, each a leaky dwell that a missing or contrary sample drains. Anywhere else the gate keeps what it saw.
- `fn thrust(&self) -> Thrust` — What the gate has seen since BOOSTING entry (or of the last BOOSTING, outside it).
- `fn open(&self) -> bool` — Whether the gate is open: the thrust seen and then ended.

### Constants

- `const THRUST_END_MS: u32 = 20` — How long a_x must hold below 0 g for the thrust to have ended, ms of net time: two ticks of the control thread's default schedule, long enough that one sample of vibration under thrust does not end it.

## `sequencer::video`

_Tested by its own `mod tests`._

The camera by stage: the rules of the firmware's `video` thread, the one owner of the camera's recorder
(doc/specs/avionics-port.md "Video by stage"). New in Rust: plan B records no video. The thread reads the flight
word and the fused accel every 250 ms and asks its [`Trigger`] what to do ([`Trigger::look`]), and it answers the
operator's `video start` and `video stop` by [`start_refused`] and [`stop_refused`]. Nothing here touches the
camera.

- The rod trigger: the board armed in SETTING starts the flight recording ([`Owner::Flight`]) once the airframe
  stands nose up on the rod: the fused accel's +x (the body frame: +x the nose, +y left, +z up; specific force, g)
  at or above 0.87 g, within about 30 degrees of vertical, at 8 looks in a row, 2 s at the 250 ms look. It reads
  the accel, not the attitude filter, so it stands on its own. On a board with an accelerometer, a look with no
  fresh accel is a look off the rod (the count restarts): a stale channel never starts a recording on the bench. A
  board without one (no `accel` on its databoard) starts it at the episode's first look, as the arm alone did
  before the rod trigger, so it still records. The start replaces an operator's recording.
- An arm episode is a run of looks armed in SETTING: it begins at an arm there, or at a return to SETTING still
  armed (the way back from a false launch: `stage done`, the re-zero, `stage setting`), since DONE stopped the
  recording. Each episode counts its nose-up looks afresh and has one start: a start that fails, or a flight
  recording the operator's `video stop` ends, is not started again before the next episode.
- A disarm in SETTING stops the flight recording; an operator's recording runs on.
- DONE stops whatever records.
- Nothing starts outside SETTING, so the camera's register traffic on the shared I2C port 0 stays on the pad, and
  nothing stops in the air: the operator's `video stop` runs in SETTING and at DONE.

Video is a utility (the operator: at least 20 delivered frames a second, and never at the flight's expense): a
camera that fails never blocks `arm`.

### `pub enum Owner`

Who started the running recording.

### `pub enum Duty`

What the video thread does after a look at the flight word ([`Trigger::look`]).

### `pub struct Trigger`

The rod trigger (see the module notes): whether the board has an accelerometer, and what the video thread keeps of
the arm episode between its looks. A disarm and an arm between two looks are one episode: the thread sees neither.

- `fn new(accelerometer: bool) -> Trigger` — The trigger of a board with an `accelerometer` or without one, in no episode, as at every boot's first look.
- `fn look(&mut self, now: Flight, accel: Option<[f32; 3]>, recording: Option<Owner>) -> Option<Duty>` — What the video thread does now, from the flight word, the fused accel and the recording it runs: one look of the arm episode, which a look not armed in SETTING ends.

### `pub enum Refusal`

Why the operator's `video start` or `video stop` does not run ([`start_refused`], [`stop_refused`]).
[`fmt::Display`] writes the text the operator is answered with, under [`Refusal::code`].

- `fn code(self) -> &'static str` — The CC error code the refusal is answered with: `unsafe` for a stage that forbids the command, as the command gate answers; `video` for the recorder's state, as `video start` and `video stop` answer their failures.

### `pub fn start_refused(flight: Flight, recording: Option<Owner>) -> Option<Refusal>`

Why the operator's `video start` must not run now, None when it may: only in SETTING (re-checked where the video
thread serves it, since the request may wait behind a start or a stop), and only while nothing records.

### `pub fn stop_refused(flight: Flight, recording: Option<Owner>) -> Option<Refusal>`

Why the operator's `video stop` must not run now, None when it may: on the ground (SETTING, and DONE, where the
command gate refuses every other change), and only while a recording runs, whoever started it.

# `beacon` — `src/avionics/crates/beacon`

_Uses: `recorder_wire`. External: anyhow, esp-idf-svc, log._

## `beacon`

_Tested by its own `mod tests`._

The recovery beacon (doc/specs/avionics-port.md M11b, M20), shared by its two ends:

- the glider at DONE (firmware/src/beacon.rs): its own access point `<BOARD>_glider` on channel 3, its page read
  from its databoard at each request;
- the remote recorder (src/avionics/recorder, an ESP32-C6 on the glider's recorder UART): `<BOARD>_beacon` on a
  channel three up, its page from the glider's last [`Report`]. The glider writes the report as an ordinary
  recorder stream (`beacon.csv`, [`FIELDS`]) every few seconds; the recorder files it as any stream and replays its
  last row, after a restart from its card. The report sets that radio too ([`Radio`]): an empty name keeps it off.

One page for both ([`page`]), so a searcher's phone shows the same window whichever beacon it joined; the report's
row ([`Report::cells`], [`Report::parse`]); the captive portal's DNS answer ([`dns_answer`]); and, with the `esp`
feature, the access point's glue ([`esp`]).

### `pub struct Radio`

What the remote beacon's radio does, as the glider sets it.

### `pub struct Point`

A point in 1e-7 degrees, as the GNSS reads it.

- `fn degrees(self) -> (f64, f64)` — Its degrees, (latitude, longitude).

### `pub struct Height`

One source's height.

### `pub struct Battery`

The battery, where an INA226 measures it.

### `pub struct Backlog`

The recording still to copy from the flash to the card.

### `pub struct Snapshot`

What the page shows, its ages as at the moment it was read.

- `fn aged(mut self, seconds: u64) -> Snapshot` — The same, `seconds` older: a report received that long ago.

### `pub struct Report`

The glider's report to the remote beacon: one beacon.csv row.

- `fn height_texts(&self) -> [String; 2]` — The two height lists as their cells hold them: `source=metres@age` items joined by `,` (`gnss=12.3@4`), a source that never read as `source=`.
- `fn cells<'a>(&'a self, heights: &'a [String; 2]) -> [Cell<'a>; FIELDS.len()]` — The row's cells in [`FIELDS`]' order, the height lists from [`Report::height_texts`].
- `fn parse(payload: &[u8]) -> Option<Report>` — A beacon.csv row read back from its payload, `<uptime>;<cells>` as the recorder printed [`Report::cells`]; None for anything else: the header, a row of another width, a cell that does not read.

### `pub fn page(title: &str, snapshot: &Snapshot, note: Option<&str>) -> String`

The beacon's page: `title` (the access point's name), what `snapshot` holds, and a `note` under the title (the
remote beacon's: how old the glider's report is). It refreshes itself every 5 s.

### `pub fn charge(millivolts: f32) -> Option<f32>`

A 1S LiPo's charge (%) at `millivolts`, interpolated in [`LIPO_CHARGE`]; None above [`LIPO_CEILING_MV`].

### `pub fn dns_answer(query: &[u8]) -> Option<Vec<u8>>`

The answer to one DNS `query` (RFC 1035): its id and question echoed, an A record of [`ADDRESS`] for an A query,
no record for any other type; None for a datagram that is no single-question query.

### Constants

- `const ADDRESS: [u8; 4] = [192, 168, 4, 1]` — The access point's address, the one the ESP-IDF examples and most beacons use: a phone opens it as it is.
- `const FILE: &str = "beacon.csv"` — The report's stream file: its routing is `<session>_beacon.csv`.
- `const FIELDS: [&str; 18] = [` — The report's columns after `uptime` ([`Report::cells`]).

## `beacon::esp`

The beacon's access point on ESP-IDF (the `esp` feature), for both ends: the glider at DONE (firmware/src/beacon.rs)
and the remote recorder (src/avionics/recorder). Each passes its own thread and server settings: the glider's
stacks go to PSRAM on core 0, the C6 has neither PSRAM nor a second core.

The page opens by itself (a captive portal, operator 2026-10-11): the AP's address is [`ADDRESS`], its DHCP server
offers the beacon as the DNS server, a DNS responder answers every name with the beacon, and every other path
redirects to the page, so a phone's connectivity check (Android's /generate_204, iOS's /hotspot-detect.html,
Windows' /connecttest.txt) finds a portal and the phone opens it. No DHCP option 114 (RFC 8910): phones take it
only as an HTTPS captive-portal API (RFC 8908), which a beacon cannot serve. The server's wildcard matching must be
on: without it the probes got 404 (no portal, only "no internet") and the page stayed shut.

### `pub fn start(wifi: &mut BlockingWifi<EspWifi<'static>>, radio: &Radio) -> anyhow::Result<()>`

The access point `radio` names up on `wifi` (a station dropped first; started when it is not), at its TX cap, and
addressed for the portal.

Errors: a name or password too long for the driver, or a driver call that failed.

### `pub fn spawn_dns(thread: ThreadSpawnConfiguration) -> anyhow::Result<()>`

The DNS responder on its own thread (`thread`'s core, priority and stack placement): every A query answered with
[`ADDRESS`], every other one with no record, forever.

Errors: port 53 would not bind, or the thread would not spawn.

### `pub fn serve(server: HttpConfiguration, page: impl Fn() -> String + Send + Sync + 'static) -> anyhow::Result<EspHttpServer<'static>>`

The page server on port 80 (`server`'s task settings, its wildcard matching turned on): `/` answers `page()`,
every other path redirects to it, so the phones' connectivity checks find a portal. It lives as long as the value.

Errors: the server would not start.

# `control` — `src/avionics/crates/control`

_Uses: `board_config`, `databoard`, `sequencer`. External: serde_json._

_Integration tests: `tests/golden.rs`, `tests/golden_airspeed.rs`, `tests/golden_attitude.rs`, `tests/golden_commons.rs`, `tests/golden_control_trace.rs`, `tests/golden_drift.rs`, `tests/golden_fin_limit.rs`, `tests/golden_governor.rs`, `tests/golden_guidance.rs`, `tests/golden_magnetic_yaw.rs`, `tests/golden_mixer.rs`, `tests/golden_navigation.rs`, `tests/golden_on_backup.rs`, `tests/golden_pid.rs`, `tests/golden_wind.rs`._

## `control`

The flight math: one module per src/glider file it ports, f32 throughout (decision D18), and no clock read
anywhere -- a function that needs the time takes it as an argument, so the host tests drive it.

The modules, each named after the plan B file it ports:
 * [`commons`] -- the shared primitives the flight math uses: the clamp, the servo neutral, the fin table and
   the barometric altitude (the stage machine's half of commons.py is in crates/sequencer);
 * [`pid`] -- one control axis: an angle error in, a fin deflection command out, with back-calculation;
 * [`mixer`] -- the three axis commands mixed into a whole-degree angle per fin (elevons and a rudder);
 * [`airspeed`] -- the hybrid airspeed estimate (pitot, accelerometer backbone, GNSS corrector);
 * [`governor`] -- the dynamic-pressure fin governor: the estimate's throttle and the mixer's 1/v^2 cap;
 * [`navigation`] -- the landing-zone geometry the Mission resolves its target and gates with;
 * [`wind`] -- the wind triangle from the GNSS ground velocity, the heading and the airspeed;
 * [`drift`] -- the GNSS pad drift (tasks/gnss_calib.py), frozen at launch to de-bias the wind;
 * [`flight`] -- the flight law (tasks/flight.py): governor, stage gate, guidance, PIDs, mixer, wind feed;
 * [`guidance`] -- the stage-dependent guidance law: the attitude setpoints and the heading error;
 * [`attitude`] -- the primary attitude: the complementary filter (gyro, gravity, track, magnetometer), its
   blind window and the handover to a source above or below it.

And one with no plan B file, the port's own:
 * [`change`] -- a barometer's climb rate, `change`: the least-squares slope of its altitude over half a second.

## `control::airspeed`

_Tested by its own `mod tests`._

The hybrid airspeed estimate the dynamic-pressure fin governor caps the fins by (src/glider airspeed.py;
doc/specs/coludo.md "Fin authority"). Three sources, in order of trust:
 * a PITOT/static differential-pressure reading ([`AirspeedEstimator::measure`]) -- a DIRECT airspeed
   measurement, the preferred source WHEN IN-BAND: it measures the air, not the ground, so no wind offset and no
   attitude gate. It saturates past its full scale (boost, a steep dive), where the caller falls back to the
   accelerometer rather than trust an under-read;
 * accelerometer integration is the BACKBONE ([`AirspeedEstimator::predict`]) -- always running, and the only
   usable source in the boost and right after separation, when the pitot rails and the GNSS jitters;
 * a valid, sane GNSS ground speed nudges out the integrator's drift ([`AirspeedEstimator::correct`]) -- a
   complementary filter used when the pitot is absent or stale: GNSS the slow truth, accel the fast signal.

GNSS is DISTRUSTED by default: rejected without a fix and above a physical ceiling (a 100+ m/s reading at
separation is a glitch), and only ever BLENDED, never a hard replace, so one bad-but-in-range sample cannot jump
the estimate; repeated good fixes pull the drift out. The estimate is biased to over-read when uncertain: a
high airspeed tightens the governor's cap, the safe direction.

CONFIDENCE: a fresh estimator (a cold boot, or a MID-AIR RESET that runs the setup again) reads 0 before
anything charges it, and a 0 airspeed would open the fin cap to the full 45 degrees at high dynamic pressure.
[`AirspeedEstimator::confident`] says whether a TRUSTED speed exists yet: it latches once the accelerometer
charges a clearly airborne speed or the first sane GNSS fix anchors it (a direct set, not a slow blend). Until
then the governor caps conservatively rather than off the uncharged 0.

WARM START: the conservative cap keeps a recovered glider SAFE but blunt -- pinned at the unconfident 30 m/s it
starves fin authority exactly when a glider that just rebooted mid-air needs it. [`AirspeedEstimator::seed`]
closes that window from the warm-start crumb, so the recovery order is pitot, then saved, then GNSS: the crumb
is there at once (no fix to wait for), the accelerometer integrates on from it, and the pitot overrides it on
its first in-band read. GNSS comes last because a fix can take tens of seconds to come back after a reset.

f32 m/s throughout, as plan B ran it (a float path even on MicroPython, where its boxing was amortised by the
governor's throttle). Plan B's `max(0.0, min(x, ceiling))` is written out in Python's comparison order so a
NaN acceleration bounds to 0 as it does there.

### `pub struct AirspeedEstimator`

Fuse integrated acceleration with a sanity-gated GNSS ground speed and a direct pitot reading into one airspeed
estimate, in m/s (airspeed.AirspeedEstimator).

It does not know HOW the acceleration along the path is derived: the caller passes it (the governor uses
|accel| - g), so it stays testable on the host.

- `fn new(ceiling: f32, gnss_gain: f32) -> AirspeedEstimator` — A fresh, unconfident estimator at 0 m/s with this glitch `ceiling` (m/s) and GNSS blend `gnss_gain`.
- `fn value(&self) -> f32` — The current airspeed estimate, m/s.
- `fn confident(&self) -> bool` — Whether the estimate is trustworthy yet (see the module notes): false on a fresh construct or a mid-air reset until the accelerometer charges a clearly airborne speed, a sane GNSS fix anchors it, an in-band pitot reads airborne, or a crumb seeds it. The governor caps conservatively while it is false, so a spurious 0 cannot open the fins to the full 45 degrees at high dynamic pressure. A latch: once true, it stays.
- `fn seed(&mut self, airspeed: f32) -> f32` — Restore a PERSISTED airspeed in m/s, the warm-start crumb's, as the starting estimate (airspeed.seed).
- `fn predict(&mut self, acceleration_along: f32, elapsed: f32) -> f32` — Integrate the net acceleration ALONG the flight path, m/s^2, over `elapsed` seconds: the backbone (airspeed.predict).
- `fn correct(&mut self, gnss_speed: f32, has_fix: bool) -> f32` — Blend toward a GNSS ground speed in m/s ONLY if trustworthy: a live fix and within [0, ceiling] (airspeed.correct).
- `fn measure(&mut self, airspeed: f32, gain: f32) -> f32` — Fold in a DIRECT airspeed measurement in m/s, the pitot's -- the preferred source when available (airspeed.measure).

### Constants

- `const CONFIDENT_SPEED: f32 = 5.0` — The speed in m/s above which an accelerometer-charged or pitot-measured estimate counts as clearly airborne, and so trusted (airspeed._CONFIDENT_MS).
- `const DEFAULT_CEILING: f32 = 60.0` — The default glitch ceiling in m/s (airspeed.AirspeedEstimator `ceiling_ms`): the integral is clamped to it and a GNSS speed above it is rejected.
- `const DEFAULT_GNSS_GAIN: f32 = 0.2` — The default GNSS blend weight (airspeed.AirspeedEstimator `gnss_gain`): each accepted fix moves the estimate a fifth of the way.

## `control::attitude`

_Tested by `src/attitude/tests.rs`._

The PRIMARY attitude filter, from src/glider tasks/attitude.py (`Attitude`): a complementary filter over the
SEN0697 and the LSM6DSO32 (doc/specs/coludo.md "Sensors Fusion/Backup"). It derives (heading, roll, pitch) from
the gyro `rate` (the LSM6DSO32, backed up by the BMI323) and the accelerometer's gravity vector, with the BMM350
magnetometer and the GNSS ground track as its heading references, and publishes it on the `attitude` channel at
PRIORITY 1. It began as the backup to the BNO055; since 2026-10-05 it is the primary on every board, and the
SEN0253's BNO055, where one is still fitted, is the BACKUP below it at priority 2. Priority 0 is the HITL sim's.

Two regimes, chosen each tick by the fused attitude's winner ([`Attitude::tick`]):
 * a source that OUTRANKS the filter is winning (the HITL sim): MIRROR it, so the filter stays warm and the
   handover is seamless the moment that source goes quiet. A source ranked BELOW it (the BNO055) is NEVER
   mirrored, even when it wins a tick because the filter was late: copying it would overwrite the primary's
   estimate with the backup's, and its heading is in another frame (magnetic, not the learned track);
 * otherwise FREE-RUN: integrate the gyro, and while the accelerometer reads about 1 g (a trustworthy gravity
   vector) and the airframe is not turning, pull roll and pitch toward the gravity angles; pull the yaw toward
   the GNSS track when moving, and toward the magnetometer once its offset to the track has been learned.
   Roll and pitch stay gravity-referenced, so the glider holds wings-level and its pitch.

BLIND: with no fresh gyro for [`BLIND_CYCLES`] ticks in a row the filter is blind, on every board and in either
regime; [`Attitude::blind`] feeds the health flag `attitude-blind`, which until it existed was the only thing that
could have said the filter had lost its gyro. What the filter then publishes depends on what is below it:
 * a FRESH source ranked below it (the SEN0253's BNO055): WITHHOLD, so the channel goes stale and the databoard
   hands the attitude to that backup, instead of masking it with an estimate frozen at its last value (roll and
   pitch hold: the accelerometer pull is gated off without a yaw rate);
 * nothing fresh below (every board without a SEN0253): publish as before the backup existed. Withholding there
   would hand the flight law no attitude at all -- neutral fins -- which no flight or HITL run has validated.

Both end on the first fresh gyro sample.

FRESH INPUTS ONLY. Plan B was migrated from `value()` to `read()` for every input: `value()` extrapolates without
bound, so the filter would blend an invented gyro rate or gravity vector and report an attitude with the same
confidence as a real one. A filter written to keep an attitude when another part had failed -- the case where
its own inputs are most likely stale too -- cannot tell fresh from invented. Here every input of [`Readings`] is
present only when a fresh source vouches for it (crates/databoard has no extrapolation at all).

TWO FRAMES. The filter computes in plan B's frame, the one its formulas and the HITL sim speak: the aerospace
body frame, x forward, y RIGHT, z DOWN, rates by the right-hand rule (roll right wing down, pitch nose up, yaw
clockwise seen from above, all positive), and the accelerometer read as the GRAVITY vector (the specific force
it measures, negated). Only there do plan B's roll = atan2(ay, az), pitch = atan2(-ax, |(ay, az)|) and magnetic
heading = atan2(-my, mx) hold; plan B's hitl.py publishes `rate` as (roll, pitch, yaw) rate and the field "in
the consumer's convention" for that reason, which is all the HITL validation covers.

The IMUs, though, are mounted per their MARKS, x forward, y LEFT, z UP (the operator, 2026-10-06), and the
drivers publish in that frame (a part mounted otherwise is mapped into it by its board config's `axes` key,
crates/drivers `axes`). The two frames are a half turn apart about x, so the gyro reads (x, -y, -z) in the
filter's frame, the field (x, -y, -z), and the accelerometer, the half turn negated into gravity, (-x, y, z).
[`Readings::from_marks`] is that map, the ONE place the mounting enters the filter, and [`rate_from_marks`] hands
the flight law's D term the same (roll, pitch, yaw) rate. Fed the marks frame unmapped, the filter would read
the pitch nose-DOWN positive with the gyro and gravity agreeing on the wrong sign (nothing would flag it), turn
the gyro yaw against the GNSS track pull, and run the magnetic heading backwards, so the "constant" offset it
learns against the track would be twice the heading. Plan B had no such map: its drivers publish the raw axes
(its notes say gx -> roll, gy -> pitch, gz -> yaw, and "field calibration flips a sign").

Units: f32 degrees, seconds and g (decision D18). Plan B holds every angle as a centidegree fixnum, takes its
atan2 from an integer CORDIC and its square root from an integer routine, a MicroPython workaround for the
boxed float (doc/specs/avionics-port.md "What is deliberately not ported, or changed"). This port computes the
real-number filter those integers stand for. Integer centidegrees survive only at the telemetry and inspect
boundary ([`Attitude::row`], [`Attitude::inspect`]).

The databoard, the log and attitude.csv are the caller's, as in crates/sequencer: [`Attitude::tick`] reads no
channel and no clock, and answers what to publish and what to log in a [`Tick`]. The caller pushes
`published`, logs `log`, and writes [`Attitude::row`] to attitude.csv on every tick that published, undecimated
(plan B records only what it publishes). What the caller reads and when are pure helpers here too:
[`assemble`] builds a tick's [`Readings`] from the databoard, [`Pace`] says which control ticks step the filter,
and [`on_backup`] answers the health flag `attitude-backup`.

Deliberate differences from plan B, besides the mounting map above:
 * attitude.csv takes a row on every publish: `telemetry_ms` is read but thins nothing. Plan B decimated the
   stream to it (10 rows a second at its default), a MicroPython-era economy; here every measured sample is reported
   (doc/specs/avionics-port.md, "Measured -> reported");
 * plan B's integer error is gone: its pulls are right shifts, which floor, so a pull approaching its target from
   below stalls up to 2^shift - 1 centidegrees short for good (0.15 degree at corr_shift 4, 0.31 at course_shift
   and mag_offset_shift 5, 0.63 at mag_shift 6); its gyro delta floors up to a centidegree a tick and never
   recovers it; its CORDIC is good to ~0.05 degree; its from_float() truncates the accelerometer to the centi-g
   (up to ~0.6 degree) and its integer square root the pitch's denominator (up to ~0.3 degree). Here a pull of
   shift s moves exactly 2^-s of the error and the angles are f32 atan2;
 * the step integrates the elapsed MICROSECONDS. Plan B floors them to whole milliseconds, under-integrating every
   tick by its sub-millisecond part (up to 5 % of a 20 ms tick). The gap rule is plan B's, on whole milliseconds:
   more than 500 or negative takes the nominal period;
 * a reading that is not finite (NaN, an infinity) is treated as absent: one NaN in the state would poison the
   attitude for good, since no pull can bring a NaN back. Plan B's integers cannot be NaN, and its from_float()
   raises on one, which crashes its task;
 * a config key of the wrong type takes its default (a whole-number key needs a JSON integer); plan B stores
   what arrives and raises where it is used. Its truncation of the band edges and gates to the centi-unit
   (int(0.29 * 100) is 28) is gone too: the config number is used as written.

Kept although it is a hazard: the accelerometer pull is NOT wrapped, as plan B's fixed.blend_cd is not. Near
inverted, an estimate of roll 170 and a gravity vector at roll -170 are 20 degrees apart the short way, but the
pull takes the error the long way, -340: at 1/4 per pull the published roll goes 85, 21, -27, -62, -89, -109,
sweeping through wings-level instead of moving 20 degrees. It is pinned by the golden fixture and a unit test, and
raised with the operator as a plan B defect; wrapping it would be a deliberate, documented deviation.

### `pub struct Settings`

The filter's knobs: the attitude component's board-config keys (named in each field's note), with plan B's
`Attitude.setup()` defaults for a missing one ([`Settings::default`]).

- `fn from_json(config: &Value) -> Settings` — The settings an attitude component's config block holds; a key that is missing or of the wrong type takes its default, and so does every key of a block that is not an object.

### `pub struct Orientation`

An attitude, in degrees: the `attitude` channel's value. Plan B's channel carries (heading float degrees, roll
centidegrees, pitch centidegrees); here all three are f32 degrees.

- `fn from_sample([heading, roll, pitch]: [f32; 3]) -> Orientation` — The attitude a sample of the `attitude` channel carries: (heading, roll, pitch), plan B's order.
- `fn to_sample(self) -> [f32; 3]` — This attitude as a sample of the `attitude` channel, for the push: (heading, roll, pitch), plan B's order.

### `pub struct Fused`

The fused `attitude` parameter's FRESH winner this tick, as the caller's databoard read answers it.

### `pub struct Readings`

What one tick may read, each present only when a FRESH source vouches for it (see the module notes).

The vectors are in the FILTER's frame (x forward, y right, z down; see the module notes): a caller holding the
drivers' channels, which speak the marks frame, builds them with [`Readings::from_marks`]; only a source already
in the filter's frame (plan B's HITL sim, the golden fixtures) fills them directly.

- `fn from_marks(rate: Option<[f32; 3]>, acceleration: Option<[f32; 3]>, magnetic: Option<[f32; 3]>) -> Readings` — The gyro rate (degrees per second), the accelerometer (g) and the magnetometer as the drivers publish them, in the MARKS frame (x forward, y left, z up), mapped into the filter's frame; every other reading absent, for the caller to add (`Readings { course, ..Readings::from_marks(rate, acceleration, magnetic) }`). The one place the mounting enters the filter (see the module notes).

### `pub struct Tick`

One tick's answer: what to push and what to log. Plan B pushed and logged on the spot.

### `pub struct MagneticYaw`

The MAGNETIC yaw (plan B `_magnetic_heading` and `_magnetic_yaw`): learn the magnetic-to-track offset while the
GNSS track is good, and use it as an absolute heading when the track is not.

The magnetometer is the only absolute yaw reference that does not need a GNSS fix, and boost is exactly when a
consumer receiver loses lock: without it, the heading runs free through the phase that makes the most drift.

The order matters. While a track is available the magnetometer steers nothing: it is MEASURED against the
track, and the difference -- declination plus the mounting, as one lump -- is low-passed into the offset. The
moment the track goes (a boost dropout, a phase below the course gate), that offset turns the magnetometer into a
standalone reference and the yaw is pulled toward it, as it was toward the track. Two references pulling at once
would fight; one at a time does not.

It learns ONE CONSTANT, which is sound only because the driver removes what is not constant: hard and soft iron
shift the heading by an amount that depends on where the nose points (up to 214 degrees on the bench fixture), so
no single offset can absorb them. bmm350.calibrate() does, from a level circle the operator turns; until then the
device sits on the not-ready list. On an uncalibrated magnetometer this still runs and is still worth having --
a reference that is wrong but stationary bounds the pull better than a free-running gyro -- just much less.

- `fn new(shift: u32, offset_shift: u32, level: f32) -> MagneticYaw` — A magnetic yaw with nothing learned: the pull shift, the offset's learning shift and the level gate in degrees (the settings' `magnetic_shift`, `magnetic_offset_shift` and `magnetic_level`).
- `fn known(&self) -> bool` — Whether the offset has been learned (attitude.csv `mag_known`, inspect `mag_known`).
- `fn offset(&self) -> f32` — The learned offset in degrees, [0, 360) (attitude.csv `mag_offset_cd`, inspect `mag_offset`).
- `fn heading(&self, magnetic: Option<[f32; 3]>, roll: f32, pitch: f32) -> Option<f32>` — The heading from the magnetometer in degrees, [0, 360), or None when it cannot be trusted this tick: no fresh field, or the airframe past the level gate in `roll` or `pitch` (degrees).
- `fn step(&mut self, magnetic: Option<[f32; 3]>, roll: f32, pitch: f32, tracking: Option<f32>, yaw: f32) -> f32` — One tick of the magnetic yaw: `tracking` is the GNSS course in degrees when the track is usable this tick (it takes precedence), `yaw` the filter's yaw in degrees. Returns the yaw, pulled toward the magnetic reference only when the track is gone and the offset is known; while tracking it only learns the offset.

### `pub struct Attitude`

The complementary-filter attitude (plan B `Attitude`): the primary, published at priority 1.

- `fn new(settings: Settings, now_us: u64) -> Attitude` — A filter with these knobs, set up at `now_us` (the first tick integrates from it, and the first accelerometer pull comes an `acceleration_period_ms` after it): level, heading north, nothing learned, not seeded.
- `fn settings(&self) -> &Settings` — The knobs.
- `fn orientation(&self) -> Orientation` — The filter's attitude as it stands after the last tick, whether that tick published it or withheld it.
- `fn blind(&self) -> bool` — Whether the filter is BLIND: no fresh gyro for [`BLIND_CYCLES`] ticks. The health flag `attitude-blind`.
- `fn free(&self) -> bool` — Whether the filter is free-running rather than mirroring a source that outranks it.
- `fn seeded(&self) -> bool` — Whether a source that outranks the filter has ever been mirrored.
- `fn magnetic(&self) -> &MagneticYaw` — The magnetic reference: whether its offset is learned, and the offset.
- `fn tick(&mut self, now_us: u64, readings: &Readings) -> Tick` — One tick at `now_us` (plan B `_tick`): watch the gyro, then mirror a source that outranks the filter or free-run, and answer what to publish and log.
- `fn row(&self) -> [i32; 6]` — The attitude.csv row ([`FIELDS`]): heading, roll and pitch in whole centidegrees, the `free` and `mag_known` flags as 0/1, and the magnetic offset in centidegrees. Integers only, as plan B records them.
- `fn inspect(&self) -> Value` — The operator-facing snapshot (`inspect attitude`; plan B `inspect`, its attitude keys -- the firmware's Inspectable adds the name and health): the flags, the angles as text to the hundredth, and the magnetic reference, which is otherwise invisible -- whether the offset has been LEARNED (the magnetometer steers nothing until it has) and what it converged to.
- `fn stats(&self) -> Value` — The diagnostics (`stats attitude`): the same snapshot as [`Attitude::inspect`], as in plan B.
- `fn probe(gyro_fresh: bool) -> Result<(), &'static str>` — The on-demand self-test (plan B `probe`): the gyro `rate` is present, `gyro_fresh`, the filter's core input. Err carries the operator text, [`PROBE_FAILURE`].

### `pub fn rate_from_marks([x, y, z]: [f32; 3]) -> [f32; 3]`

The gyro rate in the marks frame (degrees per second about x forward, y left and z up, by the right-hand rule) as
the (roll, pitch, yaw) rate the filter and the flight law's D term take: roll right wing down, pitch nose up, yaw
clockwise. About y left the right-hand rule turns the nose DOWN and about z up it turns LEFT, hence two signs.

### `pub fn inspection([heading, roll, pitch, free, magnetic_known, magnetic_offset]: [i32; 6], seeded: bool, blind: bool) -> Value`

The operator-facing snapshot ([`Attitude::inspect`]) from a published [`Attitude::row`] and the two flags the row
does not carry: the firmware's executor makes it from what the control thread publishes as integers, so the
flight core formats no text.

### Constants

- `const BLIND_CYCLES: u32 = 5` — Consecutive ticks with no fresh gyro before the filter is BLIND: 100 ms at the 20 ms period. One or two gyro-less ticks follow every GC pause or stall, and holding the last attitude through those is right; five in a row means the gyro is gone. Plan B's `_BLIND_CYCLES`.
- `const STREAM: &str = "attitude.csv"` — The attitude.csv stream the caller records the filter into (plan B's `Telemetry('attitude.csv', ...)`).
- `const FIELDS: [&str; 6] = [` — The attitude.csv columns, the telemetry contract ([`Attitude::row`] fills them): centidegree integers and 0/1 flags.
- `const GYRO_LOST: &str = "gyro lost -- attitude filter blind"` — The line logged when the gyro has been gone for [`BLIND_CYCLES`] ticks: once per transition, not per tick.
- `const GYRO_BACK: &str = "gyro back -- attitude filter sighted"` — The line logged on the first fresh gyro sample after the filter went blind.
- `const PROBE_FAILURE: &str = "attitude: no gyro rate -- attitude filter blind"` — The probe's failure (plan B `probe`): a dead gyro leaves the filter blind, and the board would fly on the BNO055 backup if one is fitted and on a frozen roll and pitch if not, so it is surfaced before the flight.

## `control::attitude::integration`

_Tested by `src/attitude/integration/tests.rs`._

The attitude filter on the board: the pure helpers its caller needs around [`super::Attitude::tick`].
 * [`assemble`] -- what one step reads from the databoard (plan B `_tick`'s reads): the fused `rate`, `accel` and
   `mag` through the marks map, the fused `attitude`'s winner with its rank, and whether a source below the filter
   is fresh to take over from it;
 * [`Pace`] -- which control ticks step the filter: every `period_ms` on the uptime clock (plan B's task slept its
   period between ticks);
 * [`on_backup`] -- the health flag `attitude-backup` (plan B cc_client `_on_backup`): whether a channel's fused
   winner is outranked by an enabled provider of it.

No clock and no lock: the times are arguments and the databoard's fused reads are lock-free, so the host tests
drive them over a real databoard.

### `pub struct Quantities<'board>`

The databoard quantities one filter step reads, as the firmware's topology holds them.

### `pub struct Pace`

Which control ticks step the filter (plan B's task slept `period_ms` between its ticks): every `period_ms` on the
uptime clock, judged at each tick's wake within half a tick.

So a wake that comes a little early still takes its step, and the steps stay on their grid through the ticks'
jitter: 20 ms at a 10 ms tick is every second tick, never the third. A period at or below the tick steps on every
tick, the fastest the tick can run it. A wake more than half a tick past its due time (a stall, or the first wake,
due at 0) starts a new grid a period on from itself, so the grid's edges fall between the wakes, never on one,
and no burst of catch-up steps follows: the next step comes at least a period less half a tick later. Two steps
are never closer than a period less a tick.

- `fn new(period_ms: u32, tick_us: u64) -> Pace` — The pace of a filter stepping every `period_ms` on a control tick of `tick_us`.
- `fn due(&mut self, now_us: u64) -> bool` — Whether the filter steps on the tick that woke at `now_us`. When it does, the next step is due a period on: from this step's due time while the wake was on time within half a tick, else from the wake itself.

### `pub fn assemble(quantities: &Quantities, rank: i32, now_us: u64) -> Readings`

The readings of one filter step at `now_us` (plan B `_tick`'s reads): the fused `rate`, `accel` and `mag` mapped
from the marks frame ([`Readings::from_marks`]); the fused `attitude`'s fresh winner with the rank its source
registered at; and whether a source ranked below `rank`, the filter's own ([`super::Settings::rank`]), is fresh.
The GNSS course and ground speed stay absent until the GNSS is ported.

Each is the databoard's fused read: the first source fresh within the quantity's shared window, in rank order,
never extrapolated and, for a vector, never reconciled. So the BMI323 carries `rate` and `accel` the moment the
LSM6DSO32 goes stale, and back. A reading that is not finite is handed on as the fused read answers it:
[`super::Attitude::tick`] takes it as absent.

`now_us` is read after the tick's pushes: the databoard takes a push stamped after it as not fresh.

### `pub fn on_backup(config: &Value, channel: &str, source: Option<&str>) -> bool`

Whether `channel` is flying on a BACKUP: whether its fused winner `source` is outranked by an ENABLED provider of
it (plan B cc_client `_on_backup`, the health flag `attitude-backup`).

"On the backup" is degraded only where a better-ranked source exists to have fallen off, so the answer comes from
the config's ranks, not from a name: the attitude filter was the BNO055's backup until 2026-10-05 and is now the
primary, with the SEN0253's BNO055 as ITS backup where one is fitted. A provider configured but disabled (the
layout unfitted it) does not count: it was never going to publish. An enabled one counts whether or not it came
up, so the BNO055 standing in for a filter that failed its setup is a backup. A source the config does not
declare (the HITL sim, `hitl`) is no fallback, and neither is nothing fresh at all (`source` None, a different
problem).

`config` is the active board config as the bring-up read it, after the layout fitted it. Its providers are the
enabled devices with their ranks as the bring-up registers them (crates/board_config `directory` and `provides`),
so the flag ranks exactly as the databoard does: a priority is Python's int and saturates past i32, and a name
given twice takes the later entry's rank, as plan B's walk does. Plan B answers otherwise only on a document it
raises on or validate() refuses: its walk counts a provider with no name, which never comes up, and compares a
float priority, which the registration reads as 0.

True also when `config` is not an object and something is fresh, as plan B answers a config it cannot read: an
unknown never silently hides a real fallback.

## `control::change`

_Tested by its own `mod tests`._

A barometer's `change`: its climb rate in m/s, climb positive -- the least-squares slope of the barometer's
ALTITUDE over its own samples of the last `WINDOW_US` (doc/specs/avionics-port.md M8, "Every barometer's
quantities"). A quantity new beside plan B's four, which published no rate: each barometer's run holds one
[`Change`], pushes what it answers as `change` and writes it as its row's fifth column (crates/controller
`BAROMETER_FIELDS`); M12's sim link derives the simulated barometer's the same way.

The ALTITUDE, not the elevation: the two differ by the ground, so their slopes are one, but a re-zero of the
ground (`calibrate`, `update <baro> {"ground": ...}`, the DONE recovery, a warm start) steps the elevation with no
climb at all. Fed the altitude, `change` never sees one.

A LEAST-SQUARES slope over the window, not the difference of two readings: every sample in the window counts, so
an altitude noise of sigma reads as sigma / sqrt(sum (t - tbar)^2) of slope, the sum over the window's own stamps
in seconds -- the more samples and the wider they spread, the quieter. Each barometer is fitted only against
itself, so the offset between two of them never reads as a climb.

No reading until the window holds `MINIMUM_SAMPLES` samples spanning `MINIMUM_SPAN_US`: two samples are a
difference, not a fit, and a short span multiplies the noise. A gap longer than the window empties it, so a
barometer back from a dropout reads None until it has refilled that much: its samples from before the gap are
never fitted against those after it. A non-finite altitude (NaN, an infinity) is refused with the history kept:
it would poison every slope for a whole window.

The ring is fixed, `CAPACITY` samples, and nothing allocates (the firmware boxes the whole, so the other members
of a port do not pay for it inline). The BMP581's 50 Hz keeps 26 samples in the window, the ICP-10111's ~8 Hz four
or five. A source faster than one sample per 16 ms would keep only its newest 32, spanning less than the window,
and one faster than one per 8 ms never spans `MINIMUM_SPAN_US`, so it never reads a change: no barometer here
runs that fast (plan B's HITL sim publishes at 50 Hz).

Stamps are microseconds of uptime, u64, as the databoard's. The ring keeps each sample's low 32 bits, so a sample
is 8 bytes; the newest stamp is kept whole and a gap is judged on it, so the samples kept are never a second apart
and the low words' wrapping difference is exact across their 71-minute wrap. f32 throughout (decision D18): the
sums run on each sample's time and altitude relative to the newest's, small numbers.

### `pub struct Change`

One barometer's `change`: its samples of the last `WINDOW_US`, in a fixed ring.

- `fn push(&mut self, altitude_m: f32, now_us: u64) -> Option<f32>` — Takes one reading and answers the slope with it.

## `control::commons`

_Tested by its own `mod tests`._

The shared primitives of the flight math (src/glider commons.py): the parts the ported modules use, each
next to the plan B name it ports. Each port adds what it needs here, so commons stays one module, as in plan B.

Plan B's `_upy`/`_opt` variant pairs (bytecode against viper or native) are a MicroPython speed workaround and
are not ported: each primitive is one plain function.

* [`between`] -- plan B's clamp, with Python's comparison semantics (it never panics, unlike `f32::clamp`).
* [`SERVO_NEUTRAL_DEGREES`] -- the zero-deflection fin angle the mixer centres on.
* [`fin_deflection_limit`] -- the dynamic-pressure fin governor's table (doc/specs/coludo.md "Fin
  authority"): the largest fin deflection the airframe takes at a given airspeed.
* [`altitude_m`] -- the barometric altitude every baro driver publishes, from [`SEA_LEVEL_PA`].

The stage machine's half of commons.py (`dwell_step`, `apogee_step`) is in crates/sequencer, module `commons`.

### `pub fn fin_deflection_limit(speed: f32) -> i32`

The largest fin deflection in whole degrees from neutral for an airspeed in m/s (commons.fin_deflection_limit):
the dynamic-pressure governor's table lookup.

The speed truncates toward zero to a whole m/s, as plan B's `int()` does, so 29.99 m/s reads the 29 m/s entry;
a negative speed reads the 0 m/s entry and anything past 80 m/s the last one. The board's
`fins.limit_multiplier` safety dial is the caller's to apply (the governor), never baked into the table.

NaN, which no airspeed estimate produces (the estimator's bound turns it into 0), reads the TIGHTEST entry:
plan B's `int()` raises there instead, and of the two directions an over-read is the safe one (a high airspeed
tightens the cap). Infinities saturate to the matching end.

### `pub fn between(low: f32, value: f32, high: f32) -> f32`

Clamp `value` to [`low`, `high`] (commons.between): `low` if below, `high` if above, else `value`.

Python's comparisons, in plan B's order: unlike `f32::clamp`, it never panics (a NaN bound or `low > high`
simply answers by the comparisons), and a NaN `value` passes through. With `low = -x, high = x` it is a
symmetric clamp; an infinite bound leaves that side open.

### `pub fn altitude_m(pressure_pa: f32) -> f32`

The barometric altitude in metres above the sea-level datum, by the ISA approximation (commons.altitude_m).

One definition, because a second baro is a CROSS-CHECK: two drivers computing altitude with their own copy of
this could differ by an edit rather than by the air, and the disagreement would read as a sensor fault.

A pressure of zero or below is a dead read, not a height: it answers 0.0, as plan B does (test_bmp581). NaN
passes through as NaN, as it does in plan B (`nan <= 0` is false there too).

f32 (decision D18): within 3.4 mm of plan B's f64 over 30-110 kPa (measured). The input alone rounds by
0.0078 Pa near 100 kPa, and the 0.19 power of a ratio near 1 resolves to 6e-8, which the 44 330 m scale turns
into 2.6 mm -- a thirtieth of what the barometers resolve (about 1 Pa, 8.4 cm), where every decision taken on
an altitude is in whole metres. Not worth an f64.

### Constants

- `const SERVO_NEUTRAL_DEGREES: i32 = 90` — The default fin/servo neutral angle in degrees (commons.SERVO_NEUTRAL_DEG): the zero-deflection centre the mixer holds when disarmed or degraded, and the one the HITL sim assumes when it recovers fin deflections. A board may override it through the mixer config `neutral_deg`.
- `const FIN_TABLE_TOP_SPEED: usize = 80` — The airspeed, in whole m/s, where the fin table saturates (commons._FIN_VMAX): well past any speed the glider flies; every faster speed takes the last entry.
- `const SEA_LEVEL_PA: f32 = 101_325.0` — The ISA sea-level pressure in Pa (commons.SEA_LEVEL_PA): the datum of [`altitude_m`], one definition for every baro driver, so two of them cannot disagree about it.

## `control::drift`

The GNSS pad drift (plan B tasks/gnss_calib.py). A stationary receiver's position walks slowly (the satellite
geometry, the ionosphere, multipath), and over a minute on the pad the walk is close to a constant velocity
that the minute of flight keeps. A fixed antenna is not carried by the wind, so on the pad the whole reported
ground velocity is that drift: averaged through SETTING and frozen at launch, it is subtracted from the GNSS
ground velocity before the wind triangle, where it would otherwise read as wind (the flight loop, M13).

One [`DriftCalibrator::pass`] per `period_ms` (1 s): in SETTING the fused speed and course, both fresh, fold into
the mean, restarted once at the first armed pass (arming is the operator saying "on the rail, still"; before it
the airframe was carried out, a walk and no drift); off the pad (BOOSTING on, a warm start into GLIDING included)
the mean freezes when it has `min_samples`, else the drift stays zero. Frozen is final.

One divergence: plan B divides by its count when `min_samples` is 0 and nothing was sampled (the task dies on
the ZeroDivisionError); the port freezes a zero drift there.

### `pub struct DriftCalibrator`

The pad drift calibration's state.

- `fn new(min_samples: u32) -> DriftCalibrator` — A calibrator that freezes a mean of at least `min_samples` samples.
- `fn pass(&mut self, stage: Stage, armed: bool, ground: Option<[f32; 2]>) -> Option<([f32; 2], u32)>` — One pass (plan B's run loop body): `stage` and `armed` as the flight word holds them, `ground` the fused GNSS speed (m/s) and course (degrees) when both are fresh.
- `fn drift(&self) -> [f32; 2]` — The frozen drift, east and north, m/s; zero until the freeze (plan B `drift()`).
- `fn samples(&self) -> u32` — The samples since the restart.
- `fn frozen(&self) -> bool` — Whether the drift froze.

## `control::flight`

The flight law (src/glider tasks/flight.py): the control step that turns the sensed state into fin angles, one
call per control tick, composed of this crate's parts in plan B's order -- the governor (the airspeed estimate
and the dynamic-pressure fin cap, every tick), the stage gate, the attitude, the entry (holds captured, PIDs
reset), the guidance law, the three PIDs (the gyro rate as their D term), the mixer, and the wind estimator fed
once per new GNSS sample with the pad drift taken out.

The caller reads the databoard and the mission (fresh values only: a stale quantity is None) and hands them in
as a [`Step`]; [`FlightLaw::step`] answers the fin angles to write, in the mixer's surface order. No clock, no
allocation per step: the firmware calls it from the control thread.

Units are plan B's real-number law in f32 (decision D18): degrees, seconds, m/s. Where plan B floored a
centidegree command to whole degrees (`cmd // SCALE`), the port floors the degree command, so the fins see the
same whole-degree demand.

### `pub struct Step`

What the control step reads this tick.

### `pub struct FlightLaw`

The flight law's state.

- `fn new(flight: &Value, fins: &Value) -> FlightLaw` — The law of the flight entry `flight` (gains, schedule, guidance, wind) and the config's `fins` section (the mixer, the limit multiplier), as plan B's setup builds it.
- `fn step(&mut self, input: &Step) -> bool` — One control step (plan B `_tick`). Returns whether it actuated: then [`FlightLaw::angles`] are the fin angles to write. Out of a control stage, disarmed, with no fresh attitude, or with the boost still on the rod, it actuates neutral; out of a control stage while nothing engaged it does not actuate at all, so an operator's fin command on the pad stands (plan B returns before `_actuate`).
- `fn angles(&self) -> &[i32]` — The fin angles last actuated, in the mixer's surface order (neutral before any).
- `fn reach(&self) -> Option<crate::guidance::Reach>` — The zone's reach from the last control step's inputs (plan B flight.vitals `reach`), None without a zone.
- `fn landing_turn_radius(&self) -> Option<f32>` — The endgame's turn-radius floor at the airspeed estimate, metres (plan B vitals `r_min_m`).
- `fn seed_airspeed(&mut self, airspeed: f32)` — Seed the airspeed estimate (a warm start's crumb, plan B flight.seed_airspeed), its fin cap with it.
- `fn airspeed(&self) -> f32` — The airspeed estimate, m/s.
- `fn cap(&self) -> i32` — The fin cap the governor set, whole degrees.
- `fn active(&self) -> bool` — In a control stage, the PIDs engaged.
- `fn commands(&self) -> [i32; 3]` — The last per-axis demands [roll, pitch, yaw], whole degrees.
- `fn guidance(&self) -> &Guidance` — The guidance law: its setpoints, heading error, reckoning and reach.
- `fn wind(&self) -> &WindEstimator` — The wind estimator.
- `fn mixer(&self) -> &Mixer` — The mixer: its surfaces, in the order of the angles.
- `fn glide_ratio(&self) -> f32` — The nominal L/D for the reach estimate.
- `fn steps(&self) -> u32` — Control steps run.

## `control::governor`

_Tested by its own `mod tests`._

The dynamic-pressure fin governor (src/glider governor.py; doc/specs/coludo.md "Fin authority"), sibling of
[`crate::pid`], [`crate::mixer`] and [`crate::airspeed`]. It owns the airspeed ESTIMATE (the pitot when in
band, else the accelerometer backbone with the GNSS corrector), the ADAPTIVE THROTTLE that paces it, and the
mixer's authority cap: [`crate::commons::fin_deflection_limit`], which goes as 1/v^2, times the board's
`fins.limit_multiplier` safety dial.

The sensors come in as the caller's fresh [`Readings`] -- plan B injected databoard handles and called
`read()`, never the databoard itself -- and nothing here reads a clock: the caller passes the elapsed time.

WHY A THROTTLE AT ALL. On MicroPython the estimator's update is a float path (a square root, an integral, a
blend), measured at ~22 KB/s of GC-off leak at 100 Hz, so plan B updates it at full rate only where the
estimate cannot be trusted to pace itself (the boost and the active deceleration, a fresh dive), and
elsewhere by the DISTANCE-CONSTANT law: at clamp(speed, floor, ceiling) Hz, one update per ~1 m of TRAVEL.
Probed over 1..60 m/s against the earlier error-adaptive law: exactly 1.00 m per update across 5..50 m/s (the
old law ranged 0.04..1.90 m with a 9.5x step at its 20 m/s full-rate trigger), staleness that scales itself (an
overspeed shrinks its own next interval, so no absolute-speed trigger), and 3.3x less leak in a 30 m/s dive.
Rust has no such leak, but the throttle SHAPES the estimate -- the accelerometer is sampled once per update
and held over the whole accumulated interval, and each GNSS or pitot blend is one per update -- so it is ported
unchanged, as behaviour. The estimator integrates the ACCUMULATED elapsed time, so the cadence never changes
the integral's sum, only how fresh the cap is (the cap persists between updates).

### `pub struct GovernorConfig`

The governor's knobs, resolved once from the flight component's config (governor.GovernorConfig). The fields
keep plan B's meaning; their config keys are named on each.

- `fn new(config: &Value) -> GovernorConfig` — The knobs a flight component config holds (governor.GovernorConfig(config)); a missing key takes plan B's default. A value that is not a number also takes the default: validate() refuses such a config before it gets here, where plan B would fail on it at its first use.
- `fn update_interval(&self, speed: f32) -> f32` — The estimator's update interval in seconds at a speed in m/s: the distance-constant law (governor.GovernorConfig.update_interval).

### `pub struct Readings`

The sensors one governor step may read, each FRESH or None (plan B's injected handles' `read()`, collapsed: a
stale or absent channel is None).

A STALE accelerometer must be None, not its last value extrapolated. It is the estimator's backbone: every
predict compounds into the airspeed the cap comes off, and the checkpoint persists that airspeed across a warm
start. A dead IMU extrapolates toward a steady ~1 g, which reads as level flight and drifts the estimate
toward the ceiling -- safe in DIRECTION (a high estimate tightens the cap), but it defeats the freshness-paced
update and survives a reboot through the crumb. No sample is better than an invented one.

### `pub struct Governor`

Cap the mixer's control authority by the estimated airspeed: torque goes as v^2 (governor.Governor).

[`Governor::step`] each control slice decides full rate or throttled, updates the estimator over the accumulated
time, and writes the deflection cap into the mixer's limit.

- `fn new(config: GovernorConfig, fin_limit_multiplier: f32) -> Governor` — A governor with these knobs and the board's `fins.limit_multiplier` (default 1.0; a smaller one tightens the whole schedule), over a fresh, unconfident estimator.
- `fn airspeed(&self) -> f32` — The current airspeed estimate in m/s: the boost rod gate, the telemetry and the warm-start crumb read it.
- `fn estimator(&self) -> &AirspeedEstimator` — The estimator, read-only: its value and whether it is trusted yet.
- `fn seed_airspeed(&mut self, mixer: &mut Mixer, airspeed: f32)` — Restore the airspeed in m/s the warm-start crumb persisted, and cap the mixer off it AT ONCE (governor.Governor.seed_airspeed).
- `fn step(&mut self, mixer: &mut Mixer, elapsed: f32, full_rate_override: bool, pitch: f32, readings: &Readings)` — One control slice (governor.Governor.step): accumulate `elapsed` seconds and, when due, update the estimator and write the fin cap into the mixer.

### Constants

- `const STANDARD_GRAVITY: f32 = 9.81` — Standard gravity as plan B writes it, m/s^2: the 1 g a level accelerometer reads, taken off |accel| before integrating (9.81, not 9.80665, so the estimate matches plan B's).

## `control::guidance`

_Tested by `src/guidance/tests.rs`._

The stage-dependent guidance law (src/glider guidance.py), sibling of pid, mixer and navigation. It turns
(stage, heading) into the attitude setpoints and the heading error the PIDs chase: the boost rod-vertical
hold, bank-to-turn toward the landing zone, the GPS-degrading heading tiers (fix, dead reckoning, launch
point, blind), the loiter orbit and the endgame holding patterns, and the low final-approach centreline
tracker. The per-stage law is a `match` on the stage, plan B's dispatch table: a new stage is one arm and one
method.

Host-testable by construction: nothing here reads a sensor, a channel or a clock. Plan B injects the Mission,
the governor, the wind estimator and databoard handles, each read with `read()` -> (value, source, age); here
the caller hands their answers in as [`Inputs`], and the time as `now_us`. NOT an extrapolated `value()`: the
laser agl is out of range for most of a flight, and an extrapolation answers a stale channel with confident
fiction -- the failure that once ended a flight at apogee. So an input is present only when a FRESH source
vouches for it (`crates/databoard` has no extrapolating accessor at all).

The results land in slots the caller reads ([`Guidance::roll_setpoint`], [`Guidance::pitch_setpoint`],
[`Guidance::heading_error`]) rather than a returned tuple, as in plan B. That is behaviour, not only a
MicroPython allocation trick: when the boost law holds the fins neutral the slots keep their last values, and
the flight row records them.

Units: the setpoints are f32 degrees. Plan B holds them as centidegree fixnums (fixed.py), truncating toward
zero, a MicroPython workaround (doc/specs/avionics-port.md "What is deliberately not ported, or changed");
here they keep their fraction, at most 0.01 degree from plan B's. The heading error stays INTEGER degrees, as
plan B's: sub-degree precision is
irrelevant to a servo, it is an integer column of the telemetry contract (flight.csv `heading_error`, the
health vitals), and its smoothing filter's rounding rule is part of the law (see [`SteeringFilter`]).

Deliberate differences from plan B (none changes a flight with a valid config):
 * `compute` takes no setpoint argument: plan B passes the stage's setpoint dict through to every law, and no
   law reads it.
 * a law that needs the stage's configured setpoints and finds none answers with zeros; plan B raises
   (KeyError). The flight task only computes a configured control stage, so neither happens in flight.
 * the five knobs whose 0 switches a feature off (`nav_bank_gain`, `land_bank_gain`, `endgame_alt_m`,
   `final_approach_agl`, `steer_filter_shift`) are read by their TRUTH, as plan B's gates read them: a
   present null, false or empty value switches the feature OFF, true flies as 1, and only an absent key
   takes the default (see [`GuidanceConfig::from_json`]). That is plan B's behaviour, not a difference. The
   difference is in what plan B cannot fly: any other knob of the wrong type, and a non-empty text in one of
   those five, takes its default here, where plan B stores it and raises wherever it is used (or flies a
   bool as its 0 or 1); a whole float shift (3.0) is that shift here, where plan B's `>>` raises on a float.
   Plan B's config validate() refuses a non-number for eight of the knobs before a flight.
 * before the first [`Guidance::enter`] the captured heading is north (0); plan B's is None and raises where
   it is used. The flight task enters before it computes.

### `pub enum Heading`

The endgame HOLDING pattern (guidance.Heading), named as the config names it.

The discriminants are plan B's ids. [`Heading::resolve`] turns a config name into one. `Auto` defers to the
Mission (the mission crate's `Mission::endgame_heading`, plan B's mission.endgame_heading), which picks the
pattern from the zone's long/short aspect against [`Heading::OVAL_ASPECT`] and
[`Heading::TWO_LOBES_ASPECT`]: the Mission owns the zone, so it owns that decision, and hands its answer in
with the [`Landing`].

- `fn name(self) -> &'static str` — The config name (plan B `Heading.PATTERNS`).
- `fn resolve(name: &str) -> Heading` — The pattern a config name names (plan B `Heading.resolve`): case-insensitive, and every '-' dropped, so 'o-o' aliases 'oo'. An unknown name falls back to `Auto`, which lets the Mission decide.

### `pub struct Setpoint`

A control stage's configured attitude setpoint in degrees (a `stages` entry of the flight config).

### `pub struct GuidanceConfig`

The guidance knobs, resolved from the flight component's config ONCE (guidance.GuidanceConfig).

One place for the defaults and their reasons. A field keeps its board.config key's meaning; where the field
name differs from the key, the doc names the key. Every default equals what config_default ships (plan B's
test_fallbacks_match_shipped): a partial config must fly the law the panel reports.

- `fn from_json(config: &Value, position_window_ms: u32) -> GuidanceConfig` — The guidance knobs from the flight component's config (`flight` in board.config) and the GNSS channels' databoard window in milliseconds, the default tier-1 freshness gate.

### `pub struct Landing`

What guidance reads of the Mission's landing zone, resolved ONCE per zone (plan B's memoized
`mission.zone_points()`, and the Mission's `endgame_heading()` answer).

The zone never moves in flight, so the caller (the Mission's owner) builds this when the zone is set and hands
it in with every tick's [`Inputs`].

- `fn new(top_left: Position, bottom_right: Position, endgame_heading: Heading) -> Landing` — The landing geometry of the zone with these two diagonally opposite corners, and `endgame_heading`, the pattern the Mission resolves `Auto` to for this zone (the mission crate's `Mission::endgame_heading`). The Mission owns that rule, as it owns the zone; guidance only asks when its config says 'auto'.

### `pub struct Fix`

A GNSS position some source vouches for, with its age.

### `pub struct Inputs`

What plan B's injected handles answer this tick: the caller reads its channels and the Mission and hands the
answers in (see the module notes).

### `pub struct Reach`

Whether the zone is still within gliding reach (guidance.reachability), as the flight panel shows it.

- `fn to_json(self) -> Value` — The health vitals' `reach` object, as plan B's dict reads: `{reachable, margin_m, distance_m, headwind_mps}`.

### `pub struct SteeringFilter`

The steering noise filter (guidance `_filter_error`): an all-integer EMA on the heading error.

The measured heading jitters at the control rate under sensor noise (the navigation target is already
cached at the GNSS rate). The state is in sixteenths of a degree; alpha is 1/2^shift. A GENUINE target change
(a jump over 90 degrees: an overfly flip, a law handover) re-seeds the state, so steering follows at once;
only per-step jitter is smoothed. Shift 0 disables it.

The answer rounds the sixteenths toward ZERO, not toward negative infinity. Floor division rounds a negative
state away from zero (-15 // 16 == -1) and a positive one toward it, which on a symmetric oscillation leaves a
one-sided bias in the STEERING command: measured -0.5 degree of steady output for a +/-8 degree input whose
true mean is zero, whichever way the oscillation starts. Small, but systematic -- it always pushes the same
way, so it does not average out over a flight. Only the final division needed it: a symmetric shift was
measured and changes nothing, and a CONSTANT error the filter was seeded with stays exact. Rust's integer
division truncates toward zero, which is that rule; the shift stays arithmetic (flooring), as plan B's `>>`
is. Same class as the PID integral ratchet: every control path that averages a signed quantity has to say
which way it rounds.

One asymmetry remains, and is ported as it is: the flooring shift moves the state by nothing for a positive
step under 2^shift sixteenths but by at least one sixteenth for a negative one, so a constant approached from
below stalls up to 7/16 short and reads one degree low (seeded at 40, a steady 48 settles at 47), while one
approached from above arrives exactly. Plan B's own test allows that degree.

- `fn new(shift: u32) -> SteeringFilter` — A filter with this EMA divisor power (alpha = 1/2^shift), unseeded.
- `fn reset(&mut self)` — Forget the state: the next error seeds it (a fresh control entry).
- `fn filter(&mut self, error: i32) -> i32` — The filtered heading error in whole degrees for this step's raw `error`: the raw error itself when the filter is off, when it seeds, or on a genuine target change.

### `pub struct Guidance`

The per-stage control law (guidance.Guidance).

[`Guidance::setpoint`] gates the control stages; [`Guidance::enter`] captures the holds on entering control;
[`Guidance::compute`] runs the stage's law and fills the setpoint slots.

- `fn new(config: GuidanceConfig) -> Guidance` — A guidance law with these knobs, before any control entry: level setpoints, no heading error.
- `fn config(&self) -> &GuidanceConfig` — The knobs.
- `fn setpoint(&self, stage: Stage) -> Option<Setpoint>` — The stage's configured attitude setpoint, or None when it is not a CONTROL stage (guidance.setpoint): SETTING, BOOSTING and DONE are not, by default -- no actuation under thrust or on the ground. The flight task holds the fins neutral on None and computes only a stage that has one.
- `fn roll_setpoint(&self) -> f32` — The roll setpoint in degrees the last law left (it keeps its value while the boost law holds neutral).
- `fn pitch_setpoint(&self) -> f32` — The pitch setpoint in degrees the last law left.
- `fn heading_error(&self) -> i32` — The heading error in whole degrees the last law left.
- `fn reckoning(&self) -> bool` — Whether the last navigation pass navigated on dead reckoning, with no usable GNSS fix (flight.csv `reckoning`).
- `fn target_heading(&self) -> Option<f32>` — The heading the steering law aims at, degrees, as last computed (and cached for the navigation period); None before the first navigation pass after a control entry.
- `fn reckoned_offset(&self) -> (f32, f32)` — The dead-reckoned `(east, north)` metres travelled since the last real fix (zero while a fix is fresh).
- `fn min_turn_radius(&self, bank_degrees: f32, airspeed: f32) -> f32` — The tightest coordinated turn, metres, the airframe can HOLD at `bank_degrees` and `airspeed` m/s (guidance.min_turn_radius): R = v^2 / (g tan(bank)); 0 when the airspeed or the bank is unusable (<= 0).
- `fn endgame_bank(&self, airspeed: f32) -> f32` — The steepest bank, degrees, the ENDGAME spiral may hold at `airspeed` m/s, stall-margin bounded (guidance.endgame_bank).
- `fn landing_turn_radius(&self, airspeed: f32) -> Option<f32>` — The endgame turn-radius floor, metres, at the bank the endgame will ACTUALLY hold at `airspeed` (guidance.landing_turn_radius): the precision bound the flight panel reports for a land-short-vs-stretch decision.
- `fn reachability(&self, glide_ratio: f32, inputs: &Inputs) -> Option<Reach>` — Can the glider still glide to the zone from here (guidance.reachability)?
- `fn enter(&mut self, heading: f32, roll: f32, pitch: f32)` — Entering a control stage from a non-control one (guidance.enter): capture the heading to hold blind and the rod-vertical attitude (`roll`, `pitch`, degrees) for the boost hold, and reset the navigation cache, the steering filter and the endgame lobe, so the first controlled step steers fresh. The dead-reckoning seed survives: a re-entry does not forget where the glider last was.
- `fn compute(&mut self, stage: Stage, heading: f32, now_us: u64, inputs: &Inputs) -> bool` — Run `stage`'s law at the glider's `heading` (degrees) and the caller's `now_us`: fill the setpoint slots and say whether the fins may actuate (guidance.compute).

### `pub fn heading_error(target: f32, current: f32) -> i32`

The shortest signed heading error `target - current` in INTEGER degrees, wrapped into [-180, 180], so 350 ->
10 is +20 and not -340 (guidance.heading_error).

The difference is truncated toward zero first (plan B's `int()`), then wrapped (commons.wrap180, which keeps
both +180 and -180 as they are). A NaN difference is no error at all here, where plan B's `int()` raises.

## `control::mixer`

_Tested by its own `mod tests`._

The control-surface mixer (src/glider mixer.py), sibling of [`crate::pid`] and [`crate::governor`]. It maps the
control axes -- roll, pitch and yaw, each a deflection command in degrees -- to a servo angle per fin, for the
airframe's mixing: ELEVONS (the two elerons move together for pitch and differentially for roll) and a RUDDER
(the yaw fin), with a hard +/- limit on the control deflection.

Signs are config (the `surfaces` gains), set during bench alignment: a surface that deflects the wrong way gets
its gain sign flipped. MECHANICAL NEUTRAL (each fin's true zero) is NOT here: it is the servo driver's `trim`,
per fin, so boot, failsafe and control all land on it alike. The mixer commands one COMMON neutral and the
driver offsets each fin to its centre.

The fins take WHOLE degrees, as plan B's sg90 driver does (its PWM path is integer, and its compare-and-set
skips a fin whose angle did not change). Plan B mixed integer commands -- the flight task floored each PID
output to whole degrees first (`// fixed.SCALE`) -- and sg90's `_clamp` rounded whatever reached it with
Python's `round()`, half to even. This mixer takes the f32 commands and rounds each fin's angle the same way,
half to even, where the sum is formed: an integer command mixes exactly as in plan B, and a fractional one
lands on the angle plan B's driver would have set. Whether the flight loop keeps plan B's floor (it biases
every command half a degree low on average, and sends -0.01 degrees as -1) is the flight port's to decide.

Plan B's zero-allocation shapes (the pre-resolved surface tuples, the shared output dict, `actuate` fused into
the mixing loop) exist for MicroPython's GC; here [`Mixer::mix`] rewrites one buffer and allocates nothing.

### `pub struct Surface`

One control surface: its fin's name and how much of each axis command it takes (mixer.py `surfaces`).

### `pub trait Fin`

A fin servo the mixer drives: plan B's "object with set_angle(angle)", the sg90 driver in flight.

NOT YET HERE: plan B's actuate() also called each fin's `settle()` after `set_angle()`, so sg90's slew
coalescing writes a command it folded mid-slew once the horn arrives. That is the servo driver's behaviour, and
it joins this trait with the sg90 port.

- `fn set_angle(&mut self, angle: i32)` — Command this whole-degree angle. The driver adds its mechanical trim and clamps to its own physical range.
- `fn angle(&self) -> Option<i32>` — The whole-degree angle the driver last wrote to its servo -- after its own range clamp, before its trim -- or None before the first: plan B's sg90 `angle` attribute, which [`Mixer::angles`] reads for the operator.

### `pub struct Mixer`

Mix (roll, pitch, yaw) deflection commands into a whole-degree angle per fin (mixer.Mixer).

`angle = round_half_even(neutral + between(-limit, roll_gain * roll + pitch_gain * pitch + yaw_gain * yaw,
limit))`. Each fin's mechanical zero is its driver's trim, applied downstream.

- `fn new(config: &Value) -> Mixer` — The mixer a board's `fins.mixer` config block describes (mixer.Mixer(config)); null or a missing key takes plan B's default.
- `fn neutral(&self) -> i32` — The common neutral angle in whole degrees.
- `fn limit(&self) -> i32` — The control-deflection limit in whole degrees: the config's until the governor's first update, the dynamic-pressure cap after (governor.Governor.cap reads it here too).
- `fn surfaces(&self) -> &[Surface]` — The surfaces, in the order [`Mixer::mix`] answers and [`Mixer::actuate`] expects the fins.
- `fn mix(&mut self, roll: f32, pitch: f32, yaw: f32) -> &[i32]` — The whole-degree angle per surface for these axis commands in degrees (mixer.Mixer.mix, then sg90's round()).
- `fn neutralise(&mut self) -> &[i32]` — The neutral angle of every fin (mixer.Mixer.neutralise): the safe, control-disabled output.
- `fn actuate<F: Fin>(&mut self, fins: &mut [Option<F>], roll: f32, pitch: f32, yaw: f32)` — Mix and command each fin in one pass (mixer.Mixer.actuate).
- `fn missing<'mixer, F>(&'mixer self, fins: &[Option<F>]) -> Vec<&'mixer str>` — The surfaces with no fin to drive in `fins` (mixer.Mixer.missing), in [`Mixer::surfaces`] order.
- `fn angles<'mixer, F: Fin>(&'mixer self, fins: &[Option<F>]) -> Vec<(&'mixer str, Option<i32>)>` — The angle each fin in `fins` was last commanded, by surface name, in [`Mixer::surfaces`] order (mixer.Mixer.angles): the health vitals' `fins`.

## `control::navigation`

_Tested by its own `mod tests`._

Landing-zone navigation geometry, 'heading-to-home' (src/glider navigation.py), sibling of the guidance law
that flies it. The mission's landing zone is a latitude/longitude rectangle given by two diagonally opposite
corners (doc/specs/coludo.md). Its TARGET is the centre; its two GATES are the midpoints of the two SHORTER
sides, so the glider enters along the long axis (the documented "vector to the shortest boundary entrance").
[`steer`] heads for the nearer gate until the glider is inside the zone, then for the centre; [`approach`]
tracks the long-axis centreline low on final.

Equirectangular (flat-earth) math, as in plan B: "not exact but about", which is plenty at zone scale (under a
kilometre or so). [`offset`] is the one geographic primitive; [`distance`], [`bearing`] and [`range_bearing`]
derive from it, and [`advance`] is its inverse.

Positions are integers. A [`Position`] counts 1e-7 degree units, 1.1 cm along a meridian. A float degree
cannot hold that: plan B is single precision on the board, where an f32 ulp at latitude 48 is 3.8e-6 degree
(0.42 m), so a 0.2 m step added to it is exactly zero. The geometry therefore never forms an f32 degree of
latitude or longitude. It subtracts the integers (exact), and only the difference becomes f32 metres; at
zone scale that is a few thousand units, exact in f32 up to 2^24 units (186 km). The one absolute angle
left is the mean latitude a cosine is taken of. Rounding it to f32 moves the cosine by up to about
tan(latitude) x 2e-7 of itself, and never less than the cosine's own rounding (3e-8): the worst measured
is 1.5e-7 between 45 and 50 degrees, 7e-7 between 70 and 75, and 2e-6 between 80 and 85, growing without
bound toward a pole as the cosine itself goes to zero. Only the east-west part of a distance takes it. Over
5 km a distance stays within about 1 mm of plan B's CPython f64 answer at mid latitudes (at most 2.6e-7 of
it up to 50 degrees) and within 5 mm up to 80 degrees, with no f64 at all (decision D18), closer than plan
B's own board gets with its f32 degrees.

SAFETY: the gates are FIXED to the short sides, and [`steer`] always vectors to one (and turns ~180 degrees
back through it on an overshoot) with NO knowledge of what lies beyond any side (trees, the launch pad,
people). So the operator must ORIENT the zone: choose the corners in launch.config so the two short-side
entrances face hazard-free approach corridors and the long sides border the hazards. Aerodynamics (a long
run-in, lower crosswind) and safety (clear corridors) only align when it is laid out that way, and the
firmware cannot verify it (doc/specs/coludo.md "Zone orientation -- an operator safety decision").

Plan B's third leg id, `UNKNOWN` ("not steering", a sentinel steer() never produces), is not ported: no
caller reads it.

### `pub struct Position`

A point on the globe in 1e-7 degree units. Every position takes this integer form, so it keeps its
centimetre at any latitude (see the module notes).

- `fn from_degrees(latitude: f64, longitude: f64) -> Position` — The position nearest these decimal degrees, as JSON carries them (f64, the boundary decision D18 allows).
- `fn degrees(self) -> [f64; 2]` — `[latitude, longitude]` in decimal degrees, for a JSON answer (f64, the boundary decision D18 allows).

### `pub enum Leg`

Which waypoint the heading [`steer`] returns aims at this tick (navigation.TARGET / navigation.GATE).

The discriminants are plan B's ids, so a log line or a row that carries the leg reads the same on both
firmwares.

### `pub struct Steering`

What [`steer`] answers: the heading to fly, the waypoint it aims at, and which kind of waypoint that is.

Plan B returns the same three as a tuple `(bearing_degrees, waypoint, leg)`.

### `pub fn e7_from_degrees(degrees: f64) -> i32`

The 1e-7 degree units nearest `degrees`: how a JSON coordinate (an f64) enters. Rounding to the nearest
unit gives back exactly the seven decimals a launch.config carries.

### `pub fn degrees_from_e7(e7: i32) -> f64`

Decimal degrees of `e7` units, for a JSON answer.

Divides by 1e7, which f64 holds exactly, rather than multiplying by 1e-7, which it does not. The quotient is
then correctly rounded, so 255_143_790 prints as 25.514379, the text it came from.

### `pub fn offset(from: Position, to: Position) -> (f32, f32)`

The metre `(east, north)` offset from `from` to `to` (navigation.offset), the one geographic primitive.

The longitude difference wraps into [-180, 180) degrees as plan B's `(dlon + 180) % 360 - 180` does (Python's
`%` floors, as `rem_euclid` does), so a span across the anti-meridian takes the short way. Coludo flies nowhere
near 180 degrees, but the guard is free. A caller that needs both range and bearing to one point takes
[`range_bearing`], which computes this once instead of twice.

### `pub fn advance(position: Position, east: f32, north: f32) -> Position`

`position` moved by a metre `(east, north)` offset (navigation.advance): the inverse of [`offset`], with the
same flat-earth model.

It exists for DEAD RECKONING: with no GNSS fix the glider propagates its last known position by the air
velocity it can still measure (airspeed along the heading) plus the last wind estimate, so navigation keeps a
moving position instead of a frozen one. Equirectangular like [`offset`], which is exact enough over the
hundreds of metres a flight covers. The longitude is measured at the mean of the old and new latitudes, as
[`offset`] measures it, and closer to a pole than a cosine of 1e-9 it does not move at all (plan B's guard).

The answer is rounded to the nearest unit (5.6 mm at most along each axis). That is the point of the integer
position: plan B on the board adds the offset to an f32 degree, whose 0.42 m ulp swallows a slow glider's
steps whole, which is why its dead reckoning accumulates metres and converts once; here the conversion keeps
its centimetre wherever it is done. Plan B does not wrap the longitude either; a result past the i32 range
(some 35 degrees beyond the anti-meridian) saturates.

### `pub fn compass(east: f32, north: f32) -> f32`

The compass bearing of a metre `(east, north)` offset (navigation.compass): degrees, 0 = north, 90 = east,
clockwise, wrapped into [0, 360).

A tiny negative angle wraps to 360 itself in f32 (as it can in plan B's f64): a caller comparing headings
takes the difference the short way, never the raw value.

### `pub fn bearing(from: Position, to: Position) -> f32`

The compass bearing from `from` to `to` in degrees (navigation.bearing).

### `pub fn distance(from: Position, to: Position) -> f32`

The straight-line distance in metres from `from` to `to` (navigation.distance), equirectangular.

### `pub fn range_bearing(from: Position, to: Position) -> (f32, f32)`

The distance in metres AND the bearing in degrees from `from` to `to`, in one pass (navigation.range_bearing).

The fused primitive for a caller that needs both (the loiter: orbit radius and tangent heading;
reachability: range and the wind's projection): one [`offset`] instead of the [`distance`] and [`bearing`]
pair that would compute it twice.

### `pub fn zone(top_left: Position, bottom_right: Position) -> (Position, Position, Position)`

Resolve a zone rectangle to `(target, gate_a, gate_b)` (navigation.zone): the centre, and the midpoints of
the two shorter sides, so the glider enters along the long axis.

A zone wider (east-west) than tall gates on its left and right edges, a taller one on its top and bottom. An
exact square counts as wide, as plan B's `lon_span >= lat_span` does. Corner order does not matter: the
centre is the mean, the sides are absolute, and the gates sit on the corners' own coordinates, so either
diagonal pair gives the same two side midpoints. The centre is the integer mean of the corners, rounded
toward zero, which is at most half a unit (5.6 mm) from plan B's exact average.

SAFETY: the gates are fixed to the short sides, and nothing here knows what lies beyond them. Orienting the
zone so its entrances face clear corridors is the operator's decision (see the module notes).

### `pub fn zone_aspect(top_left: Position, bottom_right: Position) -> f32`

The zone rectangle's long/short side ratio (navigation.zone_aspect), >= 1; 1.0 when a side is degenerate.

The endgame picks its holding pattern from it: a squarish zone orbits one circle, an elongated strip flies
along its long axis (the mission crate's `Mission::endgame_heading`, which hands guidance its answer).

### `pub fn inside(position: Position, top_left: Position, bottom_right: Position) -> bool`

Whether `position` lies within the zone rectangle, its edges included (navigation.inside); corner order does
not matter.

An integer comparison, so it answers exactly as plan B's comparison of the same positions in degrees does
(dividing by 1e7 keeps the order).

### `pub fn steer(position: Position, top_left: Position, bottom_right: Position) -> Steering`

The heading toward the landing target through the nearer gate (navigation.steer).

Head for the closer short-side entrance until inside the zone, then for the centre. Stateless and
re-evaluated each tick, so the overshoot loop is emergent: a glider that crosses the zone and leaves by the
far side still high finds the gate it just crossed is now the nearer one, turns back (~180 degrees) and
re-approaches through it. No waypoint memory: the spec's "recalculate to the nearest alternative entry and
loop" just happens.

### `pub fn steer_to(position: Position, top_left: Position, bottom_right: Position, target: Position, gate_a: Position, gate_b: Position) -> Steering`

[`steer`] with the zone geometry (target and the two gates) ALREADY resolved (navigation.steer_to).

A caller that steers every navigation tick resolves the per-flight-constant [`zone`] once (the guidance
law's `Landing`) instead of paying for it on each call. [`inside`] still takes the corners: a cheap integer
comparison. Equidistant gates go to `gate_a`, as plan B's `<=` sends them.

### `pub fn cross_track(position: Position, point: Position, heading: f32) -> f32`

The signed perpendicular distance in metres from `position` to the line through `point` along compass
`heading` degrees (navigation.cross_track): positive to the RIGHT of the line, looking along `heading`.

### `pub fn approach(position: Position, top_left: Position, bottom_right: Position, heading: f32, cross_gain: f32, intercept_max: f32) -> f32`

The final-approach heading that TRACKS the zone's long-axis centreline, the strip (navigation.approach).

Used low on final instead of homing to the centre POINT: the glider intercepts the line at up to
`intercept_max` degrees (`cross_gain` degrees per metre off it), then flies down it, so a crosswind is crabbed
out and the touchdown holds the narrow strip. (A banked crab, not a wing-low slip: that would need a
sideslip-capable airframe model, and the residual at strong wind is airframe-bound, not a control gap.)
`heading`, the glider's current one, picks which way along the strip to fly.

### `pub fn approach_to(position: Position, target: Position, gate_a: Position, gate_b: Position, heading: f32, cross_gain: f32, intercept_max: f32) -> f32`

[`approach`] with the zone geometry ALREADY resolved (navigation.approach_to; see [`steer_to`]).

The centreline runs from `gate_a` to `gate_b`; when that direction is more than 90 degrees off `heading` the
glider flies it the other way, the along-strip direction it is already going. Off to the right of the line it
aims left to intercept (and vice versa), capped at `intercept_max`.

### Constants

- `const METRES_PER_DEGREE: f32 = 111_320.0` — Metres per degree of latitude, and per degree of longitude times cos(latitude) (src/glider `commons.M_PER_DEG`).

## `control::pid`

_Tested by its own `mod tests`._

The PID controller of one control axis (src/glider pid.py), sibling of [`crate::mixer`]: an angle error in,
a fin deflection command out, both in degrees. One instance per axis (roll, pitch, yaw); [`Pid::reset`] on
(re)entering a control stage.

Anti-windup is TWO mechanisms, as in plan B:
 * the integral and output CLAMPS bound the magnitude, and follow the governor's live fin cap through
   [`Pid::set_limit`], so the integral cannot wind up to a stale 45 degrees behind a tight 5 degree cap and
   dump a saturating deflection when the cap reopens;
 * BACK-CALCULATION bleeds the integral by whatever demand the saturated fin could not fly, and stops at
   zero rather than cross it (see [`Pid::step`], which also records where that guard does not hold).

f32 degrees and seconds (decision D18). Plan B ran this law in fixed point (fixed.py fixnums: centidegree
errors and outputs, integer-millisecond time, gains scaled by 100) for one reason: MicroPython boxes a heap
float on every float operation, and with the GC disabled in flight the old float PID leaked 176 B a step,
about 56 KB/s over three axes at 100 Hz. Rust allocates nothing either way, so this computes the real-number
law those integers stood for. What the fixed point added on top is NOT ported, because it is quantisation and
not behaviour:
 * every result floored to 0.01 (degree, degree-second, degree per second): up to 0.0141 degrees of output a
   step against this port, measured over the golden fixture pid.json (its tolerance is the plan's 0.02, with
   gains up to 3 / 0.4 / 0.4); the port's own f32 rounding adds nothing measurable at that scale;
 * gains truncated to 0.01 by `int(gain * 100)`: `kp=0.29` flew as 0.28, since 0.29 x 100 is 28.999999999999996
   in binary, and any gain under 0.01 as 0;
 * the integral ceiling of 2^28 centidegree-milliseconds (2684 degree-seconds) under the "unbounded" default,
   a small-int bound no flight reaches (the limit follows the 45 degree cap);
 * the floor of a positive integral under 0.01 degree-seconds to zero, which made the stop-at-zero guard
   below take its wrong branch and wind the integral AGAINST the error (plan B: a 0.1 degree error at kp=100
   against a 5 degree cap drove it to -1.25 degree-seconds in one step). The real law keeps that integral
   positive and stops it at zero.

Plan B's `dt_ms` was `dt_us // 1000`, so a sub-millisecond step counted as no time at all; here the elapsed
time is the caller's seconds, as measured. Nothing reads a clock: the caller passes the time.

### `pub struct Gains`

The three gains of one axis (pid.Pid `kp`, `ki`, `kd`): output degrees per degree of error, per
degree-second of integral, and per degree per second of rate. Zero by default, as in plan B, and every
shipped config leaves the `gains` block empty, so ki is 0 in flight today.

### `pub struct Pid`

A PID controller for one control axis: error (degrees) -> fin deflection command (degrees).

`kp * error + ki * integral + kd * derivative`, each part clamped. The derivative is the measured rate (the
gyro) when the caller has a fresh one -- derivative-on-measurement, clean and with no kick when the setpoint
steps -- else the change of the error over the elapsed time.

- `fn new(gains: Gains, integral_limit: f32, output_limit: f32, anti_windup_shift: u32) -> Pid` — A controller with these gains and limits (pid.Pid).
- `fn reset(&mut self)` — Clear the integral and the derivative history (pid.Pid.reset).
- `fn set_limit(&mut self, limit: i32)` — Retune the output clamp and the integral clamp to the live fin authority, in whole degrees (pid.Pid.set_limit).
- `fn integral(&self) -> f32` — The integral's state in degree-seconds: how much accumulated error the ki term is acting on. For the tests and the operator's view of wind-up; the control law never needs to read it.
- `fn step(&mut self, error: f32, elapsed: f32, rate: Option<f32>) -> f32` — One control step (pid.Pid.step): the fin command in degrees for this error, clamped to the output limit.

### Constants

- `const ANTI_WINDUP_SHIFT: u32 = 2` — The back-calculation strength plan B ships (pid._ANTI_WINDUP_SHIFT): each saturated step bleeds the integral by 1/2^2 = 1/4 of the demand the fin could not fly, so it eases out rather than snapping.

## `control::wind`

_Tested by its own `mod tests`._

Wind estimation from GNSS (src/glider wind.py). The MINIMAL method, proven first: the WIND TRIANGLE.

The GNSS reports the ground velocity (course and ground speed); the attitude gives the heading; the governor
gives the airspeed. The air mass the glider flies through is moving, so
  wind = ground velocity - air velocity = (ground speed along the course) - (airspeed along the heading).
An EMA smooths the per-fix estimate. The CROSSWIND component is bias-free; the ALONG-heading component inherits
the governor's airspeed error (an over-read biases the head/tailwind), which is acceptable for the rough uses
(the reachability margin, the approach crab, dead reckoning). If the field data shows that bias hurts, the
airspeed-free GPS-only min/max-ground-speed method is the next layer (kept out until a corner case earns it).

The estimator OWNS its config subtree (the board `wind` block) and is operator-facing, as plan B's
Inspectable is: [`WindEstimator::inspect`] publishes `triangle_alpha` and the physical envelope
(`calm_speed`..`max_speed`), [`WindEstimator::update`] retunes them live, [`WindEstimator::stats`] reports
the estimate. It is fed once per GNSS fix, off the hot loop, through [`WindEstimator::observe`].

Deliberate differences from plan B (the same ones crates/mission makes at the JSON boundary):
 * an `update` value that is not a number (text, a bool, null) is ignored and not listed as changed. Plan B
   stores whatever arrives: a text `max_speed` then raises on the next fix, and `True` flies as 1;
 * the changed names come in key order, because serde_json's map is sorted; plan B lists them in the order the
   request named them;
 * the tunables read back as JSON floats. Plan B echoes the JSON number it was given, so a `max_speed` set as
   `12` reads back `12` there and `12.0` here: the same value of another JSON kind.

### `pub struct WindEstimator`

Estimate the wind (east and north, m/s) from the GNSS ground velocity against the air velocity
(wind.WindEstimator): the wind triangle, EMA-smoothed.

- `fn new(config: &Value) -> WindEstimator` — An estimator bound to the board's `wind` config block: `triangle_alpha`, `calm_speed` and `max_speed`.
- `fn observe(&mut self, course: f32, ground_speed: f32, airspeed: f32, heading: f32)` — Fold in one GNSS fix: this sample's wind triangle, EMA-blended into the estimate (wind.py `observe`).
- `fn components(&self) -> (f32, f32)` — The wind as `(east, north)` m/s, the direction it blows TOWARD; `(0, 0)` below the calm floor (wind.py `components`). What dead reckoning adds to the air velocity, and what reachability projects.
- `fn speed(&self) -> f32` — The wind speed in m/s, 0 below the calm floor (wind.py `speed`).
- `fn direction(&self) -> f32` — Where the wind blows FROM, the meteorological convention, in degrees: 0 = from the north, clockwise, in [0, 360]; 0 when calm (wind.py `direction`).
- `fn inspect(&self) -> Value` — The operator-facing snapshot (`inspect wind`; wind.py `inspect`): the tunables, the speed to the centimetre per second and the direction to the degree, as plan B rounds them.
- `fn update(&mut self, properties: &Map<String, Value>) -> Vec<&'static str>` — Apply a live retune (`update wind`; plan B's Inspectable `update`): each writable property whose number differs from the current value is set. Returns the names actually changed (see the module notes for the order and the values that are ignored).
- `fn stats(&self) -> Value` — The diagnostics for the wind soak and telemetry (`stats wind`; wind.py `stats`): the method ('triangle' once a fix was folded in, else 'none'), the estimate, and the raw EMA components before the calm floor.

# `flash_ring` — `src/avionics/crates/flash_ring`

_Uses: `recorder_wire`. External: none._

_Integration tests: `tests/concurrent.rs`, `tests/power_cut.rs`, `tests/ring.rs`._

## `flash_ring`

The flash ring (doc/specs/avionics-port.md, "Storage design"): the 20 MiB `log` partition as 5120 blocks of one
4 KiB erase sector each. Power can drop at any moment, so getting every record into flash fast is the critical
path, and the SD copy comes second. Three firmware tasks share the partition, each in one role:
* the ring_writer ([`Ring`]) appends chunks of binary records, verbatim, into its open block, and opens the next
  erased block (its [`header`] first) when a chunk does not fit;
* the ring_eraser ([`Eraser`]) erases blocks the saver has passed, and when fewer than about 1 % of the blocks
  are free it erases the oldest block even unsaved (newest wins), non-flight blocks first;
* the ring_saver reads blocks in sequence order ([`BlockTable::next_block`]) and walks their [`chunk`]s, to
  print them for the SD. It lives in crates/recorder (`Saver`), beside the record decoder it prints with: this
  crate cannot depend on that one, nor on `std::fs`. The eraser erases behind the position it publishes.

They coordinate through the [`BlockTable`], one pair of AtomicU32 per slot that the bring-up [`scan()`] builds,
and through two published positions, which the firmware keeps (each two AtomicU32 under a seqlock: the P4 has
no 64-bit atomics). Nothing here takes a lock. The writer acts only on erased slots and its own open one, and
the eraser only on the others, so the two never act on the same slot; the flash driver serialises the calls.

A flash error never wedges a task on one slot: the writer hands a slot it cannot read to the eraser and opens
the next one, and the eraser gives up a slot whose erase keeps failing ([`ERASE_ATTEMPTS`]) and erases the next
one by its rules. One bad sector costs one slot, never the logging.

Builds meet each other's blocks: after an update, or when a reboot in flight comes back on an older image. The
block header and the 4 KiB slot grid are therefore frozen across builds, and everything behind the header is
versioned ([`header`]).

The crate ports no src/glider module (plan B sent text over a UART to the Luckfox), so the spec is the plan.
Everything runs over the [`Flash`] trait: the firmware plugs in `esp_partition_*`, and the host tests run the
same code over [`MemoryFlash`], an in-memory NOR flash that cuts power on demand. `no_std` + `alloc`.

### `pub trait Flash`

A NOR flash region (the ring's partition); offsets are bytes from the region's start.

The ring relies on NOR semantics, and [`MemoryFlash`] enforces them. A program only clears bits (1 -> 0), so it
only ever lands on erased bytes. Only an erase sets bits back, one whole 4 KiB sector at a time. Each task holds
its own handle to the same region (on the board a partition pointer, on the host a `&MemoryFlash`).

- `fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), Self::Error>` — Reads `buffer.len()` bytes at `offset`.
- `fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>` — Programs `data` at `offset`, 1 -> 0 bits only and in address order. The ring makes one call per block header and one per chunk, its header first, so a cut can tear only a call's tail.
- `fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error>` — Erases the sector at `offset` (a multiple of [`BLOCK_BYTES`]) back to 0xFF.
- `fn size(&self) -> u32` — The region's size in bytes.

### `pub struct Position`

A place in the ring: a block by its global sequence number, and a byte offset inside it. The writer publishes
the end of what it has programmed ([`Ring::position`]), and the saver how far it has saved. Positions order by
block first, so a later position compares greater.

### `pub enum Error<E>`

Why a ring call failed.

### Constants

- `const BLOCK_BYTES: u32 = 4096` — A block: one NOR erase sector. The eraser erases one per step, because a 4 KiB erase (13-17 ms) fits the writer's power-cut bound and a 64 KiB one (95-107 ms) does not.

## `flash_ring::backlog`

_Tested by its own `mod tests`._

`saver-behind`'s judge (doc/specs/avionics-port.md "The ring_eraser and newest wins", Degraded tokens: SD mounted
and `ring_unsaved_s` growing for more than 60 s). The firmware looks at the flash-to-SD backlog
([`crate::Figures::unsaved`]) at every `health` read and asks [`BacklogTrend::behind`].

The backlog is a sawtooth, not a line: it climbs while the saver collects a batch and drops by up to a batch (64
blocks) when the batch goes to the card. So a fall between two looks says nothing: a card slower than the writer
still commits a batch now and then, and the backlog still grows across a minute. What tells growth from noise is
the bottom of the sawtooth, the backlog just after a commit: each window of [`SAVER_BEHIND_MS`] keeps its lowest
look, and the saver is behind when that low is no lower than the previous window's and the saver never caught up
with the writer in the window. The health reads come every 2 s from the hub, so a window's low misses a commit's
bottom by at most 2 s of writing (8 blocks at the flight rate), where a batch is 64.

It judges the block count, of which `ring_unsaved_s` is the count at the measured write rate: a change of rate
alone is not the saver falling behind.

### `pub struct BacklogTrend`

The flash-to-SD backlog followed from one look to the next, a window of [`SAVER_BEHIND_MS`] at a time.

Behind, when over a whole window the backlog's low did not fall below the previous window's and the saver never
caught up: a dead card, a card slower than the writer, a saver that never started. A saver that keeps up catches
up ten times a second, and one draining a backlog at the card's speed lowers the low from window to window. The
verdict holds until the next window ends, or until the saver catches up.

- `const fn new() -> BacklogTrend` — A trend with no look yet.
- `fn behind(&mut self, unsaved: u32, caught_up_ms: Option<u32>, now_ms: u32) -> bool` — Takes a look at the backlog, and says whether the saver is behind.

### Constants

- `const SAVER_BEHIND_MS: u32 = 60_000` — How long a window of the backlog's trend is, ms: the spec's 60 s.

## `flash_ring::chunk`

_Tested by its own `mod tests`._

The chunk (doc/specs/avionics-port.md, "Block and chunk format"): what one writer program call appends to a
block, at an 8-aligned offset behind the block header. The next chunk starts at `align8(offset + 8 + length)`;
the padding in between is never programmed (bench gate G1 proved partial-page programs safe, so chunks stay
unpadded).

| off | type | field |
|---|---|---|
| 0 | u16 | length = payload bytes; `0xFFFF` = the erased tail, no chunk here |
| 2 | u16 | record count |
| 4 | u32 | CRC over bytes 0..4 and the payload (zlib CRC-32) |
| 8 | … | payload: whole records (spec "Producer side"), verbatim |

A record never straddles chunks or blocks, so every whole chunk decodes on its own. The records themselves are
the recorder's codec; to this crate a payload is bytes and a count.

[`chunks`] walks a block for the saver, the `/ring` exit and the tools. A chunk whose CRC fails, or whose
length runs past the block, is **torn**: a program call that a cut or a flash error stopped part way. After a
cut it is the last chunk of its boot's last block, since a new boot never appends to an old block. After a
failed call without a cut, the writer retries the same records at the next offset (spec "The ring_writer", step
7), so a whole chunk may follow it. The walk therefore steps over a torn chunk in 8-byte steps until the next
whole chunk or the erased tail: the CRC tells a whole chunk from the debris. [`Torn::followed`] says which of
the two it was, so a retried program is not logged as a loss.

### `pub enum ChunkError`

Why bytes are not a whole chunk.

### `pub struct Chunk<'a>`

One whole chunk found in a block.

### `pub struct Torn`

A torn chunk: bytes at a chunk offset that are neither a whole chunk nor the erased tail.

Whether its records are lost depends on what tore it. A cut tears only the last chunk of its boot's last block
(a new boot never appends to an old block), and those records are lost. A failed program is retried by the
writer at the next offset, so its records are in a later chunk: of the same block when a whole chunk follows
the debris ([`Torn::followed`]), else of the boot's next block, since a retry that does not fit closes the
block. The saver's rule (spec "The ring_saver") is therefore: a torn chunk is a loss, logged as `block
<sequence>: torn chunk at <offset>, k records lost`, only when nothing whole follows it in its block and no
later block of the same boot (boot id and session) is in flash.

### `pub enum Walked<'a>`

What a block walk finds at a chunk offset.

### `pub struct Chunks<'a>`

The chunks of a block, in address order. See [`chunks`].

### `pub const fn chunk_bytes(payload_length: usize) -> usize`

The bytes a chunk with this payload takes in its block: the header, the payload, and the padding up to the
next 8-aligned offset (left erased).

### `pub fn encode(chunk: &mut Vec<u8>, payload: &[u8], records: u16) -> Result<(), ChunkError>`

Writes the chunk of `payload` and its `records` into `chunk` (cleared first): the bytes one program call puts at
the chunk's offset. Returns `ChunkError::Length` when the payload is longer than [`MAX_PAYLOAD`], writing
nothing.

### `pub fn decode(bytes: &[u8]) -> Result<(u16, &[u8]), ChunkError>`

The chunk at the start of `bytes`: its record count and payload, with the CRC checked and the payload inside
`bytes`. Pass the rest of a block, so a chunk running past the block's end fails with `Length`.

### `pub fn chunks(block: &[u8]) -> Chunks<'_>`

The chunks of `block` (its bytes from offset 0, the header included): every whole chunk and every torn one,
in address order, from the first chunk offset up to the erased tail.

Pass the whole block for a closed one. For the writer's open block, pass the bytes up to its published
position, which the walk then treats as the block's end: nothing is read past it.

### Constants

- `const CHUNK_HEADER_BYTES: usize = 8` — The chunk header's size: length, record count, CRC.
- `const ALIGNMENT: usize = 8` — Chunks start on this alignment, so a walk that lost its place finds the next chunk in 8-byte steps.
- `const MAX_PAYLOAD: usize = BLOCK_BYTES as usize - HEADER_BYTES - CHUNK_HEADER_BYTES` — The largest payload: a block less its header and one chunk header (4032 bytes). One such chunk fills a block.

## `flash_ring::eraser`

_Tested by its own `mod tests`._

The ring_eraser's side of the ring (doc/specs/avionics-port.md, "The ring_eraser and newest wins"): the only
task that erases the `log` partition, one 4 KiB block per [`Eraser::erase_step`] (13-17 ms; a 64 KiB erase is
never issued). Its thread (core 0, priority 3) calls the step again and again, paced by the saver's progress
and the erased reserve, on the pad and in flight alike.

Each step erases the first of:
1. a **foreign** slot: it holds nothing recoverable, so no loss. The search runs from the writer's slot on, so
   on an unformatted ring the slot the writer needs next is the first one formatted;
2. a **saved** block: one the saver passed at least [`ERASE_DELAY_MS`] ago (the normal rule), or one behind
   the previous boot's saver position when the bring-up trusts it ([`Eraser::new`]);
3. under **newest wins**, once no more than [`crate::Geometry::reserve_blocks`] slots are erased or saved: the
   **oldest** block even unsaved, a non-flight one while any is left, flight blocks only after them. Each is a
   loss: the step returns it for the log line and counts it in the table. Starting at the reserve rather than
   below it keeps one block of margin for the block the writer opens before the next step.

It never erases the writer's open block, nor an erased slot (which is where the writer opens its next block).
A slot whose erase failed [`ERASE_ATTEMPTS`] times in a row is given up until the next boot and passed over,
so one bad sector never stops the eraser.

### `pub struct Loss`

An unsaved block newest wins erased: the log line `<uptime> ring :: block <sequence> (boot <id>, session <s>,
flight <0|1>) erased unsaved` is made from it, its message by [`Loss`]'s `Display`.

### `pub enum Erasure`

What one [`Eraser::erase_step`] did.

### `pub struct Eraser<'table, F: Flash>`

The ring as its one eraser sees it.

- `fn new(flash: F, table: &'table BlockTable, saved: Option<Position>) -> Result<Eraser<'table, F>, Error<F::Error>>` — The eraser of this boot, after the bring-up scan.
- `fn erase_step(&mut self, saver: Option<Position>, now_ms: u32) -> Result<Erasure, Error<F::Error>>` — One step: erases at most one block, by the rules in the module doc.

### Constants

- `const ERASE_DELAY_MS: u32 = 100` — How long a block stays after the saver has passed it, before the normal rule erases it: 100 ms. A saver position must stay published this long before it counts, so a block is erased 100-200 ms after the saver passed it (spec "The ring_eraser and newest wins").
- `const ERASE_ATTEMPTS: u8 = 3` — How many erases in a row may fail on one slot before the eraser gives the slot up for the rest of the boot: 3.

## `flash_ring::geometry`

_Tested by its own `mod tests`._

The ring's shape: how many 4 KiB blocks (slots) it has, where slot `k` lives, and the erased reserve. A block
is one erase sector, so the shape is a single number.

### `pub struct Geometry`

How many blocks the ring has. The board's is [`Geometry::BOARD`]; the tests use small ones, so that a power cut
can land on every flash operation of a workload within seconds.

- `fn new(blocks: u32) -> Option<Geometry>` — A geometry of `blocks` slots. Returns None below two blocks (the writer fills one while the eraser frees another), or past 4 GiB, which a u32 offset cannot address.
- `fn blocks(self) -> u32` — The number of blocks (slots).
- `fn bytes(self) -> u32` — The whole ring's size in bytes.
- `fn slot_offset(self, slot: u32) -> u32` — The flash offset of a slot's first byte.
- `fn reserve_blocks(self) -> u32` — The blocks the eraser keeps free: about 1 % of the ring, at least one. 61 blocks on the board (~245 KB, about 15 s at the flight rate). Fewer erased slots is `ring-low`, and the arm gate does not pass (spec "The ring_eraser and newest wins"). The operator decided it is a constant with no config key.

## `flash_ring::header`

_Tested by its own `mod tests`._

The block header (doc/specs/avionics-port.md, "Block and chunk format"): 56 little-endian bytes at a block's
offset 0, programmed once, with one program call, when the writer opens the block.

| off | type | field |
|---|---|---|
| 0 | `[u8; 4]` | magic `F7 43 42 31` |
| 4 | u8 | version = 1 |
| 5 | u8 | flags: bit0 flight (opened while the stage was BOOSTING..DONE) |
| 6 | u16 | reserved 0 |
| 8 | u32 | block sequence (global: +1 on every block opened, across boots) |
| 12 | u32 | boot id (0 = uncounted) |
| 16 | `[u8; 32]` | session label, ASCII, zero-padded |
| 48 | u32 | reserved 0 |
| 52 | u32 | header CRC over bytes 0..52 (zlib CRC-32; `esp_rom_crc32_le` matches) |

The header proves itself with its CRC, so a header torn by a cut, or the bytes a cut erase leaves, never reads
as a block. The reserved fields are written 0 and ignored on read: the CRC already protects them.

**The header and the slot grid are the same in every layout version.** A later version may change the chunks
and the records behind the header, never these 56 bytes (apart from the version byte and the reserved fields),
nor the 4 KiB grid of slots they sit on ([`crate::BLOCK_BYTES`]). That rule is what lets a build meet another
build's blocks (after an update, or when a reboot in flight comes back on an older image) and still order them
by sequence, keep them, and give up their flight blocks last. It cannot decode their chunks, so it never saves
them, and the eraser takes them only under newest wins ([`crate::Slot`]).

The rule holds only while [`VERSION`] tells the layouts apart: a build reads a block of its own version with
its own codecs, and prints what it reads to the SD with valid line CRCs. So the version covers everything
behind the header, owned by two crates: the chunk format (this crate's [`crate::chunk`]) and the record
format (crates/recorder, spec "Producer side": the kinds, the cell tags, the LOG layout). A change to either
bumps it. Each side pins its bytes to the version in a test: this crate a byte-exact chunk, the recorder a
byte-exact record of each kind, so a format change fails until the version moves with it.

### `pub struct Session`

A session label as the block header holds it (doc/specs/avionics-port.md "Row, log and index text"): the
session prefix of every file of the boot, which names the SD directory the saver copies the block into.

It is 1 to 32 bytes of ASCII letters, digits, `-` and `_`. That takes all three forms of the prefix: a
`recorder.session` label, the boot id as `%06u`, and `YYYYMMDD_HHMMSS_<6 digits>`. It never holds a `/`, a
`.` or a `;`, so it is safe as a directory name and in a wire line. The alphabet is part of the frozen header.

- `fn new(label: &str) -> Option<Session>` — The session `label`. Returns None when it is empty, longer than 32 bytes, or holds a byte outside `[A-Za-z0-9_-]`.
- `fn as_str(&self) -> &str` — The label.

### `pub enum HeaderError`

Why 56 bytes are not a block header.

### `pub struct BlockHeader`

A block header's fields. The CRC is derived from the bytes, so [`BlockHeader::encode`] writes it and
[`BlockHeader::decode`] checks it; it is never held.

- `fn encode(&self) -> [u8; HEADER_BYTES]` — The 56 bytes the writer programs at the block's offset 0.
- `fn decode(bytes: &[u8]) -> Result<BlockHeader, HeaderError>` — The header at the start of `bytes`, checked for magic, CRC and session label, of any layout version: the header is the same in all of them. The caller compares [`BlockHeader::version`] with [`VERSION`] before it walks the chunks.

### Constants

- `const MAGIC: [u8; 4] = [0xF7, 0x43, 0x42, 0x31]` — The block magic `"\xF7CB1"`: 0xF7 never occurs in UTF-8, and the magic is neither all-0 nor all-1, so neither erased flash, programmed-out flash, nor MicroPython-era text reads as a block.
- `const VERSION: u8 = 1` — The layout version this build writes and decodes: the chunk format and the record format behind the header (see the module doc). Bump it on any change to either; the tests that pin those bytes say so.
- `const HEADER_BYTES: usize = 56` — The header's size. A block's first chunk starts here (8-aligned).
- `const SESSION_BYTES: usize = 32` — The session label field's size: the longest label the block header holds.

## `flash_ring::memory`

_Tested by its own `mod tests`._

An in-memory NOR flash for the host: the tests run the ring over it, and a host tool can build a ring image
with it. It enforces NOR's rules, so a ring bug fails a test instead of passing on forgiving memory:
* a program refuses any 0 -> 1 bit change, and leaves the flash as it was;
* an erase takes a whole 4 KiB sector;
* a call out of range fails.

The writer and the eraser each hold their own handle on the board, so here both borrow one `MemoryFlash`:
[`Flash`] is implemented for `&MemoryFlash` as well, over interior mutability.

It also fails on demand. [`MemoryFlash::cut_power`]: after a set number of program and erase calls, the next
one is torn (only its first bytes take effect) and every later call fails, as on a dead board; the test then
boots again over [`MemoryFlash::into_image`]. [`MemoryFlash::fail`] tears one call the same way but leaves the
flash alive, as a flash error the writer retries.

### `pub enum MemoryError`

Why a [`MemoryFlash`] call failed.

### `pub struct MemoryFlash`

A NOR flash in RAM, with operation counters and an optional fault.

- `fn new(size: u32) -> MemoryFlash` — A fully erased flash of `size` bytes.
- `fn from_image(image: Vec<u8>) -> MemoryFlash` — A flash holding `image`, such as the image of a flash whose power was cut, to boot again over.
- `fn image(&self) -> Vec<u8>` — A copy of the flash content.
- `fn into_image(self) -> Vec<u8>` — The flash content, to boot again over or keep.
- `fn cut_power(&self, operations: u32, torn_bytes: usize)` — Cuts the power on a later call. The next `operations` program or erase calls complete, then the following one is torn: only its first `torn_bytes` bytes are programmed (or erased to 0xFF), it fails with [`MemoryError::PowerCut`], and so does every call after it, reads included. A `torn_bytes` past the call's length lets it complete before the power goes: it was done but never returned.
- `fn fail(&self, operations: u32, torn_bytes: usize)` — Fails one later call: as [`MemoryFlash::cut_power`], but it fails with [`MemoryError::Failed`] and the calls after it work, as a flash error that a retry gets past.
- `fn reads(&self) -> u32` — Read calls so far.
- `fn programs(&self) -> u32` — Program calls so far, a torn one included.
- `fn erases(&self) -> u32` — Erase calls so far, a torn one included.

## `flash_ring::ring`

The ring_writer's side of the ring (doc/specs/avionics-port.md, "The ring_writer" and "Ring geometry and block
life cycle"). The writer thread owns the [`Ring`] and makes every call; the eraser and the saver work on the
same partition through their own handles and the shared [`BlockTable`].

A block's life, as the writer sees it:
1. **Erased.** The writer takes the next erased slot after its current one, reads the whole block once to
   prove it erased (one 4 KiB read, ~0.4 ms), and programs the block header there: one program call.
2. **Open.** Each [`Ring::append`] programs one chunk at the next 8-aligned offset with one program call. A
   chunk that does not fit closes the block. A record never straddles blocks, and a new boot never appends to
   a previous boot's block, so a torn chunk left by a cut is always its block's last.
3. **Closed.** Valid ring data, which the saver copies and the eraser erases once the saver has passed it, or
   under newest wins.

When no slot is erased, an append returns [`Error::WouldBlock`] and programs nothing: the records stay in the
producers' PSRAM rings until the eraser frees a block. Nothing is spilled or queued here.

The flight flag of a block is fixed when the block opens. Every record from BOOSTING on must lie in a flight
block (spec "Ring geometry and block life cycle"), and a reboot in flight brings storage up long before the
warm start decides anything (spec "Bring-up order": storage at step 3, the warm restore at step 8). So the
writer starts with the flag the crumb gives ([`Ring::new`]) and [`Ring::set_flight`] corrects it later.

### `pub struct Ring<'table, F: Flash>`

The flash ring as its one writer sees it.

- `fn new(flash: F, table: &'table BlockTable, next_sequence: u32, boot_id: u32, session: Session, flight: bool) -> Result<Ring<'table, F>, Error<F::Error>>` — The writer of this boot, after the bring-up scan. Nothing is read or programmed here; the first [`Ring::append`] opens the first block.
- `fn append(&mut self, payload: &[u8], records: u16) -> Result<Position, Error<F::Error>>` — Programs one chunk of `payload` (whole records, verbatim) holding `records` records, with one program call into the open block, or into a newly opened one when it does not fit there.
- `fn room(&self) -> usize` — The largest payload the next [`Ring::append`] programs into the open block: the rest of the block behind one more chunk header. When no block is open, or the open one cannot take another chunk header, the next append opens a new block anyway, and it is [`MAX_PAYLOAD`]. The writer copies records into its chunk while they fit this, so a block fills before it closes.
- `fn set_flight(&mut self, flight: bool)` — Sets whether the stage is in flight (the caller sets it from BOOSTING to DONE). A change closes the open block, so every record appended from now on lies in a block whose header carries the new flag: newest wins gives up flight blocks last.
- `fn flight(&self) -> bool` — Whether the blocks opened from now on carry the flight flag.
- `fn position(&self) -> Position` — The writer position to publish (`writer_position`, two AtomicU32 under the firmware's seqlock): the saver reads up to here and never further.
- `fn table(&self) -> &'table BlockTable` — The block table this writer shares with the eraser and the saver. The recorder's writer reads its budget figures from it for the status line ([`BlockTable::figures`]), so the figures always describe the ring it writes and never a table passed in beside it.

## `flash_ring::scan`

_Tested by its own `mod tests`._

The bring-up scan (doc/specs/avionics-port.md, "Bring-up: the scan, the positions and the session-start
record"). It reads the 56-byte header at each of the 5120 block starts (about 0.15 s, estimated) and classifies
each slot into the [`BlockTable`] the three ring tasks share. It runs in the ring_writer thread before the first
append, so the rest of boot goes on and the producers queue in PSRAM meanwhile.

One header per block is enough to classify it, because a block is one erase sector and the writer programs its
header first. What the header cannot show (a cut erase that left an erased header over dirty bytes) the writer
catches with its full read of a block before it opens it.

The ring positions in the `cfg` NVS are hints, written about once a second: the scan is the truth for what is
in flash and where the writer goes. A hint only keeps the block sequence from going back. The previous boot
may have opened blocks that are no longer in flash (saved and erased, or a header a cut tore), and a sequence
number is never given twice.

A hint is trusted only within [`HINT_REACH`] of the newest block in flash. One beyond it is corrupt, or was
written in another build's encoding (a reboot in flight that came back on another image reads the positions
the other image left). Taken as it is, it would push the numbering up to u32::MAX, where no block can open.

### `pub struct Hints`

The ring positions a previous boot left in the `cfg` NVS, when there are any.

### `pub struct Scan`

What the bring-up scan found.

### `pub fn classify(header: &[u8]) -> Slot`

What a slot holds, judged by the 56 bytes at its start: [`Slot::Erased`] when they are all 0xFF,
[`Slot::Valid`] or [`Slot::OtherVersion`] for a whole header by its version, else [`Slot::Foreign`].
The scan never finds a block open: a new boot opens a block of its own.

### `pub fn scan<F: Flash>(flash: &F, geometry: Geometry, hints: Hints) -> Result<Scan, Error<F::Error>>`

Scans the ring: one 56-byte read per slot, classified into a new [`BlockTable`]. The writer's cursor goes to
the newest block's slot (it opens the first erased slot after it), and [`Scan::next_sequence`] goes past the
newest block and every hint within [`HINT_REACH`] of it.

Args:
* `flash` - the `log` partition.
* `geometry` - the ring's shape.
* `hints` - the positions the previous boot left in the `cfg` NVS (none on a first boot).

Returns the scan. Errors: `Geometry` when the geometry is larger than the flash, and `Flash` when a read fails.

### Constants

- `const HINT_REACH: u32 = 1 << 31` — How far a hint may run ahead of the newest block in flash (of 0 when flash holds none) and still raise the numbering: 2^31 blocks, 8 TiB of writing, some 17 years at the flight rate. A real hint is at most a few blocks ahead (headers a cut tore or a failed program spent), or anywhere when the ring was erased whole; the reach takes both, and refuses the top half of the u32 range that only a corrupt hint reaches.

## `flash_ring::table`

_Tested by its own `mod tests`._

The block table (doc/specs/avionics-port.md, "Bring-up: the scan, the positions and the session-start
record"): what each of the ring's slots holds, shared by the three ring tasks through atomics. It has 5120
entries on the board (48 KiB in PSRAM): a state word and a block sequence per slot, each an AtomicU32.

Who changes a slot is fixed by its state, so the tasks never race on one:
* the writer moves a slot out of ERASED: to OPEN when it opens the block there, or to FOREIGN when its check
  finds the slot not wholly erased (a cut erase left an erased header over dirty bytes), or cannot read it. It
  moves its own OPEN slot to VALID when it closes the block;
* the eraser moves VALID, OTHER-VERSION and FOREIGN slots to ERASED, and nothing else;
* the saver, the figures and the tools only read.

A slot is written sequence first, then its state with Release; a reader loads the state with Acquire, then the
sequence. A reader racing an erase and a reopen of the same slot can still pair an old state with a new
sequence, so the saver checks the header it reads from flash against the table before it trusts a block.

### `pub enum Slot`

What a slot holds.

The spec's UNSAVED and SAVED states are both [`Slot::Valid`]: which one a block is depends on the saver's
position, which the table does not keep ([`BlockTable::figures`] takes it).

- `fn block_sequence(self) -> Option<u32>` — The sequence of a block that holds ring data (open, valid or of another version). None for the others.

### `pub struct Figures`

The ring's budget, for health and inspect (`ring:{…}`), the degraded tokens and the arm gate.

- `fn erased_seconds(&self, bytes_per_second: f32) -> f32` — `ring_erased_s`: how long the writer goes before newest wins, at `bytes_per_second` (the measured write rate). Erased and saved blocks count, since the saved ones are erased within 200 ms. A zero rate gives infinity: nothing is written, so newest wins never comes.
- `fn unsaved_seconds(&self, bytes_per_second: f32) -> f32` — `ring_unsaved_s`: the flash-to-SD backlog at `bytes_per_second`, by the same measure. `saver-behind` is this growing for more than 60 s with a card mounted.

### `pub struct BlockTable`

What each slot holds, shared by the writer, the eraser and the saver through atomics.

- `fn geometry(&self) -> Geometry` — The ring's shape.
- `fn slot(&self, slot: u32) -> Slot` — What `slot` holds now.
- `fn figures(&self, saver: Option<Position>) -> Figures` — The budget figures, walking every slot (5120 pairs of loads on the board, tens of microseconds). `saver` is the saver's published position: the blocks before its block are saved. None (no card, or no saver yet) counts every closed block unsaved.
- `fn next_block(&self, from: u32) -> Option<(u32, u32)>` — The block of this layout, closed or the writer's open one, with the lowest sequence at or after `from`, as (slot, block sequence); None when there is none. One walk over every slot, as [`BlockTable::figures`].
- `fn saver_start(&self, hint: Option<Position>) -> Position` — Where the saver starts (spec "Bring-up", step 4): at its `hint` from the `cfg` NVS when the hint names a block of this layout still in flash, else at the oldest such block, so nothing in flash is left out. With none at all it is the ring's start, before any block this boot will open.

# `golden` — `src/avionics/crates/golden`

_Uses: `cc_protocol`. External: serde_json._

_Integration tests: `tests/fixtures.rs`._

## `golden`

_Tested by its own `mod tests`._

Golden vectors: the fixtures `src/avionics/golden/generate.py` writes from plan B (src/glider run on
CPython), loaded and compared against the Rust port. Dev-only: a crate takes this as a dev-dependency and
its test does `golden::run(include_str!("../../../golden/<fixture>.json"), |input| port(input))`, so the
fixture is compiled in and the test reads no file.

A fixture says how its values compare (`"compare": {"kind": ..., "tol": ...}`):
 * `exact` -- equal JSON values; an integer never equals a float;
 * `abs` -- numbers within `tol`;
 * `rel` -- numbers within `tol` times the expected magnitude;
 * `angle_deg` -- numbers within `tol` degrees the short way around the circle (359.9 and 0.1 are 0.2
   apart);
 * `bytes_hex` -- strings of hex digit pairs naming the same bytes, either case.

Under every kind the structure (keys, lengths) and every other value compare exactly: an expected INTEGER
(a count, a flag, a raw reading, an `int()` or `//` result) wants the same integer, never a float near it;
the tolerance applies only where plan B answered a float. Expected values and tolerances are f64 because
they come from JSON, the boundary decision D18 allows; a port's f32 widens to f64 exactly, so the
comparison adds no rounding of its own.

The CC dispatcher fixture compares reply lines instead of values: see [`dispatch`].

### `pub enum Compare`

How a fixture's expected values compare with a port's answers.

- `fn from_json(block: &Value) -> Compare` — The comparison a fixture's `compare` block names, such as `{"kind": "abs", "tol": 0.02}`.
- `fn check(self, case: &str, expected: &Value, actual: &Value) -> Result<(), String>` — Compare one expected value with the port's answer, all the way down.

### `pub struct Fixture`

A fixture document, loaded from the text the caller compiled in.

- `fn parse(text: &str) -> Fixture` — Load a fixture's text.
- `fn get(&self, member: &str) -> &Value` — A top-level member, such as `meta`, `cases` or a fixture's own `config_id`.
- `fn compare(&self) -> Compare` — How this fixture's values compare.

### `pub fn run(text: &str, mut port: impl FnMut(&Value) -> Value)`

Run every `{name, input, output}` case of a fixture through `port` and compare its answer by the fixture's
kind.

Panics listing every failing case, and also when the fixture has no cases or a case lacks its input or
output (a misspelt member would otherwise read as null and pass a port that answers null): a run that
compared nothing must not read as a pass.

### `pub fn hex(bytes: &[u8]) -> String`

The lowercase hex of `bytes`: how a port hands bytes to a `bytes_hex` comparison.

### Constants

- `const PLAN_B_TAG: &str = "micropython_plan_B_2"` — The git tag plan B is frozen at, which every fixture names in `meta.tag` (golden/generate.py `_TAG`). The tests assert it through this one constant, so moving the tag on purpose is one edit here, not one per crate.

## `golden::dispatch`

_Tested by its own `mod tests`._

The CC dispatcher fixture (`src/avionics/golden/cc_dispatch.json`): request lines sent IN ORDER to one
dispatcher, and the reply each must get. Plan B keeps state between commands (the stage, the log window,
the clock), so the order is part of the fixture.

A case is `{name, request, status, arguments_kind, expected}`: `status` is the reply's first token (null
when no reply is due) and `expected` describes its decoded positional arguments by `arguments_kind`
([`ArgumentsKind`]). Optional members:
 * `extra_keys` -- `{key: type}` a `json_keys` object may carry beyond the expected keys;
 * `owed` -- `{status, arguments_kind, expected}`: the answer the Rust board gives on purpose instead of
   plan B's. A port is checked against `owed` when it is present;
 * `note` -- prose for the reader.

Reply lines are parsed with `cc_protocol`, as the hub parses them, so a base64-carried argument compares by
its decoded text.

### `pub enum ArgumentsKind`

How a case's `expected` list describes a reply's decoded positional arguments.

- `fn from_name(name: &str) -> ArgumentsKind` — The kind a case names.
- `fn check(self, expected: &[Value], arguments: &[String], extra_keys: &Map<String, Value>) -> Result<(), String>` — Compare a reply's decoded positional arguments with `expected`.

### `pub struct Case`

One dispatcher case, checked and resolved: the answer a port must give is `owed`'s where the case has one.

- `fn check(&self, reply: Option<&str>) -> Result<(), String>` — Compare a port's reply line (None for no reply) with this case's answer.

### `pub fn cases(text: &str) -> Vec<Case>`

The cases of a dispatcher fixture, checked: known kinds and type names, an `any` case that lists its code,
unique names, `extra_keys` only on a `json_keys` answer.

Panics naming the first broken case, and when there are no cases.

### `pub fn run(text: &str, mut port: impl FnMut(&str) -> Option<String>)`

Send every request of a dispatcher fixture to `port`, in order, and check each reply line (None for no
reply) against the case's answer.

Panics listing every failing case, and when the fixture is broken (see [`cases`]).

### Constants

- `const TYPE_NAMES: [&str; 7] = ["null", "bool", "int", "float", "str", "list", "object"]` — The JSON type names a `json_keys` expectation uses; `float|null` allows either.

# `layout` — `src/avionics/crates/layout`

_Uses: `board_config`. External: serde_json._

_Integration tests: `tests/golden.rs`._

## `layout`

_Tested by its own `mod tests`._

Which board is this, decided by I2C scan before any driver is set up: the port of src/glider layout.py
(doc/hardware.md "Transition"; doc/specs/avionics-port.md M5). The BUS MAP comes from where the moving devices
answer; the attitude parts then follow from the revision and from what the scan found.

There are two bus maps, and they differ only in which bus four devices hang off, so one firmware can serve
both if it can tell them apart. It can: four addresses swap buses between the maps, which is four independent
votes rather than one hinge, so a single dead device cannot flip the verdict.

```text
                   i2c:0                          i2c:1
    v0.1 map   0x18 0x63 0x25 0x29  + module      0x40
    v1.x map   0x18 0x40            + module      0x63 0x25 0x29

    SEN0697  BMI323 0x69  BMP581 0x47  BMM350 0x15   i2c:0   PRIMARY: expected on every scanned board
    SEN0253  BNO055 0x28  BMP280 0x76                i2c:0   BACKUP: enabled only when the scan finds it
```

THE MAP IS DECIDED BY PLACEMENT ALONE. The attitude module sits on i2c:0 on every board, so where it answers
says nothing about where the front devices are. It used to vote anyway, in one tally with the placement votes,
and that put a v0.1 board on the wrong map: TMS-7C, the one v0.1 board left and now carrying a SEN0697, read
v1.1 on three module votes against two placement votes, and its laser, pitot and ICP-10111 were moved to an
i2c:1 with nothing on it (scan 2026-10-05: i2c:0 0x15 0x18 0x25 0x29 0x47 0x63 0x69, i2c:1 empty).

The three revision names stay, because configs and captures carry them:

```text
    v0.1   the v0.1 map + the SEN0697 + the ADXL375 on SPI    TMS-7C
    v1.1   the v1.x map + the SEN0697                         the taster, TMS-7F
    v1.0   the v1.x map + the SEN0253 alone                   legacy: TMS-7E as built -- DECLARED only
```

A SCAN NEVER ANSWERS v1.0. The v1.x map always scans as v1.1, because the SEN0697 is the module every board is
expected to carry: when it is missing, its setup fails and `arm` refuses. A scanned v1.0 used to switch it off
instead -- a two-module board whose SEN0697 connector had worked loose read v1.0 and would have flown on the
backup with nothing refusing. v1.0 survives as a DECLARED revision for the legacy build and the configs saved
for it, and there the SEN0253 IS the module: [`apply`] enables it.

Everywhere else THE SEN0253 IS A BACKUP: a taster or an experimental board may carry both modules (different
addresses on the same bus), while a flight board normally carries the SEN0697 alone. After a scan, [`apply`]
enables the SEN0253 parts that answered and disables the rest; a declared v0.1 / v1.1 runs no scan, so the
config's own `enabled` stands. Wherever it ends up enabled, [`apply`] RANKS it below the SEN0697 and the
attitude filter on every quantity they share -- in code, whatever the config says, because every config saved
before 2026-10-05 carries the BNO055 at attitude priority 0 (see [`BELOW`]).

The ADXL375 is on SPI and invisible here, so "no ADXL375" is carried by the map, never scanned for.

`0x18` is the ANCHOR: the ES8311 audio codec soldered to the WaveShare board itself. It says nothing about the
revision (every board has it), and that is exactly what an anchor is for: proving the scan reached a live bus
rather than returning an empty set. Because it is soldered down, no sensor swap can remove it. The other
known-present addresses stay in the list as fallbacks, in case a board ever ships without the codec.

The port, against plan B:
 * The scan is INJECTED, and plan B's `_scan` rules around it are kept HERE, not left to the firmware: a bus the
   config does not define is never scanned and casts no votes, and a bus that fails answers an empty set ("no
   votes from here", never evidence). [`resolve`] and [`detect`] take a closure `(bus id, bus spec) ->
   Result<Addresses, _>`, the part of `_scan` that touches hardware (`I2C(...)` and `scan()` at [`SCAN_HZ`]);
   the crate calls it only for a bus spec plan B would have opened (`scan_bus`), and reads an `Err` as an
   empty set. The P4 has both I2C ports whatever the config says, so a closure left to decide would probe a bus
   plan B never touched and cast votes plan B never cast. Plan B's re-clock dance after a runtime scan is not
   ported: `i2c_master` speed is per device (§6).
 * [`resolve`] returns what plan B logged and stored in the `RESOLVED` global: a [`Resolved`], whose
   [`Resolved::label`] is the health payload's layout and [`Resolved::log_lines`] the boot-log lines, word for
   word.
 * The config is a `serde_json::Value`, read with Python's `==`, `%s` and truthiness ([`board_config::python`]),
   so every change string and every edit is plan B's (the golden fixture compares them exactly, with the
   config_id that follows). Where plan B would raise on a config validate() refuses (a non-int priority, a
   `board` that is not an object, a null clock in `%d`), the port reads the nearest safe value instead; each
   function names its case.
 * Layout moves buses, clocks, `enabled`, a few control pins and priorities; it never touches a device's
   `axes` (the IMUs are mounted per their marks, x forward, y left, z up, and an `axes` key is the
   per-airframe exception the drivers read), nor `pins`.

### `pub enum Revision`

A board revision, as configs (`board.layout`) and captures name it.

- `fn name(self) -> &'static str` — The revision's name: 'v0.1', 'v1.0' or 'v1.1'.
- `fn from_name(name: &str) -> Option<Revision>` — The revision a name declares; None for anything else ('auto', 'undecided', a typo), which plan B scans.
- `fn bus_map(self) -> BusMap` — The bus map this revision lays out (plan B `_MAP`).

### `pub enum BusMap`

One of the two bus maps. Placement votes between these and nothing else.

- `fn name(self) -> &'static str` — The map's name: 'v0.1' or 'v1.x'.

### `pub struct Addresses(u128)`

The 7-bit addresses that answered on one bus: a set as a bitmask, no allocation.

- `fn insert(&mut self, address: u8)` — Add an address. One above 0x7F is not a 7-bit address -- nothing can answer there -- and is dropped.
- `fn contains(self, address: u8) -> bool` — Whether `address` answered.
- `fn is_empty(self) -> bool` — Whether nothing answered.
- `fn union(self, other: Addresses) -> Addresses` — The addresses in either set.
- `fn iter(self) -> impl Iterator<Item = u8>` — The addresses, ascending.

### `pub type Scan = [Addresses; 2]`

One scan: what answered on i2c:0 and on i2c:1, indexed by bus id (plan B's `seen`).

### `pub struct Verdict`

The decision on one scan (plan B `_verdict`'s tuple).

### `pub enum Resolved`

What [`resolve`] concluded this boot: the revision, how it was reached, and every change it made.

- `fn revision(&self) -> Option<Revision>` — The revision applied, None when undecided.
- `fn changes(&self) -> &[String]` — Every change made to the config.
- `fn label(&self) -> String` — The health payload's layout (plan B's `RESOLVED`): 'v0.1 (declared)', 'v1.1' or 'undecided'. The operator should see which revision the firmware decided it runs on WITHOUT reading the boot log, since a wrong verdict and a miswired board look identical from the device list.
- `fn log_lines(&self) -> Vec<String>` — The boot-log lines plan B's resolve() logs, word for word.

### `pub fn verdict(seen: &Scan) -> Verdict`

The decision on one scan: the map by placement, the revision the scan names for it, the backup found (plan B
`_verdict`).

Placement is the only evidence for a map. A MOVING device votes for whichever map puts it on the bus it actually
answered on; absence never votes AGAINST, so a dead or unfitted device costs its map one vote rather than casting
one for the other, and no single failure flips a verdict. A TIE (0-0 included: a scan that saw only the anchor
and the attitude module) is undecided, because the module cannot break it -- it is on i2c:0 whichever map this
is. An empty scan, or one with no anchor on i2c:0, is not trustworthy and decides nothing.

The modules are counted for the detail line and nothing else: the v1.x map is v1.1 whichever module answered,
so a missing SEN0697 fails its setup rather than reading as legacy v1.0.

### `pub fn detect<Failure>(document: &Value, scan: impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, Failure>) -> (Option<Revision>, String)`

Scan both buses through `scan` (i2c:0, then i2c:1, each only when the config defines it: see `scan_bus`) and
decide the layout (plan B `detect`, the CC `detect` command): the revision (None when undecided) and the
evidence text. Nothing is changed.

`scan` is the hardware: open bus `id` on the spec's pins at [`SCAN_HZ`] and list what answers; an `Err` (the bus
would not open, the scan failed) reads as an empty set.

### `pub fn fitted(device: &str, revision: Option<Revision>) -> bool`

Whether `device` is physically present on `revision` -- the ONE place that question is answered (plan B
`fitted`). `revision` None is an undecided scan (or any name that is not a revision).

Every caller that instead wrote its own revision literal has eventually been wrong: resolve() gated on ('v0.1',
'v1.0') and so ignored v1.1 entirely, and test_spibus asked `revision != 'v1.0'` and so demanded an ADXL375 from a
v1.1 board that has none. Both read fine until a third revision existed. Ask here instead, and adding a revision
updates every caller at once.

UNDECIDED answers False for the parts some revision removes, and for the moving devices: unknown is not evidence
of presence, and a test that demands a part on an unidentifiable board reports a hardware fault when what it
found was an inconclusive scan. (The follower `laser_agl_l1x` is not a moving device, so undecided answers True
for it, as plan B does.)

The SEN0253 answers True on a backup revision (v1.0) only, where it is the module. No other revision guarantees it:
only a scan finds it there, and the resolved config's `enabled` is what records the result.

### `pub fn apply(document: &mut Value, revision: Revision, found: Option<&[&str]>) -> Vec<String>`

Rewrite the config in place for a revision: bus membership, the i2c:1 clock, what is fitted, the control pins,
the ranks (plan B `apply`).

Only the four moving devices (and their socket followers), one bus clock, the revision-dependent parts, the
backup module, a few optional control pins and the backup ranking ([`BELOW`]) differ -- no pin is renumbered, so
`pins` is untouched.

`found` is the backup (SEN0253) parts a scan found: those are enabled and the rest disabled. None means there was
no scan (a declared revision) and the config's own `enabled` stands -- except on a backup revision (v1.0), which
fits the whole SEN0253 either way.

Plan B's quirks, kept: a moving device whose `id` already matches keeps its `bus` whatever it says; a missing
`id` or clock reads as null and is rewritten. Returns the human-readable change strings for the boot log, empty
when the config already matched (the normal case on a board whose profile was written for it).

### `pub fn rank(document: &mut Value, changes: &mut Vec<String>, revision: Option<Revision>)`

Demote each ENABLED device in [`BELOW`] to strictly below its enabled superiors, per shared quantity (plan B
`_rank`), appending a change string for every priority moved.

Bus-independent, which is why [`resolve`] runs it on an undecided scan too (`revision` None): the ranking says
which source the databoard prefers, not where anything is wired.

NOT on a backup revision (v1.0): there the SEN0253 IS the module, and the board carries no BMM350, so the filter has
no heading reference of its own -- demoted below it, the BNO055 would never seed it and the heading would start
at 0 and stay gyro-integrated. The SEN0253 keeps the rank its config gives it, as it always flew there.

`provides` that is absent or falsy reads as nothing; a superior's entry is read for `priority` only.

### `pub fn fit(document: &mut Value, name: &str, present: bool, changes: &mut Vec<String>)`

Enable a device the board carries, disable one it does not -- unless the operator unfitted it (plan B `_fit`),
appending a change string when `enabled` changes. A device the config does not declare is skipped.

`fitted: false` is the operator saying THIS part is dead or removed (only the literal false: plan B's
`is not False`). Without it, a part the board is supposed to carry would be re-enabled here: an `enabled: false`
was undone, and verify/arm refused on a part nobody could take out of the config.

### `pub fn resolve<Failure>(document: &mut Value, scan: impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, Failure>) -> Resolved`

The boot entry point (plan B `resolve`): honour a declared `board.layout`, else scan, else change the bus map
not. The config is rewritten in place; `scan` (as [`detect`] takes it) is asked for i2c:0 then i2c:1, only when
nothing is declared, and only for a bus the config defines (`scan_bus`).

An explicit 'v0.1' / 'v1.0' / 'v1.1' always wins over the scan, so a board can be pinned when a sensor is
unfitted and would otherwise abstain its way to a wrong verdict. Anything else ('auto', the default; null; a
typo) scans: every name in [`Revision::ALL`] is checked, not a literal pair, so a revision added later cannot
silently fall through to the scan, which is what once made a declared v1.1 ignored. A declared v0.1 / v1.1 never
touches the SEN0253's `enabled`: with no scan there is nothing to say whether it is fitted, so the config's own
value stands (a declared v1.0 fits it -- it IS v1.0). Whatever the path, an enabled SEN0253 is ranked below the
SEN0697 ([`BELOW`]).

A config with NO `layout` key means v0.1, not 'auto'. A valid board.config is used wholesale, so a profile
written before the field existed genuinely arrives without it, and every such profile belongs to a board built
the old way. TMS-7C, the one v0.1 board left, flies keyless profiles from make_telemetry_config.py; defaulting it
to a scan would put a built, closed airframe's bus map at the mercy of its sensors answering (7C casts three
placement votes, so a front harness fault is all it takes to make it undecided). A `board` that is not an object
reads as keyless here (plan B would raise; validate() refuses such a config).

UNDECIDED applies no fallback guess: the wrong revision re-buses every moving sensor at once, which is a worse
failure than running the config as written and saying so. The ranking still runs -- it depends on no bus, and an
old profile's p0 BNO055 is as wrong there as anywhere.

### Constants

- `const SCAN_HZ: u32 = 100_000` — The scan clock: every part tolerates 100 kHz, and the v1.x front bus runs there permanently.
- `const BELOW: [(&str, &[&str]); 3] = [` — The RANKING [`apply`] enforces whatever priorities the config carries (plan B `_BELOW`): (device, the devices it must rank strictly BELOW). Wherever the device is enabled, each quantity it provides that an ENABLED superior also provides is demoted to one past the worst-ranked such superior. It only ever demotes, and a quantity no enabled superior provides is left alone.

# `sim_link` — `src/avionics/crates/sim_link`

_Uses: `recorder_wire`. External: none._

## `sim_link`

_Tested by its own `mod tests`._

The HITL sim-link's two frames (doc/specs/sim-link.md): SENSORS, the host's physics as the board's databoard
quantities, and FINS, the board's stage, arming and fins back. Fixed sizes, little-endian, a CRC-32 trailer;
encoding and decoding never allocate, so the board's endpoint runs them on its receive path as they are.

A decode refuses a frame of the wrong length, magic, version or kind, a bad CRC, or a non-finite float in a
present field ([`Refusal`]); the sequence order ([`newer`]) is the receiver's to apply.

### `pub enum Refusal`

Why a frame was dropped.

### `pub struct Sensors`

One SENSORS frame: the host's simulated sensors, each None when the host does not send it (its flag clear).

- `fn encode(&self) -> [u8; Self::BYTES]` — The frame's bytes.
- `fn decode(bytes: &[u8]) -> Result<Sensors, Refusal>` — The frame in `bytes`.

### `pub struct Fins`

One FINS frame: the board's answer to a SENSORS frame it applied.

- `fn encode(&self) -> [u8; Self::BYTES]` — The frame's bytes.
- `fn decode(bytes: &[u8]) -> Result<Fins, Refusal>` — The frame in `bytes`.

### `pub fn newer(seq: u32, last: u32) -> bool`

Whether sequence number `seq` is newer than `last` across the wrap: `(seq - last) mod 2^32` in 1..2^31.

### Constants

- `const MAGIC: [u8; 4] = *b"CSL1"` — Every frame's first four bytes.
- `const VERSION: u8 = 1` — The frame layout's version.

# `drivers` — `src/avionics/crates/drivers`

_Uses: `board_config`, `control`, `sequencer`. External: embedded-hal, serde_json._

_Integration tests: `tests/general_call.rs`, `tests/golden_bmi323.rs`, `tests/golden_bmm350.rs`, `tests/golden_bmp581.rs`, `tests/golden_icp10111.rs`, `tests/golden_ina226.rs`, `tests/golden_lsm6dso32.rs`, `tests/golden_sdp810.rs`, `tests/golden_separation.rs`, `tests/golden_servo.rs`, `tests/golden_vl53.rs`, `tests/probe.rs`, `tests/rearm.rs`._

## `drivers`

The device logic of the src/glider drivers (src/glider/drivers/*.py), one module per plan B file and named
after it: the register init tables as data, the conversions, the CRCs, the calibration and tare math, and the
re-arm a general call makes necessary. Everything talks through embedded-hal 1.0 (`I2c`, `DelayNs`,
`OutputPin`), so the same code runs over the firmware's `I2cPort` and over the scripted [`mock`] bus the host
tests use.

What is NOT here: the run loops, the databoard channels, the telemetry streams and the CC surface. Plan B's
Task owned all of them; in Rust they belong to the bus owner thread and its per-part glue (firmware
`sensors/<part>.rs`, doc/specs/avionics-port.md §1.2). A driver here is a passive object the owner calls
between its own transactions, and nothing in it reads a clock: time arrives as an argument or as a
`DelayNs` wait.

Shared by every driver:
 * [`device`] -- the [`device::Device`] trait every bus part implements (setup and the general-call
   [`device::Device::rearm`]), the [`device::Fault`] a conversation ends in, and [`device::Strikes`], the
   consecutive-failure counter (src/glider task.py);
 * [`commons`] -- the Sensirion CRC-8 and the wire-level id diagnosis (src/glider commons.py);
 * [`mock`] -- the scripted bus, delay and pin the host tests drive the drivers with;
 * [`freshness`] -- new conversions only: an IMU half judged by its data-ready flag and corrected by its bytes, and
   a part read without a flag judged by changed bytes, with the standby rule (new, avionics-port.md M9 D1, D3);
 * [`axes`] -- the optional per-sensor `axes` key: a part mounted other than per its marks (new, avionics-port.md
   §7.2 Q7);
 * [`debounce`] -- two stamps for a two-level input behind a latch, the first and the latest edge of an episode,
   recorded by its interrupt (or by its tick's own reads) and taken once the input has been quiet: the separation
   switch's contact, and any other (a remove-before-flight pin, a breakwire).

The parts, each named after its plan B module:
 * [`sdp810`] -- the pitot: dynamic pressure and airspeed;
 * [`ina226`] -- the power monitor: bus voltage, current and power;
 * [`vl53l1x`] and [`vl53l4cx`] -- the two laser rangers that share one socket at 0x29;
 * [`separation`] -- the stage-separation switch: its latch over [`debounce`] (separated, nested or a blip), the
   separation moment and the contact's wobble, and the thrust gate that holds a separation in BOOSTING pending
   until the glider's a_x shows the boost ended (or the backstop);
 * [`lsm6dso32`] -- the primary IMU on SPI: accel and gyro `rate`, its write-then-verify bring-up;
 * [`bmi323`] -- the SEN0697's IMU on I2C: the same `accel` and `rate`, behind the LSM6DSO32;
 * [`icp10111`] -- the primary barometer: the TDK polynomial, the two-phase measurement, the general call;
 * [`bmp581`] -- the SEN0697's backup barometer;
 * [`bmm350`] -- the SEN0697's magnetometer and its hard/soft-iron calibration;
 * [`bmp280`] and [`bno055`] -- the SEN0253's backup barometer and backup attitude (the v0.1 boards): the
   BNO055's own NDOF fusion, its stall judge and its learned profile;
 * [`sg90`] and [`mg90s`] -- the fin servos' degree-to-pulse map;
 * [`bluetooth`] -- the BLE radio switch, a stub in plan B's no-BLE shape: the requested state and its `update`.

## `drivers::axes`

_Tested by its own `mod tests`._

The optional per-sensor `axes` key of the board config: how a sensor's axes map onto the airframe's. NEW in the
port, with no plan B module behind it (avionics-port.md §7.2 Q7, the operator's decision of 2026-10-06).

The IMUs are mounted per their marks: the sensor's x forward, y left and z up are the airframe's, the frame every
`accel`, `rate` and `mag` sample is published in. Its consumers map it into their own in one place, crates/control
`attitude::Readings::from_marks`: the attitude filter computes in the aerospace frame (x forward, y right, z down),
so the (roll, pitch, yaw) rate the filter and the PID D term take is (gx, -gy, -gz), not plan B's gx -> roll,
gy -> pitch, gz -> yaw, which reads a part mounted per its marks nose-down-positive. Plan B had no way to state an
exception, and two parts publish into ONE channel: the LSM6DSO32 and the BMI323 both feed `accel` and `rate`, so a
part glued in turned would make a failover flip the airframe's axes in flight. The key states the exception, per
sensor, and the firmware maps each sample through it before publishing, so every source of a channel speaks the
airframe's axes.

The value is a signed permutation, three `<sign><axis>` pairs naming, for the airframe's x, y and z in turn, the
sensor axis that reads along it: `"-y+x+z"` means airframe x = -sensor y, airframe y = +sensor x, airframe z =
+sensor z (the part turned 90 degrees about z). No key is the identity `"+x+y+z"`, so every existing board file
keeps its meaning.

A MIRROR is refused: a part's axes are right-handed, a mounting is a rotation, and a rotation keeps handedness, so a
permutation with determinant -1 (two axes swapped without a sign, or one sign flipped alone) cannot describe any
mounting and is a typo. Plan B's config validation ignores keys it does not know, and the Rust validation keeps its
accept set, so a malformed `axes` is not a config error: the device refuses it at setup, naming the key.

The map is applied to CONVERTED values (f32 g and degrees per second, i32 centidegree columns), never to a raw i16
count, whose -32768 has no positive counterpart.

THE MAGNETOMETER IS THE EXCEPTION to "the firmware maps each sample": the BMM350 driver applies its own map,
AFTER its calibration ([`crate::bmm350::Bmm350::mounted`]), and the firmware must not map `mag` again. The order
matters there and nowhere else. The calibration is learnt from a LEVEL turn in the SENSOR's own x/y plane and
persisted in NVS as a 16-byte blob shared with plan B, which has no axes and so speaks sensor x/y; observing in the
sensor frame keeps that blob valid across a reflash both ways, and only the corrected vector is turned onto the
airframe's axes. And a mounting that tilts the sensor's z off the vertical ([`Axes::keeps_vertical`] false) is
refused for the magnetometer: its x/y plane would not be level, a level turn would barely move one of its axes,
and the coverage check would refuse every calibration.

### `pub enum AxesError`

Why an `axes` value was refused.

### `pub struct Axes`

A sensor-to-airframe axis map: a signed permutation that is a rotation.

- `fn parse(text: &str) -> Result<Axes, AxesError>` — Parse a `<sign><axis>` triple such as `"-y+x+z"`; spaces are not allowed, nor a missing sign.
- `fn from_config(config: &Value) -> Result<Axes, AxesError>` — The map a sensor's config block states: its `axes` key, or the identity when there is none (or it is null).
- `fn keeps_vertical(&self) -> bool` — Whether the airframe's z reads the sensor's z, either way up: the part is turned about z or flipped over, so its x/y plane stays level. A magnetometer needs it ([`AxesError::Tilted`]).
- `fn apply<T: Copy + Neg<Output = T>>(&self, vector: [T; 3]) -> [T; 3]` — Map one sensor vector onto the airframe's axes.

### Constants

- `const KEY: &str = "axes"` — The config key.

## `drivers::bluetooth`

_Tested by its own `mod tests`._

The BLE radio switch (src/glider/drivers/bluetooth.py), a stub in plan B's no-BLE shape (doc/specs/avionics-port.md
§6). Plan B's entry declares whether Bluetooth should be on (`radio`, default false) and its setup applies it; on a
board with no BLE module its `active` reads None. This firmware starts no BLE radio (the C6 coprocessor's BLE is
unused on the glider), so the stub keeps only what the operator sees: the requested `radio`, which `update` changes,
beside an `active` that is always null (the firmware's inspect), and a probe that passes, since an absent radio is
never in the wrong state.

### `pub struct Bluetooth`

One `bluetooth` entry: the BLE radio state the operator asks for (plan B's `radio`).

- `fn from_config(config: &Map<String, Value>) -> Bluetooth` — Plan B's setup: the radio state `config` (the entry) declares, its `radio`, false when absent (`config.get('radio', False)`; an explicit null stays null).
- `fn radio(&self) -> &Value` — The requested radio state (`inspect`'s `radio`).
- `fn update(&mut self, properties: &Map<String, Value>) -> Vec<&'static str>` — Plan B's `update`: a `radio` in `properties` unequal to the requested state (Python's `!=`, so an int 1 is true) becomes it.

## `drivers::bmi323`

_Tested by `src/bmi323/tests.rs`._

The BMI323 6-axis IMU on the SEN0697 (src/glider/drivers/bmi323.py): accel and gyro on the shared I2C bus,
publishing the SAME channels as the LSM6DSO32 -- `accel` in g and `rate` in degrees per second -- at priority 1
behind it, so the two are interchangeable sources for one channel and the attitude filter runs off whichever
the databoard hands it. `rate` was a single-source channel on every board before this part: a dead LSM6DSO32
took the PID's D term with it. Two independent 6-axis parts on different buses (one SPI, one I2C) remove that.

Two details the register map does not make obvious:
 * every I2C register read returns TWO DUMMY BYTES before the data ([`DUMMY_BYTES`]), so a 6-word burst is a
   14-byte read whose payload starts at offset 2. Reading it like an LSM6DSO32 returns plausible nonsense, and
   a one-byte chip-id read returns a dummy byte: the first hardware bring-up rejected a perfectly good part that
   way, while the BMP581 beside it (no dummy bytes) came up fine;
 * registers are 16-bit and written least significant byte first.

START-UP: the gyro reports 0x8000 ([`NO_SAMPLE`]) until it has a sample, measured at ~20 ms on the bench. Setup
waits it out rather than publish the marker, which scales to -2000 dps: a start-up artefact arriving in `rate` as
a full-scale rotation is the kind of thing a control loop acts on before anyone sees it. The accel's -32768 is
a legitimate -16 g rail, not a marker, and is never refused.

Register values are Bosch's own (bmi3_defs.h). Scaling: +/-16 g is 2048 counts per g, and +/-2000 dps is
2000/32768 dps per count, both exact in f32 (powers of two over small integers). The telemetry keeps plan B's
integer gyro columns: centidegrees per second, `raw x 2000 x 100 // 32768`, floored.

MOUNTING: the part is mounted per its marks, x forward, y left, z up, and publishes in that frame. It is a
property of how the module is glued in, not of the part; the board config's optional `axes` key states an
exception ([`crate::axes`]). The consumers map the marks frame into their own in one place, crates/control
`attitude::Readings::from_marks` (the (roll, pitch, yaw) rate is (gx, -gy, -gz), `attitude::rate_from_marks`,
for the filter and the PID D term alike): plan B's gx -> roll, gy -> pitch, gz -> yaw read a part mounted per
its marks nose-down-positive and yawing left.

Deliberate differences from plan B, both in the re-arm (the port's rule: a re-arm restores everything setup
wrote, as for the INA226):
 * it routes data-ready to INT1 again when the wire is declared. Plan B's rearm() restored the two sensor
   configurations only, so a general call left INT1 unrouted and the driver fell back to polling after 20
   silent wakes, reporting `interrupt_silent`;
 * a gyro that never leaves start-up after the re-arm is reported ([`Fault::Timeout`]) for the owner's log.
   Plan B awaited the start-up and dropped the verdict.

NEW CONVERSIONS ONLY (avionics-port.md M9 D1b), the LSM6DSO32's rule ([`crate::freshness::Flagged`]). The flight
read is [`Bmi323::read_fresh`]: ONE 16-byte burst from STATUS 0x02 -- the dummy bytes, the STATUS word, then the
six data words -- where plan B read 14 bytes from ACC_DATA_X on the data-ready edge. MEASURED on the taster and
7F (M9 stage 0): STATUS bit 7 is drdy_acc and bit 6 drdy_gyr (bit 5 drdy_temp, bit 0 por_detected, set once
after power-on), both rise together every 10.0 ms at the configured 100 Hz, exactly on the bursts whose data
changed, and reading STATUS clears them. So the flag in a burst describes that burst, and a conversion landing
between the STATUS word and the data reads as raced (flag clear, bytes changed: pushed once) with its flag left
set for the next burst over the same bytes (the duplicate: skipped). A flagged half is never called stale for
identical bytes: the gyro's LSB is 0.061 dps, so at rest two conversions in a row often repeat. Never fresh, and
never stored: a frame with any gyro axis at [`NO_SAMPLE`] or all three accel axes at it (a reset part reads
0x8000 frames, which changed bytes would otherwise push once as -16 g and -2000 dps; one accel axis at -32768 is
the -16 g rail, fresh). Setup and the re-arm forget the stored bytes. The bare [`Bmi323::read`] stays plan B's
14 bytes from ACC_DATA_X (setup's start-up looks and clearing read, the probe), so setup and the re-arm make plan
B's transactions; the STATUS burst is the golden fixture's one listed divergence. The firmware calls the part
unhealthy after [`STALE_LIMIT`] reads in a row with neither half fresh.

Not here (firmware glue): the INT1 GPIO interrupt and its wake count, the channel push, the telemetry stream, the
probe's log lines and the inspect JSON. As for the LSM6DSO32, the firmware arms the GPIO edge interrupt before
setup, which ends with the clearing read.

### `pub type RegisterWrite = (u8, u16)`

One 16-bit register write: (register, value), sent least significant byte first.

### `pub struct Sample`

One burst, past the dummy bytes: raw signed counts.

- `fn decode(frame: &[u8; FRAME_LENGTH]) -> Sample` — Decode a 14-byte burst: two dummy bytes, then accel x, y, z and gyro x, y, z, each signed 16-bit little-endian (plan B's `struct.unpack_from('<6h', buffer, 2)`). Accel comes FIRST out of this part.
- `fn gyro_started(&self) -> bool` — Whether the gyro has left its start-up state: no axis holds [`NO_SAMPLE`].
- `fn is_sample(&self) -> bool` — Whether the burst is a sample at all: the gyro started, and not all three accel axes at [`NO_SAMPLE`] (a reset part's 0x8000 frame; one accel axis there is the -16 g rail).
- `fn accel_g(&self) -> [f32; 3]` — The accel in g: the `accel` channel and the telemetry's ax, ay, az (exact: a power-of-two scale).
- `fn rate_degrees_per_second(&self) -> [f32; 3]` — The gyro in degrees per second: the `rate` channel. Exact in f32: 2000/32768 is 125/2048.
- `fn rate_centidegrees_per_second(&self) -> [i32; 3]` — The gyro in centidegrees per second, plan B's integer to the digit (`raw x 2000 x 100 // 32768`, floored, so -1 count is -7 and +1 is 6): the telemetry's gx, gy, gz columns. The full scale reads 199993 and -200000.

### `pub struct Bmi323`

One BMI323 on an I2C bus.

- `fn new(address: u8, interrupt_wired: bool) -> Bmi323` — A driver for the part at `address`; `interrupt_wired` when its `int_pin` resolves to a GPIO (on the v1.x map layout routes `accel_int1` to it; the v0.1 attitude socket has no INT wire, and the driver polls).
- `fn forget(&mut self)` — Forget the stored data bytes and the raced marks, so the next [`Bmi323::read_fresh`] is judged by its flags alone. Setup and the re-arm forget by themselves.
- `fn address(&self) -> u8` — The I2C address.
- `fn interrupt_silent(&self) -> Option<bool>` — The operator view of the INT line (inspect's `interrupt_silent`): None with no wire declared (polled BY DESIGN), false while INT1 drives the sampling, true when a declared line went silent. Reporting true for an unwired board made "not connected" and "connected but dead" read alike, and only the second is a fault.
- `fn wake(&mut self, interrupt_runs: u32) -> bool` — Account one sampling wake: `interrupt_runs` INT1 edges since the last (0 = the period ran out, 1 = healthy, more = an overrun, a scheduling symptom); the telemetry row carries the same count as its `irq_runs` column.
- `fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>>` — Read CHIP_ID PAST the two dummy bytes (a 3-byte read; the third byte is the identity).
- `fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Sample, Fault<Bus::Error>>` — Read one burst: accel and gyro, which also clears data-ready.
- `fn read_fresh<Bus: I2c>(&mut self, bus: &mut Bus) -> Result<(Fresh, Sample), Fault<Bus::Error>>` — Read one sample and which of its halves are new: ONE 16-byte burst from STATUS (the dummy bytes, the STATUS word, then the six data words). Each half is judged by its flag, drdy_acc for the accel and drdy_gyr for the gyro, corrected by its bytes against the last burst's (the module notes); drdy_temp and por_detected are ignored. A frame that is no sample ([`Sample::is_sample`]) is never fresh and leaves the stored bytes as they were. Reading STATUS clears the flags.
- `fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Sample, Fault<Bus::Error>>` — The on-demand self-test (bmi323.probe): the identity reads back, then one burst reads. The firmware logs the accel magnitude with it: gravity is the one reference on a bench, and a part reading far from 1 g at rest is mis-scaled or mis-configured, which a bare "it answered" probe would pass.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus, port: u8) -> String where Bus::Error: core::fmt::Debug,` — Classify a failed setup (bmi323.diagnose), in plan B's words, on the port `port` (`i2c:<port>`). Not the shared one-byte id read, which on this part reads a dummy byte and would call a healthy sensor the wrong device.

### Constants

- `const ADDRESS: u8 = 0x69` — The default I2C address (0x68 when the address pin is pulled the other way).
- `const CHIP_ID: u8 = 0x00` — CHIP_ID: the identity register.
- `const STATUS: u8 = 0x02` — STATUS: the data-ready flags, drdy_acc (bit 7) and drdy_gyr (bit 6), cleared by reading it; ACC_DATA_X follows, so one burst from here reads the flags and the sample together.
- `const ACC_DATA_X: u8 = 0x03` — ACC_DATA_X: the first of six contiguous 16-bit words, accel x, y, z then gyro x, y, z.
- `const ACC_CONF: u8 = 0x20` — ACC_CONF: the accel's mode, range and rate.
- `const GYR_CONF: u8 = 0x21` — GYR_CONF: the gyro's mode, range and rate.
- `const IO_INT_CTRL: u8 = 0x38` — IO_INT_CTRL: the INT1 pin's electrical behaviour.
- `const INT_MAP2: u8 = 0x3B` — INT_MAP2: where the accel and gyro data-ready interrupts go.
- `const CMD: u8 = 0x7E` — CMD: the command register (the soft reset).
- `const IDENTITY: u8 = 0x43` — What CHIP_ID reads on a BMI323 (past the dummy bytes).
- `const SOFT_RESET: RegisterWrite = (CMD, 0xDEAF)` — The soft reset: CMD = 0xDEAF.
- `const SOFT_RESET_SETTLE_MS: u32 = 5` — The wait after the soft reset before the configuration is written.
- `const CONFIGURATION: [RegisterWrite; 2] = [(ACC_CONF, 0x7038), (GYR_CONF, 0x7048)]` — The configuration after the reset, in plan B's order: * ACC_CONF = 0x7038: high-performance mode, +/-16 g, 100 Hz; * GYR_CONF = 0x7048: high-performance mode, +/-2000 dps, 100 Hz.
- `const ROUTE_INTERRUPT: [RegisterWrite; 2] = [(IO_INT_CTRL, 0x0005), (INT_MAP2, 0x0400)]` — Accel data-ready to INT1, written only when an INT1 wire is declared: * IO_INT_CTRL = 0x0005: INT1 output enabled (bit 2), active high (bit 0), push-pull; * INT_MAP2 = 0x0400: acc_drdy -> INT1 (selection 1 at bit 10).
- `const DUMMY_BYTES: usize = 2` — The bytes the part prepends to every I2C register read.
- `const FRAME_LENGTH: usize = DUMMY_BYTES + 12` — The burst read: the dummy bytes, then six signed 16-bit words.
- `const STATUS_FRAME_LENGTH: usize = DUMMY_BYTES + STATUS_WORD_LENGTH + 12` — The status burst from STATUS: the dummy bytes, the STATUS word, then the six data words.
- `const STALE_LIMIT: u32 = 3` — Consecutive reads with no new conversion (neither half fresh) before the firmware calls the part unhealthy: one such read is a tick between two conversions, three in a row are a part that stopped converting.
- `const NO_SAMPLE: i16 = i16::MIN` — What the gyro reports until it has a sample (0x8000): never published as a rate.
- `const START_UP_POLLS: u32 = 50` — How many times setup looks for the gyro's first sample, [`START_UP_POLL_MS`] apart: half a second, against the ~20 ms the bench measured.
- `const START_UP_POLL_MS: u32 = 10` — The wait before each start-up look.
- `const INTERRUPT_SILENT_LIMIT: u32 = 20` — Consecutive timed-out wakes (no INT1 edge) before a DECLARED line is called dead: at 100 Hz a fifth of a second.
- `const ACCEL_COUNTS_PER_G: f32 = 2048.0` — Accel counts per g at +/-16 g.
- `const GYRO_FULL_SCALE_DEGREES_PER_SECOND: i64 = 2000` — The gyro's full scale in degrees per second, over [`GYRO_FULL_SCALE_COUNTS`].
- `const GYRO_FULL_SCALE_COUNTS: i64 = 32768` — The gyro's counts at full scale.

## `drivers::bmm350`

_Tested by `src/bmm350/tests.rs`._

The BMM350 3-axis magnetometer on the SEN0697 (src/glider/drivers/bmm350.py): the `mag` channel.

Why it is driven at all, having been dismissed once: heading. The attitude filter integrates gyro z for heading
and bounds the drift with the GNSS ground TRACK, which works only above the course gate and not at all while the
fix is out. Boost is exactly when a consumer GNSS module loses lock, so the one reference that corrects heading
is missing during the phase that introduces the most drift. A magnetometer is the only sensor on the board that
bounds heading WITHOUT a fix.

RAW COUNTS, deliberately. Turning counts into microtesla needs each part's OTP compensation coefficients, and
that is worth skipping: heading needs a DIRECTION, and the hard/soft-iron fit this airframe needs anyway (carbon,
servo currents, a booster) also absorbs the per-axis sensitivity differences. So the channel carries counts --
raw until the operator calibrates, then centred and normalised to +/-[`UNIT`] -- and only the absolute field
strength is given up, which nothing here wants. The counts are integers by nature (24-bit), and the calibration
is integer extrema of them, so this module computes in integers, as plan B does, to the count: these are not
fixed-point stand-ins for a real number but the part's own unit, and the NVS blob and the telemetry columns
(mx, my, mz) carry them as they are.

Two details worth stating: every I2C register read returns TWO DUMMY BYTES before the data (the BMI323's quirk
too), and each axis is 24-bit two's complement, little-endian.

THE OTP POWER-OFF IS NOT OPTIONAL ([`CONFIGURATION`]). It is Bosch's own init step: without it every data
register reads 0x7F and the part never produces a sample, while its chip id -- and so the old probe -- reads
perfectly. That is how plan B's driver shipped and passed bring-up on the SEN0697 without its magnetometer ever
having measured a field: `mag` published a constant, the calibration refused it as a dead part, and the filter's
magnetic yaw would have pulled toward a fixed vector. [`not_measuring`] is the probe's check for exactly that.

THE CALIBRATION is the operator's: run() accumulates the evidence continuously ([`Bmm350::observe`]: the
extrema of x and y, and which octants of the circle the samples visited), so by the time the operator presses
calibrate after turning the airframe through a level circle the evidence already exists. [`Bmm350::calibrate`]
removes what a single learned offset cannot: the circle's CENTRE (hard iron) and its OUT-OF-ROUNDNESS (soft iron
and per-axis scale). Both shift heading by an amount that depends on where the nose points, so the attitude
filter's one learned track offset cannot absorb either. It refuses a partial turn on ANGULAR COVERAGE, not on the
shape of the spans (see [`Bmm350::calibrate`]).

Register values are Bosch's own (bmm350_defs.h).

THE RATE (avionics-port.md M9 D2, a divergence): 12.5 Hz with averaging 8 (PMU_CMD_AGGR_SET 0x37), where plan B
ran 100 Hz with averaging 4 (0x24) and read it at 10 Hz. Heading moves slowly, so every conversion is read and
recorded at the rate its consumers need, with twice the averaging; the firmware polls every [`POLL_PERIOD_MS`],
half a conversion period, so a sample is at most that old when it is read.

NEW CONVERSIONS ONLY (D3). [`Bmm350::read_fresh`] judges each read by its 9 axis bytes
([`crate::freshness::Changes`]; the dummy bytes and the temperature are not compared): fresh when they changed,
which every conversion does (a still board's noise spans hundreds of counts). Polled twice a conversion, each
conversion is fresh once, and only a fresh field is [`Bmm350::observe`]d, so the evidence counts conversions,
never a read twice. A SUSPENDED part repeats its bytes, and once it has for
[`crate::freshness::STALE_PERIODS`] conversion periods the read is [`Change::Stale`], where the firmware calls it
unhealthy. No register is written for it. Plan B's `period_ms` is unused.

NO SAMPLE IS NOT A SAMPLE. MEASURED on the board (M9 stage 4): right after a re-arm the axes read [`NO_DATA`]
until the first conversion lands. Changed bytes would call that pattern fresh (it differs from the last real
sample), the run would observe it, and the span's x and y maximum would be 8 355 711 for the rest of the boot,
refusing every calibration on coverage. So a field that is no sample ([`Field::measured`] false: raw x and y at
[`NO_DATA`], or all three axes zero) is never judged and never stored: [`Bmm350::read_fresh`] answers it
[`Change::Unchanged`], or [`Change::Stale`] once the stale clock of the last real change runs out, so a part stuck
there still turns unhealthy. And [`Bmm350::observe`] never accounts one as evidence, whoever reads it (the probe
observes too), a divergence from plan B, whose `_read` accounted every burst.

THE AXES ([`crate::axes`], new in the port; plan B has none). The order against the calibration is fixed here, in
the driver, because it is the one place it can go wrong: the evidence ([`Bmm350::observe`]), the calibration
([`Bmm350::calibrate`]) and the NVS blob are all in the SENSOR's own frame, and the map turns only the PUBLISHED
vector onto the airframe's axes, last ([`Bmm350::mounted`]); the firmware does not map `mag` again. Two reasons:
the blob is shared with plan B, which speaks sensor x/y, so a calibration taken on either firmware stays valid
after a reflash to the other whatever the mounting; and a level turn sweeps the sensor's x/y plane only while
that plane is level, so a mounting that tilts the sensor's z off the vertical is refused at construction
([`crate::axes::AxesError::Tilted`]) rather than left to refuse every calibration on coverage, forever.

THE CALIBRATION'S ORDER (D9), plan B's: persist, then apply. The bus owner answers a calibrate with the
[`Calibration`] [`Bmm350::calibrate`] computes, NOT applied; the executor writes its 16 bytes to NVS; only then does
the owner put it in force ([`Bmm350::set_calibration`]). A refused calibration has nothing to persist, and one
that could not be persisted is never applied. At boot the blob is restored after setup ([`Bmm350::restore`]: a
blob of any other length leaves the axes raw). The calibration column's text is [`format_instruction`] of the
values inspect carries (calibrated, sectors, samples), so the executor formats it from the owner's atomics without
borrowing the live part.

Not here (firmware glue): the NVS read and write of the calibration (namespace `coludo`, key [`NVS_KEY`], 16
bytes, shared with plan B so a calibration survives a reflash between the two), the channel push, the telemetry
stream, the probe's log lines and the inspect JSON.

### `pub enum Step`

One step of the bring-up table: a register write, or a wait the part needs before the next one.

### `pub struct Field`

One burst's axes, raw counts (the temperature that follows them is not used).

- `fn decode(frame: &[u8; FRAME_LENGTH]) -> Field` — Decode a 14-byte burst past the dummy bytes: three 24-bit two's-complement axes, little-endian (bmm350._axis). A magnetometer reads NEGATIVE on half of every turn, so an unsigned decode would still give plausible numbers -- and a heading wrong in exactly one half-plane.
- `fn measured(&self) -> bool` — Whether the burst is a sample at all (the raw burst, before any calibration): not x and y at [`NO_DATA`], the part's own "no sample" (until its OTP is powered off, and after a re-arm until the first conversion), and not all three axes zero (the field is never zero on Earth). [`not_measuring`]'s raw checks, without its reasons.

### `pub enum ProbeFailure<BusError>`

Why the probe failed.

### `pub struct Calibration`

A hard/soft-iron calibration: the circle's centre and radii, in counts.

- `fn to_bytes(&self) -> [u8; CALIBRATION_BYTES]` — The NVS blob: four i32 little-endian (plan B's `struct.pack('<4i')`).
- `fn from_bytes(blob: &[u8]) -> Option<Calibration>` — A calibration from an NVS blob; None for any length but [`CALIBRATION_BYTES`] (a short blob is not a calibration, and the axes stay raw).
- `fn correct(&self, field: &Field) -> [i32; 3]` — Re-centre a raw field and scale x and y to +/-[`UNIT`] (the z axis passes raw): all a heading needs, since atan2 cares about the ratio, not the magnitude. Floored division, as plan B's `//`; a zero radius divides by 1 (`radius or 1`). Computed wide and saturated to +/-[`i32::MAX`] -- symmetric, so the axes map can negate any corrected value: only a corrupt blob with a tiny radius gets there.

### `pub struct Span`

The calibration evidence: the extrema of x and y since boot.

- `fn x_span(&self) -> i32` — The x extent.
- `fn y_span(&self) -> i32` — The y extent.

### `pub struct Bmm350`

One BMM350 on an I2C bus, with its calibration evidence.

- `fn new(address: u8) -> Bmm350` — A driver for the part at `address`, mounted per its marks, with no evidence and no calibration.
- `fn mounted(self, axes: Axes) -> Result<Bmm350, AxesError>` — The same driver, mounted as the config's `axes` says ([`Axes::from_config`]): the published vector is turned onto the airframe's axes AFTER the calibration, which stays in the sensor frame (see the module notes). A map that tilts the sensor's z off the vertical is [`AxesError::Tilted`]: the firmware refuses the device at setup with it, naming the key.
- `fn axes(&self) -> Axes` — The sensor-to-airframe map the published vector goes through.
- `fn address(&self) -> u8` — The I2C address.
- `fn calibration(&self) -> Option<Calibration>` — The calibration in force, if any.
- `fn set_calibration(&mut self, calibration: Calibration)` — Put a calibration in force: one [`Bmm350::calibrate`] computed, once the firmware has persisted it (D9), or one restored from NVS ([`Bmm350::restore`]).
- `fn restore(&mut self, blob: &[u8]) -> Option<Calibration>` — Restore the calibration NVS holds, after setup (bmm350._restore), so a power cycle does not cost the operator a turn. Returns the calibration put in force; None for a blob of any length but [`CALIBRATION_BYTES`], which is not a calibration: nothing changes and the axes stay raw.
- `fn calibrated(&self) -> bool` — Whether a hard/soft-iron calibration is in force (inspect's `calibrated`).
- `fn span(&self) -> Option<Span>` — The extrema seen since boot (inspect's `span`).
- `fn seen(&self) -> u32` — Samples seen since boot (inspect's `samples`).
- `fn covered(&self) -> u32` — How many of the eight octants the turn has visited: the progress the operator is working on (inspect's `sectors`).
- `fn instruction(&self) -> String` — The outstanding instruction, or '' once calibrated: what CC shows in the calibration column (plan B's text), [`format_instruction`] of this part's state. No sample seen is no span: both are set by the first [`Bmm350::observe`], and the count saturates rather than wrap back to 0.
- `fn observe(&mut self, field: &Field) -> [i32; 3]` — Account one raw sample and return what the channel publishes (bmm350._read after the bus read): the raw counts while uncalibrated (so the channel works from the first boot and improves when the operator does the turn), the corrected ones once calibrated. On the run, only a [`Change::Fresh`] field is observed (the module notes), so the evidence never counts one conversion twice. A field that is no sample ([`Field::measured`] false) is published as it reads, for the probe's [`not_measuring`] verdict, but never accounted.
- `fn calibrate(&self) -> Result<Calibration, String>` — Compute the hard/soft-iron calibration from the turn the operator has just done (bmm350.calibrate's arithmetic). Nothing is collected HERE: [`Bmm350::observe`] has been gathering the evidence all along, and nothing is applied here either.
- `fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>>` — Read CHIP_ID PAST the two dummy bytes (a 3-byte read; the third byte is the identity).
- `fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Field, Fault<Bus::Error>>` — Read one burst's raw axes (the evidence is accounted by [`Bmm350::observe`]).
- `fn read_fresh<Bus: I2c>(&mut self, bus: &mut Bus, now_us: u64) -> Result<(Change, Field), Fault<Bus::Error>>` — Read one burst's raw axes and whether they are a new conversion ([`Change`]): the same 14-byte burst as [`Bmm350::read`], judged by whether its 9 axis bytes changed since the last judged read, at `now_us` on the owner's clock. Only a [`Change::Fresh`] field is observed and published; at [`Change::Stale`] the firmware calls the part unhealthy (a suspended part). A field that is no sample ([`Field::measured`] false, as after a re-arm) is not judged and not stored: [`Change::Unchanged`], or [`Change::Stale`] once the last real change is three conversion periods old. The probe's read does not touch the judge.
- `fn probe<Bus: I2c>(&mut self, bus: &mut Bus) -> Result<[i32; 3], ProbeFailure<Bus::Error>>` — The on-demand self-test (bmm350.probe): the identity reads back, then one sample is read, accounted (as plan B's probe reads through `_read`, which feeds the calibration evidence; a field that is no sample is not accounted, see [`Bmm350::observe`]) and checked with [`not_measuring`]. Ok carries the published sample.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus, port: u8) -> String where Bus::Error: core::fmt::Debug,` — Classify a failed setup (bmm350.diagnose), in plan B's words, on the port `port` (`i2c:<port>`). Not the shared one-byte id read, which on this part reads a dummy byte and would call a healthy sensor the wrong device.

### `pub fn not_measuring(raw: &Field, published: [i32; 3]) -> Option<&'static str>`

Why a probe's sample says the part is not measuring (bmm350.probe), or None when it is.

`published` is what the channel would carry (raw, or corrected once calibrated) and `raw` the burst as read:
 * all published axes zero: the field is never zero on Earth, so zero means the part is not measuring (a
   SUSPENDED part answers every read with the same counts, which a bare "it answered" probe passes);
 * the RAW x and y at 0x7F7F7F: the part's own "no sample", what it reads until its OTP is powered off. Checked
   on the raw burst, because a calibration would disguise it.

### `pub fn format_instruction(calibrated: bool, covered: u32, seen: u32) -> String`

The calibration column's text from the values inspect carries (plan B's calibration()): '' when a calibration
is in force; the turn asked for with no sample `seen` yet; otherwise the progress, `covered` octants of
[`SECTORS`] and the samples seen. Pure, so the executor formats it from the owner's atomics;
[`Bmm350::instruction`] is this of the live part.

### Constants

- `const ADDRESS: u8 = 0x15` — The default I2C address (I2C_ADSEL high; 0x14 when pulled low).
- `const CHIP_ID: u8 = 0x00` — CHIP_ID: the identity register.
- `const PMU_CMD_AGGR_SET: u8 = 0x04` — PMU_CMD_AGGR_SET: the output data rate and averaging.
- `const PMU_CMD: u8 = 0x06` — PMU_CMD: the power mode command.
- `const MAG_X_XLSB: u8 = 0x31` — MAG_X_XLSB: mag x, y, z then temperature, three bytes each, twelve contiguous bytes.
- `const OTP_CMD_REG: u8 = 0x50` — OTP_CMD_REG: the OTP command register.
- `const CMD: u8 = 0x7E` — CMD: the command register (the soft reset).
- `const IDENTITY: u8 = 0x33` — What CHIP_ID reads on a BMM350 (past the dummy bytes).
- `const CONFIGURATION: [Step; 9] = [` — The bring-up (and re-arm) table, in Bosch's order (bmm350._configure): * CMD = 0xB6, the soft reset, then 25 ms: the part reloads its OTP after a reset; * OTP_CMD_REG = 0x80 (PWR_OFF_OTP), then 5 ms: until this is written the part never measures; * PMU_CMD_AGGR_SET = 0x37: averaging 8 (bits 5:4 = 3), 12.5 Hz (bits 3:0 = 7, Bosch's BMM350_DATA_RATE_12_5HZ; 100 Hz is 4, and plan B wrote 0x24, averaging 4 at 100 Hz). Averaging 8 is allowed at 50 Hz and below. The one byte the port writes differently from plan B (D2), pinned in the golden fixture as a listed divergence; * PMU_CMD = 0x02 (UPD_OAE), then 5 ms: a new rate needs its own apply command; * PMU_CMD = 0x01 (normal mode), then 10 ms.
- `const DUMMY_BYTES: usize = 2` — The bytes the part prepends to every I2C register read.
- `const FRAME_LENGTH: usize = DUMMY_BYTES + 12` — The burst: the dummy bytes, three axes and the temperature, three bytes each.
- `const NO_DATA: i32 = 0x7F_7F7F` — What every axis reads while the OTP is still powered: the part's own "no sample", never a field.
- `const NVS_KEY: &str = "bmm350_cal"` — The NVS key of the calibration (namespace `coludo`): four i32 little-endian, x centre, y centre, x radius, y radius.
- `const CALIBRATION_BYTES: usize = 16` — The calibration blob's length.
- `const UNIT: i64 = 1000` — The corrected axes are normalised to +/- this, so the heading's atan2 sees equally scaled integers.
- `const MINIMUM_SPAN: i32 = 2000` — The span, in counts, below which no field change was seen. MEASURED on the v1.1 bench (2026-09-23): the part's field sphere fitted at ~10 000 counts radius, so a level turn spans roughly 8 000-10 000 counts while a still board's noise spans ~200. It used to be 50 -- below that noise -- which let a board that never moved pass as calibrated.
- `const MINIMUM_REACH: i32 = 1000` — Counts from the centre before a sample may set a sector (still-board noise is ~+/-100).
- `const SECTORS: u32 = 8` — The circle is split into octants; covering them is what proves a real turn.
- `const NEEDED_SECTORS: u32 = 7` — Octants a turn must cover: 7 of 8 is 315 degrees, so a lap that stops just short still counts.
- `const MINIMUM_SAMPLES: u32 = 100` — Samples seen before a calibration is considered at all.
- `const CONVERSION_PERIOD_US: u64 = 80_000` — The conversion period at the configured 12.5 Hz, microseconds.
- `const POLL_PERIOD_MS: u32 = 40` — How often the firmware reads the part, ms: half a conversion period, so each conversion is read within 40 ms of landing (and read twice, fresh once).

## `drivers::bmp280`

_Tested by `src/bmp280/tests.rs`._

The BMP280 barometer on the SEN0253 (src/glider/drivers/bmp280.py), M18: the backup altitude channel of the v0.1
boards (TMS-7C, 7D), where the SEN0697's BMP581 took its place on v1.x. It publishes what the BMP581 does, under
the same names: pressure (Pa), temperature (degrees C), altitude (m above the sea-level datum) and elevation (m above
its own startup ground zero). `update {"rezero": true}` re-captures the ground zero.

THE PART. Unlike the BMP581 it outputs RAW counts: the factory calibration (24 bytes at 0x88: dig_T1..T3,
dig_P1..P9, T1 and P1 unsigned, the rest signed, little-endian) is read once at setup, and each conversion goes
through Bosch's 64-bit integer compensation, plan B's `_compensate` to the bit ([`Calibration::compensate`]; a
degenerate calibration gives 0 Pa, as plan B's). The data frame is 6 bytes from 0xF7, pressure then temperature,
each 20 bits big-endian (msb, lsb, xlsb's top nibble). The configuration is plan B's: IIR filter x4 with a 0.5 ms
standby (CONFIG 0x08), temperature x1 and pressure x4 oversampling in normal mode (CTRL_MEAS 0x2F), about one
conversion every 14 ms. Its power-on mode is SLEEP, so anything that resets it behind the driver's back (the
ICP-10111's general-call reset shares i2c:0 with it on v0.1) stops it converting and it returns its last sample
forever; [`Device::rearm`] writes the mode again, as plan B's `rearm`.

The rest is the BMP581's (crates/drivers bmp581, M9), for one backup barometer to behave as the other: the
double-burst guard against bus garbage and the operating range ([`Outcome::Torn`], [`Outcome::Implausible`]), the
freshness judge on changed bytes ([`Outcome::Stale`] for a part in SLEEP), the guarded ground capture, the
operator's `ground` and `rezero`. The data registers are shadowed for a burst read (the datasheet), so a pair that
disagrees is a conversion landing between the two bursts or garbage, never a half-updated frame.

Units: f32 throughout, the conversions as plan B's single-precision floats make them (the integer pressure to f32,
then the exact division by 256; the integer centidegrees to f32 over 100); the altitude is commons' ISA formula.

Not here (firmware glue): the channel push, the telemetry stream, the freshness of the altitude a re-zero takes,
the probe's log lines and the inspect JSON.

### `pub type RegisterWrite = (u8, u8)`

One register write: (register, value).

### `pub struct Calibration`

The factory calibration: Bosch's dig_T1..T3 and dig_P1..P9.

- `fn decode(bytes: &[u8; CALIBRATION_LENGTH]) -> Calibration` — The calibration from its 24 bytes (little-endian words; T1 and P1 unsigned, the rest signed).
- `fn compensate(&self, adc_temperature: i32, adc_pressure: i32) -> (f32, f32)` — Bosch's 64-bit integer compensation (plan B's `_compensate`, bit for bit): raw counts to (Pa, degrees C). The pressure is 0 when the calibration is degenerate (its divisor 0), as plan B's.

### `pub struct Reading`

One conversion, in the units the channels and the telemetry row carry.

- `fn decode(frame: &[u8; FRAME_LENGTH], calibration: &Calibration) -> Reading` — Decode the 6-byte frame (bmp280._read): pressure then temperature, 20 bits each, big-endian, compensated.

### `pub enum Outcome`

What one [`Bmp280::read_fresh`] yields (as the BMP581's [`crate::bmp581::Outcome`]).

### `pub enum Refusal`

Why an operator `update` was refused (as the BMP581's).

### `pub struct Bmp280`

One BMP280 on an I2C bus.

- `fn new(address: u8) -> Bmp280` — A driver for the part at `address`; the calibration and the ground zero are read at setup.
- `fn address(&self) -> u8` — The I2C address.
- `fn ground(&self) -> f32` — The ground zero, m above the datum.
- `fn elevation(&self, reading: &Reading) -> f32` — Metres above the ground zero: the `elevation` channel.
- `fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>>` — Read CHIP_ID.
- `fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>>` — Read one conversion.
- `fn read_fresh<Bus: I2c>(&mut self, bus: &mut Bus, now_us: u64) -> Result<Outcome, Fault<Bus::Error>>` — Read one conversion and what it is ([`Outcome`]): the frame twice back to back, believed when both agree in range, then judged by its changed bytes at `now_us` (the BMP581's `read_fresh`).
- `fn capture_ground<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<f32, Fault<Bus::Error>>` — Average [`GROUND_SAMPLES`] guarded readings, [`GROUND_SAMPLE_MS`] apart, into the ground zero (plan B's `_ground_zero`, and the port's calibrate as the BMP581's). Nothing changes when a sample fails.
- `fn update(&mut self, properties: &Value, fresh_altitude: Option<f32>) -> Result<Vec<&'static str>, Refusal>` — Apply an operator property change (bmp280.update): `{"ground": <m>}` sets the zero, `{"rezero": true}` takes `fresh_altitude` (None: [`Refusal::NoFreshAltitude`]). Returns the changed property names.
- `fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>>` — The on-demand self-test (bmp280.probe): the identity reads back, then one conversion reads.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String` — Classify a failed setup at the wire level (bmp280.diagnose through the one-byte id read).

### Constants

- `const ADDRESS: u8 = 0x76` — The default address (SDO low; 0x77 when high).
- `const CHIP_ID: u8 = 0xD0` — CHIP_ID: [`IDENTITY`].
- `const CALIBRATION: u8 = 0x88` — The factory calibration's first register: [`CALIBRATION_LENGTH`] bytes.
- `const CALIBRATION_LENGTH: usize = 24` — The calibration's length: dig_T1..T3 and dig_P1..P9, 2 bytes each.
- `const CTRL_MEAS: u8 = 0xF4` — CTRL_MEAS: oversampling and mode.
- `const CONFIG: u8 = 0xF5` — CONFIG: standby time and IIR filter.
- `const DATA: u8 = 0xF7` — The data frame's first register: [`FRAME_LENGTH`] bytes, pressure then temperature.
- `const IDENTITY: u8 = 0x58` — The BMP280's identity.
- `const CONFIGURATION: [RegisterWrite; 2] = [(CONFIG, 0x08), (CTRL_MEAS, 0x2F)]` — Plan B's configuration, in its order: the IIR filter x4 and a 0.5 ms standby, then temperature x1, pressure x4, normal mode.
- `const CONVERSION_PERIOD_US: u64 = 14_000` — One conversion period in normal mode, us: 1 + 2 x 1 + 2 x 4 + 0.5 ms typical measuring (13.3 ms at most) and the 0.5 ms standby, rounded up.
- `const REZERO_WINDOW_US: u64 = STALE_PERIODS * CONVERSION_PERIOD_US` — The re-zero's window: an altitude older than this is no ground to latch.
- `const FIRST_CONVERSION_MS: u32 = 50` — The wait after the configuration for the first normal-mode conversion (plan B's 50 ms).
- `const FRAME_LENGTH: usize = 6` — The data frame's length.
- `const GROUND_SAMPLES: u32 = 8` — Readings averaged into the ground zero (plan B's 8).
- `const GROUND_SAMPLE_MS: u32 = 20` — The gap between ground samples (plan B's 20 ms).
- `const GROUND_TRIES: u32 = 3` — Pairs one ground sample may take before the capture fails ([`Fault::Garbage`]).
- `const PRESSURE_RANGE_PA: RangeInclusive<f32> = 30_000.0..=110_000.0` — The part's pressure range (the datasheet's 300..1100 hPa): agreeing bursts outside it are garbage.
- `const TEMPERATURE_RANGE_C: RangeInclusive<f32> = -40.0..=85.0` — The part's temperature range.

## `drivers::bmp581`

_Tested by `src/bmp581/tests.rs`._

The BMP581 barometer on the SEN0697 (src/glider/drivers/bmp581.py): the BACKUP altitude channel. It publishes
pressure (Pa), temperature (degrees C), altitude (m above the sea-level datum) and elevation (m above its own
startup ground zero), exactly as the BMP280 it replaced, so nothing downstream learns a new name. `update
{"rezero": true}` re-captures the ground zero (after warm-up, just before launch).

THE RE-ZERO (avionics-port.md M9 D7, a divergence). Plan B's BMP581 had no `calibrate`: an operator calibrate
sweep answered Task's no-op success for it and never re-zeroed the backup barometer, so the two barometers' zeros
split at the first re-zero. The port gives it the ICP-10111's surface, and the firmware runs it wherever it runs
the ICP's: `calibrate` re-captures the ground from [`GROUND_SAMPLES`] new readings ([`Bmp581::capture_ground`],
setup's own average), and so does the DONE recovery; `update {"ground": <m>}` sets it (M8's warm start rebases it
to the crumb's pad); `update {"rezero": true}` takes the channel's own altitude when fresher than
[`REZERO_WINDOW_US`].

Simpler than the BMP280 in the way that matters: the part outputs COMPENSATED values, so there is no factory
calibration to read and no compensation polynomial to carry. Pressure is a 24-bit unsigned count of 1/64 Pa;
temperature a 24-bit two's-complement count of 1/65536 degrees C. Reading a sub-zero temperature as unsigned
yields ~+256 degrees C, the kind of wrong that still renders as a plausible number on a chart.

Register values are Bosch's own (bmp5_defs.h), and the configuration is the one the TMS-7 nose logger flew:
pressure enabled at 4x oversampling, 50 Hz, normal mode. The part powers on in STANDBY, so anything that resets
it behind the driver's back stops it converting and it returns its last sample forever -- an altitude that
quietly stops moving. [`Device::rearm`] writes the mode again: on v1.1 the ICP's general call cannot reach it
(the two barometers are on different buses, much of the redundancy's value), but on v0.1 the SEN0697 shares i2c:0
with the ICP, and a later rewire must not silently reintroduce the failure.

NEW CONVERSIONS ONLY (avionics-port.md M9 D2, D3). The firmware polls the part on every owner tick (100 Hz) and
[`Bmp581::read_fresh`] judges each read by its 6 data bytes ([`crate::freshness::Changes`]): fresh when they
changed, which every conversion does (its noise is tens of counts), so each 50 Hz conversion is published once.
No data-ready register is read: the BMP581's needs INT_SOURCE written, a register plan B never wrote. A part in
standby repeats its bytes, and once it has for [`crate::freshness::STALE_PERIODS`] conversion periods the read
is [`Outcome::Stale`], where the firmware calls it unhealthy. Plan B's `period_ms` is unused.

BUS GARBAGE (measured on the taster and 7F, M9 stage 4): about 0.1 % of the 6-byte bursts at 50 Hz on every tick
are isolated single transfers that are not the part's sample -- all 0xFF (262 144 Pa), one byte repeated
(0x1B1B1B, 27 756 Pa), or tears that look plausible (94 577 Pa at a normal 27.7 C; 103 829 Pa at 101 C; 192 100
Pa at -44 C); a direct probe read `fffffffeffff` once in 2560. Changed bytes call every one of them fresh, and
each became a +-9000 m elevation spike and a +-5000 m/s `change`. So `read_fresh` reads the burst TWICE back to
back and believes it only when both are byte-identical ([`Outcome::Torn`] otherwise), then only inside the part's
operating range ([`Outcome::Implausible`] otherwise, for garbage that repeats). Neither is judged: the stored bytes
and the stale clock stay as they were, and the next tick reads again. A conversion that lands between the two
bursts tears the pair too, about one read in 80 (one burst takes ~0.25 ms at i2c:0's 400 kHz, against the 20 ms
period), so most torn pairs are that, not garbage; the next tick,
10 ms later, reads that conversion twice, so none is lost, only delayed by a tick. The ground capture (setup, a
calibrate, the DONE re-zero) takes each of its samples through the same guard, which a latched ground needs most:
one garbage burst in plan B's single reads moved the 8-sample ground by about 1.1 km until the next re-zero. A
sample whose pair is torn or implausible is read again, up to [`GROUND_TRIES`] pairs, and a capture that runs out
fails ([`Fault::Garbage`]) and keeps the ground it had. The guard never touches the freshness judge.

Units (decision D18): every value is f32, as plan B's float channels and telemetry columns are. The conversions
are exact (power-of-two scales over 24-bit counts); the altitude is commons' ISA formula, one definition for every
barometer so two of them can only disagree by the air.

Not here (firmware glue): the channel push, the telemetry stream, the freshness of the altitude a re-zero takes
(the firmware passes it in), the probe's log lines and the inspect JSON.

### `pub type RegisterWrite = (u8, u8)`

One register write of a bring-up table: (register, value).

### `pub struct Reading`

One conversion, in the units the channels and the telemetry row carry.

- `fn decode(frame: &[u8; FRAME_LENGTH]) -> Reading` — Decode the 6-byte burst (bmp581._read): temperature 24-bit two's complement, pressure 24-bit unsigned, both little-endian.

### `pub enum Outcome`

What one [`Bmp581::read_fresh`] yields: the double burst's verdict, with the reading when there is one to use.

### `pub enum Refusal`

Why an operator `update` was refused; nothing has changed when it is.

### `pub struct Bmp581`

One BMP581 on an I2C bus.

- `fn new(address: u8) -> Bmp581` — A driver for the part at `address`; the ground zero is captured at setup.
- `fn address(&self) -> u8` — The I2C address.
- `fn ground(&self) -> f32` — The ground zero, m above the datum.
- `fn elevation(&self, reading: &Reading) -> f32` — Metres above the ground zero: the `elevation` channel, offset-free per sensor.
- `fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>>` — Read CHIP_ID: one byte, no dummy bytes on this part.
- `fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>>` — Read one conversion.
- `fn read_fresh<Bus: I2c>(&mut self, bus: &mut Bus, now_us: u64) -> Result<Outcome, Fault<Bus::Error>>` — Read one conversion and what it is ([`Outcome`]): the 6-byte burst TWICE back to back (see the module notes). Two bursts that differ are [`Outcome::Torn`]; two alike outside the operating range are [`Outcome::Implausible`]; neither is judged. Otherwise the bytes are judged by whether they changed since the last judged read, at `now_us` on the owner's clock: only an [`Outcome::Fresh`] reading is published, and at [`Outcome::Stale`] the firmware calls the part unhealthy (a part in standby). A bus fault on either burst is the transport's, and judges nothing. Setup's and a calibrate's own reads ([`Bmp581::read`]) do not touch the judge.
- `fn capture_ground<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<f32, Fault<Bus::Error>>` — Average [`GROUND_SAMPLES`] readings, [`GROUND_SAMPLE_MS`] apart, into the ground zero (bmp581._ground_zero, which plan B called from setup only; the port's `calibrate` and DONE recovery too, D7). Each sample goes through the garbage guard, a torn or implausible pair read again up to [`GROUND_TRIES`] pairs (the module notes; plan B read each sample once, the golden fixture's listed divergence). Nothing changes when a read fails or a sample runs out of tries ([`Fault::Garbage`]): setup fails its attempt, a calibrate reports it, and the ground in force stays. The freshness judge is not touched.
- `fn update(&mut self, properties: &Value, fresh_altitude: Option<f32>) -> Result<Vec<&'static str>, Refusal>` — Apply an operator property change (bmp581.update): `{"ground": <m>}` SETS the zero directly -- the warm start rebases a rebooted barometer to the crumb's pad altitude, because a mid-air re-zero would make elevation read ~0 at altitude and fake an immediate landing. `{"rezero": true}` re-captures it from `fresh_altitude`, the channel's own value when fresher than three periods (None otherwise: [`Refusal::NoFreshAltitude`]).
- `fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>>` — The on-demand self-test (bmp581.probe): the identity reads back, then one conversion reads.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String` — Classify a failed setup at the wire level (bmp581.diagnose through the shared one-byte id read; correct on this part, which has no dummy bytes): sorted with [`id_classify`].

### Constants

- `const ADDRESS: u8 = 0x47` — The default I2C address (0x46 when the address pin is pulled the other way).
- `const CHIP_ID: u8 = 0x01` — CHIP_ID: the identity register.
- `const TEMP_DATA_XLSB: u8 = 0x1D` — TEMP_DATA_XLSB: temperature xlsb, lsb, msb then pressure xlsb, lsb, msb, six contiguous bytes.
- `const OSR_CONFIG: u8 = 0x36` — OSR_CONFIG: the oversampling and the pressure enable.
- `const ODR_CONFIG: u8 = 0x37` — ODR_CONFIG: the output data rate, the power mode and deep standby.
- `const IDENTITY: u8 = 0x50` — What CHIP_ID reads on a BMP581.
- `const CONFIGURATION: [RegisterWrite; 2] = [(OSR_CONFIG, 0x50), (ODR_CONFIG, 0xBD)]` — The configuration, in plan B's order (the mode the re-arm writes again): * OSR_CONFIG = 0x50: press_en (bit 6), osr_p = 4x (bits 5:3), osr_t = 1x; * ODR_CONFIG = 0xBD: deep standby disabled (bit 7), 50 Hz (bits 6:2 = 0x0F, the 16th of Bosch's ODR table from 240 Hz down, as Linux's bmp580 driver numbers it), normal mode (bits 1:0 = 01): already the 50 Hz D2 asks for.
- `const CONVERSION_PERIOD_US: u64 = 20_000` — The conversion period at the configured 50 Hz, microseconds.
- `const REZERO_WINDOW_US: u64 = STALE_PERIODS * CONVERSION_PERIOD_US` — How fresh the altitude a `rezero` takes must be: [`STALE_PERIODS`] conversion periods (plan B's three periods, at the rate the part now converts and is read).
- `const FIRST_CONVERSION_MS: u32 = 50` — The wait after the configuration for the first normal-mode conversion to complete.
- `const FRAME_LENGTH: usize = 6` — The burst: temperature then pressure, three bytes each, little-endian.
- `const GROUND_SAMPLES: u32 = 8` — Readings averaged into the ground-zero reference, at setup and at a calibrate.
- `const GROUND_SAMPLE_MS: u32 = 20` — The wait after each ground-zero reading: the part converts at 50 Hz, so each reading is a fresh conversion.
- `const GROUND_TRIES: u32 = 3` — The pairs of bursts one ground sample may read before the capture fails ([`Fault::Garbage`]): a torn or implausible pair is read again at once. Garbage is about one burst in a thousand and a conversion tears about one pair in 80, so three in a row is a bus that has stopped answering the part.
- `const PRESSURE_COUNTS_PER_PASCAL: f32 = 64.0` — Pressure counts per Pa.
- `const TEMPERATURE_COUNTS_PER_DEGREE: f32 = 65536.0` — Temperature counts per degree C.
- `const PRESSURE_RANGE_PA: RangeInclusive<f32> = 30_000.0..=125_000.0` — The part's operating pressure range, Pa (datasheet: 300 to 1250 hPa): a burst read twice alike outside it is garbage that repeated, never a sample ([`Outcome::Implausible`]).
- `const TEMPERATURE_RANGE_C: RangeInclusive<f32> = -40.0..=85.0` — The part's operating temperature range, degrees C (datasheet: -40 to 85 C), as [`PRESSURE_RANGE_PA`].

## `drivers::bno055`

_Tested by `src/bno055/tests.rs`._

The BNO055 9-DOF IMU on the SEN0253 (src/glider/drivers/bno055.py), M18: the BACKUP attitude source of the v0.1
boards (TMS-7C, 7D), at priority 2 below the attitude filter. In NDOF mode the chip fuses its own accelerometer,
gyro and magnetometer and reports absolute orientation; [`Sample`] reads the 24-byte ACC..EUL block (accel, mag,
gyro, Euler, all int16 little-endian): the Euler angles (heading, roll, pitch, 16 LSB a degree, in plan B's
attitude convention: the channel carries them as plan B pushed them) for `attitude`, the accelerometer (100 LSB a
m/s², gravity included) in g for `accel` as a low-g backup.

THE STALL (plan B's measured failure, [`Bno055::judge`]). The fusion core can latch: the Euler triple repeats bit
for bit forever while the raw accel and gyro in the same block keep streaming (measured across a power cycle, both
fusion modes, either clock). The channel then stays FRESH, so no staleness guard downstream fires. A still part
legitimately repeats its Euler too, so the judgement is made ONLY WHILE THE PART ROTATES, by its own gyro in the
same block (|gx| + |gy| + |gz| above [`TURNING_LSB`]): [`STALL_SAMPLES`] rotating samples in a row with a frozen
Euler latch the stall, the firmware withholds `attitude` (it goes stale, the databoard moves on) and keeps `accel`
(that half still works), and the first Euler that moves clears it.

THE RESTART (measured on the taster, 2026-10-12). After every switch into NDOF (the bring-up, a re-arm, the
profile's read and write) the fusion reads an all-zero Euler for 60-80 ms before its first real one: a level
attitude that is no measurement. [`Bno055::judge`] answers [`Fusion::Starting`] for those, until the first non-zero
triple.

CALIBRATION (plan B's latch and profile). NDOF does not converge without motion, and the magnetometer is the gate:
the operator's figure-8 takes CALIB_STAT's mag field to 3. The register is the chip's confidence in RECENT data and
falls back to 2 within a minute of sitting still while the learned offsets stay, so convergence is a LATCH
([`Bno055::calibrated`]), set when mag reaches [`MAG_CALIBRATED`] or when a saved profile is restored. The learned
profile ([`PROFILE_BYTES`] at 0x55, readable and writable in CONFIG mode only) is saved by `calibrate`
([`Bno055::read_profile`]) and written back at bring-up ([`Bno055::restore_profile`]), so a power cycle on the rail
needs no figure-8. [`Bno055::owed`] says what is still owed: the figure-8, then the save.

Not here (firmware glue): the channel pushes and the stream, the profile's NVS key, the probe's log lines.

### `pub struct Sample`

One read of the block, in the channels' units.

- `fn decode(block: &[u8; BLOCK_LENGTH]) -> Sample` — Decode the 24-byte block (bno055.sample).

### `pub struct CalibrationState`

CALIB_STAT's four fields, 0..3 each.

- `fn encode(self) -> u8` — CALIB_STAT again: what [`CalibrationState::decode`] reads back.
- `fn decode(raw: u8) -> CalibrationState` — Decode CALIB_STAT.

### `pub enum Fusion`

What [`Bno055::judge`] makes of a sample's fusion.

### `pub struct Bno055`

One BNO055 on an I2C bus.

- `fn new(address: u8) -> Bno055` — A driver for the part at `address`.
- `fn address(&self) -> u8` — The I2C address.
- `fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>>` — Read CHIP_ID.
- `fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Sample, Fault<Bus::Error>>` — Read one sample of the block.
- `fn judge(&mut self, sample: &Sample) -> Fusion` — Judge a sample's fusion (bno055._fusion_alive; see the module notes): a moved Euler clears the stall, a frozen one counts a strike only while the part's own gyro says it rotates, [`STALL_SAMPLES`] strikes latch it.
- `fn stalled(&self) -> bool` — Whether the fusion is latched stalled.
- `fn poll_calibration<Bus: I2c>(&mut self, bus: &mut Bus) -> Result<bool, Fault<Bus::Error>>` — Read CALIB_STAT (bno055._poll_calibration's read; the firmware paces it to ~1 Hz). Returns true when this read set the convergence latch (log it once).
- `fn calibration_state(&self) -> Option<CalibrationState>` — The last CALIB_STAT read, None before the first.
- `fn calibrated(&self) -> bool` — Has the magnetometer ever converged this session, or was a profile restored: the latch, which the live register regularly disagrees with (it regresses while the offsets stay).
- `fn restored(&self) -> bool` — The latch came from a restored profile.
- `fn owed(&self) -> Option<String>` — What is still owed (bno055.calibration): [`owed`] of this part.
- `fn read_profile<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<[u8; PROFILE_BYTES], Fault<Bus::Error>>` — The learned profile, for `calibrate` to save (bno055._save_profile): CONFIG mode for the read, NDOF again afterwards whatever happened, so a failed read never leaves the fusion off. The fusion pauses ~50 ms: a ground action, never on the poll path. Marks the profile saved only when the firmware says it persisted it ([`Bno055::mark_saved`]).
- `fn mark_saved(&mut self)` — The firmware persisted the profile: nothing more is owed.
- `fn saved(&self) -> bool` — The learned profile is saved (restored, or `calibrate` saved it).
- `fn restore_profile<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs, profile: &[u8; PROFILE_BYTES]) -> Result<(), Fault<Bus::Error>>` — Write a saved profile back (bno055._restore_profile): CONFIG mode, the profile, NDOF again; sets the latch and the saved flag, so a boot with valid offsets owes nothing.
- `fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Sample, Fault<Bus::Error>>` — The on-demand self-test (bno055.probe): the identity reads back, then one sample.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String` — Classify a failed setup at the wire level (bno055.diagnose through the one-byte id read).

### `pub fn owed(state: Option<CalibrationState>, converged: bool, saved: bool) -> Option<String>`

What is still owed (bno055.calibration), from the last CALIB_STAT read (`state`, None before the first), the
convergence latch and whether the profile is saved: the figure-8 while unconverged, then the save; None once both
are done. A free function, so the firmware words it from its run's published state, off the bus owner.

### Constants

- `const ADDRESS: u8 = 0x28` — The default address (COM3 low; 0x29 when high).
- `const CHIP_ID: u8 = 0x00` — CHIP_ID: [`IDENTITY`].
- `const OPR_MODE: u8 = 0x3D` — OPR_MODE: the operating mode.
- `const PWR_MODE: u8 = 0x3E` — PWR_MODE: the power mode.
- `const DATA: u8 = 0x08` — The ACC..EUL block's first register: [`BLOCK_LENGTH`] bytes.
- `const CALIB_STAT: u8 = 0x35` — CALIB_STAT: sys bits 7..6, gyr 5..4, acc 3..2, mag 1..0; 3 is fully calibrated.
- `const PROFILE: u8 = 0x55` — The learned profile's first register (ACC_OFFSET_X_LSB .. MAG_RADIUS_MSB): [`PROFILE_BYTES`].
- `const IDENTITY: u8 = 0xA0` — The BNO055's identity.
- `const MODE_CONFIG: u8 = 0x00` — CONFIG mode: the fusion off, the profile readable and writable.
- `const MODE_NDOF: u8 = 0x0C` — NDOF mode: full 9-DOF absolute-orientation fusion.
- `const POWER_NORMAL: u8 = 0x00` — The normal power mode.
- `const DEFAULT_PERIOD_MS: u32 = 20` — The block read's default period (plan B's `period_ms` 20: 50 Hz; the fusion runs at 100 Hz inside).
- `const CALIBRATION_PERIOD_MS: u32 = 1000` — How often the firmware reads CALIB_STAT for the operator (plan B's ~1 Hz).
- `const NVS_KEY: &str = "bno055_calib"` — The NVS key of the saved profile (plan B's `bno055_calib`, shared with it).
- `const MODE_SETTLE_MS: u32 = 25` — The wait after a mode switch (plan B's 25 ms; the datasheet's 7 ms to CONFIG, 19 ms from it).
- `const BLOCK_LENGTH: usize = 24` — The block's length: accel, mag, gyro, Euler, 6 bytes each.
- `const EULER_OFFSET: usize = 18` — The Euler triple's offset in the block.
- `const GYRO_OFFSET: usize = 12` — The gyro's offset in the block.
- `const PROFILE_BYTES: usize = 22` — The profile's length.
- `const TURNING_LSB: i32 = 80` — |gx| + |gy| + |gz| above this (16 LSB a deg/s, ~5 deg/s summed) is a part that rotates: a frozen Euler is a fault.
- `const STALL_SAMPLES: u32 = 50` — Rotating samples in a row with a frozen Euler that latch the stall: ~1 s at the 50 Hz default.
- `const MAG_CALIBRATED: u8 = 3` — CALIB_STAT's mag level that means the figure-8 is done.

## `drivers::commons`

_Tested by its own `mod tests`._

The drivers' half of src/glider commons.py: the primitives more than one device needs, each next to the plan B
name it ports. The flight-math half is in crates/control (`commons`), the stage machine's in crates/sequencer.

Plan B's `_upy`/`_opt` variant pairs (bytecode against viper) are a MicroPython speed workaround and are not
ported: each primitive is one plain function.

* [`sensirion_crc8`] -- the CRC-8 over one two-byte word of a Sensirion (SDP8xx) or TDK (ICP-101xx) frame.
* [`id_classify`] -- an identity byte read back after a failed setup, sorted into the most likely wiring fault.
* [`python_float`] -- an operator's JSON property read as plan B's `float()` reads it (a builtin, not commons.py,
  but every barometer's `update` applies it, so it lives with the other shared primitives).

### `pub fn sensirion_crc8(first: u8, second: u8) -> u8`

The CRC-8 over the two data bytes of one Sensirion frame word (commons.sensirion_crc8): polynomial 0x31
(x^8 + x^5 + x^4 + 1), seed 0xFF, no reflection, no final XOR. The datasheet's worked example is 0xBEEF -> 0x92.

The ICP-10111 uses the same CRC over its words, so both drivers check frames with this one function.

### `pub fn id_classify(read: Option<u8>, expected: u8) -> String`

Sort an identity byte read back from a part into an operator-readable wire-level diagnosis (commons.id_classify).

The deeper 'why' a driver's diagnose answers when its setup failed, so `verify`/`probe` say 'chip-select not
asserting' instead of only 'absent / miswired?'. `read` is None when the read itself failed (no I2C ack, an SPI
error); `expected` is the part's documented identity byte. The texts are plan B's, word for word: the operator
and the dashboard read them on both firmwares.

### `pub fn python_float(value: &Value) -> Option<f32>`

An operator's JSON property as plan B's `float(props[key])` reads it, or None where `float()` raised.

`float()` is more lenient than a JSON number: it takes a numeral STRING ("12.5", with surrounding whitespace,
an exponent, a sign) and a bool (True is 1.0), so `{"ground": "12.5"}` set a ground on plan B and must here too.
It raises on a non-numeric string, a null, a list and an object.

Two deliberate differences, both refusals of values `float()` took: a value that is not finite in f32 ("nan",
"inf", or a magnitude past f32's range), since a NaN or infinite ground would poison every later elevation --
the endgame band, the landing fallback and the launch backup -- and no JSON number can be one, so only a typed
string carries it; and CPython's digit-group underscores ("1_000"), which Rust's number parser does not take
and no operator types into a ground.

## `drivers::debounce`

_Tested by `src/debounce/tests.rs`._

A debounce for a two-level input behind a latch, in two stamps: when the input first moved and when it last moved.
It knows edges and times, not what a level means: the owner keeps the latch and decides what a concluded episode
means from the level it reads at the conclusion. A remove-before-flight pin or a breakwire would be owners alike.

THE MODEL (the operator's): an EPISODE is the edges no quiet time separates. Its first edge sets the start of the
[`StampPair`], every edge its finish ([`StampPair::record`]: wait-free and allocation-free, so the input's both-edge
interrupt calls it). The owner's tick takes the pair once the finish is `quiet_us` old ([`StampPair::take`]), and
the [`Episode`] it gets is the first and the last edge; the pair is empty again for the next one. The owner reads
the level for that same tick and decides by it: a level the latch does not hold moves the latch, the level it
holds is a blip (the input moved and came back). No count of reads and no deadline: an input that never holds still
for the quiet keeps its episode open, dated by the first edge of the whole run. An episode ends when a tick SEES it
quiet, so an edge that comes after the quiet ran out but before the tick took the pair is still the episode's: the
episode is the edges no gap of a quiet and the tick's own delay separates.

THE TICK'S OWN READ is evidence too ([`StampPair::observe`]): when no episode is open and the level the tick reads
is not the latch's, the tick opens one at its read. That covers a change before the interrupt was armed, an edge
the interrupt lost, and a latch that started from a level the input has since left: the latch follows the input a
quiet after the read that saw it, with or without an interrupt. A read of the latch's own level records nothing.

A POLLED input (no interrupt: none wired, or arming it failed) feeds the same pair from its reads
([`StampPair::poll`]): a read whose level differs from the previous read's is an edge at the read's time, any other
read is observed. One path for both: a polled episode is dated by its reads, so its moment and its duration are
quantized to them, and a change and its undoing between two reads go unseen.

THE TICK, in order: read the level, read the clock, observe (or poll), take, and on an episode decide with that
same level. The level comes before the take: an edge after the take's load belongs to the next episode, so it can
never decide this one. The residual: an edge in the instant before the read, inside the interrupt's entry latency
(well under a µs on one core), is in the level yet reaches the pair after the take, so it decides the concluding
episode as well as starting the next one, which then decides again by its own read. At worst a blip whose quiet
ran out on this very tick concludes on the next episode's first edge: the latch moves, and the next episode moves
it back if the line returns; an owner whose move cannot be undone states what that costs it.

THE RACE: the interrupt may record while the tick takes. A take is decided by its load of the finish: an edge
recorded before that load is the episode's (it is the latest, so the episode is not quiet and goes on), any edge
after it is the next episode's. The take then releases the start and reloads the finish: an edge between the load
and the release stored a finish but found the start set, so the take re-seeds the start with it, and an edge after
the release started the next episode itself. So no episode loses its start and no later edge joins a concluded
episode. The residual: when two or more edges land inside those few instructions (stretched by the interrupts' own
run time, a few µs), the next episode starts at a later one of them, late by at most that span; its finish, and
the episode concluded, are exact. The tick's own read opens an episode by an exchange from empty, and sets the
finish only where no edge stored one since: an edge the interrupt records meanwhile is never overwritten by the
read's older time. Only the tick observes, polls and takes.

THE MEMORY ORDER is sequentially consistent throughout. A take releases the start, then reloads the finish; an edge
stores the finish, then exchanges the start: each side writes one word and then reads the other, which only one
total order makes safe across cores (else a take could miss an edge that also missed its release, and the next
episode would never start). The fences cost a few instructions per edge and per tick.

ONE CORE is what THE RACE assumes: the interrupt runs to its end before the tick goes on, so a record is one step
for the tick. On the board it is (the interrupt and the tick share core 1). Across cores a record's two writes can
straddle the tick's few instructions, and two residuals come in, both for an edge inside them. A take that loads
the edge's finish but releases the start before the edge's exchange concludes the edge, and the exchange then
opens the next episode with it too; a take concludes only an edge a quiet old, so that needs a record that outlasts
the quiet: in practice a quiet of 0. The next take concludes that edge again (a single-edge episode at the
concluded finish), or with the edges after it, dated from it. An observation the edge straddles may date the
episode by the read's time or the edge's, whichever the race leaves, never with a finish before the start. Closing
the first would take a third word (the concluded finish); an interrupt moved off the tick's core is the owner's to
weigh against it.

THE CLOCK is the caller's wrapping u32 of microseconds (esp_timer's, truncated): it wraps every 71.6 minutes. A
stamp of 0 means empty, so an edge that would stamp 0 stamps 1, a µs late. The quiet is an elapsed time, a wrapping
difference read as i32 ([`elapsed_us`]), valid within 35.8 minutes, so the clock may wrap mid-episode; a finish
after the take's `now_us` (an edge between the caller's clock read and the take) is a negative interval, never
quiet. How long a quiet is sane is the config validator's to bound (board_config's `LONGEST_DEBOUNCE_MS`).

### `pub struct Episode`

One episode, as [`StampPair::take`] concludes it.

- `fn duration_us(&self) -> u32` — From the first edge to the last, µs (wrapping): 0 for a single edge.

### `pub struct StampPair`

The open episode's first and latest edge, shared by the input's interrupt and the owner's tick (see the module
notes). A static: [`StampPair::new`] is const.

- `const fn new() -> StampPair` — An empty pair: no episode open.
- `fn record(&self, now_us: u32)` — An edge at `now_us`: it is the finish, and the start when no episode is open. Wait-free, for an interrupt.
- `fn observe(&self, now_us: u32, level: bool, latched: bool)` — The tick's own read, `level` at `now_us`, against the level the owner's latch stands for, `latched`: when they differ and no episode is open, it opens one at the read (THE TICK'S OWN READ). Anything else records nothing.
- `fn poll(&self, now_us: u32, level: bool, previous: bool, latched: bool)` — A polled input's read, `level` at `now_us` (no interrupt): an edge when it differs from the `previous` read's level, else [`StampPair::observe`]d against `latched`. Nothing else writes the pair of a polled input, so the edge is a plain [`StampPair::record`]. The caller keeps `previous`: at first the level its latch started from.
- `fn episode_open(&self) -> bool` — Whether an episode is open: an edge, or a read off the latch, came and no take has concluded it yet. The owner asks it after its take, to know the input has held its level since the last episode concluded (an edge an instant later is the next tick's to see).
- `fn take(&self, now_us: u32, quiet_us: u32) -> Option<Episode>` — Take the open episode once it is quiet at `now_us`: its latest edge `quiet_us` old or more.

### `pub fn elapsed_us(now_us: u32, then_us: u32) -> i32`

The signed microseconds from `then_us` to `now_us` on the wrapping u32 clock: negative when `now_us` is the earlier
one. Valid while the two are less than half the clock's period apart, 35.8 minutes.

## `drivers::device`

_Tested by its own `mod tests`._

The device half of src/glider task.py: what every bus driver offers its owner ([`Device`]), the fault a device
conversation ends in ([`Fault`]), and the consecutive-failure counter the run loops escalate on ([`Strikes`],
plan B's `Task.strike`).

Plan B's Task also owns the run loop, the channels, the telemetry stream and the inspect/update surface. In Rust
those belong to the bus owner thread and its per-part glue (avionics-port.md §4.1: one owner per resource), so a
driver is a passive object: the owner hands it the bus and a delay for each conversation, and serves its own
mailbox between conversations. That is also why plan B's `Task.claim()` has no counterpart here: the owner
thread is the only caller there is.

The general-call rule (operator, 2026-10-06; plan B commit 76c138c): the ICP-10111 un-wedges a latched part with
an I2C general-call reset, which resets EVERY device on that bus that honours it. The bus owner then calls
[`Device::rearm`] on every device registered on that port, found by membership, never by a hard-coded list.
Every driver offers it, including the ones plan B left to recover on their own failure path.

### `pub trait Device<Bus>`

One device on a bus, as its owner thread drives it.

`Bus` is the transport (an embedded-hal `I2c`, or an `SpiDevice` for the SPI parts); a driver implements the
trait for every bus type that carries it, so the firmware's `I2cPort` and the host tests' scripted bus run the
same code. The methods take the delay as a `dyn` so the owner can keep its devices as `dyn Device` and walk
them by membership after a general call.

- `fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Self::BusError>>` — Bring the part from its power-on (or an unknown) state to producing data: plan B setup()'s bus half.
- `fn rearm(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Self::BusError>>` — Restore what a bus-wide reset (the ICP's general call) took from the part, so it produces valid data again.

### `pub enum Fault<BusError>`

Why a device conversation failed.

### `pub struct Strikes`

A run of CONSECUTIVE failures, acted on once (plan B `Task.strike`).

Four plan B drivers had hand-rolled this (icp10111 read failures -> general-call reset, sdp810 -> continuous-mode
restart, lsm6dso32 INT1 timeouts -> poll fallback, bno055 frozen fusion -> withhold attitude) and they had already
drifted apart: two fired at `== limit`, one at `>= limit`, each with its own counter and reset. The subtle part
is the same every time: act ONCE while the fault persists, not on every tick, and rearm only on a genuine
success.

- `fn strike(&mut self, failed: bool, limit: u32) -> bool` — Count one pass: true exactly ONCE, on the failure that brings the run to `limit`; false on every other pass, further failures included. A good pass (`failed` false) rearms the count.
- `fn count(&self) -> u32` — The consecutive failures so far.

## `drivers::freshness`

_Tested by `src/freshness/tests.rs`._

New conversions only (avionics-port.md M6, M9 D1, D1b and D3): how a driver read on its owner's tick tells a
conversion it has not returned before from one it has. Plan B pushed every read, but its reads were slower than
the parts; the port's tick is not, so without this a part slower than the tick would publish an old sample as a
new one, and a part in standby would publish its last sample forever. No register is written for it: the parts
are read as plan B configured them. Two judges, one per kind of part:

* [`Flagged`], for the IMUs (the LSM6DSO32 and the BMI323), whose one burst carries a data-ready flag per half
  (accel, gyro) next to the sample. THE FLAG DECIDES; THE BYTES ONLY CORRECT IT. Bytes cannot change without a
  conversion, but a new conversion can repeat the last one's bytes: the gyro's LSB is 0.061-0.07 dps, so at rest
  two conversions in a row are often bit-identical, and bytes alone would drop real pushes. The flags and the
  output are read in one frame, yet not at one instant, so a conversion can land between the status byte and the
  data: the burst then carries a NEW sample under a CLEAR flag. Two corrections: (a) a half with its flag clear
  but its bytes changed since the last read is that raced conversion: fresh, and marked raced; (b) a half with
  its flag set over bytes identical to the last read's, when that read pushed it as raced, is the raced
  conversion's own flag read late: the duplicate, not fresh. A flagged half is otherwise never called stale for
  identical bytes (it is counted as `identical`, the measured residual: a raced conversion whose half is
  bit-identical to the one before is lost, as one with no flag is). After [`Flagged::forget`] nothing is stored
  and a read is judged by its flag alone.

* [`Changes`], for the BMP581 and the BMM350, read with no data-ready register (the BMP581's needs INT_SOURCE
  written, a register plan B never wrote): fresh when the data bytes changed. Their noise is tens to hundreds of
  counts, so two real conversions practically never repeat every byte, and a part in standby, which returns its
  last sample forever, reads the same bytes: [`Change::Unchanged`] while that is a part slower than its poll, and
  [`Change::Stale`] once it has lasted [`STALE_PERIODS`] conversion periods, where the firmware calls the part
  unhealthy.

A frame a driver knows is not a sample (an impossible status, an all-zero burst, a 0x8000 start-up marker) never
reaches a judge: the driver answers it not fresh and the stored bytes stay those of the last real sample.

### `pub struct Fresh`

Which halves of an IMU burst are new conversions (push `accel`, push `rate`), and how the bytes corrected the
flags, per half (`inspect control`'s `raced` and `identical` counters).

### `pub struct Flagged`

The flag-with-byte-corrections judge of an IMU's two halves (see the module notes): the last judged read's bytes
and, per half, whether that read pushed it as raced.

- `fn judge(&mut self, accel_flag: bool, accel: &[u8; HALF_LENGTH], rate_flag: bool, rate: &[u8; HALF_LENGTH]) -> Fresh` — Judge one burst's halves by their data-ready flags, corrected by their bytes, and store the bytes for the next.
- `fn forget(&mut self)` — Forget the stored bytes and the raced marks: the next read is judged by its flags alone (at setup, a re-arm, a bus re-add, and when the firmware resumes reads it skipped).

### `pub enum Change`

What a [`Changes`] read says about its sample.

### `pub struct Changes<const LENGTH: usize>`

The changed-bytes judge of a part read without a data-ready flag (see the module notes): the last read's data
bytes and when they last changed.

- `fn judge(&mut self, bytes: &[u8; LENGTH], now_us: u64, period_us: u64) -> Change` — Judge one read's data `bytes`, taken at `now_us`, on a part converting every `period_us`.
- `fn hold(&mut self, now_us: u64, period_us: u64) -> Change` — Answer a read that is no sample at all (a part's own "no data" pattern), taken at `now_us`, WITHOUT judging it: nothing is stored, so the next real conversion is judged against the last real one. It is [`Change::Unchanged`] until the stale clock runs out, then [`Change::Stale`], so a part that never leaves the pattern still turns unhealthy. With nothing judged yet, the clock starts at the first such read.

### Constants

- `const HALF_LENGTH: usize = 6` — The bytes of one half of an IMU sample: three axes, two bytes each.
- `const STALE_PERIODS: u64 = 3` — The conversion periods a [`Changes`] part may repeat its bytes before it is [`Change::Stale`] (D3: a standby part goes stale, then unhealthy after three periods; plan B's re-zero takes an altitude fresher than three, too).

## `drivers::icp10111`

_Tested by `src/icp10111/tests.rs`._

The ICP-10111 barometer (TDK ICP-101xx, on the SEN0517; src/glider/drivers/icp10111.py): the PRIMARY altitude
channel, 8.5 cm accuracy. It publishes pressure (Pa), temperature (degrees C), altitude (m above the sea-level
datum) and elevation (m above its own startup ground zero), which drives the endgame band, the landing fallback
and the launch backup.

Command-based, not register-mapped: [`Device::setup`] verifies the product id, reads the four OTP calibration
constants and averages the ground zero; [`Icp10111::poll`] runs the TWO-PHASE measurement the bus owner needs --
issue "measure, pressure first, normal mode" ([`MEASURE`], 6.3 ms and 1.6 Pa RMS), then collect the 9-byte frame
at least [`MEASURE_MS`] later -- so the owner's other devices keep their slots while the part converts. Plan B
slept through the conversion in its own asyncio task; the bus owner thread cannot sleep, so the wait became a
phase with a deadline, and the time arrives as an argument.

THE CADENCE ON THE TICK. The rule is plan B's: the next measurement is issued a period after the collect. The
owner polls on the 100 Hz control tick, so each phase lands on a wake: the issue at 0, the collect on the first
wake at least [`MEASURE_MS`] later (20 ms), and the next issue a period after that collect. With the default
100 ms period that deadline falls on a tick boundary (120 ms) and is met only when that wake lands no earlier
after its tick than the collect's did; a wake even 1 us earlier misses it and issues on the next tick (130 ms).
So the board's cycle is 120 or 130 ms, about 8 Hz, never the 10 Hz the period suggests (plan B's own loop,
sleeping a period after its read, ran at ~112 ms). Anchoring the next issue to the previous issue instead would
give 10 Hz, if the detectors fed by the elevation slope ever need a regular cadence.

THE WIRE RULE: only what the wire says about the part strikes -- a NAK, a lost arbitration, a bus error, a CRC
reject. A bus failure the port reports as [`ErrorKind::Other`] never reached the part: the firmware's `BusBusy`
(the `i2c_master` bus lock not had in time, because another handle on the port held it -- the camera's SCCB on
7C's i2c:0) or a driver refusal. It is reported ([`Poll::Failed`], never `recovering`), the next issue comes a
period later, and the run of strikes is left exactly as it was, neither counted nor rearmed: counted, a streaming
camera would fire general calls that reset every part on the port for nothing; rearmed, it would hide a wedge in
progress. The recovery's own general call obeys the same rule: one the port refuses reset nothing, so it is
reported the same way and goes again a period later, and [`Poll::GeneralCall`] and [`Poll::Rearm`] wait for the
call that reached the wire (a NAK there is still ignored: nothing may honour it). Reported anyway, they would send
the owner walking the peers for a reset that never happened and spend the run: a part latched at that moment
would never escalate again. Likewise setup tells the owner of a general call only when it reached the wire. Plan B
never met this failure (no camera on its buses).

THE CRCs: the part appends a CRC-8 to each of its three words, and plan B once read them and threw them away.
That is the expensive kind of silence: a corrupted I2C word does not raise, it yields a plausible altitude. This
part has a documented latch-up habit on this board, which is exactly the state that puts bad bytes on the wire,
so a frame failing any CRC is REFUSED and takes the same path as a part that stopped answering.

THE GENERAL CALL. Measured after the 2026-07-06 OOM-soak panic: a mid-conversion part NAKs until the conversion
drains, and an unclean reboot can LATCH its digital core -- the address still acks (a scan sees 0x63) but every
command NAKs. The addressed soft reset (0x805D) RE-WEDGED the latched bench part; the I2C GENERAL-CALL reset
(0x06 to address 0x00) recovered it fully (5 of 5 measurements after). So setup touches the part plainly (a clean
boot never needs a reset), drains a possible busy window on the first NAK, and fires the general call only from
the second; and the run loop, after [`RECOVER_AFTER`] consecutive failures, waits a period for a busy
conversion to drain, fires the general call, and lets the bus settle.

The general call resets EVERY part on the bus that honours it, back to its power-on state (the operator's rule,
2026-10-06; plan B commit 76c138c): the bus owner then calls [`Device::rearm`] on every OTHER device registered on
that port, found by membership. Plan B walked its registry from inside this driver; here the driver reports
it -- [`Icp10111::take_general_call`] after setup, [`Poll::Rearm`] from the run loop -- and the owner, who holds
the registry, walks it. One deliberate difference: plan B re-armed the peers only when its setup then SUCCEEDED,
so a general call followed by a dead ICP left the SEN0697 parts set up before it (7C, i2c:0) in their power-on
state; the port reports every general call that went out, success or not.

THE POLYNOMIAL ([`compensate`]) is TDK's, in f32 (decision D18), and stays f32 by measurement (avionics-port.md
Q8): the golden fixture pins plan B's answer twice, under CPython's f64 and under the board's single precision
(MicroPython on the P4 computes it in f32, op by op), and the port matches the board's EXACTLY, bit for bit, while
staying within 0.024 Pa of the f64 over the fixture's cases (0.032 Pa over a denser sweep of 30-110 kPa and
-40..85 degrees C) -- a fiftieth of the part's 1.6 Pa RMS noise, under 3 mm of altitude. Plan B cached the
temperature-dependent half per 37 raw counts (~0.1 degrees C) to stop allocating floats with the GC off in flight;
that cache is a MicroPython workaround (avionics-port.md §6, not ported), so the port computes the whole
polynomial per sample, which moves the answer by up to 0.39 Pa against plan B's cached one (3.3 cm; the fixture's
`measured`) -- a larger change than f32 makes, and toward the part's true reading.

Not ported: plan B's conversation guard and sample cache (`_read`, `claim`): the owner thread is the only caller.
For the same reason the probe ([`Icp10111::probe`]) is one direct conversion, where plan B's issued no I2C and
waited for the run loop's FRESH pressure (avionics-port.md M5 design decisions, 'Device probes vs bus probe').
Not here (firmware glue): the channel push, the telemetry stream, the freshness of the altitude a re-zero takes,
the log lines and the inspect JSON.

### `pub type Otp = [i16; OTP_WORDS]`

The four OTP calibration constants, signed 16-bit, read once at setup.

### `pub struct Coefficients`

The temperature-dependent half of the conversion (icp10111._calibrate): the triple the pressure line needs.

### `pub struct Reading`

One conversion, in the units the channels and the telemetry row carry.

- `fn convert(otp: &Otp, pressure_raw: u32, temperature_raw: u16) -> Reading` — Convert a frame's raw words with the part's OTP.

### `pub enum Poll<BusError>`

What one [`Icp10111::poll`] did, for the owner to act on.

### `pub enum Refusal`

Why an operator `update` was refused; nothing has changed when it is.

### `pub struct Icp10111`

One ICP-10111 on an I2C bus.

- `fn new(address: u8, period_ms: u32) -> Icp10111` — A driver for the part at `address`, measuring every `period_ms` (`period_ms` in the config, default [`DEFAULT_PERIOD_MS`]).
- `fn address(&self) -> u8` — The I2C address.
- `fn period_ms(&self) -> u32` — The measurement period, ms (`period_ms`): a re-zero takes an altitude fresher than three of them.
- `fn otp(&self) -> Option<Otp>` — The OTP constants setup read.
- `fn ground(&self) -> Option<f32>` — The ground zero, m above the datum.
- `fn elevation(&self, reading: &Reading) -> Option<f32>` — Metres above the ground zero: the `elevation` channel (None before a ground zero is held).
- `fn instruction(&self) -> &'static str` — The outstanding instruction, or '' once a ground zero is held (plan B's calibration()).
- `fn take_general_call(&mut self) -> bool` — Whether a general call went out since the last ask (setup's first-touch escalation), cleared as it reports: on true the owner re-arms every other device registered on this port.
- `fn capture_ground<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<f32, Fault<Bus::Error>>` — Average [`GROUND_SAMPLES`] fresh conversions into the ground zero (icp10111._ground_zero; the calibrate path too): each is a genuine conversion, back to back, never a cached sample, or the average would collapse to one reading.
- `fn probe<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<Reading, Fault<Bus::Error>>` — The on-demand self-test (icp10111.probe, adapted per avionics-port.md M5 design decisions, 'Device probes vs bus probe'): one blocking conversion -- the command, the [`MEASURE_MS`] wait, the CRC-checked frame (icp10111._measure) -- converted with the part's OTP. [`Fault::Configuration`] before setup has read the OTP, and nothing is sent.
- `fn poll<Bus: I2c>(&mut self, bus: &mut Bus, now_us: u64) -> Poll<Bus::Error>` — One step of the run loop at `now_us` (the owner's clock): at most one transfer, never a wait.
- `fn update(&mut self, properties: &Value, fresh_altitude: Option<f32>) -> Result<Vec<&'static str>, Refusal>` — Apply an operator property change (icp10111.update): `{"ground": <m>}` SETS the zero directly (the warm start rebases a rebooted barometer to the crumb's pad altitude: a mid-air re-zero would read ~0 at altitude and fake an immediate landing); `{"rezero": true}` re-captures it from `fresh_altitude`, the channel's own value when fresher than three periods (None: [`Refusal::NoFreshAltitude`]). Returns the changed property names.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String` — Classify a failed setup (icp10111.diagnose): re-issue the product-id command and sort the masked word with [`id_classify`] (command-based, so not the bus's register diagnosis, but the same categories).

### `pub fn coefficients(otp: &Otp, temperature_raw: u16) -> Coefficients`

TDK's ICP-101xx calibration: a raw temperature and the OTP constants to the (a, b, c) triple
(icp10111._calibrate). The raw pressure appears nowhere in it.

Every operation is plan B's, in plan B's order, so the f32 result is the board's to the bit (see the module
notes): `t x t` is an integer first (|t| <= 2^15, so it fits), then one rounding to f32.

### `pub fn compensate(otp: &Otp, pressure_raw: u32, temperature_raw: u16) -> f32`

The TDK polynomial: raw pressure and temperature to pressure in Pa (icp10111._compensate, without plan B's
per-37-counts cache of the triple: see the module notes).

### `pub fn temperature(temperature_raw: u16) -> f32`

The temperature in degrees C of a raw temperature word: -45 + 175 x raw / 2^16 (exact but for the one sum).

### `pub fn decode(frame: &[u8; FRAME_LENGTH]) -> Option<(u32, u16)>`

The raw words of a measurement frame, after every CRC passed: (pressure, 24-bit; temperature, 16-bit); None when
any of the three words fails its CRC.

### Constants

- `const ADDRESS: u8 = 0x63` — The fixed I2C address.
- `const READ_ID: [u8; 2] = [0xEF, 0xC8]` — Read the product id: a 16-bit word (and its CRC), whose low six bits are 0x08 on an ICP-101xx.
- `const GENERAL_CALL_ADDRESS: u8 = 0x00` — The I2C general-call address.
- `const GENERAL_CALL_RESET: [u8; 1] = [0x06]` — The general-call reset command: it reaches a wedged core that every addressed command NAKs.
- `const OTP_UNLOCK: [u8; 5] = [0xC5, 0x95, 0x00, 0x66, 0x9C]` — Unlock the OTP for reading (address 0x0066 and its CRC 0x9C, behind the command 0xC595).
- `const OTP_READ: [u8; 2] = [0xC7, 0xF7]` — Read the next OTP word (auto-incrementing): issued once per calibration constant.
- `const MEASURE: [u8; 2] = [0x48, 0xA3]` — Measure, pressure first, normal mode: 6.3 ms, 1.6 Pa RMS.
- `const RESET_TRIES: u32 = 4` — Attempts at the first touch, spanning one maximum-length conversion window (~95 ms with the waits).
- `const RESET_RETRY_MS: u32 = 30` — The wait after a failed first touch (and after a general call) before the next command.
- `const RECOVER_AFTER: u32 = 3` — Consecutive run-loop failures before the recovery escalates to the general call.
- `const MEASURE_MS: u32 = 12` — The conversion wait for normal mode (6.3 ms typical), with margin: collect no earlier.
- `const IDENTITY_MASK: u16 = 0x3F` — The product id's significant bits.
- `const IDENTITY: u16 = 0x08` — The product id's significant bits on an ICP-101xx.
- `const WORD_LENGTH: usize = 3` — The answer to the id and OTP reads: one word and its CRC (unchecked, as plan B: the id mask and the OTP's later use are the checks).
- `const OTP_WORDS: usize = 4` — How many OTP calibration constants the part carries.
- `const FRAME_LENGTH: usize = 9` — The measurement frame, three words and their CRCs: pressure bytes 0 and 1, then pressure byte 3 and a pad, then temperature bytes 6 and 7.
- `const GROUND_SAMPLES: u32 = 8` — Readings averaged at setup (and at a calibrate) to fix the ground-zero reference.
- `const DEFAULT_PERIOD_MS: u32 = 100` — The default read period, ms: the wait from a collect to the next issue, 120 or 130 ms a cycle on the 10 ms tick (see the module notes).
- `const GROUND_INSTRUCTION: &str = "stand the glider STILL on the pad -- this captures its ground reference"` — What the calibration column shows until a ground zero is held (plan B's calibration() text).

## `drivers::ina226`

_Tested by `src/ina226/tests.rs`._

The INA226 high-side current, voltage and power monitor (src/glider/drivers/ina226.py): the battery (or 5 V)
supply line, for consumption tracking and the servo probe's closed-loop verdict. [`Device::setup`] verifies the
die id, programs the conversion configuration, and writes the calibration computed from the shunt resistance and
the expected full-scale current (the only board-specific numbers); [`Ina226::read`] polls the bus voltage,
current and power.

The part measures the SHUNT VOLTAGE directly (2.5 uV/LSB), so the absolute accuracy comes from the CALIBRATION
register, not a precise resistor: Current_LSB = max_current / 2^15, CAL = 0.00512 / (Current_LSB x shunt). To
trust the watt-hours, calibrate `shunt_mohms` against a KNOWN current once and back out the effective value: a
2-wire ohmmeter cannot resolve a 0.01 ohm shunt. Gotcha from the bench: the module needs VBS jumpered to IN-, or
the voltage and power read 0.

INTEGER milli-units, exactly plan B's: the readings are mV, mA and mW, which are also the telemetry units, so no
float is involved anywhere (decision D18 asks for none here). From the datasheet:
 * bus voltage: 1.25 mV/LSB -> mV = raw x 5 // 4;
 * current: Current_LSB = max / 2^15 -> mA = raw x max_current_ma // 32768 (signed: a reversed shunt or a charging
   current reads negative);
 * power: Power_LSB = 25 x Current_LSB -> mW = raw x max_current_ma // 32768 x 25 (divided BEFORE the x 25, as
   plan B does it, so the digits match);
 * calibration = 0.00512 x 2^15 x 1e6 // (max_current_ma x shunt_mohms) = 167772160 // (mA x mOhm).

THREE SEPARATE READS, DELIBERATELY: the register pointer does NOT auto-increment. Probed on the board, a 6-byte
read from 0x02 returns the bus voltage and then 0xFFFF padding, not power and current. Each register needs its own
pointer write. Do not "optimise" this back into one read.

The re-arm restores CONFIGURATION, CALIBRATION, and with the hardware alert armed ALERT LIMIT and MASK/ENABLE: a
general call brings the part back with CALIBRATION = 0, which makes every current and power reading wrong rather
than absent, and with the alert disarmed. Plan B restored the first two only (avionics-port.md §6).

Not here (firmware glue): the ALERT GPIO and its interrupt counter (the `alerts` column), the log-once of new
over-current trips, the channel push, the probe's log lines and the inspect JSON.

### `pub struct Measurement`

One reading, in the integer milli-units the channels and the telemetry row carry.

### `pub struct Settings`

The INA226's settings, from its config block (the keys plan B's setup() reads, with its defaults).

- `fn from_config(config: &Value, alert_wired: bool) -> Settings` — Read a config block; `alert_wired` says whether the glue resolved an `alert_pin` to a GPIO. A missing key takes plan B's default.

### `pub struct Ina226`

One INA226 on a bus.

- `fn new(settings: &Settings) -> Ina226` — A driver for the part these settings describe; nothing is written until setup.
- `fn address(&self) -> u8` — The I2C address.
- `fn calibration(&self) -> Option<u16>` — The CALIBRATION value setup wrote, the one the re-arm restores; None before a successful setup.
- `fn alert_ma(&self) -> i64` — The over-current trip in mA while the hardware alert is armed, else 0 (plan B's inspect and log value).
- `fn convert(&self, bus_voltage: u16, current: i16, power: u16) -> Measurement` — Convert the three raw registers to milli-units (ina226._read's arithmetic): bus voltage (u16), current (s16) and power (u16).
- `fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Measurement, Fault<Bus::Error>>` — Read the live bus voltage, current and power (ina226._read): THREE separate 2-byte reads, bus voltage, current, power, in plan B's order (see the module notes for why never one).
- `fn identify<Bus: I2c>(&self, bus: &mut Bus) -> Result<(), Fault<Bus::Error>>` — Read the die id; [`Fault::Identity`] when it is not the INA226's.
- `fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Measurement, Fault<Bus::Error>>` — The on-demand self-test (ina226.probe): the die id reads back, then one live read.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String` — Classify a failed setup at the wire level (ina226.diagnose through i2cbus): read the die id's high byte, 0x22, and sort it with [`id_classify`].

### `pub fn calibration(max_current_ma: i64, shunt_mohms: i64) -> Option<u16>`

The CALIBRATION register value for a full-scale current and a shunt (plan B's `_CAL_NUM // (mA x mOhm)`).

None where plan B's setup failed: a zero product (its ZeroDivisionError) or a quotient outside the 16-bit
register (its `struct.pack('>H')` error) -- every negative product, whose floored quotient is negative. A
quotient of 0 (a product past the numerator) fits and is written, as plan B writes it; every current and power
then reads 0. The arithmetic is i128 because plan B's integers do not overflow: two i64 config values multiply
past i64, and the quotient is then a valid 0.

### `pub fn alert_limit(alert_ma: i64, shunt_mohms: i64) -> Option<u16>`

The ALERT LIMIT register value for an over-current trip: mA x mOhm -> shunt-voltage LSBs of 2.5 uV, that is
`alert_ma x shunt_mohms x 2 // 5` (floored, in i128 for the reason [`calibration`] gives). None when it does
not fit the register, a negative trip included (plan B then fell back to polling only).

### Constants

- `const ADDRESS: u8 = 0x40` — The default I2C address (A1 = A0 = GND).
- `const REGISTER_CONFIGURATION: u8 = 0x00` — Configuration register.
- `const REGISTER_BUS_VOLTAGE: u8 = 0x02` — Bus voltage register: u16, 1.25 mV/LSB.
- `const REGISTER_POWER: u8 = 0x03` — Power register: u16, Power_LSB = 25 x Current_LSB.
- `const REGISTER_CURRENT: u8 = 0x04` — Current register: s16, Current_LSB per bit.
- `const REGISTER_CALIBRATION: u8 = 0x05` — Calibration register.
- `const REGISTER_MASK_ENABLE: u8 = 0x06` — Mask/Enable register: which condition drives ALERT, plus the read-back flags.
- `const REGISTER_ALERT_LIMIT: u8 = 0x07` — Alert limit register: the trip, in the selected condition's LSBs.
- `const REGISTER_DIE_ID: u8 = 0xFF` — Die id register.
- `const DIE_ID: u16 = 0x2260` — The INA226's die id.
- `const CONFIGURATION: u16 = 0x4327` — The configuration plan B writes: continuous shunt and bus, 4-sample average, 1.1 ms conversions (~9 ms an update).
- `const MASK_ENABLE_SOL: u16 = 0x8000` — Mask/Enable SOL: ALERT on Shunt Over-Voltage, which is over-current independent of the rail voltage. Transient (the latch bit LEN stays clear), so the line drops again with the current.
- `const CALIBRATION_NUMERATOR: i128 = 167_772_160` — The calibration numerator for milli-unit inputs: 0.00512 x 2^15 x 1e6.
- `const POWER_LSB_RATIO: i128 = 25` — Power_LSB / Current_LSB.
- `const DEFAULT_SHUNT_MOHMS: i64 = 10` — The default shunt, milliohms.
- `const DEFAULT_MAX_CURRENT_MA: i64 = 5_000` — The default full-scale current, mA.
- `const DEFAULT_ALERT_MA: i64 = 3_000` — The default over-current trip, mA (with an ALERT pin wired).
- `const DEFAULT_PERIOD_MS: u32 = 100` — The default read period, ms.

## `drivers::lsm6dso32`

_Tested by `src/lsm6dso32/tests.rs`._

The LSM6DSO32 6-DoF IMU (src/glider/drivers/lsm6dso32.py): the PRIMARY raw `accel` and the primary gyro `rate`.
A +/-32 g accel range covers the 8-12 g boost without clipping and still resolves ~1 mg for the airspeed
integrator; the gyro runs at +/-2000 dps. Both at 104 Hz with block data update, so the 12 output bytes at
OUTX_L_G (gyro x..z, then accel x..z, contiguous, signed little-endian) never mix two conversions.

Wired on SPI (its own chip-select, 4-wire mode 3) for clean high-rate reads; I2C at 0x6A also works when the
config names bus 'i2c'. Multi-byte reads auto-increment through CTRL3_C.IF_INC, so the SPI command byte is the
register with the read flag and NO multi-byte address bit ([`OverSpi`]).

WRITE-THEN-VERIFY bring-up, up to [`IDENTITY_ROUNDS`] rounds of "write CTRL3_C, read WHO_AM_I". It used to be 5
bare reads, and the shape was wrong as well as the count. MEASURED on TMS-7E: with the part out of step, WHO_AM_I
returns 0x00 for 30 consecutive reads and stays at 0x00 -- reading does not recover it, however long you read.
Writing CTRL3_C does, immediately. The interface has to be PINNED, and a write issued before the part is
listening is simply lost, so the write is retried rather than issued once up front. How far that risk reaches
was measured too: interrupting the driver mid-traffic and rebooting (12 tries) and a bus retune at 1, 8 and
5 MHz (4 for 4) all recovered on the first read with the old one-shot strategy. The only reproducer found is a
peripheral CREATED AND DEINITIALISED under the part, which a pin test does and no flight path does. So this is
not a fix for a flight failure; it is the right shape for the operation, at microseconds a round.

NEW CONVERSIONS ONLY (avionics-port.md M6, M9 D1). The flight read is [`Lsm6dso32::read_fresh`]: ONE 16-byte
burst from STATUS_REG -- the status, the reserved 0x1F, OUT_TEMP, then the same 12 output bytes -- so the
data-available flags and the sample come out of one chip-select frame. The firmware pushes `accel` only on a fresh
accel half and `rate` only on a fresh gyro half, writes the row on either, and calls the part unhealthy after
[`STALE_LIMIT`] ticks in a row with neither (avionics-port.md §6's VL53 rule: a repeated value is never
republished as fresh). Plan B pushed every pass, but its pass was slower than the ODR; the 100 Hz tick is not, so
without the flags a part slower than the tick would publish an old sample as a new one. Never fresh, and never
stored: an impossible status (a bit the datasheet fixes at 0, as the 0xFF of an SPI bus nothing drives), so a part
that lost power, chip-select or MISO goes stale the same way; and a burst whose 12 output bytes are ALL zero (a
chip-select that never asserts reads 16 x 0x00). The zero guard is the whole frame, not a half: a well-biased gyro
at rest reads (0, 0, 0).

THE FLAG DECIDES; THE BYTES ONLY CORRECT IT ([`crate::freshness::Flagged`], D1). One frame keeps the window
between the flags and the output small, not empty: a conversion that lands after the status byte reads with its
flag clear but its bytes changed. That half is a raced conversion, pushed once and marked raced, where M6 lost it
(a 30 ms gap between rows, measured at 9.2 and 8.4 a minute on the taster and 7F). If that read leaves its flag
set, the next read finds the flag over the same bytes: the duplicate, skipped. A flagged half is otherwise never
called stale for identical bytes, since at rest a new gyro conversion often repeats the last one bit for bit. The
residual is a raced conversion whose half is bit-identical to the one before: still lost, now counted (the
[`Fresh`] `identical` marks), not assumed. The driver stores the last read's 12 bytes and a raced mark per half;
setup and the re-arm forget them, and the firmware calls [`Lsm6dso32::forget`] when it re-adds the bus handle or
resumes reads it skipped, so the first read after any of them is judged by its flag alone. A bare
[`Lsm6dso32::read`] (the probe, setup's clearing read) clears data-ready without storing anything, so a conversion
it took from under the flags reads as raced on the next burst, and is pushed: it never was.

THE PART'S OWN ODR. 104 Hz is nominal: the conversions run off the part's own oscillator, whose per-die offset
INTERNAL_FREQ_FINE reports ([`Lsm6dso32::frequency_fine`], [`odr_hz`], 0.15 % per LSB). A part slower than the
100 Hz tick has no new conversion behind a tick now and then (the taster measured about 1 % of rows with
`irq_runs` 0, so it runs at about 99 Hz); a faster one always has, and skips about 4 conversions a second
instead. The firmware logs the ODR once at bring-up, so each board says which case it is.

INT1 AND `irq_runs`. The control tick paces the reads: the part is read at the top of every tick, and the config's
`period_ms` is unused. INT1 data-ready is evidence, not a pacer: the firmware's GPIO interrupt only counts its
rising edges, and the tick swaps the count out just BEFORE the read -- plan B's order, whose run loop waited for
the edge, then read. With data-ready latched (COUNTER_BDR_REG1 `dataready_pulsed` 0, the reset default plan B
never wrote) INT1 stays high until the burst reads the output, so a read with a new conversion behind it sees 1;
0 means no new conversion, or a dead line; and a 0 followed by a 2 is a conversion landing in the microseconds
between the swap and the burst. The latch also means a late tick never piles up edges: more than 2 needs another
read of the output (a probe) to clear data-ready between two swaps. The firmware arms the interrupt AFTER setup,
on the control thread: a conversion that lands between setup's clearing read and the arming leaves INT1 high
with no edge to count, so the first tick's read is what clears it, and the first row's `irq_runs` is 0.

A dead INT line used to masquerade as a healthy sensor: plan B's fallback timeout became the sampling rate,
measured at exactly 2.0 Hz on the bench with the driver reporting healthy, while `rate` has a 20 ms freshness
window and no backup on a board without a BMI323 -- so the PID's D term quietly degraded to
derivative-on-error. On the tick a dead line no longer slows the sampling, but it still hides a broken wire, so
[`Lsm6dso32::wake`] keeps plan B's verdict as the operator's view: [`INTERRUPT_SILENT_LIMIT`] reads in a row
without an edge mean the line is dead, and one edge means it is back.

Units (decision D18): accel in f32 g and gyro in f32 degrees per second for the channels. The telemetry row
keeps plan B's integers: the gyro columns are the RAW centidegree-per-second integer `raw x 70 x 100 // 1000`
(never a formatted decimal; the host tools divide by 100 when they render) next to the accel in g and
`irq_runs`.

Not here (firmware glue): the GPIO interrupt and its edge count, the stale run and its log line, the channel
push, the telemetry stream, the probe's log lines, the bring-up's ODR line and the inspect JSON.

### `pub type RegisterWrite = (u8, u8)`

One register write of a bring-up table: (register, value).

### `pub trait Interface<Bus>`

How the part is reached: plan B's register window over whichever bus the config names (lsm6dso32._transport).

The two windows read and write the same registers; only the framing differs. The LSM6DSO32 is generic over it,
so one driver serves both buses.

- `fn write(&self, bus: &mut Bus, register: u8, value: u8) -> Result<(), Self::BusError>` — Write one register.
- `fn read(&self, bus: &mut Bus, register: u8, buffer: &mut [u8]) -> Result<(), Self::BusError>` — Read consecutive registers from `register` on into `buffer` (the part auto-increments through IF_INC).

### `pub struct OverSpi`

The SPI window (plan B's spibus device with `mb_bit=None`): the `SpiDevice` frames each transfer with the part's
own chip-select. A read sends `0x80 | register` then clocks the data in; a write sends the register then the
value. No multi-byte address bit: the part auto-increments from CTRL3_C.IF_INC, and setting one would address a
different register.

The bus owner discards the first transfers after the SPI peripheral appears (plan B measured 1 to 11 stale 0x00
reads on a fresh peripheral; avionics-port.md M5 "discard-N"), so this window has no resync of its own.

### `pub struct OverI2c`

The I2C window at an address (plan B's i2cbus device): a register write is one write of the register and the
value (writeto_mem); a register read writes the register, then reads after a repeated start
(readfrom_mem_into).

### `pub enum Transport`

Which bus the config names for the part (plan B's `_transport()` dispatch).

- `fn from_config(config: &Value) -> Transport` — The transport a config block names: `bus` (absent means 'spi'; see [`Transport::I2c`] for every other value) and, for I2C, `addr` (default [`ADDRESS`]; a value that is not a 7-bit address, 0x00..=0x7F, takes the default too, where plan B would have failed its first transfer).

### `pub struct Sample`

One output burst, as the part reports it: raw signed counts.

- `fn decode(frame: &[u8; FRAME_LENGTH]) -> Sample` — Decode the 12-byte burst: gyro x, y, z then accel x, y, z, each signed 16-bit little-endian (plan B's `struct.unpack('<hhhhhh')`). Gyro comes FIRST out of this part; the BMI323 puts accel first.
- `fn accel_g(&self) -> [f32; 3]` — The accel in g: the `accel` channel and the telemetry's ax, ay, az.
- `fn rate_degrees_per_second(&self) -> [f32; 3]` — The gyro in degrees per second: the `rate` channel, in the marks frame (x forward, y left, z up, through the board config's `axes` where a part is mounted otherwise). The (roll, pitch, yaw) rate is (gx, -gy, -gz): crates/control `attitude::rate_from_marks` maps it, for the attitude filter and the PID D term alike.
- `fn rate_centidegrees_per_second(&self) -> [i32; 3]` — The gyro in centidegrees per second, plan B's integer to the digit (`raw x 70 x 100 // 1000`, floored): the telemetry's gx, gy, gz columns.

### `pub struct Lsm6dso32<F>`

One LSM6DSO32, reached through the window `F` ([`OverSpi`] or [`OverI2c`]).

- `fn new(interface: F, interrupt_wired: bool) -> Lsm6dso32<F>` — A driver on this window; `interrupt_wired` when the config's `int_pin` resolves to a GPIO.
- `fn forget(&mut self)` — Forget the stored output bytes and the raced marks, so the next [`Lsm6dso32::read_fresh`] is judged by its flags alone. Setup and the re-arm forget by themselves; the firmware calls it when it re-adds the bus handle (a bustune rung) and when it resumes reads it skipped (`imu mute`).
- `fn interrupt_wired(&self) -> bool` — Whether an INT1 wire is declared (inspect's `interrupt`).
- `fn interrupt_silent(&self) -> bool` — Whether the INT line is called dead (inspect's `interrupt_silent`): the tick still samples, so `rate` stays fresh; what is lost is the evidence that the INT1 wire works, so the operator sees it.
- `fn wake(&mut self, interrupt_runs: u32) -> bool` — Account one read: `interrupt_runs` INT1 edges arrived since the previous swap (see the module notes: 0 = no new conversion, or a dead line; 1 = healthy; 2 after a 0 = a conversion that landed between the swap and the burst; more = another read of the output (a probe) cleared data-ready between two swaps: with data-ready latched, a late tick never piles up edges); the telemetry row carries the same count as its `irq_runs` column.
- `fn identity<Bus>(&self, bus: &mut Bus) -> Result<u8, Fault<F::BusError>> where F: Interface<Bus>,` — Read WHO_AM_I once (the probe's first step).
- `fn read<Bus>(&self, bus: &mut Bus) -> Result<Sample, Fault<F::BusError>> where F: Interface<Bus>,` — Read one sample: the 12-byte burst at OUTX_L_G, which also clears data-ready.
- `fn read_fresh<Bus>(&mut self, bus: &mut Bus) -> Result<(Fresh, Sample), Fault<F::BusError>> where F: Interface<Bus>,` — Read one sample and which of its halves are new: ONE 16-byte burst from STATUS_REG (the status, the reserved 0x1F, OUT_TEMP, then the 12 output bytes). Each half is judged by its flag, XLDA (bit 0) for the accel and GDA (bit 1) for the gyro, corrected by its bytes against the last burst's (the module notes; TDA, bit 2, is ignored). Never fresh, and the stored bytes left as they were: a status with a [`STATUS_RESERVED`] bit set (impossible from a live part, so a dead SPI part goes stale and turns unhealthy after [`STALE_LIMIT`] ticks instead of publishing 0xFF bytes forever), or all 12 output bytes zero. The reserved byte and the temperature are discarded, and the output bytes decode exactly as [`Lsm6dso32::read`]'s. Over SPI that is one chip-select frame, `0x80 | 0x1E` then 16 bytes clocked in; over I2C, the register then a 16-byte read after a repeated start. Like `read`, it clears data-ready.
- `fn frequency_fine<Bus>(&self, bus: &mut Bus) -> Result<i8, Fault<F::BusError>> where F: Interface<Bus>,` — Read INTERNAL_FREQ_FINE once (the bring-up's ODR line, through [`odr_hz`]): the part's own ODR offset, a two's-complement byte.
- `fn probe<Bus>(&self, bus: &mut Bus) -> Result<Sample, Fault<F::BusError>> where F: Interface<Bus>,` — The on-demand self-test (lsm6dso32.probe): WHO_AM_I reads back the LSM6DSO32's identity, then one sample reads. The firmware logs each step and folds the magnitude of the accel into the line.
- `fn diagnose<Bus>(&self, bus: &mut Bus) -> String where F: Interface<Bus>,` — Classify a failed setup at the wire level (lsm6dso32.diagnose through the bus window): read WHO_AM_I once and sort it with [`id_classify`] (0x00 is a chip-select not asserting, 0xFF a floating MISO).

### `pub fn odr_hz(fine: i8) -> f32`

The part's own output data rate at the configured 104 Hz setting, in Hz, from its INTERNAL_FREQ_FINE byte: the
datasheet's (6667 + 0.0015 x fine x 6667) / ODR_Coeff with ODR_Coeff 64, that is 104.17 Hz x (1 + 0.15 % per
LSB). 0 is the nominal 104.17 Hz; -33 is about 99.0 Hz, and the extremes are about 84.2 Hz (-128) and 124.0 Hz
(127).

### Constants

- `const ADDRESS: u8 = 0x6A` — The default I2C address (SDO low; 0x6B when high), for a config that names bus 'i2c'.
- `const WHO_AM_I: u8 = 0x0F` — WHO_AM_I: the identity register.
- `const CTRL1_XL: u8 = 0x10` — CTRL1_XL: accel output data rate and full scale.
- `const CTRL2_G: u8 = 0x11` — CTRL2_G: gyro output data rate and full scale.
- `const CTRL3_C: u8 = 0x12` — CTRL3_C: BDU (block data update), IF_INC (register auto-increment) and SIM (3- or 4-wire SPI).
- `const INT1_CTRL: u8 = 0x0D` — INT1_CTRL: what drives the INT1 pin (accel data-ready is bit 0).
- `const OUTX_L_G: u8 = 0x22` — OUTX_L_G: the first of the 12 output bytes, gyro x..z then accel x..z, signed little-endian and contiguous, so one burst fetches both.
- `const STATUS_REG: u8 = 0x1E` — STATUS_REG: the data-available flags, XLDA (bit 0, accel) and GDA (bit 1, gyro), each set by a new conversion and cleared when its output is read. The registers after it run on to the output bytes: 0x1F (reserved), OUT_TEMP at 0x20..0x21, then OUTX_L_G, so one burst from here reads the flags and the sample together.
- `const INTERNAL_FREQ_FINE: u8 = 0x63` — INTERNAL_FREQ_FINE: the part's own ODR offset from nominal, two's complement, 0.15 % per LSB ([`odr_hz`]).
- `const IDENTITY: u8 = 0x6C` — What WHO_AM_I reads on an LSM6DSO32.
- `const IDENTITY_ROUNDS: u32 = 64` — How many write-then-verify rounds setup spends before deciding the part is absent (see the module notes): each is two short transfers, once, at setup, so an absent part is reported a few hundred microseconds later than a one-shot read would.
- `const PIN_INTERFACE: RegisterWrite = (CTRL3_C, 0x44)` — The write that PINS the interface, repeated before each identity read: CTRL3_C = 0x44, that is BDU = 1 (a burst never mixes two conversions), IF_INC = 1 (multi-byte reads auto-increment) and SIM = 0 (4-wire SPI).
- `const CONFIGURATION: [RegisterWrite; 2] = [(CTRL1_XL, 0x44), (CTRL2_G, 0x4C)]` — The configuration written once the identity is verified, in plan B's order: * CTRL1_XL = 0x44: accel 104 Hz (ODR 0100), FS_XL = 01, which is +/-32 g on this part; * CTRL2_G = 0x4C: gyro 104 Hz (ODR 0100), FS_G = 11, +/-2000 dps.
- `const ROUTE_INTERRUPT: RegisterWrite = (INT1_CTRL, 0x01)` — Accel data-ready routed to INT1 (INT1_CTRL bit 0), written only when an INT1 wire is declared.
- `const FRAME_LENGTH: usize = 12` — The output burst: gyro x, y, z then accel x, y, z, two bytes each.
- `const STALE_LIMIT: u32 = 3` — Consecutive reads with no new conversion (neither half fresh) before the firmware calls the part unhealthy: one such tick is a part slower than the tick, three in a row are a part that stopped converting.
- `const ACCEL_G_PER_COUNT: f32 = 0.000976` — Accel sensitivity at +/-32 g: 0.976 mg per count (datasheet), in g.
- `const GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT: i32 = 70` — Gyro sensitivity at +/-2000 dps: 70 millidegrees per second per count (datasheet).
- `const INTERRUPT_SILENT_LIMIT: u32 = 3` — Consecutive reads with no INT1 edge behind them before the line is called dead rather than jittery.

## `drivers::mg90s`

_Tested by its own `mod tests`._

The MG90S metal-gear positional fin servo (src/glider/drivers/mg90s.py). Electrically IDENTICAL to the SG90 --
the same 50 Hz frame, ~500..2500 us pulse to angle, open-loop with no feedback -- so it is the SG90's
[`crate::sg90::Servo`] with this module's [`MG90S`] type: the angle-to-pulse map, the per-fin trim, the clamp, the
update and the probe verdict are all shared. What differs is MECHANICAL: metal gears hold more torque, so the
aerodynamic load cannot back-drive the horn (an SG90's plastic train lets wind slide the fin off its commanded
angle). Preferred on the yaw fin, the rudder that sees the most steady wind pressure.

TRAVEL is per instance through the component's min_deg/max_deg, whose SPAN must equal the servo's mechanical
travel so one command degree is one degree of rotation. The 180-degree variant uses the default 0..180 and drops
in exactly where an SG90 was. The 360-degree-travel MG90S wants its 360 degrees CENTRED on the mixer neutral (90):
`min_deg: -90, max_deg: 270` -> command 90 = servo mid = 1500 us = fin centre (so boot, probe and failsafe all sit
centred), and +/-45 command = +/-45 degrees of fin. A fin only uses neutral +/- the mixer limit, so the spare range
is unused and the ~0.18-degree pulse step is still plenty. MIXED fleets are fine: each fin's `driver` is
independent (an mg90s yaw with sg90 elevons), and the mixer commands angles by name, blind to the servo type.

### Constants

- `const MG90S: ServoType = ServoType { name: "mg90s", slew_ms_per_60: 100, settle_margin_ms: 60, engine_min_mw: 600, en...` — The MG90S: ~0.1 s per 60 degrees (metal gear, a touch quicker than the SG90's 0.15), and a probe window shifted up for the larger current (600..5000 mW). Both are datasheet-approximate TYPE defaults; tune them per built rig through the component's engine_*_mw config.

## `drivers::mock`

_Tested by its own `mod tests`._

The scripted bus the host tests drive the drivers with: an embedded-hal `I2c` ([`MockBus`]), a `DelayNs`
([`MockDelay`]) and an `OutputPin` ([`MockPin`]) that all check their calls against ONE script, in order.

One script for the three is the point: a driver's correctness is often an ORDER -- the SDP810 must be stopped,
left 3 ms, and only then started; the INA226's three registers are three separate reads; the VL53's XSHUT goes
low, waits, goes high, waits, then polls -- and only a shared timeline can say the wait came between the two
writes and not after them. A call that does not match the next scripted step panics at once, naming the step,
both sides and the bytes in hex; [`MockBus::done`] then insists every scripted step happened.

Plan B has no such mock (its tests run on the board against real parts); the golden builders capture the same
kind of transcript from plan B's own code, and the crate's golden tests replay it through this bus.

### `pub enum Step`

One step of a script: what the driver must do next, and what the part answers.

### `pub struct MockBus`

The scripted I2C bus. Clone it to share the script; [`MockBus::delay`] and [`MockBus::pin`] hand out the delay
and the pin on the same script.

- `fn new() -> MockBus` — An empty script.
- `fn expect_write(&self, address: u8, bytes: &[u8]) -> &MockBus` — Script a plain write of `bytes` to `address`.
- `fn expect_read(&self, address: u8, bytes: &[u8]) -> &MockBus` — Script a plain read from `address`, answered with `bytes`.
- `fn expect_write_read(&self, address: u8, write: &[u8], read: &[u8]) -> &MockBus` — Script a register read: `write` (the register address) then a read answered with `read`.
- `fn expect_delay_ms(&self, milliseconds: u32) -> &MockBus` — Script a wait of `milliseconds`.
- `fn expect_pin(&self, high: bool) -> &MockBus` — Script a control pin driven `high` (true) or low.
- `fn expect(&self, step: Step) -> &MockBus` — Script any step, as a golden transcript lists them.
- `fn then_fail(&self) -> &MockBus` — Make the last scripted step fail: the bus NAKs it ([`MockError::NAK`], nothing is read back), the pin refuses it.
- `fn then_fail_other(&self) -> &MockBus` — Make the last scripted step fail as the port, not the wire, fails it: the bus answers [`MockError::OTHER`] (nothing is read back), the pin refuses it.
- `fn delay(&self) -> MockDelay` — The delay on this bus's script.
- `fn pin(&self) -> MockPin` — A control pin on this bus's script.
- `fn performed(&self) -> usize` — How many scripted steps have happened so far.
- `fn done(&self)` — Insist that every scripted step happened; panics listing the ones that did not.

### `pub struct MockError`

The error the scripted bus answers a failing step with, as the embedded-hal kind the driver reads:
[`MockError::NAK`] for a step scripted with [`MockBus::then_fail`], [`MockError::OTHER`] for one scripted with
[`MockBus::then_fail_other`].

### `pub struct MockDelay`

The scripted delay: each wait must be the next scripted step, to the nanosecond.

### `pub struct MockPin`

The scripted control pin (an XSHUT line): each level driven must be the next scripted step.

### `pub struct MockPinError`

The error the scripted pin answers a failing step with.

### `pub struct NoPin`

The pin a part without a control line is given: there is nothing to drive, so nothing can fail.

### Constants

- `const MockError: MockError = MockError::NAK` — TRANSITIONAL: [`MockError::NAK`] under the unit struct's old value name, kept only so the four `Fault::Bus(MockError)` lines of `tests/probe.rs` compile until they read `MockError::NAK`; then this alias and its lint allow go. New code names the kind.

## `drivers::sdp810`

_Tested by `src/sdp810/tests.rs`._

The SDP810-500Pa differential-pressure sensor (Sensirion SDP8xx, thermal flow-through): the pitot/static
AIRSPEED channel (src/glider/drivers/sdp810.py). Command-based, not register-mapped: [`Device::setup`] clears any
prior continuous mode, starts continuous measurement (differential pressure, temperature-compensated,
average-till-read) and validates one CRC-checked frame; [`Sdp810::read`] reads the 9-byte frame each period,
scales the tared dynamic pressure and derives the airspeed once, for the channel and the telemetry row alike.

Bench-verified: 0x25 on i2c:0 (SDA 7 / SCL 8), scale factor 60 (Pa = raw / 60), zero ~0.2 Pa. Tube polarity
(blow-verified 2026-08-14, and anchored to a PIN because left/right got it wrong once): P+ = the barb OPPOSITE
the '1'/SCL mark -> pitot (total); P- = the '1'/SCL-side barb -> interior static. The interior-static PRESSURE
bias is tared out by the zero offset (a pad tare, `update {"zero": true}`); the position-span error folds into
`air_density`, the single q -> v knob (a GNSS-vs-q calm pass trims it).

What plan B learnt the hard way, and this port keeps:
 * STOP BEFORE START. A prior unclean reboot can leave the part mid-continuous, where a re-issued start is
   rejected; the datasheet requires a stop first. Setup and the restart both send stop, wait, start.
 * ALL THREE CRCs ([`frame_ok`]), because the scale word is latched at setup and divides every reading after it:
   one corrupted scale would mis-scale the airspeed for the whole flight. The ICP-10111 on the same bus was
   measured dropping a bit in 0.50 % of its frames.
 * THE SCALE LATCH: setup reads the part's own scale factor (60 for the -500Pa part, 240 for the -125Pa) and every
   reading divides by it; a part answering 0 keeps the -500Pa default.
 * THE RESTART LADDER: the part holds its MODE in volatile state, so anything that resets it (the ICP's
   general-call reset, by design) drops it to idle and every read after fails. Five consecutive read failures
   re-issue stop-then-start, once per run of failures. A CRC reject neither counts toward it nor rearms it.
 * THE TARE LADDER: the config's `zero_offset_pa` is a guess; a tare stored by an earlier session (NVS
   `sdp810_zero`, an i32 of Pa x 100, the key plan B shares) wins over it at boot; an operator tare captures the
   live reading AND is persisted; a direct offset (the warm start restoring its crumb, mid-air) is applied but
   never written to flash. A tare before the first frame is refused out loud, and 0 Pa is a real tare.

Units: f32 Pa and m/s for the channels (decision D18). The integer units survive only at the telemetry and NVS
boundary, as plan B defines them: dynamic pressure in Pa x 100 (`raw x 100 // scale - zero`, computed from the
raw count in integers so the column is plan B's to the digit), airspeed in cm/s (truncated), temperature in
degrees C x 100 (`raw // 2`). The zero offset itself is kept in that NVS unit, so a tare reads back after a reboot
as exactly what was captured.

Where the port departs from plan B, on purpose:
 * AN AIR DENSITY THAT IS NOT ABOVE ZERO IS REFUSED (an update) or replaced by [`AIR_DENSITY`] (the config).
   Plan B took any number: at 0 every positive q raised ZeroDivisionError in its read path, so no sample went
   out; below 0 it took a complex root. In f32 those become an infinite or NaN airspeed handed straight to the
   governor's estimator, which is worse than no sample, so neither is allowed in.
 * A Pa FIGURE MEANT AS A WHOLE HUNDREDTH IS THAT HUNDREDTH (`HUNDREDTH_SNAP`). Plan B truncated `Pa x 100`,
   binary noise included: 0.29 Pa became 28 (CPython) and on the single-precision board 1.18 became 117. The
   warm start carries the tare as f32 Pa and hands it back here, so truncating would lose a hundredth on about
   half of all tares (333, the plan's own anchor, came back 332); snapping makes that round trip exact.

Not here (firmware glue): the NVS read and write themselves, the channel push, the probe's freshness wait, the
log lines and the inspect JSON.

### `pub struct Settings`

The SDP810's settings, from its config block (the keys plan B's setup() reads, with its defaults).

- `fn from_config(config: &Value) -> Settings` — Read a config block; a missing or non-numeric key takes plan B's default, and so does an air density that cannot divide (0 or below: see the module notes).

### `pub struct Sample`

One good frame, converted.

### `pub struct Failure<BusError>`

Why a read produced no sample, and what the restart ladder did about it.

### `pub enum Refusal`

Why an operator `update` was refused; nothing has changed when it is.

### `pub struct Update`

What an accepted `update` did.

### `pub struct Sdp810`

One SDP810 on a bus.

- `fn new(settings: &Settings) -> Sdp810` — A driver for the part these settings describe, untared, at the default scale until setup latches the part's.
- `fn restore_zero(&mut self, stored: i32)` — Restore the tare a previous session stored (NVS [`NVS_ZERO_KEY`], Pa x 100): it WINS over the config.
- `fn latch_scale(&mut self, word: u16)` — Latch the scale word of setup's good frame: every reading after it divides by it. A part answering 0 keeps the -500Pa default rather than divide by zero.
- `fn address(&self) -> u8` — The I2C address.
- `fn scale(&self) -> u16` — The scale factor in use (Pa = raw / scale).
- `fn zero_offset_pa(&self) -> f32` — The zero offset, Pa.
- `fn zero_offset(&self) -> i32` — The zero offset in the NVS unit, Pa x 100.
- `fn tared(&self) -> bool` — Whether a tare was captured, restored or set (0 Pa included).
- `fn air_density(&self) -> f32` — The q -> v span, kg/m^3.
- `fn temperature(&self) -> f32` — The sensor temperature of the last frame, degrees C (0 before the first).
- `fn pressure(&mut self, raw: i16) -> f32` — The tared dynamic pressure, Pa, of a raw signed count (sdp810._pressure); the count becomes the tare source.
- `fn pressure_hundredths(&self, raw: i16) -> i32` — The telemetry's dynamic pressure, Pa x 100, of a raw count: `raw x 100 // scale - zero`, plan B's column to the digit (floored, so -600 at scale 60 is -1000 and 200 is 333).
- `fn airspeed(&self, pressure: f32) -> f32` — The indicated airspeed, m/s, of a dynamic pressure in Pa: q = rho v^2 / 2 inverted (sdp810._airspeed).
- `fn read<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<Sample, Failure<Bus::Error>>` — Read and convert one frame: the run loop's pass (sdp810.run).
- `fn stop<Bus: I2c>(&mut self, bus: &mut Bus)` — Best-effort stop of continuous mode (the setup-failure and teardown path); a dead bus is swallowed.
- `fn tare(&mut self) -> Option<i32>` — Capture the last reading as the zero offset: the pad tare (update `{"zero": true}`, calibrate).
- `fn calibrate(&mut self) -> Result<i32, &'static str>` — Calibrate: capture the still-air zero (sdp810.calibrate). Returns the tare to persist, or plan B's [`CALIBRATE_NO_READING`] before the first frame.
- `fn set_zero_offset(&mut self, pascal: f32)` — Set the zero offset directly, in Pa: the warm start restoring its crumb (what [`Sdp810::zero_offset_pa`] gave it, returned to the hundredth), or the operator. It is a tare (0.0 Pa included) but is NOT persisted: mid-air, a flash write is not wanted.
- `fn set_air_density(&mut self, density: f32) -> Result<(), Refusal>` — Set the q -> v span, kg/m^3 (the GNSS-vs-q calm-pass trim); a density that is not finite and above 0 is refused and the span is left as it was.
- `fn calibration(&self, dynamic_pressure: Option<f32>) -> Option<String>` — The still-air tare instruction while one is owed, None once a tare was captured, restored or set (sdp810.calibration); `dynamic_pressure` is the channel's latest, Pa, for the '(now ...)' part.
- `fn update(&mut self, properties: &Value) -> Result<Update, Refusal>` — Apply an operator property change (sdp810.update), in plan B's order: `zero` (a truthy flag: tare and persist), `zero_offset_pa` (a direct offset, not persisted), `air_density`.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String` — Classify a failed setup at the wire level (sdp810.diagnose): read one frame.

### `pub fn frame_ok(frame: &[u8]) -> bool`

Whether a 9-byte frame is whole and every word's CRC is right: differential pressure, temperature AND scale
(sdp810._frame_ok).

It once checked only the pressure word, as "the flight-relevant field". The scale word is more flight-relevant
than that: setup latches it and every reading afterwards divides by it, so one corrupted scale mis-scales the
airspeed for the rest of the flight. A 1-in-200 chance of that is not worth two skipped checks over six bytes.

### `pub fn signed16(high: u8, low: u8) -> i16`

A big-endian signed 16-bit word from its two bytes: the part's two's-complement fields (sdp810._signed16).

### `pub fn airspeed_cms(airspeed: f32) -> i32`

The telemetry's `airspeed_cms` of an airspeed in m/s: truncated toward zero, as plan B's `int(airspeed x 100)`.

### Constants

- `const ADDRESS: u8 = 0x25` — The fixed I2C address (SDP800/810-500Pa and -125Pa; the -x1 variants answer at 0x26).
- `const COMMAND_START: [u8; 2] = [0x36, 0x15]` — Start continuous measurement: differential pressure, temperature-compensated, average-till-read.
- `const COMMAND_STOP: [u8; 2] = [0x3F, 0xF9]` — Stop continuous measurement, back to idle: required before re-starting after an unclean reboot.
- `const FRAME_LENGTH: usize = 9` — The frame: differential pressure [0, 1] + CRC, temperature [3, 4] + CRC, scale factor [6, 7] + CRC.
- `const FIRST_RESULT_MS: u32 = 20` — The wait after a start before the first frame is read: the first continuous result lands ~8 ms after the start, with margin.
- `const STOP_SETTLE_MS: u32 = 3` — The wait after a stop before the part accepts the next command: it settles in ~0.5 ms, with margin.
- `const RESTART_AFTER: u32 = 5` — Consecutive read failures before the restart re-issues stop-then-start.
- `const START_TRIES: u32 = 2` — Start attempts at setup: a stop and a restart clear a part left mid-continuous.
- `const DEFAULT_SCALE: u16 = 60` — The scale factor of the -500Pa part (Pa = raw / scale), kept when the part answers 0; the -125Pa reports 240.
- `const AIR_DENSITY: f32 = 1.18` — The default air density, kg/m^3: ~25 degrees C at sea level (Florida, not ISA's 15 degrees C). The calm-pass trim overrides it per airframe.
- `const DEFAULT_PERIOD_MS: u32 = 20` — The default read period, ms: ~50 Hz (the part's tau63 under 3 ms allows a fast poll).
- `const NVS_ZERO_KEY: &str = "sdp810_zero"` — The NVS key (namespace `coludo`) of the stored tare: an i32 of Pa x 100, shared with plan B.
- `const CALIBRATE_NO_READING: &str = "no reading yet"` — What [`Sdp810::calibrate`] answers before the first frame (plan B's calibrate() text).

## `drivers::separation`

_Tested by `src/separation/tests.rs`._

The stage-separation switch (src/glider/drivers/separation.py): a copper ring on the booster that bridges two copper
strips on the glider, routing 3V3 to a pin while nested (HIGH) and opening on separation (LOW, held there by the
pin's internal pull-down). A latch over the generic [`crate::debounce`]: the interrupt and the tick stamp the
contact's episodes there, this module says what a concluded one means. A separation in BOOSTING drives the
documented BOOSTING -> GLIDING transition (the booster ejects the glider at apogee) once the thrust gate lets it;
anywhere else it is recorded and moves nothing, so a ground test in SETTING keeps go/no-go correct. A re-nest is
recorded and never reverses a stage.

THE DECISION ([`Separation::conclude`]): the level read when the episode concludes, a quiet after its last edge,
decides it. LOW with the latch nested is `separated`, HIGH with the latch separated is `nested`, and the latch's own
level is a `blip`: the contact moved and came back. Blips are recorded too, so a flight shows every movement of the
contact: a boost blip is a blip row and never a separation, but for one instant. The level is read for the tick
just before the take, so a blip whose quiet runs out on that very tick, with the next blip's 1 -> 0 inside the
interrupt's entry latency before the read (well under a µs a tick), concludes separated (crate::debounce, THE
TICK). Under thrust the thrust gate holds that separation pending and the next blip's re-nest clears it; once the
gate is open, in BOOSTING that GLIDING stands. A board powered up unassembled reads LOW at bring-up, so its latch
starts separated, and assembling it concludes `nested` (with the interrupt, or by the tick's own read when no edge
came): the latch ends at 1.

THE THRUST GATE ([`ThrustGate`], the operator's; it lives in crates/sequencer, module `thrust`, where the stage
machine keeps one per accelerometer, and is re-exported here): in BOOSTING a separation counts only once the
booster has stopped pushing the glider, so vibration under thrust can never deploy. The gate reads the glider's own
axial specific force, a_x on the body frame's +x (the nose; +y left, +z up), from a fresh `accel` every tick.
Thrust is SEEN when a_x holds above the sequencer's `launch_g` for its `launch_ms` of net time (the launch
detector's leaky dwell, sequencer::commons::dwell_step), and has ENDED when a_x then holds below 0 g for 20 ms of
net time: the drag deceleration the glider feels the moment nothing pushes it, by burnout or by an early
aerodynamic separation of a destabilized stack, which so opens the gate itself. Both start afresh at each BOOSTING
entry. A missing a_x drains a dwell like a sample that misses. A wrong axis or sign, no IMU, or a bench hold never
sees thrust, so the gate stays shut and only the backstop decides: a mis-mounted IMU can never open the gate under
thrust.

THE PENDING SEPARATION ([`Separation::tick`]): a separation concluded in BOOSTING while the gate is shut moves
nothing and writes no row: it is pending. It counts (GLIDING, the row `separated;gliding`, dated by its episode's
first edge) only on a tick that reads the pin LOW with no episode open, the contact held LOW since its last edge:
the first such tick with the gate open, or by THE BACKSTOP, 100 ms after its last edge, far beyond the contact's
wobble and any blip, so a dead or confused IMU can never hide a real separation. A LOW read inside an open episode
(a bounce of the contact closing again) counts nothing: the episode's conclusion decides. One that ends LOW again
(the contact touched and let go) is part of the pending separation: its last edge becomes the separation's, the
backstop runs from there, and an open gate counts it on the conclusion's own tick. A re-nest clears it: the open and
its re-nest are one `blip` row from the first edge to the last, and the counted latch never moved
([`Separation::separated`]). When BOOSTING ends otherwise (the sequencer's apogee or burnout, the flight timeout, an
operator hold), a pending separation counts on the first such tick, as a separation anywhere outside BOOSTING does:
the row in the new stage, nothing advanced. Outside BOOSTING the gate changes nothing, and with the gate held open
the tick is [`Separation::conclude`] exactly (plan B's parity, tests/golden_separation.rs).

THE PHYSICS, as the operator measures it: at separation the glider slides out over the strips' 2 cm at about 3 m/s,
so contact may be lost and restored for about 7 ms, under 10 at worst. THE SEPARATION MOMENT ([`Event::moment_us`])
is the episode's first edge, the first 1 -> 0; THE DURATION ([`Event::duration_us`]) runs from it to the episode's
last edge, so 0 for a clean single edge. With the interrupt both are its stamps, exact to the µs; on a polled pin,
or a change the tick's own read saw, they are the reads that saw the changes, multiples of the tick. For a re-nest
or a blip both describe its own episode alike.

THE QUIET is `debounce_ms`, 10 ms by default ([`DEFAULT_DEBOUNCE_MS`]: the operator's "10 ms or less"; plan B's was
20): how long the pin must see no edge before the episode concludes. It must outlast the longest contact gap inside
the wobble: a gap that holds the quiet up to a tick ends the episode there, so one separation splits into rows (a
blip and then the separation, which its own first edge then dates; the stage never reverses either way). It is also
the line between a blip and a separation: a contact that stays open past the quiet, until a tick reads it, is a
separation. Every shipped document still writes plan B's 20 -- config_default, which a board also boots when no
stored config passes validate, and the 20261003 launch records -- so a board waits 10 ms once its config says 10
or drops the key.

THE LATENCY, at the default 10 ms quiet on the 10 ms control tick (the one place this crate states it): an episode
concludes on the first tick at least 10 ms after its last edge, 10-20 ms after it. Polled, the last change is seen
by the read after it, up to a tick late, and the episode concludes a quiet after that read: 10 ms later at the
default. separation.csv shows it: a row's `uptime` is the conclusion's tick, its `moment` the first edge, its
`duration` the wobble. Under the thrust gate in BOOSTING: the nominal separation is the ejection charge, about 4 s
after thrust end, long after the gate opened, so it counts at its conclusion as above. An early separation counts
when the gate opens, 20-30 ms after a_x went negative (the 20 ms dwell and the tick's phase), or at its conclusion
if that is later. Under a shut gate an open counts by the backstop, 100-110 ms after its last edge; an open of up to
100 ms is a blip, and a quiet of 100 ms or more meets the backstop at the conclusion itself.

Fed plan B's own reads (golden separation.json, each flagged edge recorded at its read), the port records plan B's
separations and re-nests on the same reads in 34 of its 42 scenarios, and mostly a blip where plan B's settle
ended on the latch's own level; tests/golden_separation.rs lists every difference. None is a GLIDING plan B did not
make, but under plan B's own 0 ms debounce, which never settled a held open.

`debounce_ms` is read as every driver reads its block ([`Settings::from_config`], the default for a missing or
malformed value) and converted to the quiet's µs once, by [`Separation::new`]; how long a quiet is sane is
board_config's validate to refuse (`LONGEST_DEBOUNCE_MS`, 1000 ms), so every config a board runs has passed it.

The transition is the control thread's own commit to the flight word (sequencer::FlightWriter::separate), not the
stage machine's: separation is the PRIMARY BOOSTING -> GLIDING trigger and separation.csv (event, stage, moment,
duration; durable) its authoritative record; the sequencer's apogee and burnout timeout are only the fallbacks,
recorded in sequencer.csv. A post-flight tool reading the BOOSTING -> GLIDING reason consults separation.csv first.

Not here (firmware glue): the GPIO, its pull-down and its both-edge interrupt recording into a static
[`crate::debounce::StampPair`], the tick's read, the fresh a_x the gate is fed, the commit, the telemetry push (the
moment rebuilt on the uptime column's clock, the duration in µs) and the log line.

### `pub enum Kind`

What a concluded episode was, by the level read at its conclusion.

### `pub struct Event`

What separation.csv records: a concluded episode, or under the thrust gate a pending separation counted or cleared.

- `fn name(&self) -> &'static str` — The event name: `separated`, `nested` or `blip`.
- `fn row(&self) -> [&'static str; 2]` — The separation.csv row's text cells, plan B's two columns: (`event`, `stage`). The `moment` and `duration` columns after them are the caller's: [`Event::moment_us`] on the uptime column's clock, which the caller knows, and [`Event::duration_us`].

### `pub struct Settings`

The separation switch's settings, from its config block.

- `fn from_config(config: &Value) -> Settings` — Read a config block: `debounce_ms`, [`DEFAULT_DEBOUNCE_MS`] when it is missing or not a whole number of ms that fits a u32 (negative, fractional, text). It bounds nothing: board_config's validate refuses an absurd one.

### `pub struct Separation`

The separation switch: the latch, its quiet, and a separation pending under the thrust gate.

- `fn new(settings: &Settings, high: bool) -> Separation` — A switch whose pin read `high` at setup: LOW (the contact open) starts it separated. Its quiet is `settings.debounce_ms` in µs; past the µs clock's range (4 294 967 ms, which the validator's bound keeps out of every config a board runs) it saturates, a quiet that never comes.
- `fn separated(&self) -> bool` — The counted latch, true = separated (separation.separated): the latch, but for a separation still pending under the thrust gate, which is no separation yet. The AUTHORITATIVE reading for the warm-start gate, from this driver's own pin rather than a second reader of the same GPIO.
- `fn pending(&self) -> bool` — Whether a separation is pending under the shut thrust gate (`control`'s `pending`).
- `fn latched(&self) -> bool` — The pin level the latch stands for, true = HIGH = nested: the `latched` the tick's own read is observed against (crate::debounce). A pending separation is in it: the latch follows the pin, the gate decides what counts.
- `fn quiet_us(&self) -> u32` — The quiet, µs: the tick takes an episode once its latest edge is this old.
- `fn conclude(&mut self, episode: Episode, high: bool, stage: Stage) -> Event` — Conclude `episode` by the pin level `high` the tick read for it, in `stage` (separation._apply): separated, nested or a blip (see the module notes, THE DECISION). A separation in BOOSTING advances to GLIDING; nothing else moves a stage, so a re-nest never reverses one. Ungated: [`Separation::tick`] adds the thrust gate.
- `fn tick(&mut self, pair: &StampPair, high: bool, now_us: u32, stage: Stage, gate_open: bool) -> Option<Event>` — One tick under the thrust gate at `now_us` in `stage`: take `pair`'s episode once it is quiet, conclude it by the level `high` the tick read for it ([`Separation::conclude`]), and hold a separation in BOOSTING pending while the gate is shut (`gate_open`, [`ThrustGate::open`]) until a LOW read with no episode open counts it or a re-nest clears it (see the module notes, THE PENDING SEPARATION). The caller has offered the same read to the pair first (observed, or polled), against [`Separation::latched`].

### Constants

- `const DEFAULT_DEBOUNCE_MS: u32 = 10` — The default quiet, ms: how long the pin must see no edge before an episode concludes (`debounce_ms`), for a block without the key -- every shipped document still writes plan B's 20. The operator's "10 ms or less", outlasting the contact's wobble of about 7 ms (see the module notes, THE QUIET).

## `drivers::sg90`

_Tested by `src/sg90/tests.rs`._

The SG90 micro fin servo (src/glider/drivers/sg90.py), and the servo logic the MG90S shares ([`crate::mg90s`]):
one instance per fin (yaw, left and right elevon), each on a 50 Hz PWM pin. The command unit is INTEGER DEGREES,
linearly mapped to a pulse width (min_us..max_us over min_deg..max_deg, integer math) and CLAMPED to the range,
so a bad command can never drive the horn past the linkage.

OPEN-LOOP -- NO POSITION FEEDBACK. A 3-wire SG90 only RECEIVES a PWM command; there is no wire back, so the board
cannot read where the horn is. Everything reported ([`Servo::angle`], [`Servo::pulse_us`]) is the LAST COMMANDED
value, what was asked for, not a measurement: a stalled or jammed surface still reads its target. The one
closed loop available is the INA226: [`Servo::probe_verdict`] judges a probe sweep by the power it drew.

Each fin has a `trim` (whole degrees), its MECHANICAL zero offset: a horn on a ~18-degree spline plus linkage slop
is never installed exactly, so each engine is zeroed individually. The trim is added to every command in the
degree domain before the pulse map, so boot, failsafe and control all land on the fin's true centre; the pulse is
then bounded to [min_us, max_us] so a large trim can never over-drive the horn.

Python's arithmetic, reproduced (avionics-port.md §5.1): `round()` rounds a half to EVEN (sg90.py:251: 90.5 ->
90, 91.5 -> 92), and the pulse map's `//` floors toward minus infinity, below min_deg included.

The flight loop drives a servo through the mixer: [`Servo`] is a `control::mixer::Fin`, so `Mixer::actuate` sends
each fin its whole-degree angle and the compare-and-set decides whether anything is written.

Not ported (avionics-port.md §6): the command coalescing (built, off by default, and no measured benefit in five
flown policies), and plan B's duty_u16 (the firmware's LEDC has its own resolution and takes the pulse). Not here
(firmware glue): the PWM itself, the shared slew gate (`fins.concurrency`), the telemetry push, the probe's sweep
timing and its INA reads, and the inspect JSON.

### `pub struct ServoType`

The defaults that belong to the servo TYPE, not the instance: a component's config still overrides the probe
window per fin.

### `pub struct Settings`

A servo's settings, from its component block (the keys plan B's setup() reads, with its and the type's defaults).

- `fn from_config(config: &Value, servo_type: &ServoType) -> Settings` — Read a component block for a servo of `servo_type`; a missing or non-integer key takes the default.

### `pub struct Command`

One command written to the PWM.

- `fn row(&self, done: bool) -> [i32; 3]` — The `<name>.csv` row: `angle`, `pulse_us`, `done` (0 when a command is issued, 1 when a move has, by its estimate, completed).

### `pub struct Move`

A move under way: the command issued and how long its travel is estimated to take.

### `pub enum Refusal`

Why an operator `update` was refused; nothing has changed when it is.

### `pub enum ProbeVerdict`

What a probe sweep showed, judged by the power it drew (sg90.probe's closing verdict).

- `fn failure(&self) -> Option<String>` — The probe's failure text (plan B's, word for word), None when the verdict passes.

### `pub struct Servo`

One fin servo: its range, its trim, and the last command (the only position there is).

- `fn new(servo_type: &ServoType, settings: &Settings) -> Servo` — A servo of `servo_type` with these settings, commanded to the setup angle (the neutral by default): plan B's setup() drives the PWM there before anything else may move it. The returned servo's [`Servo::command`] is that first command.
- `fn servo_type(&self) -> &ServoType` — The servo type.
- `fn angle(&self) -> i32` — The last commanded angle, whole degrees (COMMANDED, never measured).
- `fn pulse_us(&self) -> i32` — The last pulse driven, us.
- `fn command(&self) -> Command` — The last command written.
- `fn trim(&self) -> i32` — The mechanical zero offset, whole degrees.
- `fn neutral(&self) -> i32` — The range's centre, degrees.
- `fn engine_min_mw(&self) -> i32` — The probe's least power rise, mW.
- `fn engine_max_mw(&self) -> i32` — The probe's HIGH-draw flag, mW.
- `fn clamp(&self, degrees: f64) -> i32` — A command in degrees, rounded half to even and clamped to [min_deg, max_deg] (sg90._clamp).
- `fn pulse(&self, angle: i32) -> i32` — The pulse, us, for an already-clamped angle (sg90._write's map): `min_us + (angle + trim - min_deg) x (max_us - min_us) // span`, or min_us for a zero span, bounded to [min_us, max_us].
- `fn write(&mut self, angle: i32) -> Command` — Write an already-clamped angle (sg90._write): map it, drive it, remember it. Returns the command, whose row the caller pushes with `done` 0.
- `fn apply(&mut self, degrees: f64) -> Command` — Clamp then write (sg90._apply): the operator and move() path.
- `fn set_angle(&mut self, degrees: f64) -> Option<Command>` — The 100 Hz flight-loop command, compare-and-set (sg90.set_angle): clamp, then write ONLY when the angle changed, so a held fin costs nothing. None when it did not change.
- `fn travel_ms(&self, degrees: i32, settle: bool) -> i32` — How long a move of `degrees` occupies the horn, ms (sg90.travel_ms): the type's rated slew, plus the settle margin when `settle`. An ESTIMATE -- there is no feedback -- which is why an SG90 and an MG90S answer the same call differently.
- `fn begin_move(&mut self, degrees: f64) -> Move` — Start a gated move (sg90.move): the travel is estimated from the angle held BEFORE the move, then the clamped target is written. The caller holds the slew gate for the travel and then pushes the same command with `done` 1: the completion record is this snapshot, never a later angle.
- `fn update(&mut self, properties: &Value) -> Result<(Vec<&'static str>, Option<Command>), Refusal>` — Apply an operator override, immediate and ungated (sg90.update): `trim` (whole degrees, rounded half to even) then `angle` (clamped). A trim alone re-drives the current angle, so CC can zero each engine live and watch the fin settle on its true centre. Returns the keys applied, in that order, and the command written, if any.
- `fn probe_targets(&self, limit: Option<i32>) -> [(&'static str, i32); 3]` — The probe sweep's stops, in order: (`min`, ...), (`max`, ...), (`neutral`, neutral). With no `limit`, plan B's full range, min_deg and max_deg. With `limit` (whole degrees, the board config's `fins.mixer.limit_deg`), neutral -/+ limit inside min_deg..max_deg: the port's own, at the operator's word (2026-10-11) -- a sweep to the servo's own ends drives the fins past the linkage's deflection range, so the wings had to be opened for every probe; the configured range is the one the fins fly.
- `fn probe_verdict(&self, watching: bool, sampled: u32, peak_rise_mw: i32) -> ProbeVerdict` — Judge a probe sweep (sg90.probe's close): `watching` says a fresh INA226 power baseline was taken, `sampled` how many fresh readings the sweep got, and `peak_rise_mw` the largest reading minus the baseline (the probe starts it at 0, so a fall never counts).

### `pub fn round_half_even(degrees: f64) -> i32`

Python's `round()` of a whole-degree command: a half to even (2.5 -> 2, -1.5 -> -2).

f64 on purpose: an operator's angle or trim arrives as a JSON number (an f64) and plan B rounds that f64; in f32
a value a hair above a half could land ON the tie and round the other way. The mixer's whole degrees pass through
unchanged.

### Constants

- `const PERIOD_US: i32 = 20_000` — The servo frame, us: 50 Hz.
- `const DEFAULT_MIN_US: i32 = 500` — The default pulse at min_deg, us (an SG90 spans ~500..2500 us).
- `const DEFAULT_MAX_US: i32 = 2_500` — The default pulse at max_deg, us.
- `const DEFAULT_MIN_DEGREES: i32 = 0` — The default command range's low end, degrees.
- `const DEFAULT_MAX_DEGREES: i32 = 180` — The default command range's high end, degrees.
- `const PROBE_SETTLE_MS: u32 = 300` — The probe's settle before it takes the resting power baseline, ms (only with a fresh INA226).
- `const PROBE_SAMPLES: u32 = 24` — The INA226 samples the probe takes at each sweep target, one every [`PROBE_SAMPLE_MS`]: the travel window.
- `const PROBE_SAMPLE_MS: u32 = 20` — The probe's sampling period at each sweep target, ms.
- `const SG90: ServoType = ServoType { name: "sg90", slew_ms_per_60: 150, settle_margin_ms: 60, engine_min_mw: 500, engi...` — The SG90: ~0.15 s per 60 degrees (plastic gear), probe window 500..3500 mW.

## `drivers::vl53l1x`

_Tested by `src/vl53l1x/tests.rs`._

The VL53L1X time-of-flight laser ranger (src/glider/drivers/vl53l1x.py), and the VL53 family machinery it shares
with the VL53L4CX ([`crate::vl53l4cx`]): the above-ground-level (AGL) channel for the last metres of the glide,
where the barometer is useless. The family uses 16-BIT register addresses.

ONE SOCKET, TWO SILICONS: the VL53L1X (model id 0xEACC, the VL53L1X Ultra-Lite-Driver init) and the
VL53L4CD/L4CX (0xEBAA, the VL53L4CD ULD init) both answer at 0x29, and neither configuration produces ranges on
the other part. An I2C scan cannot tell them apart, so each [`Silicon`] checks its own model id strictly and
refuses the other ([`Fault::Identity`]): a board declares both, and the fitted one wins, exactly as an absent
device is skipped. If that check ever loosened, the wrong init block would be written to a real part: it acks,
it configures, and it silently never ranges.

RANGE IS THE REASON TO CARE WHICH IS FITTED: the L1X is declared 2-4 m against the L4CX's 4-6 m, and the landing
trigger (`land_agl_m`) defaults to 5.0 m -- unreachable by an L1X, so the transition would silently fall back to
barometric elevation, the thing the laser exists to replace. A board fitted with this part wants `land_agl_m`
around 3.0 (doc/hardware.md).

The setup ([`Device::setup`]): an optional XSHUT reset, the firmware boot wait, the model id, the default
configuration block, one VHV calibration ranging cycle (start, wait, clear, stop, then the VHV writes), the
L4CX's timing budget, and continuous ranging. NO TIMING BUDGET IS SET on the L1X: the L4CX's RANGE_CONFIG math is
the VL53L4CD ULD's and does not apply to this silicon, whose ULD uses a table keyed on distance mode instead; the
configuration block's own timing stands until that table is checked against the part.

DATA-READY BEFORE EVERY PUBLISH ([`Vl53::range`]): plan B read the status and the distance whenever its loop woke,
so in poll mode a range already published was read and published again as fresh. Here a range is read only when
GPIO__TIO_HV_STATUS says a new one is waiting (avionics-port.md §6, changed on purpose).

The re-arm runs the whole setup again (XSHUT pulse included when wired): plan B had no re-arm for either part,
and a re-initialised ranger is the safe state after a general call whether or not the part honoured it.

Not here (firmware glue): the GPIO1 data-ready interrupt and its `irq_runs` count, the channel push, the probe's
log lines and the inspect JSON.

### `pub struct Silicon`

What tells one silicon of the family from the other: its id, its init block, its start value and whether it
takes the timing budget.

### `pub enum Ranging`

One range read ([`Vl53::range`]).

- `fn agl(self) -> Option<f32>` — The above-ground level in metres a valid range publishes (plan B's `distance_mm / 1000.0`), else None.

### `pub struct Settings`

A VL53 ranger's settings, from its config block (the keys plan B's setup() reads, with its defaults).

- `fn from_config(config: &Value) -> Settings` — Read a config block; a missing or non-integer key takes plan B's default.

### `pub struct Vl53<Shutdown>`

One VL53-family ranger on a bus: which silicon it expects, and its XSHUT line if one is wired.

- `fn new(silicon: &'static Silicon, settings: &Settings, xshut: Option<Shutdown>) -> Vl53<Shutdown>` — A driver for `silicon` with these settings and, when wired, its XSHUT line; nothing is touched until setup.
- `fn silicon(&self) -> &'static Silicon` — The silicon this driver expects.
- `fn address(&self) -> u8` — The I2C address.
- `fn has_xshut(&self) -> bool` — Whether an XSHUT line is wired.
- `fn data_ready<Bus: I2c>(&self, bus: &mut Bus) -> Result<bool, Fault<Bus::Error>>` — Whether a measurement is waiting: GPIO__TIO_HV_STATUS bit 0 equals the polarity.
- `fn range<Bus: I2c>(&self, bus: &mut Bus) -> Result<Ranging, Fault<Bus::Error>>` — Read the latest measurement, if one is waiting, and release the interrupt (vl53l1x._range_locked, behind the data-ready check plan B lacked).
- `fn identify<Bus: I2c>(&self, bus: &mut Bus) -> Result<(), Fault<Bus::Error>>` — Read the model id; [`Fault::Identity`] when it is not this silicon's (the check behind [`Vl53::probe`], and setup's id gate).
- `fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<u16, Fault<Bus::Error>>` — The on-demand self-test (vl53l1x.probe, vl53l4cx.probe): the model id reads back as this silicon's, all 16 bits in one register read; answers that id, for the owner's 'probe: model id ok' line.
- `fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String` — Classify a failed setup at the wire level (vl53l1x.diagnose through i2cbus): read the model id's high byte (0xEA for the L1X, 0xEB for the L4CX) and sort it with [`id_classify`]; the other silicon in the socket reads as a 'wrong device'.

### `pub fn range_status(raw: u8) -> u8`

The ULD status of a raw RESULT__RANGE_STATUS byte: its low 5 bits through [`STATUS_RTN`], 255 past the table.

### `pub fn polarity(gpio_hv_mux_ctrl: u8) -> u8`

The data-ready polarity GPIO_HV_MUX__CTRL selects: bit 4 set means active-low data-ready (0), clear active-high
(1).

### Constants

- `const ADDRESS: u8 = 0x29` — The default I2C address of both silicons.
- `const FIRMWARE_SYSTEM_STATUS: u16 = 0x00E5` — FIRMWARE__SYSTEM_STATUS: bit 0 set once the part's firmware has booted.
- `const IDENTIFICATION_MODEL_ID: u16 = 0x010F` — IDENTIFICATION__MODEL_ID: two bytes, the silicon's model id.
- `const DEFAULT_CONFIGURATION_START: u16 = 0x002D` — The first register of the default configuration block (0x2D..=0x87).
- `const DEFAULT_CONFIGURATION_LENGTH: usize = 91` — The default configuration block's length: registers 0x2D..=0x87.
- `const GPIO_HV_MUX_CTRL: u16 = 0x0030` — GPIO_HV_MUX__CTRL: bit 4 selects the data-ready interrupt polarity.
- `const GPIO_TIO_HV_STATUS: u16 = 0x0031` — GPIO__TIO_HV_STATUS: bit 0 equals the interrupt polarity while a measurement is waiting.
- `const SYSTEM_INTERRUPT_CLEAR: u16 = 0x0086` — SYSTEM__INTERRUPT_CLEAR: writing 1 releases the interrupt, which arms the next measurement.
- `const SYSTEM_MODE_START: u16 = 0x0087` — SYSTEM__MODE_START: the start value starts continuous ranging, 0x00 stops it.
- `const RESULT_RANGE_STATUS: u16 = 0x0089` — RESULT__RANGE_STATUS: the low 5 bits are the device range status, mapped through [`STATUS_RTN`].
- `const RESULT_DISTANCE: u16 = 0x0096` — RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: two bytes, the distance in mm.
- `const VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND: u16 = 0x0008` — VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, written 0x09 after the calibration cycle (both ULDs).
- `const VHV_CONFIG_INIT: u16 = 0x000B` — VHV_CONFIG__INIT, written 0x00 after the calibration cycle: start the VHV from the last temperature (both ULDs).
- `const BOOT_TIMEOUT_MS: u32 = 100` — How many 1 ms polls the firmware has to boot.
- `const RANGE_TIMEOUT_MS: u32 = 200` — How many 1 ms polls the VHV calibration ranging cycle has: it can take tens of ms.
- `const XSHUT_MS: u32 = 2` — How long XSHUT is held low, and how long the part is left after it goes high (and with no XSHUT at all).
- `const DEFAULT_PERIOD_MS: u32 = 50` — The default sampling period, ms: the interrupt-silent fallback cadence (plan B once read a `period_us` key no config supplies, and every laser fell back to 500 ms against a 100 ms freshness window).
- `const STATUS_RTN: [u8; 24] = [` — The raw range status (RESULT__RANGE_STATUS & 0x1F) mapped to the ULD status; 0 is a valid measurement, 255 a reserved code. The two ULDs share it.
- `const DEFAULT_CONFIGURATION: [u8; DEFAULT_CONFIGURATION_LENGTH] = [` — The VL53L1X default configuration, registers 0x2D..=0x87: ST's canonical ULD array.
- `const VL53L1X: Silicon = Silicon { name: "vl53l1x", model_id: 0xEACC, default_configuration: &DEFAULT_CONFIGURATION, ...` — The VL53L1X: model id 0xEACC (its high byte 0xEA tells it from the L4CX's 0xEB), the ULD block above, start 0x40, no timing budget.
- `const DEFAULT_TIMING_BUDGET_MS: i64 = 50` — The default timing budget, ms (the L4CX only).

## `drivers::vl53l4cx`

_Tested by `src/vl53l4cx/tests.rs`._

The VL53L4CX time-of-flight laser ranger, Adafruit 5425 (src/glider/drivers/vl53l4cx.py): the newer 0xEBAA
silicon shared by the VL53L4CD and L4CX, which takes the VL53L4CD Ultra-Lite-Driver init -- the older VL53L1X
(0xEACC) configuration does NOT produce ranges on it. Single-target distance; the L4CX multi-target extras are
unused.

It shares everything else with the VL53L1X -- the register map, the boot wait, the VHV calibration cycle, the
range read with its data-ready check, the re-arm -- so the driver is [`crate::vl53l1x::Vl53`] with this module's
[`VL53L4CX`] silicon. What is this part's own: its model id, its default configuration block, its start value
(0x21), and the timing budget ([`timing_budget`]) setup programs after the VHV writes.

THE TIMING BUDGET is the ULD's integer math, kept integer: its 2^30 term overflows a single-precision float (the
reason plan B kept it integer on MicroPython), and here i128 keeps any configured budget exact as Python's
integers did. The macro period comes from the part's measured oscillator frequency; the budget minus a fixed
2.5 ms overhead (continuous mode, inter-measurement 0) is spread over the macro periods of RANGE_CONFIG A (x16)
and B (x12), each written as an 8-bit mantissa and an exponent byte.

### `pub fn macro_period(oscillator: u16) -> Option<i128>`

The macro period, in us scaled by 2^12 (the oscillator register is MHz in 4.12 fixed point, and the budget is
scaled to match): `(2304 x 2^30 // oscillator) >> 6`, multiplied before dividing as plan B does it. None for an
oscillator of 0, where plan B writes nothing.

### `pub fn timing_budget(oscillator: u16, budget_ms: i64) -> Option<[(u16, u16); 2]>`

The RANGE_CONFIG A and B words for a budget (vl53l4cx._set_timing_budget's arithmetic), in write order, or None
for an oscillator of 0.

Python's floor division and arithmetic shifts are reproduced for any integer budget, a negative or sub-overhead
one included (its mantissa is then the low byte of a negative count, as Python's `& 0xFF` takes it).

### Constants

- `const DEFAULT_CONFIGURATION: [u8; DEFAULT_CONFIGURATION_LENGTH] = [` — The VL53L4CD default configuration, registers 0x2D..=0x87: the ST/Adafruit ULD block for the 0xEBAA silicon; ranging is started separately (SYSTEM__MODE_START is 0x00 here).
- `const VL53L4CX: Silicon = Silicon { name: "vl53l4cx", model_id: 0xEBAA, default_configuration: &DEFAULT_CONFIGURATION...` — The VL53L4CD/L4CX: model id 0xEBAA, the ULD block above, start 0x21, and the timing budget.
- `const OSC_MEASURED_FAST_OSC_FREQUENCY: u16 = 0x0006` — OSC_MEASURED__FAST_OSC__FREQUENCY: two bytes, the part's measured oscillator frequency; 0 leaves the budget alone.
- `const REGISTER_0X0024: u16 = 0x0024` — The register written [0x05, 0x00] before the timing budget: the VL53L4CD ULD's raw `WrWord(0x0024, 0x500)`, which it names no further.
- `const REGISTER_0X0024_VALUE: [u8; 2] = [0x05, 0x00]` — The word written to [`REGISTER_0X0024`].
- `const RANGE_CONFIG_TIMEOUT_MACROP_A: u16 = 0x005E` — RANGE_CONFIG__TIMEOUT_MACROP_A: the A-phase integration time, as a mantissa and an exponent byte.
- `const RANGE_CONFIG_TIMEOUT_MACROP_B: u16 = 0x0061` — RANGE_CONFIG__TIMEOUT_MACROP_B: the B-phase integration time, as a mantissa and an exponent byte.

# `flight_recorder` — `src/avionics/recorder`

_Uses: `beacon`, `recorder_wire`. External: anyhow, base64, esp-idf-svc, log._

## `flight_recorder`

The remote flight recorder (M20, doc/specs/avionics-port.md M19/M20) on an ESP32-C6 SuperMini: the P4's
uart_saver sends its recorder's wire lines, `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`, and this board files them
on its own SD card, so the data survives the P4 and the P4's card. It replaces the Luckfox's recorderd.

- The link: UART1 RX on GPIO17 (the board's RX pin) at 921600 baud, 8N1; TX + GND only from the P4.
- The card: SPI2, SCK GPIO19, MOSI GPIO20, MISO GPIO14, CS GPIO18, FAT at `/sd`.
- The lines are split as recorderd split them (crates/recorder_wire `split`), and filed where the P4's ring_saver
  files them on its own card (spec §3.10), so the same tools read both cards and the files match byte for byte: a
  stream line to `<session>/recordings/<routing>`, a session.csv line to the session's `session.csv` and the
  card's `sessions.csv`, a log line to the session's `recorder.log`, each line's session named by its routing (the
  P4's tee, crates/recorder saver `tee`). The P4's rules for a file's end hold too: a header line printed again
  after a reboot stays out of a file that holds one, and a torn last line gets its `\n` first. Each line is filed
  as it came, the payload after its routing, once: a line the P4 sends again after a reboot (no newer than its
  file's last) stays out. Its wrapper is checked (recorder_wire `verify`) and counted; a damaged line goes raw to
  the session's `damaged.log`, so the data files hold what the P4's hold.
- Lines gather in memory and go to the card once a second: per file, open for append, write, close (FatFs syncs
  at close), so a power cut costs at most the last second.
- Every 5 s one status line on the USB console: bytes and lines in, good and damaged, sessions, card errors. The
  console (crate::console, tools/recorder_console.py) lists, measures, removes and pulls the card's files.
- A card that does not answer is no stop: the link is read and counted all the same (lines then go unfiled,
  counted), and the mount is tried again every 10 s. The first try traces the card's init on the console, each SPI
  command and the card's answer.
- A mounted card is read back before anything is written: its name, size and SPI clock, the volume's free space
  and the root's entries. A card that cannot be read is not written to (the mount counts as failed).
- The remote beacon (crate::remote): the glider's beacon.csv report sets an access point `<BOARD>_beacon` and its
  page, the glider's own; the last report on the card is replayed after a restart.

## `flight_recorder::console`

The flight recorder's USB console: its card from the host (tools/recorder_console.py is the host's end of it).
One command a line, one reply line or a run of them ending in a `<command> ::` line, `err <command>: <why>` on a
refusal. The console has no driver, so its reads never wait; the thread looks again every [`POLL`], below the link's
loop (crate `LINK_PRIORITY`): a walk over a session's files must never hold the UART reader.

**Every reply starts with an empty line.** With no driver, output nobody reads is dropped, except the last 64 bytes
the USB FIFO still holds; they leave in front of the next write, so a reply's first line came glued to a status
line's head from a minute before (`I (137624) ... 55231 lindf 15327166464 15624077312`) and the host never saw it.
The empty line ends such a fragment, and the host reads it as a line of its own.

- `ls [dir]`: a directory's entries, one `ls <name>;<size>` each (`<name>/` and no size for a directory), then
  `ls :: <count> entries`.
- `df`: `df <free bytes> <total bytes>` of the card's volume.
- `rm <path>`: a file, or a directory with all it holds; `rm :: <path>`. The card's root and the session being
  filed now are refused (its lines would go on landing in files the receiver no longer knows).
- `get <path>`: a whole file, `get <size>`, then its bytes in base64 lines of [`GET_CHUNK`] bytes, then
  `get :: <crc32>` (zlib's CRC-32 in hex, as Python's `binascii.crc32`): the console's text is not byte-exact (it
  turns `\n` into `\r\n`), base64 is.
- `files [dir]`: every file under the card (or `<dir>`), one `file <path>;<size>;<crc32>` each, then
  `files :: <count> files, <bytes> bytes`: a file being filed is read up to where it ends at that moment.
- `read <path> <offset> <length>`: at most 4 KB of a file, one line `read <offset> <length> <hex>`.

### `pub fn spawn() -> anyhow::Result<()>`

Starts the console thread.

Errors: the thread would not spawn.

### Constants

- `const GET_CHUNK: usize = 3072` — The bytes of a file a `get` line carries (4 KB of base64).
- `static FILING: Mutex<String> = Mutex::new(String::new())` — The session the filer files now (crate `Filer`): `rm` refuses it.
- `static FORGET: AtomicBool = AtomicBool::new(false)` — Set by `rm`: the filer forgets what it knew of the card's directories and files (crate `Filer`), since the files it had looked at may be gone.

## `flight_recorder::remote`

The remote beacon (M20, doc/specs/avionics-port.md M19/M20; crates/beacon): the glider writes a report to
beacon.csv every few seconds, an ordinary stream this recorder files, and the beacon replays its last row. The row
sets the radio (the access point's name, empty for off; its password, empty for an open one; its channel, its TX
cap, its on/off cycle) and the page's figures, the same page the glider serves at DONE. Every report also goes to
the card's root beacon.csv, and after a restart its last row is read back ([`last_report`]), so a recorder that
loses power and comes back beacons again.

The access point is a captive portal at 192.168.4.1 (crates/beacon `esp`); the page notes how old the glider's
report is (the figures' ages grow with it), or that it came back from the card with its age unknown.

### `pub struct RemoteBeacon`

The remote beacon: the last report, and the access point it asks for.

- `fn new(modem: Modem<'static>, event_loop: EspSystemEventLoop, nvs: Option<EspDefaultNvsPartition>) -> Self` — Off, until a report turns it on; the Wi-Fi driver is made from `modem`, `event_loop` and `nvs` then.
- `fn report(&mut self, report: Report, received: Option<Instant>)` — The glider's `report`, received now (`received` Some) or read back from the card (None).
- `fn step(&mut self)` — The access point as the last report's radio asks: off for an empty name; on, or silent, by its cycle, staying on while a phone is connected (spec M20: a searcher who found it keeps it); started again only when what goes on the air changed (name, password, channel, TX cap), never for the same one again: a new cycle applies as it is. Called from the link's loop, often.
- `fn state(&self) -> String` — What the beacon does now, for the status line: `off`, or its name and `on` or `silent`, and where its report came from: `live` from the link, `card` read back after a restart.

### `pub fn last_report(root: &str) -> Option<Report>`

The glider's last report on the card under `root`: the last line of the root's beacon.csv, where the recorder
keeps each report's wire line (crate `Filer::line`); None without one.

# `gnss` — `src/avionics/crates/gnss`

_Uses: `control`, `sequencer`. External: none._

_Integration tests: `tests/golden.rs`._

## `gnss`

_Tested by its own `mod tests`._

The shared GNSS receiver (src/glider gnss.py): NMEA over a dedicated UART, talker-agnostic (GP, GN, BD, GL).
 * RMC gives the position, the ground speed and the course;
 * GGA gives the altitude above mean sea level, the elevation above the GNSS ground zero (the first altitude: a
   barometer backup) and the signal quality behind the fix (quality, satellites used, HDOP);
 * GSV (satellites in view and their C/N0), GSA (fix mode, satellites used) and the ATGM's antenna text are the
   sky diagnostics, folded into one row per burst. They tell a weak antenna from a dead receiver: the 2026-10-03
   flights never fixed, and with these sentences switched off their logs could not say why. They run on the pad
   only (before BOOSTING and again from DONE): in flight a burst delayed the epoch's RMC by up to ~0.6 s on the
   2026-10-04 bench, so the 200 ms position channel went stale once per burst.

This crate is the receiver's logic, with no UART and no clock: the firmware's gnss thread hands in each line
read and the time ([`Gnss::line`]), runs the 1 s tick ([`Gnss::tick`]), and publishes what comes back: an
[`Update`] (the channel values, the `<name>.csv` and `<name>_gga.csv` rows, the events) and a [`Tick`] (the
events, the sky switch frame to write, the `<name>_sky.csv` row). The module-specific sentence selection and
rate are the drivers' command bytes ([`drivers::atgm336h`], [`drivers::neo6mv2`], [`drivers::neom8n`]).

The NEO-M8N delivers binary instead (M16, [`ubx`]): the thread hands in each UBX frame ([`Gnss::message`]), and a
NAV-PVT fills the same [`Update`] an RMC and a GGA fill (the position, the speed and course, the altitude and
elevation, both rows), a NAV-SAT the sky row. The outage events watch PVT as they watch RMC and GGA.

Positions are [`Position`] integers in 1e-7 degree, parsed straight from the ddmm.mmmm digits
([`degrees_e7`]): plan B's board is single precision, where an f32 degree at latitude 48 has a 0.42 m ulp. The
other values are f32 (decision D18) and the telemetry integers stay integers (hdop_cd, the sky row).

Deliberate differences from plan B (marked in the golden fixture or covered by the unit tests):
 * hdop_cd (the GGA row's HDOP in hundredths) is read from the decimal text. Plan B wrote `int(hdop * 100)` of
   the float, which truncates a binary fraction: on CPython 0.29 gives 28 and 1.15 gives 114 (about 6 % of the
   two-decimal values lose a unit), and the board's f32 loses a unit in other cases;
 * a ddmm.mmmm field must be digits, with at least one degree and two minute digits before the dot, in i32 units.
   Plan B slices it around the dot into `int()` and `float()`, so '4807' (no dot) reads 17.45 degrees and a
   sign or an exponent goes through; here the line is dropped, as plan B drops one whose field raises;
 * a sentence that is not ASCII fails its checksum. Plan B decoded it as UTF-8 and checksummed by character
   index over the encoded bytes, which only lines up for ASCII;
 * a field is read as MicroPython's `int()`/`float()` read it (surrounding ASCII whitespace, a sign, `inf` and
   `nan`), but an underscore between digits, which Python accepts, is malformed here;
 * the outage sweep reports RMC before GGA (CPython's dict order; a board's MicroPython dict has its own).

Not ported: the UART, `probe()` and `diagnose()` (the firmware's I/O), and inspect's databoard values (the
channels are the firmware's).

### `pub enum Antenna`

The ATGM's antenna supervisor (`$GPTXT,01,01,01,ANTENNA OPEN`) as the sky row's code. OPEN means the feed draws
no current: a PASSIVE antenna reads OPEN too, and on the 2026-10-04 bench the passive 12x12 patch the airframes
flew cost the ATGM ~8-10 dB against the active ones (OK). The NEO-6M reports no antenna status over NMEA, so its
rows carry 0.

- `fn name(self) -> &'static str` — The code as a word, for inspect and the events (plan B `_ANTENNA_NAMES`).

### `pub enum Sentence`

The sentences watched for an outage. RMC and GGA are tracked apart because they fail apart: a receiver holding
almanac but no fix keeps emitting GGA (quality 0) while RMC goes void, and a selection mistake can silence one.
The NEO-M8N's UBX-NAV-PVT carries both in one message, watched as PVT. A sentence never heard is never lost, so
each module's outages name its own.

- `fn name(self) -> &'static str` — The sentence's type as NMEA spells it (PVT: UBX's message name).

### `pub struct SkyRow`

The sky as one `<name>_sky.csv` row, summed over the systems: all integers, like hdop_cd.

- `fn columns(&self) -> [i32; 8]` — The row's eight columns in the stream's order: in_view, used, mode, cn0_1..cn0_4, antenna.

### `pub struct FixRow`

One `<name>.csv` row: the fix an RMC pushed.

### `pub struct GgaRow`

One `<name>_gga.csv` row: GGA's altitude and the signal quality behind the fix. GGA has a stream of its own so
each sentence keeps its real cadence and an outage in one shows as a gap in its stream alone; these are the
numbers that say whether a fix loss was the sky, the antenna or the receiver.

### `pub enum Event`

A durable diagnostic the receiver raises (`<name>_events.csv` and the log). Display gives plan B's text.

### `pub enum SkyFrame`

Which of a driver's two sky switch frames to write.

### `pub struct SkySwitch`

A driver's sky switch: the bytes its setup made, written whole at each edge of the flight window.

- `fn frame(&self, frame: SkyFrame) -> &[u8]` — The frame to write for `frame`.

### `pub struct Update`

What one line produced, for the firmware to publish in field order: the channels (a position, then the speed
and course; or the altitude, then the elevation), the rows, the events.

### `pub struct Tick`

What one tick produced (plan B `_sweeping`'s pass): the events in order (outages, the sky switch, the first sky
view), the frame to write, and the sky row when a burst has just ended.

### `pub struct Malformed`

A field plan B's `int()`/`float()`/`degrees()` would raise on: the rest of the line is dropped, as plan B's run
loop drops a line that raises (what it already applied stays applied).

### `pub struct Gnss`

The receiver: plan B's Gnss task state, without the UART.

- `fn new() -> Gnss` — A receiver as its setup leaves it: nothing heard, the diagnostics on.
- `fn line(&mut self, raw: &[u8], now_us: u64) -> Update` — One line as the UART gave it (plan B's run loop body): count it, decode it, strip it, parse it, then sweep for outages (on every line, so a sentence that stopped is seen even while another flows).
- `fn parse(&mut self, line: &str, now_us: u64) -> Update` — Parse one NMEA sentence (plan B `_parse`): RMC -> the fix, GGA -> the altitude and quality, GSV/GSA/TXT -> the sky. A sentence without '$' or with a bad checksum is ignored. A malformed field drops the rest of the line, keeping what was already applied, as plan B does.
- `fn message(&mut self, message: &ubx::Message, now_us: u64) -> (Update, Option<ubx::Answer>)` — One UBX frame as the NEO-M8N delivered it (M16): count it, apply a NAV-PVT ([`Gnss::pvt`]) or a NAV-SAT ([`Gnss::sat`]), hand back an ACK's answer, then sweep for outages, as [`Gnss::line`] does for a line. Any other message is counted and ignored.
- `fn pvt(&mut self, pvt: &ubx::Pvt, now_us: u64) -> Update` — One navigation solution: what an RMC and a GGA give together. The fix flag, quality, satellites and the DOP always (the position DOP stands in for HDOP: NAV-PVT carries no horizontal one); with a fix the position, the ground speed and the heading of motion, and the fix row; with a 3D fix the altitude and the elevation above the first one; the GGA row on every solution, as on every GGA.
- `fn sat(&mut self, sat: ubx::Sat, now_us: u64)` — One sky view: it stands for the GSV and GSA burst, and restarts the burst's quiet clock (one message is the whole burst).
- `fn sky_row(&self) -> SkyRow` — The sky as the `<name>_sky.csv` row (plan B `_sky_row`): each talker's last whole group and each system's last GSA count; all zeros before the first diagnostics.
- `fn settle(&mut self, now_us: u64) -> Option<(SkyRow, Option<Event>)>` — Push the sky row once a burst is over (plan B `_settle`): on the first tick that finds diagnostics no row carries yet AND none of them in the last half second.
- `fn sky_window(&mut self, stage: Stage) -> Option<(SkyFrame, Event)>` — Keep the diagnostics to the pad (plan B `_sky_window`): on before BOOSTING and again from DONE, off from BOOSTING until DONE. Only a change writes, one write per edge; the frame is the driver's, made at setup. Returns the frame and its event at an edge, None while the window holds.
- `fn sweep(&mut self, now_us: u64) -> Vec<Event>` — Event every sentence that went quiet past the outage gap, once per outage (plan B `_sweep`). It needs a clock that runs when nothing arrives, so the run loop calls it on every line AND the tick on its own clock: a receiver that goes completely silent (power lost, connector out) parses nothing.
- `fn tick(&mut self, now_us: u64, stage: Stage) -> Tick` — One pass of the 1 s tick (plan B `_sweeping`'s loop body): the outage sweep, the sky window for the stage now (read on every tick, not once), then the burst-ending settle.
- `fn fix(&self) -> bool` — The last RMC had a valid fix.
- `fn fix_quality(&self) -> i32` — GGA's fix quality: 0 none, 1 GPS, 2 DGPS.
- `fn satellites(&self) -> i32` — GGA's satellites used in the fix: watch it rise with a better antenna.
- `fn hdop(&self) -> f32` — GGA's horizontal dilution of precision (lower is better geometry).
- `fn cn0_mean(&self) -> f32` — The mean of the four strongest C/N0 in dB-Hz: the "good enough to launch" yardstick (see [`SkyRow::cn0`]).
- `fn lines(&self) -> u32` — Lines read so far.
- `fn diagnosing(&self) -> bool` — Whether the diagnostics are on.

### `pub fn checksum_ok(sentence: &str) -> bool`

Verify a sentence's `*hh` XOR checksum over the characters between '$' and the last '*' (plan B
`checksum_ok`).

False on a missing '*', and when the (at most two) characters after it are no hex number. They are read as
Python's `int(text, 16)` reads them: surrounding whitespace and a sign allowed, either case, so `*5` (one digit)
and `*0b` pass when they name the checksum. A sentence that is not ASCII fails (see the crate notes).

### `pub fn degrees_e7(value: &str, hemisphere: &str) -> Result<Option<i32>, Malformed>`

An NMEA ddmm.mmmm (or dddmm.mmmm) field and its hemisphere as signed 1e-7 degree units (plan B `degrees`, in
integers).

The degrees are the digits before the last two before the dot, the minutes the rest; the units are rounded to
the nearest (a tie away from zero). S and W are negative, any other hemisphere (even empty) positive, as in plan
B. Minutes of 60 or more are taken as they come, as plan B takes them.

Returns Ok(None) for an empty field (no fix yet), Ok(units), or Malformed when the field is not digits with a
dot, has fewer than three digits before the dot, or names more than an i32 of units (past 214.7 degrees).

### `pub fn nmea(body: &str) -> Vec<u8>`

Wrap a command body in `$...*hh\r\n` with its XOR checksum (plan B `nmea`), for the PCAS/PMTK/PUBX config
sentences. The checksum covers the body's bytes (plan B counted characters: the same for ASCII, which every
command is).

### Constants

- `const DIAGNOSTICS_S: i32 = 10` — The sky diagnostics come about this often, in seconds, at any fix rate: the drivers' dividers are `hz` times this (plan B `gnss.DIAGNOSTICS_S`).
- `const SWEEP_MS: u32 = 1000` — The tick's period in milliseconds: the outage sweep's own clock, for a receiver that sends nothing, and the burst-ending clock of the sky row (plan B `_SWEEP_MS`).

## `gnss::drivers`

The GNSS module drivers (src/glider drivers/atgm336h.py, drivers/neo6mv2.py; the NEO-M8N's is the port's own, M16):
the bytes each module's setup sends, and the sky switch pair it keeps. The ATGM336H and the NEO-6M share the NMEA
receiver of the crate root; the NEO-M8N delivers UBX ([`crate::ubx`]). Each sits on the same UART: the board
config's `driver` picks one.

## `gnss::drivers::atgm336h`

ATGM336H GNSS, GPS + BDS on a CASIC chip (src/glider drivers/atgm336h.py): the CASIC reconfiguration as bytes.

RMC at `hz` (the position) plus GGA at ~1 Hz (the altitude and elevation, a baro backup): both fit 9600 baud
(~10 Hz RMC is ~700 B/s, ~1 Hz GGA ~70 B/s, under 960). Then the sky diagnostics every ~10 s: GSA, GSV and the
antenna text, ~0.5 kB a burst (~50 B/s); on every fix they would not fit (5 kB/s at 10 Hz). PCAS is the CASIC
command set; the PMTK pair goes out too, a fallback for MTK-variant modules (each side ignores the other's
sentences). The 2026-10-04 bench (src/gnss_bench) flew exactly this sequence and measured the diagnostics NOT to
disturb the flight rates: at hz 10, 250 RMC + 25 GGA in 25 s, the diagnostics at ~9.5 s and ~19.4 s.

### `pub fn commands(hz: i32) -> [Vec<u8>; 5]`

The setup sentences for `hz` fixes per second, framed, in send order (plan B `_commands`).

The flight init first (RMC every fix, GGA every `hz`th, the measurement period, the MTK pair), then the
diagnostics: the same PCAS03 with GSA, GSV and the antenna text raised from 0 to every Nth fix (N = `hz` x 10 s,
capped at 99). A rate below 1 counts as 1, so there is never a divide by zero.

### `pub fn sky_switch(hz: i32) -> SkySwitch`

The sky switch for `hz` (plan B `_configure`'s return): off is the flight init's own PCAS03 (RMC + GGA, nothing
else: the flight output as it was before the diagnostics), on is the diagnostics' PCAS03.

### Constants

- `const SEND_GAP_MS: u32 = 80` — The pause between two setup frames, in milliseconds (plan B's `_configure`): the module parses one sentence before the next arrives.

## `gnss::drivers::neo6mv2`

GY-NEO6MV2, a u-blox NEO-6M (src/glider drivers/neo6mv2.py): the u-blox reconfiguration as bytes. A drop-in
alternative to the ATGM336H on the same UART (lower `hz`: the NEO-6M tops out near 5 Hz).

`$PUBX,40` selects RMC (the position) and GGA at ~1 Hz (the altitude and elevation) on the UART and silences the
rest; UBX-CFG-RATE sets the measurement period; UBX-CFG-NAV5 the airborne dynamic model; then `$PUBX,40` turns
GSA and GSV back on every ~10 s, the sky diagnostics (the NEO-6M reports no antenna status over NMEA). The
2026-10-04 bench (src/gnss_bench) flew this sequence without the NAV5 frame and measured the diagnostics NOT to
disturb the flight rates: at hz 5, 125 RMC + 25 GGA in 25 s, GSA + GSV every ~10 s.

### `pub fn ubx(class: u8, id: u8, payload: &[u8]) -> Vec<u8>`

A UBX binary frame (plan B `_ubx`): the sync characters, the class, the id, the little-endian payload length, the
payload, and the 8-bit Fletcher checksum over class..payload.

Panics when the payload is longer than UBX's u16 length can say (65535 bytes); the frames here are 6 and 36.

### `pub fn commands(hz: i32) -> [Vec<u8>; 10]`

The setup frames for `hz` fixes per second, in send order (plan B `_commands`).

The flight init first: the `$PUBX,40` selection (RMC every fix, GGA every `hz`th, GLL/GSA/GSV/VTG silenced to
stay within 9600 baud), UBX-CFG-RATE (the period; navRate 1, GPS time), UBX-CFG-NAV5 (airborne <4g). Then the
diagnostics: GSA and GSV every Nth fix (N = `hz` x 10 s, capped at 255). A rate below 1 counts as 1.

### `pub fn sky_switch(hz: i32) -> SkySwitch`

The sky switch for `hz` (plan B `_configure`'s return): off is the flight init's own GSA and GSV at 0 (the flight
output as it was before the diagnostics), on is the diagnostics' GSA and GSV, each pair one write. u-blox parses
its input sentence by sentence, but this back-to-back pair (the init puts 40 ms between them) is not bench-checked
yet.

### Constants

- `const SEND_GAP_MS: u32 = 40` — The pause between two setup frames, in milliseconds (plan B's `_configure`).

## `gnss::drivers::neom8n`

_Tested by its own `mod tests`._

The u-blox NEO-M8N (M16): binary delivery. Plan B had no driver for it (its NEO-M8N ran as the NEO-6M, on NMEA);
this one switches the module's UART to UBX only and reads the navigation solution as one UBX-NAV-PVT per epoch
(crate::ubx), so no epoch is half delivered and no text is parsed. Every frame is UBX:

* UBX-CFG-PRT: UART1 at [`BAUD`] (8N1), UBX and NMEA taken in, UBX only out. 9600 baud carries a 100-byte NAV-PVT
  at most ~9 times a second with nothing else, so the port goes up first;
* UBX-CFG-RATE: the measurement period (the entry's `hz`; the M8N runs GPS + GLONASS at up to 10 Hz);
* UBX-CFG-NAV5: the airborne <4g dynamic model, the NEO-6M driver's (drivers::neo6mv2 says why);
* UBX-CFG-MSG: NAV-PVT on every solution, and NAV-SAT every `hz` x 10 s, the sky diagnostics (its switch).

Each frame is answered with an ACK-ACK or an ACK-NAK, which the firmware logs: the NMEA path could only tell a
refusal from a `$GPTXT ... inv format`.

**The baud after a P4 reboot (autobaud):** the module keeps the port settings until its own power cycle, so a P4
that reboots while the module stays powered finds it at [`BAUD`], not at its power-on 9600. The firmware writes
the CFG-PRT frame at both rates ([`port`] at [`BAUD`] first, then at the power-on rate) and talks at [`BAUD`]
after: whichever rate the module listens at, one copy lands, and the other is noise it drops.

### `pub fn port(baud: u32) -> Vec<u8>`

The CFG-PRT frame: UART1 at `baud`, 8N1, UBX and NMEA in, UBX out.

### `pub fn commands(hz: i32) -> [Vec<u8>; 4]`

The setup frames after the port's ([`port`]), at `hz` solutions per second, in send order: the period, the
dynamic model, NAV-PVT on every solution, NAV-SAT every 10 s.

### `pub fn sky_switch(hz: i32) -> SkySwitch`

The sky switch for `hz`: NAV-SAT off for the flight, back every 10 s on the pad.

### Constants

- `const SEND_GAP_MS: u32 = 40` — The pause between two setup frames, ms: the module answers each with its ACK first.
- `const BAUD: u32 = 38_400` — The UART's rate after the configuration: NAV-PVT (100 bytes) at 10 Hz and a NAV-SAT (~300 bytes) every 10 s use about a quarter of it.

## `gnss::ubx`

_Tested by its own `mod tests`._

u-blox's binary protocol (UBX), the NEO-M8N's delivery (M16): a frame is `µb`, the class and the id, the
little-endian payload length, the payload, and the 8-bit Fletcher checksum over class..payload ([`frame`] builds
one, [`Frames`] finds them in the byte stream). What the receiver reads of it:
* UBX-NAV-PVT ([`Pvt`]): the whole navigation solution in one message per epoch -- the time, the fix type and
  flags, the satellites used, the position already in 1e-7 degree integers, the heights in mm, the NED velocity,
  the ground speed and the heading of motion, the position DOP. It replaces NMEA's RMC + GGA pair: one message, so
  no epoch is ever half delivered, and no text to parse;
* UBX-NAV-SAT ([`Sat`]): every satellite tracked, its C/N0 and whether the solution uses it: the sky diagnostics
  (NMEA's GSV + GSA);
* UBX-ACK-ACK / ACK-NAK: the module's answer to each configuration frame;
* UBX-MON-VER ([`Version`]): the module's own name for itself, its answer to the poll [`frame`] with no payload
  makes: proof that it hears the board, and which receiver it is (a clone shows here).

The layouts are the u-blox 8 / M8 Receiver Description's (protocol 15 and later). Integers only: the position
never passes through a float (`Position`'s 1e-7 degree, as the NMEA path keeps it).

### `pub struct Message`

A frame's class, id and payload, its checksum passed.

### `pub struct Frames`

The UBX frames in a byte stream, found byte by byte: anything between frames (the module's NMEA before its
configuration took, line noise) is skipped, and a frame whose checksum fails is dropped and counted.

- `fn new() -> Frames` — Nothing heard yet.
- `fn push(&mut self, byte: u8) -> Option<Message>` — One byte: the frame it completes, if any.
- `fn rejected(&self) -> u32` — Frames dropped so far: a failed checksum, or a length past [`MAXIMUM_PAYLOAD`].

### `pub struct Pvt`

One UBX-NAV-PVT: the navigation solution of one epoch.

- `fn decode(payload: &[u8]) -> Option<Pvt>` — The payload of a NAV-PVT, None when it is not one (a short or long payload).
- `fn fixed(&self) -> bool` — Whether the solution is a fix to use: gnssFixOK, and a 2D, 3D or GNSS + dead-reckoning fix (NMEA RMC's `A`).
- `fn quality(&self) -> i32` — The fix quality as GGA spells it: 0 none, 1 GPS, 2 differential.
- `fn mode(&self) -> i32` — The fix mode as GSA spells it: 1 none, 2 2D, 3 3D.

### `pub struct Sat`

One UBX-NAV-SAT, folded: what the sky row reads of it.

- `fn decode(payload: &[u8]) -> Option<Sat>` — The payload of a NAV-SAT, None when its length does not match its count.

### `pub struct Version`

One UBX-MON-VER: the module's software and hardware versions and its extension strings (`PROTVER=18.00`,
`MOD=NEO-M8N`, ...).

- `fn decode(payload: &[u8]) -> Option<Version>` — The payload of a MON-VER, None when its length is not 40 plus whole extensions.

### `pub enum Answer`

The module's answer to a configuration frame.

- `fn decode(id: u8, payload: &[u8]) -> Option<Answer>` — The ACK class's message `id` with `payload`, None for anything else.

### `pub fn frame(class: u8, id: u8, payload: &[u8]) -> Vec<u8>`

One UBX frame of `class`/`id` around `payload`.

Panics when the payload is longer than UBX's u16 length can say (65535 bytes).

### `pub fn configuration_name(id: u8) -> String`

The name a configuration message (UBX-CFG `id`) goes by in the u-blox manual, for the log: `CFG-PRT`; a hex id for
one the drivers do not send.

### Constants

- `const SYNC: [u8; 2] = [0xB5, 0x62]` — UBX's two sync characters, `µb`.
- `const NAVIGATION_CLASS: u8 = 0x01` — UBX-NAV, the navigation results class.
- `const PVT: u8 = 0x07` — UBX-NAV-PVT: the navigation solution.
- `const SAT: u8 = 0x35` — UBX-NAV-SAT: the satellites.
- `const ACKNOWLEDGEMENT_CLASS: u8 = 0x05` — UBX-ACK, the acknowledgement class.
- `const ACK: u8 = 0x01` — UBX-ACK-ACK: the configuration frame was taken.
- `const NAK: u8 = 0x00` — UBX-ACK-NAK: the configuration frame was refused.
- `const CONFIGURATION_CLASS: u8 = 0x06` — UBX-CFG, the configuration class.
- `const MONITOR_CLASS: u8 = 0x0A` — UBX-MON, the monitoring class.
- `const VERSION: u8 = 0x04` — UBX-MON-VER: the receiver's software, hardware and protocol versions.
- `const MAXIMUM_PAYLOAD: usize = 1280` — The longest payload [`Frames`] keeps: NAV-SAT with 100 satellites is 1208 bytes; a longer length is noise.

# `mission` — `src/avionics/crates/mission`

_Uses: `control`. External: serde_json._

_Integration tests: `tests/golden.rs`, `tests/golden_field.rs`._

## `mission`

_Tested by its own `mod tests`._

The mission (src/glider mission.py): the per-launch identity the operator sets before a flight. It holds a
launch id, the launch site and its position (a known origin), the landing zone, and the known sites for
CC-less field operation, as the launch.config document. Unlike the board config (hardware, stable across
flights) it changes every launch.

This crate is the document and its rules, with no storage and no clock. [`Mission::load`] takes the stored
text and [`Mission::persisted`] gives the document to store. [`Mission::inspect`] takes the clock text and
epoch from its caller, and the live GNSS fix is an argument too (`fix`: a FRESH fix or None, as plan B's
databoard read gives it). A time set is the caller's job: [`epoch_request`] says whether an update asks for
one, by plan B's `set_time` rule, and [`TimeSet`] holds the momentary inputs riding with it for the
session.csv row.

Positions are [`Position`] integers (1e-7 degree; see `control::navigation`). A JSON number is an f64, so a
coordinate is f64 at the JSON boundary and nowhere else (decision D18): it is read, range-checked and
rounded to units on the way in, and divided back out on the way out. Metres are f32.

Deliberate differences from plan B:
 * `sites` survive. Plan B's persisted() leaves them out, so every `set-config launch` silently erased the
   CC-less site list at the next boot. Here [`Mission::persisted`] keeps them, and [`Mission::merge`] (the
   `set-config launch` path) replaces them when its payload carries a `sites` list.
 * `launch_id` and `site` are text and `altitude` is metres. Plan B stores whatever JSON value arrives;
   here an update of another type is ignored (and not listed as changed), and a stored one loads as unset
   ('' or None). A site entry whose `name` is not text is named as if it had none (`site<n>`), where plan B
   stored the Python repr of the value.
 * Metres and coordinates always go out as JSON floats. Plan B echoes the JSON number it was given, so an
   altitude, coordinate or zone corner sent as `5` reads back `5` there and `5.0` here: the same value of
   another JSON kind. Keeping the kind would mean storing it beside every number, for no reader that needs
   it.
 * A position is rounded to the nearest 1e-7 degree (1.1 cm) on the way in. Plan B keeps the f64 it was
   given: an update that moves a coordinate by less than half a unit is listed as changed there and not
   here, and `persisted`, `inspect` and `get-config launch` echo 7 decimals where plan B echoes all of them
   (CC's `assist` sends NMEA-derived degrees with about 15 significant digits).
 * Nothing is memoized. Plan B caches the zone geometry by identity for its hot path; here it is a little
   integer and f32 work, done when asked.

The CC-less field agent (plan B tasks/field.py) is [`field`]: the site by GPS through [`Mission::select_site`] and
[`Mission::fallback_zone`], and the auto-arm's stillness dwell. A milestone that needs one ports it here
first, as M13 did `zone_aspect` and `endgame_heading` for the guidance law.

### `pub struct Zone`

A landing zone: two diagonally opposite corners, as the operator set them.

The corner order is kept as given. The geometry does not depend on it (navigation::zone), but the operator
chose it, and the stored document gives it back unchanged.

### `pub struct Site`

A known launch location for CC-less field operation (doc/specs/coludo.md "Field operation without CC"): the
board picks the nearest one whose pad is within range of its GNSS fix.

### `pub struct Geometry`

The landing zone resolved against the launch point (mission.py `geometry`).

### `pub struct TimeSet`

The momentary inputs that ride with a time set (`update mission {epoch, ...}`) into its session.csv row:
written to the row, never stored (mission.py `_index_time`). An invalid one is None and becomes an empty
cell; it never refuses the row or the time set.

- `fn from_properties(properties: &Map<String, Value>) -> TimeSet` — The time-set inputs of an `update mission` request, each validated as plan B's `_index_time` does.

### `pub struct Mission`

The operator-set launch identity: the launch.config document.

The fields are public to read. Change them through [`Mission::update`] or [`Mission::merge`], which
validate as plan B does; a field set directly skips that.

- `fn load(stored: Option<&str>, max_range_m: f32) -> Mission` — The mission from the stored launch.config text (mission.py `Mission(path, max_range_m)`).
- `fn update(&mut self, properties: &Map<String, Value>) -> Vec<&'static str>` — Apply the editable mission fields of an `update mission` request (mission.py `update`, less the clock).
- `fn merge(&mut self, properties: &Map<String, Value>) -> Vec<&'static str>` — Apply a `set-config launch` payload: [`Mission::update`], plus `sites`.
- `fn position_set(&self) -> bool` — Whether the operator set the launch position (both coordinates): the dashboard's 'launchpad_set'.
- `fn freeze_launch(&mut self, fix: Option<Position>) -> Option<Position>` — Pins the live GNSS `fix` as the launch point (mission.py `freeze_launch`), at arm and again at BOOSTING entry, the last moments the board is known to be on the pad: mid-flight the fix can drop, and the launch point must not drop with it. An operator-set position always wins, and with no fix nothing changes. In memory only, as plan B's: the pinned point is the operator's from here, so a later freeze keeps it.
- `fn max_range_m(&self) -> f32` — The range limit in metres from the launch point to any zone point (the board config's `field.max_range_m`).
- `fn select_site(&mut self, fix: Position) -> Option<String>` — CC-less site selection (mission.py `select_site`): the nearest known site whose pad is within `max_range_m` of the live `fix` becomes the mission's site and zone, a later site at an equal distance winning as plan B's `<=` lets it. The launch point is left as it is: the live fix stands in for it until `arm` pins it.
- `fn fallback_zone(&mut self, fix: Position, bearing_deg: f32, near_m: f32, width_m: f32, depth_m: f32) -> Zone` — The spiral-landing fallback (mission.py `fallback_zone`): a generous axis-aligned box the mission adopts as its zone, site 'fallback'. Its wide side (`width_m`) faces the pad, its near edge `near_m` out from `fix`, running `depth_m` away; its centre `near_m + depth_m / 2` from the fix at `bearing_deg` (the operator's clear sector). Degrees in f64, as plan B's CPython reference computes them, then rounded to the position units.
- `fn launch_point(&self, fix: Option<Position>) -> Option<Position>` — The launch origin (mission.py `launch_point`): the operator-set position when both coordinates are set, else the live GNSS `fix`, else None.
- `fn geometry(&self, fix: Option<Position>) -> Option<Geometry>` — The landing zone resolved against the launch point (mission.py `geometry`); None when the zone or the launch point is unset.
- `fn zone_aspect(&self) -> f32` — The landing zone's long/short side ratio, >= 1 (mission.py `zone_aspect`); 1.0 when no zone is set, as plan B's. [`Mission::endgame_heading`] decides by it.
- `fn endgame_heading(&self) -> Heading` — The endgame holding pattern the guidance law's 'auto' setting resolves to (mission.py `endgame_heading`), decided by the zone's SHAPE, its long/short aspect k.
- `fn probe(&self, fix: Option<Position>) -> Option<String>` — The pre-flight self-test (mission.py `probe`): a launch position is known (set, or the live fix) and, when a landing zone is set, all three of its points are within range.
- `fn inspect(&self, fix: Option<Position>, clock: &str, epoch: i64) -> Value` — The operator-facing snapshot (`inspect mission`; mission.py `inspect`), with plan B's keys and meanings.
- `fn persisted(&self) -> Value` — The document as launch.config stores it: launch_id, site, latitude, longitude, altitude, zone and sites. No computed geometry and no clock. `get-config launch` answers it, and the dashboard's editor sends it back through `set-config launch`.

### `pub fn epoch_request(properties: &Map<String, Value>) -> Option<i64>`

The epoch an `update mission` request asks the clock to be set to, by plan B's `set_time` rule: an integer
(a bool, a float or text is not) of Unix seconds no earlier than 2000-01-01. None when there is no such
request.

The caller sets its clock from it and lists 'epoch' as changed. A time set also writes the boot's
session.csv row from [`TimeSet::from_properties`].

### `pub fn clock_text(epoch: i64) -> String`

A Unix epoch as the board clock's text, 'YYYY-MM-DDTHH:MM:SS' in UTC (mission.py `clock`, which reads the
board's UTC RTC).

### Constants

- `const DEFAULT_MAX_RANGE_M: f32 = 200.0` — The default range limit, in metres, from the launch point to any zone point (mission.py `_DEFAULT_MAX_RANGE_M`). The real figure is the board config's `field.max_range_m`: a glide-range property of the airframe.

## `mission::field`

The CC-less field agent (plan B tasks/field.py; doc/specs/coludo.md "Field operation without CC"), disabled by
default. On the pad (SETTING) it makes at most two decisions:
 1. the site by GPS: on the first fresh fix the mission adopts the nearest known site within `max_range_m`
    ([`Mission::select_site`]), else the synthesized fallback zone at the clear-sector bearing
    ([`Mission::fallback_zone`]);
 2. the auto-arm (opt-in): once the board sat still (a fresh accel inside the still band) with a fresh fix for
    `auto_arm_dwell_s`, with a zone set and not armed, [`Field::tick`] asks for the arm. The caller runs the arm's
    gates and arms.

The port's own: plan B armed the Controller directly, past every gate of the CC `arm`. Here the caller keeps the
passive gates (the operator, 2026-10-11) and, when one refuses, calls [`Field::refused`], which restarts the dwell:
the agent asks again after another whole dwell, where plan B's direct arm never failed.

A knob plan B would run with but that makes no sense refuses the entry, naming the key ([`Settings::from_config`]).

### `pub struct Settings`

The field entry's knobs (plan B field.py setup), each with plan B's default.

- `fn from_config(fields: &Map<String, Value>) -> Result<Settings, String>` — The knobs from the entry's fields; a missing key takes plan B's default.

### `pub enum SitePick`

What the site decision did on a tick, for the log line.

### `pub struct Tick`

One tick's decisions.

### `pub struct Field`

The field agent's state.

- `fn new(settings: Settings) -> Field` — The agent with `settings`, nothing decided yet.
- `fn settings(&self) -> &Settings` — The knobs.
- `fn tick(&mut self, now_ms: u64, setting: bool, armed: bool, fix: Option<Position>, accel: Option<[f32; 3]>, mission: &mut Mission) -> Tick` — One decision step at `now_ms` (plan B `_tick`), only in SETTING (`setting`): the site by the first fresh `fix`, then, with the auto-arm on, unarmed and a zone set, the dwell over a fresh `accel` (g) inside the still band. A missing fix or accel, or one outside the band, restarts the dwell.
- `fn refused(&mut self)` — The arm the dwell asked for was refused: the dwell restarts, so the agent asks again after another whole one.

# `panics` — `src/avionics/crates/panics`

_Uses: `flash_ring`. External: none._

## `panics`

_Tested by its own `mod tests`._

The panic log (doc/specs/avionics-port.md, "Panic log"): the `panic` partition keeps the board's last panics as
text records, round-robin over [`SLOT_BYTES`] slots, the way Linux's mtdoops keeps oopses in an MTD partition. A
panic is fatal or not:
* fatal: the board reset (a panic, a watchdog, a brownout). The text is what the board captured as it went down
  -- the panic handler's output and a Rust panic's message, which components/panic_capture keeps in RTC RAM across
  the reset -- and the next boot writes the record, with the OS up, never the panic handler;
* not fatal (an oops): the board noticed something broken and went on (a radio link it parked, a bus it had to
  reset). The firmware writes the record as it goes on.

A slot holds a record when its first [`HEADER_BYTES`] start with a whole header line:

```text
COLUDO-PANIC 1 seq=12 fatal=1 cause=panic boot=351 session=000351 uptime_ms=1583111 firmware=avionics-0.1.0 config=3aa09342698a recorded=352 length=2817 intact=1
```

`seq` counts records over the partition's life, so the newest is the largest and the next record goes into the slot
after it, erasing the oldest (a u32: past 4 billion records the order would break, which a board never reaches). The text goes in first and the header last: a power cut leaves no header, so a torn
record is an empty slot, the next one written there. Anything else in a slot (an erased one, another firmware's
bytes) is no record either.

Everything runs over flash_ring's [`Flash`] trait, so the host tests use its [`flash_ring::MemoryFlash`] and its
power cuts. `no_std` + `alloc`.

### `pub struct Tags`

What a record says about the panic it records: its header line's tags.

- `fn header(&self) -> String` — The header line, `\n` included. A tag's value keeps letters, digits and `._:+/-`; anything else becomes `_`, an empty value is written `-`, and a text tag is cut to [`TAG_BYTES`], so the line splits on spaces and stays within [`HEADER_BYTES`].
- `fn parse(bytes: &[u8]) -> Option<Tags>` — The tags of a slot's first bytes: None unless they start with a whole header line of this layout (ASCII, ending in `\n` within [`HEADER_BYTES`], with `seq` and `length`). A length past [`TEXT_BYTES`] is held to it.

### `pub enum PanicLogError<E>`

Why a panic log call failed.

### `pub struct PanicLog<F: Flash>`

The panic partition: its slots and the records they hold.

- `fn open(flash: F) -> Result<Self, PanicLogError<F::Error>>` — Reads every slot's header.
- `fn capacity(&self) -> u32` — How many records the partition holds at most.
- `fn records(&self) -> Vec<(u32, &Tags)>` — The records, oldest first, with their slots.
- `fn newest(&self) -> Option<&Tags>` — The newest record's tags, if any.
- `fn append(&mut self, mut tags: Tags, text: &[u8]) -> Result<Tags, PanicLogError<F::Error>>` — Writes a record into the slot after the newest one (slot 0 on an empty log), erasing whatever it held: the text first (held to [`TEXT_BYTES`]), the header last. `tags.seq` and `tags.length` are set here.
- `fn text(&self, seq: u32) -> Result<Option<Vec<u8>>, PanicLogError<F::Error>>` — The record with this `seq` as text: its header line, then its text. None when no slot holds it.

### Constants

- `const SLOT_BYTES: u32 = 2 * BLOCK_BYTES` — One record's slot: two erase sectors.
- `const HEADER_BYTES: u32 = 512` — The room for the header line at the slot's start; the text follows it. The line's fixed part is under 170 bytes, and each of its four text tags is held to [`TAG_BYTES`], so a header always fits.
- `const TAG_BYTES: usize = 64` — The longest text tag (cause, session, firmware, config) a header keeps; a longer one is cut.
- `const TEXT_BYTES: u32 = SLOT_BYTES - HEADER_BYTES` — The most text a record keeps.

# `recorder` — `src/avionics/crates/recorder`

_Uses: `flash_ring`, `recorder_wire`. External: log._

_Integration tests: `tests/end_to_end.rs`, `tests/golden.rs`, `tests/newest_wins_cut.rs`, `tests/power_cut.rs`, `tests/repeats.rs`, `tests/rows.rs`, `tests/saver.rs`, `tests/saver_tools.rs`, `tests/writer.rs`._

## `recorder`

The recorder (doc/specs/avionics-port.md "Storage design": "Producer side", "The ring_writer", "The
ring_saver" Decode, "Row, log and index text", "CC tees and the HTTP exits"), the port of src/glider
recorder.py with the UART to the Luckfox replaced by the flash ring:

```text
producers (any core) -- binary records --> PSRAM SPSC ring per thread --> Writer::step --> flash_ring::Ring
  Telemetry::push, Recorder::log             (+ misc ring, try_lock)        | verbatim, into chunks
  Recorder::index_session                                                   +--> CC tees (log, tlm)

flash block --> Decoder::block --> wire lines `[@<routing>@]{OPEN};<payload>;<CLOSE>\n` (ring_saver, /ring, tools)

flash_ring::Ring --> Saver::step: Walk (in sequence order, up to writer_position) --> Decoder --> batches
  --> Fs, the card's root: <session>/recordings/<session>_<file>, <session>/recordings/recorder.log,
      <session>/recordings/session.csv + sessions.csv (layout); fsync
  --> saver_position (the eraser erases behind it)
```

* [`Registry`]: the streams, built before any thread spawns and then fixed.
* [`Recorder`]: the shared state, one per boot (`&'static` on the board): the producer rings, the tees, the
  counters, `writer_position`, the flight flag, and [`Recorder::log`], [`Recorder::index_session`],
  [`Recorder::stats`], [`Recorder::inspect`] and [`Recorder::set_stats_ms`] (plan B's `update`).
* [`Telemetry`]: a stream's producer side, `due`/`push`, owned by its thread.
* [`Writer`]: the ring_writer's loop body, [`Writer::step`], generic over [`flash_ring::Flash`].
* [`Decoder`]: a flash block back to the lines recorderd wrote, for the ring_saver and the tools.
* [`Saver`]: the ring_saver's loop body, [`Saver::step`], over a [`Walk`] of the ring and the card's [`Fs`]
  ([`StdFs`] on the board and the host): the flash emptied to the SD in batches, `saver_position` published only
  once a batch is fsynced.
* [`layout`]: the one place that maps (session, kind) to a directory on the card, and the global index's name;
  the saver, the firmware's camera and the host tests all ask it.

Nothing on a producer's path or in the writer prints a record: the records are binary from the producer to
flash, and only the decoder and a tee's drain make text, off the flight path (invariant 4).

Portable and host-tested; it runs on the ESP32-P4 (rv32imafc) too, so it holds no 64-bit atomic (the P4 has
none) and no mutex on a producer's path. It needs std for the thread-local ring binding and the locks that
guard the shared `misc` ring and a tee's consumer side, neither on a hot path.

## `recorder::byte_ring`

_Tested by its own `mod tests`._

The single-producer, single-consumer byte ring under every recorder queue (doc/specs/avionics-port.md
"Producer side"): a producing thread's records on their way to the ring_writer, and the writer's copies of
them on their way to a CC tee.

A record is a u16 length (the record's whole size, a multiple of 4) followed by its bytes, padded with zeros
to that size: the record format's own length field, so a record crosses the ring, the flash chunk and the tee
byte for byte. A length of 0 is the wrap marker: the record that did not fit before the buffer's end starts
at offset 0. The producer owns `head` and the consumer owns `tail`, both AtomicUsize: the producer publishes
a record with a Release store of `head`, and the consumer frees its bytes with a Release store of `tail`,
each read by the other side with Acquire. Neither side ever writes the other's index, so no lock is taken
and no 64-bit atomic is needed (the P4 has none).

`head == tail` means empty, so a record may never make `head` reach `tail`: a full ring refuses the record
and keeps everything queued. The free space can be split in two by the buffer's end, and a record takes one
piece whole, so a ring takes records much smaller than itself (a record is at most one 4032-byte chunk
payload, the smallest ring 128 KiB).

The buffer is the caller's, `&'static mut [u8]`: the firmware hands over PSRAM it allocated with
`heap_caps_malloc` (and never frees), the host tests a leaked Vec. Owning it for good is what makes the raw
accesses below sound: no other reference to those bytes can exist.

## `recorder::decoder`

_Tested by its own `mod tests`._

The block decoder (doc/specs/avionics-port.md "The ring_saver", Decode; "CC tees and the HTTP exits"): one
flash block, its header and its chunks, turned into the wire lines recorderd wrote. The ring_saver (M4) files
them on the SD, `GET /ring` streams them, and the host tools read a ring image through it, so a session reads
the same whichever way it left the board.

For each block:
* the block header names the session (the SD directory) and the boot. A block of another layout version is
  refused ([`BlockError::Version`]): this build cannot read its chunks, and keeps it for the build that can;
* the chunk walk checks every CRC and steps over a torn chunk to the next whole one
  ([`flash_ring::chunk::chunks`]);
* each record prints as its line (a ROW `<uptime>;v1;...;vn`, a HEADER its column list, a LOG
  `<uptime_us> <descriptor> :: <message>`), wrapped with its routing: `<session>_<file>`, the bare `session.csv`
  for the index, or none for recorder.log: `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`. A record whose file name
  the saver would not file goes to recorder.log, unrouted;
* a HEADER record prints only when its file starts or its columns change, per boot of a session (its block
  headers' session and boot id): the writer repeats every header about once per MiB, and those repeats never
  reach the SD. Each boot's first header does, as plan B and recorderd wrote one per boot: a session label
  reused over several boots (a stale `recorder.session` in a saved config) appends them all to one file, and
  its second header line is what makes `flight_telemetry` flag the splice. Two uncounted boots (boot id 0)
  under one label stay indistinguishable;
* a torn chunk is a loss only when nothing whole follows it in its block and no later block of its boot is in
  flash (spec "Block and chunk format"): a failed program the writer retried is not. A loss prints as a
  recorder.log line of its own, `<uptime> ring :: block <sequence>: torn chunk at <offset>, <k> records lost`,
  wrapped like any other, so a wrapped session never holds an unwrapped line.

The loss line's uptime is the newest one the decoder has seen in that boot, so recorder.log never goes back in
time within it (a backward jump of more than half a 2^30 period reads as a ticks wrap to `flight_telemetry`),
and a boot that reuses a label is never stamped with an earlier boot's uptimes. Its `k` is the torn chunk's
record count as read, `?` when the count never landed.

The decoder keeps that state for the boot it is in alone: blocks come in `block_sequence` order and a boot's
blocks are numbered one after another, so a boot left behind never comes back, and a walk over a backlog of
many boots holds no more than one boot's headers.

### `pub enum BlockError`

Why a block is not decoded.

### `pub struct BlockSummary`

What one block held.

### `pub struct DecodedLine<'a>`

One decoded line.

- `fn body(&self) -> &'a [u8]` — The line as its file holds it, `{OPEN};<payload>;<CLOSE>\n`: the wire line after `@<routing>@`, which is exactly what recorderd appended to the routing's file.

### `pub struct Decoder`

Decodes blocks into wire lines. It remembers, for the boot it is in, the columns each file's header line printed
and the newest uptime seen, so feed it a ring's blocks in `block_sequence` order: a block of another boot starts
that memory again. A new one prints every file's first header again: the ring_saver keeps one for as long as it
appends to the same files.

- `fn new() -> Decoder` — A decoder that has printed nothing yet.
- `fn block(&mut self, block: &[u8], boot_goes_on: bool, each: impl FnMut(DecodedLine<'_>)) -> Result<BlockSummary, BlockError>` — Decode one block into its lines, in record order, calling `each` for every line.
- `fn block_from(&mut self, block: &[u8], from: u32, boot_goes_on: bool, mut each: impl FnMut(DecodedLine<'_>)) -> Result<BlockSummary, BlockError>` — As [`Decoder::block`], for the part of the block from offset `from` on: the ring_saver reads the writer's open block in parts, as the writer's position moves, and each chunk must reach the decoder once (it keeps which headers printed, and the newest uptime). A chunk that ends at or before `from` was decoded with an earlier part, and is passed over whole: nothing printed, nothing remembered, nothing counted. One that only starts before it (a position no writer publishes, from a damaged hint) is decoded again: a duplicate at worst, never a line left out. A torn chunk before `from` was judged with the part that held it.

## `recorder::fs`

_Tested by its own `mod tests`._

The SD card as the ring_saver uses it (doc/specs/avionics-port.md "The ring_saver", "SD and flash file layout"):
the card's root, `/sdcard` on the board (the first partition), holding one directory per session and the global
index, where [`crate::layout`] places them. The saver owns every file of a session's `recordings/` and the global
index, so nothing else appends there while it runs.

[`Fs`] is the few calls the saver makes, each naming its file by a path relative to the card's root with `/`
between the parts (`000123/recordings/000123_health.csv`, `sessions.csv`). It exists so that the host tests can
cut the card's power at any call; [`StdFs`] serves the board (IDF's FatFs VFS under `std::fs`) and the host tests
alike.

What the saver relies on, as FatFs gives it: an append lands at the file's end, and the bytes written since the
last `fsync` may be lost at a cut, whole or in part, never the ones before it. So what a file holds after a cut or
a failed call is what it held before, then a prefix of what was appended since: the saver resumes past it.

### `pub trait Fs`

The calls the ring_saver makes on the card. A file opened for appending is closed by dropping it.

- `fn create_dir_all(&mut self, directory: &str) -> Result<(), Self::Error>` — Creates `directory` and every missing parent; an existing one is fine.
- `fn tail(&mut self, file: &str, buffer: &mut [u8]) -> Result<u64, Self::Error>` — Reads the end of `file` into `buffer`: its last `buffer.len()` bytes, or all of them when it is shorter. Returns its size in bytes, 0 when it does not exist, so `buffer[..size.min(buffer.len())]` holds its end.
- `fn open_append(&mut self, file: &str) -> Result<Self::File, Self::Error>` — Opens `file` for appending, creating it when it does not exist.
- `fn write(&mut self, file: &mut Self::File, bytes: &[u8]) -> Result<(), Self::Error>` — Appends all of `bytes` to `file`.
- `fn fsync(&mut self, file: &mut Self::File) -> Result<(), Self::Error>` — Makes everything appended to `file` so far durable: a cut no longer loses it.

### `pub struct StdFs`

The card through `std::fs`, under a root directory: the board's `/sdcard`, or a host test's temporary directory.

- `fn new(root: impl Into<PathBuf>) -> StdFs` — The card whose root is `root`. Nothing is touched until the first call.

## `recorder::layout`

_Tested by its own `mod tests`._

The card's layout (doc/specs/avionics-port.md "SD and flash file layout", the session object): the ONE place
that maps (session, kind) to a directory, so no consumer builds a path of its own. Every path is relative to the
card's root (`/sdcard` on the board, the first partition) with `/` between its parts:

```text
sessions.csv                     GLOBAL_INDEX: every boot's session.csv rows (wrapped lines, no routing)
<session>/recordings/            RECORDINGS: the ring_saver's files of the session
  <session>_<stream>.csv  recorder.log  session.csv
<session>/videos/                VIDEOS: the camera's files of the session
<session>/<kind>/                any later kind, beside recordings/
videos/<session>/                a two-partition card's videos: the second partition's mount point
_<label>/recordings/             an earlier boot's RESERVED label, escaped (see below)
```

Session first: each session keeps all its data in its own directory, so FAT never holds many files in one
directory, a damaged directory costs one session, not the collection, and a session is pulled, listed and removed
as one. One partition is the layout (the operator, 2026-10-11: the HITL A/B showed two partitions no clearly better
on I/O or latency, and one is the more space-efficient, no fixed split). A two-partition card still records: its
videos go to its second partition, mounted at `videos/` ([`videos_partition_directory`]).

**A session may not take a name the card's root holds for something else** ([`RESERVED`], in any case: FAT folds
case): `videos`, or its recordings would land in the videos directory, on the videos partition; and the M4
firmware's `recordings` and `video`, which the tools still read on a card that firmware wrote (a session
`recordings` would be one directory in both layouts). [`is_reserved`] says so where the label is chosen: the
firmware's session prefix falls back to the boot id with a warning. A label is checked only there, for the running
boot; the ring_saver also files earlier boots' blocks by their header's label, and the M4 firmware allowed these
names, so the layout itself files a reserved session under `_<label>` (`_videos/recordings/`). No session this
firmware names begins with `_` (a label is letters, digits and `-`, a boot id digits, the dated form begins with
a digit), so that directory is no other session's, and nothing of a session ever lands on the videos partition.

Pure and host-tested: the saver routes every line through [`file_path`], the camera asks [`directory`] for its
directory, and the host tests and tools read the card by the same names.

### `pub fn directory(session: &str, kind: &str) -> String`

The directory of `session`'s `kind`, relative to the card's root: `<session>/<kind>`, [`RECORDINGS`], [`VIDEOS`]
and any later kind alike. A reserved session ([`is_reserved`], an earlier boot's label) is `_<session>`:
`_videos/recordings`.

Args:
* `session` - the session label (the boot id `%06u`, a `recorder.session` label, or the dated form).
* `kind` - [`RECORDINGS`], [`VIDEOS`], or a later kind's name.

Returns the directory, without a trailing `/`.

### `pub fn file_path(path: &mut String, session: &str, kind: &str, name: &str)`

Builds in `path` (emptied first) the path of the file `name` in `session`'s `kind` directory: [`directory`], then
`/<name>`. The ring_saver builds every line's path in one reused String this way, with no allocation once it has
grown.

Args:
* `path` - where the path is built; what it held is dropped.
* `session` - the session label.
* `kind` - [`RECORDINGS`], [`VIDEOS`], or a later kind's name.
* `name` - the file's name in that directory.

Returns nothing: the path is in `path`.

### `pub fn videos_partition_directory(session: &str) -> String`

A two-partition card's directory for `session`'s videos, relative to the card's root: `videos/<session>`, on the
second partition (mounted at `videos/`). The one place a session's data leaves its directory, since the card's
partitions are its own; a one-partition card keeps them in [`directory`]`(session, VIDEOS)`.

### `pub fn is_reserved(label: &str) -> bool`

Whether `label` may not name a session: it is one of [`RESERVED`] in any case. FAT folds case, so `Videos` would
share the videos directory on a one-partition card, and on a two-partition card make a directory beside the mount
point that the board's HTTP listing names twice.

### Constants

- `const GLOBAL_INDEX: &str = "sessions.csv"` — The global index at the card's root: every boot's session.csv rows, as recorderd kept the Luckfox's `/userdata/recordings/session.csv`. Its lines carry the routing `session.csv` (recorder_wire's `SESSION_INDEX`): only the file's name differs, so it never reads as one session's index.
- `const RECORDINGS: &str = "recordings"` — The ring_saver's kind: `<session>/recordings`, which holds the session's streams, recorder.log and session.csv.
- `const VIDEOS: &str = "videos"` — The camera's kind: `<session>/videos`. Also the name of a two-partition card's second partition's mount point under the card's root ([`videos_partition_directory`]), so no session may take it ([`is_reserved`]).
- `const RESERVED: [&str; 3] = [VIDEOS, "recordings", "video"]` — The names at the card's root that are no session's: [`VIDEOS`], the videos partition's mount point, and the M4 firmware's root directories `recordings` and `video` (its `recordings/<session>/`, `video/<session>/`), which tools/board_pull.sh still reads on a card that firmware wrote. Compared in any case ([`is_reserved`]).
- `const RESERVED_ESCAPE: char = '_'` — What a reserved session's directory begins with, before its label ([`directory`]): a byte no session this firmware names begins with.

## `recorder::published`

_Tested by its own `mod tests`._

What the ring tasks publish for other threads, with no 64-bit atomic (the P4 has none):
* [`PublishedPosition`], a ring position: the end of what is programmed (`writer_position`,
  doc/specs/avionics-port.md "The ring_writer", step 6), which the ring_saver never reads past, and where the
  saver goes on (`saver_position`, "The ring_saver"), which the eraser erases behind. A block sequence and an
  offset that must be read as a pair, so a seqlock over AtomicU32, in its two-copy "latch" form so that a reader
  never waits for the task that stores it;
* [`PublishedStats`], the counters and ring figures `stats`/`inspect` report. Each counter is an AtomicU32 read
  on its own, since a report may mix two steps' values without harm; the ring figures are copied whole.

### `pub struct Stats`

The recorder's health: what `stats` and the status line report.

## `recorder::record`

_Tested by its own `mod tests`._

The record (doc/specs/avionics-port.md "Producer side", the record format table): what a producer queues
instead of text, and what the ring_writer copies, verbatim, into a flash chunk. The same bytes sit in a
producer's PSRAM ring, in flash, and in a CC tee; only the ring_saver and the `cc` executor print them, so
`core::fmt` (53 us for a 20-field float row) stays off the flight core and off the flash path.

Little-endian, 4-byte aligned:

| off | size | field |
|---|---|---|
| 0 | u16 | length: the whole record with this header, its name and its padding, a multiple of 4. 0 is a PSRAM ring's wrap marker and never reaches flash |
| 2 | u8 | kind: 1 HEADER, 2 ROW, 3 LOG |
| 3 | u8 | flags: bit0 durable (its producer woke the writer, so it skipped the tick) |
| 4 | u8 | name length |
| 5 | u8 | count: a ROW's cells, else 0 |
| 6 | u16 | reserved, 0 |
| 8 | u64 | uptime_us (esp_timer, unwrapped) |
| 16 | ... | the destination file name, then the body |

The destination file name is the stream's file without the session prefix (`imu_lsm6dso32.csv`), the bare
`session.csv`, or `recorder.log` for every LOG: the block header names the session, so a record never does.
The body:
* HEADER: the stream's column list, `uptime;<field>;...`, so the record reads as `<file>@<columns>` and every
  block reads without the registry, even under a later firmware;
* ROW: the cells, a tag byte then its value: 0 None, 1 I32, 2 I64, 3 F32, 4 Bool (one byte), 5 E7 (an i32 of
  1e-7 degrees), 6 Text (a length byte, then the bytes);
* LOG: a descriptor length byte, the descriptor, then the message.

The column list and the LOG message run to the record's end, so their zero padding is stripped when read. To
keep that exact, a producer copies text with its control bytes made spaces (NUL among them, and CR and LF,
which would split a line), and a Text cell with its `;` made a space too, which would split a cell: the record
never holds what the printed line may not. A Text cell and a descriptor are cut to 255 bytes and a message to
1 KiB, each on a character boundary, so every text reads back as valid UTF-8.

A record never straddles chunks, so the longest is a chunk's payload, 4032 bytes ([`LONGEST`]).

**Versioned with the chunks.** A block of another layout version is never decoded by this build
(flash_ring::header::VERSION covers the chunk and the record formats alike), so any change here bumps that
version. The test `the_record_format_moves_only_with_the_layout_version` pins one record of each kind, byte
for byte, to it.

## `recorder::recorder`

_Tested by its own `mod tests`._

The Recorder (src/glider recorder.py `Recorder`; doc/specs/avionics-port.md "Producer side"): the state every
thread shares, built once in bring-up before any thread spawns and leaked to `&'static` on the board.

Producers never format text and never take a lock on their path:
* a producing thread [`Recorder::attach`]es itself to its own PSRAM ring, once (control, i2c0, i2c1, gnss,
  supervisor, executor). From then on its [`crate::Telemetry`] rows and [`Recorder::log`] lines go into that
  ring alone: a single-producer, single-consumer byte ring, with the ring_writer the one consumer;
* any other thread (an unregistered one, IDF's vprintf hook) shares the `misc` ring, behind a `try_lock`:
  contention drops the record and counts it, it never waits.

A thread finds its ring through a thread-local binding set by `attach`, so a log line reaches the right ring
from anywhere (the `log` crate bridge has no handle to pass). The binding names the Recorder by a process-
unique id, so a thread attached to one Recorder writes into the `misc` ring of another.

A durable record (a durable stream's, a session.csv row) wakes the writer through [`Wake`] as soon as it is
queued, so it skips the writer's ~10 ms tick (spec "The ring_writer": a durable record is at risk for at most
~35 ms, not ~45).

The error policy is plan B's: telemetry is evidence and raises ([`Overflow`] when its ring is full, counted
in `tlm.dropped`), a log line is a convenience and is dropped with a count (`log.dropped`).

### `pub struct Overflow`

A telemetry record that could not be queued: its producer's ring is full (or, for the `misc` ring, busy), or
the row is longer than a flash chunk holds (4032 bytes) or has more than 255 cells. Counted in `tlm.dropped`.

### `pub struct Identity`

Who this boot is, for the session.csv rows and the status line.

### `pub struct Settings`

The recorder's config (`recorder.*`), resolved by bring-up.

### `pub struct Inspection<'r>`

What `inspect` reports (plan B `Recorder.inspect`): the session, the status line's period, and the PSRAM the
producer rings hold, which replace plan B's `tlm_capacity`, `log_capacity` and `cell_size`.

### `pub struct Buffers`

The PSRAM the rings and tees live in. The firmware allocates each buffer once (`heap_caps_malloc`, SPIRAM)
and never frees it, so it hands them over as `&'static mut`.

### `pub type Clock = Box<dyn Fn() -> u64 + Send + Sync>`

The uptime in microseconds (`esp_timer_get_time` on the board): what a log line and a session.csv time-set
row are stamped with.

### `pub type Wake = Box<dyn Fn() + Send + Sync>`

Wakes the writer thread at once (`xTaskNotifyGive` on the board): called right after a durable record is
queued, from the producer's thread. It must not block.

### `pub enum AttachError`

Why a thread cannot attach to a ring.

### `pub struct Attachment<'r>`

A thread's hold on its ring. While it lives, the thread's telemetry and log records go into that ring; when
it drops, the ring is free for another thread. It stays on the thread that attached (it is not Send).

### `pub struct Recorder`

The recorder's shared state. See the module notes.

- `fn new(registry: Registry, identity: Identity, settings: Settings, flight: bool, buffers: Buffers, clock: Clock, wake: Wake) -> Recorder` — The Recorder over `buffers`, with its streams, boot identity and settings, the uptime `clock` and the writer's `wake`. Nothing runs until a [`crate::Writer`] takes it; records queue in the rings meanwhile.
- `fn attach(&self, ring: usize) -> Result<Attachment<'_>, AttachError>` — Attach the calling thread to producer ring `ring`, as its one producer, until the returned [`Attachment`] drops.
- `fn log(&self, descriptor: &str, message: &str) -> bool` — A best-effort recorder.log line, `<uptime_us> <descriptor> :: <message>` (plan B `Recorder.log`): stamped now, the descriptor cut to 255 bytes and the message to 1 KiB, queued as a LOG record.
- `fn index_session(&self, epoch_s: i64, utc_offset: Option<i32>, source: &str, cc_position: Option<(i32, i32)>) -> Result<(), Overflow>` — Append a session.csv row for a time set (plan B `Recorder.index_session`): this boot tied to the wall clock, stamped now, queued as a durable ROW to `session.csv` (a time set wakes the writer). The writer writes the `boot` and `anchor` rows itself, each after the HEADER record; a time set always comes after the boot row, so it needs no header of its own.
- `fn bind_config_id(&self, config_id: &str) -> bool` — Binds the config id the session.csv rows carry (empty until then), once: the bring-up knows it only after the layout has rewritten the config, and the Recorder opened before that so the layout's lines are kept. Returns false, keeping the first, when one was bound already.
- `fn drain_log_tee(&self, duration_ms: i64) -> TeeBatch` — The CC `log [ms]` command: the log lines buffered since the last call, keeping the window open `duration_ms` more (0 or less stops it).
- `fn drain_telemetry_tee(&self, duration_ms: i64) -> TeeBatch` — The CC `tlm [ms]` command: the telemetry lines (`@<routing>@<row>`) buffered since the last call, keeping the window open `duration_ms` more (0 or less stops it).
- `fn stats(&self) -> Stats` — The recorder's health for `stats`: what the writer published last, and the drop counts.
- `fn writer_position(&self) -> Option<Position>` — `writer_position`: the end of what the writer has programmed (block sequence, 8-aligned offset), for the ring_saver, which never reads beyond it. None until the writer's first step. Before this boot's first chunk it is offset 0 of the block this boot opens first, so every older block is readable whole.
- `fn saver_position(&self) -> Option<Position>` — `saver_position`: where the ring_saver goes on (block sequence, offset), every line before it on the SD card and fsynced (doc/specs/avionics-port.md "The ring_saver"). Blocks before its block are saved, and the eraser erases them 100-200 ms later. None while no saver runs (no card), when every closed block is unsaved.
- `fn relay_position(&self) -> Option<Position>` — `relay_position`: how far the uart_saver has sent (M19, [`crate::Relay`]); None while no relay runs.
- `fn erase_horizon(&self) -> Option<Position>` — What the eraser may take as done: the saver's position, held back to the relay's while a relay runs behind it, so a block is erased only once it is on the card AND out of the UART. None while no saver runs.
- `fn set_flight(&self, flight: bool)` — Tell the recorder whether the stage is in flight: true from BOOSTING, false from DONE, and false when the warm start of a reboot in flight is refused (doc/specs/avionics-port.md "Ring geometry and block life cycle"). The writer applies a rise before each chunk it programs and a fall once everything queued before it is programmed, and a change closes its open block, so every record queued in flight lies in a block whose header carries the flag (newest wins gives flight blocks up last).
- `fn running(&self) -> bool` — Whether a writer runs: until one does, [`crate::Telemetry::due`] is false, and records only queue.
- `fn registry(&self) -> &Registry` — The streams.
- `fn settings(&self) -> Settings` — The recorder config, with `stats_ms` as it is now.
- `fn set_stats_ms(&self, stats_ms: u32) -> bool` — The operator's `update recorder {"stats_ms": n}` (plan B `Recorder.update`): a new status-line period in ms, which the writer takes at its next step. Only a positive value that differs from the current one counts.
- `fn inspect(&self) -> Inspection<'_>` — The `inspect` snapshot: see [`Inspection`].

## `recorder::registry`

_Tested by its own `mod tests`._

The stream registry (doc/specs/avionics-port.md "Producer side"): every telemetry file a boot writes, with
its destination file name, its column list and its decimation. Bring-up builds it before any thread spawns
and hands it to the [`crate::Recorder`], which never changes it again: a producer names its stream by a 16-bit
id, and the record it queues carries the file name, never the session (the block header names that).

Stream 0 is the shared session index, the bare `session.csv`: the ring_saver files its rows both in the
session's directory and in the global index every boot appends to. Every other stream's file prints as
`<session>_<file>`.

The registry keeps each stream's column list, `uptime;<field>;...`, built once. A HEADER record carries it, so
that every flash block reads without the registry, and the writer repeats it about once per MiB by copying
these bytes: nobody formats a header on the flight path.

A stream whose records could never reach the SD whole is refused here, at bring-up, rather than losing every
row later: a file name the ring_saver would not file (`[A-Za-z0-9._-]{3,64}`: its rows would land in
recorder.log), a field that would break the column list, more fields than a row counts, or a HEADER record
longer than a chunk holds.

### `pub struct StreamId(pub(crate) u16)`

A stream's id: its index in the registry.

### `pub struct Stream`

One telemetry stream.

- `fn file(&self) -> &str` — The destination file name its records carry.
- `fn columns(&self) -> &[u8]` — Its column list, `uptime;<field>;...`, without a line end.
- `fn decimate_us(&self) -> i32` — Its decimation setting: own window (> 0), the global one (0), or none (< 0).
- `fn durable(&self) -> bool` — Whether its records wake the writer.

### `pub enum RegistryError`

Why a stream cannot be registered.

### `pub struct Registry`

The streams of one boot.

- `fn new() -> Registry` — A registry holding only stream 0, the session index `session.csv`, durable and never decimated.
- `fn add(&mut self, file: &str, fields: &[&str], decimate_us: i32, durable: bool) -> Result<StreamId, RegistryError>` — Registers the stream `file` with these fields (its columns after `uptime`).
- `fn stream(&self, id: StreamId) -> Option<&Stream>` — The stream `id` names, or None when this registry has no such stream.

## `recorder::relay`

The uart_saver's walk (M19 stage 2, doc/specs/avionics-port.md M19): a [`Walk`] of the flash ring with a cursor of
its own, so the recorder's lines go out of the UART whether a card is mounted or not, and a reset costs the
receiver nothing. The firmware's uart_saver thread calls [`Relay::step`] again and again, writes each part's lines
to its UART, and once they have left it calls [`Relay::sent`]: the position published there is how far the
receiver has been sent, which the firmware keeps as the `uart` NVS hint and the next boot starts from. A part sent
again after a reset is the receiver's to drop (its files hold each line once: no line is newer than its file's
last).

Stage 1 teed the ring_saver's lines into a queue instead. The saver tees a part when it stages it and the card has
it a commit later, so a part committed but still queued at a reset was never sent, and never sent again by the
next boot, which starts from the card's position (1.5 s of one session on the C6's card, 2026-10-11); and with no
card nothing was sent at all.

**Each line names its session** ([`tee`]): a log line goes routed `<session>_recorder.log`, a session.csv line
`<session>_session.csv`, the body as the saver files it, its checks made under its own routing.

**The eraser keeps what the relay has not sent:** [`Recorder::erase_horizon`] is the saver's position held back to
the relay's, so a saved block is erased only once the UART has sent it too.

**Recency first:** a relay more than its `behind` blocks behind the writer jumps to the oldest block within it,
the blocks it passed counted ([`Relay::skipped`]): the last seconds before an impact matter most, and a UART slower
than the ring (a backlog after a long reset, a slower link) must not spend them on old ones.

### `pub struct Relay<'r, 't, F: Flash>`

The uart_saver's walk: see the module notes.

- `fn new(recorder: &'r Recorder, flash: F, table: &'t BlockTable, start: Position, behind: u32) -> Relay<'r, 't, F>` — A relay from `start` (the `uart` hint through [`BlockTable::saver_start`], or this boot's first block), published as sent at once: nothing before it is the relay's to send.
- `fn step(&mut self, mut send: impl FnMut(&[u8])) -> Result<Option<Position>, F::Error>` — Reads and decodes the next part of the ring, handing each line, routed under its session, to `send`.
- `fn sent(&self, position: Position)` — Publishes `position` as sent: every line before it has left the UART. The eraser may now take the blocks before it (and before the saver's position) as done.
- `fn cursor(&self) -> Position` — The next byte to read.
- `fn skipped(&self) -> u32` — The blocks jumped over to stay within `behind` of the writer.

### Constants

- `const BEHIND_BLOCKS: u32 = 64` — How far behind the writer the firmware's relay may fall before it jumps forward, in 4 KiB blocks: ~10 s of the flight's ~26 KB/s of records.

## `recorder::rows`

_Tested by its own `mod tests`._

The error policy's latches (doc/skills.md "Error policy"): a failure that comes back every tick is logged once a
run, the first of the run, and the run ends at the next success. [`LogOnce`] is the latch itself; [`Rows`] is a
stream that uses it for its refused rows. The recorder counts every refused row (`tlm.dropped`); the log says only
the first of a run.

### `pub struct LogOnce`

A failure's log latch: open until the first failure of a run, shut until the run ends.

- `fn first(&mut self) -> bool` — A failure: true when it is the first of its run, the one to log.
- `fn clear(&mut self)` — A success: the run of failures ended, and the next failure logs again.
- `fn on(&self) -> bool` — Whether a run of failures is on.

### `pub struct Rows<'r>`

A stream as a run records it: its [`Telemetry`] (None when the board records nothing, or the registry has no such
stream) and the latch of its refused rows, logged as `<name> :: <label> <overflow>`.

- `fn new(name: &'static str, label: &'static str, telemetry: Option<Telemetry<'r>>) -> Rows<'r>` — The rows of the run `name`, its stream called `label` in the log; a refusal logs again after an accepted row.
- `fn logged_once(mut self) -> Rows<'r>` — As [`Rows::new`], but only the first refusal ever is logged.
- `fn recording(&self) -> bool` — Whether the stream records at all.
- `fn push(&mut self, now_us: u64, row: &[Cell]) -> bool` — Queue `row` at `now_us` (nothing when the stream does not record). Returns whether the row was refused and the refusal is the one logged.

## `recorder::saver`

The ring_saver (doc/specs/avionics-port.md "The ring_saver", "Duplicates", "SD and flash file layout"), minus its
thread: the firmware's saver thread (core 0, priority 4, below the video writer) calls [`Saver::step`] again and
again. It copies the flash ring to the SD card as text, in `block_sequence` order, and publishes how far it got
(`saver_position`), behind which the eraser erases.

**Why here and not in crates/flash_ring**, where the spec's M4 scope places the walk, the decode and the batches:
the decode is this crate's [`Decoder`] over this crate's record codec, and flash_ring cannot depend on this crate
(this crate depends on it), nor on `std::fs`. So flash_ring gives the table query the walk needs
([`BlockTable::next_block`]) and keeps the ring's own rules; the walk, the decode and the batches live here.

Two parts, so that the uart_saver (M19, [`crate::Relay`]) shares the decode without touching the SD path:
* [`Walk`]: a cursor over the ring and the decoder. [`Walk::peek`] reads the next unread part of the ring, one
  4 KiB read of one block (of the writer's open block only up to the writer's published position, never past
  it), and [`Walk::decode`] turns it into wire lines. With no card mounted, a walk of its own, with its own
  cursor, can feed the uart_saver, and `saver_position` keeps meaning "on the SD and fsynced";
* [`Saver`]: a walk, the [`Fs`] of the card, and the batches. Each part is decoded into a staging area first,
  then joins the batch: the point where M19's handoff takes each block's lines, before any SD write. Staging
  touches no card (every card decision is made when the batch goes to the card), and a part is staged once per
  run, a card error included, so M19 would get each line once and an SD stall never repeats a decode.

**Routing** (recorderd's, spec "The ring_saver"), into the recordings directory of the block header's session
that [`crate::layout`] names, `<session>/recordings`: a stream line to `<session>/recordings/<session>_<file>`, a
session.csv line to `<session>/recordings/session.csv` and to the global index at the card's root
(`sessions.csv`, [`layout::GLOBAL_INDEX`]), a log line (and a record the saver would not file) to
`<session>/recordings/recorder.log`. The bytes are the wire line after its routing, `{OPEN};<payload>;<CLOSE>\n`,
as recorderd appended them; the global index's lines keep the routing `session.csv`, the file's name aside.

**Batches** are at most [`BATCH_BLOCKS`] blocks or [`BATCH_MS`] of collecting, whichever comes first, of one boot
(a stricter rule than the spec's one session: two boots under one label are two batches), touching at most
[`BATCH_FILES`] files: a block that would pass that starts the next batch (a single block that names more files
alone goes as a batch of its own). Then, for each file in turn: open it, append its lines in writes of up to
[`WRITE_BYTES`], fsync, close; and only once every file is fsynced is `saver_position` published past the batch.
The eraser may then erase those blocks 100-200 ms later. There is no journal: a cut anywhere in a batch leaves
the published position before it, and the next boot saves the batch again, duplicates (spec "Duplicates") that
the tools drop.

**A card error** keeps the batch and the walk: the files already fsynced stay done, and every later step goes on
with the file that failed, before anything else. It looks at that file's size again and appends only what did
not land, since what a file holds after a failed call is a prefix of what was appended to it ([`Fs`]). So a card
that refuses one file for a while (FatFs's FR_LOCKED while an HTTP pull reads it) or every call (a dead card)
costs a card call or two a step: nothing is read, decoded or appended again, and nothing new joins meanwhile.

**A run** is the saver's life from [`Saver::new`]: one per boot and card (a card put in later gets a saver of
its own). Before its first append to a file, and again after the walk has left that file's boot behind, the run
looks at the file's end (its last [`TAIL_BYTES`]): a line a cut left torn gets its `\n` first, so the fragment
stays a line of its own (damage the tools count) and never merges with a line saved again.

**Headers after a reboot.** The tools can never drop a header line as a duplicate (it carries no uptime), and a
file's second header reads as two boots appended into it (`flight_telemetry` flags the splice). A run that
starts behind what the card holds (a reboot resumes at the `cfg` NVS hint, written about once a second; a lost
hint starts at the oldest block) meets headers the card holds already, as the writer's repeats or among the
lines it saves again, and must not print them again. Each file is judged on its own, by what it held when the
run first looked at it (whole lines: a fragment a cut left alone is none):
* under a label of one boot (the boot id `%06u`, or the dated form; never a `recorder.session` label) a file that
  held a line holds that boot's header, whatever the hint;
* the global index holds every boot: it holds a boot's header when its last row names that boot (a row carries
  the boot id and the session). A hint stale across a boot boundary prints the earlier boot's header there once
  more, which the tools leave out of session.csv anyway;
* under a `recorder.session` label reused across boots, a file holds several boots' lines and no stream line
  names its boot. A header of the boot the run began in is dropped from a file that held lines; any later
  boot's prints, so the splice still shows. That is exact when the run begins in the label's last boot on the
  card, which the firmware secures by writing [`Saved::position`] to the `cfg` NVS hint after every committed
  batch, before the next step (about once a second when caught up). One case stays open: a file that held an
  earlier boot's lines, while every line of the run's first boot for it lies past the published position (a cut
  in that boot's first batch, its first second when caught up, or in the batch where the stream first wrote),
  misses that boot's header. The lines are all there; only that file's splice mark is missing.

**Never a line left out.** The walk moves past a block only once it is decoded, or once it is gone from the ring:
newest wins erased it unsaved, and the eraser logged that loss. A decoded part stays in the batch, or staged
beside it, until the card has it. A flash read that fails is tried again at the next step, from the same place.

**Memory** stays that of one boot: the decoder keeps the boot it is in, and the run forgets a boot's files once
the walk has left it (a boot's blocks are numbered one after another), so a backlog of many short boots, a
reset loop on the bench, costs no more than one.

### `pub enum Ahead`

What lies ahead of a walk's cursor.

### `pub struct Walk<'r, 't, F: Flash>`

A cursor over the ring, in `block_sequence` order, and the decoder that turns each part it reads into wire lines.
See the module notes.

- `fn new(recorder: &'r Recorder, flash: F, table: &'t BlockTable, start: Position) -> Walk<'r, 't, F>` — A walk from `start`, with a decoder that has printed nothing yet.
- `fn cursor(&self) -> Position` — The next byte to decode.
- `fn peek(&mut self) -> Result<Ahead, F::Error>` — Reads the next unread part of the ring: the block the cursor stands in, from its offset 0 up to the block's end once closed, or up to the writer's published position in the writer's own block, with one flash read. A block that is gone (newest wins erased it, or it was never opened) is passed over to the next one. Nothing past the writer's published position is ever read.
- `fn decode(&mut self, each: impl FnMut(DecodedLine<'_>)) -> Option<BlockSummary>` — Decodes the part the last [`Walk::peek`] read, from the cursor's offset on (each chunk once, across the parts of the writer's open block), calling `each` with every line; then the cursor moves past the part.

### `pub struct Saved`

What one committed batch put on the card.

### `pub enum SaveError<FlashError, CardError>`

Why a step stopped.

### `pub struct Saver<'r, 't, F: Flash, S: Fs>`

The ring_saver: a [`Walk`] from the published position, the card, and the batch being collected. See the module
notes.

- `fn new(recorder: &'r Recorder, flash: F, table: &'t BlockTable, card: S, start: Position) -> Saver<'r, 't, F, S>` — The saver of this boot, starting a run at `start`, which it publishes as `saver_position` at once: every block before it is on this card (a trusted hint), or there is none (a new card starts at the oldest block).
- `fn step(&mut self, now_ms: u32) -> Result<Option<Saved>, SaveError<F::Error, S::Error>>` — One step: collects what the walk reads into the batch, and puts the batch on the card once it is due (see the module notes), publishing `saver_position` past it. A batch the card failed goes on first, and nothing is read until it is on the card. Commits one batch at most; call it again at once while it commits full batches (a backlog), else every 100 ms or so.
- `fn position(&self) -> Position` — The published `saver_position`: everything before it is on the card and fsynced.

### Constants

- `const BATCH_BLOCKS: u32 = 64` — The most blocks one batch holds (256 KiB of flash): a part of a block read earlier counts with it.
- `const BATCH_FILES: usize = 32` — The most files one batch appends to, the global index included. With per-session directories that is every stream of a boot, its recorder.log and both indexes; the board's `max_files` (40) also serves video and HTTP.
- `const BATCH_MS: u32 = 1000` — How long a batch collects before it goes to the card, in ms. A saver that keeps up with the writer therefore commits once a second; one behind commits as soon as a batch holds [`BATCH_BLOCKS`].
- `const WRITE_BYTES: usize = 32 << 10` — The most bytes one write call appends.
- `const TAIL_BYTES: usize = 1024` — How much of a file's end the saver reads when it looks at the file: more than any header or session.csv line, so a fragment a cut left alone is told from a file that holds a whole line, and the global index's last row is read whole.

## `recorder::tee`

_Tested by its own `mod tests`._

A CC tee (doc/specs/avionics-port.md "CC tees and the HTTP exits"; src/glider recorder.py `_TeeSink`): a
poll-model copy of the log or the telemetry records for the CC `log [ms]` and `tlm [ms]` commands.

The ring_writer copies each record in, verbatim and never as text, while the operator's window is open; the
command's executor takes the batch out and only then prints it (invariant 4: text is made off the flight
path). `drain(ms)` returns what was buffered since the last call and keeps the window open `ms` more (0 or
less closes it). A window nobody renews lapses: the writer marks it so at its next record, and the next drain
throws the stale batch away, so a lost link never fills the buffer with records from long ago. Best effort: a
full buffer drops the record and counts it, never blocking the writer.

The writer and the drain run on their own threads, and the writer preempts the executor, so a drain never closes
the window while it works. It reads where the writer has stored up to, then moves the window in one swap (the
new deadline, or closed), and takes only the records before that point. A record the writer checked against the
new deadline is stored past it and waits for the next drain; so is one it checked just before the swap and
stored just after it, which that next drain prints as long as the window it was checked in was open. Holding
the window closed for a whole drain (lock, walk, formatting up to 256 KiB) would make the writer pass over every
record meanwhile, uncounted, and the CC reads `dropped` to tell an incomplete window from a quiet sensor.

A drain also throws away what it finds in a closed tee. The writer checks the window and then copies its
record, so a record whose check came just before a closing drain lands after that drain's batch ended: plan B,
single-threaded, never had such a line, and returned to the next session hours later it would read as fresh.

The deadline is a wrapping u32 of uptime milliseconds (the P4 has no 64-bit atomics), as the M1 firmware
tee keeps it: 0 means closed and 1 lapsed, so an open deadline is at least 2. The records go through a byte
ring in PSRAM: the writer is its one producer, and the drain its one consumer, serialised by a lock that only
the executor side takes.

### `pub struct TeeBatch`

What one drain returns.

## `recorder::telemetry`

A typed telemetry stream (src/glider recorder.py `Telemetry`; doc/specs/avionics-port.md "Producer side"),
owned by its producer thread.

The first push queues the stream's HEADER record and then its row, whatever the decimation. After that a row
goes out only when the stream's window has passed since the last row that did: the stream's own
`decimate_us` when positive, else the global `recorder.telemetry_ms` (0 = every row), never for a negative
one. The global is read at each use, never latched: plan B once folded it in at construction, before the
recorder had read its config, and capped a 100 Hz accelerometer at 50 Hz while the config said uncapped.

A hot producer asks [`Telemetry::due`] first and builds its row only then. Time is an argument (the
producer's own `esp_timer` read), never a clock read here. A push copies typed cells into the producer's
PSRAM ring and formats nothing.

### `pub struct Telemetry<'r>`

One stream's producer state.

- `fn new(recorder: &'r Recorder, stream: StreamId) -> Option<Telemetry<'r>>` — The producer side of `stream`; None when the Recorder's registry has no such stream.
- `fn due(&self, now_us: u64) -> bool` — Whether a push at `now_us` would queue a row, so a hot producer builds its row only then. False while no writer runs: with nowhere to go, a stream is not due (plan B's rule, which kept a recorder-less bench from paying for a raised and caught error every tick).
- `fn push(&mut self, now_us: u64, row: &[Cell]) -> Result<(), Overflow>` — Queue `row` at `now_us`, the stream's HEADER first on the first push. A decimated row is skipped, Ok.

## `recorder::writer`

The ring_writer (doc/specs/avionics-port.md "The ring_writer"), minus its thread: the firmware's writer thread
(core 0, priority 7) calls [`Writer::step`] on its ~10 ms tick ([`TICK_US`]), and at once when a producer's
durable record wakes it ([`crate::Wake`]). Each step:
1. programs first a chunk the flash ring refused last time (see below);
2. writes this boot's session.csv `boot` row (its HEADER record, then the ROW) before anything else, and the
   `anchor` row likewise a minute later ([`ANCHOR_AFTER_US`]); and about once per MiB written
   ([`HEADER_REPEAT_BYTES`]) copies the HEADER record of every stream this boot has started again, so a
   session whose first blocks newest wins erased unsaved still gets its headers within the next MiB;
3. drains the producer rings: telemetry records (HEADER, ROW) at once, LOG records only once they are
   [`MERGE_HORIZON_US`] old, then merged across the rings by uptime, so recorder.log stays in uptime order;
4. copies whole records, verbatim, into a chunk while they fit the open block, so a block fills before it
   closes: a record never straddles chunks or blocks. A record that no longer fits programs the chunk, and the
   next chunk goes into the next block;
5. programs each chunk with one call ([`flash_ring::Ring::append`]). Then it copies each record into the CC tee
   whose window is open, releases the records from their rings, and publishes `writer_position`;
6. logs the status line every `stats_ms`, a LOG record of its own merged by uptime with the producers': plan
   B's counters, plus the flash ring's figures against the ring_saver's published position, both positions and
   the write rate measured since the last status line, which `stats` reports too.

The writer prints no record: the ring_saver and the `cc` executor do (invariant 4). It makes three short texts
of its own, rarely: the status message once per `stats_ms`, and the utc cell of the `boot` and `anchor` rows.

**A record leaves its ring only after its chunk is programmed** (invariant 2: "better duplicates in both than
lost data"). A ring's tail moves only after a successful program, and never past a LOG record still held for
the merge. When the flash ring refuses a chunk, the chunk is kept, whole, and programmed first at the next
step; nothing more is read meanwhile, the producer rings fill, and only then does [`crate::Telemetry::push`]
raise (and a log line drop). The flash ring refuses a chunk when:
* no erased block is left to open (`WouldBlock`): the eraser has not freed one yet;
* a program failed (`Flash`): the flash ring has moved past whatever the call left, so the retry goes at the
  next offset, and a call that had landed whole leaves the chunk in flash twice: a duplicate, which the tools
  drop (spec "Duplicates"). The torn bytes of one that had not are stepped over by the saver's walk;
* every block sequence number is spent (`Exhausted`), which only a corrupt hint or another build's blocks
  bring about: the records then wait for good, and the writer logs why once.

**Flight blocks.** The writer applies the stage the firmware last told the Recorder
([`crate::Recorder::set_flight`]), and the flash ring closes its block on a change, so that every record from
BOOSTING to DONE lies in a block with the flight flag (newest wins gives those up last):
* a rise (BOOSTING) applies before every chunk, and the flag is read after the records in it, so a record
  queued after the rise is in a flight block (one queued just before it may be too, which costs nothing);
* a fall (DONE) applies only at the start of a step, once every record queued before it is programmed: a whole
  step has drained the rings and merged the logs held for the [`MERGE_HORIZON_US`] since the fall was seen, and
  no chunk waits. Applied at once, the last LANDING rows and log lines, queued before DONE but drained after,
  would land in a pad block, which newest wins gives up first.

### `pub struct Progress`

What one step did.

### `pub struct Writer<'r, 't, F: Flash>`

The ring_writer over the flash ring of `F`. See the module notes.

- `fn new(recorder: &'r Recorder, mut ring: Ring<'t, F>) -> Option<Writer<'r, 't, F>>` — The writer of `recorder` into `ring` (after the bring-up scan). From now on the Recorder runs: [`crate::Telemetry::due`] answers, until the writer drops.
- `fn step(&mut self, now_us: u64, epoch_s: i64) -> Progress` — One pass of the loop at uptime `now_us`, with the wall clock `epoch_s` (Unix seconds) for the session.csv rows. See the module notes.
- `fn ring(&self) -> &Ring<'t, F>` — The flash ring.
- `fn into_ring(self) -> Ring<'t, F>` — The flash ring, giving the writer up (and with it the Recorder's claim).

### Constants

- `const TICK_US: u64 = 10_000` — The writer thread's tick: the firmware calls [`Writer::step`] this often, and at once on a durable record's wake. With an erase and an NVS page erase holding the flash, it bounds what a power cut loses: about 11 ms of rows typical, at most about 45 ms (spec "The ring_writer", the power-cut bound).
- `const MERGE_HORIZON_US: u64 = 20_000` — A LOG record is programmed once it is this old, so a line a slower thread queues a little later still lands in uptime order. It adds to a log line's power-cut bound (at most about 65 ms).
- `const ANCHOR_AFTER_US: u64 = 60_000_000` — The session.csv `anchor` row goes out this long after the writer starts: it lists the boot for a reader that missed its `boot` row, dated if the clock was set by then.
- `const HEADER_REPEAT_BYTES: u32 = 1 << 20` — The flash between two repeats of the started streams' HEADER records: 1 MiB, about a minute at the flight rate, counted by the writer position (block headers and the unused tails of closed blocks included), so a reader that starts anywhere finds every header within the next MiB of flash. The repeats cost one copy each, and the ring_saver prints none of them unless a file starts there or its columns change.

# `warmstart` — `src/avionics/crates/warmstart`

_Uses: `control`, `sequencer`. External: serde, serde_json._

_Integration tests: `tests/golden.rs`._

## `warmstart`

_Tested by its own `mod tests`._

In-flight reboot recovery (src/glider warmstart.py; doc/specs/coludo.md "In-flight reboot & warm start", and the
boot rules of doc/specs/avionics-port.md §4.6). A mid-air reset (a watchdog, a panic, a crash) must not turn the
glider ballistic. The checkpoint keeps a tiny CRUMB in NVS carrying the live flight state; at boot, before any
thread runs, the crumb and the physical signals decide whether the board resumes the saved stage (a warm start)
or boots cold, and the normal detectors (separation, apogee, landing) re-evaluate from there.

The crumb lives in NVS, never in a file: a file write locks the scheduler and wears the data flash, while NVS
commits to its own partition in milliseconds. It is two keys in the namespace [`NAMESPACE`], the same as plan B's,
so a crumb plan B wrote reads back here and the other way round:
 * [`CRUMB_KEY`], one JSON blob of at most [`BLOB_MAX`] bytes (about 250 in practice): full precision, no
   per-field key bookkeeping, and a new field is a member rather than an NVS schema change;
 * [`STAGE_KEY`], the stage id as an i32, written LAST. It IS the flag: 0 ([`NO_STAGE`], what an unwritten key
   reads as) means no flight was checkpointed, anything else is the stage to recover. Written last, a torn
   write leaves the previous stage, never a new stage pointing at a half-written blob; the i32 is authoritative
   over the blob's own copy.

Only a FLYING board writes a crumb: armed (a disarmed passive flight must never warm-start into an armed stage)
and launched by its own stage machine (an operator's held stage is a bench test, never a flight to resume). The
recovery identity (the launch point, the zone, the pad altitude, the pitot tare) is frozen once, at BOOSTING
entry, and rides every later crumb.

The pieces, each plan B's function as data, so the firmware's `warm.rs` and supervisor only move bytes:
 * [`Crumb`], with [`Crumb::from_json`] and [`Crumb::to_json`], and [`save`]/[`clear`]/[`load`] over the [`Nvs`]
   trait (the firmware implements it on esp-idf NVS; the tests on memory);
 * [`should_restore`], the gate, with [`ResetReason`] (the ESP-IDF reset reasons and which count as a reset);
 * [`ApplyPlan`], what a passed gate restores, in plan B's order;
 * [`restore`], plan B's boot decision end to end;
 * [`Checkpoint`], plan B's Checkpoint task as a pure schedule: when to write, and the identity it carries;
 * [`Keeper`], the schedule over NVS: a crumb only while flying, a clear when the flight ends, and a refused
   write retried until it lands.

Nothing here reads a clock: the time is an argument. A crumb's `stamp` is whole seconds since 2000-01-01 UTC,
MicroPython's esp32 epoch, so plan B's crumbs and these share one clock; the firmware subtracts
[`UNIX_SECONDS_AT_2000`] from `gettimeofday`. Metres, m/s and pascals are f32 (decision D18); the launch and
zone are `Position` integers in 1e-7 degrees, f64 only as JSON numbers.

Deliberate differences from plan B (doc/specs/avionics-port.md §4.6, M8; each one marked in the golden fixture,
but the last, which no exact fixture case can hold and the unit tests pin):
 * the operator's boot rules of 2026-10-06: any power-up after DONE is a new session in SETTING (plan B restored
   DONE after a reset within 600 s), and only a crumb in BOOSTING..LANDING warm-starts (plan B restored SETTING,
   re-armed, after a reset on the pad). No crumb is SETTING in both;
 * a crumb stage that is no flight stage is refused. Plan B passed it ('recover ?') and then raised in
   `Controller.set_stage()`, crashing the boot halfway through the restore;
 * the P4's CPU_LOCKUP (a double exception, an in-flight crash) is a reset; MicroPython maps it to 0, cold;
 * a crumb blob that is JSON but not an object reads as no crumb; plan B's load() raised TypeError there and
   crashed the boot. A value of the wrong JSON type reads as absent, where plan B computed on whatever arrived;
 * the writer never puts NaN or infinity in the blob (they become null); CPython would write NaN, which a board's
   MicroPython json cannot read back;
 * the zone's centre, which stands in for a missing launch point at the freeze, is the integer mean of the corners'
   1e-7 degree units, rounded toward zero as the mission's zone centre is. Plan B averages the float degrees, so
   when the corners' units sum to an odd number the two differ by half a unit (5.6 mm). The checkpoint fixture
   compares exactly and holds coordinates of at most 7 decimals, so it cannot carry that half unit.

The [`Keeper`] departs from plan B's Checkpoint task on purpose too, each departure pinned by the unit tests:
 * plan B saved a crumb whenever the board was armed; the keeper saves one only while armed AND launched, so an
   armed bench hold (`arm`, then `stage gliding`) writes none, and a reset never resumes it as a launched flight;
 * plan B left the last crumb in NVS after a disarm; the keeper clears it on any edge out of flying (a disarm, or
   a hold, which clears `launched`). The recovery from a false launch (disarm, `stage done`, the re-zero, `stage
   setting`) so leaves no armed BOOSTING crumb for a reset within [`MAX_AGE_S`] to warm-start on the pad. The
   cost: plan B's simulated HITL reboot, whose outage begins with a disarm, finds no crumb;
 * plan B wrote each crumb once, whatever NVS answered; the keeper retries a refused crumb or clear on every poll
   until it lands. The schedule never re-checkpoints DONE, so a lost DONE crumb left the last airborne one, and a
   reset then warm-started a landed glider;
 * plan B latched every fresh altitude in SETTING as the pad; the keeper latches one only while the elevation is
   under a ceiling (half the sequencer's `launch_alt_m`). A launch the accel trigger missed stays in SETTING
   through the baro launch dwell, and plan B latched its "pad" metres up the climb.

Not ported: the GC handling (plan B collected and disabled the GC on an airborne warm start), the log lines
(the firmware logs the [`Reason`] it gets back), and MicroPython's SOFT_RESET (an interpreter reboot without a
chip reset), which has no ESP-IDF reason.

### `pub struct NvsError(pub i32)`

A failed NVS operation: the `esp_err_t` the firmware's NVS returned (0x1102 is ESP_ERR_NVS_NOT_FOUND).

### `pub trait Nvs`

The NVS namespace as the warm start uses it: one i32 and one blob, read at boot and written by the checkpoint.

The firmware implements it over esp-idf's NVS handle; the tests over memory. A key that was never written is an
error, as esp-idf reports it.

- `fn get_i32(&mut self, key: &str) -> Result<i32, NvsError>` — The i32 under `key`.
- `fn get_blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, NvsError>` — Read the blob under `key` into `buffer` and return its length. A blob longer than the buffer is an error (ESP_ERR_NVS_INVALID_LENGTH), as esp-idf reports it.
- `fn set_blob(&mut self, key: &str, value: &[u8]) -> Result<(), NvsError>` — Store `value` as the blob under `key`.
- `fn set_i32(&mut self, key: &str, value: i32) -> Result<(), NvsError>` — Store `value` as the i32 under `key`.
- `fn commit(&mut self) -> Result<(), NvsError>` — Make the writes durable.

### `pub struct Identity`

The recovery identity: what a flight launched with, frozen at BOOSTING entry and carried by every crumb after.

These do not change through a flight, so they are captured once (the launch fix pinned at the last ground
moment) rather than sampled per checkpoint: the pitot tare in particular is a calibration taken on the pad, and
re-reading it in flight would persist whatever a disturbed sensor happened to show.

- `fn is_empty(&self) -> bool` — Nothing frozen yet (plan B's empty `_static`): the next BOOSTING entry captures afresh. The warm-start count is no frozen value: a flight that warm-started with nothing frozen still freezes at BOOSTING.

### `pub struct Crumb`

The checkpoint crumb: the live flight state plus the recovery identity, as the NVS blob carries it.

Every field may be absent, as in plan B's dict: a torn or older crumb lacks some, and the gate and the apply
plan decide what an absent one means. A field of the wrong JSON type, or a coordinate off the globe, reads as
absent.

- `fn from_json(value: &Value) -> Option<Crumb>` — The crumb a blob holds: None when the JSON is not an object (plan B raised TypeError there and crashed the boot). Every member is read on its own (see [`Crumb`]); an unknown member is ignored.
- `fn to_json(&self) -> String` — The blob text: compact JSON, well under [`BLOB_MAX`] bytes (about 250 for a full crumb).

### `pub enum ResetReason`

Why the board reset, as ESP-IDF's `esp_reset_reason()` reports it (esp_system.h `esp_reset_reason_t`, IDF 5.5;
the discriminant is the code).

Plan B saw MicroPython's `machine.reset_cause()`, which the esp32 port derives from these
(ports/esp32/modmachine.c, v1.29.0): SW, PANIC and EXT are HARD_RESET; INT_WDT, TASK_WDT and WDT are WDT_RESET;
POWERON and BROWNOUT are PWRON_RESET; DEEPSLEEP is DEEPSLEEP_RESET; everything else is 0.

- `fn from_code(code: u32) -> ResetReason` — The reason for an `esp_reset_reason_t` code; [`ResetReason::Unknown`] for a code this list does not know.
- `fn name(self) -> &'static str` — The ESP-IDF name after `ESP_RST_`, as the golden fixture and the boot log write it.
- `fn from_name(name: &str) -> Option<ResetReason>` — The reason with this [`name`](ResetReason::name), None for any other text.
- `fn is_reset(self) -> bool` — Whether this is a RESET the warm start may recover from, rather than a cold boot (§4.6).

### `pub enum Reason`

Why the gate answered as it did: plan B's reason texts, plus the ones the boot rules add. Display gives the text
the boot log writes.

### `pub struct Rebase`

The baros a warm start rebases, and the grounds it rebases them to.

- `fn ground_of(&self, index: usize) -> f32` — The ground of the `index`-th baro of [`Rebase::baros`]: its own, else the pad altitude.

### `pub struct ApplyPlan`

What a passed gate restores (plan B `_apply_restore`), as data. The firmware's `warm.rs` applies it before any
thread runs, in the field order, which is plan B's:
 1. the mission's launch point and zone, BEFORE the arm: arming freezes the launch point from the live fix
    unless one is set, and mid-air that fix is not the pad;
 2. the baro rebase, the pitot tare and the airspeed seed, before anything reads altitude or airspeed;
 3. the checkpoint's identity, else the first checkpoint after the restore saves a crumb with none, and a second
    reset skips the rebase and the tare (and a warm start into BOOSTING would freeze the mid-air altitude as the
    pad);
 4. the stage, the arm (only if the crumb was armed), and the warm-started mark (degraded `WARM-STARTED`).

- `fn new<'a>(crumb: &Crumb, sensors: impl IntoIterator<Item = (&'a str, &'a str)>) -> Option<ApplyPlan>` — The plan for a crumb whose stage is a flight stage; None otherwise (the gate never passes one).
- `fn mission_update(&self) -> Option<Map<String, Value>>` — The mission update as plan B sends it, `{latitude, longitude, zone}` in decimal degrees, for the mission's update path; None when the crumb carried no launch point and zone.

### `pub enum Restore`

The boot decision (plan B `restore`).

- `fn keeps_flight(&self) -> bool` — Whether the recorder's flight flag stays up: the recorder opens with it raised for a crumb [`load`] reads in BOOSTING..LANDING, and only a warm start keeps it. Cold, Disabled and Refused lower it, Cold included: a stage i32 whose blob does not read is Cold with nothing cleared, so a flag kept there would stay up all session, and on every later boot.
- `fn stall_restart_suppressed(&self, reset: ResetReason) -> bool` — Whether this boot restarts the board for no control stall: a warm start after `reset`, this boot's reason, a software one. Off the pad only the supervisor's stall restart resets the board in software (`reboot` runs only in SETTING, and a failing Wi-Fi link parks rather than restarting), so such a boot is a stall restart's own warm start, and a stall that comes back every boot must leave the board stalled rather than cycle it through warm starts. A warm start after any other reset (a panic, a watchdog), and every boot that is not warm, restart for a stall as usual.

### `pub struct Step`

What one pass of the checkpoint schedule asks for.

### `pub struct Freeze`

What [`Checkpoint::step`] freezes at BOOSTING entry, read by the caller when asked (plan B `_freeze_static`).

### `pub struct Live`

The live values a checkpoint records (plan B `_checkpoint`); each absent when its source is stale.

### `pub struct Checkpoint`

The checkpoint schedule (plan B's Checkpoint task): a crumb on every stage change and every period while
airborne, the recovery identity frozen at BOOSTING entry, the pad altitude latched in SETTING.

SETTING and DONE checkpoint once on entry and never periodically: nothing moves during the long pad dwell or the
post-landing wait, so re-writing every period would only wear the flash. The [`Keeper`] decides which
checkpoints also save the NVS crumb (only a flying board's); the telemetry row goes out on every one.

- `fn new(checkpoint_s: f32, now_us: u64) -> Checkpoint` — The schedule for a `checkpoint_s` cadence (the component's config, default 1), started at `now_us`.
- `fn poll_ms(&self) -> u64` — The polling period in milliseconds the owner sleeps between [`step`](Checkpoint::step)s.
- `fn identity(&self) -> &Identity` — The recovery identity the crumbs carry now.
- `fn seed(&mut self, identity: Identity)` — Carry a restored crumb's identity forward (the warm start's [`ApplyPlan::identity`]). The schedule starts empty, which only BOOSTING entry fills, so without this the first checkpoint after a warm start went out with no identity.
- `fn step(&mut self, stage: Stage, now_us: u64, fresh_altitude_m: Option<f32>, freeze: impl FnOnce() -> Freeze) -> Step` — One poll (plan B's `run` loop body): the stage now, the time, a FRESH altitude reading or None, and the identity inputs, read only when this pass freezes.
- `fn crumb(&self, stage: Stage, live: Live, stamp_s: i64) -> Crumb` — The crumb to save for a checkpoint at `stage` (only an armed flight saves one): the identity, then the live state, the airspeed rounded to 0.1 m/s, stamped `stamp_s` (seconds since 2000).

### `pub enum Write`

An NVS action of the [`Keeper`]: the one it owes, and the one a pass committed.

### `pub struct Pass`

What one [`Keeper::pass`] did.

### `pub struct Counters`

The [`Keeper`]'s NVS tallies since boot, for `inspect checkpoint`.

### `pub struct Keeper`

The checkpoint schedule over NVS: the supervisor runs one [`pass`](Keeper::pass) every
[`poll_ms`](Keeper::poll_ms).

The crumb is keyed on FLYING, `armed && launched` of one flight word load, so the stage and the arming a crumb
carries are of one moment:
 * a checkpoint due while flying owes a crumb. Every hold clears `launched`, so an armed bench hold (`arm`, then
   `stage gliding`) owes none: keyed on the arming alone, a reset would resume it as a launched flight;
 * leaving flying (a disarm, or any hold) while a crumb may be in NVS owes a clear, which replaces an owed crumb.
   The recovery from a false launch (disarm, `stage done`, the re-zero, `stage setting`) would otherwise leave an
   armed BOOSTING crumb, and a reset within [`MAX_AGE_S`] would warm-start the pad board armed;
 * an owed action is tried on every pass until NVS commits it, an owed crumb rebuilt from the pass that tries
   it. The schedule counts a checkpoint done whether its crumb landed or not and never re-checkpoints DONE, so
   without the retry a refused DONE crumb leaves the last airborne one, and a reset warm-starts a landed glider.

A crumb may be in NVS from a [`seed`](Keeper::seed) (the warm start's restored crumb) or from any crumb attempt,
a refused one included (its stage i32 may have landed before the commit failed), until a clear commits.

The pad latch has a ceiling: the schedule latches a fresh altitude as the pad only while the fresh elevation is
under it. A launch the accel trigger missed stays in SETTING through the baro launch dwell, where plan B latched
the pad up the climb; the ceiling bounds that error, and a drifted pad's, to the ceiling itself.

- `fn new(checkpoint_s: f32, now_us: u64, pad_ceiling_m: Option<f32>) -> Keeper` — A keeper for a `checkpoint_s` cadence ([`Checkpoint::new`]) started at `now_us`, its pad latch under `pad_ceiling_m` metres of elevation: half the sequencer's `launch_alt_m`, or None without a sequencer, which latches as plan B does.
- `fn poll_ms(&self) -> u64` — The polling period in milliseconds between [`pass`](Keeper::pass)es ([`Checkpoint::poll_ms`]).
- `fn owed(&self) -> Option<Write>` — The action NVS has not committed yet, None when nothing is owed.
- `fn counters(&self) -> Counters` — The NVS tallies since boot.
- `fn identity(&self) -> &Identity` — The recovery identity the crumbs carry now ([`Checkpoint::identity`]).
- `fn seed(&mut self, identity: Identity)` — Carry a warm start's restored identity forward ([`Checkpoint::seed`]) and mark its crumb live, so the edge out of the restored flight clears it.
- `fn pass(&mut self, nvs: &mut impl Nvs, flight: Flight, now_us: u64, fresh_elevation_m: Option<f32>, live: Live, freeze: impl FnOnce() -> Freeze, stamp_s: impl FnOnce() -> i64) -> Pass`

### `pub fn save(nvs: &mut impl Nvs, crumb: &Crumb) -> bool`

Commit the crumb to NVS: the blob, then the stage i32 LAST, then the commit (plan B `save`).

The i32 is the flag, written last, so a torn write leaves the previous stage, never a new stage pointing at a
half-written blob.

Returns true when committed; false, having written nothing, when the crumb has no stage, and false when NVS
refused a write (absent or full). Never panics: a failed checkpoint must never block the flight.

### `pub fn clear(nvs: &mut impl Nvs) -> bool`

Reset the stage i32 to [`NO_STAGE`] (plan B `clear`), so the next boot is unambiguously cold: after a rejected
warm start, and when a flight ends ([`Keeper`]). The blob stays: the i32 alone decides, so the clear is one fast
write.

Returns: true when committed; false when NVS refused (the stage i32 may still hold the crumb's stage). Never
panics.

### `pub fn load(nvs: &mut impl Nvs) -> Option<Crumb>`

The last checkpoint crumb (plan B `load`): the stage i32 gates it, then the blob fills it in.

Returns None when no flight was checkpointed (the stage unwritten or [`NO_STAGE`]) or the blob is missing,
longer than [`BLOB_MAX`], torn, not JSON or not a JSON object: a cold boot. The crumb's stage is the i32's.

### `pub fn should_restore(crumb: Option<&Crumb>, separated: bool, reset: ResetReason, now_s: i64) -> (bool, Reason)`

The warm-start gate (plan B `should_restore`, behind the operator's boot rules): is this a legitimate mid-flight
reset to recover the crumb's stage into?

The checkpoint re-stamps the crumb every second aloft, so its STAGE is trustworthy; the gate confirms this is a
genuine, RECENT reset of a real flight. In order, the first that fails is the reason:
 * a crumb, with a stage and a stamp;
 * the stage rules (§4.6): a flight stage, not DONE (a new session), not SETTING (a cold pad boot);
 * the reset reason is a reset ([`ResetReason::is_reset`]), never a power-on;
 * the age `now_s - stamp` within 0..=[`MAX_AGE_S`]: the RTC survives a reset, so continuity holds, while a
   power cycle restarts it and breaks the arithmetic;
 * GLIDING and LANDING need the separation latch SEPARATED: the physical latch no software can fake, so a
   landed-then-nested glider never recovers into a glide. BOOSTING is the boost itself (latch nested), so the
   reset and the age carry it.

`now_s` is in seconds since 2000, like the stamp. Returns (true, [`Reason::Recover`]) on a pass, else false with
the first failing reason.

### `pub fn restore<'a>(nvs: &mut impl Nvs, warm_start: bool, separated: bool, reset: ResetReason, now_s: i64, sensors: impl IntoIterator<Item = (&'a str, &'a str)>) -> Restore`

Decide the boot from NVS (plan B `restore`, without its logging and GC handling): load the crumb, honour the
config toggle, ask the gate, and clear the stage on a refusal.

A refused clear is ignored, as plan B swallowed it, so the stage i32 keeps the crumb for the next boot. A
refusal that lasts (DONE, SETTING, torn, no flight stage, too old) refuses it there again; one that can change by
then (a power-on, the toggle off, a nested latch, a clock behind the stamp) can pass. The [`Keeper`] clears only
a crumb it saved or was seeded with, so a later reset within [`MAX_AGE_S`] of the stamp can warm-start a landed
board. Plan B has the same gap.

`warm_start` is the checkpoint component's toggle (default on), `separated` the separation latch read at
setup, `reset` this boot's reset reason, `now_s` the clock in seconds since 2000, and `sensors` the board
config's (name, driver) pairs for the baro rebase.

### Constants

- `const NAMESPACE: &str = "coludo"` — The NVS namespace both firmwares keep their small state in (plan B `NVS('coludo')`).
- `const CRUMB_KEY: &str = "crumb"` — The NVS key of the crumb blob, the JSON document.
- `const STAGE_KEY: &str = "stage"` — The NVS key of the stage i32, the flag written last.
- `const NO_STAGE: i32 = 0` — The stage i32 that means "no crumb": what an unwritten key reads as, and what [`clear`] writes (plan B `Stage.NULL`). Never a live stage.
- `const BLOB_MAX: usize = 512` — The read buffer for the crumb blob (plan B `_BLOB_MAX`). A crumb is about 250 bytes; the headroom is for new fields. A longer blob does not fit the buffer, so it reads as no crumb (a cold boot), on both firmwares.
- `const MAX_AGE_S: i64 = 600` — The oldest crumb the gate accepts, in seconds (plan B `_MAX_AGE_S`). Airborne crumbs are re-stamped every checkpoint, so they arrive at a reset about a second old; the bound rejects a stale crumb from a past session. (Plan B also sized it as the DONE operator-wait dwell; DONE no longer warm-starts.)
- `const UNIX_SECONDS_AT_2000: i64 = 946_684_800` — Unix seconds at 2000-01-01T00:00:00Z: the stamp's epoch is MicroPython's, so the firmware subtracts this from `gettimeofday` before it stamps a crumb or asks the gate.
- `const REBASED_BAROS: [&str; 3] = ["icp10111", "bmp280", "bmp581"]` — The baro drivers whose setup re-zeroes the altitude, so a warm start rebases them to the pad (plan B `_REBASED_BAROS`). Every one: the BMP581 (TMS-7F's backup elevation) was once missing, so after a mid-air reset it read about 0 m aloft and an ICP outage handed the landing check the ground.
- `const GROUND_SLOTS: usize = REBASED_BAROS.len()` — The rebased barometers whose own pad ground a crumb carries ([`Identity::grounds_m`]): one per driver in [`REBASED_BAROS`].
- `const WARM_CAP: u32 = 3` — A flight's warm starts after which the next recovers DISARMED (the port's own, with no plan B counterpart): the law and the fins stay off, the stage machine still walks the flight to DONE and its beacon. A crash that returns every boot (a fault in the law, the fins, or what they read) would otherwise reset the board over and over in the air; a cold boot mid-air instead would never reach DONE, so nothing would beacon.

# `controller` — `src/avionics/crates/controller`

_Uses: `board_config`, `control`, `databoard`, `recorder`, `sequencer`, `sim_link`. External: log, serde_json._

_Integration tests: `tests/golden.rs`, `tests/golden_readiness.rs`._

## `controller`

_Tested by its own `mod tests`._

The device bring-up's rules: the port of src/glider controller.py's setup pass, and of the operator lines
cc_client.py builds from its outcome (doc/specs/avionics-port.md M5, "The registry"). The firmware's controller
(M5 stage 5) owns the buses and the parts and runs each attempt; this crate decides everything around them, with
no bus and no clock of its own:
 * [`set_up`] -- one entry's attempts: `board.setup_retries` of them, [`RETRY_GAP_MS`] apart and never after the
   last, the driver's diagnose folded into the reason on the last, plan B's log lines word for word
   (controller.py:385-445);
 * [`candidate`] and [`sort_alternatives`] -- the alternatives pass: a failed entry that an up device stands in
   for (the same I2C socket, the same data, another driver, and no XSHUT pulse of its own) is not fitted rather
   than failed (controller.py:130-241);
 * [`Table`] -- the outcome per directory entry (Up, Failed, Unfitted, and Crashed once an Up part's loop dies,
   M6+), the lines the bring-up ends with (controller.py:378-382), and the readers CC answers from: `health.tasks`,
   `verify`'s devices, problems and alternatives, the failure lines `probe` and `arm` print (cc_client.py:292-349,
   513, 807-816), and `inspect controller` (controller.py:630-632);
 * [`binding`] -- plan B's bind() defaults: the bus and address an entry's driver talks over (i2cbus.py:275-301,
   spibus.py:196-219), ported once so no call site compares the raw `bus`, `id` and `addr` fields itself;
 * [`stream`] and [`vetoes`] -- the telemetry stream a recorded entry writes, and the stage machine's second, its
   veto episodes: the file, columns, window (never decimated: measured -> reported) and durability (module
   `streams`);
 * [`readiness`] -- `verify`'s flight-readiness verdict, the config choices no probe can see (cc_client.py:47-107);
 * [`ring_gate`] -- the flash ring's refusal of `arm` and `verify`, under plan B's recorder probe's words;
 * [`stale_arm`] -- the refusal of an `arm` that waited on the board until the hub may have given up on it;
 * [`Stall`] -- the supervisor's rule for a control thread that stopped ticking while the board controls a flight
   (module `watchdog`).

The device directory, its pins and its `provides` come from board_config's typed view, read from the post-layout
document, as plan B's Controller reads the config layout.resolve has rewritten (main.py:97-101).

The port, against plan B:
 * Plan B creates a Task per attempt and awaits its setup(); here the caller's closure makes the attempt and
   reports its [`Attempt`], and [`set_up`] keeps the rules: the reasons, the retry gaps, the diagnose on the last
   attempt, the log lines.
 * Plan B logs `cleanup raised` when a half-set-up task's finish() raises; a Rust part is dropped, and a drop cannot
   fail, so the line has no counterpart. Nor has `[diagnose raised ...]`: a Rust diagnose answers a text, it does
   not raise.
 * Plan B's Controller may run setup() again, keeping the tasks already up; the firmware brings a board up once
   per boot, so a [`Table`] is built from one pass. [`sort_alternatives`] itself takes any up set in any order, as
   plan B's re-run hands it, and the golden fixture replays that too.
 * The socket's bus id is Python's `str(id)`: an entry with no `id` (absent or null) sits on bus 'None', which
   matches only another id-less entry, never id 0 (controller.py:149), although bind() puts both on i2c:0. Kept,
   because the alternatives rule is plan B's.
 * An `addr` outside 0..=0x7f, or not an int, is no address (board_config's `Entry`), so the entry has no socket;
   plan B keeps it as a socket and its driver fails at the first transfer. validate() refuses all but the first.
 * A device whose loop crashed stays in plan B's `failures` in crash order; [`Table`] lists every reader in
   directory order. Only the key order of a JSON object differs.

### `pub struct Socket`

An I2C socket: the place a part answers, a bus and an address (controller.py:130-149 `_socket`).

Only an I2C address identifies one. Two UART or PWM devices on one bus are two devices, and an SPI entry's
`addr` is its fallback for an I2C wiring: on SPI the chip select is the place, so two SPI parts sharing a fallback
address are two parts.

### `pub struct Candidate<'d>`

One directory entry, as the alternatives pass compares it.

### `pub enum Attempt`

What one setup attempt did, as the caller reports it to [`set_up`].

### `pub enum Binding`

The bus a device's driver talks over, as plan B's bind() resolves an entry (i2cbus.py:275-301, spibus.py:196-219).

### `pub enum Status<'t>`

A directory entry's status, as CC reads it.

### `pub struct DeviceEntry<'d>`

One directory entry of the board and how its bring-up ended.

The table is built once at boot and shared read-only (the firmware leaks it). It sees two changes afterwards, and
neither needs a lock: a crash, set at most once, and the part's runtime health, an atomic its producer writes.
That atomic is why an entry is neither `Clone` nor `PartialEq`: it is a live flag, not a value.

- `fn status(&self) -> Status<'_>` — The entry's status: its bring-up's outcome, or [`Status::Crashed`] once an up part's loop died.
- `fn active(&self) -> bool` — Whether the part came up, crashed or not: plan B's `active(name)` is not None (controller.py:287-299), which is what `health.tasks` lists.
- `fn crash(&self, reason: String) -> bool` — Record that the part's loop died: true when this is the entry's crash, false for an entry that never came up or has crashed already (the first reason stays, as a dead loop dies once).
- `fn healthy(&self) -> bool` — Whether the part's latest read succeeded (plan B's `inspect()['healthy']`).
- `fn set_healthy(&self, healthy: bool) -> bool` — Record the outcome of the part's latest read, from its producer (its one writer): true when this changed the health, the transition plan B's `note()` logs on. It writes only on a change, so a producer may call it every read without dirtying the cache line the readers on the other core share. Relaxed: the flag publishes nothing else. An entry that never came up has no reads and stays unhealthy: false, nothing written, as [`DeviceEntry::crash`] refuses it too.

### `pub struct Table<'d>`

The board's devices after bring-up: one entry per directory entry, in directory order, and the readers CC
answers from.

- `fn new(bring_up: Vec<(Candidate<'d>, Result<(), String>)>) -> Table<'d>` — The table one bring-up pass leaves: each directory entry's candidate with what [`set_up`] returned for it, in directory order (the order plan B sets them up in), and the alternatives pass run over the result ([`sort_alternatives`], with the entries that came up in that order).
- `fn entries(&self) -> &[DeviceEntry<'d>]` — The entries, in directory order; [`Status::Unfitted`] indexes this slice.
- `fn find(&self, name: &str) -> Option<&DeviceEntry<'d>>` — The entry named `name`, if the directory has one.
- `fn tasks(&self) -> Vec<(&str, bool)>` — `health.tasks`: every part that came up, with false once its loop crashed (cc_client.py:513 `{name, ok: validate()}` over the Controller's tasks).
- `fn failures(&self) -> Vec<(&str, String)>` — The failure line of every entry in plan B's `failures`: the never-up ones and the crashed ones, as `verify`'s and `arm`'s problems and `probe all` list them (cc_client.py:566, 765, 816).
- `fn alternatives(&self) -> Vec<(&str, String)>` — The not-fitted line of every alternative: `verify.alternatives` and `probe`'s second map (cc_client.py:768, 807), and what [`Table::bring_up_lines`] logs for each.
- `fn devices(&self) -> Vec<(&str, String)>` — `verify.devices` (cc_client.py:807-815): every entry's line, `'<driver> -- up'`, `'<driver> -- down: <reason>'` (a crashed part is down too), or its not-fitted line.
- `fn not_up_line(&self) -> Option<String>` — The bring-up's summary line, `controller :: <n> device(s) not up: <names, sorted>`, over plan B's `failures` (controller.py:380-382); None when every entry is up or an alternative.
- `fn bring_up_lines(&self) -> Vec<String>` — The lines plan B's setup() logs once every entry has had its attempts (controller.py:378-382): `controller :: <name>: <not-fitted line>` for each alternative, in directory order, then [`Table::not_up_line`] when there is one. Read them before any crash, as plan B logs them at bring-up: a crashed winner's line says it is down.
- `fn inspect(&self) -> Inspection<'_>` — The `inspect controller` snapshot: see [`Inspection`].
- `fn failure(&self, name: &str) -> Option<String>` — The failure line for one device (cc_client.py:311-326 `_failure`); None for a name that is no failed or crashed entry (plan B raises KeyError there).
- `fn probed(&self, name: &str, result: Option<&str>) -> Option<String>` — One probe result as the operator reads it (cc_client.py:329-349 `_probed`): a failing `result` named after the device's implementation (unchanged for a name that is no entry); for a passing one (None), the failure line of a device in `failures`, because a crashed part whose probe still answers is not healthy; otherwise None.
- `fn unfitted(&self, name: &str) -> Option<String>` — The not-fitted line for one alternative (controller.py:243-263 `unfitted`); None for a name that is no alternative.

### `pub struct Inspection<'t>`

What `inspect controller` reports of the bring-up: plan B's `inspect()` keys `tasks`, `failures` and
`alternatives` (controller.py:630-632), each as plan B holds it, not as CC words it. Its `stage`, `armed` and
`manual` are the sequencer's, not the table's.

### `pub fn candidate<'d>(document: &Value, entry: &Entry<'d>) -> Candidate<'d>`

The candidate a directory entry of `document` is.

The socket is plan B's `_socket`: an entry whose `bus` is 'i2c' and that has an address; an entry without a `bus`
key has none, whatever its driver defaults to (bind() would still put it on I2C). The quantities are its
`provides` keys as board_config reads them: a quantity whose spec is not an object is not one (validate()
refuses it; plan B's setup raises on it, so the entry fails anyway). The reset pin is plan B's `_reset_gpio`,
the board's `pins` map through `xshut_pin`, read as task.py `_pin_gpio` reads a pin.

### `pub fn set_up(name: &str, implementation: &str, attempts: u32, mut attempt: impl FnMut(u32, bool) -> Attempt, mut sleep_ms: impl FnMut(u32), mut log: impl FnMut(String)) -> Result<(), String>`

Bring one device up: plan B's `_bring_up` (controller.py:385-420) with its `_diagnose` (controller.py:422-445).

`attempt(number, last)` makes attempt `number` (from 1) and reports it; `last` is true on the final one, the only
one whose diagnosis is used. There are `attempts` of them, at least 1 (plan B's `max(1, setup_retries)`,
controller.py:368; board_config::setup_retries reads the count). An attempt that fails is followed by
`sleep_ms(RETRY_GAP_MS)` unless it was the last. `log` receives plan B's lines:
 * `controller :: task '<name>' up` when an attempt brings the part up (controller.py:376);
 * `controller :: no driver/activity '<implementation>' for '<name>'` for [`Attempt::NoDriver`], which ends the
   attempts (controller.py:283, 402);
 * `controller :: task '<name>' setup raised: <error>` for [`Attempt::Raised`] (controller.py:409);
 * `controller :: task '<name>' setup attempt <n>/<attempts> failed, retrying` before each gap (controller.py:417);
 * `controller :: task '<name>' setup found another part, not retried` for [`Attempt::Foreign`] (new).

Returns the failure reason when no attempt brought the part up: [`SETUP_FAILED`], or a raised attempt's error,
as the last attempt left it, with its diagnosis folded in ([`diagnosed`]); [`NO_DRIVER`] when the first attempt
found no driver. A later attempt that finds no driver keeps the reason the attempt before it left, as plan B's
loop does.

### `pub fn diagnosed(reason: &str, detail: &str) -> String`

A failure reason with the driver's diagnosis folded in: `'<reason> -- <detail>'`, or the reason alone when the
diagnosis is empty (controller.py:445).

### `pub fn sort_alternatives(candidates: &[Candidate], up_in_setup_order: &[usize], failures: &mut Vec<(usize, String)>) -> Vec<(usize, usize)>`

Plan B's alternatives pass (controller.py:185-241): move each failed entry that an up device stands in for from
`failures` to the returned (loser, winner) pairs.

Two parts that cannot share one address on one bus are declared for one socket (the VL53L4CX and the VL53L1X at
0x29), and the part that is soldered wins; the other entry's setup fails BY DESIGN. Each failure, in `failures`'
order, takes the FIRST entry of `up_in_setup_order` that passes the four rules:
 1. the same I2C socket ([`Socket`]): an SPI entry, or one without an address, is never a candidate;
 2. the same non-empty set of quantities: an entry that provides nothing has nothing to stand in for;
 3. a different implementation: one driver twice on one socket is a config mistake, and the duplicate's setup
    re-initialises the winner's own part;
 4. no XSHUT of the loser's own: its setup pulses that pin and reboots whatever part sits in the socket, so a
    loser with a [`Candidate::reset_gpio`] stays a failure, in either setup order, its reason extended with
    `'; shares XSHUT GPIO<n> with <winner> (<driver>): its setup resets the fitted laser -- enable exactly one of
    <both names, sorted> on this board'`.

When no up entry passes, the failure stays: a missing laser must still block arming. `candidates` is the
directory, and the other arguments hold indices into it (an index past it names no candidate). `failures` keeps
its order; the pairs come in that order too.

### `pub fn unfitted(loser_implementation: &str, winner_name: &str, winner_implementation: &str, socket: &Socket, winner_down: bool) -> String`

The operator's line for an unfitted alternative: which part is absent and which one took its socket
(controller.py:243-263).

`'<loser driver> not fitted -- <winner driver> (<winner>) answers i2c:1 0x29'` while the winner runs; once its
loop has crashed, `'... answered i2c:1 0x29 at setup, now down'`: the loser stays not fitted (the winner
identified itself at setup), but the line must not say the winner answers while `probe` and `arm` call it failed.

### `pub fn named(implementation: &str, why: &str) -> String`

A verdict that names the device's implementation: `'<driver> -- <why>'`, or `why` alone when there is none (a
name that is no configured device, such as 'mission') (cc_client.py:292-308 `_named`).

### `pub fn failure_line(implementation: &str, reason: &str, crashed: bool) -> String`

The verdict for a device that failed (cc_client.py:311-326 `_failure`): `'<driver> -- not connected: <reason>'`
for one that never came up, `'<driver> -- <reason>'` for one that came up and whose loop then crashed (it was
connected).

### `pub fn binding(entry: &Entry, default_kind: &str, default_address: Option<u8>) -> Binding`

Plan B's bind() defaults for one entry, ported once: which transport its driver opens and where.

`default_kind` is the driver's own transport when the entry names no `bus`: 'i2c' for most drivers and the
ADXL375 (adxl375.py:107 `get('bus', 'i2c') == 'spi'`), 'spi' for the LSM6DSO32 (lsm6dso32.py:144). An absent `id`
is 0 on I2C (i2cbus.py:297) and 1 on SPI (spibus.py:217); an absent `addr` is `default_address`, the driver's
datasheet address (i2cbus.py:301). An explicit null is NOT absent: plan B's `get(key, default)` returns it, so a
null `id` binds no bus and a null `addr` gives the driver no address. A single-transport driver reads an answer of
the other kind as no bus (plan B's would raise building its bus from the other kind's spec).

### Constants

- `const RETRY_GAP_MS: u32 = 200` — The pause between two setup attempts of one device, in milliseconds: a flaky breadboard contact settles (controller.py:418 `asyncio.sleep_ms(200)`).
- `const NO_DRIVER: &str = "no driver/activity"` — The failure reason of an entry whose implementation nothing runs (controller.py:398), and the start of the line logged for it (controller.py:283).
- `const SETUP_FAILED: &str = "setup failed (absent / miswired?)"` — The failure reason of a setup that did not bring its part up (controller.py:406), before diagnose adds the why.

## `controller::control_core`

_Tested by its own `mod tests`._

[`ControlCore`]: the control tick's pure duties (doc/specs/avionics-refactor.md R3), one code path for the board's
control thread and the SITL, so the SITL flies the board's code and not a copy of it (plan B's sim-mirrors trap):

1. the attitude filter (plan B tasks/attitude.py, M9): paced on its `period_ms`, reading the fused `rate`, `accel`
   and `mag` through the marks map, its attitude pushed and recorded on every publish; stopped at DONE;
2. the stage machine (plan B's Sequencer task, M7, M9): stepped every `period_ms` within half a tick of the wake,
   on each bound source's channel; a move made from outside writes its row, a veto episode's edges theirs, and its
   own advance commits, then the console's line, then the recorded line and the row, the recorder's flight flag
   raised before those rows and lowered after them (plan B `_tick`, `_advance`);
3. the flight law (plan B tasks/flight.py, M13): every tick, on the fresh quantities and the mission, its fins
   handed on when it actuates, and every 100 ms (a second at DONE) the flight.csv row and the vitals.

What a board has and the SITL has not comes in through [`Hooks`]: the clock read after the IMU's push (the
databoard refuses a stamp from the future), the mission and the GNSS drift, the recorder's flight flag, the
console, the state published for the executor, and the fins. The board's control thread keeps the hardware: the
gptimer, the SPI IMU, the separation switch, the GPIO interrupts and the lapse watches. Nothing here allocates or
formats text on a tick but the machine's own transition (six or fewer a flight).

### `pub trait Hooks`

What the tick needs of its host beyond the databoard: each call is cheap, never blocks and never allocates.

- `fn now_us(&mut self) -> u64` — The clock now, µs: the board's uptime read again after the IMU's push, the SITL's tick time.
- `fn mission(&mut self) -> (Option<Landing>, Option<Position>)` — The mission's landing zone and its set launch point, as the law reads them now.
- `fn drift(&mut self) -> [f32; 2]` — The pad drift's estimate (east, north, m/s), the wind's de-bias (M11).
- `fn flight_flag(&mut self, raised: bool)` — The recorder's flight flag: raised before a transition's rows, lowered after them.
- `fn moved(&mut self, stage: Stage)` — The stage machine moved itself to `stage`: the console's line, before the recorded one.
- `fn veto_edge(&mut self)` — A veto episode's edge, to count.
- `fn attitude(&mut self, row: [i32; 6], seeded: bool, blind: bool)` — The attitude filter's state after a step: its row, whether it mirrored a source that outranks it, whether it is blind.
- `fn fins(&mut self, angles: &[i32])` — The law actuated: its whole-degree fin angles, by the mixer's surfaces.
- `fn vitals(&mut self, law: &FlightLaw)` — A flight.csv sample was taken: the law's vitals.

### `pub struct SequencerSetup`

The stage machine's setup.

### `pub struct FilterSetup`

The attitude filter's setup.

### `pub struct FlightSetup`

The flight loop's setup.

### `pub struct ControlCore`

The control tick's pure duties, each present when its entry came up.

- `fn new(topology: &'static Topology, tick_us: u64, now_us: u64, filter: Option<FilterSetup>, sequencer: Option<SequencerSetup>, flight: Option<FlightSetup>) -> ControlCore` — The duties set up on `topology` for a tick of `tick_us`, started at `now_us`; a filter with no `attitude` quantity does not run (logged).
- `fn binding(&self) -> Option<(String, Vec<String>)>` — The stage machine's bound sources as one line, and the sources past their class's slots; None without one.
- `fn filter(&self) -> Option<&'static str>` — The filter's entry name, when it runs.
- `fn sequencer(&self) -> Option<&'static str>` — The stage machine's entry name, when it runs.
- `fn law(&self) -> Option<&FlightLaw>` — The flight law, when it runs: its mixer's surfaces, its fin angles.
- `fn tick(&mut self, now_us: u64, slack_us: u64, done: bool, writer: &mut FlightWriter<'_>, hooks: &mut dyn Hooks)` — One control tick that woke at `now_us`, `slack_us` being half a tick, `done` whether the flight was at DONE when it woke: the filter (not at DONE), then the stage machine, then the law, in the board's order (D11 of the M9 plan: the attitude before the stage machine).

## `controller::inputs`

_Tested by its own `mod tests`._

The flight threads' inputs from the board's quantities ([`crate::Topology`]), one implementation for the firmware
and the SITL (M14): the flight law's step ([`FlightQuantities`]), the stage machine's per-source readings
([`StageSources`]), and a simulated frame published as the `hitl` source ([`HitlWriters`]). A host that composed
its own would grade itself, not the board (the virtual_flight lesson: its private launch detector).

### `pub struct FlightQuantities<'t>`

The quantities the flight law reads.

- `fn new(topology: &'t Topology) -> FlightQuantities<'t>` — The law's quantities in `topology`.
- `fn step(&self, now_us: u64, stage: Stage, armed: bool, landing: Option<Landing>, set_launch: Option<Position>, drift: [f32; 2]) -> Step` — The law's step at `now_us` in `stage` (`armed`) from the fresh quantities: the mission's `landing` zone, the set launch point `set_launch` (else the fix), and the GNSS calibration's `drift`. The course's push time is the ground stamp: the same between samples, new with each.

### `pub struct StageSources<'t>`

The stage machine's sources: each class's bound to its slots (crates/sequencer [`Binding`]), and the quantities
they push into.

- `fn new(topology: &'t Topology) -> StageSources<'t>` — The sources in `topology`, bound to their slots in rank order.
- `fn binding(&self) -> &Binding` — The slots.
- `fn readings(&self, now_us: u64) -> SourceReadings` — Each bound source's own channel at `now_us`, fresh in its quantity's shared window (M9: the multi-source machine).

### `pub struct HitlWriters<'t>`

The `hitl` source's writers, one per quantity a SENSORS frame carries (doc/specs/sim-link.md), and the climb its
altitude derives (`change`, as every barometer's).

- `fn new(topology: &'t Topology, source: &str) -> HitlWriters<'t>` — `source`'s writers in `topology`.
- `fn publish(&mut self, sensors: &Sensors, now_us: u64) -> Option<f32>` — Every quantity `sensors` carries pushed at `now_us`, and the climb its altitude (else its elevation) derives; that climb, None until its window fills.

## `controller::readiness`

_Tested by its own `mod tests`._

The flight-readiness gate (doc/specs/avionics-port.md M7, "Arm and verify"): plan B's `_readiness`
(cc_client.py:47-107), the config choices that leave the board ballistic or unrecoverable in flight while every
part still probes healthy -- the watchdog off, the flight loop off or without a gain on an axis, a bench fin
derating still applied, and no landing zone with no CC-less site to pick one from. `verify` reports them as its
separate `ready` verdict and leaves the hardware `pass` alone: bench and HITL sessions trip them on purpose.

The port, against plan B:
 * A component without `enabled` is enabled, as the Controller reads it ([`board_config::directory`]). Plan B's
   `_readiness` alone read it as disabled, an inconsistency that is not ported (§6, "`_readiness` and `enabled`").
 * Plan B's `fins` check (a control surface whose servo has no driver) reads the flight loop's mixer, which the
   port has not built yet. The caller adds it once the mixer exists, as `verify` adds the BNO055's
   `imu_calibration` to the same map.
 * Plan B raises, so its `verify` answers an error, on two documents validate() admits: a null `fins`, and a
   `gains` that is not an object. Here the first is no derating and the second holds no gain on any axis.
 * A document validate() refuses is never run (load() falls back to the defaults), but the answer stays defined:
   a bool multiplier is Python's int, as plan B prints it (`false` is `0`); any other multiplier that is not a
   number is printed as Python's `%s` prints it, a derating all the same, where plan B's `%g` raises; an axis
   that is not an object holds no gain; and a name used twice is its first enabled component, as the Controller
   lists them, where plan B's dict keeps the last component of the name, enabled or not.

### `pub fn readiness(document: &Value, zone_set: bool, has_sites: bool) -> BTreeMap<&'static str, String>`

The flight-readiness problems of the board config `document`, `{check: problem}`, in plan B's words; empty when
the board is flight-ready (plan B's `_readiness`, cc_client.py:47-107).

`zone_set` and `has_sites` are the mission's: whether it holds a landing zone, and whether it holds CC-less sites
to select one from. The checks:
 * `watchdog` -- no enabled component named `watchdog`: a wedged flight loop never reboots;
 * `flight` -- no enabled component named `flight`: the fins hold neutral;
 * `gains` -- an enabled `flight` whose `gains` has no non-zero term (Python's truthiness) on roll, pitch or yaw,
   naming each such axis: the loop computes but cannot act;
 * `fins.limit_multiplier` -- the top-level `fins.limit_multiplier` present and unequal to 1.0 under Python's
   `==` (an int 1 is no derating), printed as Python's `%g` prints it;
 * `zone` -- neither a zone nor a site.

## `controller::ring_gate`

_Tested by its own `mod tests`._

The ring gate (doc/specs/avionics-port.md M7, "Ring gate"; §3.7 "The ring_eraser and newest wins"): `arm` and
`verify` require the flash ring to take a flight -- its bring-up scan has run, its eraser runs, and the eraser holds
its reserve (about 1 % of the ring, flash_ring's `Geometry::reserve_blocks`) erased ahead of the writer. A refusal
plan B never had: its recorder probe asked only that its two RAM rings exist (tasks/recorder.py:30-48). The
verdict keeps that probe's shape, `recorder rings: <why>`, reported under the recorder's name, so CC shows it as it
showed plan B's.

Not refusals: no card (`no-sd`, a health token), a saver behind, and newest wins erasing an unsaved block
(`ring-full`). Newest wins keeps one block more than the reserve erased, so without a card the gate does not
flicker as the writer opens blocks.

### `pub fn ring_gate(scanned: bool, eraser_running: bool, erased_ahead: u32, reserve_blocks: u32) -> Option<String>`

Why the flash ring cannot take a flight, as the recorder's probe verdict `recorder rings: <why>`; None when it
can.

`scanned` says the writer's bring-up scan has run, `eraser_running` that the ring_eraser thread started,
`erased_ahead` counts the erased blocks ahead of the writer (flash_ring's `Figures::erased_ahead`), and
`reserve_blocks` is the reserve the eraser keeps (`Geometry::reserve_blocks`). The first check that fails names
the why, in this order: a ring never scanned has no figures to trust, and one with no eraser never refills its
reserve, whatever it holds now.

## `controller::stale_arm`

_Tested by its own `mod tests`._

The arm's age rule (doc/specs/avionics-port.md §4.4, "The arrival stamp and the 25 s rule"): the executor stamps
each request line as it arrives, and before it posts an `arm` it refuses one that has waited too long. A line waits
behind the commands queued ahead of it (a hub probe sweep is up to ~18 s) and then through its own sweep, and the
hub gives up on an answer after 30 s (src/control/board.py): an arm posted later than that would arm a board whose
operator was told it did not answer. Plan B had no such rule: its one asyncio loop served the hub's lines in
lockstep.

### `pub fn stale_arm(waited: Duration) -> Option<String>`

Why an `arm` that has waited `waited` since it arrived must not be posted, as `arm` refuses it under the key
`arm`: `the request waited <n> s: send it again`, `<n>` in tenths of a second rounded up, so a wait just past the
limit never reads as the limit itself. None while the wait is within [`ARM_WAIT_LIMIT`].

## `controller::streams`

_Tested by its own `mod tests`._

The telemetry streams (doc/specs/avionics-port.md M6, M9): which entry writes which stream, with its file, its
columns, its decimation window and whether it is durable, as plan B's drivers and tasks build their Telemetry at
setup (drivers/lsm6dso32.py:124-126, drivers/bmi323.py:100-102, drivers/icp10111.py:123-125,
drivers/bmp581.py:69-71, drivers/bmm350.py:99-100, tasks/attitude.py:100-102, drivers/separation.py:57,
tasks/sequencer.py:110, warmstart.py:318). M6 records the LSM6DSO32, the ICP-10111 and the separation switch, M7
the stage machine's transitions, M8 the checkpoint's timeline, and M9 the SEN0697 (the BMI323, the BMP581 and the
BMM350), the attitude filter and the stage machine's veto episodes; every other part has none yet.

The port, against plan B:
 * Plan B registers a stream in its task's setup. The firmware registers one at `recorder::open` for every named
   entry, enabled or not (board_config's `entries`), so a part the layout enables at boot still has its stream.
 * MEASURED -> REPORTED (the operator, 2026-10-08): no device stream is decimated. Plan B thinned each one to its
   entry's `telemetry_ms` (0 or absent: the global `recorder.telemetry_ms`) and attitude.csv to 100 ms by default,
   a MicroPython-era economy. Here every sample a part measures, and every attitude the filter publishes, is a row,
   and each part runs at the rate its consumers need instead. `telemetry_ms` is read by nothing here, whatever it
   holds (plan B multiplied it by 1000: null raised at setup, text made a text that failed later, a bool read as 0
   or 1 ms; validate() still refuses all three).
 * Plan B passes `separation.csv` no window, so the global decimates it, while its note says "durable, every
   event". Here the stream is durable and never decimated.
 * Plan B's sequencer passes its `<name>.csv` no window either. Its rows are the stage transitions, a handful a
   flight, so here that stream too is durable and never decimated.
 * Plan B's checkpoint passes `checkpoint.csv` no window either. Its rows come once a stage change and once a
   `checkpoint_s` aloft, and the last one before a reset is the post-mortem's record of where the flight was, so
   here it is durable and never decimated too.
 * So every stream here is never decimated; the device streams stay not durable (many rows a second, which the
   ring_writer's own cadence carries), the event records durable.
 * A barometer's row gains a fifth column, `change`, its climb rate in m/s (crates/control `change`), appended so
   plan B's four stay where they were. A reader takes plan B's four-column rows as they are
   (tools/recorder_flight.py).
 * separation.csv gains two columns, `moment` and `duration`: the first edge of the contact's episode the row
   concludes, on the uptime column's own clock, microseconds, and the microseconds from it to the episode's last
   edge (the wobble). A row's `uptime` is the tick that concluded the episode, a quiet after its last edge, so the
   latency is in the data. A reader takes plan B's two-column rows as they are (tools/recorder_flight.py).
 * Its `event` gains a third value: plan B's `separated` and `nested`, and the port's `blip`, an episode that ended
   on the level the latch already held (the contact moved and came back), so a flight shows every movement of the
   contact. A blip moves neither the latch nor the stage; the readers take the event as a word, whichever it is.
 * The stage machine's entry writes a second stream, the port's own ([`vetoes`]): `vetoes.csv`, a begin and an end
   row per veto episode of the multi-source machine (crates/sequencer `VetoEdge`), fixed, durable and never
   decimated like the transitions' record. Those rows stay out of `<name>.csv`, whose every row
   tools/assemble_capture.py takes for a stage event.

### `pub struct Stream`

A telemetry stream, as the recorder registers it.

### `pub fn stream(entry: &Entry) -> Option<Stream>`

The stream `entry` writes, when its part is one the port records, every one never decimated: the IMUs (LSM6DSO32,
BMI323), the barometers (ICP-10111, BMP581) and the BMM350 as `<name>.csv`, the attitude filter as the fixed
`attitude.csv`, the fin servos as `<name>.csv`; the separation switch as the fixed `separation.csv`, the stage machine as `<name>.csv` and the
checkpoint as the fixed `checkpoint.csv`, the last three durable. None for any other part. The stage machine's
second stream is [`vetoes`].

### `pub fn companions(entry: &Entry) -> Vec<(String, Stream)>`

The streams an entry writes beside its own `<name>.csv`, each with the device key it is registered under: the
GNSS's `<name>_gga.csv`, `<name>_sky.csv` and the durable `<name>_events.csv` (plan B gnss.py, task.py:104), all
never decimated. Empty for any other entry.

### `pub fn vetoes(entry: &Entry) -> Option<Stream>`

The stream the stage machine's entry writes beside its own `<name>.csv`: its veto episodes, the fixed `vetoes.csv`
whatever the entry's name, durable and never decimated like the transitions' record. None for any other entry.

### Constants

- `const IMU_FIELDS: [&str; 7] = ["ax", "ay", "az", "gx", "gy", "gz", "irq_runs"]` — The IMUs' columns, the LSM6DSO32's and the BMI323's alike (plan B drivers/lsm6dso32.py:125, drivers/bmi323.py:101): accel in g, the gyro as raw centidegrees a second, and the INT1 edges the tick saw.
- `const BAROMETER_FIELDS: [&str; 5] = ["altitude", "temperature", "pressure", "elevation", "change"]` — The barometers' columns, the ICP-10111's and the BMP581's alike: plan B's four (drivers/icp10111.py:124, drivers/bmp581.py:70), m AMSL, degrees C, Pa and m above the ground, then the port's `change`, the climb rate in m/s (crates/control `change`), appended.
- `const ABSOLUTE_FIELDS: [&str; 6] = ["heading", "roll", "pitch", "ax", "ay", "az"]` — The BNO055's columns (plan B drivers/bno055.py:105, M18): the heading in degrees, the roll and the pitch in centidegrees (integers, as plan B pushed its fixnums), the accelerometer in g.
- `const MAGNETOMETER_FIELDS: [&str; 3] = ["mx", "my", "mz"]` — The BMM350's columns (plan B drivers/bmm350.py:99): the field per axis in the part's own counts, raw until the operator calibrates it, then centred and normalised (crates/drivers `bmm350`).
- `const PITOT_FIELDS: [&str; 3] = ["dynamic_pressure", "airspeed_cms", "temperature"]` — The SDP810 pitot's columns (plan B drivers/sdp810.py:164-166): the tared dynamic pressure in Pa x 100, the airspeed in cm/s and the temperature in degrees C x 100, all integers as plan B pushed them.
- `const POWER_FIELDS: [&str; 4] = ["voltage_mv", "current_ma", "power_mw", "alerts"]` — The INA226's columns (plan B drivers/ina226.py:93-95): mV, mA, mW and the over-current ALERT trips since boot.
- `const FLIGHT_FIELDS: [&str; 13] = [` — The flight loop's columns (plan B tasks/flight.py:146-148), as `flight.csv` whatever the entry's name: the stage id, active, the airspeed estimate in cm/s, the fin cap, the roll and pitch setpoints in centidegrees, the heading error, the per-axis demands in whole degrees, the wind in cm/s and its from-direction, and whether it navigates dead-reckoned.
- `const SERVO_FIELDS: [&str; 3] = ["angle", "pulse_us", "done"]` — The fin servos' columns (plan B drivers/sg90.py:187, the MG90S alike): the commanded angle in whole degrees, the pulse driven in us, and done (0 a command issued, 1 a move completed by its estimate).
- `const LASER_FIELDS: [&str; 2] = ["agl", "irq_runs"]` — The laser rangers' columns (plan B drivers/vl53l1x.py:150, vl53l4cx.py alike): the height above ground in m and the data-ready edges its read took.
- `const GNSS_FIELDS: [&str; 4] = ["lat", "lon", "speed_kn", "course"]` — The GNSS fix's columns (plan B gnss.py:173): lat and lon (written from the 1e-7 degree integers, 7 decimals), the ground speed in knots and the course in degrees, as an RMC gives them.
- `const GGA_FIELDS: [&str; 5] = ["altitude_m", "elevation_m", "quality", "satellites", "hdop_cd"]` — The GNSS's GGA columns (plan B gnss.py:186-188, `<name>_gga.csv`): altitude and elevation in m, the fix quality, the satellites used and the HDOP in hundredths.
- `const SKY_FIELDS: [&str; 8] = ["in_view", "used", "mode", "cn0_1", "cn0_2", "cn0_3", "cn0_4", "antenna"]` — The GNSS's sky columns (plan B gnss.py:195-197, `<name>_sky.csv`): one row per diagnostics burst.
- `const EVENT_FIELDS: [&str; 1] = ["event"]` — A task's durable event record (plan B task.py:104, `<name>_events.csv`): one text per event.
- `const SEPARATION_FIELDS: [&str; 4] = ["event", "stage", "moment", "duration"]` — The separation switch's columns: plan B's event (`separated` or `nested`, and the port's `blip`) and the stage after it (drivers/separation.py:57), then the port's `moment`, the uptime (µs) of the first edge of the episode the event concludes (drivers::separation `Event::moment_us`: for a separation, the separation moment), and `duration`, the µs from it to the episode's last edge (`Event::duration_us`: the contact's wobble, 0 for a clean single edge).
- `const SEQUENCER_FIELDS: [&str; 2] = ["stage", "reason"]` — The stage machine's columns (plan B tasks/sequencer.py:110): the stage it moved to, or found the board moved to, and why (`launch |a|=...`, or `external` for a move it did not make).
- `const VETOES_FIELDS: [&str; 7] = ["detector", "edge", "decider", "kinds", "contrary", "live", "value"]` — The stage machine's veto episodes' columns, the port's own (crates/sequencer `VetoEdge`, one row per edge): * `detector` -- the detector held (`Detector::name`: `launch_altitude`, `launch_acceleration`, `apogee` or `stillness`); * `edge` -- `begin` on the episode's first vetoed step, `end` on the first step after it (`VetoEdge::name`); both rows carry a vetoed step, an end row the episode's last; * `decider` -- the deciding source's device name (`Binding::source` of `Veto::decider`); * `kinds` -- the vetoes that held it (`Veto::kinds`: `peer`, `at_rest` or `thrust`, two joined by `+`); * `contrary` and `live` -- how many of the decider's live peers were contrary, and how many voting sources were live, the decider included (`Veto::contrary`, `Veto::live`); * `value` -- the decider's value (`Veto::value`): |accel| in g for the accel launch and the stillness, the elevation in m for the altitude launch and the apogee.
- `const CHECKPOINT_FIELDS: [&str; 5] = ["stage", "altitude", "speed", "airspeed", "ticks_ms"]` — The checkpoint's columns (plan B warmstart.py:318-319): the stage's id, 1 (SETTING) to 5 (DONE), as plan B pushes `controller.stage`; the altitude in m AMSL, the speed and the airspeed in m/s; and the uptime in ms the checkpoint was taken at.

## `controller::topology`

The board's quantities, by sample type, each with its sources: built once at bring-up (crates/controller
`topology`, from the entries that came up), then shared read-only by the flight threads, the firmware's and the
SITL's alike.

### `pub struct Topology`

- `fn scalar(&self, quantity: &str) -> Option<&Parameter<f32>>` — The scalar quantity `quantity` (altitude, pressure, ...), when a provider of it came up.
- `fn vector(&self, quantity: &str) -> Option<&Parameter<[f32; 3]>>` — The vector quantity `quantity` (accel, rate, mag, attitude), when a provider of it came up.
- `fn position(&self, quantity: &str) -> Option<&Parameter<[i32; 2]>>` — The position quantity `quantity` (`position`), when a provider of it came up.
- `fn add(&mut self, kind: Kind, quantity: &'static str, source: &'static str, priority: i32, timeout_ms: u32, reconcile: bool) -> bool` — Registers `source` for `quantity` of `kind` at `priority` with `timeout_ms`, the quantity made on its first source. Whether it was added (a quantity holds [`databoard::MAXIMUM_SOURCES`] at most).
- `fn describe(&self) -> Vec<String>` — One line per quantity: `databoard :: pressure <- baro_icp10111 p0, baro_bmp581 p1; window 200 ms`.
- `fn inspect(&self, now_us: u64) -> serde_json::Map<String, Value>` — Plan B's `inspect databoard` (databoard.py:506-511): each quantity's fused reading at `now_us` -- its value (a scalar, a vector, a position in degrees), the source that won, its age in ms, all null when no source is fresh -- and the learned reconcile offsets by source.
- `fn stats(&self) -> serde_json::Map<String, Value>` — Plan B's `stats databoard` (databoard.py:513-515): each quantity's sources, sorted by name as plan B's `sources()` sorted them.

### `pub enum Kind`

One kind of quantity: what its samples are.

### `pub fn topology(entries: &[Entry<'static>], table: &Table) -> (Topology, Vec<String>)`

The topology of the entries `table` holds as up, in setup order, and its warnings: a quantity no board knows, a
source past its quantity's limit. The firmware's bring-up and the SITL build it alike.

### Constants

- `const HITL_QUANTITIES: [&str; 13] = [` — The quantities the `hitl` entry provides (plan B hitl.py's `Databoard.provide` at setup, its entry has no `provides`), each at priority 0 with plan B's 1 s timeout; `change` the port's own, as every barometer's.

## `controller::watchdog`

_Tested by its own `mod tests`._

The supervisor's stall rule (doc/specs/avionics-port.md M8, "Warm start, checkpoint, supervisor watchdog"): while
the board controls a flight, the control thread must keep ticking, and one whose last tick is older than
`stall_ms` has stalled, so the supervisor restarts the board. Plan B's watchdog task judged the same by the flight
task's heartbeat TIMESTAMP (tasks/watchdog.py `_stalled`), independent of its own poll: a supervisor that checks
late, or a check that lands between two ticks, sees the true age of the last tick.

The port, against plan B:
 * The heartbeat is the uptime in ms as a wrapping u32, which the control thread stores beside its tick count (the
   P4 has no 64-bit atomics), and the age is the signed wrapping difference. So the u32 wrap (every 49.7 days) is
   no stall, and neither is a `now` the supervisor read just before the control thread stored a newer tick (the
   two run on different cores): that age is negative. An age past `i32::MAX` ms (24.8 days) reads negative too,
   but the supervisor checks every `period_ms`, so a stall is caught long before it is that old.
 * The limit is at least two tick periods: one late tick is jitter, not a stall, so a `stall_ms` below that
   cannot restart a healthy board.
 * A control thread that has never ticked is a failed bring-up, not a stall: the rule judges only one that has.
 * Whether the board controls a flight is the caller's to say: armed, launched, in an airborne stage. Plan B
   judged only while its flight task controlled the fins, so never on a passive flight.

### `pub struct Stall`

The stall rule, its limit fixed at bring-up from the watchdog's `stall_ms` and the control tick's period.

- `fn new(stall_ms: f32, tick_ms: f32) -> Stall` — The rule for a `stall_ms` limit on a control thread that ticks every `tick_ms`.
- `fn step(&self, now_ms: u32, last_tick_ms: u32, ticked: bool, controlling: bool) -> bool` — Whether the control thread has stalled: the board controls a flight, the thread has ticked, and its last tick is older than the limit.

# `parts` — `src/avionics/crates/parts`

_Uses: `board_config`, `control`, `controller`, `databoard`, `drivers`, `recorder`. External: embedded-hal, log, serde, serde_json._

_Integration tests: `tests/runs.rs`._

## `parts`

_Tested by its own `mod tests`._

The I2C parts as their bus owner runs them (doc/specs/avionics-refactor.md R2; plan B's task registry and its
drivers' run loops), over embedded-hal, so the host tests drive them on the drivers' mock bus:

- what runs each board-config implementation ([`Kind`], [`classify`]);
- [`Part`]: one I2C part's driver, built afresh from its entry for every setup attempt, as plan B creates a task
  per attempt; its owner sets it up, re-arms it after a general call and asks its diagnose when the last attempt
  failed;
- [`Member`]: a part that came up, with its [`Run`] (its channels, its stream, its health; plan B's run loop) and
  its [`Shared`] state, which the owner publishes ([`published::Published`]) and the executor reads (plan B's
  `inspect`, [`own`]);
- the calibrations a part keeps across power cycles, read through the board's [`Store`] ([`restore_profiles`],
  [`restore_saved`]).

The board's own facilities -- the control thread's GPIO interrupts and the recorder's streams -- come in through
[`Platform`]; the firmware keeps the platform, the registry of names and the calibrations' NVS writes.

### `pub enum Kind`

What runs an entry's implementation (its `driver`, else its `activity`).

### `pub trait Platform: Sync`

What a part's run needs of the board beyond its bus: the GPIO interrupts the control thread serves (it installs
the ISR service), the control thread's lapse watches, and the recorder's streams. Every call is cheap and never
blocks: the owner calls them on its wake.

- `fn interrupts_ready(&self) -> bool` — Whether the GPIO interrupt service runs: a run arms its line only after.
- `fn arm_second_interrupt(&self, gpio: u8) -> Result<(), String>` — Counts the second IMU's INT1 edges on `gpio` from now on.
- `fn take_second_edges(&self) -> u32` — The second IMU's INT1 edges since the last take.
- `fn arm_alert_interrupt(&self, gpio: u8) -> Result<(), String>` — Counts the power monitor's ALERT edges on `gpio` from now on.
- `fn alert_edges(&self) -> u32` — The power monitor's ALERT edges since boot.
- `fn lapses(&self, name: &str) -> Vec<(String, u32)>` — The lapses of source `name`'s channels the control thread watched, by quantity.
- `fn rows(&self, name: &'static str) -> Rows<'static>` — The rows of the stream registered under `name` (a refused row logged once a run).

### `pub fn classify(entry: &Entry) -> Kind`

What runs `entry`. The LSM6DSO32 is an SPI part unless its entry binds it to I2C (plan B's lsm6dso32.py:144: the
`bus` defaults to 'spi').

### `pub fn rezeroes(implementation: &str) -> bool`

Whether the implementation re-zeroes a barometer's ground: the ICP-10111, the BMP280 and the BMP581 (M9), whose
`calibrate` and `update` (`ground`, `rezero`) also run at DONE, the way back from a false launch ([`Part::rezeroes`]
says it of a built part).

### `pub fn imu_axes(entry: &Entry) -> Result<Axes, String>`

The `axes` of an IMU entry (drivers::axes, M9): the identity when absent.

Errors: a malformed key or a mirror, naming the key (the part's setup fails with it).

### `pub fn default_address(implementation: &str) -> Option<u8>`

The datasheet address an I2C part's driver binds when its entry has no `addr` ([`KNOWN`]); None for an
implementation that is no I2C part.

### Constants

- `const NOT_PORTED_YET: &str = "no driver/activity -- not ported yet"` — The failure reason of a plan B implementation this port has not reached yet (D13 of the M5 plan): plan B's `no driver/activity`, with why.
- `const KNOWN: [(&str, u8); 11] = [` — Every I2C part this port drives, by implementation name, with the datasheet address its driver binds when its entry has no `addr` (plan B i2cbus.py:301). A new part is a line here, its driver, its run and its [`Part`] arm.

## `parts::absolute`

The BNO055's run (M18): the fused attitude of the v0.1 boards' backup, withheld while the fusion is stalled.

### `pub struct AbsoluteState`

What `inspect`, `verify` and the heartbeat show of the BNO055 beyond its channels (plan B bno055.inspect).

- `fn instruction(&self) -> Option<String>` — Its pending calibration, in plan B's words (bno055.calibration()); None when nothing is owed.

### `pub struct Absolute`

The BNO055's outputs (M18, plan B bno055.run): every period one block read; the fused Euler to `attitude` unless
the fusion is stalled (the channel then goes stale and the databoard falls to the next source), the accelerometer
through the airframe's axes to `accel` always (that half still works), a row to its stream, and CALIB_STAT read
about once a second for the operator.

## `parts::barometer`

A barometer's run (M6, M9, M18): the ICP-10111's two-phase measurement and the backups' double reads (the BMP581
of the SEN0697, the BMP280 of the SEN0253), each conversion published to five channels and a row.

### `pub struct BarometerState`

What `inspect` shows of a backup barometer's double read (M9): the pairs that disagreed (mostly a conversion
landing between the two bursts, about 1 read in 80; the rest the bus garbage the double read exists for) and the
agreeing pairs outside the part's operating range, since boot.

### `pub struct Barometer`

A barometer's outputs: its five databoard channels (plan B's four in its push order, then `change`) and its stream.

## `parts::due`

_Tested by its own `mod tests`._

[`Due`]: when a polled run reads next.

## `parts::inertial`

_Tested by its own `mod tests`._

The BMI323's run (M9): a backup to the SPI IMU, read on every owner wake.

### `pub struct InertialState`

What `inspect` shows of the BMI323 beyond its channels (plan B bmi323.py:305-317 and the port's own).

- `fn odr_hz(&self) -> Option<f32>` — Its measured ODR, Hz, over its fresh accel conversions: None before two of them.

### `pub struct Inertial`

The BMI323's outputs (M9): its `accel` and `rate` channels (a backup to the SPI IMU, at its config priority) and
its stream, every conversion through the airframe's axes. Read every owner wake, paced by the control tick; its
INT1, when wired, is evidence only (D12 of the M9 plan; plan B sampled on the edge).

## `parts::inspect`

The I2C parts' own `inspect` keys (plan B's drivers' `inspect`): [`own`] reads a part's published state and its
channels' latest values. Typed: a `json!` value would widen every f32 to f64 (a reading of 0.774944 would print as
0.774944007396698).

### `pub struct Lookup<'a>`

What an inspect answer reads besides the part's published state.

### `pub enum Own`

An I2C part's own inspect keys (plan B icp10111.py:455-459, bmi323.py:305-317, sdp810.py, ina226.py, bno055.py,
vl53l1x.py, bmm350.py:382-387), in plan B's order.

### `pub fn own(implementation: &str, shared: Option<Shared>, lookup: &Lookup, platform: &dyn Platform) -> Option<Own>`

The own keys of the I2C part `implementation` named in `lookup`, from its `shared` state (a part that came up has
one; one that did not shows its kind's empty state); None for an implementation with no keys here: the
LSM6DSO32 (on I2C it has no run; on SPI it is the control thread's), a BNO055 with no state, any non-I2C part.

## `parts::magnetometer`

_Tested by its own `mod tests`._

The BMM350's run (M9): its `mag` channel and the evidence its calibration is computed from.

### `pub struct MagnetometerState`

What `inspect` and the pending calibrations show of the magnetometer beyond its channel (plan B
bmm350.py:382-387).

- `fn instruction(&self) -> String` — Its pending calibration, in plan B's words (bmm350.calibration()); empty when it is calibrated.
- `fn extrema(&self) -> Option<[i32; 4]>` — The extrema, None before a sample.

### `pub struct Magnetometer`

The BMM350's outputs (M9): its `mag` channel and stream, read every [`drivers::bmm350::POLL_PERIOD_MS`] (twice a
conversion at 12.5 Hz), each fresh conversion observed once for the calibration's evidence (plan B bmm350.run).

## `parts::member`

[`Member`]: a part that came up, as its port's owner keeps it, with its [`Shared`] state: what the owner publishes
and the executor reads (`inspect`, `verify`, the pending calibrations).

### `pub enum Shared`

A part's published state, by kind: the owner writes it, any thread reads it. Each lives as long as the board
(made once, when its part came up).

### `pub struct Member<Pin>`

An I2C part that came up, as its port's owner keeps it.

- `fn new(name: &'static str, part: Part<Pin>, axes: Axes, interrupt: Option<u8>, period_ms: Option<u32>) -> Member<Pin>` — The part named `name` that came up, its state published from now on, no run yet.
- `fn calibrate<Bus: I2c>(&mut self, port: &mut Bus, delay: &mut dyn DelayNs) -> Calibrated where Pin: OutputPin,` — Plan B's `calibrate()`: None on success, else why. The barometers capture their ground again from fresh conversions (plan B icp10111.py:337-344); the pitot captures its tare, owed to NVS; a part with nothing to calibrate succeeds when nothing is pending (plan B's default `calibration() or None`).
- `fn apply_calibration(&mut self, blob: &[u8]) -> Option<String>` — Puts the calibration `blob` in force once the executor persisted it (D9 of the M9 plan): the BMM350's, logged in plan B's words; the BNO055's profile is in force already, so it is only marked saved. None on success, else why.
- `fn update(&mut self, properties: &Value, now_us: u64) -> Result<Vec<&'static str>, String>` — Plan B's `update()`: the changed properties. The barometers take `ground`, and `rezero` from their own altitude channel when fresh (the ICP-10111: three periods); the pitot its zero, owed to NVS; a part with nothing writable changes nothing.
- `fn restore_pitot_zero(&mut self, pascal: f32) -> bool` — The warm start's pitot tare (the crumb's, Pa): put in force without persisting (a flash write mid-air is not wanted). Whether the part is a pitot that took it.

## `parts::part`

[`Part`]: one I2C part's driver, as its bus owner drives it (plan B's task, built per attempt), and [`Run`]: what
it does on its owner's tick beyond its bus work.

### `pub enum Event`

What a tick asks of the owner beyond the part itself.

### `pub enum Calibrated`

What a part's `calibrate` answers on its owner.

### `pub enum Part<Pin>`

One I2C part, as its bus owner drives it; `Pin` is a laser's XSHUT line.

- `fn build(document: &Value, entry: &Entry, address: u8, xshut: impl FnOnce(u8) -> Result<Pin, String>) -> Result<Part<Pin>, String>` — A fresh driver for `entry`, an I2C part ([`crate::Kind::I2c`]) at `address`, its binding's (plan B's task created for one attempt). Its INT and ALERT wires resolve through the board's pins (plan B `_pin_gpio`), its XSHUT through `xshut`, which makes the line's driver.
- `fn setup<Bus: I2c>(&mut self, port: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), (String, bool)>` — Plan B's setup (its bus half): the part from its power-on state to producing data.
- `fn rearm<Bus: I2c>(&mut self, port: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), String>` — What a general call took from the part, restored (never its setup: that would re-zero a barometer's ground and reload the pitot's tare).
- `fn diagnose<Bus: I2c>(&self, port: &mut Bus, id: u8) -> String` — Plan B's diagnose after the last failed attempt: what the part answers at the wire level, on port `id`.
- `fn probe<Bus: I2c>(&mut self, name: &str, port: &mut Bus, delay: &mut dyn DelayNs) -> Option<String>` — Plan B's on-demand self-test (`probe`): the part identifies itself and one reading comes back, logged in plan B's shape (`<name> :: probe: <step> ...`, then `probe: <step> ok <value>` or `probe FAILED: <step>: <why>`). None when healthy, else the failure, as plan B answers. The ICP-10111 and the SDP810 take one reading of their own, where plan B waited for its run loop's (D16 of the M5 plan: the owner cannot race itself). A driver identifies and reads in one call, so a two-step probe logs its steps as one.
- `fn tick<Bus: I2c>(&mut self, name: &str, run: &mut Run, port: &mut Bus, delay: &mut dyn DelayNs, now_us: u64) -> Option<Event>` — One owner tick for the part named `name` (M6: the ICP-10111's two-phase measurement, plan B's run loop), publishing through `run`. Returns [`Event::Rearm`] when the part's recovery asks the owner to re-arm the port's other parts.
- `fn first(&self) -> bool` — Whether the part is an IMU, whose run goes first in each owner wake (D13 of the M9 plan): the lowest latency for the backup IMUs, the BMI323 (v1.x) and the BNO055 (v0.1; a bench board can carry both).
- `fn calibration(&self) -> &'static str` — The calibration the part waits for, in plan B's words (`calibration()`); empty when none. The ICP-10111 asks for its ground until it holds one (setup captures it, so an up ICP never asks); the BMM350's and the BNO055's are worded on the executor from their published state ([`crate::MagnetometerState::instruction`], [`crate::AbsoluteState::instruction`]).
- `fn rezeroes(&self) -> bool` — Whether the part re-zeroes a barometer's ground, as [`crate::rezeroes`] says of its implementation: its `update` and `calibrate` also run at DONE.
- `fn take_general_call(&mut self) -> bool` — Whether the part sent a general call since the last ask: only the ICP-10111 sends one, to free itself.

### `pub enum Run`

What a part does on its owner's tick beyond its bus work: where its readings go (M6: the ICP-10111; M9: the
BMI323). Each variant is boxed, so the other members of a port do not pay for the largest inline (M9).

- `fn new<Pin>(member: &Member<Pin>, topology: &'static Topology, entry: Option<&'static DeviceEntry<'static>>, platform: &'static dyn Platform) -> Option<Run>` — The run `member`'s part needs on its owner's tick, its writers claimed from `topology` for its source name, writing the health of its table `entry`, publishing to its shared state; None for a part with nothing to run.
- `fn stops_at_done(&self) -> bool` — Whether the run stops at DONE (M11b): the IMU, the magnetometer and the pitot; the beacon's sources go on.

## `parts::pitot`

_Tested by its own `mod tests`._

The SDP810's run (M10): the pitot's dynamic pressure and airspeed, and the tare its owner captures for the
executor to persist.

### `pub struct PitotState`

What `inspect` shows of the pitot beyond its channels (plan B sdp810.py inspect).

### `pub struct PitotShared`

The pitot's shared state: its published state (None until first published) and the tare its owner captured that
the executor has not persisted yet -- the NVS write is the executor's, never the flight core's.

- `fn state(&self) -> Option<PitotState>` — Its state, None before its first publish.
- `fn take_owed(&self) -> Option<i32>` — The tare owed to NVS (Pa x 100), taken: None when none waits.

### `pub struct Pitot`

The SDP810's outputs (M10, plan B sdp810.run): `dynamic_pressure` (Pa) and `airspeed` (m/s), read every
`period_ms` (20: 50 Hz) from the part's continuous mode, a row of plan B's integers per read; a bus failure counts
toward the driver's restart ladder, a CRC reject is only noted.

## `parts::power`

The INA226's run (M10): the battery's voltage, current and power, and the over-current ALERT's trips.

### `pub struct PowerState`

The INA226's latest reading for `inspect` (plan B ina226.py inspect).

### `pub struct Power`

The INA226's outputs (M10, plan B ina226.run): `voltage` (mV), `current` (mA) and `power` (mW), read every
`period_ms` (100), a row of plan B's integers per read.

## `parts::published`

_Tested by its own `mod tests`._

[`Published`]: a part's state, written by its owner and read whole by the executor (a seqlock). The owner never
waits: a write is two counter stores around a copy. A reader retries while a write is under way, so it gets one
write's state, never a mix of two. Plan B read its tasks' fields directly: one interpreter, no torn reads.

One writer at a time: the part's owner on its thread, or main before the owners start (bring-up, the restores).
RV32 has no 64-bit atomics, so the counter is a u32; it would have to wrap twice during one read to fool a
reader, and a read takes microseconds.

### `pub struct Published<T: Copy>`

A `T` one writer publishes and any thread reads whole.

- `const fn new(value: T) -> Published<T>` — A slot holding `value`.
- `fn publish(&self, value: T)` — Replaces the value. One writer at a time (see the module notes).
- `fn read(&self) -> T` — The latest value, whole.

## `parts::ranger`

A laser ranger's run (M10): the VL53L1X's or the VL53L4CX's height above the ground.

### `pub struct Ranger`

A laser ranger's outputs (M10, plan B vl53l1x.run): `agl` (m), polled every `period_ms` (50), published only for a
waiting, valid range (the data-ready check plan B lacked; an invalid status is read, cleared, never published).

## `parts::restore`

What each part learned, restored at bring-up from the board's store, so a power cycle does not cost the operator a
turn (plan B's `_restore` in each driver's setup). Once, on main, before the owners run; best effort, as plan B's:
no saved value or a bus error leaves the calibration owed.

### `pub trait Store`

The board's store of calibrations (NVS on the board): read-only here; the executor writes.

- `fn blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, String>` — The blob saved under `key`, into `buffer`: its length.
- `fn integer(&mut self, key: &str) -> Result<i32, String>` — The integer saved under `key`.

### `pub fn restore_profiles<Pin, Bus: I2c>(store: &mut dyn Store, members: &mut [Member<Pin>], port: &mut Bus, delay: &mut dyn DelayNs)`

Restores each BNO055's learned profile (plan B bno055._restore_profile) among `members`, the parts of one port,
written over `port` (CONFIG mode, the profile, NDOF).

### `pub fn restore_saved<'a, Pin: 'a>(store: &mut dyn Store, members: impl Iterator<Item = &'a mut Member<Pin>>)`

Restores what needs no bus among `members`, every port's: the pitot's tare (the store's wins over the config's
`zero_offset_pa`, plan B sdp810.setup, so a reboot mid-glide keeps it), then each magnetometer's hard/soft-iron
calibration (plan B bmm350._restore).

# `sitl` — `src/avionics/sitl`

_Uses: `board_config`, `control`, `controller`, `mission`, `recorder`, `sequencer`, `sim_link`. External: log, serde_json._

## `sitl`

SITL (M14): the Rust flight code on the host, behind the same UDP sim-link the board serves (doc/specs/sim-link.md),
lockstep with the host's physics. tools/hitl_host.py `--sitl` flies it as it flies a board.

The pieces are the board's own, from the same crates: the topology the board's bring-up builds from its config
(crates/controller `topology`, the `hitl` entry its only provider: every sensor it masks is simulated), a SENSORS
frame published into it as the `hitl` source (`HitlWriters`), and the board's control tick itself
(crates/controller `ControlCore`: the attitude filter, the stage machine and the flight law, in the board's order,
R3 of doc/specs/avionics-refactor.md), with the mission from the launch payload the host sends a board. The SITL is
the board's code path minus the platform, never a copy of it. What the host has none of is not here: the servos (a
FINS answer carries the law's angles, as a board's centred fins at its start), the separation switch, the
checkpoint and the warm start, the GNSS drift calibration (no drift), the recorder (no rows).

Lockstep: the clock is the frame's physics time (`sim_time_ms`, after a 10 s boot as the golden shims'), never the
wall's. Each frame first runs the control ticks due before it on what the earlier frames left, then is published,
then runs the tick due at its own time, then is answered: what a realtime board sees, every run the same.

  sitl --config <board config JSON> [--mission <launch JSON>] [--port 1236]

# `firmware` — `src/avionics/firmware`

_Uses: `beacon`, `board_config`, `cc_protocol`, `control`, `controller`, `databoard`, `drivers`, `flash_ring`, `gnss`, `layout`, `mission`, `panics`, `parts`, `recorder`, `recorder_wire`, `sequencer`, `sim_link`, `warmstart`. External: anyhow, base64, embedded-hal, esp-idf-svc, log, serde, serde_json, sha2._

## `firmware`

What the avionics binary and the on-board benches share.

## `firmware`

Avionics bring-up (doc/specs/avionics-port.md "Bring-up order"): the clock, the console's UART, then the boot's
phases (crate::boot: storage, media, ports, layout, the recorder's threads, the board, the devices, the warm
start's recovery, the flight's threads, the operator's side) in order, and the radio forever. A panic anywhere
prints its backtrace and reboots (sdkconfig: PANIC_PRINT_REBOOT).

### `pub fn register_system_commands()`

The system's own console command (the Glider Command Language): `panic kind=... confirm`, which proves a panic
reboots the board, warm-starting a flight in the air (crate::warm), and lands in the panic log
(crate::panics_logger); and that the supervisor's watchdog resets a board whose supervisor stops feeding the task
watchdog (`twdt`) or whose control thread stalls (`stall`). Open in every stage, for the console alone: it is the
one hands-free reset in GLIDING and at DONE (`reboot` runs only in SETTING), and never one the hub can send. `twdt`
and `stall` are refused when nothing would reset the board: a stalled control thread could not even be disarmed.

## `firmware::allocator`

The heap allocator: std's own (IDF's malloc), except that a thread whose stack lives in PSRAM allocates in PSRAM.

Internal RAM is the P4's scarce memory (doc/specs/avionics-port.md §4.7): every byte of it is DMA-capable, and
esp_hosted asserts and reboots the board when it gets none for a Wi-Fi frame. IDF keeps every malloc of 4 KB or
less internal (SPIRAM_MALLOC_ALWAYSINTERNAL), and a parsed JSON document is thousands of such allocations: one
`set-config board` of a 7 KB config took the internal heap from ~70 KB free to 20 bytes, on the CC executor.

The slow core-0 threads (the CC executor, the console, the supervisor, the file server) already run on PSRAM stacks
(they never call esp_mmu_map or esp_mmu_unmap, the one runtime path that turns the cache off). The stack pointer is
the marker: a Rust allocation made on a PSRAM stack goes to PSRAM, and falls back to std's path when PSRAM has none
(a fallback only while CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS stays off, IDF's default: with it a full PSRAM
aborts). Everything else -- the flight and ring threads, boot on main, interrupts -- allocates exactly as before.
C code (NVS, FatFs, esp_hosted) calls malloc itself and is untouched.

What such a thread needs in internal RAM it asks for by name: [`InternalBuffer`] (a flash read's destination).

It also counts the Rust allocations made on core 1, the flight core ([`flight_core_allocations`]): an allocation
there takes the heap's lock, which core 0 shares, so a tick that allocates can wait on core 0's malloc. The
control thread's `control` command reports the count, which should stop growing once the bring-up is over.

### `pub struct Allocator`

std's allocator, with PSRAM for the threads whose stack is in PSRAM.

### `pub struct InternalBuffer`

A zeroed byte buffer in internal RAM, whatever the calling thread's stack.

For a destination that must be internal on a PSRAM-stack thread: esp_flash_read fills internal RAM directly, and
reads into any other memory through an internal bounce buffer it allocates per call, after walking the internal
heap for its largest block with core 0's interrupts masked -- thousands of times per ring pull, and a failed
allocation fails the read.

- `fn zeroed(length: usize) -> Option<Self>` — `length` (not zero) zeroed bytes of internal RAM; None when the internal heap has no such block.

### `pub fn flight_core_allocations() -> u32`

The Rust allocations made on core 1, the flight core, since boot (the control thread, the I2C owners).

## `firmware::beacon`

The recovery beacons (M11b, M20; doc/specs/avionics-port.md), the page and the report shared with the remote
recorder through crates/beacon:

- This board's own: at DONE the link thread, the radio's one owner, turns the C6 co-processor from a station into
  the open access point `<BOARD>_glider` on channel 3 at the maximum TX power, a captive portal serving one page
  for a searcher's phone ([`Beacon::start`]): the last good fix with a map link, the elevation from every source
  (the GNSS, each barometer, the laser: how high the glider hangs, should it land in a tree), the battery where an
  INA226 is fitted, and the flash->SD copy still to go. Power-off or a reset ends it; any power-up after DONE is a
  new session in SETTING.
- The remote recorder's (src/avionics/recorder, an ESP32-C6 on the recorder's UART, M20): every [`REPORT_EVERY`] the
  beacon_feed thread writes the same page's figures and that beacon's radio to beacon.csv, an ordinary recorder
  stream ([`spawn_feed`]). The recorder replays the last row: `<BOARD>_beacon` three channels above this board's own
  (operator 2026-10-11), a minute on and a minute silent. It comes on BEFORE launch, with the camera: armed in
  SETTING and standing on the rod (crates/sequencer `video::Trigger`, the same look), or any stage past SETTING (a
  launch, a warm start in the air, `stage done`); then it stays on until power-off, so a glider that dies in flight
  owes it nothing. Until then its name is empty, which keeps it off: a new session in SETTING turns it off. Never
  in a HITL flight, whose frames ride this board's Wi-Fi.

The figures are read from the databoard's channels at each request and each report, every source's latest value
whatever its age (a fix lost in the trees is still the best guess), with that age.

### `pub struct Beacon`

The beacons of one board.

- `fn new(board_id: &str, topology: &'static Topology, battery_mah: Option<f32>) -> Beacon` — The beacons of board `board_id` reading `topology`, its battery `battery_mah` when the config gives one.
- `fn start(&self, wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<EspHttpServer<'static>>` — This board's access point up on `wifi` (the station dropped) and the page served: the server lives as long as the board.
- `fn page(&self) -> String` — The page, read now (also on the file server as `/beacon`: the bench sees it before DONE).

### `pub fn spawn_feed(beacon: Beacon) -> anyhow::Result<()>`

The beacon_feed thread (core 0, priority 2, a PSRAM stack: it never touches the flash): a look for the rod every
[`LOOK_EVERY`], and a report to beacon.csv every [`REPORT_EVERY`] and at once when the remote beacon comes on,
forever (see the module notes). Without a recorder, or with its stream unregistered, there is nothing to feed.

Errors: the thread would not spawn.

### Constants

- `const REPORT_EVERY: Duration = Duration::from_secs(5)` — How often the beacon_feed thread writes a report (and at once when the remote beacon comes on).
- `const STREAM: &str = "beacon"` — The recorder's device key the beacon.csv stream is registered under (crate::recorder `device_stream`).

## `firmware (binary bench)`

Phase 6 on-board benchmarks, the software-only part: B1 CPU, B2 FPU, B3 memory, B5 threads and pacing
(scratchpad plan, section 2.1). Every result is one console line,
`bench;<group>;<name>;<value>;<unit>`, which `tools/avionics_console.py --reset --wait 60` collects.

Short intervals are timed with the RISC-V `cycle` counter (the HP core clock, 360 MHz on rev v1.x; it
wraps every ~12 s, so nothing timed with it runs that long); longer ones with esp_timer (1 us).

## `firmware (binary cases)`

Our own flight code, case by case, against MicroPython: each case is a faithful port of the src/glider
function it is named after, with the same inputs as src/glider/test/bench_cases.py, which times the real
MicroPython modules. Built once per rustc flag set (doc/benches/avionics-20261005/README.md), so the
table compares languages AND optimisation levels. Prints `cases;<name>;<nanoseconds per call>;ns`.

Timing is best-of-5 runs (the one fewest interrupts landed in), loop and call included -- on the
MicroPython side the call overhead IS part of what a call costs, so it is included there too.

## `firmware (binary dma)`

B3b: PSRAM bandwidth as DMA sees it -- the camera, ISP and H.264 encoder move frames with DMA, not the CPU
(CPU copies are latency-bound: bench.rs B3, 57 MB/s for 1 MB). Times AXI-GDMA async memcpy over every route
at three burst sizes; prints `dma;<route>_burst<n>;<MB/s>;MB/s`.

The completion callback runs in an ISR, so it only bumps an atomic counter -- integer only (a float in an ISR
aborts the chip).

## `firmware (binary flash)`

B4: can the on-board flash take telemetry in flight without stalling the control loop? Runs on the `log`
partition (partitions.csv) and prints `flash;<group>;<name>;<value>;<unit>` lines:
 * latency of a 4 KB sector erase, a 64 KB block erase and a 256 B page program, and the sustained program
   rate into erased flash;
 * the stall test: a 1 kHz loop on core 1 (esp_timer -> task notify, priority 20, executing ordinary code)
   records how late each tick wakes while core 0 erases and programs continuously -- and the same loop with
   the flash idle, as the reference;
 * the time to erase the whole partition (the pad's pre-erase).

The build's sdkconfig decides which variant is measured (XIP from PSRAM, the default since B4; code in flash
with CONFIG_SPIRAM_XIP_FROM_PSRAM=n): the `meta;xip_from_psram` line says which.

## `firmware (binary isa)`

What the ESP32-P4 HP core really implements, and what Rust gets out of it: ISA extensions, atomics and
locks (internal RAM and PSRAM, across cores), and the PIE SIMD / hardware-loop kernels of esp-dsp.
Results print as `isa;<group>;<name>;<value>;<unit>`.

Each extension probe executes one instruction by its raw encoding (so the assembler needs no feature
flag) and checks the result. An unimplemented one traps and the chip reboots; a crumb in RTC no-init
memory names the probe that was running, and the next boot reports it as `trapped` and carries on.

## `firmware (binary peak)`

Peak throughput of the ESP32-P4 HP cores, the headline numbers: EEMBC CoreMark (Espressif's port, run as
shipped), f32 / f64 FLOPS, 32-bit integer OPS, the PIE SIMD int16 MAC rate through esp-dsp, and streaming
memory bandwidth -- each on one core, then on both cores at once. Results print as
`peak;<group>;<name>;<value>;<unit>`; CoreMark prints its own report first.

The arithmetic kernels keep eight (or sixteen) independent accumulator chains, so the pipeline waits on its
own results as little as possible: that is throughput, unlike bench.rs, which times dependent (latency-bound)
operations. Eight f32 chains do not hide the FPU latency; sixteen reach ~1 FMA per 1.44 cycles.

## `firmware (binary ring)`

G1 (doc/specs/avionics-port.md, risk R1): does the GD 0xC84019 NOR take many small programs into one 256 B
page? The flash ring appends 40-200 B frames at 8-byte-aligned offsets, one esp_partition_write_raw each, so a
page is programmed several times over (only 1 -> 0 bit changes into erased bytes). If the part cannot, frames
must be padded to whole pages (about twice the flash per second). Runs on the `log` partition above 16 MB (the
4-byte-addressed half) and prints `ring;<group>;<name>;<value>;<unit>`:
 * frames: 10 000 frames of 40-200 B, each header + payload in ONE program at an 8-aligned offset, then every
   byte read back -- the frames and the 0xFF gaps between them;
 * dense: 32 programs of 8 B into each of 64 pages, read back (the worst case the ring can produce);
 * the per-frame program latency spread and the 64 KB read that checks a block is erased before use.

## `firmware (binary sd)`

S1/S2: the microSD on SDMMC slot 0 (shared host with the C6 on slot 1). Mounts it (never formats), prints the
card, then writes 32 MB per chunk size from an internal-RAM buffer and from a PSRAM buffer, timing every write
(the worst case sizes the PSRAM ring in front of the card) and the sustained rate including the final fsync,
and repeats into a preallocated contiguous file (the raw-like path). Reads a sample back to verify. Touches
only its own files under /sdcard/COLUDO and removes them. Prints `sd;<group>;<name>;<value>;<unit>`.

## `firmware (binary soak)`

Thermal soak: how hot does the die get with both HP cores at full load, and how fast? 10 min of f32 FMA
on both cores (the peak.rs kernel), then 3 min idle, the die temperature printed every 5 s as
`soak;<seconds>;<celsius>;<load|idle>`, and a summary. Run it bare and with a heatsink to compare; the
camera heat test (V8) repeats it with the video pipeline running.

The burners sleep 1 ms every ~100 ms, so the idle tasks still run and the task watchdog stays quiet
(~1% duty lost).

## `firmware::board`

Who this board is and what it runs on: the id, this boot's count, the persistent store, the vitals.

### `pub struct Board`

This board: its id, this boot's id, its vitals.

- `fn new(id: String, boot_id: Option<i32>, session: String, thermometer: Option<Thermometer>, sd_mounted: bool) -> Self` — The board of this boot: `boot_id` from [`count_boot`], which ran first because the boot id names the recorder's files, and the `session` prefix those files carry.
- `fn celsius(&self) -> Option<f32>` — The die temperature in °C, rounded to 0.1 (what health reports).
- `fn banner(&self) -> String` — One line naming the firmware, the silicon and this boot -- the first thing the console prints.

### `pub fn count_boot(partition: EspDefaultNvsPartition) -> anyhow::Result<Option<i32>>`

Count this boot: NVS `coludo`/`boot` + 1 in the namespace plan B shares, committed before anything records
(src/glider main._next_boot), since the boot id names the recorder's files.

Only a key that was never written starts the count at 1. Any other read error says nothing about how far
the count got, and restarting it would reuse ids -- so it, and a count the store will not take, give None.

Errors: the namespace would not open (the `nvs` partition is missing or unusable).

### `pub fn uptime_ms() -> i64`

Milliseconds since boot.

### `pub fn heap_free(capabilities: u32) -> usize`

Free heap bytes with the given capabilities (internal RAM, PSRAM).

### `pub fn reset_reason() -> &'static str`

Why the chip last reset, as IDF names it: the banner, the provenance line and the recorder's session-start
record carry it.

### Constants

- `const FIRMWARE_VERSION: &str = concat!("avionics-", env!("CARGO_PKG_VERSION"))` — The firmware's name on the wire: the hub shows it as the board's version.

## `firmware::boot`

The bring-up's phases (doc/specs/avionics-port.md "Bring-up order"; doc/specs/avionics-refactor.md R4), in the
order `main` runs them, each documented with what it needs and what it gives:

1. [`storage`]: the boot count and the board config, which name this boot's records; the recorder opened before
   anything else logs, in flight when NVS holds a crumb in BOOSTING..LANDING; the last boot's panic filed;
2. [`media`]: the card (one FAT volume, or two: the recordings and the videos), the camera encoder's internal block
   reserved before anything else splits it, the oops thread;
3. [`ports`]: the I2C ports, each created by its owner thread on core 1 and lent to main;
4. [`layout`]: the board's revision by the ports' scan, which rewrites the config and so names its id; the HITL
   mask; the ports at the rewritten config's clocks;
5. [`recorder`]: the ring tasks (the writer, the eraser, the saver that copies the flash to the card) and the
   uart_saver;
6. [`board`]: the board's identity and the provenance line;
7. `devices::bring_up`: the devices in config order on the lent ports;
8. [`recovery`]: the warm start's restore, which may commit a flight through the flight word's writer and rebase the
   barometers while main still holds both; the calibrations; each port back to its owner with its parts;
9. [`flight`]: the fins, the control thread (which takes the writer), the supervisor, the sim-link, the GNSS;
10. [`operator`]: the commands, the camera's thread, the CC executor, the UART console, the card's entry file;
11. [`radio`]: the beacons, then Wi-Fi through the C6 and the CC hub, forever.

### `pub type Lent = Vec<(owner::Parked, I2cPort)>`

The ports main lends the bring-up: each one's parked owner and the port.

### `pub struct Storage`

What [`storage`] gives the boot.

### `pub struct Media`

What [`media`] gives the boot.

### `pub struct Recovered`

What [`recovery`] gives the boot.

### `pub fn checkpoint(phase: &str)`

Logs the internal RAM left after the boot phase `phase`: free, and the DMA-capable part of it. One line a phase,
so each feature's price in internal RAM shows in the boot log where it was paid (doc/specs/avionics-port.md §4.7).

### `pub fn storage(partition: EspDefaultNvsPartition) -> anyhow::Result<(Storage, board_config::Loaded)>`

The boot count, the board config and the recorder, needing only NVS (`partition`): every fin pin held low, the
recorder opened (every line and row kept from here, queued in PSRAM until the ring_writer runs), the config's
lines, and the last boot's panic filed before anything here can panic again.

Gives the boot's storage, and the board config as it loaded, which the layout rewrites before the run freezes it.

Errors: NVS would not count the boot, or the settings partition would not open.

### `pub fn media(document: &serde_json::Value, temp_sensor: TempSensor<'static>) -> Media`

The die's thermometer (`temp_sensor`), the card, and the camera of `document`: the encoder's ~138 KB internal block
reserved before the ring threads' stacks and buffers split it (no camera or no card: no recording, no
reservation); then the oops thread, before the bring-up records a general call as an oops.

### `pub fn ports(document: &serde_json::Value) -> Lent`

The I2C ports of `document`, each created by its owner thread on core 1 and lent here: port 0 always (the camera's
SCCB is wired to it), port 1 when the config defines it. A port that will not come up is logged and left out.

### `pub fn layout(document: &mut serde_json::Value, lent: &mut Lent) -> layout::Resolved`

Which board this is, by the scan of the `lent` ports, before anything reads `document` for its devices (plan B
main.py:91-104): the config is rewritten in place, and from here on it is the one this boot runs. Then the HITL
mask, and the ports at the rewritten config's clocks. Gives the layout's verdict.

### `pub fn recorder(document: &serde_json::Value, uart1: UART1<'static>, settings: EspCustomNvsPartition, card: Option<recorder::Card>, config_id: &str)`

The recorder's threads for `document` under `config_id`: the uart_saver (M19: the wire lines to a receiver on
`uart1`, before the saver that feeds it starts), then the ring_writer, which scans the `log` partition in its own
thread and starts the ring_eraser and, with a `card`, the ring_saver -- all after the encoder's reservation. Boot
goes on, and the producers queue meanwhile.

### `pub fn board(storage: &Storage, loaded: &board_config::Loaded, config_id: &str, thermometer: Option<Thermometer>, sd_mounted: bool) -> Arc<board::Board>`

The board of `storage` running `loaded` under `config_id`, with the media's thermometer and card, and the
provenance line (plan B main): the build and the config this capture comes from, stamped into the capture itself;
the saved config's id last, the one `set-config board` answered.

### `pub fn recovery(storage: &Storage, document: &serde_json::Value, devices: &mut devices::Devices, lent: Lent) -> Recovered`

The warm start, while the parts and the flight word's one writer are still main's and nothing else runs: a reset
in flight resumes the crumb's stage, its barometers rebased to the pad and the mission's launch point and zone
restored (doc/specs/avionics-port.md "Bring-up order"; crate::warm). Then the parts' calibrations from NVS, as plan
B's setups restored them, and each `lent` port back to its owner with the parts of `devices` that came up on it.

### `pub fn flight(document: &'static serde_json::Value, devices: devices::Devices, storage: &Storage, board: Arc<board::Board>, writer: FlightWriter<'static>, seed: Option<warmstart::Identity>, uart2: UART2<'static>, topology: &'static devices::Topology, table: &'static controller::Table<'static>)`

### `pub fn operator(context: commands::Context, credentials: &config::Credentials, console: Option<crate::console::Console>, camera: Option<firmware::video::Camera>, sd_mounted: bool) -> anyhow::Result<commands::Executor>`

The operator's side: each component's commands (the Glider Command Language: each registers its own as it
activates, so `help` lists exactly what this board runs), the card's commands and the camera's one owner thread
when the card mounted, the CC executor over `context`, the UART `console` (it drains the UART before taking it
over, so the lines above survive), the banner, and the card's entry file (M21) once bring-up is done. Gives the
executor.

Errors: the executor or the console would not start.

### `pub fn radio(document: &serde_json::Value, board: &board::Board, topology: &'static devices::Topology, modem: Modem<'static>, event_loop: EspSystemEventLoop, partition: EspDefaultNvsPartition, credentials: config::Credentials, executor: &commands::Executor) -> !`

The radio, forever: the recovery beacons (M11b, M20: the battery's capacity from the INA226 entry of `document`,
when the config gives one; the remote recorder's report to beacon.csv every few seconds, its radio off until
DONE), then Wi-Fi through the C6 and the CC hub (crate::link).

## `firmware::buses::i2c`

The I2C ports, on IDF's new `i2c_master` driver (doc/specs/avionics-port.md M5, design decisions). Each port is
created exactly once, by its owner thread on core 1 (crate::owner in the binary), so its interrupt runs on core 1;
a second `i2c_new_master_bus` on a live port would tear its pins down (i2c_master.c:1058, 1170-1173). Port 0
(SDA 7 / SCL 8) also carries the camera's SCCB, whose own 400 kHz device handle esp_video adds to the bus
published here ([`port0`]). The legacy driver (esp-idf-hal's I2cDriver) must never touch either port.

[`I2cPort`] is plan B's i2cbus.Bus on that driver: an `embedded_hal::i2c::I2c` for the drivers crate, one device
handle per address at the port's speed, and plan B's wedge rule -- 4 consecutive failures across the port, cured,
counting on from zero (i2cbus.py:22, 138-184). What counts is a wire failure: `ESP_ERR_INVALID_STATE` (a NAK, an
SCL timeout, lost arbitration; the driver cannot tell them apart). A bus-lock timeout (`ESP_ERR_TIMEOUT`: the
camera can hold the lock for its 500 ms SCCB timeout) is not the bus's fault, nor is a caller error, a probe or a
scan. The cure is `i2c_master_bus_reset` (an FSM reset and 9 SCL pulses); the handles survive it, and are made
again only when the live speed differs from the configured one (after a bustune), as plan B's `reclock()`.

### `pub enum I2cError`

Why an I2C transaction failed.

### `pub struct PortFigures`

A port's counters, for `i2c touch`, `bustune` and `inspect`.

### `pub struct I2cPort`

One I2C port: its bus, its device handles and plan B's wedge counter.

- `fn create(id: u8, sda: u8, scl: u8, clock_hz: u32) -> anyhow::Result<I2cPort>` — Creates port `id`'s bus on pins `sda`/`scl` at `clock_hz`, with plan B's MicroPython settings: the XTAL clock, a glitch filter of 7, internal pull-ups on (beside the board's), the synchronous driver (async refuses probes). Call it on the port's owner thread (core 1), once: the bus's interrupt stays on the creating core. Port 0's handle is published for the camera ([`port0`]).
- `fn id(&self) -> u8` — The port number.
- `fn configure(&mut self, clock_hz: u32) -> Result<(), I2cError>` — Sets the speed the running config asks for (the post-layout `freq`), and runs at it.
- `fn retune(&mut self, clock_hz: u32) -> Result<(), I2cError>` — Runs every handle at `clock_hz` (a bustune rung; not saved: [`I2cPort::configure`] or a wedge cure restores the configured speed). Each handle is added again at the new speed, then the old one removed: IDF has no speed change in place.
- `fn probe(&mut self, address: u8) -> Result<bool, I2cError>` — Whether `address` acknowledges (a START, the address, a STOP, at IDF's fixed 100 kHz). Never counted toward a wedge, as plan B's scan was not (layout.py:191-213).
- `fn scan(&mut self) -> Result<layout::Addresses, I2cError>` — What acknowledges on the port, probed as MicroPython's `scan` probes ([`SCAN_ADDRESSES`]), for the layout (plan B layout.py `_scan`). An address whose probe fails reads as absent, as MicroPython's scan reads it, so one glitch or long clock stretch costs one address, not the bus's votes. Never counted toward a wedge.
- `fn general_call_reset(&mut self) -> Result<(), I2cError>` — Sends the general call's software reset (0x00, 0x06): every part that honours it resets. Counted toward a wedge, its NAK included (nothing on the bus honours it), as plan B's `Bus.writeto` counted it (i2cbus.py:228-243). The caller re-arms the port's members (crate::owner).
- `fn take_general_call(&mut self) -> bool` — Whether a general call went out on this port since the last ask (from any caller: a driver's own call too).
- `fn restart_run(&mut self)` — Starts the failure run from zero (plan B test_i2cbus.py:55), so `i2c touch` counts toward a wedge from a known place.
- `fn figures(&self) -> PortFigures` — The port's counters.

### `pub fn port0() -> anyhow::Result<sys::i2c_master_bus_handle_t>`

Port 0's master bus, for the camera's SCCB (crate::video): created by port 0's owner at boot, never here.

Errors: the owner has not created it (yet).

### Constants

- `const TRANSFER_TIMEOUT_MS: i32 = 50` — The transfer timeout, ms: plan B's MicroPython passes 50 (machine_i2c.c:36).
- `const WEDGE_FAILURES: u32 = 4` — Consecutive failures across a port that make a wedge (plan B i2cbus.py:22 `_BUS_FAIL_LIMIT`).
- `const GENERAL_CALL: (u8, u8) = (0x00, 0x06)` — The general-call address and its software-reset command (I2C spec 3.1.14).

## `firmware::buses`

The buses the parts hang off (doc/specs/avionics-port.md M5): the I2C ports, each created by its owner thread on
core 1 (crate::owner in the binary), and SPI2, the IMU bus (M6's control thread owns it).

## `firmware::buses::spi`

SPI2, the IMU bus (doc/specs/avionics-port.md M5, design decisions): a thin `spi_master` wrapper. The config's
`spi` bus 1 is SPI2_HOST (MicroPython's `SPI(1)`); plan B ran it on SCK 48 / MOSI 47 / MISO 46 at 5 MHz, mode 3.

- **No DMA:** the data goes through the 16-word buffer (64 B a transaction), so no transfer allocates or syncs
  the cache (with DMA, every unaligned buffer would, spi_master.c:1186-1262), and buffers may be anywhere.
- **Polling, no interrupt flags, no callbacks:** `spi_device_polling_transmit` busy-waits on the caller's core, no
  ISR and no context switch (spi_master.c:1424-1517); the interrupt the first device allocates is pinned to core 1
  (`isr_cpu_id`) should a queued transfer ever be used. There is no Rust `post_cb` (esp-idf-hal's SpiDeviceDriver
  always installs one), so nothing of ours runs in an interrupt (the §4.7 rule for this bus).
- **One full-duplex chip-select frame per transaction:** the P4 has no half-duplex transfer that both writes and
  reads, so a register read is one 1+N byte frame (the drivers set the read flag themselves).
- **Discard on first use (plan B spibus.py:25, 44-75):** on plan B's bus the first 1..11 framed reads after a
  peripheral appeared returned 0x00, so a handle's first transaction first clocks 16 framed reads of register 0
  (side-effect-free on both parts) and drops them, errors too: a resync, never a probe.

Its owner is M6's control thread; in M5 the SPI parts are set up on main and held.

### `pub enum SpiError`

Why an SPI transaction failed.

### `pub struct SpiBus`

SPI2, initialised for the program: never freed, so never opened twice (a second open is refused).

- `fn open(specification: SpiSpecification) -> anyhow::Result<SpiBus>` — Initialises SPI2 on the config's `buses.spi.1` (plan B spibus.py:131-147): no DMA, a 64-byte transfer limit, master, no interrupt flags, the interrupt pinned to core 1.
- `fn handle(&self, chip_select: u8) -> anyhow::Result<SpiHandle>` — A device handle for chip select `chip_select`, at the bus's mode and its configured clock (see [`SpiBus::handle_at`]).
- `fn clock_hz(&self) -> u32` — The bus's configured clock, Hz (`buses.spi.1.baud`).
- `fn handle_at(&self, chip_select: u8, clock_hz: u32) -> anyhow::Result<SpiHandle>` — A device handle for chip select `chip_select` at `clock_hz` (a bustune rung), at the bus's mode, full duplex, one queued transaction (polling never queues). A chip select holds one handle at a time: drop the old one first -- a second device on the pin takes it over, and removing the first then resets the pin under the second (spi_common.c:763-791).

### `pub struct SpiHandle`

One SPI2 device: its handle, and whether its first-use discard ran.

- `fn chip_select(&self) -> u8` — The chip-select pin.

### `pub fn hold_high(chip_select: u8) -> anyhow::Result<()>`

Holds chip select `chip_select` high (inactive): an enabled SPI part with no driver yet, whose floating chip select
could make it fight the IMU on MISO (plan B spibus.py:42: chip selects idle high).

Errors: the pin would not configure.

### Constants

- `const DISCARD_READS: usize = 16` — Framed reads discarded on a handle's first use (plan B spibus.py:25 `_RESYNC_READS`).
- `const FRAME_BYTES: usize = 64` — The most bytes one transaction moves without DMA: the peripheral's data buffer.

## `firmware::clock`

The board's wall clock: Unix seconds (UTC) from the system time, which survives a software reset (IDF keeps it
in RTC memory) but not a power cycle.

A cold board starts the clock at 2000-01-01 plus its uptime, the way MicroPython's RTC does: the hub reads a
whoami `epoch` inside 2000 as "unset" and sets it from its own clock with `update mission {epoch}`, in SETTING
only (src/control/server.py `_sync_clock`). A board left at 1970 would never be synced.

### `pub fn start()`

Start the clock at 2000-01-01 plus the uptime when nothing set it since power-on.

### `pub fn epoch() -> i64`

Now, in Unix seconds.

### `pub fn set(epoch: i64)`

Set the clock to `epoch` Unix seconds.

### `pub fn text() -> String`

Now as `YYYY-MM-DDTHH:MM:SS` (UTC), what the dashboard shows.

### Constants

- `const UNSET_BEFORE: i64 = 946_684_800` — 2000-01-01T00:00:00Z in Unix seconds: a clock before it was never set (src/glider mission `_EPOCH_OFFSET`).

## `firmware::commands::config`

get-config / set-config / reset-config -- the named configs (src/glider cc_client `_register_config`).

- `board`/`running`: the config this boot runs (immutable for the run: a change is save + reboot).
- `saved`: what the next boot loads -- the saved board.config through the same load the boot uses, so a
  read-modify-write starts from it and never reverts a save not yet rebooted.
- `default`: the built-in default.
- `launch`: launch.config, merge-applied into the mission and saved.

### `pub fn get_config(message: &Message, context: &mut Context) -> Reply`

`get-config [name]`: board/running, saved, default, or launch (default `board`).

### `pub fn set_config(message: &Message, context: &mut Context) -> Reply`

`set-config <name> <json>`: `board` validates and saves the whole document (it runs from the next boot) and
replies its config_id; `launch` merges into the mission and is saved.

### `pub fn reset_config(_message: &Message, context: &mut Context) -> Reply`

`reset-config`: forget the saved board.config, so the next boot runs the default.

## `firmware::commands::control`

stage / arm / disarm / report -- the stage hold and the actuation gate (src/glider cc_client
`_register_control`, controller.hold/resume/arm/disarm).

The flight word has one writer, the control thread: `stage`, `arm` and `disarm` post their request to the
executor's request slot (crate::control::request), which the next tick applies by plan B's Controller rules, and
log the controller's lines from its answer, in plan B's order.

### `pub fn stage(message: &Message, _context: &mut Context) -> Reply`

`stage` reports the stage; `stage <name>` forces it and holds it (a ground test), except that `stage setting`
is a return to the ground, not a hold; `stage auto` releases a hold. Replies {stage, manual}, the flight as the
control thread committed it.

A hold logs `controller :: stage -> <stage>` to the console only, then `controller :: stage <stage> (held|auto)`,
even when the stage did not change, as plan B's `hold` logs through `set_stage`; a release logs `controller ::
stage auto`. A name that is no stage is refused before anything is posted.

### `pub fn arm(_message: &Message, context: &mut Context) -> Reply`

`arm`: enable actuation only when nothing is wrong (plan B cc_client.py:547-576); replies {armed}, the arming the
control thread committed, pins the live GNSS fix as the launch point when the operator set none, and logs
`controller :: armed`.

Refused, `err unsafe` with every problem as {name: why}: each device that is not up (an entry the firmware cannot
run yet included, plan B's rule), the stage problem (off the ground, or held), and in SETTING each failing probe of
the sweep: the mission, the parts, the recorder's ring gate. A request that waited on the board until the hub may
have given up on its answer is refused under `arm` (controller::stale_arm), checked last, after the sweep. The
control thread checks the stage again when it applies the arm, since a launch may be detected during the sweep:
refused then under `stage` alone.

### `pub fn auto_arm(context: &mut Context) -> Result<(), String>`

The field agent's arm (commands::field): the CC `arm`'s passive gates -- every device that is not up, the stage
problem, the mission's probe and the recorder's ring gate -- and never the probe sweep, which moves the fins (the
operator, 2026-10-11); then the same commit as `arm`.

Errors: the problems as `{name: why}` JSON text, or the control thread's refusal.

### `pub fn disarm(_message: &Message, _context: &mut Context) -> Reply`

`disarm`: actuation off -- always allowed, it is how the operator recovers. Replies {armed}, the arming the control
thread committed (always false).

### `pub fn report(_message: &Message, _context: &mut Context) -> Reply`

`report`: the controller's stats (src/glider Controller.stats): the stage and the per-task snapshots.

## `firmware::commands::diagnostics`

probe / verify / calibrate / detect / bustune -- the self-tests and the pre-flight check (src/glider cc_client
`_register_diagnostics`). The I2C parts are probed on their bus owners (crate::owner), the SPI IMU on the control
thread; the separation switch, the bluetooth stub and the recorder's flash ring here, on the executor. `verify`
answers the hardware `pass` and, apart, plan B's flight-readiness verdict over the config (controller::readiness).

### `pub fn probe(message: &Message, context: &mut Context) -> Reply`

`probe [name|all]`: {name: null | problem} for the objects with a self-test, as plan B answers it
(cc_client.py:729-780). `probe all` lists every device that came up (null, or its crash), every one that never
did (its failure line) and the recorder (its ring gate), and, when there are alternatives, their not-fitted lines in
a second map; `probe <alternative>` answers null for it, then that map.

### `pub fn probe_problems(context: &Context) -> Vec<(&'static str, String)>`

The failing probes of every part with a self-test and of the recorder's ring, named as plan B's verify and arm name
them (`_named`), and a thread that did not answer, under its name.

### `pub fn verify(_message: &Message, context: &mut Context) -> Reply`

`verify`: the pre-flight check (plan B cc_client.py:782-845) -- every device's line; the problems: the failures (an
entry the firmware cannot run yet included), the stage problem, and in SETTING the probe sweep with the mission's
and the ring gate's (elsewhere `probe: not run`); `pass` when there is none; and apart from them the config's
flight-readiness verdict (`ready`, `readiness`), which bench and HITL sessions trip on purpose.

### `pub fn probe_mission(context: &Context) -> Option<String>`

The mission's self-test (a launch position is set, the zone is in range), logged as plan B's mission.probe logs
it, so an operator's log window shows the check and its verdict.

### `pub fn calibrate(message: &Message, context: &mut Context) -> Reply`

`calibrate`: what needs calibrating, {name: instruction} (plan B `Inspector.calibration_all()`, possibly {});
`calibrate <device>` runs one, in SETTING (a barometer's also at DONE: the false launch's recovery), answering
{device: null} on success, else why (plan B cc_client.py:873-904). A device that came up with nothing to calibrate
succeeds (plan B's default); one that never came up, or no device, has no calibrate.

### `pub fn detect(_message: &Message, context: &mut Context) -> Reply`

`detect`: scan both I2C buses, on their owners, and answer the revision they look like beside the one this boot
applied, changing nothing (plan B cc_client `detect`): what is wired now, which takes effect only at the next
boot. Scans with the running (post-layout) config's bus specs, as plan B scans the controller's.

### `pub fn bustune(message: &Message, context: &mut Context) -> Reply`

`bustune <kind> <id> <freq>`: run sensor bus `<kind>:<id>` at `<freq>` Hz in place, never saved (the CC sweep
saves the chosen clock with set-config and a reboot), and answer each part's probe (plan B
controller.bustune, cc_client.py:855-870): {kind, id, freq, devices: {name: 'ok' | '<driver> -- <why>'}, all_ok}.

The verdicts are the parts that came up on that bus, probed now (operator, 2026-10-07): plan B also counted every
other config entry on the bus as down -- disabled ones included, and here the parts whose drivers are not ported
yet -- so its sweep never found a ceiling on a bus with one.

## `firmware::commands::field`

The CC-less field agent on the executor (M11; crates/mission field, plan B tasks/field.py). The executor owns the
mission the agent changes, so it ticks the agent itself, every `period_ms`, between command lines: the site by
GPS on the first fresh fix, and the auto-arm once the dwell is complete. The auto-arm keeps the CC `arm`'s passive
gates (the operator, 2026-10-11): the devices, the stage, the mission and the recorder's ring gate, and not the
probe sweep, which would move the fins with nobody there. A refusal logs every problem and restarts the dwell.

### `pub struct Agent`

The field agent once its entry came up.

- `fn new(member: FieldMember) -> Agent` — The agent of `member`, its first pass a period from now (plan B sleeps before its first tick).
- `fn wait(&self) -> Duration` — How long until its next pass.

### `pub fn tick(context: &mut Context)`

The agent's pass when it is due: the decisions, their log lines, and the auto-arm through the passive gates.

## `firmware::commands::identity`

whoami / ping / health -- who the board is and how it is doing (src/glider cc_client `_register_identity`).

### `pub fn whoami(_message: &Message, context: &mut Context) -> Reply`

`iam <id> <json>`: who the board is -- the one reply carrying the id.

### `pub fn ping(_message: &Message, _context: &mut Context) -> Reply`

`pong`.

### `pub fn health(_message: &Message, context: &mut Context) -> Reply`

`ok <json>`: the heartbeat vitals. The stage, the arming and the hold come from one snapshot of the flight word.

## `firmware::commands::inspection`

objects / inspect / update / stats -- the operator-facing objects (src/glider cc_client `_register_inspection`,
inspector.Inspector). The registry: the controller (the flight state), the mission, the recorder (with the flash
ring and the ring_saver) when it opened, and every device that came up (plan B registers each task as it comes
up), whose snapshot is plan B's `Task.inspect` -- name, ok, healthy -- and, for the parts the port drives so far,
their own keys read off the databoard (the IMU, the barometer, the separation switch), null once stale. The
bluetooth stub (drivers::bluetooth) adds plan B's `radio`, which `update` changes, and `active`, always null. The
checkpoint adds the crumb NVS holds now, this boot's gate, the NVS action it owes and its tallies (crate::warm);
the watchdog its settings, whether the task watchdog is armed, whether the board controls a flight, the control
tick's age, the longest gaps between feeds and between stall checks, and whether its restart is suppressed.

### `pub fn objects(_message: &Message, context: &mut Context) -> Reply`

`objects`: the names `inspect`/`update`/`stats` accept.

### `pub fn inspect(message: &Message, context: &mut Context) -> Reply`

`inspect <object>`: its property snapshot.

### `pub fn update(message: &Message, context: &mut Context) -> Reply`

`update <object> <json>`: apply the writable properties; replies {changed: [names actually changed]}. A change: only
in SETTING, and at DONE too for a barometer's re-zero (super::change_refused), checked first as the table's gate
is.

### `pub fn set_clock(properties: &serde_json::Map<String, Value>) -> bool`

Set the wall clock when the properties carry an acceptable `epoch` (plan B Mission.set_time), and tie this boot
to it with a session.csv row (plan B Mission._index_time); whether the clock was set.

`utc_offset`, `cc_position` and `source` ride in the same properties as momentary inputs: written to the row,
never stored, an invalid one an empty cell (mission::TimeSet). A row the full ring refuses is logged and lost,
never the time set: failing the update would have CC report a time set that happened as refused.

### `pub fn stats(message: &Message, context: &mut Context) -> Reply`

`stats <object>`: its runtime numbers.

## `firmware::commands`

The CC commands (doc/specs/cc-protocol.md; src/glider cc_client.py): one request line in, at most one reply line
out, through one table.

Every line -- from the hub (link.rs) and from the console -- runs on one executor thread, one at a time, so a
handler owns the state it touches (the mission) without a lock, and two operators can never interleave half a
command. The envelope is plan B's: an unparsable line answers `err badargs`, an unknown command `err badcmd`,
a handler failure `err internal`, an empty line nothing.

Changes run only in SETTING (the operator's rule, 2026-10-06): every command that changes the board --
`update`, `set-config`, `reset-config`, `probe`, `detect`, `bustune`, `reboot`, `calibrate <device>` -- answers
`err unsafe` in any other stage, DONE included (no reboot there: the beacon and the flash-to-SD copy run on). The
one exception is a barometer's re-zero, its `update` and `calibrate`, which also run at DONE: the way back from a
false launch without a power cycle (`stage done`, the re-zero, `stage setting`). Reads, `stage` and `disarm` stay
open: they are how the operator recovers. Plan B left `update` open in flight and allowed the rest at DONE
(doc/specs/avionics-port.md §6). The thread that serves a change checks the stage again (the bus owners, the
control thread's mailbox), since a probe sweep takes seconds and a launch may be detected meanwhile.

### `pub struct Context`

What the handlers share: the board, the settings store, and the state the executor owns.

### `pub enum Code`

The error codes on the wire (cc-protocol.md "Errors"), as plan B answers them.

### `pub struct Fault`

A command's failure: `err <code> <detail>`.

- `fn new(code: Code, detail: impl Into<String>) -> Self` — A fault with this code and text.

### `pub type Reply = Result<String, Fault>`

A handler's answer: the whole reply line, or the fault to report.

### `pub struct Executor`

The handle callers post lines through; clone one per caller.

- `fn start(context: Context) -> anyhow::Result<Self>` — Spawn the executor thread (core 0, priority 5, beside the link and the console) owning `context`.
- `fn ask(&self, line: &str, origin: Origin) -> Option<String>` — Run one line from `origin` and wait for its reply; None for a line with no reply. The line is stamped as it arrives, before it queues behind the lines ahead of it.

### `pub fn handle(line: &str, context: &mut Context, origin: Origin) -> Option<String>`

Answer one request line -- the one point of injection for the hub and the console alike; None for an empty
line. A CC command answers from [`TABLE`]; any other name goes to the Glider Command Language registry
(firmware::gcl: `help`, and every command a component registered), under the same SETTING gate; a name neither
knows answers `err badcmd`.

### `pub fn names() -> Vec<&'static str>`

The CC commands' names, for the GCL `help` (firmware::gcl::set_cc_names).

### `pub fn outside_setting() -> Option<String>`

Why a command that changes the board must not run now, or None in SETTING (plan B's `_in_flight`, narrowed).

### `pub fn diagnostic_refused() -> Option<String>`

Why a bus diagnostic (`i2c touch`, `i2c read`, `i2c general-call`, `imu mute`; `bustune` but its way back) must not
run now, or
None in SETTING while disarmed: it holds a bus owner off its parts, so never in the last seconds on the pad (M9; plan
B gates only on the stage).

### `pub fn change_refused(context: &Context, target: &str) -> Option<String>`

Why a change to the device `target` must not run now, or None when it may: in SETTING, and at DONE when the
target is a barometer that re-zeroes (parts::rezeroes), its `update` and `calibrate`.

### `pub fn parse_json(text: &str, code: Code, detail: Option<&str>) -> Result<serde_json::Value, Fault>`

Parse a CC JSON payload, refusing one nested deeper than [`JSON_DEPTH`] before serde_json recurses into it.
A failure answers `code` with `detail` (None: the parser's own message).

### `pub fn ok_json(value: &impl serde::Serialize) -> Reply`

`ok <json>`: the usual successful reply.

### `pub fn waited() -> Duration`

How long the line the executor runs now has waited since it arrived: in the queue and in its own handler so far
(`arm` refuses one that waited too long, controller::stale_arm). Zero off the executor.

## `firmware::commands::streaming`

log / tlm -- poll-model streaming (src/glider cc_client `_register_streaming`): each call returns what was
buffered since the last one and keeps the window open `ms` more (default 1000; 0 or less stops).

Both drain the recorder's CC tees (doc/specs/avionics-port.md "CC tees and the HTTP exits"): while a window is
open, the ring_writer copies each record it programs into the matching tee, verbatim, and the drain here, on the
executor thread, prints them. So the stream shows exactly what reached the flash ring, and the text is made off the
flight path. Replies keep plan B's shape, without the CRC wrapper: `{"lines": [...], "dropped": n}` for `log`,
`{"samples": ["@<routing>@<row>", ...], "dropped": n}` for `tlm`. A board whose recorder did not open answers an
empty batch.

The writer tees a record only once it is programmed, and a log line only after the 20 ms merge horizon, so a
drain first waits for the writer to catch up with the request (crate::recorder::catch_up, ~20-30 ms): it then
carries every line and row queued before it, as plan B's immediate tee did.

### `pub fn log(message: &Message, _context: &mut Context) -> Reply`

`log [ms]`: {lines, dropped}.

### `pub fn telemetry(message: &Message, _context: &mut Context) -> Reply`

`tlm [ms]`: {samples, dropped}, each sample `@<routing>@<row>`.

## `firmware::commands::system`

reboot -- restart the board (src/glider cc_client `_register_system`).

### `pub fn reboot(_message: &Message, _context: &mut Context) -> Reply`

`reboot`: replies `ok`, then restarts (components/safe_restart). When internal RAM cannot give the delayed restart
a stack, it restarts at once: the reply is lost, the reboot asked for is not.

## `firmware::config`

The board's settings: the built-in default config, and the `cfg` NVS partition (partitions.csv) that holds what
src/glider kept as files on its VFS -- board.config and launch.config in namespace `config`, the Wi-Fi passwords
(plan B's `<ssid>.creds`) in namespace `creds`. tools/provision.sh writes the whole partition before an airframe
is sealed; the CC commands and the console keep it current after that.

Documents are JSON text in blobs: NVS strings stop at one 4 KB page, and a board config is ~10 KB.

### `pub struct Store`

The documents: board.config and launch.config.

- `fn open(partition: EspCustomNvsPartition) -> anyhow::Result<Self>` — Open the documents' namespace.
- `fn board(&self) -> Option<String>` — The saved board.config text; None when there is none (the board then runs the default).
- `fn save_board(&self, text: &str) -> anyhow::Result<()>` — Save board.config, replacing the old one only once the new one is written (plan B: atomic_write_json).
- `fn remove_board(&self) -> anyhow::Result<bool>` — Forget board.config, so the next boot runs the default; whether there was one.
- `fn launch(&self) -> Option<String>` — The saved launch.config text; None when there is none (the mission then starts from its default).
- `fn save_launch(&self, text: &str) -> anyhow::Result<()>` — Save launch.config, replacing the old one only once the new one is written.

### `pub struct Credentials`

The Wi-Fi passwords, one per network (plan B: `<ssid>.creds` beside board.config). Never in the image, never in
git: tools/provision.sh or the console `wifi` command writes them. Cheap to clone: each call opens its own NVS
handle, so the link and the console share it without a lock (NVS serialises internally).

- `fn new(partition: EspCustomNvsPartition) -> Self` — The passwords on this partition.
- `fn password(&self, ssid: &str) -> Option<String>` — The stored password for `ssid`; None when none (or an empty one) was stored -- plan B falls back to the config's password on an empty `<ssid>.creds` line too.
- `fn set_password(&self, ssid: &str, password: &str) -> anyhow::Result<()>` — Store the password for `ssid`; the link uses it on its next attempt at that network. Refuses what the radio could never use: an SSID outside 1..=32 bytes, a password over 64 (tools/provision.sh's limits).
- `fn register_commands(&self)` — The credential store's console command (the Glider Command Language), registered at boot. Console only: over CC the password would land in the hub's log.

### `pub fn defaults() -> &'static Value`

Plan B's built-in board config (src/glider config_default.default(), generated into
src/avionics/golden/config_default.json), stamped with this build's version: the fixture pins
board.firmware_version to 'dev', and plan B's load() restamps a saved config from the running firmware.

### `pub fn partition() -> anyhow::Result<EspCustomNvsPartition>`

Take the `cfg` partition once (esp-idf-svc refuses a second take, and dropping the last handle deinitialises
it). A partition another firmware left foreign is erased and formatted, so this fails only on a partition table
without `cfg`.

### `pub fn replace_blob(nvs: &EspNvs<NvsCustom>, key: &str, bytes: &[u8]) -> anyhow::Result<()>`

Write `bytes` under `key` with IDF's own update, which writes the new entry before it erases the old one, so a
cut keeps one of the two. (esp-idf-svc's set_blob erases the key first: a cut there loses the document, and
the next boot would run the default without saying so.) The recorder's session-start record goes through it
too (crate::recorder).

## `firmware::console`

The device console on UART0 (the board's CH343P USB-serial port): a line reader, nothing more. Every line goes to
the CC executor, the one point of injection (commands/mod.rs), tagged as the console's, so the console runs
exactly the commands the hub runs -- the CC commands and every Glider Command Language command a component has
registered (crate::gcl; `help` lists them) -- under the same gates, plus the few that are console only (`wifi`:
its password must never cross the network).

tools/avionics_console.py is the operator's end of it: history, completion from `help`, readable replies.

The UART driver goes in first thing in main ([`install`]), the reader only once the executor runs
([`Console::serve`]): without the driver IDF's console write polls the transmit FIFO, so the log printer
(crate::logging) spun on core 0 above main's priority and held the bring-up for every line it printed, ~8 ms
each at 115200 baud (the fast warm boot, operator 2026-10-11). With it a write waits on the transmit interrupt.

### `pub struct Console(BlockingStdIo<'static, UartDriver<'static>>)`

UART0 with its driver in place, stdin and stdout through it, until [`Console::serve`] reads it.

- `fn serve(self, executor: Executor) -> anyhow::Result<()>` — Serve the console on its own thread (core 0, priority 5, beside the CC executor it feeds), forever.

### `pub fn register_commands()`

The console's own command (crate::gcl): `console baud rate=<n>`, for big transfers over the console (the operator,
2026-10-11: more reliable than Wi-Fi; tools/ota.py --console). The answer goes at the old rate, the rate changes
after it; the next boot is back at 115200 (the bootloader's and the ROM's rate).

### `pub fn install(uart: impl Uart + 'static, transmit: impl OutputPin + 'static, receive: impl InputPin + 'static) -> anyhow::Result<Console>`

Take over UART0: its driver in place and the console's stdio through it, interrupt-driven from here.

Errors: the driver would not install (the console then stays IDF's polling one, and has no reader).

## `firmware::control::duties`

The board's side of the control tick's duties (crates/controller `ControlCore`, R3): the hooks, and the state the
executor reads of the filter and the stage machine.

### `pub fn attitude_inspection() -> Option<serde_json::Value>`

What `inspect` shows of the attitude filter (plan B tasks/attitude.py `inspect`): its flags, angles and magnetic
reference as the filter last published them; None when no filter runs.

### `pub fn probe_attitude(only: Option<&str>) -> Option<(&'static str, Option<String>)>`

The attitude filter's probe (plan B tasks/attitude.py `probe`), on the executor: its name and verdict, the gyro
`rate` fresh now (None healthy, else plan B's text), logged in plan B's shape; None when no filter runs or
`only` names another object.

### `pub fn attitude_blind() -> bool`

Whether the attitude filter is blind (`attitude-blind`, plan B cc_client): no fresh gyro for its blind window.

### `pub fn sequencer_inspection() -> Option<(String, Vec<String>, u32)>`

What `inspect` shows of the stage machine beyond the table (M9): its slots' sources, the ignored ones, and the veto
edges it reported. None before the control thread binds it.

## `firmware::control::imu`

The primary IMU on SPI2 (M6): its read on each tick, its INT1 edges, its probe, its clock rung and its mute.

### `pub struct SpiRung`

A `bustune spi` rung's outcome: the IMU's name, its verdict at the asked clock (None healthy, else why), and the
clock its handle runs at afterwards (the configured one when the driver refused the asked one, 0 when neither came
back).

### `pub fn bustune_spi(clock_hz: u32) -> Option<Result<SpiRung, String>>`

A `bustune spi` rung on the control thread (SPI2's owner); None when the thread drives no IMU. A clock above
[`MAXIMUM_SPI_HZ`] is the IMU failing the rung, answered here and never sent: the handle is not touched, and the
CC's ladder (5, 8, 10, 16, 20 MHz) still finds the part's ceiling.

Errors: the control thread is busy, did not answer in time, or refused the rung outside SETTING.

### `pub fn probe_imu() -> Option<Result<(&'static str, Option<String>), String>>`

The IMU's probe on the control thread (CC `probe`): its name and verdict (None healthy, else why); None when the
thread drives no IMU.

Errors: the control thread is busy, did not answer in time, or refused the probe outside SETTING.

### `pub fn imu() -> Option<(&'static str, bool, bool)>`

What `inspect` shows of the IMU the control thread drives (plan B lsm6dso32.py:300-308, beyond the channels and
the table's `healthy`): its name, whether INT1 is wired, whether INT1 is silent. None without one.

## `firmware::control::interrupts`

The GPIO interrupts the control thread serves: the service it installs on core 1, the second IMU's INT1 and the
INA226's ALERT, counted.

### `pub fn arm_second_interrupt(gpio: u8) -> Result<(), sys::EspError>`

Arms the second IMU's INT1 (the BMI323's, M9) on GPIO `gpio`, counting into [`take_second_edges`]. Only once the
control thread runs ([`running`]): it installed the GPIO interrupt service on core 1 before it ran.

Errors: the pin would not configure, or its handler would not attach.

### `pub fn arm_alert_interrupt(gpio: u8) -> Result<(), sys::EspError>`

Arms the INA226's ALERT on GPIO `gpio`: falling edges, the internal pull-up holding the open-drain, active-low line
high (plan B `Pin(gpio, Pin.IN, Pin.PULL_UP)`, IRQ_FALLING), counting into [`alert_edges`]. Only once the control
thread runs ([`running`]): it installed the GPIO interrupt service on core 1 before it ran.

Errors: the pin would not configure, or its handler would not attach.

### `pub fn alert_edges() -> u32`

The INA226's ALERT trips since boot.

### `pub fn take_second_edges() -> u32`

The second IMU's INT1 edges since the last take.

## `firmware::control`

The control thread (doc/specs/avionics-port.md §4.2, §4.3; M6): the flight tick on core 1 at priority 20, paced by
a gptimer alarm at the config's `schedule_hz` (board_config::schedule_hz). The timer is created on this thread, so
its interrupt is allocated on core 1, and its ISR only notifies: this thread first, then each I2C owner
(crate::owner), which run their own tables on the same tick. No float, no lock and no allocation in the ISR.

Each tick takes the notifications that piled up: one is the tick, any more are ticks this thread missed (an
overrun). Its lateness is measured against an ideal grid anchored at the first tick (the gptimer and esp_timer share
the crystal). The figures are atomics the console reads (`control`), never a lock on this thread's path.

The tick's work (§4.3), as the milestones bring it: the primary IMU, the LSM6DSO32 on SPI2 (M6) -- its INT1 edges
counted by a core-1 GPIO interrupt (`irq_runs`, plan B's evidence of a live line), one 16-byte burst from its status
register, and only a new conversion published: `accel` on XLDA, `rate` on GDA, the row on either, written on this
thread's own producer ring; [`STALE_LIMIT`] reads in a row with neither make the part unhealthy, with one line (a
part slower than the tick reads stale now and then: that is its ODR, not a fault); the separation switch (M6) --
a both-edge
GPIO interrupt on core 1 records the contact's episode (its first and latest edge, esp_timer us) into a stamp pair;
each tick reads the pin, offers the read (observed, or polled when the arm failed), takes the pair once no edge came
for `debounce_ms` and concludes it by that read -- separated, nested or a blip, each recorded with its moment and
wobble in the durable separation.csv. In BOOSTING a separation counts only once the thrust gate is open (M7,
drivers::separation): the glider's own +x specific force, read from a fresh `accel` each tick, showed the thrust and
then its end; before that it is pending, and counts when the gate opens or by the backstop (the pin held open
100 ms). A counted one moves the stage to GLIDING.

The stage machine (M7, sequencer::StageMachine), built from the first enabled `sequencer` entry: stepped every
`period_ms` on the uptime clock (every tick when the tick is slower), after the requests, with `accel`, `elevation`
and `agl` read fresh at that moment. Its own advance commits to the flight word, then logs `controller :: stage ->
X` to the console only, then records `<entry> :: stage -> X (reason)` and the `<entry>.csv` row `stage;reason`; a
move made from outside (the separation, an operator) writes its row only, before any advance's, as plan B's `_tick`
and `_advance` do. The reason text is made here, on core 1, only for the machine's own transition (six or fewer a
flight): with the separation's event line, the one exception to "text only off the flight path" (§3.1).

When the thread or its tick does not start, each entry it drives (the IMU, the separation switch, the stage
machine) is crashed with "the control thread did not start": nothing reads or steps them this boot, so `arm`
refuses.

Each tick stores its heartbeat, the uptime in ms as a wrapping u32 (the P4 has no 64-bit atomics), which the
supervisor's stall check reads (crate::supervisor): one Relaxed store, and one Relaxed load of the flag
`panic kind=stall` sets, on which the thread stops for good, so the check is proved on the board.

By subsystem (R3 of doc/specs/avionics-refactor.md): this module is the thread itself -- its timer, its wakers,
its start and its tick, the figures `control` shows; the filter, the stage machine and the flight law are the
control tick's duties (crates/controller `ControlCore`, the SITL's too) behind the board's hooks ([`duties`]);
the SPI IMU ([`imu`]), the separation switch ([`separation`]), the GPIO interrupts ([`interrupts`]), the lapse
watches ([`watch`]) and the requests ([`requests`]) each have their own file.

The flight word's one writer lives here (M7, crate::status): this thread commits every change of the stage, the
hold, the arming and the launch, between two ticks' work, so no change races a detector. The separation's GLIDING
commits through it, and so do the operator's requests: the executor posts each one to its request slot, which the
tick serves first among its requests (§4.3, step 4), and logs it from the answer, so this thread takes no lock and
makes no kernel call for a request. The mailbox after the slot carries the SETTING-only `probe` and `bustune`, each
checked against the stage again where it is served: a launch detected during a probe sweep never runs a probe. The
recorder's flight flag follows each commit: raised before a transition's rows, lowered after them. `control` shows
the tick's work time (from the wake to the end of its work), whether the flight word holds the machine's launch,
and what the thrust gate has seen.

### `pub fn running() -> bool`

Whether the control thread runs: its tick, and the GPIO interrupt service it installed first.

### `pub fn start(schedule_hz: u32, spi_bus: Option<SpiBus>, spi: Vec<SpiMember>, separation: Option<SeparationMember>, sequencer: Option<SequencerMember>, attitude: Option<AttitudeMember>, fins: Option<crate::fins::Fins>, centre: Option<usize>, flight: Option<crate::devices::FlightMember>, topology: &'static Topology, table: &'static Table<'static>, writer: FlightWriter<'static>) -> anyhow::Result<()>`

### `pub fn last_tick_ms() -> u32`

The uptime in ms, as a wrapping u32, of the latest tick ([`ticked`] says whether there was one).

### `pub fn ticked() -> bool`

Whether a tick has stored the heartbeat this boot. Acquire: load it before [`last_tick_ms`], so a check that
straddles the first tick never pairs `true` with the boot's 0 (controller::Stall).

### `pub fn stall()`

Stops the control thread at its next tick (`panic kind=stall`): the supervisor's stall check is what restarts the
board. The caller checks first that it will (crate::supervisor::stall_refused).

### `pub fn register_commands()`

The control thread's console command (the Glider Command Language): `control`, the tick's figures.

### Constants

- `const LATENESS_BUDGET_US: u32 = 200` — The lateness budget a tick must keep, microseconds (the M6 proof: max <= 200 us under FHD@25 and bus load).
- `const GNSS_SOURCE: &str = "gnss"` — The device name the GNSS publishes `elevation` under (plan B tasks/sequencer.py `_GNSS_SOURCE`): the stage machine binds it to its own slot, which never starts a flight (sequencer::GNSS_SOURCE).

## `firmware::control::requests`

The control thread's requests: the executor's flight-word slot and the mailbox of probes, bus tunes and fin updates,
served on the tick.

### `pub fn probe_fins(only: Option<&str>) -> Option<Result<Vec<crate::fins::Swept>, String>>`

The fins' probe sweep (plan B sg90.probe, M13): every fin, or the one named `only`, one at a time through its
stops on the control thread, the INA226's power sampled as each travels. SETTING and disarmed only; a launch, an
arm or a stage change before a fin leaves the fins left unswept.

Returns: each fin's sweep; None when no fin runs, or `only` names none.

Errors: the control thread is not running, busy, refused outside SETTING, already sweeping, or did not answer.

### `pub fn update_fin(name: &str, properties: &Value) -> Result<Result<Vec<&'static str>, String>, String>`

The operator's `update <servo> {trim|angle}` (M13): posted to the control thread, the fins' owner.

Returns: the keys applied, or the servo's refusal (a key that is not a number).

Errors: no fin by that name, the control thread is not running, is busy, refused outside SETTING, or did not
answer in time.

### `pub fn request(request: sequencer::Request) -> Result<sequencer::Answer, String>`

Applies an operator `request` (the executor's `stage`, `arm`, `disarm`) to the flight word through its one writer, this
thread: posted to the executor's slot, served at the next tick, withdrawn when not taken in [`REQUEST_DEADLINE`].
While the thread is not running nothing commits, so a request that would change nothing (a disarm of a disarmed
board) is answered with the flight as it stands, at once, and any other is not applied. The executor's alone.

Returns: the answer: the flight committed, or why an arm was refused.

Errors: why there is no answer, saying whether the request was applied.

## `firmware::control::separation`

The separation switch (M6, M7): its edges, its debounced episodes and the thrust gate that counts one.

### `pub fn publish_separated(separated: bool)`

Publishes the separation latch: bring-up seeds it from the level it read, the tick keeps it.

### `pub fn switch_level() -> Option<(&'static str, bool)>`

The separation switch's name and pin level (true = HIGH = nested), for its probe; None without one.

### `pub fn separated() -> bool`

The separation switch's debounced latch (plan B `separated()`): true = separated.

## `firmware::control::watch`

The lapse watches (M9): a tick at which an IMU channel is not fresh while its part is healthy.

### `pub fn lapses_of(source: &str) -> Vec<(String, u32)>`

`source`'s watched channels' lapses, by quantity (`inspect` of an IMU); empty before the thread starts.

## `firmware::devices`

The device bring-up (doc/specs/avionics-port.md M5, "The registry"; plan B's Controller.setup, controller.py:
353-420): every enabled entry of the post-layout config, sensors then components, in config order, one at a time,
on main while nothing else runs. crates/controller holds the rules (the attempts and their gaps, the diagnose on
the last, the alternatives pass, the operator's lines); this module makes each attempt on the hardware
(crate::sensors), keeps the parts that came up for their bus owners, and builds the databoard topology from them.

- An I2C part runs on the port its entry binds, lent to main by its owner (crate::owner); the parts go back with
  the port when the owner starts. The I2C parts come up first, one thread per lent port, each in config order on
  its own bus: the buses are independent, so one bus's slow setups (a barometer's ground capture) never hold the
  other's (the fast warm boot, operator 2026-10-11).
- SPI: the chip select of every part with no driver yet is held high before any SPI part is set up (a floating
  select could fight MISO, plan B spibus.py:42); the LSM6DSO32 is set up on one handle reused across its attempts
  (the first-use discard follows the peripheral, not the handle). Its owner is M6's control thread.
- A general call during a setup (the ICP-10111 frees a latched bus with one) re-arms every part of that port
  already up, the sender excepted, and is written as an oops record (§4.8).
- The stage machine (M7): the first enabled `sequencer` entry's settings, read by sequencer::Settings::from_config;
  a key it refuses fails the entry, naming the key, so `arm` refuses. Its steps are the control thread's.
- The checkpoint and the watchdog (M8): the first enabled `checkpoint` entry's cadence and the first enabled
  `watchdog` entry's settings, read by board_config's views, which refuse a key plan B could not run with and so
  fail the entry. Both run on the supervisor (crate::warm, crate::supervisor); a refused watchdog leaves the task
  watchdog at IDF's warn-only default.
- The topology: one Parameter per quantity with a provider that came up, its sources registered in setup order
  (plan B's `provide` at the end of each successful setup), then frozen.

### `pub struct SpiMember`

The LSM6DSO32 on SPI2 once it came up: the control thread, SPI2's owner, takes it (crate::control).

### `pub struct SeparationMember`

The separation switch once it came up: the control thread reads it every tick (crate::control).

### `pub struct SequencerMember`

The stage machine once its entry came up: the control thread steps it (crate::control).

### `pub struct AttitudeMember`

The attitude filter once its entry came up: the control thread steps it (crate::control, M9).

### `pub struct CheckpointMember`

The checkpoint once its entry came up: the supervisor runs it (crate::warm::Keeping).

### `pub struct WatchdogMember`

The watchdog once its entry came up: the supervisor runs it (crate::supervisor).

### `pub struct GnssMember`

The GNSS once its entry came up: its thread takes it with UART2 (crate::gnss, M11).

### `pub struct GnssCalibMember`

The pad drift calibration once its entry came up: the supervisor passes it (crate::gnss::Calibration, M11).

### `pub struct FieldMember`

The CC-less field agent once its entry came up: the executor ticks it (crate::commands field, M11).

### `pub struct HitlMember`

The HITL sim-link once its entry came up: its thread binds the port (crate::sim_link, M12).

### `pub struct FlightMember`

The flight loop once its entry came up: the control thread runs it (crate::flight, M13).

### `pub struct ServoMember`

A fin servo once its entry came up: its pin and logic, for crate::fins.

### `pub struct Devices`

What the bring-up leaves.

### `pub fn crash<'name>(table: &Table, names: impl IntoIterator<Item = Option<&'name str>>, reason: &str)`

Crashes each entry of `table` named in `names` that came up, with `reason`: the thread that would run it did not
start (the control thread, the supervisor), so nothing runs it this boot, verify lists it and `arm` refuses.

### `pub fn bring_up(document: &'static Value, ports: &mut [(owner::Parked, I2cPort)], recording: bool) -> Devices`

Brings every enabled entry of `document` (the post-layout config) up, in config order, on the ports main holds on
loan (`ports`), and logs plan B's lines and the topology. `recording` is whether the recorder opened: its entry
is up only then; the camera's only when its sensor answers ([`camera_answers`]).

## `firmware::diagnostics`

Operator diagnostics, registered as GCL commands (crate::gcl) at boot:
- `load seconds=5` -- per-core CPU load and the busiest tasks over a window;
- `pace seconds=10 source=gptimer` -- a 1 kHz loop on core 1 (priority 20, the flight thread's place) woken by a
  gptimer interrupt allocated on core 1 (the flight design) or by an esp_timer (its task lives on core 0), and how
  late each tick wakes;
- `heap [stacks|tasks]` -- where the RAM is: internal, DMA-capable, RTC, SPM and PSRAM heaps; each task's stack
  (where it lives, how much of it was never touched); and (in an investigation build) each task's heap share.

Run them while the camera records to price video against the flight loop. `load` and `pace` are SETTING-only:
`pace` runs a priority-20 loop on the flight core, and neither belongs anywhere near a flight.

(The `flashwrite` load generator is gone: it wrote raw patterns into the `log` partition, which is the live
telemetry ring now. src/bin/flash.rs keeps that measurement.)

### `pub enum Source`

What wakes the paced loop.

### `pub fn heap() -> String`

`heap`: where the RAM is, as one CC line. Internal RAM is the scarce kind on the P4: the camera's encoder holds one
~138 KB block of it, esp_hosted copies every Wi-Fi frame into internal DMA memory and asserts when it gets none,
and the SD driver bounces every transfer whose buffer is not DMA-ready through internal DMA memory. So: free bytes,
the largest free block (what one allocation can get) and the lowest free since boot, for internal RAM, its
DMA-capable part, and PSRAM.

The RTC (LP SRAM) and SPM heaps are internal but not DMA-capable, and IDF hands them to plain requests last, so
they show what a cold, non-DMA object could still use.

### `pub fn heap_stacks() -> String`

`heap stacks`: each task's stack as `<task>=<never-used bytes>/<region>:<core>` (region as [`stack_region`]; core
`any` when unpinned), internal stacks first, then by name: the protocol's plain characters, so a value reads as is.
The never-used figure is FreeRTOS's high-water mark: the least free the stack has had since the task started, in
bytes.

### `pub fn heap_tasks() -> String`

`heap tasks`: each task's current heap use, internal and PSRAM, as `<task>=<internal>/<psram>` bytes, the largest
internal users first. Only in an investigation build with CONFIG_HEAP_TASK_TRACKING: that option crashes on any
allocation made while the scheduler is suspended, so it never ships. Task handles are matched against the live
task list (a deleted task's handle is never dereferenced); allocations of tasks that are gone show as `gone`.

### `pub fn heap_tasks() -> String`

`heap tasks` in a build without CONFIG_HEAP_TASK_TRACKING: why there is no per-task figure.

### `pub fn load(seconds: u64) -> String`

Per-core load over `seconds` and the busiest tasks, as one CC line.

### `pub fn pace(seconds: u64, source: Source) -> String`

A 1 kHz loop on core 1 for `seconds`, woken by `source`: how late each tick woke, as one CC line.

### `pub fn register_commands()`

The diagnostics' console commands (the Glider Command Language), registered at boot.

## `firmware::entry`

The entry file (M21, doc/specs/avionics-port.md): `/sdcard/entry.gcl`, one GCL line per line, `#` comments. Once
bring-up is done, each line runs in order through the executor (origin [`Origin::Entry`], the one point of
injection), under the normal gates, so changes still run only in SETTING; a console-only command is refused, since
the file sits on a card the hub can pull. Each line and its answer go to recorder.log. It replaces MicroPython's
run-any-script flexibility for bench tests, soaks and HITL setup (`ring load seconds=600`, `video start`,
`pace seconds=60`).

**A bench mode never rides into a flight:** while the file is on the card, `verify` and `arm` refuse with
`entry-file` ([`problem`]); `entry clear confirm` (or `rm confirm path=entry.gcl`) takes it away.

The file is written over CC, no card reader needed ([`register_commands`], SETTING only): `entry add text=<line>`
appends a line (the hub carries its spaces base64), `entry show` lists them, `entry clear confirm` removes the
file. They run from the next boot.

### `pub fn problem() -> Option<String>`

Why `verify` and `arm` refuse while the file is on the card; None when it is not.

### `pub fn start(executor: Executor) -> anyhow::Result<()>`

Runs the entry file's lines on a thread of their own, in order, through `executor`, when the card holds one; at
once otherwise. Call it once bring-up is done and the card is mounted.

Errors: the runner would not spawn.

### `pub fn register_commands()`

The entry file's commands (crate::gcl), registered when the card mounts: they write the file the next boot runs.

### Constants

- `const NAME: &str = "entry.gcl"` — The file's name at the card's root.

## `firmware::files`

HTTP access to the microSD and to the flash ring (read-only; the one write is an OTA image): how recordings and logs come off the board over Wi-Fi
-- on the pad, and after landing. Nothing is ever written.
* `GET /sd/<path>` returns a file, or a directory's listing (one `<name>;<bytes>` line per entry). Paths are
  confined to the card, laid out as crates/recorder's layout says: a session's recordings at
  `/sd/<session>/recordings/`, its videos at `/sd/videos/<session>/`, the global index `/sd/sessions.csv`. On a
  two-partition card `videos` is the videos partition's mount point, which partition 1's root does not hold, so
  the root's listing names it itself. A file another task holds open for writing (the ring_saver appending a
  batch, the video writer) answers `503` with `Retry-After: 1`: FatFs's lock table (CONFIG_FATFS_FS_LOCK) refuses
  the open (FR_LOCKED, EACCES through the VFS), as it refuses a full table (ENFILE). A saver batch holds a file for
  a moment, so the client asks again; the video file being recorded stays busy until `video stop`.

  A file's reply carries its size as of the open in `X-Coludo-Bytes`, the one proof a client has that the file
  came whole: esp-idf-svc 0.53 drops the Content-Length and sends the body chunked, and when a read fails
  mid-file it still closes the reply as if it were whole. The listing cannot stand in for it: a session the
  board is still saving grows between the listing and the read. The size at the open is the size sent, since the
  lock table keeps every writer out until the reader closes the file. tools/board_pull.sh checks it.
* `GET /ring[?boot=N]` streams every valid block of the `log` partition decoded to wire lines,
  `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`, in block_sequence order: a byte-exact wire capture, the lines the
  ring_saver would file (doc/specs/avionics-port.md "CC tees and the HTTP exits"). `boot=N` keeps only that
  boot's blocks.
* `GET /ring/image` streams the raw 20 MiB partition, as `esptool read_flash 0xc00000 0x1400000` reads it.
* `POST /ota?sha256=<hex>` writes a firmware image into the next app slot (M15, crate::ota): the one write this
  server takes, on the pad and disarmed only.
* `GET /camera/preview.jpg` is the camera's latest preview (M7b): 720x384, made once a second while the camera
  records on the pad (404 before the first), for the operator to check its aim.
* `GET /beacon` is the recovery beacon's page as the access point serves it at DONE (crate::beacon): the bench sees
  it before then.

tools/ring_split.py turns either into session directories: the recovery path when the card failed in flight. The
ring keeps changing while it streams (the writer appends, the eraser erases), so either is a snapshot taken over
some seconds; a block erased meanwhile is left out, and the tools drop the duplicates a pull may share with the SD.
A block that stays unreadable is left out too (`/ring/image` sends it as zeros, so the image keeps its size), and
recorder.log says so.

### `pub fn serve(beacon: crate::beacon::Beacon) -> anyhow::Result<EspHttpServer<'static>>`

Start the server; it serves for as long as the returned handle lives.

## `firmware::fins`

The fin servos (M13; plan B drivers/sg90.py, mg90s.py, the logic in crates/drivers sg90): one LEDC channel per
servo on one 50 Hz timer at 14-bit duty (1.22 us a step). Every servo pin of the config is driven LOW from the
first moment of the boot ([`hold_low`]): a floating fin pin picks up the others' pulses and every servo twitches
(the bench gotcha). At bring-up's end [`open`] gives each pin its channel with no pulse. On a cold boot the control
thread then centres the fins ([`Fins::centre`], stepped each tick), a group at a time, `fins.concurrency` per
group, each group given its travel time before the next: three servos starting together draw their peaks
together, which a 1 A bench supply does not carry. The centring never holds the control thread's start (it took
~0.9 s at concurrency 1: the fast warm boot, operator 2026-10-11); a warm start skips it, the restored law taking
the fins from its first tick, and a launch during it hands the fins to the law.

From there the control thread owns the fins: the flight loop's compare-and-set [`Fins::set_angle`] (M13 stage 4)
and the operator's `update <servo> {trim|angle}` through its mailbox ([`Fins::update`]). A write is two LEDC
register updates; the row in `<name>.csv` (`angle`, `pulse_us`, `done`) goes through the recorder's lock-free
producer ring. The executor reads each fin's state through atomics (`inspect`).

### `pub struct Fins`

The fins, owned by one thread: bring-up's main, then the control thread.

- `fn centre(&mut self, concurrency: usize)` — Begins the boot centring, `concurrency` fins a group (at least 1), stepped by [`Fins::step_centring`].
- `fn centring(&self) -> bool` — Whether the boot centring is under way: the operator's fin requests wait for its end.
- `fn step_centring(&mut self, now_us: u64, may_move: bool)` — One control tick of the boot centring at `now_us`: each fin to its configured angle (`angle`, else the range's neutral), trim included, a group at a time. The horn's start is unknown, so a group is given a half-range travel with the settle margin before the next is issued; each move is recorded issued, then completed. `may_move` false (the stage left SETTING: a launch) ends it where it is, the law's from then.
- `fn set_angle(&mut self, index: usize, degrees: f64)` — The flight loop's command for fin `index` (plan B `set_angle`): compare-and-set, nothing written when the clamped angle is the one already commanded.
- `fn rest(&mut self)` — At DONE (M11b): every fin's pulse off, so the servos draw no holding current while the beacon runs on the battery; the next command drives a fin again.
- `fn update(&mut self, name: &str, properties: &Value) -> Option<Result<Vec<&'static str>, String>>` — The operator's `update <servo> {trim|angle}` (plan B `update`): immediate and ungated; a new trim alone re-drives the current angle so the fin moves to its new centre.
- `fn index(&self, name: &str) -> Option<usize>` — The index of the fin named `name`.
- `fn sweep(&self, only: Option<&str>) -> Option<Sweep>` — A probe sweep of every fin, or of the one named `only`; None when `only` names no fin.
- `fn step_sweep(&mut self, sweep: &mut Sweep, now_us: u64, power: Option<f32>, may_move: bool) -> Option<Vec<Swept>>` — One control tick of `sweep` at `now_us`: `power` the INA226's fresh reading (mW), `may_move` whether a fin may move now (SETTING, disarmed, no launch), checked before each fin -- when it may not, the fins left are not swept. Returns the fins' sweeps once it is over.

### `pub struct Swept`

One fin's probe sweep, for the probe's lines (crate::commands, plan B sg90.probe) and its verdict.

### `pub struct Sweep`

The probe sweep in progress (plan B sg90.probe, as a control-thread state machine): one fin at a time, each to
its stops with the power sampled while it travels; stepped every control tick, never waiting.

### `pub fn hold_low(document: &Value)`

Drives every servo pin of `document` (sensors and components with a servo driver, enabled or not) LOW as an
output, before anything else touches the pins: a floating fin pin makes every servo twitch.

### `pub fn open(members: Vec<ServoMember>, probe_limit: Option<i32>) -> anyhow::Result<Fins>`

Gives each servo its LEDC channel (no pulse yet); the fins, for the control thread, which centres them on a cold
boot ([`Fins::centre`]). `probe_limit` bounds the probe sweep ([`Sweep`]).

Errors: the LEDC timer or a channel would not configure (the fins are then crashed by the caller).

### `pub fn count(only: Option<&str>) -> Option<usize>`

How many fins a sweep of `only` (every fin when None) covers; None when no fin runs, or `only` names none.

### `pub fn commanded(name: &str) -> Option<i32>`

Fin `name`'s commanded angle, whole degrees, once a pulse was driven: None before its first write (a warm start
skips the centring, so a fin waits for the restored law) and for a name that is no fin.

### `pub fn inspection(name: &str) -> Option<[i32; 3]>`

Fin `name`'s state for `inspect` (plan B sg90 inspect): the commanded angle, the pulse driven (us), the trim; None
for a name that is no fin.

## `firmware::flight`

The board's side of the flight loop (M13; plan B tasks/flight.py). The loop itself is the control tick's
(crates/controller `ControlCore`): every tick the law reads the databoard's fresh quantities and the mission, and
every 100 ms a flight.csv row samples what it believed and demanded. Here: its surfaces bound to the fin servos
([`bind`]), whose compare-and-set (crate::fins) takes the angles when it actuates, so a held fin costs nothing; the
vitals the `health` panel shows, published through atomics ([`publish_vitals`]); and the warm start's seed.

The mission is the executor's: it publishes the zone, the launch point and the endgame pattern after every line
([`publish_mission`]) into a sequence-locked view the control thread reads without a lock.

### `pub fn publish_mission(mission: &Mission)`

Publishes `mission`'s zone, launch point and endgame pattern for the control thread (the executor, after every
line; main once before the threads start).

### `pub fn mission_view() -> (Option<Landing>, Option<Position>)`

The mission as the control thread reads it: the landing zone and the set launch point, read again when the
executor wrote meanwhile.

### `pub fn launch_e7() -> Option<(i32, i32)>`

The mission's launch point as the flight loop reads it (set by CC, or pinned at arm), (latitude, longitude) in
1e-7 degrees: the beacons' map link while no fix came this boot (crate::beacon).

### `pub fn keep_warm_airspeed(airspeed: f32)`

Keeps a warm start's airspeed for the law's seed (crate::warm, before the control thread starts), logged here: a
float formatted on the control thread costs its stack.

### `pub fn bind(name: &str, law: &FlightLaw, fins: Option<&Fins>) -> Vec<Option<usize>>`

The law's mixer surfaces bound to the fin servos by name, for the flight loop of entry `name`: each surface's fin
index, None for a surface with no servo (logged: a third of the authority lost). The surfaces are kept for
`health` and the readiness.

### `pub fn warm_airspeed() -> Option<f32>`

A warm start's airspeed, the law's seed; None when none was kept.

### `pub fn publish_vitals(law: &FlightLaw)`

Publishes the law's vitals for `health` (plan B flight.vitals): after each flight.csv sample, on the control thread.

### `pub fn vitals(attitude: Option<[i32; 3]>) -> Option<Value>`

The flight panel's vitals (plan B flight.vitals) as `health.flight` shows them; None when no flight loop runs.
`attitude` is the fused one, whole degrees.

### `pub fn airspeed() -> Option<f32>`

The airspeed estimate for the checkpoint's crumb, m/s; None while no flight loop runs.

### `pub fn unbound_surfaces() -> Option<Vec<&'static str>>`

The mixer surfaces with no servo (plan B `_readiness` `fins`), None while no flight loop runs.

## `firmware::gcl`

The Glider Command Language: the operator's console commands, and the registry the board's components add
theirs to when they activate. After the moisture project's bench consoles, written exactly as the CC protocol
writes a line (doc/specs/cc-protocol.md): positional words, then `name=value` named parameters.

```text
command [sub] [word ...] name=value name=value
video start bars fps=25 kbps=8000
pace seconds=10 source=gptimer
panic confirm
```

**Components own their commands.** A component registers its commands when it activates (the SD card when it
mounts: `sdtest`; the camera when the config declares one and the card mounts: `video ...`; the diagnostics at
boot: `load`, `pace`; a driver when its part answers) and withdraws them when it goes. Linking or removing a
component therefore needs no edit anywhere else and no feature switch: `help` lists exactly what this board runs
right now.

**One declaration per command:** its words, what it does, its named parameters (kind, range, default, help) and
its bare words. The parser, the validation and `help` all read it, so a command cannot accept what its help
does not list, and a typo is answered with the names it does take instead of being ignored.

**Values** follow the CC encoding: a value with a space or `=` travels as `name=base64:<data>`, which
tools/avionics_console.py does for you, and the CC parser decodes it before a command sees it. Replies are CC lines (`ok ...` /
`err <code> ...`), so the contract runner and the hub parse them unchanged; the console tool renders them.

**A command that takes the board away needs the word `confirm`** (moisture's `clear confirm`): a stray paste must
not panic the flight computer.

**One point of injection.** The CC executor (commands/mod.rs) is the only dispatcher: the console and the hub
both hand it their lines, tagged with their [`Origin`]. It answers a CC command from its own table and hands
anything else here, so a GCL command is reachable from the hub as from the console -- under the same gates. Each
command declares its [`Gate`] (a change runs only in SETTING, as the CC commands that change the board do) and
its [`Reach`] (a command whose arguments must never cross the network -- `wifi` carries a password, and the hub
logs every line -- is console only).

The registry is behind a lock: the console reads it a line at a time, components write it at activation --
neither is a hot path, and nothing in the flight loop ever touches it.

### `pub enum Kind`

What a named parameter accepts.

### `pub struct Parameter`

One named parameter of a command: `name=value`.

### `pub struct Word`

One bare word a command takes (`bars`, `confirm`).

### `pub enum Gate`

Where a command may run (the CC commands' rule, doc/specs/avionics-port.md §6).

### `pub enum Reach`

Who may send a command.

### `pub enum Origin`

Where a line came from.

### `pub type Run = Box<dyn Fn(&Arguments) -> String + Send + Sync>`

What a command does with its validated arguments; it answers one CC line.

### `pub struct Command`

One command, as a component registers it.

### `pub struct Arguments`

A command's arguments, validated against its declaration and completed with the defaults.

- `fn integer(&self, name: &str) -> i64` — An Integer parameter's value. Only declared names reach here and those were validated, so a miss is a declaration bug: it reads as 0 rather than taking the console down.
- `fn text(&self, name: &str) -> &str` — A Text or Choice parameter's value ('' for an undeclared name).
- `fn word(&self, name: &str) -> bool` — Whether the bare word was given.

### `pub fn register(commands: Vec<Command>)`

Add a component's commands; a command whose words are already taken replaces the old one (a component that
re-activates re-registers).

### `pub fn withdraw(owner: &str)`

Remove every command a component registered (it deactivated, or its part went away).

### `pub fn set_cc_names(names: Vec<&'static str>)`

Tell `help` which CC commands exist (the CC table is the binary's; plain names keep the dependency one-way).

### `pub fn handle(line: &str, origin: Origin, refusal: Option<&str>, diagnostic_refusal: Option<&str>) -> Option<String>`

Answer one line if it names a GCL command (or `help`); None when it names none, so the caller answers
`err badcmd`.

* `origin` - who sent it: a [`Reach::Console`] command refuses the hub.
* `refusal` - why a change must not run now (the stage is not SETTING), or None in SETTING: a [`Gate::Setting`]
  command answers `err unsafe <refusal>`, the CC commands' own answer.
* `diagnostic_refusal` - why a bus diagnostic must not run now (not SETTING, or armed), or None: a
  [`Gate::Disarmed`] command answers `err unsafe <diagnostic_refusal>`.

### Constants

- `const CONFIRM: Word = Word { name: "confirm", about: "required: this command takes the board away", }` — The word that arms a command which takes the board away.

## `firmware::gnss`

The GNSS receiver on its UART (M11; plan B gnss.py, its logic in crates/gnss). One thread (core 0, low priority,
PSRAM stack) owns the UART: it writes the module's configuration (the driver's command bytes with plan B's gaps),
then reads lines forever, hands each to the engine ([`gnss::Gnss::line`]) and publishes what comes back:
`position` (1e-7 degree), `altitude`, `elevation`, `speed` and `course` on the databoard (the GNSS's `elevation`
is the stage machine's own GNSS slot, by its name), and the four streams `<name>.csv`, `<name>_gga.csv`,
`<name>_sky.csv`, `<name>_events.csv`. Once a second the engine's tick runs: the outage sweep, the sky window for
the stage now (the diagnostics on the pad, off from BOOSTING to DONE: the switch frame written whole), and the sky
row when a burst has ended.

The executor sees it through atomics (`inspect`, `probe`, `health`), so the thread holds no lock and makes its
text only for its own event rows and log lines.

The NEO-M8N (`neom8n`, M16) delivers UBX instead of NMEA (gnss::drivers::neom8n): the thread finds its frames in
the byte stream (gnss::ubx `Frames`) and hands each to the engine ([`gnss::Gnss::message`]), which fills the same
update a line does. Its bring-up ([`bring_up_m8`]) finds the module's rate by listening (it keeps its port
settings over a P4 reboot), names it (UBX-MON-VER), raises the port to the driver's baud, and falls back to UBX at
the power-on rate, then to NMEA, when the module does not follow; each frame's ACK or NAK is logged, a refusal and a
fallback are events.

The pad drift calibration (plan B tasks/gnss_calib.py, [`Calibration`]) is the supervisor's duty: it reads the
fused speed and course, whoever provides them, and publishes the frozen drift through atomics for `inspect` and
the flight loop (M13).

### `pub enum Module`

The module on the UART, as the entry's driver names it.

- `fn of(implementation: &str) -> Option<Module>` — The module the entry's implementation names, None for one that is no GNSS module.

### `pub struct Inspection`

What `inspect` shows of the GNSS beyond its channels (plan B gnss.py inspect).

### `pub struct Calibration`

The pad drift calibration once its entry came up: the supervisor passes it every `period_ms`.

- `fn new(member: GnssCalibMember, topology: &'static Topology) -> Calibration` — The calibration of `member`, its quantities found in `topology`.
- `fn name(&self) -> &'static str` — The entry's name.
- `fn period_ms(&self) -> u32` — Its pass period, ms.
- `fn pass(&mut self, now_us: u64)` — One pass at `now_us` (plan B's run loop body): the flight word's stage and arming, the fused speed and course when both are fresh (read, never extrapolated: a stale fix must not enter the mean), and the freeze's log line.

### `pub fn start(member: GnssMember, uart: impl Uart + 'static, topology: &'static Topology) -> anyhow::Result<()>`

Starts the GNSS thread on `uart` (UART2) for `member`, its quantities found in `topology`.

Errors: the UART would not open, or the thread would not spawn.

### `pub fn position(topology: &Topology) -> Option<Position>`

The fused `position` now (plan B `data['position'].value()`): the live fix `health` shows and the mission's launch
point falls back to; None while no source of it is fresh.

### `pub fn inspection() -> Option<Inspection>`

The GNSS's state for `inspect`; None before the thread runs.

### `pub fn probe(only: Option<&str>) -> Option<(&'static str, Option<String>)>`

The GNSS's probe (plan B gnss.py probe), on the executor: the lines read advance within 1.5 s, logged in plan B's
shape. Its name and verdict (None healthy, else why); None when no GNSS runs or `only` names another object.

### `pub fn drift() -> [f32; 2]`

The frozen pad drift, east and north, m/s: zero until the freeze, and without a calibration (plan B
`GnssCalib.drift()`, the flight loop's wind de-bias, M13).

### `pub fn calibration_inspection() -> Option<([f32; 2], u32, bool)>`

The calibration's state for `inspect` (plan B gnss_calib.py inspect): the drift, its samples, whether it froze.
None without a calibration.

## `firmware::link`

The CC link: join Wi-Fi through the on-board ESP32-C6 (esp_hosted over SDIO), dial the hub, serve its lines,
and re-dial forever (src/glider drivers/wifi.py + tasks/cc_link.py + cc_client.Client) -- until the stage machine
detects the launch.

**After the launch** (`launched` in the flight word, crate::status) the link never rejoins, hunts or dials: the
glider leaves range, and a session still up runs on until it drops by itself. The radio is never switched off (a
Wi-Fi stop and start through the C6 is an unproven path that can park the link until a reboot). An operator hold
clears `launched`, so a held bench board keeps its normal reconnects. Plan B keyed its hunting pause on the
airborne stage instead (drivers/wifi.py).

The networks are the board config's (`wifi.networks`, board_config::wifi), tried round-robin with each one's
own retry clock; a password comes from the `creds` store, else the config's fallback. The hub is the config's
`cc_host`, the `.1` of the board's own subnet when it is unset, and nothing when it is '' (standalone: Wi-Fi
still joins, for the file server).

### `pub fn stack_up() -> bool`

Whether the TCP/IP stack is up, so a socket can be opened (it may not hold an address yet).

### `pub fn run(modem: Modem<'static>, event_loop: EspSystemEventLoop, partition: EspDefaultNvsPartition, wifi: Wifi, credentials: Credentials, executor: &Executor, beacon: crate::beacon::Beacon) -> !`

Join, dial, serve -- forever: the driver started ([`start`]), the file server once the TCP/IP stack is up, then
a look every [`TICK`] ([`Link::look`]).

### `pub fn register_commands()`

The C6 link's console command (the Glider Command Language), registered at boot.

## `firmware::logging`

Every log line into recorder.log (doc/specs/avionics-port.md "Producer side": `Recorder::log`), next to the
console as before: the `log` crate's records from Rust, and IDF's own ESP_LOGx lines from C through a vprintf
hook (components/log_hook). From recorder.log a line reaches the flash ring and, while an operator window is
open, the CC `log` tee (commands/streaming.rs).

A recorder.log line reads `<uptime_us> <descriptor> :: <message>`. Our own messages carry their `descriptor :: `
already (`ring :: scanned ...`); any other gets the last segment of its module path, and an IDF line its tag.

The logging path never blocks and allocates at most one bounded string (error policy: logs are best effort). A
message with arguments is formatted into a string cut at [`LINE_BYTES`] (the recorder cuts a message at 1 KiB
anyway); a plain one is taken as it is. The text is on the heap, not the stack, because a `log` call can run on
a task with a tight stack (IDF's system event task has 2.3 KiB). `Recorder::log` then copies it into the thread's
own PSRAM ring, or `try_lock`s the shared `misc` one: a full or busy ring drops the line and counts it. Lines
logged before the recorder opens reach the console only; main opens it right after the board config, so that is
the boot count and the config load.

No line prints in place once the printer runs (first thing in main): a console write blocks once the UART's
transmit buffer fills (115200 baud drains ~11 KB a second, and a CC reply can be kilobytes). A blocked control
tick ran 4.5 ms late (M6, measured); the supervisor feeds the task watchdog, and a console write waits out a whole
reply (IDF's UART VFS holds its write lock for a write's whole buffer, and a `tlm` drain can be hundreds of KB,
tens of seconds at 115200 baud); and the boot's own lines held the bring-up, ~8 ms each (the fast warm boot,
operator 2026-10-11: logging never delays anything). A line goes into a bounded FreeRTOS queue with a non-blocking
send instead, dropped and counted when the queue is full, and a printer thread on core 0 writes it out. Its
recorder.log copy is unchanged: the thread's own ring, lock-free.

IDF's own startup lines are at WARN (sdkconfig.defaults: each was a blocking write before main); [`install`]
raises the level to INFO, so every line after it prints as before.

A line logged to the [`CONSOLE_ONLY`] target reaches the console (and the panic capture) and never recorder.log.

### `pub fn console_dropped() -> u32`

Console lines dropped because the printer's queue was full.

### `pub fn wait_for_printer(limit: Duration)`

Waits until the printer wrote out every line queued before this call, for at most `limit`: a line logged right
before a restart then reaches the console. At once without a printer.

### `pub fn install()`

Route the `log` crate through the console and recorder.log, and IDF's C log lines into recorder.log too (instead
of `EspLogger::initialize_default`). Call first thing in main: lines before the recorder opens reach the console.

### Constants

- `const CONSOLE_ONLY: &str = "console"` — The log target whose lines reach the console only: never recorder.log, so neither the recording nor the CC `log` tee, which reads recorder.log. Plan B printed its Controller's lines to the console only (main.py:75, 101), and the tools find the stage events in a recording by the text `stage ->` (tools/flight_kpi.py, flight_report.py, flight_video.py), which must stay the stage machine's: so the controller's `controller :: stage -> X` logs here (doc/specs/avionics-port.md §4.4).

## `firmware::ota`

Firmware OTA (M15, doc/specs/avionics-port.md): a new image into the app slot that is not running, on the pad. The
data comes by either of two transports, the decision over CC:
- **The console** (the operator, 2026-10-11: more reliable than Wi-Fi for big transfers): `ota begin size=<n>
  sha256=<hex>`, then `ota write offset=<n> crc=<crc32 hex> data=<base64url, no padding>` lines in order, then
  `ota end` (`ota abort` drops it). tools/ota.py raises the console to 2 Mbaud for it first (`console baud`). A line
  the UART damaged (at 2 Mbaud a late interrupt drops bytes) fails its CRC or offset and writes nothing; the
  transfer stays, and the sender sends that line again.
- **Wi-Fi:** `POST /ota?sha256=<hex>` on the file server (crate::files), the whole image in one request. It
  starves the CC link while 3 MB cross into flash (the hub dropped the board for a few seconds).

Either way only in SETTING and disarmed, one at a time: the image (esptool elf2image's app.bin, as firmware/flash.sh
makes it) goes into the next slot, its SHA-256 is checked against the sender's and its header and app description
read back: the bootloader never re-hashes the app (CONFIG_BOOTLOADER_SKIP_VALIDATE_ALWAYS), so these are its checks. `ota status` names the running, the boot and the next slot, the running build (the first
8 bytes of its ELF's SHA-256, as esptool stamps it) and what was received; `ota commit confirm` (SETTING, disarmed)
makes the received slot the one otadata boots, and the next `reboot` runs it.

**No app rollback, ever** (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=n): a reset always boots the slot otadata selects.
`arm` refuses while the running slot is not the selected one ([`problem`]), so a committed build never waits for
a reset in flight.

**No flash mapping:** the image goes in by SPI1 erases and writes (esp_partition, as the ring writer writes the
telemetry ring), checked by our own SHA-256 and the image's header and app description (plain reads), and the slot
is selected by writing otadata the way IDF's esp_rewrite_ota_data does (an alternating sector, the sequence number,
its CRC). esp_ota_end and esp_ota_set_boot_partition verify the image by mapping it, which turns the cache off and
parks the flight core: from a PSRAM stack it asserted, and from an internal one the chip's watchdog reset the board
in the verify, in both directions. The firmware maps flash nowhere at runtime.

### `pub fn problem() -> Option<String>`

Why `arm` refuses: the running slot is not the one otadata boots (a committed build waits for a reboot on the
ground, never for a reset in flight); None when they are the same.

### `pub fn receive(mut request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()>`

`POST /ota?sha256=<hex>`: the whole image in one request (see the module notes). Answers 200 with the slot, size
and build; 409 refused (off the pad, armed, busy, bad terms); 500 a write or a check that failed (the slot is left
unselected).

### `pub fn register_commands()`

Reads the slots (on the main task: its stack is internal) and registers the OTA's console commands (crate::gcl).

## `firmware::owner`

The bus owners (doc/specs/avionics-port.md §4.2, M5 design decisions): one thread per I2C port, on core 1, that
creates its port (so the port's interrupt runs on core 1) and is the only user of it from then on. At boot the
owner lends the port to main, which scans it for the layout and sets its parts up in config order (plan B's
`Controller.setup`, nothing else running), then hands it back ([`start`]); from then the owner serves its mailbox
([`ask`]): the commands that need the bus (`i2c touch`, `i2c general-call`, `detect`'s scan, `probe`, `bustune`)
run on the owner, never beside it, over the parts that came up on its port (crate::devices). M6 adds the owner's
read table to the same thread. Every request is a change, run only in SETTING (a barometer's re-zero at DONE too):
the owner checks the stage again as it serves one, since the executor checked its gate once and a probe sweep then
takes seconds, so a launch detected meanwhile never runs a probe in flight.

The stacks are internal: the bus threads are flight-path threads (§4.7, internal RAM budget), and an internal
stack also keeps the thread's allocations internal (crate::allocator's rule).

### `pub enum Request`

What a command asks an owner to do.

### `pub type Verdict = (&'static str, Option<String>)`

A part's probe verdict: None when healthy, else why (plan B's `probe()`).

### `pub enum Answer`

An owner's answer.

### `pub struct Parked`

A spawned owner waiting for its port back (from [`spawn`] to [`start`]).

### `pub type Rearmed = (&'static str, Result<(), String>)`

One part's re-arm after a general call: its name, and the fault when the re-arm failed.

### `pub fn calibrations() -> Vec<(&'static str, String)>`

Every part's pending calibration, by name, sorted: empty when nothing is pending. The magnetometer's and the
BNO055's are worded here, on the executor, from their published state (crate::sensors::owed_calibrations).

### `pub fn spawn(id: u8, sda: u8, scl: u8, clock_hz: u32) -> anyhow::Result<(Parked, I2cPort)>`

Spawns port `id`'s owner (core 1), which creates the port on pins `sda`/`scl` at `clock_hz` and lends it to the
caller until [`start`].

Errors: the thread would not spawn, or the port would not be created (its owner then ends).

### `pub fn start(parked: Parked, port: I2cPort, mut members: Vec<Member>, topology: &'static Topology, table: &'static Table<'static>) -> anyhow::Result<()>`

Hands the port back to its owner with the parts that came up on it (crate::devices), each run with its `table`
entry, whose health the run writes; the owner serves its mailbox from then on.

Errors: the owner is gone.

### `pub fn task(id: u8) -> Option<sys::TaskHandle_t>`

Port `id`'s owner task, once its thread started: the control tick wakes it.

### `pub fn rearm(port: &mut I2cPort, members: &mut [Member], sender: &str, delay: &mut Delay) -> Vec<Rearmed>`

After a general call on `port` (an I2C software reset of every part that honours it), re-arms each of its
`members` but the `sender`, in their order (by name, as plan B walks `Inspector.names()`, icp10111.py:293-329),
logging plan B's line for each one that fails. Returns each re-arm; the caller records the walk ([`record`]).

### `pub fn record(port: u8, sender: &str, calls: u32, rearmed: &[Rearmed])`

Records general calls on port `port` by `sender`: `calls` of them, and `rearmed` (every re-arm they caused, in
order). One log line, and one oops record (doc/specs/avionics-port.md §4.8) when anything was re-armed: a bring-up
whose ICP keeps calling would otherwise fill the 16-slot panic log with records of nothing and push the real
panics out.

### `pub fn ask(port: u8, request: Request, deadline: Duration) -> Result<Answer, String>`

Asks port `port`'s owner to run `request`, and waits up to `deadline` for its answer.

Errors: no such port, its owner not started (yet) or gone, no answer in time, or the owner refused the request in
this stage (the refusal's text).

### `pub fn scan(port: u8) -> Result<Addresses, String>`

Port `port`'s scan, on its owner (the `detect` command): what acknowledges.

Errors: no owner, no answer in time, or the scan failed (the bus busy or broken).

### `pub fn probe_port(port: u8, name: Option<&str>) -> Result<Vec<Verdict>, String>`

Port `port`'s parts probed on its owner: the one named, or every one (`probe`, `verify`, `arm`). An owner that
is not started (no such port in the config) has no parts: an empty list.

Errors: the owner did not answer in time, or is gone.

### `pub fn update(name: &str, properties: &serde_json::Value) -> Result<Option<Result<Vec<&'static str>, String>>, String>`

The part `name`'s property change, on whichever owner holds it (CC `update`): None when no owner holds it.

Errors: the owner did not answer in time; the inner error is the part's refusal.

### `pub fn calibrate(name: &str, persist: impl Fn(&'static str, &[u8]) -> Result<(), String>) -> Result<Option<Option<String>>, String>`

The part `name`'s calibration on its owner (CC `calibrate <name>`): None when no owner holds such a part, else
plan B's answer (None success, else why). A calibration computed but not in force (the BMM350's) is handed to
`persist` (NVS, on this executor thread) and only then applied on the owner (D9 of the M9 plan: plan B's order); a
calibration that would not persist is not applied.

Errors: the owner did not answer in time.

### `pub fn bustune(port: u8, clock_hz: u32) -> Result<Result<Vec<Verdict>, I2cError>, String>`

Port `port` run at `clock_hz` on its owner, and its parts probed there (`bustune`).

Errors: no owner, or no answer in time; the inner error is the port refusing the speed.

### `pub fn register_commands()`

The owners' console commands (the Glider Command Language): `i2c general-call`, `i2c read` (a part's register
facts), and `i2c touch`, which proves the wedge rule on the board. All three hold the owner off its parts while
they run, so they run only in SETTING while disarmed.

### Constants

- `const ANSWER_DEADLINE: Duration = Duration::from_secs(8)` — How long [`ask`] waits for an owner's answer: a touch is a few transfers of at most 50 ms each, a probe of every part a conversion or two each.

## `firmware::panics_logger`

The panics logger (crates/panics, doc/specs/avionics-port.md "Panic log"): the board's last panics as tagged text
records in the `panic` partition, round-robin, the way Linux's mtdoops keeps oopses.
- A fatal panic (a panic, a watchdog, a brownout: the board reset): components/panic_capture keeps what the panic
  handler printed, and a Rust panic's message, in RTC RAM across the reset, and [`start`] writes it at the next boot
  with the OS up -- never from the panic handler. A watchdog or brownout reset with nothing captured is recorded
  too, a header alone.
- An oops (the board noticed something broken and went on): [`oops`] hands it to the panic log's own thread, which
  writes it; the caller never waits for the flash.

Each fatal record is also logged at boot, so the session's recorder.log and CC's log stream carry it. Read the log
with `panic log` (a summary) or `GET /panic` on the file server (the records as text, newest first).

### `pub fn start(boot_id: u32, session: &str, config_id: &str)`

Opens the panic log and records what the last boot left; then names this boot for its own capture and installs the
Rust panic hook. Call it on main right after the recorder opens, so its lines are kept.

Args:
* `boot_id` - this boot's id.
* `session` - its session label.
* `config_id` - the saved board config's id: the layout has not rewritten it yet ([`name_config`] names the
  rewritten one).

### `pub fn name_config(config_id: &str)`

Names the config this boot runs once the layout has rewritten it (plan B main.py:120-124: the post-layout id),
for this boot's capture and its oopses from here on. [`start`] named the saved config's id, since a panic in the
layout itself is that config's.

### `pub fn start_oops() -> anyhow::Result<()>`

Starts the panic log's thread (core 0, priority 2, PSRAM stack: it never calls esp_mmu_map/unmap, the one
runtime path that turns the cache off), which writes the oopses [`oops`] queues; until it runs they are dropped.
Call it after the camera's encoder reserved its internal block (the thread's control block is internal).

Errors: the thread would not spawn.

### `pub fn oops(cause: &str, text: &str)`

Records an oops: something broke and the board went on. Never waits: the record goes to the panic log's thread,
and when its queue is full (or the log is not running) the oops is dropped, counted (`panic log`'s `dropped`).

Args:
* `cause` - what happened, one word (`c6-link-parked`): the record's `cause` tag.
* `text` - what the record says about it.

### `pub fn texts(seq: Option<u32>) -> Option<Vec<u8>>`

The records as text, newest first, each followed by an empty line; only `seq`'s when given (empty when no record
has it). A record whose read fails is a line saying so. None when the log is not open.

### `pub fn register_commands()`

The panic log's console command (the Glider Command Language): `panic log`.

## `firmware::recorder`

The recorder on the board (doc/specs/avionics-port.md "Storage design", and "Bring-up order" step 3): the glue
between crates/recorder, crates/flash_ring and the ESP32-P4. Power can drop at any moment, so every record goes
to flash as soon as it can; the ring_saver copies the flash to the SD card behind it, and the ring_eraser erases
what the saver has passed.

```text
producers (any core) --binary records--> PSRAM rings --> ring_writer (core 0, prio 7) --> `log` partition, 20 MiB
  supervisor, executor, eraser,          + misc ring          |  one chunk per ~10 ms tick    |          ^
  everyone else (log bridge, IDF)                             +--> CC tees (`log`, `tlm`)     |          | 4 KiB erases
                                                                                              v          |
                      /sdcard/<session>/recordings/... <-- ring_saver (core 0, prio 4) -- saver_position --+
                      /sdcard/sessions.csv                 text, batches, fsync           ring_eraser (core 0, prio 3)
```

The bring-up runs in two steps, so no line of the boot is lost and boot never waits on flash:
1. [`open`], on main right after the board config: the stream registry, the producer rings and the CC tees in
   PSRAM, and the Recorder, published to every thread. From then on every log line and telemetry row queues in
   PSRAM, whoever makes it.
2. [`start`], on main once the SD card's state is known and the camera's encoder has its internal RAM: spawns the
   ring_writer. Its bring-up routine (spec "The scan, the positions and the session-start record") reads the ring
   positions and the previous boot's session-start record from the `cfg` NVS, scans the 5120 block headers,
   writes this boot's record, spawns the ring_eraser and, with a card mounted, the ring_saver, and only then
   starts programming; boot goes on meanwhile, and the producers queue.

What lives in the `cfg` NVS, namespace `recorder`, all of it hints the scan can do without:
* `writer`: the writer position, written about once a second by the ring_eraser, never by the writer itself (a
  u64: block sequence << 32 | offset); it only keeps the block numbering from going back after the newest blocks
  were erased;
* `saver`: the saver position, the same u64, written by the ring_saver after every batch it commits (about once a
  second when it keeps up), and by the bring-up once, before the session-start record names a new card. It is
  the saver's only record of progress, and is trusted only for the card it was saved to;
* `start`: the session-start record, once per boot (JSON): boot id, session, config id, reset cause, the card
  (its FAT volume serial `XXXX-XXXX`, `unknown` when unreadable, null with none), and the positions this boot
  started from.

**The card the saver hint belongs to** (spec "Bring-up", steps 4 and 5). A boot compares the mounted card's
volume serial with the card the previous boot's record names. The same card: the saver starts at the hint, and
the eraser takes every block behind it as saved. Another card, or one whose serial would not read: the saver
starts at the oldest block in flash, so the new card gets everything still there, and nothing counts as saved.
No card: no saver (the uart_saver walks with a cursor of its own all the same, M19), and the blocks behind the
hint count as saved, since they are on the card it was saved to. The bring-up writes the new saver start as the hint
BEFORE the record that names the new card, so a cut between the two never leaves an old card's position beside
a record naming the new card: the worst a cut there does is re-save into duplicates (spec "Duplicates").

IDF serialises every NVS call, on every partition, behind one global lock, which a config save (the executor's
~10 KB blob) holds for tens of ms. So the ring_writer never touches the NVS after its bring-up: the eraser and
the saver, the two lowest ring tasks, write the hints and can wait for the lock without harm. What reaches the
writer is only the flash lock of the NVS's own flash calls (a page erase, at most ~16 ms, inside the writer's
power-cut bound).

Nothing here runs on core 1, the flight core, and no producer takes a lock (crates/recorder). Every big buffer is
in PSRAM. What stays in internal RAM is the three ring threads' stacks (writer 8 KiB, eraser 6 KiB, saver 8 KiB;
the supervisor's runs in PSRAM, supervisor.rs) and the crates' working buffers: about 18 KiB for the writer and
the eraser (the writer's chunk, the ring's chunk and its 4 KiB erased-check buffer, the eraser's per-slot failure
counts), and for the saver its 4 KiB block buffer, which esp_flash fills directly (a PSRAM buffer would bounce
through internal DMA memory), its 1 KiB tail buffer, and its batch while it collects: the lines of up to a second,
one growing buffer per file, internal while a file's buffer is 4 KiB or less. The 48 KiB block table is in PSRAM:
the heap puts any allocation over 4 KiB there (CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL).

### `pub enum Producer`

The producer rings a thread attaches to, by the index `Recorder::attach` takes (spec "Producer side": one ring per
producing thread). The flight threads (control, i2c0, i2c1, gnss) join as they are ported. Every other thread
shares the `misc` ring.

### `pub struct Card`

The microSD card as the ring knows it at boot: mounted, and named by its FAT volume serial (spec "Bring-up", step
4: the saver hint is trusted only for the card it was saved to).

- `fn name(self) -> String` — The card as the session-start record names it: partition 1's serial as `blkid` shows it (`1A2B-3C4D`), or `unknown`.

### `pub struct RingView`

The flash ring once the bring-up scan has run: what `GET /ring`, `GET /ring/image`, the `ring` command, `stats`
and `health` read.

### `pub struct Partition(*const sys::esp_partition_t)`

A data partition found by its label, as a [`Flash`]: the `log` partition (partitions.csv: 20 MiB at 0xc00000), the
ring's, and the `panic` partition, the panic log's. Each ring task holds its own copy; esp_flash serialises the
calls, NVS's on the same chip among them.

- `fn by_label(label: &CStr) -> Option<Partition>` — The data partition labelled `label` (`log`, the ring; `panic`, crate::panics_logger); None when the table has none.

### `pub struct StatsReport`

`stats recorder`: plan B's report (the session and the two queues' counters), plus the flash ring and the saver.

### `pub struct InspectReport`

`inspect recorder`: plan B's properties (the session, the status line's period, the queues' sizes), plus the flash
ring and the saver.

### `pub fn get() -> Option<&'static Recorder>`

The running recorder; None before [`open`], or when PSRAM would not hold its rings.

### `pub fn health_stream() -> Option<StreamId>`

The health.csv stream; None without a recorder.

### `pub fn device_stream(name: &str) -> Option<StreamId>`

The stream of device `name` (controller::stream); None without a recorder, or for a device that writes none.

### `pub fn telemetry(stream: &str) -> Option<Telemetry<'static>>`

The producer side of the device stream `stream`: None without a recorder, or when the registry has no such stream.

### `pub fn rows(name: &'static str) -> Rows<'static>`

The device `name`'s stream as its run records it: its rows, a refused one logged once a run as `<name> ::
telemetry <overflow>` (recorder::Rows); recording nothing without a recorder or such a stream.

### `pub fn ring() -> Option<&'static RingView>`

The flash ring; None until the writer's bring-up scan has run (or when it found no `log` partition).

### `pub fn session_prefix(document: &Value, boot_id: Option<i32>) -> String`

This boot's file prefix (spec "Row, log and index text", Session prefix; plan B `Recorder.session()`): the board
config's `recorder.session` when it is a valid label, else the boot id as `%06u`, else
`YYYYMMDD_HHMMSS_<6 random digits>` from the clock (2000-01-01 plus the uptime while it is unset).

A label the card's layout reserves (`videos`, `recordings` or `video`, in any case: crates/recorder layout
`is_reserved`) counts as no label: `videos` would put the session's recordings into the videos directory, on a
two-partition card the videos partition, and the other two are an M4 card's root directories, which the tools
still read. [`open`] logs that it was ignored.

Args:
* `document` - the board config this boot runs.
* `boot_id` - this boot's NVS count; None when uncounted.

Returns the prefix: it names the SD directory, every file of the boot, the video directory and whoami's session.

### `pub fn open(document: &Value, boot_id: Option<i32>, session: &str, flight: bool) -> anyhow::Result<()>`

Builds the recorder and publishes it ([`get`]): the stream registry, the producer rings and the CC tees in PSRAM,
and the Recorder itself. From here on every log line and telemetry row queues in PSRAM, to be programmed once
[`start`] has the ring_writer running. Registers the recorder's commands (`ring`, `recorder update`).

Args:
* `document` - the board config this boot runs: `recorder.telemetry_ms`, `recorder.stats_ms`,
  `recorder.session` (only to log a label that was ignored) and the board id.
* `boot_id` - this boot's NVS count; None when uncounted.
* `session` - this boot's file prefix ([`session_prefix`]).
* `flight` - whether the bring-up starts in flight: a crumb in BOOSTING..LANDING (crate::warm::peek), a reboot in
  flight. The warm start's restore lowers it again unless the boot is warm.

The config id comes later, with [`start`]: the layout rewrites the config after this, and the id is the
rewritten config's (plan B main.py:120-124).

Errors: PSRAM would not hold a ring, the prefix is not a session label, the registry refused a
stream, or the recorder opened before. The board runs on without recording then.

### `pub fn start(settings: EspCustomNvsPartition, card: Option<Card>, config_id: &str) -> anyhow::Result<()>`

Spawns the ring_writer (core 0, priority 7). Its bring-up routine, run at the main task's priority, scans the `log`
partition, writes the session-start record and spawns the ring_eraser and, with a card, the ring_saver; then the
writer steps forever. Boot goes on meanwhile.

The three ring threads run on PSRAM stacks, so their buffers are PSRAM too (crate's global allocator): their flash
reads and programs go through [`BOUNCE`], and their card writes through the SD driver's own bounce. Internal RAM is
the camera's and the flight's (2026-10-13: the flight load had taken the DMA-capable heap to ~0.3 KB).

Args:
* `settings` - the `cfg` partition: the ring positions and the session-start record live there.
* `card` - the mounted microSD card; None without one. Its volume serial tells whether the previous boot's saver
  position is this card's (see the module notes).
* `config_id` - the running config's id, the post-layout one: bound into the session.csv rows and written into
  the session-start record.

Errors: the recorder did not open, or the thread would not spawn.

### `pub fn catch_up()`

Waits until the ring_writer has programmed, and so teed, every record queued before this call, for at most
[`CATCH_UP_LIMIT_MS`]; at once when no writer runs. The CC `log` and `tlm` drains call it first.

A log line reaches its tee only once the writer programs it, at least the merge horizon (20 ms) after it was
logged, where plan B teed it at once. Without the wait a drain that closes the window (`log 0` right after the
command whose lines the operator wants) would miss the last ~30 ms of lines, and the writer would then find the
window closed and drop them: the contract's `probe mission` then `log 0` would come back empty.

### `pub fn start_uart<U: esp_idf_svc::hal::uart::Uart + 'static>(_uart: U, tx: u8, baud: u32) -> anyhow::Result<()>`

Installs the uart_saver's UART (M19, doc/specs/avionics-port.md M19): `uart`'s TX on GPIO `tx` at `baud`, to a
receiver that keeps the recorder's lines when the board and its card are lost (the C6 flight recorder of M20,
src/avionics/recorder; a Luckfox's recorderd): plan B's wire lines, each naming its session. The uart_saver itself
starts with the ring's bring-up ([`start_relay`]), which it needs.

Errors: the driver would not install.

### `pub fn backlog() -> Option<(u32, Option<f32>)>`

The flash->SD backlog for the recovery beacon's page (crate::beacon): the ring's unsaved blocks and how many
seconds of recording they hold; None before the bring-up scan has run.

### `pub fn stats_report() -> Option<StatsReport>`

The `stats recorder` report; None without a recorder (it is then not among the objects).

### `pub fn inspect_report() -> Option<InspectReport>`

The `inspect recorder` report; None without a recorder.

### `pub fn probe() -> Option<Option<String>>`

The recorder's probe, the ring gate (controller::ring_gate): the flash ring can take a flight once its bring-up
scan has run, its eraser runs and the eraser holds its reserve erased ahead of the writer. Logged as plan B's
recorder probe logs (tasks/recorder.py:30-48): `recorder :: probe: recorder rings ...`, then `probe: rings ok (<the
figures judged>)` or `probe FAILED: recorder rings: <why>`. It walks the block table as `health` does, but never
moves health's backlog trend ([`degraded`]).

Returns: None without a recorder (nothing to probe); else None when the ring can take a flight, or the verdict.

### `pub fn degraded() -> Vec<&'static str>`

The flash ring's degraded tokens for `health` (spec "The ring_eraser and newest wins", Degraded tokens):
* `ring-unformatted`: the eraser is formatting the foreign slots of a first boot;
* `ring-low`: fewer erased blocks ahead of the writer than the ~1 % reserve: the eraser is not keeping up;
* `ring-full`: newest wins erased an unsaved block within the last minute ([`RING_FULL_HOLD_MS`]);
* `saver-behind`: a card is mounted, and over a minute the backlog did not fall and the saver never caught up
  ([`BacklogTrend::behind`]): a dead card, a card slower than the writer, a saver that never started.

Nothing before the bring-up scan has run. Call it from `health` alone: it is where the backlog's trend is followed,
from one health read to the next.

### Constants

- `const NAME: &str = "recorder"` — The recorder's object name (plan B `Recorder.name`): `inspect`, `stats`, `update` and `probe` reach it by it while a recorder runs, whatever its config entry is called, and its probe lines start with it.
- `const VETOES: &str = "vetoes"` — The device key `vetoes.csv` is registered under ([`device_stream`]): the stage machine's veto episodes, written by its entry beside `<name>.csv` (crates/controller `vetoes`).

## `firmware::sd`

The microSD card (components/sd_card): mount once at boot, name the card by its FAT volume serial, report free
space, a write test an operator can run from the console, and the card's files from the console or the hub: `ls`,
`df`, `rm` (the flight recorder's console has the same three, src/avionics/recorder).

The card holds one or two FAT volumes (doc/specs/avionics-port.md "SD and flash file layout"): partition 1, the
recordings, at [`MOUNT`] (`/sdcard`), and on a two-partition card partition 2, the videos, at [`videos_mount`]
(`/sdcard/videos`, crates/recorder layout's `videos` directory under the card's root). A one-partition card keeps
`videos/` as a plain directory on its one volume. Paths on the card come from crates/recorder's layout, never
from here.

### `pub enum Layout`

How the card's volumes are mounted (components/sd_card `sd_card_layout_t`).

- `fn name(self) -> &'static str` — How the boot log names it.

### `pub struct Space`

One mounted volume's size and free space, in whole megabytes.

### `pub struct Volumes`

The card's volumes as [`measure_free`] found them.

### `pub struct WriteTest`

What a write test measured.

### `pub fn videos_mount() -> String`

Where the videos partition is mounted on a two-partition card: the layout's videos directory under [`MOUNT`]
(14 bytes, within IDF's ESP_VFS_PATH_MAX of 15). IDF's VFS takes the longest matching prefix, so it shadows that
directory of partition 1. vfs_fat keeps it cut to 13 bytes, so its by-path calls (`esp_vfs_fat_info`) never find
it: nothing here asks them by path ([`measure_free`] goes by the FatFs drive).

### `pub fn mount() -> bool`

Mount the card (never formats): partition 1 at [`MOUNT`] and, when the MBR's partitions 1 and 2 are both FAT and
sound, partition 2 at [`videos_mount`]. False when there is no usable card (a sector 0 that will not read
included); the rest of the board runs on.

### `pub fn layout() -> Option<Layout>`

How the card is mounted; None without a card.

### `pub fn videos_mounted() -> bool`

Whether the videos partition is mounted at [`videos_mount`]: a two-partition card whose partition 2 mounted.

### `pub fn volume_serial() -> Option<u32>`

The mounted card's FAT volume serial number (`blkid`'s UUID `XXXX-XXXX`), partition 1's, read from its boot
sector: what the session-start record names the card by, so the next boot knows whether the card is the one this
boot saved to. None when no card is mounted or the boot sector would not read.

### `pub fn videos_serial() -> Option<u32>`

The videos partition's FAT volume serial number, for the boot log; None unless it is mounted, or when its boot
sector would not read.

### `pub fn serial_name(serial: u32) -> String`

A FAT volume serial as `blkid` shows it: `1A2B-3C4D`.

### `pub fn measure_free() -> Volumes`

Measure both volumes' free space now (slow: each volume's lock and possibly a FAT scan) and remember the
recordings volume's, which `health` reports as `sd_free_mb`. Call off the hot paths -- after mounting, after a
write burst.

Returns each volume as measured, for the caller to log ([`describe`]).

### `pub fn describe(volumes: &Volumes) -> String`

The volumes as one log text: `recordings 4092 MB, 4000 MB free; videos 11000 MB, 10990 MB free`, or the one
volume's figures alone on a one-partition card.

### `pub fn free_megabytes() -> Option<u32>`

The last measured free space of the recordings volume (partition 1, or a one-partition card's only volume) in
whole megabytes; never touches the card. None until measured.

### `pub fn write_test(megabytes: usize, kilobytes_per_second: Option<u32>) -> anyhow::Result<WriteTest>`

Write `megabytes` to `<mount>/COLUDO/sdtest.bin` in 128 KB chunks from a cache-aligned PSRAM buffer (the
video/mover path), then remove it. `kilobytes_per_second` paces the writes (the video case: ~1000 at 8 Mbit/s);
None writes flat out.

### `pub fn register_commands(session: String)`

The card's console commands (the Glider Command Language, crate::gcl), registered when the card mounts.

Args:
* `session` - this boot's session, which `rm` refuses.

### Constants

- `const MOUNT: &str = "/sdcard"` — Where the card's root is mounted: partition 1 (or a one-partition card's only volume).

## `firmware::sensors`

The board's side of the I2C parts (crates/parts, doc/specs/avionics-refactor.md R2): the platform they run on
([`PLATFORM`]: the control thread's GPIO interrupts and lapse watches, the recorder's streams), a laser's XSHUT
line on esp-idf-hal ([`Xshut`]), the registry of their published state by name ([`register`], [`shared`]), and
the calibrations' NVS: read at bring-up ([`restore_calibrations`]), written on the executor
([`persist_pitot_tare`]; the BMM350's and the BNO055's blobs through owner::persist).

### `pub type Part = parts::Part<Xshut>`

One I2C part on this board: its laser XSHUT on an esp-idf-hal pin.

### `pub type Member = parts::Member<Xshut>`

An I2C part that came up on this board.

### `pub struct Board`

The board the parts run on.

### `pub struct Xshut(ManuallyDrop<PinDriver<'static, Output>>)`

A VL53 ranger's XSHUT line, when the config wires one (v0.1; layout nulls it on v1.x), driven for the rest of the
run once a part first takes it. Dropping an esp-idf-hal PinDriver disables the pad, so the line would float when a
failed attempt's part is dropped: on a pin two laser entries share (the default config's GPIO5), the fitted
laser's XSHUT after the other entry's attempts. Plan B's `Pin` was never released either.

- `fn output(gpio: u8) -> Result<Xshut, String>` — GPIO `gpio` as an output for a ranger's XSHUT (crates/parts Part::build's `xshut`).

### `pub fn register(member: &Member)`

Registers `member`'s published state under its name, when it has one.

### `pub fn shared(name: &str) -> Option<Shared>`

The published state of the part named `name`.

### `pub fn owed_calibrations() -> Vec<(&'static str, String)>`

The calibrations the magnetometers and the BNO055s owe, by name, in plan B's words (bmm350.calibration(),
bno055.calibration()): worded here, on the executor, from their published state, so the owner never formats on its
tick (D9 of the M9 plan).

### `pub fn absolute_state() -> Option<AbsoluteState>`

The BNO055's state, None when no BNO055 came up (`verify`, the heartbeat).

### `pub fn pitot_zero_pa() -> Option<f32>`

The pitot's zero offset in force, Pa, None before its state is known or when no pitot came up (the checkpoint).

### `pub fn persist_pitot_tare(partition: &esp_idf_svc::nvs::EspDefaultNvsPartition) -> Option<String>`

Persists the pitot tare the owner captured, if one waits (plan B sdp810._persist_zero: NVS `sdp810_zero`, the
Pa x 100 i32 shared with plan B), on the executor after the owner answered a `calibrate` or an `update`. None when
nothing waited or it persisted, else why it did not (the tare stays in force either way).

### `pub fn restore_calibrations(partition: &esp_idf_svc::nvs::EspDefaultNvsPartition, members: &mut [Vec<Member>; 2], ports: &mut [(crate::owner::Parked, I2cPort)])`

Restores what each part learned from NVS after its setup (crates/parts restore): each BNO055's profile over its
lent port, then the pitot's tare and each magnetometer's calibration. Once at bring-up, on main, before the owners
run.

### Constants

- `static PLATFORM: Board = Board` — The one platform every part's run is handed.

## `firmware::sim_link`

The HITL sim-link's board end (doc/specs/sim-link.md, M12), only with an enabled `hitl` entry: one thread (core 0,
priority 12, PSRAM stack) owns a UDP socket on the entry's `sim_port`. Each SENSORS frame the host sends that
decodes and is newer than the last one applied is published at once: every present quantity through the `hitl`
source's databoard writers (priority 0), stamped with the frame's arrival on this board's clock, `change` derived
from the altitude as a barometer derives it, and a row in each simulated sensor's own stream in its driver's shape,
so the session reads as a flight's. Each applied frame is answered with a FINS frame to its sender: the stage, the
arming, the launch, and no fins (M12 flies passive).

A frame that is refused (crates/sim_link `Refusal`) or not newer is dropped and counted; `inspect hitl` shows the
counts. The real sensors the simulator replaces were masked at boot (board_config::mask_simulated), and the
entry refused to come up if one still runs (crate::devices).

### `pub struct Inspection`

What `inspect hitl` shows (the port's own; plan B's hitl inspected its body, now the host's).

### `pub fn running() -> bool`

Whether the sim-link runs this boot: the link then joins again after the launch (crate::link), since the
simulated sensors ride the Wi-Fi.

### `pub fn start(member: HitlMember, topology: &'static Topology) -> anyhow::Result<()>`

Starts the sim-link thread for `member`, its quantities found in `topology`.

Errors: the thread would not spawn.

### `pub fn inspection() -> Option<Inspection>`

The sim-link's state; None without a running `hitl` entry.

## `firmware::status`

The flight word (doc/specs/avionics-port.md §4.1): the stage, the operator hold, the arming and the launch in one
atomic. A reader takes one [`sequencer::FlightWord::load`], a consistent snapshot, so health never pairs one
moment's stage with another's arming. Its one writer commits a warm start's restored flight on main, before any
thread runs (crate::warm), and is then handed to the control thread (crate::control), which commits the stage
machine's advances, the separation's, and the operator's requests, posted to its request slot.

### Constants

- `static FLIGHT: FlightWord = FlightWord::new()` — This board's flight word: on the ground until a warm start's restore or the control thread commits a change.
- `static WARM_STARTED: AtomicBool = AtomicBool::new(false)` — Whether this boot is a warm start: a reset in flight restored the stage the crumb saved (crate::warm). Set once on main before the threads start, never cleared; health's `degraded` names it.

## `firmware::supervisor`

The supervisor thread (doc/specs/avionics-port.md "Threads, cores, priorities": core 0, priority 9), on a 100 ms
base tick that never ends:
 * the task watchdog, when the board config enables a `watchdog` (plan B's tasks/watchdog.py): on its first loop
   the supervisor reconfigures IDF's task watchdog to panic after `wdt_timeout_ms` with no idle task watched (the
   built sdkconfig only warns, and plan B's `machine.WDT` reset), subscribes itself, and feeds it every base tick.
   A supervisor that stops looping, wedged or starved, panics the board, a reset the warm start recovers from;
 * the control thread's stall check (controller::Stall), every `period_ms` on the base tick's grid: while the board
   controls a flight (armed, launched, in an airborne stage), a control tick older than `stall_ms` (and two tick
   periods) restarts the board. The line `<watchdog> :: control loop stalled (stage=X) -> reset` is kept in this
   session's recorder.log first, and the restart (components/safe_restart) never stops Wi-Fi: through the C6 that
   is a remote call that a parked link makes wait seconds, past the task watchdog. A boot that is a stall restart's
   own warm start never restarts for a stall again (crate::warm::stall_restart_suppressed), so a stall that comes
   back every boot leaves the board stalled rather than cycling it;
 * health.csv at 1 Hz, plan B's board_health columns unchanged (spec "Row, log and index text"), so
   recorder_flight, flight_report and flight_video read the Rust board's vitals as they read plan B's;
 * the checkpoint (crate::warm::Keeping) every `poll_ms`, when the board config enables one: the crumb a reset in
   flight warm-starts from, and checkpoint.csv;
 * the GNSS pad drift calibration (crate::gnss::Calibration) every `period_ms`, when the board config enables one.

It runs with or without a recorder: without one, its rows go nowhere and the rest goes on; and once subscribed to
the task watchdog it never leaves its loop. The field logic joins it as it is ported (M11). The
NVS and flash locks its checkpoint takes are core 0's: the flight tick never waits on them. Its console lines go
through the logger's printer, as the flight core's do (crate::logging): a console reply holds the UART for
seconds, and a line printed in place would wait that out with the task watchdog unfed.

The port, against plan B: the task watchdog watches the supervisor alone, and the control thread through the stall
check, where plan B's watched its whole event loop (a hung I2C bus included: here a bus transfer times out in 50
ms, so a hung bus shows as failing reads on a live owner). It is a task entry, so a timeout's backtrace names the
starved task. The stall check runs while the board controls a flight, a passive one included, where plan B's ran
only while its flight task controlled the fins. A refused key fails the entry at bring-up, where plan B's arm
refused the task its run loop crashed on.

Its neighbours in the record need nothing from it: the ring_writer logs the `'session': '<s>'` status line every
`recorder.stats_ms` itself, and writes session.csv's `boot` and `anchor` rows; a clock set writes its own row
(commands/inspection.rs).

### `pub struct Inspection`

What `inspect` shows of the watchdog beyond its entry.

### `pub fn start(board: Arc<Board>, table: &'static Table<'static>, keeping: Option<Keeping>, watchdog: Option<WatchdogMember>, calibration: Option<Calibration>, tick_ms: f32) -> anyhow::Result<()>`

Spawns the supervisor (core 0, priority 9).

Args:
* `board` - the vitals it samples (the die temperature).
* `table` - the device table: the watchdog's entry is crashed when the task watchdog will not arm, and every
  supervised entry when the thread does not start.
* `keeping` - the checkpoint, when its entry came up and its NVS opened.
* `watchdog` - the watchdog, when its entry came up.
* `calibration` - the GNSS pad drift calibration, when its entry came up.
* `tick_ms` - the control tick's period, ms: the stall limit is at least two of them.

Errors: the thread would not spawn; the checkpoint's, the watchdog's and the calibration's entries are crashed
then.

### `pub fn starve() -> Result<(), &'static str>`

Stops feeding the task watchdog (`panic kind=twdt`), which panics the board `wdt_timeout_ms` later; the supervisor
runs on meanwhile.

Errors: the task watchdog is not armed, so nothing would reset the board.

### `pub fn stall_refused() -> Option<&'static str>`

Why `panic kind=stall` would not restart the board, None when it would: the stall check must run (an enabled
watchdog), its restart must not be suppressed, and the board must control a flight with its control thread
ticking. Without them the control thread would stop for good, and the board with it: `disarm` needs its tick.

### `pub fn inspect() -> Inspection`

The watchdog's [`Inspection`].

### Constants

- `const HEALTH_FILE: &str = "health.csv"` — The health stream's file: `<session>_health.csv` on the SD.
- `const HEALTH_FIELDS: [&str; 8] = [` — The health stream's columns after `uptime` (plan B tasks/board_health.py, doc/telemetry.md): * `temp` - the die temperature in °C, to 0.1; * `mem_free` - free 8-bit heap, internal and PSRAM together, bytes; * `load` - the busiest core's CPU load, whole percent; * `oom_s`, `land_s`, `leak_kbps`, `rescues`, `rescue_ms` - plan B's memory forecast and rescue, which Rust has no use for (no GC), and the landing forecast that comes with the elevation slope: `None` and 0, as the spec says.

## `firmware::thermal`

The ESP32-P4's on-die temperature sensor.

Native code at full load, profiling and the camera pipeline heat the die; the readings go on the health
heartbeat and on every bench run, so a number always comes with its thermal context.

### `pub struct Thermometer(TempSensorDriver<'static>)`

The started sensor.

- `fn new(sensor: TempSensor<'static>) -> anyhow::Result<Self>` — Install and start the sensor.
- `fn celsius(&self) -> Option<f32>` — The die temperature in °C, None when the read fails.

## `firmware::threads`

The firmware's threads: one way to spawn a std thread (a FreeRTOS task underneath) on a core, at a priority, with
its stack in the memory it needs (doc/specs/avionics-port.md §4.7, the internal RAM budget). esp-idf-svc takes a
thread's name, core, priority and stack memory from a thread-local spawn configuration, so every spawn sets it,
spawns, and puts the default back for the next plain `std::thread::spawn`.

### `pub enum Stack`

Where a thread's stack lives (§4.7).

### `pub struct Spec`

A thread to spawn.

### `pub fn spawn<T: Send + 'static>(spec: Spec, body: impl FnOnce() -> T + Send + 'static) -> anyhow::Result<JoinHandle<T>>`

Spawns `body` as `spec` describes, then puts the default spawn configuration back.

Errors: the configuration would not set, or the thread would not spawn.

## `firmware::time`

The board's monotonic clock: microseconds since boot (esp_timer), the one place that reads it.

### `pub fn uptime_us() -> u64`

Microseconds since boot, from esp_timer: monotonic, never negative, safe from any context (an ISR included).

## `firmware::video`

The flight camera (components/coludo_video): OV5647 -> ISP -> H.264 hardware encoder -> raw Annex-B on the
microSD, every frame stamped with the boot id and its uptime in an SEI NAL. Files go to
`/sdcard/<session>/videos/<start uptime ms>.h264` (a two-partition card's to `/sdcard/videos/<session>/`, its
second partition), and on the pad a recording moves on to a new file every
[`Settings::segment_seconds`] (300 s for the flight's), bounding what one damaged file can cost; the file running
when the board leaves SETTING runs on to the end (M7b, coludo_video.c). The directories are crates/recorder's
layout's (`directory`, `videos_partition_directory`). A two-partition card whose videos partition is not mounted
(it would not mount, or the table is unsound) records no video at all, rather than fill the recordings partition
the ring_saver needs (sd::Layout::VideosFailed).

**One owner, the `video` thread** (core 0, low priority, PSRAM stack), started only when the board config declares
a camera and the card mounts ([`start`]). It is the only caller of `coludo_video_start` and `coludo_video_stop`,
whose C side assumes one caller (its starts and stops never overlap). It looks at the flight word every
[`LOOK_PERIOD`] and follows the camera-by-stage rules (crates/sequencer `video`): armed in SETTING, the airframe
standing nose up on the rod for 2 s (the fused accel's +x, M9) starts the flight recording, replacing an
operator's -- at the arm itself on a board with no accelerometer; a disarm in SETTING stops it; DONE stops
whatever records; nothing starts outside SETTING. The GCL `video start` and `video stop` post to its mailbox and wait for the answer at most
[`ANSWER_DEADLINE`], so a recorder call that never returns cannot hold the CC executor. `video stats` only copies
the counters, on the executor. Video is a utility: none of this blocks `arm` or takes anything from the flight.

### `pub struct Camera`

The board's camera as the board config declares it (board_config::camera), resolved for the driver: its
oscillator, the stock one when the config gives none (logged).

- `fn from_config(document: &serde_json::Value) -> Option<Self>` — The camera the config declares, or None: no camera, no video (and no encoder reservation).

### `pub struct Settings`

What to record.

### `pub fn reserve() -> anyhow::Result<()>`

Reserves the encoder for the flight setting at boot, while the internal heap is still whole (coludo_video.h,
coludo_video_reserve): call it on the main task (core 0, where the encode task runs) before the ring threads start.

### `pub fn preview() -> Option<(Vec<u8>, i64)>`

The latest preview JPEG and its frame's uptime in us (M7b), None before the first: one is made a second while the
camera records on the pad.

### `pub fn stats() -> sys::coludo_video_stats_t`

The counters of the running (or last) recording.

### `pub fn start(session: String, boot_id: u32, camera: Camera, flight: &'static FlightWord, accel: Option<&'static databoard::Parameter<[f32; 3]>>) -> anyhow::Result<()>`

Starts the video thread (core 0, priority 2, PSRAM stack), the camera's one owner, and registers the camera's
console commands, which post to it. Call it when the board config declares a camera and the card mounts:
recording needs both. The thread's control block is internal RAM, so after the encoder's reservation
([`reserve`]).

Args:
* `session` - this boot's session: the videos directory the files go to;
* `boot_id` - this boot's id, stamped into every frame;
* `camera` - the camera the board config declares;
* `flight` - the flight word the flight recording follows;
* `accel` - the fused `accel` quantity the rod trigger reads, None on a board with no accelerometer.

Errors: the thread would not spawn. Nothing is registered then, so `help` lists no `video`.

### Constants

- `const OV5647_BAYER: i8 = 3` — The OV5647's Bayer order as the ISP must be told it: BGGR. esp_cam_sensor's tables run the readout mirrored but declare GBRG, which swapped green and magenta; BGGR (what Linux's ov5647 driver declares for the same mirrored modes) gives the colour bars exactly: white, yellow, cyan, green, magenta, red, blue, black.
- `const STOCK_CLOCK_HZ: u32 = 25_000_000` — The oscillator esp_cam_sensor's OV5647 tables assume (the Raspberry-Pi reference module's), used when the board config's camera gives no usable `clock_hz`. Modules differ: the taster's runs at 24 MHz and 7F's Arducam at 27 MHz, and a wrong clock stretches every frame rate by its ratio (measured from the SEI frame stamps, tools/video_sei.py), so each board's config declares its own (board_config::Camera).
- `const FLIGHT: Settings = Settings { width: 1920, height: 1080, fps: 25, kilobits: 8000, test_pattern: false, bayer: O...` — The flight setting, FHD@25 at 8 Mbit/s: what boot reserves the encoder for, and `video start`'s defaults (START_PARAMETERS mirrors it).

## `firmware::warm`

The warm start on the board (crates/warmstart, src/glider warmstart.py; doc/specs/avionics-port.md §4.6, M8): the
crumb's NVS namespace over ESP-IDF, the boot's restore, and the checkpoint the supervisor runs.

[`peek`] reads the crumb before the recorder opens, so a reboot in flight opens it with its flight flag up. Then
[`restore`] decides the boot on main (doc/specs/avionics-port.md "Bring-up order"), after the devices came up and
before the bus owners and the control thread start: the parts are still main's, and so is the flight word's
writer, so a passed gate rebases the barometers and commits the flight with nothing running beside it. It applies
the plan in plan B's order (crates/warmstart `ApplyPlan`) with plan B's lines: the gate, `controller :: stage -> X`
(to the console only, as the operator's holds), `controller :: armed`, then `warmstart :: WARM START -> X[, armed]`.
The gate line names the reset as text, as the boot line does, where plan B printed MicroPython's number. Every
outcome but a warm start lowers the flight flag the peek raised: a stage i32 whose blob does not read is a cold
boot with nothing cleared, and a flag kept up there would stay up on every boot after.

[`Keeping`] is plan B's Checkpoint task: crates/warmstart's Keeper over this board's NVS and databoard, polled by
the supervisor, with its checkpoint.csv row and log line on every checkpoint (a stage change, and every
`checkpoint_s` aloft). The crumb is saved only while the board flies (armed and launched), cleared on any edge out
of that, and a refused NVS write is retried every poll until it lands. The recovery identity it freezes at
BOOSTING entry comes from the mission, which the executor owns: it republishes the zone and the launch point after
every line it runs ([`publish_mission`]), and the supervisor reads them only at the freeze. A mission with a zone
and no launch point yet (none set by the operator, none pinned at arm) takes the live GNSS fix there, as plan B's
freeze_launch at BOOSTING entry did; the zone's centre stands in when there is none.

The airspeed seed goes to the flight law when the control thread starts (crate::flight, M13); M10 restores the
pitot tare. Not restored yet, logged once at the restore when a crumb carries it: the ground of a barometer other
than the ICP-10111. A warm-started flight records no video (the camera starts only when armed in SETTING) and never
rejoins the link (it is launched).

The crumb lives in the shared `nvs` partition, namespace `coludo`, which plan B and the boot count share, while
the session-start record stays in `cfg`.

### `pub struct ColudoNvs(EspNvs<NvsDefault>)`

The warm start's NVS namespace as crates/warmstart reads and writes it ([`Nvs`]): raw IDF calls, so each failure
is the `esp_err_t` [`NvsError`] carries (a key never written is ESP_ERR_NVS_NOT_FOUND, a blob longer than the
buffer ESP_ERR_NVS_INVALID_LENGTH), and a blob is written with IDF's own update, which writes the new entry before
it erases the old: esp-idf-svc's set_blob erases first, and a cut there would leave a stage i32 with no blob.

- `fn open(partition: &EspDefaultNvsPartition) -> Result<ColudoNvs, sys::EspError>` — The namespace on `partition`, created when absent (the boot count usually made it already).

### `pub struct Keeping`

The checkpoint as the supervisor runs it (plan B's Checkpoint task): the keeper over this board's NVS, the
quantities it reads, and its stream.

- `fn open(member: CheckpointMember, partition: &EspDefaultNvsPartition, topology: &'static Topology, document: &'static Value, pad_ceiling_m: Option<f32>, seed: Option<Identity>) -> Result<Keeping, String>` — The checkpoint of `member`: its NVS handle opened on `partition`, its quantities found in `topology`, its pad latch under `pad_ceiling_m` metres of elevation (half the stage machine's `launch_alt_m`; None without a stage machine, which latches as plan B), seeded with a warm start's restored identity when this boot is one, and freezing the own grounds of the barometers `document` rebases.
- `fn name(&self) -> &'static str` — The entry's name.
- `fn poll_ms(&self) -> u64` — How often the supervisor runs a [`pass`](Keeping::pass), ms: at most 500, so a stage change is caught promptly.
- `fn pass(&mut self, due_us: u64)` — One poll, its schedule at `due_us` (the supervisor's grid): the keeper's pass on one load of the flight word and the fresh readings, the clock read only when a crumb is tried (seconds since 2000, the crumb's epoch), then, for a checkpoint due, the row and the log line (plan B's `checkpoint <stage> alt=<altitude>`), after the crumb, so a full ring never costs it.

### `pub struct Inspection`

What `inspect` shows of the checkpoint beyond its entry: the crumb NVS holds now, how this boot's restore ended,
the NVS action owed, and the tallies since boot.

### `pub struct IdentityView`

The recovery identity as `inspect checkpoint` shows it: what a reset in flight would recover to.

### `pub fn peek(partition: &EspDefaultNvsPartition) -> bool`

Whether NVS holds a crumb [`warmstart::load`] reads in BOOSTING..LANDING: the recorder opens with its flight flag
up then, so a reboot in flight writes flight blocks from its first record. [`restore`] lowers it again for every
outcome but a warm start.

### `pub fn restore(partition: &EspDefaultNvsPartition, document: &Value, separated: bool, mission: &mut Mission, members: &mut [Vec<Member>; 2], writer: &mut FlightWriter<'static>, flight_flag: bool) -> Option<Identity>`

Decides this boot from the crumb (plan B `restore`) and applies a warm start, on main after the devices came up
and before the bus owners and the control thread start (see the module notes). The gate takes the checkpoint
entry's `warm_start` toggle, the separation switch's counted latch, this boot's reset reason, and the clock in
seconds since 2000; a refusal clears the crumb, so the next boot is cold.

Args:
* `partition` - the shared `nvs` partition, the crumb's.
* `document` - the board config this boot runs: the toggle, and its sensors' drivers for the barometers' rebase.
* `separated` - the separation switch's counted latch at setup; false without a switch.
* `mission` - the mission, its launch point and zone restored from the crumb.
* `members` - the I2C parts that came up, by port: the barometers rebased to the crumb's pad.
* `writer` - the flight word's one writer, before the control thread takes it: the stage and the arming commit
  through it.
* `flight_flag` - whether the recorder opened with its flight flag up ([`peek`]): lowered here unless the boot is
  warm.

Returns: the restored recovery identity, the checkpoint's seed, on a warm start; None on any other boot.

### `pub fn stall_restart_suppressed() -> bool`

Whether this boot is a stall restart's own warm start, warm after a software reset: the supervisor then restarts
for no stall, so a stall that comes back every boot leaves the board stalled rather than cycling it
(crates/warmstart `Restore::stall_restart_suppressed`).

### `pub fn publish_mission(mission: &Mission)`

Publishes `mission`'s zone and launch point for the checkpoint's freeze: the operator's point or the one `arm`
pinned, None without one (the freeze then takes the live fix).

### `pub fn inspect(partition: &EspDefaultNvsPartition) -> Inspection`

The checkpoint's [`Inspection`], its crumb read now from `partition` (on the executor, beside the supervisor's
handle: NVS serialises its own calls).
