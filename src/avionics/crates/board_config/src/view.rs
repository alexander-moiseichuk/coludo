//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The typed, read-only view of a config document: only what the firmware consumes so far: the board id and
//! MCU, the Wi-Fi networks and the CC hub, the field's range limit (M2), and the camera. Each reader follows the plan B
//! code that reads the same keys, and is meant for a document [`crate::validate`] accepts (what load()
//! returns); for anything else it answers the nearest safe value, as each one says.
//!
//! For the device bring-up (M5): the I2C and SPI buses ([`i2c_bus`], [`spi_bus`]), plan B's device directory
//! ([`directory`]), and per entry its pins ([`pin`]) and its databoard sources ([`provides`]), plus the setup
//! attempts ([`setup_retries`]). They are meant for the post-layout document (crates/layout rewrites it in place
//! first), as plan B's Controller reads the config layout.resolve has rewritten.
//!
//! Where plan B would carry a value of the wrong type into the radio or the socket and fail there (a text
//! `retry_ms`, a numeric password, a host that is not text), the view reads it as the default or as off; each
//! such reading is named on its function. Every reader borrows the document and changes nothing.

use std::fmt;

use serde_json::{Map, Value};

use crate::python::{equals_integer, formatted, integer, truthy};

/// How long a network waits between two attempts to join it when the config does not say, in milliseconds
/// (drivers/wifi.py `wifi.get('retry_ms', 10000)`).
const DEFAULT_RETRY_MS: u32 = 10_000;

/// The CC hub's TCP port when the config does not say (cc_client.Client `wifi.get('cc_port', 1234)`).
const DEFAULT_HUB_PORT: u16 = 1234;

/// What Debug prints for a network's password when the config gives one.
const REDACTED: &str = "<redacted>";

/// What Debug prints for a network's password when the config gives none.
const NO_PASSWORD: &str = "<empty>";

/// The board's Wi-Fi as plan B's wifi driver and CC link set it up.
#[derive(Clone, Debug, PartialEq)]
pub struct Wifi {
    /// Whether the radio is used at all: false when `wifi.policy` is 'disabled', which leaves the radio
    /// untouched for the session (drivers/wifi.py run()).
    pub policy_enabled: bool,
    /// The networks to join, in round-robin order (the first is tried first). Listed even when
    /// `policy_enabled` is false, as plan B's setup() lists them.
    pub networks: Vec<Network>,
    /// Where the CC link dials.
    pub hub: Hub,
    /// The hub's TCP port (`wifi.cc_port`).
    pub hub_port: u16,
}

/// One network the board may join.
///
/// Its Debug never prints the password, only whether there is one: a Wi-Fi view ends up in log lines, panic
/// and assert messages, which reach the console, the recorder and CC's log stream.
#[derive(Clone, PartialEq)]
pub struct Network {
    /// The SSID, never empty.
    pub ssid: String,
    /// The least time between two attempts on THIS network, in milliseconds (its own backoff clock).
    pub retry_ms: u32,
    /// The config's password for it: the entry's, else the section's, else empty. It is the FALLBACK: plan
    /// B prefers a non-empty first line of `<ssid>.creds`, the firmware the credential store's entry.
    pub password: String,
    /// The transmit power cap for this network in dBm: the entry's `tx_power_dbm`, else the section's; None
    /// leaves the radio's own (drivers/wifi.py applies it as `txpower` when it joins the network). A value that
    /// is not a number reads as None.
    pub tx_power_dbm: Option<f32>,
}

impl fmt::Debug for Network {
    /// The network with its password redacted (see [`Network`]).
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let password = if self.password.is_empty() {
            NO_PASSWORD
        } else {
            REDACTED
        };
        formatter
            .debug_struct("Network")
            .field("ssid", &self.ssid)
            .field("retry_ms", &self.retry_ms)
            .field("password", &format_args!("{password}"))
            .field("tx_power_dbm", &self.tx_power_dbm)
            .finish()
    }
}

/// Where the CC link dials (tasks/cc_link.py and cc_client.Client).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Hub {
    /// No `wifi.cc_host` (absent or null): the `.1` of whatever subnet the board joins, the Control hub by
    /// convention.
    Gateway,
    /// `wifi.cc_host` is '': CC is off, and the board flies standalone.
    Off,
    /// This host, as configured.
    Host(String),
}

/// The board id (`board.id`), as the hub and CC address the board. Empty when the document has none, where
/// plan B would raise (never for a document validate() accepts).
pub fn board_id(document: &Value) -> String {
    board_text(document, "id")
}

/// The MCU (`board.mcu`). Empty when the document has none (never for a document validate() accepts).
pub fn mcu(document: &Value) -> String {
    board_text(document, "mcu")
}

/// A text member of the `board` section, or empty.
fn board_text(document: &Value, key: &str) -> String {
    document["board"][key].as_str().unwrap_or_default().to_string()
}

/// The Wi-Fi view, following plan B's drivers/wifi.py setup() for the networks and the policy, and
/// tasks/cc_link.py with cc_client.Client for the hub.
///
/// Networks: `wifi.networks` when it is a non-empty list, else `[wifi.ssid]` when that is non-empty, else
/// none. A bare string entry is `{ssid}`. Each entry starts from the section's defaults (policy 'auto',
/// enabled, the section's `retry_ms` or 10000 ms, the section's `password` or empty) and its own
/// keys override them. Note: the entry defaults for `policy` and `enabled` are fixed, NOT the section's: plan
/// B's setup() never reads a top-level `enabled`, and the top-level `policy` only decides `policy_enabled`.
/// Dropped: an entry whose ssid is empty (or not truthy), whose `enabled` is not truthy, or whose policy is
/// 'disabled'.
///
/// Readings where plan B could not join or dial at all (validate() lets these through):
///  * an entry that is neither text nor an object, or a non-list `networks`, is skipped (validate refuses
///    them); an ssid that is not text drops the entry (plan B's connect() raises on it);
///  * `retry_ms`: a number is the least whole number of milliseconds it allows (Python compares the integer
///    tick difference with it), held within 0..=u32::MAX; a bool is 0 or 1; anything else is
///    10000 ms, the default (plan B's comparison raises TypeError at the second attempt);
///  * a password that is not text is empty (MicroPython's connect() takes None as no password, and raises on
///    other types);
///  * `cc_host`: a falsy non-text value (false, 0, [], {}) dials the gateway, as plan B's `host or ...` does;
///    a truthy one is [`Hub::Off`] (plan B dials it and never connects);
///  * `cc_port`: an integer in 0..=65535, or a float equal to one (the typed view's rule for integer knobs);
///    anything else is 1234, the default (plan B never connects with it);
///  * a `wifi` section that is present but not an object (null, which validate accepts, or another kind):
///    plan B's wifi and CC setups both raise on it, so neither runs: the radio is untouched (`policy_enabled`
///    false, no networks) and [`Hub::Off`].
pub fn wifi(document: &Value) -> Wifi {
    let empty = Map::new();
    let section = match document.get("wifi") {
        None => &empty,
        Some(Value::Object(section)) => section,
        Some(_) => {
            return Wifi {
                policy_enabled: false,
                networks: Vec::new(),
                hub: Hub::Off,
                hub_port: DEFAULT_HUB_PORT,
            };
        }
    };
    Wifi {
        policy_enabled: section.get("policy").and_then(Value::as_str) != Some("disabled"),
        networks: networks(section),
        hub: hub(section.get("cc_host")),
        hub_port: hub_port(section.get("cc_port")),
    }
}

/// The networks plan B's setup() keeps, in order (see [`wifi`]).
fn networks(section: &Map<String, Value>) -> Vec<Network> {
    let default_retry = section.get("retry_ms");
    let default_password = section.get("password");
    let default_power = section.get("tx_power_dbm");
    let fallback_ssid: Vec<Value> = section
        .get("ssid")
        .filter(|ssid| truthy(ssid))
        .map(|ssid| vec![ssid.clone()])
        .unwrap_or_default();
    let entries = match section.get("networks") {
        Some(Value::Array(entries)) if !entries.is_empty() => entries,
        Some(networks) if truthy(networks) => return Vec::new(),
        _ => &fallback_ssid,
    };
    entries
        .iter()
        .filter_map(|entry| {
            let entry: Map<String, Value> = match entry {
                Value::String(_) => Map::from_iter([("ssid".to_string(), entry.clone())]),
                Value::Object(entry) => entry.clone(),
                _ => return None,
            };
            let ssid = entry
                .get("ssid")
                .and_then(Value::as_str)
                .filter(|ssid| !ssid.is_empty())?;
            let enabled = entry.get("enabled").is_none_or(truthy);
            let disabled = entry.get("policy").and_then(Value::as_str) == Some("disabled");
            (enabled && !disabled).then(|| Network {
                ssid: ssid.to_string(),
                retry_ms: retry_ms(entry.get("retry_ms").or(default_retry)),
                password: entry
                    .get("password")
                    .or(default_password)
                    .and_then(Value::as_str)
                    .unwrap_or_default()
                    .to_string(),
                // a present entry key wins even when null, as plan B's dict.update does
                tx_power_dbm: match entry.get("tx_power_dbm") {
                    Some(value) => power(value),
                    None => default_power.and_then(power),
                },
            })
        })
        .collect()
}

/// A transmit power in dBm, None for anything but a number (a JSON bool is not one here).
fn power(value: &Value) -> Option<f32> {
    // f64 at the JSON boundary, f32 for the radio
    value.as_f64().map(|dbm| dbm as f32)
}

/// A network's `retry_ms` (see [`wifi`]): absent is [`DEFAULT_RETRY_MS`].
fn retry_ms(value: Option<&Value>) -> u32 {
    match value {
        Some(Value::Bool(flag)) => u32::from(*flag),
        // f64 at the JSON boundary; `as` saturates into 0..=u32::MAX
        Some(Value::Number(number)) => number
            .as_f64()
            .map_or(DEFAULT_RETRY_MS, |milliseconds| milliseconds.ceil() as u32),
        _ => DEFAULT_RETRY_MS,
    }
}

/// The hub from `wifi.cc_host` (see [`wifi`]).
fn hub(host: Option<&Value>) -> Hub {
    match host {
        None | Some(Value::Null) => Hub::Gateway,
        Some(Value::String(host)) if host.is_empty() => Hub::Off,
        Some(Value::String(host)) => Hub::Host(host.clone()),
        Some(other) if !truthy(other) => Hub::Gateway,
        Some(_) => Hub::Off,
    }
}

/// The hub port from `wifi.cc_port` (see [`wifi`]).
fn hub_port(port: Option<&Value>) -> u16 {
    let whole =
        |number: f64| (number.fract() == 0.0 && (0.0..=f64::from(u16::MAX)).contains(&number)).then_some(number as u16);
    match port {
        Some(Value::Number(number)) => number.as_f64().and_then(whole).unwrap_or(DEFAULT_HUB_PORT),
        _ => DEFAULT_HUB_PORT,
    }
}

/// The airframe's glide range, in metres: `max_range_m` on the first sensor or component named 'field'
/// (main.py `config.device(cfg, name='field')`, which the Mission takes as its range limit).
///
/// None when the config does not say (no field device, or no `max_range_m`): the caller then uses the
/// Mission's default, as plan B does (`field_cfg.get('max_range_m', 200)`). Also None for a value that is not
/// a number (null, text, a bool), which plan B would carry into the Mission's range comparisons and fail on.
pub fn max_range_m(document: &Value) -> Option<f32> {
    let (_, field) =
        devices(document).find(|(_, device)| device.get("name").and_then(Value::as_str) == Some("field"))?;
    // f64 at the JSON boundary, metres in f32 from here on
    field
        .get("max_range_m")
        .and_then(Value::as_f64)
        .map(|metres| metres as f32)
}

/// The OV5647 camera's implementation name in a device entry (`driver`, or else `activity`).
const CAMERA_DRIVER: &str = "ov5647";

/// The oscillator range the OV5647 accepts on XVCLK, in Hz (datasheet: 6-27 MHz). A value outside it is a
/// typo (27 for 27 MHz) or another part.
const CAMERA_CLOCK_RANGE_HZ: std::ops::RangeInclusive<f64> = 6_000_000.0..=27_000_000.0;

/// The flight camera on the board's CSI connector: an OV5647. Its SCCB is the board's i2c:0, fixed by the
/// wiring, so the entry names no bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Camera {
    /// The module's oscillator (`clock_hz`), which sets the true frame rate: modules differ (24, 25 and 27 MHz
    /// seen).
    pub clock: CameraClock,
}

/// What the camera entry says about its oscillator.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CameraClock {
    /// A whole number of Hz in the OV5647's 6-27 MHz: an integer, or a float equal to one (`27e6`), the typed
    /// view's rule for integer knobs.
    Hertz(u32),
    /// No `clock_hz` (absent or null).
    Missing,
    /// A `clock_hz` that is no such number, as written (a text, a fraction, 27 meaning MHz), for the firmware's
    /// warning to show.
    Unusable(String),
}

/// The board's camera: the first sensor or component whose implementation is 'ov5647', its `driver` or else its
/// `activity` (plan B's `config.device(cfg, driver=...)` order and match), when it is enabled (`enabled` absent
/// reads as enabled).
///
/// None when the config declares no camera, or a disabled one: the board then records no video and leaves the
/// encoder's internal RAM unreserved. Plan B never had a camera of its own (the Luckfox recorded video), so this
/// entry is a schema addition: validate() checks it as any device, and older files, which have none, stay valid.
/// Plan B itself has no 'ov5647' implementation: it counts the entry as a failed device and refuses to arm.
pub fn camera(document: &Value) -> Option<Camera> {
    let (_, device) = devices(document).find(|(_, device)| implementation(device) == CAMERA_DRIVER)?;
    if !enabled(device) {
        return None;
    }
    let whole = |hertz: f64| (hertz.fract() == 0.0 && CAMERA_CLOCK_RANGE_HZ.contains(&hertz)).then_some(hertz as u32);
    let clock = match device.get("clock_hz") {
        None | Some(Value::Null) => CameraClock::Missing,
        Some(value) => match value.as_f64().and_then(whole) {
            Some(hertz) => CameraClock::Hertz(hertz),
            None => CameraClock::Unusable(value.to_string()),
        },
    };
    Some(Camera { clock })
}

/// An I2C bus's clock when its spec names none, in Hz (i2cbus.py:94 `spec.get('freq', 400000)`).
const DEFAULT_I2C_CLOCK_HZ: u32 = 400_000;

/// An SPI bus's clock when its spec names none, in Hz (spibus.py:138 `spec.get('baud', 5_000_000)`).
const DEFAULT_SPI_CLOCK_HZ: u32 = 5_000_000;

/// An SPI bus's mode when its spec names none: 3, CPOL 1 and CPHA 1, the ADXL375's (spibus.py:137
/// `spec.get('mode', 3)`, the default validate() checks against too, config.py:194-197).
const DEFAULT_SPI_MODE: u8 = 3;

/// The setup attempts when the board section does not say (controller.py:368 `get('setup_retries', 1)`).
const DEFAULT_SETUP_ATTEMPTS: i128 = 1;

/// The highest 7-bit I2C address, the only kind the buses speak.
const HIGHEST_ADDRESS: u8 = 0x7f;

/// An I2C bus, as plan B's i2cbus.Bus builds it from its spec (i2cbus.py:91-96).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct I2cSpecification {
    /// The bus id, its key under `buses.i2c`: MicroPython's `I2C(id)`, the ESP-IDF port number.
    pub id: u8,
    /// The data line's GPIO (`sda`).
    pub sda: u8,
    /// The clock line's GPIO (`scl`).
    pub scl: u8,
    /// The bus clock in Hz (`freq`; 400 kHz when absent).
    pub clock_hz: u32,
}

/// An SPI bus, as plan B's spibus.Bus builds it from its spec (spibus.py:131-147).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SpiSpecification {
    /// The bus id, its key under `buses.spi`: MicroPython's `SPI(id)`, where id 1 is the ESP32-P4's SPI2.
    pub id: u8,
    /// The clock line's GPIO (`sck`).
    pub sck: u8,
    /// The controller-out line's GPIO (`mosi`).
    pub mosi: u8,
    /// The controller-in line's GPIO (`miso`).
    pub miso: u8,
    /// The bus clock in Hz (`baud`; 5 MHz when absent).
    pub clock_hz: u32,
    /// The SPI mode, 0..=3: the clock polarity is its high bit and the phase its low bit, as plan B splits it
    /// (`mode`; 3 when absent).
    pub mode: u8,
}

/// The I2C bus `id`, as plan B builds it: `config.bus(cfg, 'i2c', id)` (config.py:681-696) handed to i2cbus.Bus,
/// which constructs `I2C(id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=spec.get('freq', 400000))`
/// (i2cbus.py:91-96).
///
/// None when the config defines no such bus: `buses.i2c.<id>` absent, null or not an object, which plan B's bind()
/// reports as no bus, so its devices are not connected. Also None for a spec that lacks `scl` or `sda` (plan B's
/// constructor raises KeyError, losing the bus and every device on it; validate() refuses it) or that names no GPIO
/// for one of them: a pin reads as [`pin`] reads a pins-map value, where plan B's `Pin()` raises.
///
/// The clock is `freq` when it is a whole number of Hz in 1..=u32::MAX: an int, or a float equal to one (the typed
/// view's rule for integer knobs). Absent or anything else, 400 kHz: validate() checks no clock, and plan B hands any
/// other value to MicroPython's constructor, which refuses null, text and a float (losing the bus) and has no
/// meaningful clock for a bool, zero or a negative.
pub fn i2c_bus(document: &Value, id: u8) -> Option<I2cSpecification> {
    let specification = bus(document, "i2c", id)?;
    Some(I2cSpecification {
        id,
        sda: gpio(specification.get("sda")?)?,
        scl: gpio(specification.get("scl")?)?,
        clock_hz: clock_hz(specification.get("freq"), DEFAULT_I2C_CLOCK_HZ),
    })
}

/// The SPI bus `id`, as plan B builds it: `config.bus(cfg, 'spi', id)` (config.py:681-696) handed to spibus.Bus,
/// which constructs `SPI(id, baudrate=spec.get('baud', 5_000_000), polarity=mode >> 1, phase=mode & 1, sck=...,
/// mosi=..., miso=...)` with `mode = spec.get('mode', 3)` (spibus.py:134-139).
///
/// None as for [`i2c_bus`]: no such bus, or a spec that lacks `sck`, `mosi` or `miso` or names no GPIO for one.
/// The clock reads as [`i2c_bus`] reads it, with 5 MHz for the default.
///
/// The mode is the one of 0..=3 that `mode` equals under Python's `==`, which are exactly the values validate()
/// admits (config.py:194-197): an int, a bool (True is mode 1, as plan B's shifts read it) or a whole float (plan B's
/// `>>` raises on a float, losing the bus; the view reads the mode it names). Absent or anything else, 3.
pub fn spi_bus(document: &Value, id: u8) -> Option<SpiSpecification> {
    let specification = bus(document, "spi", id)?;
    let mode = specification
        .get("mode")
        .and_then(|mode| (0..=3).find(|allowed| equals_integer(mode, i64::from(*allowed))))
        .unwrap_or(DEFAULT_SPI_MODE);
    Some(SpiSpecification {
        id,
        sck: gpio(specification.get("sck")?)?,
        mosi: gpio(specification.get("mosi")?)?,
        miso: gpio(specification.get("miso")?)?,
        clock_hz: clock_hz(specification.get("baud"), DEFAULT_SPI_CLOCK_HZ),
        mode,
    })
}

/// Plan B's `config.bus` (config.py:681-696): the spec under `buses.<kind>.<id>`, looked up by the id's text
/// (`str(ident)`), since the keys of a JSON object are text. None when a level is missing, or is not an object
/// (plan B raises on a level that is present but no dict; validate() refuses one).
fn bus<'d>(document: &'d Value, kind: &str, id: u8) -> Option<&'d Map<String, Value>> {
    document.get("buses")?.get(kind)?.get(id.to_string())?.as_object()
}

/// A bus clock (`freq`, `baud`) in Hz (see [`i2c_bus`]): `default` unless it is a whole number in 1..=u32::MAX.
fn clock_hz(value: Option<&Value>, default: u32) -> u32 {
    let whole =
        |hertz: f64| (hertz.fract() == 0.0 && (1.0..=f64::from(u32::MAX)).contains(&hertz)).then_some(hertz as u32);
    match value {
        // f64 at the JSON boundary: exact for every integer the range admits
        Some(Value::Number(number)) => number.as_f64().and_then(whole).unwrap_or(default),
        _ => default,
    }
}

/// A GPIO number as task.py:166 `_pin_gpio` admits one: `isinstance(gpio, int) and gpio >= 0`, Python's int, so a
/// bool is GPIO 0 or 1 (validate() refuses one) and a float, even a whole one, is none. A number past 255 is none
/// too: no MCU the config names has that many GPIOs (plan B's `Pin()` raises on it).
fn gpio(value: &Value) -> Option<u8> {
    integer(value).and_then(|number| u8::try_from(number).ok())
}

/// Which device list of the config an entry sits in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Section {
    /// `sensors`: the data providers.
    Sensors,
    /// `components`: the consumers and actuators.
    Components,
}

/// One device entry of the config, as plan B's Controller reads it to bring the device up.
///
/// The bus fields are the entry's own, with none of the defaults a driver applies: plan B's i2cbus.bind reads an
/// absent `id` as 0 and an absent `addr` as the driver's datasheet address (i2cbus.py:297-301), spibus.bind an
/// absent `id` as 1 (spibus.py:217), and each an absent `bus` as its own kind. An explicit null is not absent there
/// (bind() then finds no bus, or hands the driver no address and its setup fails), nor is an `addr` this view reads
/// as None: [`Entry::fields`] tells them apart.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry<'d> {
    /// The device's name: non-empty text, unique across both lists (validate() refuses a duplicate).
    pub name: &'d str,
    /// The list it sits in.
    pub section: Section,
    /// What runs it: the `driver` when it is truthy, else the `activity` (controller.py:280 `comp.get('driver') or
    /// comp.get('activity')`). Empty when that is not text, which names no implementation (validate() refuses it;
    /// plan B fails it as `no driver/activity`).
    pub implementation: &'d str,
    /// The bus kind (`bus`: 'i2c', 'spi' or 'uart'). None when absent, null or not text.
    pub bus: Option<&'d str>,
    /// The bus id as text, as plan B compares and looks it up (`str(comp.get('id'))`, controller.py:149 and
    /// config.py:696): an int 1 and a text '1' are one id, and a float 1.0 is '1.0', which names no bus. None when
    /// `id` is absent or null.
    pub bus_id: Option<String>,
    /// The I2C address (`addr`): an int in 0..=0x7f, Python's int (a bool is 0 or 1, which validate() refuses).
    /// None when absent or null, and also for any other value: validate() admits any int, and plan B's driver fails
    /// at its first transfer on one past 0x7f. An SPI entry may carry one too, its fallback for an I2C wiring.
    pub address: Option<u8>,
    /// The whole entry, for the keys a driver reads itself.
    pub fields: &'d Map<String, Value>,
}

/// Plan B's `Controller.directory()` (controller.py:96-103): every enabled, named device entry, `sensors` first and
/// then `components`, each in its config order, the order plan B creates and sets the devices up in.
///
/// Enabled as plan B reads `device.get('enabled', True)`: absent, or truthy. Named: a non-empty text `name`; plan B
/// would list a truthy name that is not text too, which validate() refuses. Skipped where plan B raises: an item that
/// is not an object, and a `sensors` or `components` that is not a list.
///
/// A part layout disabled is not here, as in plan B: crates/layout writes `enabled` false into the document this
/// reads (layout.py:430-453 `_fit`, for the parts layout.py:291-321 `fitted` rules out).
pub fn directory(document: &Value) -> Vec<Entry<'_>> {
    devices(document)
        .filter(|(_, device)| enabled(device))
        .filter_map(|(section, device)| {
            let name = device
                .get("name")
                .and_then(Value::as_str)
                .filter(|name| !name.is_empty())?;
            let address = device
                .get("addr")
                .and_then(integer)
                .and_then(|number| u8::try_from(number).ok())
                .filter(|address| *address <= HIGHEST_ADDRESS);
            Some(Entry {
                name,
                section,
                implementation: implementation(device),
                bus: device.get("bus").and_then(Value::as_str),
                bus_id: device.get("id").filter(|id| !id.is_null()).map(formatted),
                address,
                fields: device,
            })
        })
        .collect()
}

/// Plan B's `Task._pin_gpio(field)` (task.py:144-166): the GPIO an entry's `key` names through the board's `pins`
/// map. The entry gives a pin NAME (`"cs_pin": "lsm6dso32_cs"`), and the map its number.
///
/// None, exactly as plan B, when the feature is absent or wired off: the entry has no `key`, the map has no such
/// name, or the map gives null or a negative number. The number is Python's int, as plan B tests it
/// (`isinstance(gpio, int) and gpio >= 0`): a bool is GPIO 0 or 1 and a float none, both refused by validate().
/// Also None where plan B would fail: a `pins` that is not an object, a `key` whose value is not text (the map's
/// keys are text, so plan B finds nothing for a number and raises on a list) and a GPIO past 255, which no MCU the
/// config names has.
///
/// Plan B's default pin name for an entry that omits `key` (`_pin_gpio('pin', 'separation_switch')`) is not here: no
/// M5 part uses one.
pub fn pin(document: &Value, entry: &Entry, key: &str) -> Option<u8> {
    let name = entry.fields.get(key)?.as_str()?;
    gpio(document.get("pins")?.as_object()?.get(name)?)
}

/// One quantity a device feeds the databoard: an entry of its `provides`, as plan B's `Databoard.provide` hands it
/// to `Parameter.add_source` (databoard.py:457-483, 227).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Provide<'d> {
    /// The quantity (`altitude`, `accel`, ...): the key in `provides`.
    pub quantity: &'d str,
    /// The source's rank, the lowest one the primary (`priority`; 0 when absent).
    pub priority: i32,
    /// The source's freshness timeout in milliseconds (`timeout_ms`; 0 when absent, which the databoard reads as its
    /// default second).
    pub timeout_ms: u32,
    /// Whether this provider asks for the quantity's offset reconciliation (`reconcile`, by truthiness; false when
    /// absent).
    pub reconcile: bool,
}

/// The quantities `entry` provides, sorted by quantity: plan B's `provide(name, self.config.get('provides', {}))`
/// (databoard.py:457-483), each one `add_source(source, spec.get('priority', 0), spec.get('timeout_ms', 0) * 1000,
/// spec.get('reconcile', False))`. Sorted as controller.py:151-162 `_quantities` lists them; plan B registered them
/// in the document's order, which orders only one source's own quantities, never two sources of one quantity.
///
/// Empty when the entry provides nothing: no `provides`, or one that is null or not an object (validate() admits
/// null, and plan B's drivers raise on it in setup). A quantity whose spec is not an object is left out (plan B
/// raises on it; validate() refuses it).
///
/// `priority` and `timeout_ms` are Python's int (a bool is 0 or 1); anything else reads as 0, the absent reading
/// (validate() admits only an int). A priority past i32 saturates, so two such ranks tie (the configs rank 0..=3).
/// A timeout past u32::MAX saturates.
///
/// A negative timeout reads as 0, the default second (validate() admits it). Plan B keeps it (`expire_us or
/// _DEFAULT_EXPIRE_US`, databoard.py:254), and on a primary-tier source it becomes the quantity's SHARED window
/// (databoard.py:255-259), under which `fresh()`'s `0 <= elapsed <= window_us` (databoard.py:117) never holds: every
/// source of the quantity reads stale, not just this one. On a source below the primary tier plan B never consults
/// it either.
pub fn provides<'d>(entry: &Entry<'d>) -> Vec<Provide<'d>> {
    let fields: &'d Map<String, Value> = entry.fields;
    let Some(provides) = fields.get("provides").and_then(Value::as_object) else {
        return Vec::new();
    };
    let mut quantities: Vec<Provide<'d>> = provides
        .iter()
        .filter_map(|(quantity, specification)| {
            let specification = specification.as_object()?;
            let number = |key: &str| specification.get(key).and_then(integer).unwrap_or(0);
            Some(Provide {
                quantity: quantity.as_str(),
                // held within the target type first, so the casts are exact
                priority: number("priority").clamp(i128::from(i32::MIN), i128::from(i32::MAX)) as i32,
                timeout_ms: number("timeout_ms").clamp(0, i128::from(u32::MAX)) as u32,
                reconcile: specification.get("reconcile").is_some_and(truthy),
            })
        })
        .collect();
    // by code point, as Python's sorted() orders str
    quantities.sort_by(|left, right| left.quantity.cmp(right.quantity));
    quantities
}

/// How many times the bring-up tries a device's setup before it calls the device failed: plan B's `max(1,
/// board.get('setup_retries', 1))` (controller.py:368). Despite the key's name, a count of ATTEMPTS: 1 is one try and
/// no retry.
///
/// Python's int (a bool is 0 or 1, which `max` lifts to 1); 1 when absent, and also for anything else, and for a
/// `board` that is not an object: plan B's bring-up raises on text (`max(1, '3')`), null, and a float
/// (`range(1, 3.0 + 1)`). A count past u32::MAX saturates.
pub fn setup_retries(document: &Value) -> u32 {
    let attempts = document
        .get("board")
        .and_then(|board| board.get("setup_retries"))
        .and_then(integer)
        .unwrap_or(DEFAULT_SETUP_ATTEMPTS);
    // held within u32 first, so the cast is exact
    attempts.clamp(1, i128::from(u32::MAX)) as u32
}

/// Every device entry that is an object, with its list: plan B's `config.get('sensors', []) +
/// config.get('components', [])` (controller.py:96-98), in that order. A `sensors` or `components` that is not a
/// list holds none here, and an item that is not an object is skipped (plan B raises on either).
fn devices(document: &Value) -> impl Iterator<Item = (Section, &Map<String, Value>)> {
    [(Section::Sensors, "sensors"), (Section::Components, "components")]
        .into_iter()
        .filter_map(|(section, key)| Some((section, document.get(key)?.as_array()?)))
        .flat_map(|(section, items)| items.iter().filter_map(move |item| Some((section, item.as_object()?))))
}

/// Whether an entry is enabled, as plan B's `device.get('enabled', True)` reads it: absent, or truthy.
fn enabled(device: &Map<String, Value>) -> bool {
    device.get("enabled").is_none_or(truthy)
}

/// What runs an entry: `driver or activity`, the driver when it is truthy, else the activity (validate's `runs`);
/// empty when that is not text.
fn implementation(device: &Map<String, Value>) -> &str {
    device
        .get("driver")
        .filter(|driver| truthy(driver))
        .or(device.get("activity"))
        .and_then(Value::as_str)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::{
        Camera, CameraClock, DEFAULT_HUB_PORT, DEFAULT_RETRY_MS, Hub, Network, board_id, camera, max_range_m, mcu, wifi,
    };
    use serde_json::{Value, json};

    /// A network as the view lists it.
    fn network(ssid: &str, retry_ms: u32, password: &str) -> Network {
        Network {
            ssid: ssid.to_string(),
            retry_ms,
            password: password.to_string(),
            tx_power_dbm: None,
        }
    }

    #[test]
    fn transmit_power_inherits_and_an_entry_null_wins() {
        let powers = |section: Value| -> Vec<Option<f32>> {
            networks_of(section)
                .iter()
                .map(|network| network.tx_power_dbm)
                .collect()
        };
        assert_eq!(powers(json!({"tx_power_dbm": 11, "networks": ["a"]})), [Some(11.0)]);
        assert_eq!(
            powers(json!({"tx_power_dbm": 11, "networks": [{"ssid": "a", "tx_power_dbm": 8.5}]})),
            [Some(8.5)]
        );
        assert_eq!(
            powers(json!({"tx_power_dbm": 11, "networks": [{"ssid": "a", "tx_power_dbm": null}]})),
            [None]
        );
        assert_eq!(powers(json!({"networks": ["a"]})), [None]);
        assert_eq!(powers(json!({"tx_power_dbm": "high", "networks": ["a"]})), [None]);
        assert_eq!(powers(json!({"tx_power_dbm": true, "networks": ["a"]})), [None]);
    }

    /// The networks of a document holding just this wifi section.
    fn networks_of(section: Value) -> Vec<Network> {
        wifi(&json!({"wifi": section})).networks
    }

    #[test]
    fn the_board_id_and_mcu_come_from_the_board_section() {
        let document = json!({"board": {"id": "TMS-7F", "mcu": "esp32p4"}});
        assert_eq!(board_id(&document), "TMS-7F");
        assert_eq!(mcu(&document), "esp32p4");
        assert_eq!(board_id(&json!({})), "");
        assert_eq!(mcu(&json!({"board": {"mcu": 5}})), "");
        assert_eq!(board_id(&json!([])), "");
    }

    #[test]
    fn no_wifi_section_dials_the_gateway_with_no_networks() {
        let view = wifi(&json!({}));
        assert!(view.policy_enabled);
        assert_eq!(view.networks, []);
        assert_eq!(view.hub, Hub::Gateway);
        assert_eq!(view.hub_port, DEFAULT_HUB_PORT);
    }

    #[test]
    fn a_wifi_section_that_is_not_an_object_leaves_the_radio_alone() {
        for section in [json!(null), json!([]), json!("panda")] {
            let view = wifi(&json!({"wifi": section}));
            assert!(!view.policy_enabled, "{section}");
            assert_eq!(view.networks, [], "{section}");
            assert_eq!(view.hub, Hub::Off, "{section}");
        }
    }

    #[test]
    fn the_ssid_is_the_fallback_for_no_networks() {
        let expected = [network("field", DEFAULT_RETRY_MS, "")];
        assert_eq!(networks_of(json!({"ssid": "field"})), expected);
        assert_eq!(networks_of(json!({"ssid": "field", "networks": []})), expected);
        assert_eq!(networks_of(json!({"ssid": "field", "networks": null})), expected);
        assert_eq!(networks_of(json!({"ssid": ""})), []);
    }

    #[test]
    fn entries_inherit_the_section_and_override_it() {
        let section = json!({"retry_ms": 2500, "password": "made-up-shared", "networks": [
            "lab", {"ssid": "field", "retry_ms": 500, "password": "made-up-field"}, {"ssid": "car"}]});
        assert_eq!(
            networks_of(section),
            [
                network("lab", 2500, "made-up-shared"),
                network("field", 500, "made-up-field"),
                network("car", 2500, "made-up-shared")
            ]
        );
    }

    #[test]
    fn parked_and_unnamed_entries_are_dropped() {
        let section = json!({"networks": ["", {"ssid": "off", "enabled": false}, {"ssid": "zero", "enabled": 0},
            {"ssid": "null", "enabled": null}, {"ssid": "parked", "policy": "disabled"}, {"ssid": ""},
            {"ssid": 5}, 7, null, {"ssid": "kept", "enabled": "yes", "policy": "auto"}]});
        assert_eq!(networks_of(section), [network("kept", DEFAULT_RETRY_MS, "")]);
    }

    #[test]
    fn the_top_level_policy_and_enabled_do_not_reach_the_entries() {
        let view = wifi(&json!({"wifi": {"policy": "disabled", "enabled": false, "networks": ["lab"]}}));
        assert!(!view.policy_enabled);
        assert_eq!(view.networks, [network("lab", DEFAULT_RETRY_MS, "")]);
    }

    #[test]
    fn retry_ms_reads_as_plan_b_compares_it() {
        let retry = |value: Value| networks_of(json!({"networks": [{"ssid": "lab", "retry_ms": value}]}))[0].retry_ms;
        assert_eq!(retry(json!(1500)), 1500);
        assert_eq!(retry(json!(1500.0)), 1500);
        assert_eq!(retry(json!(1500.25)), 1501, "a tick difference passes 1500.25 at 1501");
        assert_eq!(retry(json!(-5)), 0);
        assert_eq!(retry(json!(1e12)), u32::MAX);
        assert_eq!(retry(json!(true)), 1);
        assert_eq!(retry(json!("1500")), DEFAULT_RETRY_MS);
        assert_eq!(retry(json!(null)), DEFAULT_RETRY_MS);
    }

    #[test]
    fn a_password_that_is_not_text_is_empty() {
        let section = json!({"password": 1234, "networks": [{"ssid": "a"}, {"ssid": "b", "password": null}]});
        assert_eq!(
            networks_of(section),
            [network("a", DEFAULT_RETRY_MS, ""), network("b", DEFAULT_RETRY_MS, "")]
        );
    }

    #[test]
    fn debug_never_prints_a_password() {
        let view = wifi(&json!({"wifi": {"networks": [{"ssid": "lab", "password": "made-up-secret"}, "open"]}}));
        let printed = format!("{view:?}");
        assert!(!printed.contains("made-up-secret"), "{printed}");
        assert!(printed.contains("password: <redacted>"), "{printed}");
        // ...and says when there is none, which is what a failed join needs to know
        assert!(printed.contains("password: <empty>"), "{printed}");
        assert!(printed.contains("\"lab\"") && printed.contains("\"open\""), "{printed}");
        let pretty = format!("{:#?}", view.networks[0]);
        assert!(!pretty.contains("made-up-secret"), "{pretty}");
        // the comparison still sees the password
        assert_ne!(
            view.networks[0],
            network("lab", DEFAULT_RETRY_MS, "another-made-up-secret")
        );
    }

    #[test]
    fn the_hub_follows_cc_host() {
        let hub = |host: Value| wifi(&json!({"wifi": {"cc_host": host}})).hub;
        assert_eq!(hub(json!(null)), Hub::Gateway);
        assert_eq!(hub(json!("")), Hub::Off);
        assert_eq!(hub(json!("192.168.4.1")), Hub::Host("192.168.4.1".to_string()));
        assert_eq!(hub(json!("hub.local")), Hub::Host("hub.local".to_string()));
        assert_eq!(hub(json!(false)), Hub::Gateway);
        assert_eq!(hub(json!(0)), Hub::Gateway);
        assert_eq!(hub(json!(5)), Hub::Off);
        assert_eq!(hub(json!(["x"])), Hub::Off);
    }

    #[test]
    fn the_hub_port_takes_whole_numbers_in_range() {
        let port = |value: Value| wifi(&json!({"wifi": {"cc_port": value}})).hub_port;
        assert_eq!(port(json!(4321)), 4321);
        assert_eq!(port(json!(4321.0)), 4321);
        assert_eq!(port(json!(0)), 0);
        assert_eq!(port(json!(65535)), 65535);
        for refused in [
            json!(65536),
            json!(-1),
            json!(4321.5),
            json!("4321"),
            json!(null),
            json!(true),
        ] {
            assert_eq!(port(refused.clone()), DEFAULT_HUB_PORT, "{refused}");
        }
    }

    #[test]
    fn the_range_limit_is_the_field_devices() {
        let components = |field: Value| json!({"components": [{"name": "flight"}, field]});
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": 200}))),
            Some(200.0)
        );
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": 350.5}))),
            Some(350.5)
        );
        assert_eq!(max_range_m(&components(json!({"name": "field"}))), None);
        assert_eq!(
            max_range_m(&components(json!({"name": "other", "max_range_m": 50}))),
            None
        );
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": "200"}))),
            None
        );
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": true}))),
            None
        );
        assert_eq!(max_range_m(&json!({})), None);
        // sensors are searched first, and the first match wins
        let both = json!({"sensors": [{"name": "field", "max_range_m": 80}],
                          "components": [{"name": "field", "max_range_m": 200}]});
        assert_eq!(max_range_m(&both), Some(80.0));
    }

    /// A camera with this oscillator.
    fn clocked(hertz: u32) -> Option<Camera> {
        Some(Camera {
            clock: CameraClock::Hertz(hertz),
        })
    }

    /// A camera whose entry gives no usable oscillator.
    fn unclocked(clock: CameraClock) -> Option<Camera> {
        Some(Camera { clock })
    }

    /// The clock reading of a camera entry with this `clock_hz`.
    fn with_clock(clock: Value) -> Option<Camera> {
        camera(&json!({"components": [{"name": "camera", "driver": "ov5647", "clock_hz": clock}]}))
    }

    /// An unusable clock, as written.
    fn unusable(written: &str) -> Option<Camera> {
        unclocked(CameraClock::Unusable(written.to_string()))
    }

    #[test]
    fn the_camera_is_the_ov5647_device_with_its_clock() {
        let components = |camera: Value| json!({"components": [{"name": "flight", "activity": "flight"}, camera]});
        assert_eq!(with_clock(json!(27_000_000)), clocked(27_000_000));
        assert_eq!(with_clock(json!(24_000_000)), clocked(24_000_000));
        // found by its implementation, whatever its name: the driver, or else the activity (plan B's device())
        assert_eq!(
            camera(&components(
                json!({"name": "nose", "driver": "ov5647", "clock_hz": 25_000_000})
            )),
            clocked(25_000_000)
        );
        assert_eq!(
            camera(&components(
                json!({"name": "nose", "activity": "ov5647", "clock_hz": 25_000_000})
            )),
            clocked(25_000_000)
        );
        assert_eq!(
            camera(&components(
                json!({"name": "nose", "driver": "", "activity": "ov5647", "clock_hz": 25_000_000})
            )),
            clocked(25_000_000)
        );
        // a truthy driver wins over the activity
        assert_eq!(
            camera(&components(
                json!({"name": "nose", "driver": "sg90", "activity": "ov5647"})
            )),
            None
        );
        // no camera: none declared, another driver, an empty config
        assert_eq!(camera(&components(json!({"name": "camera", "driver": "sg90"}))), None);
        assert_eq!(camera(&json!({})), None);
    }

    #[test]
    fn a_disabled_camera_is_no_camera() {
        let entry = |enabled: Value| json!({"components": [{"name": "camera", "driver": "ov5647", "enabled": enabled, "clock_hz": 27_000_000}]});
        assert_eq!(camera(&entry(json!(false))), None);
        assert_eq!(camera(&entry(json!(null))), None);
        assert_eq!(camera(&entry(json!(true))), clocked(27_000_000));
        // absent reads as enabled
        assert_eq!(with_clock(json!(27_000_000)), clocked(27_000_000));
    }

    #[test]
    fn a_whole_float_is_a_clock_and_anything_else_is_kept_as_written() {
        // a float equal to a whole number of Hz, as a script's 27e6 writes it
        assert_eq!(with_clock(json!(27_000_000.0)), clocked(27_000_000));
        let parsed: Value = serde_json::from_str("27e6").expect("a JSON number");
        assert_eq!(with_clock(parsed), clocked(27_000_000));
        // the range's ends are in, a Hz past either is out (27 meaning MHz too)
        assert_eq!(with_clock(json!(6_000_000)), clocked(6_000_000));
        assert_eq!(with_clock(json!(27_000_000)), clocked(27_000_000));
        assert_eq!(with_clock(json!(27)), unusable("27"));
        assert_eq!(with_clock(json!(5_999_999)), unusable("5999999"));
        assert_eq!(with_clock(json!(27_000_001)), unusable("27000001"));
        // not a whole number of Hz, or not a number at all
        assert_eq!(with_clock(json!(24_000_000.5)), unusable("24000000.5"));
        assert_eq!(with_clock(json!(-24_000_000)), unusable("-24000000"));
        assert_eq!(with_clock(json!("27000000")), unusable("\"27000000\""));
        assert_eq!(with_clock(json!(true)), unusable("true"));
        // absent or null is missing
        assert_eq!(with_clock(json!(null)), unclocked(CameraClock::Missing));
        assert_eq!(
            camera(&json!({"components": [{"name": "camera", "driver": "ov5647"}]})),
            unclocked(CameraClock::Missing)
        );
    }

    #[test]
    fn the_first_camera_wins_sensors_first() {
        let both = json!({"sensors": [{"name": "camera", "driver": "ov5647", "clock_hz": 24_000_000}],
                          "components": [{"name": "nose", "driver": "ov5647", "clock_hz": 27_000_000}]});
        assert_eq!(camera(&both), clocked(24_000_000));
        // a disabled first one is the board's answer, as plan B's device() returns the first match
        let disabled_first = json!({"components": [
            {"name": "camera", "driver": "ov5647", "enabled": false, "clock_hz": 24_000_000},
            {"name": "nose", "driver": "ov5647", "clock_hz": 27_000_000}]});
        assert_eq!(camera(&disabled_first), None);
    }
}
