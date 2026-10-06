//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Which board is this, decided by I2C scan before any driver is set up: the port of src/glider layout.py
//! (doc/hardware.md "Transition"; doc/specs/avionics-port.md M5). The BUS MAP comes from where the moving devices
//! answer; the attitude parts then follow from the revision and from what the scan found.
//!
//! There are two bus maps, and they differ only in which bus four devices hang off, so one firmware can serve
//! both if it can tell them apart. It can: four addresses swap buses between the maps, which is four independent
//! votes rather than one hinge, so a single dead device cannot flip the verdict.
//!
//! ```text
//!                    i2c:0                          i2c:1
//!     v0.1 map   0x18 0x63 0x25 0x29  + module      0x40
//!     v1.x map   0x18 0x40            + module      0x63 0x25 0x29
//!
//!     SEN0697  BMI323 0x69  BMP581 0x47  BMM350 0x15   i2c:0   PRIMARY: expected on every scanned board
//!     SEN0253  BNO055 0x28  BMP280 0x76                i2c:0   BACKUP: enabled only when the scan finds it
//! ```
//!
//! THE MAP IS DECIDED BY PLACEMENT ALONE. The attitude module sits on i2c:0 on every board, so where it answers
//! says nothing about where the front devices are. It used to vote anyway, in one tally with the placement votes,
//! and that put a v0.1 board on the wrong map: TMS-7C, the one v0.1 board left and now carrying a SEN0697, read
//! v1.1 on three module votes against two placement votes, and its laser, pitot and ICP-10111 were moved to an
//! i2c:1 with nothing on it (scan 2026-10-05: i2c:0 0x15 0x18 0x25 0x29 0x47 0x63 0x69, i2c:1 empty).
//!
//! The three revision names stay, because configs and captures carry them:
//!
//! ```text
//!     v0.1   the v0.1 map + the SEN0697 + the ADXL375 on SPI    TMS-7C
//!     v1.1   the v1.x map + the SEN0697                         the taster, TMS-7F
//!     v1.0   the v1.x map + the SEN0253 alone                   legacy: TMS-7E as built -- DECLARED only
//! ```
//!
//! A SCAN NEVER ANSWERS v1.0. The v1.x map always scans as v1.1, because the SEN0697 is the module every board is
//! expected to carry: when it is missing, its setup fails and `arm` refuses. A scanned v1.0 used to switch it off
//! instead -- a two-module board whose SEN0697 connector had worked loose read v1.0 and would have flown on the
//! backup with nothing refusing. v1.0 survives as a DECLARED revision for the legacy build and the configs saved
//! for it, and there the SEN0253 IS the module: [`apply`] enables it.
//!
//! Everywhere else THE SEN0253 IS A BACKUP: a taster or an experimental board may carry both modules (different
//! addresses on the same bus), while a flight board normally carries the SEN0697 alone. After a scan, [`apply`]
//! enables the SEN0253 parts that answered and disables the rest; a declared v0.1 / v1.1 runs no scan, so the
//! config's own `enabled` stands. Wherever it ends up enabled, [`apply`] RANKS it below the SEN0697 and the
//! attitude filter on every quantity they share -- in code, whatever the config says, because every config saved
//! before 2026-10-05 carries the BNO055 at attitude priority 0 (see [`BELOW`]).
//!
//! The ADXL375 is on SPI and invisible here, so "no ADXL375" is carried by the map, never scanned for.
//!
//! `0x18` is the ANCHOR: the ES8311 audio codec soldered to the WaveShare board itself. It says nothing about the
//! revision (every board has it), and that is exactly what an anchor is for: proving the scan reached a live bus
//! rather than returning an empty set. Because it is soldered down, no sensor swap can remove it. The other
//! known-present addresses stay in the list as fallbacks, in case a board ever ships without the codec.
//!
//! The port, against plan B:
//!  * The scan is INJECTED, and plan B's `_scan` rules around it are kept HERE, not left to the firmware: a bus the
//!    config does not define is never scanned and casts no votes, and a bus that fails answers an empty set ("no
//!    votes from here", never evidence). [`resolve`] and [`detect`] take a closure `(bus id, bus spec) ->
//!    Result<Addresses, _>`, the part of `_scan` that touches hardware (`I2C(...)` and `scan()` at [`SCAN_HZ`]);
//!    the crate calls it only for a bus spec plan B would have opened (`scan_bus`), and reads an `Err` as an
//!    empty set. The P4 has both I2C ports whatever the config says, so a closure left to decide would probe a bus
//!    plan B never touched and cast votes plan B never cast. Plan B's re-clock dance after a runtime scan is not
//!    ported: `i2c_master` speed is per device (§6).
//!  * [`resolve`] returns what plan B logged and stored in the `RESOLVED` global: a [`Resolved`], whose
//!    [`Resolved::label`] is the health payload's layout and [`Resolved::log_lines`] the boot-log lines, word for
//!    word.
//!  * The config is a `serde_json::Value`, read with Python's `==`, `%s` and truthiness ([`board_config::python`]),
//!    so every change string and every edit is plan B's (the golden fixture compares them exactly, with the
//!    config_id that follows). Where plan B would raise on a config validate() refuses (a non-int priority, a
//!    `board` that is not an object, a null clock in `%d`), the port reads the nearest safe value instead; each
//!    function names its case.
//!  * Layout moves buses, clocks, `enabled`, a few control pins and priorities; it never touches a device's
//!    `axes` (the IMUs are mounted per their marks, x forward, y left, z up, and an `axes` key is the
//!    per-airframe exception the drivers read), nor `pins`.

use board_config::python::{equals, formatted, truthy};
use serde_json::{Map, Value};

/// The scan clock: every part tolerates 100 kHz, and the v1.x front bus runs there permanently.
pub const SCAN_HZ: u32 = 100_000;

/// A board revision, as configs (`board.layout`) and captures name it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Revision {
    /// The v0.1 map, the SEN0697 and the ADXL375 on SPI: TMS-7C.
    V0_1,
    /// The v1.x map with the SEN0253 alone: the legacy TMS-7E build. Declared only; a scan never names it.
    V1_0,
    /// The v1.x map and the SEN0697: the taster, TMS-7F.
    V1_1,
}

impl Revision {
    /// Every revision, in plan B's `_REVISIONS` order.
    pub const ALL: [Revision; 3] = [Revision::V0_1, Revision::V1_0, Revision::V1_1];

    /// The revision's name: 'v0.1', 'v1.0' or 'v1.1'.
    pub fn name(self) -> &'static str {
        match self {
            Revision::V0_1 => "v0.1",
            Revision::V1_0 => "v1.0",
            Revision::V1_1 => "v1.1",
        }
    }

    /// The revision a name declares; None for anything else ('auto', 'undecided', a typo), which plan B scans.
    pub fn from_name(name: &str) -> Option<Revision> {
        Revision::ALL.into_iter().find(|revision| revision.name() == name)
    }

    /// The bus map this revision lays out (plan B `_MAP`).
    pub fn bus_map(self) -> BusMap {
        match self {
            Revision::V0_1 => BusMap::V0_1,
            Revision::V1_0 | Revision::V1_1 => BusMap::V1x,
        }
    }

    /// The parts NOT fitted on this revision (plan B `_ABSENT`). These are the REVISION-DEPENDENT set:
    /// [`apply`] disables the ones listed for the revision and ENABLES the rest of the set, so a config that
    /// arrives with the SEN0697 disabled (anything saved before 2026-10-05) gets it switched ON, not merely left
    /// alone.
    ///
    /// v0.1 lists nothing: since 2026-10-05 it carries the SEN0697 AND the ADXL375. The SEN0253 is in no list:
    /// a backup revision (v1.0) fits it, and on any other revision only a scan does.
    fn absent(self) -> &'static [&'static str] {
        match self {
            Revision::V0_1 => &[],
            Revision::V1_0 => &["accel_adxl375", "imu_bmi323", "mag_bmm350", "baro_bmp581"],
            Revision::V1_1 => &["accel_adxl375"],
        }
    }
}

/// One of the two bus maps. Placement votes between these and nothing else.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusMap {
    /// ICP-10111, SDP810 and the laser on i2c:0, the INA226 on i2c:1.
    V0_1,
    /// The INA226 on i2c:0; the ICP-10111, SDP810 and the laser on the front harness, i2c:1.
    V1x,
}

impl BusMap {
    /// Both maps, in plan B's `_MAPS` order.
    pub const ALL: [BusMap; 2] = [BusMap::V0_1, BusMap::V1x];

    /// The map's name: 'v0.1' or 'v1.x'.
    pub fn name(self) -> &'static str {
        match self {
            BusMap::V0_1 => "v0.1",
            BusMap::V1x => "v1.x",
        }
    }

    /// The revision a SCAN names for this map (plan B `_SCANNED`): never v1.0.
    fn scanned(self) -> Revision {
        match self {
            BusMap::V0_1 => Revision::V0_1,
            BusMap::V1x => Revision::V1_1,
        }
    }

    /// The i2c:1 clock (plan B `_BUS1_HZ`). The v1.x front harness runs slow on purpose: nothing on it exceeds 50 Hz,
    /// so a quarter of the clock costs no sample rate and buys edge margin on a long unterminated run.
    fn bus1_hz(self) -> u32 {
        match self {
            BusMap::V0_1 => 400_000,
            BusMap::V1x => 100_000,
        }
    }

    /// The control pins [`apply`] writes per device (plan B `_CONTROL_PINS`), names and keys sorted as plan B
    /// walks them; None is "not routed", and every driver treats its pin as optional.
    ///
    /// v1.x: the front harness carries no laser INT/XSHUT, for either laser entry. The BMI323's data-ready is
    /// `accel_int1` (GPIO4), which the v1.x map frees because it has no ADXL375.
    /// v0.1: GPIO4 IS the ADXL375's INT1, routed on the PCB, and the attitude socket carries four wires only (SDA,
    /// SCL, 3V3, GND), so the BMI323 has no data-ready line there. If both parts kept `accel_int1`, two IRQ handlers
    /// would sit on one GPIO and the later setup would silently take over the ADXL375's interrupt.
    fn control_pins(self) -> &'static [(&'static str, &'static [PinSetting])] {
        match self {
            BusMap::V0_1 => &[("imu_bmi323", &[("int_pin", None)])],
            BusMap::V1x => &[
                ("imu_bmi323", &[("int_pin", Some("accel_int1"))]),
                ("laser_agl", &[("int_pin", None), ("xshut_pin", None)]),
                ("laser_agl_l1x", &[("int_pin", None), ("xshut_pin", None)]),
            ],
        }
    }
}

/// One control pin [`apply`] writes on a device: the config key and its value on a map (None: not routed).
type PinSetting = (&'static str, Option<&'static str>);

/// A device that MOVES between the maps: its config name, its address, and its bus on each map.
struct MovingDevice {
    /// The config name; names rather than addresses, because [`apply`] rewrites config blocks by name, and an
    /// address appearing twice in this table would be a silent bug where a name cannot be.
    name: &'static str,
    /// Its 7-bit address.
    address: u8,
    /// Its bus on the v0.1 map.
    v0_1_bus: usize,
    /// Its bus on the v1.x map.
    v1x_bus: usize,
}

impl MovingDevice {
    /// Its bus id on `map`.
    fn bus(&self, map: BusMap) -> usize {
        match map {
            BusMap::V0_1 => self.v0_1_bus,
            BusMap::V1x => self.v1x_bus,
        }
    }
}

/// The devices that MOVE, the only votes for a map (plan B `_MOVED`, in its order).
///
/// Every one votes from BOTH buses. The ICP-10111 used to count from i2c:1 only, as a guard against a second ICP
/// on i2c:0 -- considered and retired, because its ~120 ms general-call recovery must not land on the bus carrying
/// attitude. That guard cost TMS-7C, whose one ICP answers on i2c:0, its third vote. A stray second ICP now adds
/// one v0.1 vote beside the v1.x map's four, which cannot flip it.
const MOVED: [MovingDevice; 4] = [
    MovingDevice {
        name: "baro_icp10111",
        address: 0x63,
        v0_1_bus: 0,
        v1x_bus: 1,
    },
    MovingDevice {
        name: "airspeed_sdp810",
        address: 0x25,
        v0_1_bus: 0,
        v1x_bus: 1,
    },
    MovingDevice {
        name: "laser_agl",
        address: 0x29,
        v0_1_bus: 0,
        v1x_bus: 1,
    },
    MovingDevice {
        name: "power_ina226",
        address: 0x40,
        v0_1_bus: 1,
        v1x_bus: 0,
    },
];

/// Devices that occupy the SAME SOCKET as a moving one and follow it without voting (plan B `_FOLLOWS`).
///
/// `laser_agl_l1x` is a VL53L1X in the same footprint and at the same 0x29 as the VL53L4CX `laser_agl`; only one is
/// ever soldered, and each driver's model-id check decides which comes up. They must share the bus and the pin
/// treatment -- but the address must be counted ONCE, or a socket with two candidate drivers would cast two votes
/// for the same physical evidence and outweigh the parts that are really there. So followers are moved by
/// [`apply`] and ignored by the vote.
const FOLLOWS: [(&str, &[&str]); 1] = [("laser_agl", &["laser_agl_l1x"])];

/// The PRIMARY module, the SEN0697, by config name and address on i2c:0 (plan B `_PRIMARY`, in its order). Expected
/// on every revision but the declared legacy v1.0. A scan counts it for the detail line only: its absence changes
/// no verdict, it fails its setup.
const PRIMARY: [(&str, u8); 3] = [("imu_bmi323", 0x69), ("baro_bmp581", 0x47), ("mag_bmm350", 0x15)];

/// The BACKUP module, the SEN0253, by config name and address on i2c:0 (plan B `_BACKUP`), SORTED BY NAME, the order
/// plan B walks it in (`sorted(_BACKUP)`). Enabled only when a scan finds it, except on a backup revision (v1.0,
/// `BACKUP_REVISIONS`).
const BACKUP: [(&str, u8); 2] = [("baro_bmp280", 0x76), ("imu_bno055", 0x28)];

/// The revisions whose attitude module IS the SEN0253 (declared only).
const BACKUP_REVISIONS: [Revision; 1] = [Revision::V1_0];

/// ANY one of these on i2c:0 proves the scan reached a live bus. 0x18 is the on-board ES8311 codec: unpluggable,
/// revision-independent, and the only one a sensor swap cannot take away. The order is plan B's, which the
/// "no anchor" detail prints.
const ANCHORS: [u8; 5] = [0x18, 0x28, 0x76, 0x69, 0x47];

/// The RANKING [`apply`] enforces whatever priorities the config carries (plan B `_BELOW`): (device, the devices it
/// must rank strictly BELOW). Wherever the device is enabled, each quantity it provides that an ENABLED superior
/// also provides is demoted to one past the worst-ranked such superior. It only ever demotes, and a quantity no
/// enabled superior provides is left alone.
///
/// In code rather than in config_default alone, because a config is used wholesale: every config saved before
/// 2026-10-05 -- tms7f.config, tms7e.config, a taster's board.config, anything launch_config.py --base derives from
/// them -- carries the BNO055 at attitude priority 0 and the BMP280 at priority 1, tied with the BMP581. A SEN0253
/// found beside the SEN0697 on such a config would fly as the PRIMARY, with the filter mirroring it.
///
///  * the ADXL375 below the BMI323 on `accel`: it is the +/-200 g backstop, 49 mg a count, and both are fitted on
///    v0.1 only. The LSM6DSO32 stays the primary over both.
///  * the SEN0253 below the SEN0697 and the attitude filter on everything they share -- and the BNO055 below the
///    ADXL375 on `accel` as well, which config_default has always ranked it behind; without that the ADXL375's
///    demotion above would tie the two.
///
/// ORDER MATTERS: the ADXL375 is settled before the BNO055 is ranked against it.
pub const BELOW: [(&str, &[&str]); 3] = [
    ("accel_adxl375", &["imu_bmi323"]),
    (
        "imu_bno055",
        &["attitude", "imu_bmi323", "mag_bmm350", "baro_bmp581", "accel_adxl375"],
    ),
    ("baro_bmp280", &["attitude", "imu_bmi323", "mag_bmm350", "baro_bmp581"]),
];

/// The 7-bit addresses that answered on one bus: a set as a bitmask, no allocation.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct Addresses(u128);

impl Addresses {
    /// No address.
    pub const EMPTY: Addresses = Addresses(0);

    /// Add an address. One above 0x7F is not a 7-bit address -- nothing can answer there -- and is dropped.
    pub fn insert(&mut self, address: u8) {
        if address < 0x80 {
            self.0 |= 1 << address;
        }
    }

    /// Whether `address` answered.
    pub fn contains(self, address: u8) -> bool {
        address < 0x80 && self.0 & (1 << address) != 0
    }

    /// Whether nothing answered.
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// The addresses in either set.
    pub fn union(self, other: Addresses) -> Addresses {
        Addresses(self.0 | other.0)
    }

    /// The addresses, ascending.
    pub fn iter(self) -> impl Iterator<Item = u8> {
        (0..0x80_u8).filter(move |address| self.contains(*address))
    }

    /// Python's repr of plan B's `sorted('0x%02x' % a for a in seen)`, as the detail line prints it:
    /// `['0x15', '0x18']`. Two lowercase hex digits each, so the text order is the numeric order.
    fn python_list(self) -> String {
        let items: Vec<String> = self.iter().map(|address| format!("'0x{address:02x}'")).collect();
        format!("[{}]", items.join(", "))
    }
}

impl FromIterator<u8> for Addresses {
    /// The set of the addresses given (see [`Addresses::insert`]).
    fn from_iter<Items: IntoIterator<Item = u8>>(items: Items) -> Self {
        let mut addresses = Addresses::EMPTY;
        for address in items {
            addresses.insert(address);
        }
        addresses
    }
}

/// One address as a scan reads in a log, `0x18`: what [`Addresses`] debug-prints each member as.
struct Hex(u8);

impl std::fmt::Debug for Hex {
    /// Two lowercase hex digits after `0x`.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "0x{:02x}", self.0)
    }
}

impl std::fmt::Debug for Addresses {
    /// The addresses in hex, as a scan reads in a log: `{0x15, 0x18}`.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.debug_set().entries(self.iter().map(Hex)).finish()
    }
}

/// One scan: what answered on i2c:0 and on i2c:1, indexed by bus id (plan B's `seen`).
pub type Scan = [Addresses; 2];

/// The decision on one scan (plan B `_verdict`'s tuple).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Verdict {
    /// The revision the scan names, v0.1 or v1.1 (a scan never names v1.0); None when undecided, and the caller
    /// must then leave the bus map exactly as written -- a guess mis-buses every sensor at once.
    pub revision: Option<Revision>,
    /// The human-readable evidence, plan B's text.
    pub detail: String,
    /// The backup (SEN0253) parts that answered on i2c:0, by config name, sorted; empty when undecided.
    pub found: Vec<&'static str>,
}

/// The decision on one scan: the map by placement, the revision the scan names for it, the backup found (plan B
/// `_verdict`).
///
/// Placement is the only evidence for a map. A MOVING device votes for whichever map puts it on the bus it actually
/// answered on; absence never votes AGAINST, so a dead or unfitted device costs its map one vote rather than casting
/// one for the other, and no single failure flips a verdict. A TIE (0-0 included: a scan that saw only the anchor
/// and the attitude module) is undecided, because the module cannot break it -- it is on i2c:0 whichever map this
/// is. An empty scan, or one with no anchor on i2c:0, is not trustworthy and decides nothing.
///
/// The modules are counted for the detail line and nothing else: the v1.x map is v1.1 whichever module answered,
/// so a missing SEN0697 fails its setup rather than reading as legacy v1.0.
pub fn verdict(seen: &Scan) -> Verdict {
    let undecided = |detail: String| Verdict {
        revision: None,
        detail,
        found: Vec::new(),
    };
    if seen[0].union(seen[1]).is_empty() {
        return undecided("both buses scanned empty -- no devices answered".to_string());
    }
    if !ANCHORS.iter().any(|anchor| seen[0].contains(*anchor)) {
        let anchors: Vec<String> = ANCHORS.iter().map(|anchor| format!("0x{anchor:02x}")).collect();
        return undecided(format!(
            "no anchor ({}) on i2c:0 -- the scan is not trustworthy",
            anchors.join(" ")
        ));
    }
    let placement = BusMap::ALL.map(|map| {
        MOVED
            .iter()
            .filter(|device| seen[device.bus(map)].contains(device.address))
            .count()
    });
    let primary = PRIMARY.iter().filter(|(_, address)| seen[0].contains(*address)).count();
    let found: Vec<&'static str> = BACKUP
        .iter()
        .filter(|(_, address)| seen[0].contains(*address))
        .map(|(name, _)| *name)
        .collect();
    let detail = format!(
        "i2c0={} i2c1={} placement v0.1:{} v1.x:{} SEN0697 {}/{} SEN0253 {}/{}",
        seen[0].python_list(),
        seen[1].python_list(),
        placement[0],
        placement[1],
        primary,
        PRIMARY.len(),
        found.len(),
        BACKUP.len()
    );
    if placement[0] == placement[1] {
        return undecided(format!("ambiguous placement -- {detail}"));
    }
    let map = if placement[0] > placement[1] {
        BusMap::V0_1
    } else {
        BusMap::V1x
    };
    Verdict {
        revision: Some(map.scanned()),
        detail,
        found,
    }
}

/// What answered on I2C bus `bus` (plan B `_scan`, the rules around the hardware): `scan`'s answer when the config
/// defines a bus plan B would have opened, else no address.
///
/// Plan B's rules, kept here so no firmware closure can bend them:
///  * `config.bus(cfg, 'i2c', id) is None` -- no `buses.i2c."<id>"` (bus ids are JSON keys, strings), or a null one
///    -- is an empty set with no hardware touched: a board whose config defines one bus scans one bus.
///  * `I2C(id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=...)` reads both pins BEFORE it opens anything, so a
///    spec that is not an object, or lacks `scl` or `sda`, raised there, inside the `try`: an empty set, still with
///    no hardware touched. `scan` is not called for it either.
///  * whatever fails from there on (a pin the closure cannot use, a shorted or unpopulated bus) is `scan`'s `Err`,
///    an empty set: "a shorted or unpopulated bus votes for nothing".
///
/// A `buses` or `buses.i2c` that is not an object reads as no bus (plan B would raise; validate() refuses it).
fn scan_bus<Failure>(
    document: &Value,
    bus: u8,
    scan: &mut impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, Failure>,
) -> Addresses {
    let spec = document
        .get("buses")
        .and_then(|buses| buses.get("i2c"))
        .and_then(|i2c| i2c.get(bus.to_string()))
        .and_then(Value::as_object)
        .filter(|spec| spec.contains_key("scl") && spec.contains_key("sda"));
    match spec {
        Some(spec) => scan(bus, spec).unwrap_or(Addresses::EMPTY),
        None => Addresses::EMPTY,
    }
}

/// Both buses as plan B scans them, i2c:0 then i2c:1 (see `scan_bus`).
fn scan_both<Failure>(
    document: &Value,
    mut scan: impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, Failure>,
) -> Scan {
    [scan_bus(document, 0, &mut scan), scan_bus(document, 1, &mut scan)]
}

/// Scan both buses through `scan` (i2c:0, then i2c:1, each only when the config defines it: see `scan_bus`) and
/// decide the layout (plan B `detect`, the CC `detect` command): the revision (None when undecided) and the
/// evidence text. Nothing is changed.
///
/// `scan` is the hardware: open bus `id` on the spec's pins at [`SCAN_HZ`] and list what answers; an `Err` (the bus
/// would not open, the scan failed) reads as an empty set.
pub fn detect<Failure>(
    document: &Value,
    scan: impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, Failure>,
) -> (Option<Revision>, String) {
    let Verdict { revision, detail, .. } = verdict(&scan_both(document, scan));
    (revision, detail)
}

/// Whether `device` is physically present on `revision` -- the ONE place that question is answered (plan B
/// `fitted`). `revision` None is an undecided scan (or any name that is not a revision).
///
/// Every caller that instead wrote its own revision literal has eventually been wrong: resolve() gated on ('v0.1',
/// 'v1.0') and so ignored v1.1 entirely, and test_spibus asked `revision != 'v1.0'` and so demanded an ADXL375 from a
/// v1.1 board that has none. Both read fine until a third revision existed. Ask here instead, and adding a revision
/// updates every caller at once.
///
/// UNDECIDED answers False for the parts some revision removes, and for the moving devices: unknown is not evidence
/// of presence, and a test that demands a part on an unidentifiable board reports a hardware fault when what it
/// found was an inconclusive scan. (The follower `laser_agl_l1x` is not a moving device, so undecided answers True
/// for it, as plan B does.)
///
/// The SEN0253 answers True on a backup revision (v1.0) only, where it is the module. No other revision guarantees it:
/// only a scan finds it there, and the resolved config's `enabled` is what records the result.
pub fn fitted(device: &str, revision: Option<Revision>) -> bool {
    if BACKUP.iter().any(|(name, _)| *name == device) {
        return revision.is_some_and(|revision| BACKUP_REVISIONS.contains(&revision));
    }
    match revision {
        Some(revision) => !revision.absent().contains(&device),
        None => {
            !MOVED.iter().any(|moving| moving.name == device)
                && !Revision::ALL.iter().any(|revision| revision.absent().contains(&device))
        }
    }
}

/// Where a device sits (plan B `config.device(cfg, name=...)`): the first object named `name` in `sensors`, then in
/// `components`, as (section, index). A section that is not a list holds no device here (plan B would raise).
fn locate(document: &Value, name: &str) -> Option<(&'static str, usize)> {
    ["sensors", "components"].into_iter().find_map(|section| {
        let items = document.get(section)?.as_array()?;
        let index = items.iter().position(|item| {
            item.as_object()
                .and_then(|device| device.get("name"))
                .and_then(Value::as_str)
                == Some(name)
        })?;
        Some((section, index))
    })
}

/// The device named `name`, to read.
fn device<'document>(document: &'document Value, name: &str) -> Option<&'document Map<String, Value>> {
    let (section, index) = locate(document, name)?;
    document[section][index].as_object()
}

/// The device named `name`, to rewrite.
fn device_mut<'document>(document: &'document mut Value, name: &str) -> Option<&'document mut Map<String, Value>> {
    let (section, index) = locate(document, name)?;
    document[section][index].as_object_mut()
}

/// The i2c:1 bus spec, to rewrite (plan B `config.bus(cfg, 'i2c', 1)`: bus ids are JSON object keys, strings).
fn second_bus_mut(document: &mut Value) -> Option<&mut Map<String, Value>> {
    document.get_mut("buses")?.get_mut("i2c")?.get_mut("1")?.as_object_mut()
}

/// A member of a device as plan B's `device.get(key)` reads it: null when absent.
fn member<'device>(device: &'device Map<String, Value>, key: &str) -> &'device Value {
    device.get(key).unwrap_or(&Value::Null)
}

/// Whether a device is enabled as plan B's `device.get('enabled', True)` reads it, by truthiness.
fn enabled(device: &Map<String, Value>) -> bool {
    device.get("enabled").is_none_or(truthy)
}

/// A provides entry's priority, as plan B's `entry.get('priority', 0)`: an integer, 0 when absent.
///
/// In i128, because a priority is any JSON integer the config holds, i64 or u64 (serde_json's two integer kinds):
/// validate() admits any int (`_is_int`), and plan B's Python int has no ceiling, so neither the reading nor the
/// demotion one past it may wrap. validate() admits only an int, so a config that reaches layout carries nothing
/// else; a value that is not an integer reads as 0 here, where plan B would compare it or raise.
fn priority(entry: &Value) -> i128 {
    entry.get("priority").and_then(integer).unwrap_or(0)
}

/// A JSON integer as i128, whichever of serde_json's kinds holds it; None for anything else.
fn integer(value: &Value) -> Option<i128> {
    value
        .as_i64()
        .map(i128::from)
        .or_else(|| value.as_u64().map(i128::from))
}

/// The JSON value a priority is stored as: an i64 where it fits, else a u64, as plan B's int would be written.
///
/// One value is out of reach: one past u64::MAX, a demotion below a superior AT u64::MAX. The config holds no
/// integer past u64::MAX (serde_json without arbitrary precision parses one as a float, and validate() then refuses
/// it as no int), so that demotion SATURATES at u64::MAX, a tie with the superior, where plan B writes 2**64. A tie
/// is the safe side of the choice: never a promotion, and the databoard's i32 ranks tie far below it anyway.
fn stored(priority: i128) -> Value {
    i64::try_from(priority).map_or_else(
        |_| Value::from(u64::try_from(priority).unwrap_or(u64::MAX)),
        Value::from,
    )
}

/// Python's `'%d' % value`: an int in decimal, a float truncated toward zero, a bool as 1 or 0.
///
/// A float prints as Python's `int(value)` does, every digit of it: `'%d' % 1e40` is
/// `10000000000000000303786028427003666890752`, the float's exact value, which `{:.0}` prints too (Rust formats a
/// float exactly). Truncating first keeps `{:.0}` from rounding, and `+ 0.0` folds the negative zero of a truncated
/// `-0.5` into Python's `0`. A JSON number is always finite, so there is no infinity to meet.
///
/// Plan B raises on anything else (a null bus clock: `'%d' % None`); the port prints its `%s` instead, so a boot
/// with such a config logs the change rather than failing to boot.
fn percent_d(value: &Value) -> String {
    match value {
        Value::Bool(flag) => u8::from(*flag).to_string(),
        Value::Number(number) if number.is_f64() => number
            .as_f64()
            .map_or_else(|| formatted(value), |float| format!("{:.0}", float.trunc() + 0.0)),
        Value::Number(number) => number.to_string(),
        _ => formatted(value),
    }
}

/// Rewrite the config in place for a revision: bus membership, the i2c:1 clock, what is fitted, the control pins,
/// the ranks (plan B `apply`).
///
/// Only the four moving devices (and their socket followers), one bus clock, the revision-dependent parts, the
/// backup module, a few optional control pins and the backup ranking ([`BELOW`]) differ -- no pin is renumbered, so
/// `pins` is untouched.
///
/// `found` is the backup (SEN0253) parts a scan found: those are enabled and the rest disabled. None means there was
/// no scan (a declared revision) and the config's own `enabled` stands -- except on a backup revision (v1.0), which
/// fits the whole SEN0253 either way.
///
/// Plan B's quirks, kept: a moving device whose `id` already matches keeps its `bus` whatever it says; a missing
/// `id` or clock reads as null and is rewritten. Returns the human-readable change strings for the boot log, empty
/// when the config already matched (the normal case on a board whose profile was written for it).
pub fn apply(document: &mut Value, revision: Revision, found: Option<&[&str]>) -> Vec<String> {
    let map = revision.bus_map();
    let mut changes = Vec::new();
    for moving in &MOVED {
        let want = moving.bus(map);
        let followers = FOLLOWS
            .iter()
            .find(|(leader, _)| *leader == moving.name)
            .map_or(&[][..], |(_, followers)| *followers);
        for name in std::iter::once(moving.name).chain(followers.iter().copied()) {
            let Some(device) = device_mut(document, name) else {
                continue;
            };
            let id = member(device, "id");
            if !equals(id, &Value::from(want)) {
                changes.push(format!("{name} i2c:{}->{want}", formatted(id)));
                device.insert("bus".to_string(), Value::from("i2c"));
                device.insert("id".to_string(), Value::from(want));
            }
        }
    }

    if let Some(spec) = second_bus_mut(document) {
        let hz = map.bus1_hz();
        if !equals(member(spec, "freq"), &Value::from(hz)) {
            let was = spec.get("freq").map_or_else(|| "0".to_string(), percent_d);
            changes.push(format!("i2c:1 {was}->{hz} Hz"));
            spec.insert("freq".to_string(), Value::from(hz));
        }
    }

    // fit or unfit every revision-dependent part, in BOTH directions: the set is the union of every revision's
    // absent list, so nothing outside it is ever touched -- the SEN0253 included, which has its own rule below
    let mut dependent: Vec<&str> = Revision::ALL
        .iter()
        .flat_map(|revision| revision.absent().iter().copied())
        .collect();
    dependent.sort_unstable();
    dependent.dedup();
    for name in dependent {
        fit(document, name, !revision.absent().contains(&name), &mut changes);
    }

    let module: Vec<&str> = BACKUP.iter().map(|(name, _)| *name).collect();
    let found = if BACKUP_REVISIONS.contains(&revision) {
        Some(&module[..]) // the SEN0253 IS this revision's module: fitted, scan or no scan
    } else {
        found
    };
    if let Some(found) = found {
        for (name, _) in BACKUP {
            fit(document, name, found.contains(&name), &mut changes);
        }
    }

    for (name, pins) in map.control_pins() {
        let Some(device) = device_mut(document, name) else {
            continue;
        };
        for (key, target) in *pins {
            let target = target.map_or(Value::Null, Value::from);
            let current = member(device, key);
            if !equals(current, &target) {
                changes.push(format!("{name} {key} {}->{}", formatted(current), formatted(&target)));
                device.insert((*key).to_string(), target);
            }
        }
    }
    rank(document, &mut changes, Some(revision));
    changes
}

/// Demote each ENABLED device in [`BELOW`] to strictly below its enabled superiors, per shared quantity (plan B
/// `_rank`), appending a change string for every priority moved.
///
/// Bus-independent, which is why [`resolve`] runs it on an undecided scan too (`revision` None): the ranking says
/// which source the databoard prefers, not where anything is wired.
///
/// NOT on a backup revision (v1.0): there the SEN0253 IS the module, and the board carries no BMM350, so the filter has
/// no heading reference of its own -- demoted below it, the BNO055 would never seed it and the heading would start
/// at 0 and stay gyro-integrated. The SEN0253 keeps the rank its config gives it, as it always flew there.
///
/// `provides` that is absent or falsy reads as nothing; a superior's entry is read for `priority` only.
pub fn rank(document: &mut Value, changes: &mut Vec<String>, revision: Option<Revision>) {
    for (name, superiors) in BELOW {
        let backup = BACKUP.iter().any(|(backup, _)| *backup == name);
        if backup && revision.is_some_and(|revision| BACKUP_REVISIONS.contains(&revision)) {
            continue; // the SEN0253 is this revision's own module, not a backup
        }
        let Some(own) = device(document, name).filter(|own| enabled(own)) else {
            continue;
        };
        let Some(provides) = own.get("provides").and_then(Value::as_object) else {
            continue;
        };
        let mut quantities: Vec<String> = provides.keys().cloned().collect();
        quantities.sort_unstable(); // plan B's sorted(provides), whatever order the map keeps
        for quantity in quantities {
            // the worst (highest-numbered) priority among the enabled superiors that provide this quantity
            let floor = superiors
                .iter()
                .filter_map(|superior| device(document, superior).filter(|superior| enabled(superior)))
                .filter_map(|superior| {
                    let entry = superior.get("provides")?.get(&quantity)?;
                    (!entry.is_null()).then(|| priority(entry))
                })
                .max();
            let Some(floor) = floor else {
                continue;
            };
            let entry = device_mut(document, name)
                .and_then(|own| own.get_mut("provides"))
                .and_then(|provides| provides.get_mut(&quantity));
            let Some(entry) = entry.and_then(Value::as_object_mut) else {
                continue;
            };
            // in i128: one past a superior at i64::MAX is 2**63, which plan B writes as it is (see `priority`);
            // plan B's `current <= floor` is `current < floor + 1`, and the saturated case changes nothing twice
            let current = entry.get("priority").and_then(integer).unwrap_or(0);
            let demoted = (floor + 1).min(i128::from(u64::MAX));
            if current < demoted {
                changes.push(format!("{name} {quantity} p{current}->p{demoted}"));
                entry.insert("priority".to_string(), stored(demoted));
            }
        }
    }
}

/// Enable a device the board carries, disable one it does not -- unless the operator unfitted it (plan B `_fit`),
/// appending a change string when `enabled` changes. A device the config does not declare is skipped.
///
/// `fitted: false` is the operator saying THIS part is dead or removed (only the literal false: plan B's
/// `is not False`). Without it, a part the board is supposed to carry would be re-enabled here: an `enabled: false`
/// was undone, and verify/arm refused on a part nobody could take out of the config.
pub fn fit(document: &mut Value, name: &str, present: bool, changes: &mut Vec<String>) {
    let Some(device) = device_mut(document, name) else {
        return;
    };
    let wanted = present && device.get("fitted") != Some(&Value::Bool(false));
    let current = device.get("enabled").cloned().unwrap_or(Value::Bool(true));
    if !equals(&current, &Value::Bool(wanted)) {
        let what = if wanted {
            "fitted -> enabled"
        } else {
            "not fitted -> disabled"
        };
        changes.push(format!("{name} {what}"));
        device.insert("enabled".to_string(), Value::Bool(wanted));
    }
}

/// What [`resolve`] concluded this boot: the revision, how it was reached, and every change it made.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Resolved {
    /// `board.layout` named a revision (or the key was absent: v0.1); no scan ran.
    Declared {
        /// The declared revision, applied.
        revision: Revision,
        /// [`apply`]'s changes.
        changes: Vec<String>,
    },
    /// The scan decided.
    Detected {
        /// The revision the scan names, applied.
        revision: Revision,
        /// The scan's evidence.
        detail: String,
        /// [`apply`]'s changes.
        changes: Vec<String>,
    },
    /// The scan was undecided: the bus map is left as written, only the ranking ran.
    Undecided {
        /// Why: the scan's evidence, or why it is not trustworthy.
        detail: String,
        /// [`rank`]'s changes.
        changes: Vec<String>,
    },
}

impl Resolved {
    /// The revision applied, None when undecided.
    pub fn revision(&self) -> Option<Revision> {
        match self {
            Resolved::Declared { revision, .. } | Resolved::Detected { revision, .. } => Some(*revision),
            Resolved::Undecided { .. } => None,
        }
    }

    /// Every change made to the config.
    pub fn changes(&self) -> &[String] {
        match self {
            Resolved::Declared { changes, .. }
            | Resolved::Detected { changes, .. }
            | Resolved::Undecided { changes, .. } => changes,
        }
    }

    /// The health payload's layout (plan B's `RESOLVED`): 'v0.1 (declared)', 'v1.1' or 'undecided'. The operator
    /// should see which revision the firmware decided it runs on WITHOUT reading the boot log, since a wrong verdict
    /// and a miswired board look identical from the device list.
    pub fn label(&self) -> String {
        match self {
            Resolved::Declared { revision, .. } => format!("{} (declared)", revision.name()),
            Resolved::Detected { revision, .. } => revision.name().to_string(),
            Resolved::Undecided { .. } => "undecided".to_string(),
        }
    }

    /// The boot-log lines plan B's resolve() logs, word for word.
    pub fn log_lines(&self) -> Vec<String> {
        match self {
            Resolved::Declared { revision, changes } => {
                let what = if changes.is_empty() {
                    "already matched".to_string()
                } else {
                    changes.join(", ")
                };
                vec![format!("layout :: {} (declared) {what}", revision.name())]
            }
            Resolved::Detected {
                revision,
                detail,
                changes,
            } => {
                let mut lines = vec![format!("layout :: {} detected -- {detail}", revision.name())];
                if !changes.is_empty() {
                    lines.push(format!("layout :: applied {}", changes.join(", ")));
                }
                lines
            }
            Resolved::Undecided { detail, changes } => {
                let mut lines = vec![format!("layout :: UNDECIDED, bus map left as written -- {detail}")];
                if !changes.is_empty() {
                    lines.push(format!("layout :: ranked {}", changes.join(", ")));
                }
                lines
            }
        }
    }
}

/// The boot entry point (plan B `resolve`): honour a declared `board.layout`, else scan, else change the bus map
/// not. The config is rewritten in place; `scan` (as [`detect`] takes it) is asked for i2c:0 then i2c:1, only when
/// nothing is declared, and only for a bus the config defines (`scan_bus`).
///
/// An explicit 'v0.1' / 'v1.0' / 'v1.1' always wins over the scan, so a board can be pinned when a sensor is
/// unfitted and would otherwise abstain its way to a wrong verdict. Anything else ('auto', the default; null; a
/// typo) scans: every name in [`Revision::ALL`] is checked, not a literal pair, so a revision added later cannot
/// silently fall through to the scan, which is what once made a declared v1.1 ignored. A declared v0.1 / v1.1 never
/// touches the SEN0253's `enabled`: with no scan there is nothing to say whether it is fitted, so the config's own
/// value stands (a declared v1.0 fits it -- it IS v1.0). Whatever the path, an enabled SEN0253 is ranked below the
/// SEN0697 ([`BELOW`]).
///
/// A config with NO `layout` key means v0.1, not 'auto'. A valid board.config is used wholesale, so a profile
/// written before the field existed genuinely arrives without it, and every such profile belongs to a board built
/// the old way. TMS-7C, the one v0.1 board left, flies keyless profiles from make_telemetry_config.py; defaulting it
/// to a scan would put a built, closed airframe's bus map at the mercy of its sensors answering (7C casts three
/// placement votes, so a front harness fault is all it takes to make it undecided). A `board` that is not an object
/// reads as keyless here (plan B would raise; validate() refuses such a config).
///
/// UNDECIDED applies no fallback guess: the wrong revision re-buses every moving sensor at once, which is a worse
/// failure than running the config as written and saying so. The ranking still runs -- it depends on no bus, and an
/// old profile's p0 BNO055 is as wrong there as anywhere.
pub fn resolve<Failure>(
    document: &mut Value,
    scan: impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, Failure>,
) -> Resolved {
    let declared = document
        .get("board")
        .and_then(Value::as_object)
        .and_then(|board| board.get("layout"))
        .map_or(Some(Revision::V0_1), |layout| {
            layout.as_str().and_then(Revision::from_name)
        });
    if let Some(revision) = declared {
        let changes = apply(document, revision, None);
        return Resolved::Declared { revision, changes };
    }
    let Verdict {
        revision,
        detail,
        found,
    } = verdict(&scan_both(document, scan));
    match revision {
        None => {
            let mut changes = Vec::new();
            rank(document, &mut changes, None);
            Resolved::Undecided { detail, changes }
        }
        Some(revision) => {
            let changes = apply(document, revision, Some(&found));
            Resolved::Detected {
                revision,
                detail,
                changes,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        Addresses, BACKUP, BusMap, MOVED, PRIMARY, Resolved, Revision, Scan, apply, detect, fit, fitted, percent_d,
        rank, resolve, verdict,
    };
    use serde_json::{Map, Value, json};

    /// A scan from address lists, {i2c:0, i2c:1}.
    fn scan(bus0: &[u8], bus1: &[u8]) -> Scan {
        [bus0.iter().copied().collect(), bus1.iter().copied().collect()]
    }

    /// TMS-7C as it answered on 2026-10-05 (test_layout.py `_TMS7C`).
    fn tms7c() -> Scan {
        scan(&[0x15, 0x18, 0x25, 0x29, 0x47, 0x63, 0x69], &[])
    }

    /// The v1.1 taster (test_layout.py `_TASTER`).
    fn taster() -> Scan {
        scan(&[0x15, 0x18, 0x40, 0x47, 0x69], &[0x25, 0x29, 0x63])
    }

    /// A small config: the four moving devices on the v0.1 map, the SEN0697, the SEN0253 off, an ADXL375.
    fn config(layout: Option<&str>) -> Value {
        let mut board = json!({"id": "taster"});
        if let Some(layout) = layout {
            board["layout"] = json!(layout);
        }
        json!({
            "board": board,
            "buses": {"i2c": {
                "0": {"freq": 400000, "scl": 8, "sda": 7},
                "1": {"freq": 400000, "scl": 30, "sda": 31},
            }},
            "sensors": [
                {"name": "baro_icp10111", "bus": "i2c", "id": 0},
                {"name": "airspeed_sdp810", "bus": "i2c", "id": 0},
                {"name": "laser_agl", "bus": "i2c", "id": 0, "int_pin": "laser_int", "xshut_pin": "laser_xshut"},
                {"name": "power_ina226", "bus": "i2c", "id": 1},
                {"name": "accel_adxl375", "provides": {"accel": {"priority": 1}}},
                {"name": "imu_bmi323", "int_pin": null, "provides": {"accel": {"priority": 1}}},
                {"name": "imu_bno055", "enabled": false, "provides": {"attitude": {"priority": 0}}},
            ],
            "components": [{"name": "attitude", "provides": {"attitude": {"priority": 1}}}],
        })
    }

    /// A scan closure that must never be called.
    fn no_scan(bus: u8, _spec: &Map<String, Value>) -> Result<Addresses, ()> {
        panic!("a declared layout scanned bus {bus}")
    }

    /// A scan closure answering `seen` for every bus it is asked, recording which it was asked, in order.
    fn scripted<'asked>(
        seen: Scan,
        asked: &'asked mut Vec<u8>,
    ) -> impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, ()> + 'asked {
        move |bus, _spec| {
            asked.push(bus);
            Ok(seen[usize::from(bus)])
        }
    }

    #[test]
    fn revision_names_round_trip_and_nothing_else_declares() {
        for revision in Revision::ALL {
            assert_eq!(Revision::from_name(revision.name()), Some(revision));
        }
        for name in ["auto", "undecided", "v1.x", "V1.1", "", "v2.0"] {
            assert_eq!(Revision::from_name(name), None, "{name}");
        }
        assert_eq!(Revision::V1_0.bus_map(), BusMap::V1x);
        assert_eq!(BusMap::ALL.map(BusMap::name), ["v0.1", "v1.x"]);
    }

    #[test]
    fn addresses_are_seven_bit() {
        let mut addresses: Addresses = [0x18, 0x7f, 0x00].into_iter().collect();
        addresses.insert(0x80);
        addresses.insert(0xff);
        assert_eq!(addresses.iter().collect::<Vec<_>>(), [0x00, 0x18, 0x7f]);
        assert!(!addresses.contains(0x80) && !addresses.contains(0x17));
        assert_eq!(format!("{addresses:?}"), "{0x00, 0x18, 0x7f}");
        assert_eq!(addresses.python_list(), "['0x00', '0x18', '0x7f']");
        assert_eq!(Addresses::EMPTY.python_list(), "[]");
        assert!(Addresses::EMPTY.is_empty() && !addresses.is_empty());
    }

    #[test]
    fn the_tables_are_in_the_order_plan_b_walks_them() {
        // BACKUP is walked as sorted(_BACKUP); the control pins as sorted names and keys
        assert!(BACKUP.windows(2).all(|pair| pair[0].0 < pair[1].0));
        for map in BusMap::ALL {
            let pins = map.control_pins();
            assert!(pins.windows(2).all(|pair| pair[0].0 < pair[1].0), "{map:?}");
            for (_, keys) in pins {
                assert!(keys.windows(2).all(|pair| pair[0].0 < pair[1].0), "{map:?}");
            }
        }
        // a moving device's address appears once, and no module part moves
        for (index, moving) in MOVED.iter().enumerate() {
            assert!(MOVED[index + 1..].iter().all(|other| other.address != moving.address));
            assert!(PRIMARY.iter().chain(&BACKUP).all(|(name, _)| *name != moving.name));
        }
    }

    #[test]
    fn placement_alone_decides_the_map() {
        let verdict_7c = verdict(&tms7c());
        assert_eq!(verdict_7c.revision, Some(Revision::V0_1));
        assert!(verdict_7c.detail.contains("v0.1:3 v1.x:0") && verdict_7c.detail.contains("SEN0697 3/3"));
        assert_eq!(verdict(&taster()).revision, Some(Revision::V1_1));
        // NEGATIVE: the module alone, a 1-1 tie, an empty scan and a scan with no anchor decide nothing
        assert_eq!(
            verdict(&scan(&[0x18, 0x15, 0x47, 0x69, 0x28, 0x76], &[])).revision,
            None
        );
        let tie = verdict(&scan(&[0x18, 0x29, 0x40], &[]));
        assert!(tie.revision.is_none() && tie.detail.starts_with("ambiguous placement -- "));
        assert_eq!(
            verdict(&scan(&[], &[])).detail,
            "both buses scanned empty -- no devices answered"
        );
        let unanchored = verdict(&scan(&[0x63, 0x25, 0x29], &[0x18, 0x40]));
        assert_eq!(
            unanchored.detail,
            "no anchor (0x18 0x28 0x76 0x69 0x47) on i2c:0 -- the scan is not trustworthy"
        );
        assert!(unanchored.found.is_empty());
        // the legacy SEN0253-only board scans as v1.1 with the backup found, never v1.0
        let legacy = verdict(&scan(&[0x18, 0x28, 0x40, 0x76], &[0x25, 0x29, 0x63]));
        assert_eq!(legacy.revision, Some(Revision::V1_1));
        assert_eq!(legacy.found, ["baro_bmp280", "imu_bno055"]);
    }

    #[test]
    fn a_declared_or_keyless_layout_never_scans() {
        let mut keyless = config(None);
        assert_eq!(resolve(&mut keyless, no_scan).label(), "v0.1 (declared)");
        let mut declared = config(Some("v1.1"));
        let resolved = resolve(&mut declared, no_scan);
        assert_eq!(resolved.revision(), Some(Revision::V1_1));
        assert_eq!(declared["sensors"][0]["id"], 1, "the declared map is applied");
        // a board that is not an object reads as keyless (plan B would raise): v0.1, no scan
        let mut odd = json!({"board": "TMS-7C"});
        assert_eq!(
            resolve(&mut odd, no_scan),
            Resolved::Declared {
                revision: Revision::V0_1,
                changes: vec![]
            }
        );
    }

    #[test]
    fn auto_scans_both_buses_in_order() {
        let mut document = config(Some("auto"));
        let mut asked = Vec::new();
        let resolved = resolve(&mut document, |bus, spec: &Map<String, Value>| {
            assert_eq!(
                spec["scl"],
                [8, 30][usize::from(bus)],
                "the closure opens the bus on the spec's pins"
            );
            asked.push(bus);
            Ok::<_, ()>(taster()[usize::from(bus)])
        });
        assert_eq!(asked, [0, 1]);
        assert_eq!(resolved.label(), "v1.1");
        assert_eq!(
            resolved.log_lines()[0],
            format!("layout :: v1.1 detected -- {}", verdict(&taster()).detail)
        );
        assert!(resolved.changes().contains(&"baro_icp10111 i2c:0->1".to_string()));
        assert_eq!(document["buses"]["i2c"]["1"]["freq"], 100000);
        // detect answers the same verdict and changes nothing
        let before = config(Some("auto"));
        let mut asked = Vec::new();
        let (revision, _) = detect(&before, scripted(tms7c(), &mut asked));
        assert_eq!((revision, asked), (Some(Revision::V0_1), vec![0, 1]));
    }

    #[test]
    fn a_bus_the_config_does_not_define_is_never_scanned() {
        // the P4 has both ports whatever the config says: plan B scans only a bus spec it can open the pins of
        let mut no_second_bus = config(Some("auto"));
        no_second_bus["buses"]["i2c"].as_object_mut().unwrap().remove("1");
        let mut asked = Vec::new();
        let resolved = resolve(&mut no_second_bus, scripted(taster(), &mut asked));
        assert_eq!(
            asked,
            [0],
            "no i2c:1 spec: no i2c:1 scan, and none of its three v1.x votes"
        );
        assert_eq!(resolved.revision(), Some(Revision::V1_1), "the INA226 on i2c:0 alone");
        assert!(resolved.log_lines()[0].contains("i2c1=[] placement v0.1:0 v1.x:1"));
        // NEGATIVE: a null spec, a spec that is not an object, one without a pin: never handed to the closure
        for spec in [
            json!(null),
            json!("fast"),
            json!([30, 31]),
            json!({"sda": 31, "freq": 100000}),
        ] {
            let mut document = config(Some("auto"));
            document["buses"]["i2c"]["1"] = spec.clone();
            let mut asked = Vec::new();
            let (revision, _) = detect(&document, scripted(taster(), &mut asked));
            assert_eq!((revision, asked), (Some(Revision::V1_1), vec![0]), "{spec}");
        }
        // no buses at all (or not objects): nothing scanned, nothing seen, undecided
        for buses in [None, Some(json!([])), Some(json!({"i2c": "both"}))] {
            let mut document = config(Some("auto"));
            match buses {
                None => document.as_object_mut().unwrap().remove("buses"),
                Some(buses) => document.as_object_mut().unwrap().insert("buses".to_string(), buses),
            };
            let mut asked = Vec::new();
            let resolved = resolve(&mut document, scripted(taster(), &mut asked));
            assert!(asked.is_empty());
            assert_eq!(
                resolved.log_lines()[0],
                "layout :: UNDECIDED, bus map left as written -- both buses scanned empty -- no devices answered"
            );
        }
    }

    #[test]
    fn a_bus_that_fails_votes_for_nothing() {
        // the closure's Err is plan B's `except Exception: found = set()`
        let failing = |broken: u8| {
            move |bus: u8, _spec: &Map<String, Value>| {
                if bus == broken {
                    Err("a shorted bus")
                } else {
                    Ok(taster()[usize::from(bus)])
                }
            }
        };
        let (revision, detail) = detect(&config(Some("auto")), failing(1));
        assert_eq!(revision, Some(Revision::V1_1));
        assert!(detail.contains("i2c1=[] placement v0.1:0 v1.x:1"), "{detail}");
        // NEGATIVE: i2c:0 failing takes the anchor with it -- the scan is not trusted, whatever i2c:1 saw
        let mut document = config(Some("auto"));
        let resolved = resolve(&mut document, failing(0));
        assert_eq!(resolved.label(), "undecided");
        assert!(resolved.log_lines()[0].contains("no anchor"));
        assert_eq!(document["sensors"][0]["id"], 0, "the bus map is left as written");
    }

    #[test]
    fn a_demotion_never_wraps_past_the_i64_ceiling() {
        // a superior at i64::MAX: plan B's int writes 2**63, the u64 the port stores, below the superior
        let mut document = config(None);
        document["sensors"][4]["provides"]["accel"]["priority"] = json!(2); // the ADXL375 already below the BMI323
        document["sensors"][6]["enabled"] = json!(true);
        document["components"][0]["provides"]["attitude"]["priority"] = json!(i64::MAX);
        let mut changes = Vec::new();
        rank(&mut document, &mut changes, Some(Revision::V1_1));
        assert_eq!(changes, ["imu_bno055 attitude p0->p9223372036854775808"]);
        let demoted = &document["sensors"][6]["provides"]["attitude"]["priority"];
        assert_eq!(demoted.as_u64(), Some(1 << 63));
        assert!(
            demoted.as_u64().unwrap() > i64::MAX as u64,
            "below the superior, not promoted to i64::MIN"
        );
        // a superior past i64::MAX reads as itself, not as 0
        document["components"][0]["provides"]["attitude"]["priority"] = json!((1_u64 << 63) + 5);
        changes.clear();
        rank(&mut document, &mut changes, Some(Revision::V1_1));
        assert_eq!(
            changes,
            ["imu_bno055 attitude p9223372036854775808->p9223372036854775814"]
        );
        // NEGATIVE: one past u64::MAX is no integer the config holds: a tie at u64::MAX, and a second pass is quiet
        document["components"][0]["provides"]["attitude"]["priority"] = json!(u64::MAX);
        changes.clear();
        rank(&mut document, &mut changes, Some(Revision::V1_1));
        assert_eq!(
            changes,
            ["imu_bno055 attitude p9223372036854775814->p18446744073709551615"]
        );
        assert_eq!(
            document["sensors"][6]["provides"]["attitude"]["priority"],
            json!(u64::MAX)
        );
        changes.clear();
        rank(&mut document, &mut changes, Some(Revision::V1_1));
        assert!(changes.is_empty(), "{changes:?}");
    }

    #[test]
    fn an_undecided_scan_only_ranks() {
        let mut document = config(Some("auto"));
        document["sensors"][6]["enabled"] = json!(true); // a BNO055 an old profile left on at attitude p0
        let resolved = resolve(&mut document, |_bus, _spec: &Map<String, Value>| {
            Ok::<_, ()>(Addresses::EMPTY)
        });
        assert_eq!(resolved.label(), "undecided");
        // the ADXL375 tied with the BMI323 (both p1) is settled first, then the BNO055 below the filter
        assert_eq!(
            resolved.changes(),
            ["accel_adxl375 accel p1->p2", "imu_bno055 attitude p0->p2"]
        );
        assert_eq!(document["sensors"][0]["id"], 0, "the bus map is left as written");
        assert_eq!(
            resolved.log_lines(),
            [
                "layout :: UNDECIDED, bus map left as written -- both buses scanned empty -- no devices answered",
                "layout :: ranked accel_adxl375 accel p1->p2, imu_bno055 attitude p0->p2",
            ]
        );
    }

    #[test]
    fn apply_is_idempotent_and_reversible() {
        let mut document = config(None);
        let first = apply(&mut document, Revision::V1_1, Some(&[]));
        assert!(
            first.contains(&"laser_agl int_pin laser_int->None".to_string()),
            "{first:?}"
        );
        assert!(first.contains(&"accel_adxl375 not fitted -> disabled".to_string()));
        assert!(first.contains(&"imu_bmi323 int_pin None->accel_int1".to_string()));
        assert_eq!(apply(&mut document, Revision::V1_1, Some(&[])), Vec::<String>::new());
        let back = apply(&mut document, Revision::V0_1, None);
        assert!(
            back.contains(&"power_ina226 i2c:0->1".to_string())
                && back.contains(&"i2c:1 100000->400000 Hz".to_string())
        );
        assert_eq!(
            document["sensors"][4]["enabled"], true,
            "the ADXL375 is refitted on v0.1"
        );
        // the ADXL375 ranks below the BMI323 wherever both are enabled (both were p1)
        assert_eq!(document["sensors"][4]["provides"]["accel"]["priority"], 2);
    }

    #[test]
    fn fitted_false_beats_every_revision_and_scan() {
        let mut document = config(None);
        document["sensors"][5]["fitted"] = json!(false);
        let mut changes = Vec::new();
        fit(&mut document, "imu_bmi323", true, &mut changes);
        assert_eq!(changes, ["imu_bmi323 not fitted -> disabled"]);
        // only the literal false: 0 is not False
        document["sensors"][5]["fitted"] = json!(0);
        changes.clear();
        fit(&mut document, "imu_bmi323", true, &mut changes);
        assert_eq!(changes, ["imu_bmi323 fitted -> enabled"]);
        // NEGATIVE: an undeclared device is skipped, not invented
        fit(&mut document, "no_such_device", true, &mut changes);
        assert_eq!(changes.len(), 1);
    }

    #[test]
    fn fitted_answers_every_revision_and_undecided() {
        assert!(fitted("accel_adxl375", Some(Revision::V0_1)));
        assert!(
            !fitted("accel_adxl375", Some(Revision::V1_1)),
            "the literal-pair bug this replaces"
        );
        for (name, _) in PRIMARY {
            assert!(fitted(name, Some(Revision::V0_1)) && fitted(name, Some(Revision::V1_1)));
            assert!(!fitted(name, Some(Revision::V1_0)) && !fitted(name, None));
        }
        for (name, _) in BACKUP {
            assert!(fitted(name, Some(Revision::V1_0)));
            assert!(!fitted(name, Some(Revision::V0_1)) && !fitted(name, Some(Revision::V1_1)) && !fitted(name, None));
        }
        assert!(
            fitted("laser_ground", None),
            "a part no revision removes is fitted everywhere"
        );
        assert!(
            !fitted("baro_icp10111", None),
            "a moving device is not known to be anywhere"
        );
        assert!(
            fitted("laser_agl_l1x", None),
            "a follower does not vote, and plan B answers True"
        );
    }

    #[test]
    fn percent_d_formats_as_python() {
        assert_eq!(percent_d(&json!(400000)), "400000");
        assert_eq!(percent_d(&json!(100000.0)), "100000");
        assert_eq!(percent_d(&json!(-3.7)), "-3");
        assert_eq!(
            percent_d(&json!(-0.5)),
            "0",
            "truncated toward zero, and no negative zero"
        );
        assert_eq!(
            percent_d(&json!(1e40)),
            "10000000000000000303786028427003666890752",
            "every digit of the float, as Python's int()"
        );
        assert_eq!(percent_d(&json!(true)), "1");
        // where plan B raises, the port prints the %s
        assert_eq!(percent_d(&json!(null)), "None");
        assert_eq!(percent_d(&json!("fast")), "fast");
    }

    #[test]
    fn a_config_without_devices_changes_nothing_but_says_so() {
        let mut empty = json!({});
        let resolved = resolve(&mut empty, no_scan);
        assert_eq!(resolved.log_lines(), ["layout :: v0.1 (declared) already matched"]);
        assert_eq!(empty, json!({}));
    }
}
