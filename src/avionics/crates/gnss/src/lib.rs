//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The shared GNSS receiver (src/glider gnss.py): NMEA over a dedicated UART, talker-agnostic (GP, GN, BD, GL).
//!  * RMC gives the position, the ground speed and the course;
//!  * GGA gives the altitude above mean sea level, the elevation above the GNSS ground zero (the first altitude: a
//!    barometer backup) and the signal quality behind the fix (quality, satellites used, HDOP);
//!  * GSV (satellites in view and their C/N0), GSA (fix mode, satellites used) and the ATGM's antenna text are the
//!    sky diagnostics, folded into one row per burst. They tell a weak antenna from a dead receiver: the 2026-10-03
//!    flights never fixed, and with these sentences switched off their logs could not say why. They run on the pad
//!    only (before BOOSTING and again from DONE): in flight a burst delayed the epoch's RMC by up to ~0.6 s on the
//!    2026-10-04 bench, so the 200 ms position channel went stale once per burst.
//!
//! This crate is the receiver's logic, with no UART and no clock: the firmware's gnss thread hands in each line
//! read and the time ([`Gnss::line`]), runs the 1 s tick ([`Gnss::tick`]), and publishes what comes back: an
//! [`Update`] (the channel values, the `<name>.csv` and `<name>_gga.csv` rows, the events) and a [`Tick`] (the
//! events, the sky switch frame to write, the `<name>_sky.csv` row). The module-specific sentence selection and
//! rate are the drivers' command bytes ([`drivers::atgm336h`], [`drivers::neo6mv2`]).
//!
//! Positions are [`Position`] integers in 1e-7 degree, parsed straight from the ddmm.mmmm digits
//! ([`degrees_e7`]): plan B's board is single precision, where an f32 degree at latitude 48 has a 0.42 m ulp. The
//! other values are f32 (decision D18) and the telemetry integers stay integers (hdop_cd, the sky row).
//!
//! Deliberate differences from plan B (marked in the golden fixture or covered by the unit tests):
//!  * hdop_cd (the GGA row's HDOP in hundredths) is read from the decimal text. Plan B wrote `int(hdop * 100)` of
//!    the float, which truncates a binary fraction: on CPython 0.29 gives 28 and 1.15 gives 114 (about 6 % of the
//!    two-decimal values lose a unit), and the board's f32 loses a unit in other cases;
//!  * a ddmm.mmmm field must be digits, with at least one degree and two minute digits before the dot, in i32 units.
//!    Plan B slices it around the dot into `int()` and `float()`, so '4807' (no dot) reads 17.45 degrees and a
//!    sign or an exponent goes through; here the line is dropped, as plan B drops one whose field raises;
//!  * a sentence that is not ASCII fails its checksum. Plan B decoded it as UTF-8 and checksummed by character
//!    index over the encoded bytes, which only lines up for ASCII;
//!  * a field is read as MicroPython's `int()`/`float()` read it (surrounding ASCII whitespace, a sign, `inf` and
//!    `nan`), but an underscore between digits, which Python accepts, is malformed here;
//!  * the outage sweep reports RMC before GGA (CPython's dict order; a board's MicroPython dict has its own).
//!
//! Not ported: the UART, `probe()` and `diagnose()` (the firmware's I/O), inspect's databoard values (the channels
//! are the firmware's), and the UBX NAV-PVT path for the NEO-M8N (M16).

pub mod drivers;

use control::navigation::Position;
use sequencer::Stage;
use std::fmt;

/// The sky diagnostics come about this often, in seconds, at any fix rate: the drivers' dividers are `hz` times
/// this (plan B `gnss.DIAGNOSTICS_S`).
pub const DIAGNOSTICS_S: i32 = 10;

/// The tick's period in milliseconds: the outage sweep's own clock, for a receiver that sends nothing, and the
/// burst-ending clock of the sky row (plan B `_SWEEP_MS`).
pub const SWEEP_MS: u32 = 1000;

/// A sentence quiet this long (3 s) is an OUTAGE, not jitter (plan B `_SENTENCE_GAP_US`). The module is configured
/// at 1 Hz at least, so three missed intervals: loose enough that a busy loop or a single dropped line never cries
/// wolf, tight enough to catch the loss well inside a <60 s flight.
const SENTENCE_GAP_US: i64 = 3_000_000;

/// Half a second with no diagnostics sentence ends a burst (plan B `_BURST_QUIET_US`). A burst's sentences come
/// back to back, at most one RMC/GGA between two of them (~75 ms at 9600 baud; on the bench a whole ATGM burst
/// took 0.43-0.54 s, no two of its sentences more than ~0.17 s apart), and bursts are ~10 s apart: margin both ways.
const BURST_QUIET_US: i64 = 500_000;

/// RMC's speed is in knots; the airspeed governor wants m/s (plan B `_KNOTS_TO_MS`, the same truncated figure).
const KNOTS_TO_METRES_PER_SECOND: f32 = 0.514444;

/// The ATGM's antenna supervisor (`$GPTXT,01,01,01,ANTENNA OPEN`) as the sky row's code. OPEN means the feed draws
/// no current: a PASSIVE antenna reads OPEN too, and on the 2026-10-04 bench the passive 12x12 patch the airframes
/// flew cost the ATGM ~8-10 dB against the active ones (OK). The NEO-6M reports no antenna status over NMEA, so its
/// rows carry 0.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Antenna {
    /// No antenna text heard (always, on a NEO-6M).
    #[default]
    Unknown = 0,
    /// `ANTENNA OK`: an active antenna drawing current.
    Ok = 1,
    /// `ANTENNA OPEN`: no current drawn (a passive antenna, or a broken feed).
    Open = 2,
    /// `ANTENNA SHORT`: the feed is shorted.
    Short = 3,
}

impl Antenna {
    /// The code as a word, for inspect and the events (plan B `_ANTENNA_NAMES`).
    pub fn name(self) -> &'static str {
        match self {
            Antenna::Unknown => "unknown",
            Antenna::Ok => "OK",
            Antenna::Open => "OPEN",
            Antenna::Short => "SHORT",
        }
    }

    /// The status a TXT sentence's text field names (plan B `_ANTENNA_CODES`), None for any other text (the NEO's
    /// boot banner, the ATGM's firmware id).
    fn from_text(text: &str) -> Option<Antenna> {
        match text {
            "ANTENNA OK" => Some(Antenna::Ok),
            "ANTENNA OPEN" => Some(Antenna::Open),
            "ANTENNA SHORT" => Some(Antenna::Short),
            _ => None,
        }
    }
}

/// The sentences watched for an outage. RMC and GGA are tracked apart because they fail apart: a receiver holding
/// almanac but no fix keeps emitting GGA (quality 0) while RMC goes void, and a selection mistake can silence one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sentence {
    /// RMC, the fix.
    Rmc,
    /// GGA, the altitude and the signal quality.
    Gga,
}

impl Sentence {
    /// Both, in the order the sweep reports them.
    const ALL: [Sentence; 2] = [Sentence::Rmc, Sentence::Gga];

    /// The sentence's type as NMEA spells it.
    pub fn name(self) -> &'static str {
        match self {
            Sentence::Rmc => "RMC",
            Sentence::Gga => "GGA",
        }
    }

    /// The watched sentence of this type (the address after the talker), None for any other.
    fn from_kind(kind: &str) -> Option<Sentence> {
        Sentence::ALL.into_iter().find(|sentence| sentence.name() == kind)
    }

    /// Its slot in the receiver's per-sentence arrays.
    fn index(self) -> usize {
        self as usize
    }
}

/// The sky as one `<name>_sky.csv` row, summed over the systems: all integers, like hdop_cd.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SkyRow {
    /// Satellites in view, all systems (GSV).
    pub in_view: i32,
    /// Satellites used, all systems (GSA): a SUM, since PRN numbers repeat across systems (GPS 22 and BDS 22 were
    /// both in use on the bench).
    pub used: i32,
    /// The best system's fix mode (GSA: 1 none, 2 2D, 3 3D; 0 before any).
    pub mode: i32,
    /// The four strongest C/N0 in dB-Hz of all systems, strongest first, 0 where fewer were heard. Their mean is
    /// the pad yardstick: on the 2026-10-04 bench >= ~40 gave a usable fix in under a minute, ~35 was marginal (the
    /// passive patch), <= ~30 never fixed cold.
    pub cn0: [i32; 4],
    /// The antenna text's status.
    pub antenna: Antenna,
}

impl SkyRow {
    /// The row's eight columns in the stream's order: in_view, used, mode, cn0_1..cn0_4, antenna.
    pub fn columns(&self) -> [i32; 8] {
        let [first, second, third, fourth] = self.cn0;
        [
            self.in_view,
            self.used,
            self.mode,
            first,
            second,
            third,
            fourth,
            i32::from(self.antenna as u8),
        ]
    }
}

/// One `<name>.csv` row: the fix an RMC pushed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixRow {
    /// The position (lat, lon columns, written from the integers with 7 decimals).
    pub position: Position,
    /// The ground speed in knots, as RMC gives it (0 when the field is empty).
    pub speed_knots: f32,
    /// The course over ground in degrees (0 when the field is empty).
    pub course_deg: f32,
}

/// One `<name>_gga.csv` row: GGA's altitude and the signal quality behind the fix. GGA has a stream of its own so
/// each sentence keeps its real cadence and an outage in one shows as a gap in its stream alone; these are the
/// numbers that say whether a fix loss was the sky, the antenna or the receiver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GgaRow {
    /// The altitude above mean sea level in metres; 0 in a quality-only GGA (no fix yet).
    pub altitude_m: f32,
    /// The altitude above the first one heard, in metres; 0 in a quality-only GGA.
    pub elevation_m: f32,
    /// The fix quality: 0 none, 1 GPS, 2 DGPS.
    pub quality: i32,
    /// Satellites used in the fix (more is a better antenna or sky).
    pub satellites: i32,
    /// The HDOP in hundredths, from the decimal text (lower is better geometry).
    pub hdop_centi: i32,
}

/// A durable diagnostic the receiver raises (`<name>_events.csv` and the log). Display gives plan B's text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// A sentence went quiet past the outage gap. Guidance dead-reckons through a GNSS loss by design, so the
    /// trajectory looks normal and this is the only record that the receiver was dead.
    Lost {
        /// Which.
        sentence: Sentence,
        /// How long it had been quiet, in milliseconds.
        quiet_ms: i64,
    },
    /// A lost sentence arrived again.
    Back {
        /// Which.
        sentence: Sentence,
        /// How long it had been gone, in milliseconds.
        after_ms: i64,
    },
    /// The antenna status changed (the ATGM repeats it every burst; only a change is an event).
    Antenna {
        /// The new status.
        now: Antenna,
        /// The one before.
        was: Antenna,
    },
    /// The first sky row of the session.
    FirstSkyView(SkyRow),
    /// The diagnostics were switched on (the pad: before BOOSTING, and from DONE).
    SkyOn,
    /// The diagnostics were switched off for the flight.
    SkyOff,
}

impl fmt::Display for Event {
    /// Plan B's event texts.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Event::Lost { sentence, quiet_ms } => write!(formatter, "{} lost (quiet {quiet_ms} ms)", sentence.name()),
            Event::Back { sentence, after_ms } => write!(formatter, "{} back after {after_ms} ms", sentence.name()),
            Event::Antenna { now, was } => write!(formatter, "antenna {} (was {})", now.name(), was.name()),
            Event::FirstSkyView(row) => {
                let [first, second, third, fourth] = row.cn0;
                write!(
                    formatter,
                    "first sky view: {} in view, {} used, mode {}, C/N0 {first}/{second}/{third}/{fourth} dB-Hz, \
                     antenna {}",
                    row.in_view,
                    row.used,
                    row.mode,
                    row.antenna.name()
                )
            }
            Event::SkyOn => formatter.write_str("sky diagnostics on"),
            Event::SkyOff => formatter.write_str("sky diagnostics off for flight"),
        }
    }
}

/// Which of a driver's two sky switch frames to write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SkyFrame {
    /// The flight init's own selection, without the diagnostics: the flight output exactly as before they existed.
    Off,
    /// The selection with the diagnostics.
    On,
}

/// A driver's sky switch: the bytes its setup made, written whole at each edge of the flight window.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SkySwitch {
    /// The frame that switches the diagnostics off.
    pub off: Vec<u8>,
    /// The frame that switches them on.
    pub on: Vec<u8>,
}

impl SkySwitch {
    /// The frame to write for `frame`.
    pub fn frame(&self, frame: SkyFrame) -> &[u8] {
        match frame {
            SkyFrame::Off => &self.off,
            SkyFrame::On => &self.on,
        }
    }
}

/// What one line produced, for the firmware to publish in field order: the channels (a position, then the speed
/// and course; or the altitude, then the elevation), the rows, the events.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Update {
    /// The fix, when an RMC with a valid fix and both coordinates arrived (`position`).
    pub position: Option<Position>,
    /// The ground speed in m/s that went with it (`speed`, fix-gated; the airspeed governor's corrector).
    pub speed_ms: Option<f32>,
    /// The course in degrees, when the RMC carried one (`course`; the attitude filter's absolute yaw reference).
    pub course_deg: Option<f32>,
    /// The `<name>.csv` row.
    pub fix_row: Option<FixRow>,
    /// The altitude MSL in metres, when a GGA carried one (`altitude`).
    pub altitude_m: Option<f32>,
    /// The elevation above the GNSS ground zero in metres (`elevation`).
    pub elevation_m: Option<f32>,
    /// The `<name>_gga.csv` row: every GGA, with or without an altitude.
    pub gga_row: Option<GgaRow>,
    /// The events raised, in order.
    pub events: Vec<Event>,
}

/// What one tick produced (plan B `_sweeping`'s pass): the events in order (outages, the sky switch, the first sky
/// view), the frame to write, and the sky row when a burst has just ended.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Tick {
    /// The events raised, in order.
    pub events: Vec<Event>,
    /// The sky switch frame to write (and drain), when the flight window's edge was crossed.
    pub write: Option<SkyFrame>,
    /// The `<name>_sky.csv` row, when a burst has just ended.
    pub sky_row: Option<SkyRow>,
}

/// A field plan B's `int()`/`float()`/`degrees()` would raise on: the rest of the line is dropped, as plan B's run
/// loop drops a line that raises (what it already applied stays applied).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Malformed;

/// The talker a sentence's address names, `$GPGSV` -> `GP`: the GSV views are kept per talker.
type Talker = [u8; 2];

/// One talker's satellites in view, from its last WHOLE GSV group.
#[derive(Clone, Debug, PartialEq)]
struct View {
    /// The satellites in view the group announced.
    in_view: i32,
    /// Its four strongest C/N0, strongest first (fewer when fewer were tracked).
    strongest: Vec<i32>,
}

/// The receiver: plan B's Gnss task state, without the UART.
#[derive(Clone, Debug, PartialEq)]
pub struct Gnss {
    /// GSV talker -> its last whole group, in the order first heard.
    views: Vec<(Talker, View)>,
    /// GSA system (the NMEA 4.1 system id, else the talker) -> (fix mode, satellites used) from its last sentence.
    systems: Vec<(String, i32, i32)>,
    /// The talker of the GSV group being received, None before the first part 1.
    group_talker: Option<Talker>,
    /// The part the group needs next (0: no group in progress).
    group_next: i64,
    /// The C/N0 values the group has collected so far.
    group_cn0: Vec<i32>,
    /// The last antenna status heard.
    antenna: Antenna,
    /// A diagnostics sentence arrived that no sky row carries yet.
    pending: bool,
    /// When the latest one arrived, in microseconds.
    pending_us: u64,
    /// A sky row went out: only the first burst is evented.
    sky_seen: bool,
    /// The last arrival of RMC and GGA in microseconds (None: never), for the outage events.
    seen_us: [Option<u64>; 2],
    /// Whether each is currently out, so each edge fires once.
    absent: [bool; 2],
    /// The last RMC's status was A (a valid fix).
    fix: bool,
    /// GGA's fix quality: 0 none, 1 GPS, 2 DGPS.
    fix_quality: i32,
    /// GGA's satellites used.
    satellites: i32,
    /// GGA's HDOP.
    hdop: f32,
    /// Lines read (a liveness counter for probe).
    lines: u32,
    /// The GNSS ground zero: the first GGA altitude, so the elevation is offset-free.
    ground_m: Option<f32>,
    /// The diagnostics are on. The init ends with them on (the pad); a warm start into flight switches them off at
    /// its first tick.
    diagnosing: bool,
}

impl Default for Gnss {
    fn default() -> Gnss {
        Gnss::new()
    }
}

impl Gnss {
    /// A receiver as its setup leaves it: nothing heard, the diagnostics on.
    pub fn new() -> Gnss {
        Gnss {
            views: Vec::new(),
            systems: Vec::new(),
            group_talker: None,
            group_next: 0,
            group_cn0: Vec::new(),
            antenna: Antenna::Unknown,
            pending: false,
            pending_us: 0,
            sky_seen: false,
            seen_us: [None; 2],
            absent: [false; 2],
            fix: false,
            fix_quality: 0,
            satellites: 0,
            hdop: 0.0,
            lines: 0,
            ground_m: None,
            diagnosing: true,
        }
    }

    /// One line as the UART gave it (plan B's run loop body): count it, decode it, strip it, parse it, then sweep
    /// for outages (on every line, so a sentence that stopped is seen even while another flows).
    ///
    /// An empty read is no line but still sweeps. A line that is not UTF-8 (a noise byte) is dropped.
    pub fn line(&mut self, raw: &[u8], now_us: u64) -> Update {
        let mut update = Update::default();
        if !raw.is_empty() {
            self.lines = self.lines.wrapping_add(1);
            if let Ok(text) = std::str::from_utf8(raw) {
                update = self.parse(text.trim_matches(is_space), now_us);
            }
        }
        self.sweep_into(now_us, &mut update.events);
        update
    }

    /// Parse one NMEA sentence (plan B `_parse`): RMC -> the fix, GGA -> the altitude and quality, GSV/GSA/TXT ->
    /// the sky. A sentence without '$' or with a bad checksum is ignored. A malformed field drops the rest of the
    /// line, keeping what was already applied, as plan B does.
    pub fn parse(&mut self, line: &str, now_us: u64) -> Update {
        let mut update = Update::default();
        // a malformed field ends the line; what it applied before stays, as in plan B's run loop
        let _ = self.parse_into(line, now_us, &mut update);
        update
    }

    /// [`parse`](Gnss::parse)'s body, stopping at the first malformed field.
    fn parse_into(&mut self, line: &str, now_us: u64, update: &mut Update) -> Result<(), Malformed> {
        if !line.starts_with('$') || !checksum_ok(line) {
            return Ok(());
        }
        let body = line.split('*').next().unwrap_or_default();
        let fields: Vec<&str> = body.split(',').collect();
        let kind = fields[0].get(3..).unwrap_or_default(); // drop '$' and the 2-character talker
        if let Some(sentence) = Sentence::from_kind(kind) {
            self.mark(sentence, now_us, &mut update.events);
        }
        match kind {
            "RMC" if fields.len() > 9 => self.rmc(&fields, update),
            "GGA" if fields.len() > 9 => self.gga(&fields, update),
            "GSV" | "GSA" | "TXT" => self.sky(kind, &fields, now_us, &mut update.events),
            _ => Ok(()),
        }
    }

    /// RMC: the fix flag, then (fix-gated) the position, the speed and the course, and the fix row.
    fn rmc(&mut self, fields: &[&str], update: &mut Update) -> Result<(), Malformed> {
        self.fix = fields[2] == "A"; // A valid, V void
        let latitude = degrees_e7(fields[3], fields[4])?;
        let longitude = degrees_e7(fields[5], fields[6])?;
        let (true, Some(latitude_e7), Some(longitude_e7)) = (self.fix, latitude, longitude) else {
            return Ok(());
        };
        let position = Position {
            latitude_e7,
            longitude_e7,
        };
        update.position = Some(position);
        let speed_knots = optional_float(fields[7])?;
        let course_deg = optional_float(fields[8])?;
        update.speed_ms = Some(speed_knots * KNOTS_TO_METRES_PER_SECOND);
        if !fields[8].is_empty() {
            update.course_deg = Some(course_deg);
        }
        update.fix_row = Some(FixRow {
            position,
            speed_knots,
            course_deg,
        });
        Ok(())
    }

    /// GGA: the signal quality (parsed even with no altitude yet), the altitude and elevation when present, and the
    /// row either way (a quality-only GGA is still worth recording).
    fn gga(&mut self, fields: &[&str], update: &mut Update) -> Result<(), Malformed> {
        self.fix_quality = optional_int(fields[6])?;
        self.satellites = optional_int(fields[7])?;
        self.hdop = optional_float(fields[8])?;
        let (altitude_m, elevation_m) = if fields[9].is_empty() {
            (0.0, 0.0)
        } else {
            let altitude_m = python_float(fields[9])?;
            update.altitude_m = Some(altitude_m);
            let ground_m = *self.ground_m.get_or_insert(altitude_m); // the first altitude is the ground
            let elevation_m = altitude_m - ground_m;
            update.elevation_m = Some(elevation_m);
            (altitude_m, elevation_m)
        };
        update.gga_row = Some(GgaRow {
            altitude_m,
            elevation_m,
            quality: self.fix_quality,
            satellites: self.satellites,
            hdop_centi: if fields[8].is_empty() {
                0
            } else {
                hundredths(fields[8])?
            },
        });
        Ok(())
    }

    /// Fold one diagnostics sentence into the sky and restart the burst's quiet clock (plan B `_sky`).
    ///
    /// GSA comes once per system: the ATGM sends one for GPS and one for BDS (the NMEA 4.1 system id closes each),
    /// the NEO-6M a single one, so a system is keyed by that id, else by the talker. Every GSV and GSA, and an
    /// antenna text, restart the clock; any other text is not the sky and is not a burst.
    fn sky(&mut self, kind: &str, fields: &[&str], now_us: u64, events: &mut Vec<Event>) -> Result<(), Malformed> {
        match kind {
            "GSV" if fields.len() > 3 => self.gsv(fields)?,
            "GSA" if fields.len() > 14 => {
                let used = fields[3..15].iter().filter(|prn| !prn.is_empty()).count() as i32; // twelve PRN slots
                let system = fields
                    .get(18)
                    .copied()
                    .unwrap_or_else(|| fields[0].get(1..3).unwrap_or_default());
                let mode = optional_int(fields[2])?;
                match self.systems.iter_mut().find(|(name, _mode, _used)| name == system) {
                    Some(entry) => *entry = (system.to_owned(), mode, used),
                    None => self.systems.push((system.to_owned(), mode, used)),
                }
            }
            "TXT" if fields.len() > 4 => {
                let Some(antenna) = Antenna::from_text(fields[4]) else {
                    return Ok(());
                };
                if antenna != self.antenna {
                    events.push(Event::Antenna {
                        now: antenna,
                        was: self.antenna,
                    });
                    self.antenna = antenna;
                }
            }
            _ => return Ok(()),
        }
        self.pending = true;
        self.pending_us = now_us;
        Ok(())
    }

    /// One GSV sentence (plan B `_gsv`): collect its C/N0; the group's last part replaces that talker's view.
    ///
    /// A talker reports its satellites in view as a group `<parts>,<part>,<in view>`, then four satellites as PRN,
    /// elevation, azimuth, C/N0, and NMEA 4.1 (the ATGM) appends a signal id after the last. The C/N0 sits at
    /// fields 7, 11, 15, 19 whatever the count, and the signal id at a multiple of 4, so it is never read as one. An
    /// empty C/N0 is a satellite in view but not tracked: no signal to rank (not 0). A group replaces the view only
    /// when it arrived WHOLE and in order: a part lost to a bad checksum would otherwise blank that half of the sky
    /// until the next burst.
    fn gsv(&mut self, fields: &[&str]) -> Result<(), Malformed> {
        let talker = talker(fields[0]);
        let part = python_int(fields[2]).map(i64::from)?;
        if part == 1 {
            self.group_talker = Some(talker);
            self.group_cn0.clear();
        } else if part != self.group_next || self.group_talker != Some(talker) {
            self.group_next = 0; // a part out of order, or of another talker: this group cannot be whole
            return Ok(());
        }
        self.group_next = part.saturating_add(1);
        for cn0 in fields.iter().skip(7).step_by(4) {
            if !cn0.is_empty() {
                self.group_cn0.push(python_int(cn0)?);
            }
        }
        if part == i64::from(python_int(fields[1])?) {
            let in_view = optional_int(fields[3])?;
            let mut strongest = self.group_cn0.clone();
            strongest.sort_unstable_by(|first, second| second.cmp(first));
            strongest.truncate(4);
            let view = View { in_view, strongest };
            match self.views.iter_mut().find(|(known, _view)| *known == talker) {
                Some(entry) => entry.1 = view,
                None => self.views.push((talker, view)),
            }
            self.group_next = 0;
        }
        Ok(())
    }

    /// The sky as the `<name>_sky.csv` row (plan B `_sky_row`): each talker's last whole group and each system's
    /// last GSA count; all zeros before the first diagnostics.
    pub fn sky_row(&self) -> SkyRow {
        let mut strongest: Vec<i32> = self
            .views
            .iter()
            .flat_map(|(_talker, view)| view.strongest.iter().copied())
            .collect();
        strongest.sort_unstable_by(|first, second| second.cmp(first));
        strongest.resize(strongest.len().max(4), 0);
        SkyRow {
            in_view: self
                .views
                .iter()
                .fold(0, |sum, (_talker, view)| sum.saturating_add(view.in_view)),
            used: self
                .systems
                .iter()
                .fold(0, |sum, (_system, _mode, used)| sum.saturating_add(*used)),
            mode: self
                .systems
                .iter()
                .fold(0, |best, (_system, mode, _used)| best.max(*mode)),
            cn0: [strongest[0], strongest[1], strongest[2], strongest[3]],
            antenna: self.antenna,
        }
    }

    /// Push the sky row once a burst is over (plan B `_settle`): on the first tick that finds diagnostics no row
    /// carries yet AND none of them in the last half second.
    ///
    /// A burst's sentences are closer than that and bursts are ~10 s apart, so every tick inside a burst waits and
    /// the first one after it takes the whole burst, the ATGM's antenna text (after the epoch's RMC) included.
    /// Ending a burst on a sentence instead could not work for both modules (the ATGM ends on the TXT, the NEO on a
    /// GSV). Returns the row and, for the first burst of the session, its event; the burst is spent either way.
    pub fn settle(&mut self, now_us: u64) -> Option<(SkyRow, Option<Event>)> {
        if !self.pending || since(now_us, self.pending_us) < BURST_QUIET_US {
            return None;
        }
        self.pending = false;
        let row = self.sky_row();
        let first = (!self.sky_seen).then_some(Event::FirstSkyView(row));
        self.sky_seen = true;
        Some((row, first))
    }

    /// Keep the diagnostics to the pad (plan B `_sky_window`): on before BOOSTING and again from DONE, off from
    /// BOOSTING until DONE. Only a change writes, one write per edge; the frame is the driver's, made at setup.
    /// Returns the frame and its event at an edge, None while the window holds.
    pub fn sky_window(&mut self, stage: Stage) -> Option<(SkyFrame, Event)> {
        let diagnosing = !matches!(stage, Stage::Boosting | Stage::Gliding | Stage::Landing);
        if diagnosing == self.diagnosing {
            return None;
        }
        self.diagnosing = diagnosing;
        Some(if diagnosing {
            (SkyFrame::On, Event::SkyOn)
        } else {
            (SkyFrame::Off, Event::SkyOff)
        })
    }

    /// Note that `sentence` arrived, and event the way back from an outage (plan B `_mark`).
    fn mark(&mut self, sentence: Sentence, now_us: u64, events: &mut Vec<Event>) {
        let index = sentence.index();
        if self.absent[index] {
            self.absent[index] = false;
            let after_ms = since(now_us, self.seen_us[index].unwrap_or(now_us)).div_euclid(1000);
            events.push(Event::Back { sentence, after_ms });
        }
        self.seen_us[index] = Some(now_us);
    }

    /// Event every sentence that went quiet past the outage gap, once per outage (plan B `_sweep`). It needs a clock
    /// that runs when nothing arrives, so the run loop calls it on every line AND the tick on its own clock: a
    /// receiver that goes completely silent (power lost, connector out) parses nothing.
    pub fn sweep(&mut self, now_us: u64) -> Vec<Event> {
        let mut events = Vec::new();
        self.sweep_into(now_us, &mut events);
        events
    }

    /// [`sweep`](Gnss::sweep) into a caller's event list.
    fn sweep_into(&mut self, now_us: u64, events: &mut Vec<Event>) {
        for sentence in Sentence::ALL {
            let index = sentence.index();
            let Some(seen_us) = self.seen_us[index] else {
                continue; // never heard: nothing to lose
            };
            let quiet_us = since(now_us, seen_us);
            if !self.absent[index] && quiet_us > SENTENCE_GAP_US {
                self.absent[index] = true;
                events.push(Event::Lost {
                    sentence,
                    quiet_ms: quiet_us.div_euclid(1000),
                });
            }
        }
    }

    /// One pass of the 1 s tick (plan B `_sweeping`'s loop body): the outage sweep, the sky window for the stage
    /// now (read on every tick, not once), then the burst-ending settle.
    pub fn tick(&mut self, now_us: u64, stage: Stage) -> Tick {
        let mut tick = Tick::default();
        self.sweep_into(now_us, &mut tick.events);
        if let Some((frame, event)) = self.sky_window(stage) {
            tick.write = Some(frame);
            tick.events.push(event);
        }
        if let Some((row, first)) = self.settle(now_us) {
            tick.events.extend(first);
            tick.sky_row = Some(row);
        }
        tick
    }

    /// The last RMC had a valid fix.
    pub fn fix(&self) -> bool {
        self.fix
    }

    /// GGA's fix quality: 0 none, 1 GPS, 2 DGPS.
    pub fn fix_quality(&self) -> i32 {
        self.fix_quality
    }

    /// GGA's satellites used in the fix: watch it rise with a better antenna.
    pub fn satellites(&self) -> i32 {
        self.satellites
    }

    /// GGA's horizontal dilution of precision (lower is better geometry).
    pub fn hdop(&self) -> f32 {
        self.hdop
    }

    /// The mean of the four strongest C/N0 in dB-Hz: the "good enough to launch" yardstick (see [`SkyRow::cn0`]).
    pub fn cn0_mean(&self) -> f32 {
        self.sky_row().cn0.iter().map(|cn0| *cn0 as f32).sum::<f32>() / 4.0
    }

    /// Lines read so far.
    pub fn lines(&self) -> u32 {
        self.lines
    }

    /// Whether the diagnostics are on.
    pub fn diagnosing(&self) -> bool {
        self.diagnosing
    }
}

/// The signed time from `then_us` to `now_us` in microseconds (plan B `ticks_diff`, without its 2^30 wrap).
fn since(now_us: u64, then_us: u64) -> i64 {
    now_us.wrapping_sub(then_us) as i64
}

/// The talker of a sentence address, `$GPGSV` -> `GP` (bytes 1 and 2; the address is ASCII and six long here).
fn talker(address: &str) -> Talker {
    let bytes = address.as_bytes();
    [
        bytes.get(1).copied().unwrap_or_default(),
        bytes.get(2).copied().unwrap_or_default(),
    ]
}

/// MicroPython's whitespace for `strip()` and `int()`/`float()`: space and \t \n \v \f \r.
fn is_space(character: char) -> bool {
    character == ' ' || ('\t'..='\r').contains(&character)
}

/// XOR of the bytes: the NMEA checksum.
fn xor(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |checksum, byte| checksum ^ byte)
}

/// Verify a sentence's `*hh` XOR checksum over the characters between '$' and the last '*' (plan B
/// `checksum_ok`).
///
/// False on a missing '*', and when the (at most two) characters after it are no hex number. They are read as
/// Python's `int(text, 16)` reads them: surrounding whitespace and a sign allowed, either case, so `*5` (one digit)
/// and `*0b` pass when they name the checksum. A sentence that is not ASCII fails (see the crate notes).
pub fn checksum_ok(sentence: &str) -> bool {
    if !sentence.is_ascii() {
        return false;
    }
    let Some(star) = sentence.rfind('*') else {
        return false; // no `*hh`: unverifiable, the same policy as the host's control/gps.py
    };
    let computed = sentence.as_bytes().get(1..star).map_or(0, xor);
    let digits = &sentence[star + 1..(star + 3).min(sentence.len())];
    hex_number(digits) == Some(i32::from(computed))
}

/// Python's `int(text, 16)` for the checksum's one or two characters: None where it raises.
fn hex_number(text: &str) -> Option<i32> {
    let text = text.trim_matches(is_space);
    let (negative, digits) = match text.as_bytes().first() {
        Some(b'-') => (true, &text[1..]),
        Some(b'+') => (false, &text[1..]),
        _ => (false, text),
    };
    if digits.is_empty() || !digits.bytes().all(|digit| digit.is_ascii_hexdigit()) {
        return None;
    }
    let value = i32::from_str_radix(digits, 16).ok()?;
    Some(if negative { -value } else { value })
}

/// Python's `int(text)`: surrounding whitespace and a sign allowed; Malformed where it raises (or past an i32).
fn python_int(text: &str) -> Result<i32, Malformed> {
    text.trim_matches(is_space).parse().map_err(|_error| Malformed)
}

/// Python's `float(text)` as f32: surrounding whitespace, a sign, an exponent, `inf` and `nan` allowed; Malformed
/// where it raises (and on an underscore, which Python takes between digits; see the crate notes).
fn python_float(text: &str) -> Result<f32, Malformed> {
    text.trim_matches(is_space).parse().map_err(|_error| Malformed)
}

/// `int(field) if field else 0`.
fn optional_int(text: &str) -> Result<i32, Malformed> {
    if text.is_empty() { Ok(0) } else { python_int(text) }
}

/// `float(field) if field else 0.0`.
fn optional_float(text: &str) -> Result<f32, Malformed> {
    if text.is_empty() { Ok(0.0) } else { python_float(text) }
}

/// A plain decimal ('0.9', '1.15', '-0.5', '.5') in hundredths, truncated toward zero: hdop_cd from its text, so no
/// binary fraction loses a unit (see the crate notes). Malformed for anything else (an exponent, `nan`).
fn hundredths(text: &str) -> Result<i32, Malformed> {
    let text = text.trim_matches(is_space);
    let (negative, unsigned) = match text.as_bytes().first() {
        Some(b'-') => (true, &text[1..]),
        Some(b'+') => (false, &text[1..]),
        _ => (false, text),
    };
    let (whole, fraction) = unsigned.split_once('.').unwrap_or((unsigned, ""));
    let digits = |part: &str| part.bytes().all(|digit| digit.is_ascii_digit());
    if whole.len() + fraction.len() == 0 || !digits(whole) || !digits(fraction) {
        return Err(Malformed);
    }
    let whole: i64 = if whole.is_empty() {
        0
    } else {
        whole.parse().map_err(|_error| Malformed)?
    };
    let mut hundredths = whole.checked_mul(100).ok_or(Malformed)?;
    for (place, digit) in fraction.bytes().take(2).enumerate() {
        hundredths += i64::from(digit - b'0') * if place == 0 { 10 } else { 1 };
    }
    let signed = if negative { -hundredths } else { hundredths };
    i32::try_from(signed).map_err(|_error| Malformed)
}

/// Fraction digits of a minute kept: beyond nine (1e-9 minute, 1.7e-11 degree) they cannot move a 1e-7 degree unit,
/// and nine keep the integer arithmetic inside an i64 (99.999999999 minutes x 1e9 x 1e7 x 2 is 2e18).
const MINUTE_DECIMALS: usize = 9;

/// An NMEA ddmm.mmmm (or dddmm.mmmm) field and its hemisphere as signed 1e-7 degree units (plan B `degrees`, in
/// integers).
///
/// The degrees are the digits before the last two before the dot, the minutes the rest; the units are rounded to
/// the nearest (a tie away from zero). S and W are negative, any other hemisphere (even empty) positive, as in plan
/// B. Minutes of 60 or more are taken as they come, as plan B takes them.
///
/// Returns Ok(None) for an empty field (no fix yet), Ok(units), or Malformed when the field is not digits with a
/// dot, has fewer than three digits before the dot, or names more than an i32 of units (past 214.7 degrees).
pub fn degrees_e7(value: &str, hemisphere: &str) -> Result<Option<i32>, Malformed> {
    if value.is_empty() {
        return Ok(None);
    }
    let (before, after) = value.split_once('.').ok_or(Malformed)?;
    let digits = |part: &str| part.bytes().all(|digit| digit.is_ascii_digit());
    if before.len() < 3 || !digits(before) || !digits(after) {
        return Err(Malformed);
    }
    let (degrees, minutes) = before.split_at(before.len() - 2);
    let fraction = &after[..after.len().min(MINUTE_DECIMALS)];
    let scale = 10_i64.pow(fraction.len() as u32);
    let mut scaled_minutes: i64 = minutes.parse().map_err(|_error| Malformed)?;
    scaled_minutes *= scale;
    if !fraction.is_empty() {
        scaled_minutes += fraction.parse::<i64>().map_err(|_error| Malformed)?;
    }
    // units of the minutes: scaled_minutes * 1e7 / (60 * scale), rounded half up, without overflowing an i64
    let denominator = 60 * scale;
    let minute_units = (scaled_minutes * 10_000_000 * 2 + denominator) / (2 * denominator);
    let degree_units = degrees
        .parse::<i64>()
        .ok()
        .and_then(|degrees| degrees.checked_mul(10_000_000))
        .ok_or(Malformed)?;
    let units = degree_units
        .checked_add(minute_units)
        .and_then(|units| i32::try_from(units).ok())
        .ok_or(Malformed)?;
    Ok(Some(if matches!(hemisphere, "S" | "W") { -units } else { units }))
}

/// Wrap a command body in `$...*hh\r\n` with its XOR checksum (plan B `nmea`), for the PCAS/PMTK/PUBX config
/// sentences. The checksum covers the body's bytes (plan B counted characters: the same for ASCII, which every
/// command is).
pub fn nmea(body: &str) -> Vec<u8> {
    format!("${body}*{:02X}\r\n", xor(body.as_bytes())).into_bytes()
}

#[cfg(test)]
mod tests;
