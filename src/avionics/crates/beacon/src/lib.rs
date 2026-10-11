//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recovery beacon (doc/specs/avionics-port.md M11b, M20), shared by its two ends:
//!
//! - the glider at DONE (firmware/src/beacon.rs): its own access point `<BOARD>_glider` on channel 3, its page read
//!   from its databoard at each request;
//! - the remote recorder (receiver/, an ESP32-C6 on the glider's recorder UART): `<BOARD>_beacon` on a channel three
//!   up, its page from the glider's last [`Report`]. The glider writes the report as an ordinary recorder stream
//!   (`beacon.csv`, [`FIELDS`]) every few seconds; the recorder files it as any stream and replays its last row, after
//!   a restart from its card. The report sets that radio too ([`Radio`]): an empty name keeps it off.
//!
//! One page for both ([`page`]), so a searcher's phone shows the same window whichever beacon it joined; the report's
//! row ([`Report::cells`], [`Report::parse`]); the captive portal's DNS answer ([`dns_answer`]); and, with the `esp`
//! feature, the access point's glue ([`esp`]).

use std::fmt::Write as _;

use recorder_wire::Cell;

#[cfg(feature = "esp")]
pub mod esp;

/// The access point's address, the one the ESP-IDF examples and most beacons use: a phone opens it as it is.
pub const ADDRESS: [u8; 4] = [192, 168, 4, 1];

/// The report's stream file: its routing is `<session>_beacon.csv`.
pub const FILE: &str = "beacon.csv";

/// The report's columns after `uptime` ([`Report::cells`]).
pub const FIELDS: [&str; 18] = [
    "ssid",
    "password",
    "channel",
    "power_dbm",
    "on_s",
    "off_s",
    "latitude",
    "longitude",
    "fix_age_s",
    "pad_latitude",
    "pad_longitude",
    "elevations",
    "agl",
    "voltage_mv",
    "current_ma",
    "battery_mah",
    "backlog_blocks",
    "backlog_s",
];

/// A 1S LiPo's open-circuit voltage (mV) against its charge (%), for the page's charge estimate: the usual table,
/// coarse, under load it reads low.
const LIPO_CHARGE: [(f32, f32); 9] = [
    (3300.0, 0.0),
    (3500.0, 10.0),
    (3600.0, 20.0),
    (3700.0, 35.0),
    (3800.0, 50.0),
    (3900.0, 65.0),
    (4000.0, 80.0),
    (4100.0, 90.0),
    (4200.0, 100.0),
];

/// Above this the INA226 reads no 1S LiPo (the bench's 5 V USB): no charge estimate.
const LIPO_CEILING_MV: f32 = 4350.0;

/// A DNS answer's lifetime, seconds: short, the beacon is the only name server while it lasts.
const DNS_TTL_S: u32 = 60;

/// What the remote beacon's radio does, as the glider sets it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Radio {
    /// The access point's name; empty keeps the beacon off.
    pub ssid: String,
    /// Its WPA2 password; empty for an open access point.
    pub password: String,
    /// Its channel.
    pub channel: u8,
    /// Its TX cap, dBm.
    pub power_dbm: u8,
    /// Seconds on in each cycle.
    pub on_s: u32,
    /// Seconds silent after each [`Radio::on_s`] (the battery's share); 0 never silent.
    pub off_s: u32,
}

/// A point in 1e-7 degrees, as the GNSS reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    /// Latitude, 1e-7 degrees.
    pub latitude: i32,
    /// Longitude, 1e-7 degrees.
    pub longitude: i32,
}

impl Point {
    /// Its degrees, (latitude, longitude).
    pub fn degrees(self) -> (f64, f64) {
        (f64::from(self.latitude) * 1e-7, f64::from(self.longitude) * 1e-7)
    }
}

/// One source's height.
#[derive(Clone, Debug, PartialEq)]
pub struct Height {
    /// The source's name (its config entry).
    pub source: String,
    /// Metres; None when the source never read.
    pub metres: Option<f32>,
    /// Seconds since it read.
    pub age_s: u64,
}

/// The battery, where an INA226 measures it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Battery {
    /// Its voltage, mV.
    pub millivolts: f32,
    /// Its draw, mA.
    pub milliamps: Option<f32>,
    /// Its capacity (the INA226 entry's `battery_mah`), for the time left.
    pub capacity_mah: Option<f32>,
}

/// The recording still to copy from the flash to the card.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Backlog {
    /// Blocks to copy.
    pub blocks: u64,
    /// The seconds of recording they hold, when known.
    pub seconds: Option<f32>,
}

/// What the page shows, its ages as at the moment it was read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    /// The last good fix, whatever its age (a fix lost in the trees is still the best guess), and that age, s.
    pub fix: Option<(Point, u64)>,
    /// The launch pad: a map link even before any fix.
    pub pad: Option<Point>,
    /// Each source's elevation above the pad: the GNSS, each barometer.
    pub elevations: Vec<Height>,
    /// Each laser's height above the ground: how high the glider hangs, should it land in a tree.
    pub agl: Vec<Height>,
    /// The battery.
    pub battery: Option<Battery>,
    /// The recording still to copy.
    pub backlog: Option<Backlog>,
}

impl Snapshot {
    /// The same, `seconds` older: a report received that long ago.
    pub fn aged(mut self, seconds: u64) -> Snapshot {
        if let Some((_, age)) = &mut self.fix {
            *age += seconds;
        }
        for height in self.elevations.iter_mut().chain(self.agl.iter_mut()) {
            height.age_s += seconds;
        }
        self
    }
}

/// The glider's report to the remote beacon: one beacon.csv row.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    /// What the remote beacon's radio does.
    pub radio: Radio,
    /// What its page shows.
    pub snapshot: Snapshot,
}

impl Report {
    /// The two height lists as their cells hold them: `source=metres@age` items joined by `,` (`gnss=12.3@4`), a
    /// source that never read as `source=`.
    pub fn height_texts(&self) -> [String; 2] {
        let text = |heights: &[Height]| {
            let mut text = String::new();
            for (index, height) in heights.iter().enumerate() {
                if index > 0 {
                    text.push(',');
                }
                let _ = write!(text, "{}=", height.source);
                if let Some(metres) = height.metres {
                    let _ = write!(text, "{metres:.1}@{}", height.age_s);
                }
            }
            text
        };
        [text(&self.snapshot.elevations), text(&self.snapshot.agl)]
    }

    /// The row's cells in [`FIELDS`]' order, the height lists from [`Report::height_texts`].
    pub fn cells<'a>(&'a self, heights: &'a [String; 2]) -> [Cell<'a>; FIELDS.len()] {
        let radio = &self.radio;
        let snapshot = &self.snapshot;
        let point = |point: Option<Point>| {
            point.map_or([Cell::None, Cell::None], |point| {
                [Cell::E7(point.latitude), Cell::E7(point.longitude)]
            })
        };
        let [latitude, longitude] = point(snapshot.fix.map(|(point, _)| point));
        let [pad_latitude, pad_longitude] = point(snapshot.pad);
        let battery = snapshot.battery;
        [
            Cell::Text(&radio.ssid),
            Cell::Text(&radio.password),
            Cell::I32(i32::from(radio.channel)),
            Cell::I32(i32::from(radio.power_dbm)),
            Cell::I64(i64::from(radio.on_s)),
            Cell::I64(i64::from(radio.off_s)),
            latitude,
            longitude,
            snapshot.fix.map_or(Cell::None, |(_, age)| Cell::I64(age as i64)),
            pad_latitude,
            pad_longitude,
            Cell::Text(&heights[0]),
            Cell::Text(&heights[1]),
            battery.map_or(Cell::None, |battery| Cell::F32(battery.millivolts)),
            battery
                .and_then(|battery| battery.milliamps)
                .map_or(Cell::None, Cell::F32),
            battery
                .and_then(|battery| battery.capacity_mah)
                .map_or(Cell::None, Cell::F32),
            snapshot
                .backlog
                .map_or(Cell::None, |backlog| Cell::I64(backlog.blocks as i64)),
            snapshot
                .backlog
                .and_then(|backlog| backlog.seconds)
                .map_or(Cell::None, Cell::F32),
        ]
    }

    /// A beacon.csv row read back from its payload, `<uptime>;<cells>` as the recorder printed [`Report::cells`];
    /// None for anything else: the header, a row of another width, a cell that does not read.
    pub fn parse(payload: &[u8]) -> Option<Report> {
        let text = std::str::from_utf8(payload).ok()?;
        let cells: Vec<&str> = text.split(';').collect();
        let [uptime, cells @ ..] = cells.as_slice() else {
            return None;
        };
        uptime.parse::<u64>().ok()?;
        let [
            ssid,
            password,
            channel,
            power_dbm,
            on_s,
            off_s,
            latitude,
            longitude,
            fix_age_s,
            pad_latitude,
            pad_longitude,
            elevations,
            agl,
            voltage_mv,
            current_ma,
            battery_mah,
            backlog_blocks,
            backlog_s,
        ] = cells
        else {
            return None;
        };
        /// A cell's text; None for `None`.
        fn optional(cell: &str) -> Option<&str> {
            (cell != "None").then_some(cell)
        }
        let point = |latitude: &str, longitude: &str| -> Option<Option<Point>> {
            match (optional(latitude), optional(longitude)) {
                (Some(latitude), Some(longitude)) => Some(Some(Point {
                    latitude: parse_e7(latitude)?,
                    longitude: parse_e7(longitude)?,
                })),
                (None, None) => Some(None),
                _ => None,
            }
        };
        let number = |cell: &str| -> Option<Option<f32>> { optional(cell).map(str::parse).transpose().ok() };
        let fix = match (point(latitude, longitude)?, optional(fix_age_s)) {
            (Some(point), Some(age)) => Some((point, age.parse().ok()?)),
            (None, None) => None,
            _ => return None,
        };
        let battery = number(voltage_mv)?.map(|millivolts| -> Option<Battery> {
            Some(Battery {
                millivolts,
                milliamps: number(current_ma)?,
                capacity_mah: number(battery_mah)?,
            })
        });
        let backlog = optional(backlog_blocks)
            .map(|blocks| -> Option<Backlog> {
                Some(Backlog {
                    blocks: blocks.parse().ok()?,
                    seconds: number(backlog_s)?,
                })
            })
            .map_or(Some(None), |backlog| backlog.map(Some))?;
        Some(Report {
            radio: Radio {
                ssid: ssid.to_string(),
                password: password.to_string(),
                channel: channel.parse().ok()?,
                power_dbm: power_dbm.parse().ok()?,
                on_s: on_s.parse().ok()?,
                off_s: off_s.parse().ok()?,
            },
            snapshot: Snapshot {
                fix,
                pad: point(pad_latitude, pad_longitude)?,
                elevations: parse_heights(elevations)?,
                agl: parse_heights(agl)?,
                battery: battery.map_or(Some(None), |battery| battery.map(Some))?,
                backlog,
            },
        })
    }
}

/// Degrees with up to 7 decimals (`-80.3917950`, as recorder_wire prints an E7 cell) in 1e-7 degrees, exactly.
fn parse_e7(text: &str) -> Option<i32> {
    let (negative, digits) = match text.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, text),
    };
    let (whole, fraction) = digits.split_once('.').unwrap_or((digits, ""));
    if whole.is_empty() || fraction.len() > 7 || !(whole.bytes().chain(fraction.bytes())).all(|b| b.is_ascii_digit()) {
        return None;
    }
    let mut units = i64::from(whole.parse::<u32>().ok()?) * 10_000_000;
    units += format!("{fraction:0<7}").parse::<i64>().ok()?;
    i32::try_from(if negative { -units } else { units }).ok()
}

/// A height list as [`Report::height_texts`] wrote it; None when an item does not read.
fn parse_heights(text: &str) -> Option<Vec<Height>> {
    if text.is_empty() {
        return Some(Vec::new());
    }
    text.split(',')
        .map(|item| {
            let (source, reading) = item.split_once('=')?;
            let (metres, age_s) = match reading.split_once('@') {
                Some((metres, age)) => (Some(metres.parse().ok()?), age.parse().ok()?),
                None if reading.is_empty() => (None, 0),
                None => return None,
            };
            Some(Height {
                source: source.to_string(),
                metres,
                age_s,
            })
        })
        .collect()
}

/// The beacon's page: `title` (the access point's name), what `snapshot` holds, and a `note` under the title (the
/// remote beacon's: how old the glider's report is). It refreshes itself every 5 s.
pub fn page(title: &str, snapshot: &Snapshot, note: Option<&str>) -> String {
    let mut page = String::with_capacity(4096);
    let _ = write!(
        page,
        "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" \
         content=\"width=device-width,initial-scale=1\"><meta http-equiv=\"refresh\" content=\"5\">\
         <title>{name}</title><style>body{{font-family:sans-serif;margin:16px;font-size:18px}}\
         td{{padding:4px 12px 4px 0}}h2{{margin:20px 0 6px}}</style></head><body><h1>{name}</h1>",
        name = escape(title)
    );
    if let Some(note) = note {
        let _ = write!(page, "<p><i>{}</i></p>", escape(note));
    }
    page.push_str("<h2>Where</h2>");
    match snapshot.fix {
        Some((point, age)) => {
            let (latitude, longitude) = point.degrees();
            let _ = write!(
                page,
                "<p>{latitude:.6}, {longitude:.6} ({age} s ago)<br><a href=\"https://maps.google.com/?q={latitude:.6},\
                 {longitude:.6}\">open in Google Maps</a></p>"
            );
        }
        None => page.push_str("<p>no GNSS fix this boot</p>"),
    }
    if let Some(pad) = snapshot.pad {
        let (latitude, longitude) = pad.degrees();
        let _ = write!(
            page,
            "<p>launch pad: {latitude:.6}, {longitude:.6}<br><a href=\"https://maps.google.com/?q={latitude:.6},\
             {longitude:.6}\">open the pad in Google Maps</a></p>"
        );
    }
    page.push_str("<h2>How high</h2><table>");
    for (heights, label) in [
        (&snapshot.elevations, "above the pad"),
        (&snapshot.agl, "above the ground (laser)"),
    ] {
        for height in heights {
            match height.metres {
                Some(metres) => {
                    let _ = write!(
                        page,
                        "<tr><td>{}</td><td>{metres:.1} m {label}</td><td>{} s ago</td></tr>",
                        escape(&height.source),
                        height.age_s
                    );
                }
                None => {
                    let _ = write!(page, "<tr><td>{}</td><td>no reading</td></tr>", escape(&height.source));
                }
            }
        }
    }
    page.push_str("</table>");
    if let Some(battery) = snapshot.battery {
        let _ = write!(page, "<h2>Battery</h2><p>{:.2} V", battery.millivolts / 1000.0);
        if let Some(milliamps) = battery.milliamps {
            let _ = write!(page, ", {milliamps:.0} mA");
        }
        match charge(battery.millivolts) {
            Some(percent) => {
                let _ = write!(page, ", about {percent:.0} % (1S LiPo)");
                if let (Some(capacity), Some(milliamps)) =
                    (battery.capacity_mah, battery.milliamps.filter(|draw| *draw > 1.0))
                {
                    let _ = write!(page, ", about {:.1} h left", capacity * percent / 100.0 / milliamps);
                }
            }
            None => page.push_str(" (no 1S LiPo: on USB?)"),
        }
        page.push_str("</p>");
    }
    if let Some(backlog) = snapshot.backlog {
        let _ = write!(
            page,
            "<h2>Recording</h2><p>{} blocks to copy to the card",
            backlog.blocks
        );
        if let Some(seconds) = backlog.seconds {
            let _ = write!(page, " ({seconds:.0} s of recording)");
        }
        page.push_str(": keep the glider powered until 0</p>");
    }
    page.push_str("</body></html>");
    page
}

/// A 1S LiPo's charge (%) at `millivolts`, interpolated in [`LIPO_CHARGE`]; None above [`LIPO_CEILING_MV`].
pub fn charge(millivolts: f32) -> Option<f32> {
    if millivolts > LIPO_CEILING_MV {
        return None;
    }
    let (lowest, _) = LIPO_CHARGE[0];
    if millivolts <= lowest {
        return Some(0.0);
    }
    for pair in LIPO_CHARGE.windows(2) {
        let [(low_mv, low), (high_mv, high)] = [pair[0], pair[1]];
        if millivolts <= high_mv {
            return Some(low + (high - low) * (millivolts - low_mv) / (high_mv - low_mv));
        }
    }
    Some(100.0)
}

/// The answer to one DNS `query` (RFC 1035): its id and question echoed, an A record of [`ADDRESS`] for an A query,
/// no record for any other type; None for a datagram that is no single-question query.
pub fn dns_answer(query: &[u8]) -> Option<Vec<u8>> {
    if query.len() < 12 || query[2] & 0x80 != 0 || u16::from_be_bytes([query[4], query[5]]) != 1 {
        return None;
    }
    // the question's name: labels up to the zero length
    let mut end = 12;
    loop {
        let label = usize::from(*query.get(end)?);
        end += 1 + label;
        if label == 0 {
            break;
        }
        if label & 0xc0 != 0 {
            return None; // a pointer has no place in a question
        }
    }
    let question = query.get(12..end + 4)?;
    let address_query = u16::from_be_bytes([question[question.len() - 4], question[question.len() - 3]]) == 1;
    let mut answer = Vec::with_capacity(12 + question.len() + 16);
    answer.extend_from_slice(&query[..2]); // the id
    answer.extend_from_slice(&[0x81, 0x80]); // a response, recursion desired and available, no error
    answer.extend_from_slice(&[0, 1, 0, u8::from(address_query), 0, 0, 0, 0]);
    answer.extend_from_slice(question);
    if address_query {
        answer.extend_from_slice(&[0xc0, 0x0c, 0, 1, 0, 1]); // the question's name, type A, class IN
        answer.extend_from_slice(&DNS_TTL_S.to_be_bytes());
        answer.extend_from_slice(&[0, 4]);
        answer.extend_from_slice(&ADDRESS);
    }
    Some(answer)
}

/// `text` safe inside HTML.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

#[cfg(test)]
mod tests {
    use super::{Backlog, Battery, FIELDS, Height, Point, Radio, Report, Snapshot, charge, dns_answer, page, parse_e7};

    /// A report as the glider at DONE writes one.
    fn report() -> Report {
        Report {
            radio: Radio {
                ssid: "TMS-7F_beacon".into(),
                password: String::new(),
                channel: 6,
                power_dbm: 20,
                on_s: 60,
                off_s: 60,
            },
            snapshot: Snapshot {
                fix: Some((
                    Point {
                        latitude: 255_143_790,
                        longitude: -803_917_950,
                    },
                    4,
                )),
                pad: Some(Point {
                    latitude: 255_143_000,
                    longitude: -803_918_000,
                }),
                elevations: vec![
                    Height {
                        source: "gnss".into(),
                        metres: Some(12.5),
                        age_s: 4,
                    },
                    Height {
                        source: "baro_icp10111".into(),
                        metres: None,
                        age_s: 0,
                    },
                ],
                agl: vec![Height {
                    source: "laser_agl_l1x".into(),
                    metres: Some(2.0),
                    age_s: 1,
                }],
                battery: Some(Battery {
                    millivolts: 3812.5,
                    milliamps: Some(180.0),
                    capacity_mah: Some(450.0),
                }),
                backlog: Some(Backlog {
                    blocks: 12,
                    seconds: Some(3.5),
                }),
            },
        }
    }

    /// The payload the recorder prints for `report` at `uptime_us`.
    fn printed(report: &Report, uptime_us: u64) -> Vec<u8> {
        let heights = report.height_texts();
        let mut line = Vec::new();
        recorder_wire::row(&mut line, uptime_us, &report.cells(&heights));
        line
    }

    #[test]
    fn a_report_reads_back_as_it_was_written() {
        let report = report();
        let line = printed(&report, 60_000_123);
        assert_eq!(Report::parse(&line), Some(report), "{}", String::from_utf8_lossy(&line));
        // an empty one: the beacon off, nothing known
        let off = Report {
            radio: Radio {
                channel: 6,
                power_dbm: 20,
                ..Radio::default()
            },
            ..Report::default()
        };
        assert_eq!(Report::parse(&printed(&off, 1)), Some(off));
    }

    #[test]
    fn what_is_no_report_does_not_read() {
        let mut header = Vec::new();
        recorder_wire::header(&mut header, &FIELDS);
        assert_eq!(Report::parse(&header), None, "the header");
        let line = printed(&report(), 5);
        let text = String::from_utf8(line).unwrap();
        assert_eq!(Report::parse(format!("{text};x").as_bytes()), None, "a cell more");
        assert_eq!(
            Report::parse(text.replace(";6;", ";six;").as_bytes()),
            None,
            "a channel that is no number"
        );
        assert_eq!(
            Report::parse(text.replace("25.5143790", "None").as_bytes()),
            None,
            "a latitude without its longitude"
        );
        assert_eq!(
            Report::parse(text.replace("gnss=12.5@4", "gnss=12.5").as_bytes()),
            None,
            "a height without its age"
        );
        assert_eq!(Report::parse(b""), None);
    }

    #[test]
    fn degrees_read_back_to_the_unit() {
        assert_eq!(parse_e7("-80.3917950"), Some(-803_917_950));
        assert_eq!(parse_e7("0.0000001"), Some(1));
        assert_eq!(parse_e7("25.5"), Some(255_000_000));
        assert_eq!(parse_e7("7"), Some(70_000_000));
        // negative: too many decimals, no digits, a sign alone, out of range
        assert_eq!(parse_e7("1.00000001"), None);
        assert_eq!(parse_e7(".5"), None);
        assert_eq!(parse_e7("-"), None);
        assert_eq!(parse_e7("300.0"), None);
        assert_eq!(parse_e7("1e5"), None);
    }

    #[test]
    fn an_older_report_shows_older_ages() {
        let aged = report().snapshot.aged(10);
        assert_eq!(aged.fix.map(|(_, age)| age), Some(14));
        assert_eq!(aged.elevations[0].age_s, 14);
        assert_eq!(aged.agl[0].age_s, 11);
    }

    #[test]
    fn the_page_shows_every_part() {
        let snapshot = report().snapshot;
        let shown = page("TMS-7F_beacon", &snapshot, Some("the glider's report, 3 s ago"));
        for part in [
            "<title>TMS-7F_beacon</title>",
            "<i>the glider's report, 3 s ago</i>",
            "25.514379, -80.391795 (4 s ago)",
            "https://maps.google.com/?q=25.514379,-80.391795",
            "launch pad: 25.514300, -80.391800",
            "<td>gnss</td><td>12.5 m above the pad</td><td>4 s ago</td>",
            "<td>baro_icp10111</td><td>no reading</td>",
            "<td>laser_agl_l1x</td><td>2.0 m above the ground (laser)</td>",
            "3.81 V, 180 mA, about 52 % (1S LiPo), about 1.3 h left",
            "12 blocks to copy to the card (4 s of recording)",
        ] {
            assert!(shown.contains(part), "{part} in {shown}");
        }
        // negative: nothing known is said so, and a name cannot inject markup
        let empty = page("<x>", &Snapshot::default(), None);
        assert!(empty.contains("no GNSS fix this boot") && !empty.contains("launch pad") && !empty.contains("Battery"));
        assert!(empty.contains("&lt;x&gt;") && !empty.contains("<x>"));
    }

    #[test]
    fn a_lipo_charge_follows_the_table() {
        assert_eq!(charge(3300.0), Some(0.0));
        assert_eq!(charge(3750.0), Some(42.5));
        assert_eq!(charge(4200.0), Some(100.0));
        assert_eq!(charge(4300.0), Some(100.0));
        // negative: the bench's 5 V is no LiPo
        assert_eq!(charge(5000.0), None);
    }

    #[test]
    fn every_name_resolves_to_the_beacon() {
        // id 0x1234, one question: "a.b" type A class IN
        let query = [
            0x12, 0x34, 0x01, 0x00, 0, 1, 0, 0, 0, 0, 0, 0, 1, b'a', 1, b'b', 0, 0, 1, 0, 1,
        ];
        let answer = dns_answer(&query).unwrap();
        assert_eq!(&answer[..2], &[0x12, 0x34]);
        assert_eq!(&answer[6..8], &[0, 1], "one answer");
        assert!(answer.ends_with(&[0, 4, 192, 168, 4, 1]));
        // an AAAA query: echoed, no record
        let mut aaaa = query;
        aaaa[18] = 28;
        assert_eq!(&dns_answer(&aaaa).unwrap()[6..8], &[0, 0]);
        // negative: a response, two questions, a cut name
        let mut response = query;
        response[2] |= 0x80;
        assert_eq!(dns_answer(&response), None);
        let mut two = query;
        two[5] = 2;
        assert_eq!(dns_answer(&two), None);
        assert_eq!(dns_answer(&query[..14]), None);
    }
}
