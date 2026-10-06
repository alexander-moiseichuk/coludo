//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stream registry (doc/specs/avionics-port.md "Producer side"): every telemetry file a boot writes, with
//! its routing, header and decimation. Bring-up builds it before any thread spawns and hands it to the
//! [`crate::Recorder`], which never changes it again: a producer names its stream by a 16-bit id, and the
//! writer looks the rest up without a lock.
//!
//! Stream 0 is the shared session index, routed bare as `session.csv` with no session prefix, so that every
//! boot appends to one file. Every other stream routes as `<session>_<file>`.
//!
//! A routing must pass the SD mover's rule, `[A-Za-z0-9._-]{3,64}`: the mover files any other line in
//! recorder.log, so a stream with a bad name would lose every row. The registry refuses it at bring-up instead,
//! and likewise a stream whose rows or header could never be recorded: more fields than a row record counts,
//! or a header line longer than a frame holds.

use std::string::String;
use std::vec::Vec;

use recorder_wire::{SESSION_INDEX, SESSION_INDEX_HEADER};

use crate::writer::{FRAME_BYTES, wrapped_bytes};

/// The shortest routing the mover files (recorderd's rule, a routing of at least 3 bytes).
const ROUTING_SHORTEST: usize = 3;

/// The longest routing the mover files.
const ROUTING_LONGEST: usize = 64;

/// A stream's id: its index in the registry, the `stream` field of its ring records.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct StreamId(pub(crate) u16);

impl StreamId {
    /// The shared session index, `session.csv`: always stream 0.
    pub const SESSION_INDEX: StreamId = StreamId(0);
}

/// One telemetry stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Stream {
    /// `<session>_<file>`, or the bare `session.csv`.
    routing: String,
    /// The CSV header line, `uptime;<field>;...`, built once.
    header: Vec<u8>,
    /// `> 0` the stream's own decimation window in us; `0` the global `recorder.telemetry_ms`, read at use;
    /// `< 0` never decimated (events).
    decimate_us: i32,
    /// Whether each record cuts a frame right after it (sequencer, separation, checkpoint, events, session.csv).
    durable: bool,
}

impl Stream {
    /// The routing its lines carry.
    pub fn routing(&self) -> &str {
        &self.routing
    }

    /// Its CSV header line, without the line end.
    pub fn header(&self) -> &[u8] {
        &self.header
    }

    /// Its decimation setting: own window (> 0), the global one (0), or none (< 0).
    pub fn decimate_us(&self) -> i32 {
        self.decimate_us
    }

    /// Whether its records cut a frame.
    pub fn durable(&self) -> bool {
        self.durable
    }
}

/// Why a stream cannot be registered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegistryError {
    /// The session prefix is empty or holds a byte no routing may, or the routing breaks the mover's rule.
    Name,
    /// A field name is empty, or holds `;`, a space or a byte outside printable ASCII: it would break the
    /// header's columns.
    Field,
    /// More fields than a row record counts (255), or a header line longer than a frame holds: none of the
    /// stream's rows, or not its header, could ever reach flash.
    Wide,
    /// The routing is registered already: two streams would splice one file.
    Duplicate,
    /// Every 16-bit id is taken.
    Full,
}

/// Whether `routing` passes the mover's rule.
fn routable(routing: &str) -> bool {
    (ROUTING_SHORTEST..=ROUTING_LONGEST).contains(&routing.len())
        && routing
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

/// Whether `field` can be a header column.
fn column(field: &str) -> bool {
    !field.is_empty() && field.bytes().all(|byte| byte.is_ascii_graphic() && byte != b';')
}

/// The streams of one boot.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Registry {
    /// The session prefix every stream's routing starts with.
    session: String,
    /// The streams, by id.
    streams: Vec<Stream>,
}

impl Registry {
    /// A registry for `session` (recorder_wire's `session_prefix`) holding only stream 0, the session index.
    ///
    /// Errors: `Name` when `session` is empty or holds a byte outside `[A-Za-z0-9._-]`.
    pub fn new(session: &str) -> Result<Registry, RegistryError> {
        let allowed = session
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'));
        if session.is_empty() || !allowed {
            return Err(RegistryError::Name);
        }
        let index = Stream {
            routing: SESSION_INDEX.into(),
            header: SESSION_INDEX_HEADER.into(),
            decimate_us: -1,
            durable: true,
        };
        Ok(Registry {
            session: session.into(),
            streams: vec![index],
        })
    }

    /// Registers the stream `<session>_<file>` with these header fields (after `uptime`).
    ///
    /// `decimate_us` is the stream's own decimation window in us when positive, 0 for the global
    /// `recorder.telemetry_ms`, and negative for none. `durable` makes each of its records cut a frame.
    ///
    /// Errors: `Name` when the routing breaks the mover's rule, `Field` for a field that would break the header,
    /// `Wide` for more than 255 fields or a header that no frame holds, `Duplicate` when the routing is taken,
    /// `Full` past 65 536 streams.
    pub fn add(
        &mut self,
        file: &str,
        fields: &[&str],
        decimate_us: i32,
        durable: bool,
    ) -> Result<StreamId, RegistryError> {
        let routing = std::format!("{}_{file}", self.session);
        if !routable(&routing) {
            return Err(RegistryError::Name);
        }
        if !fields.iter().all(|field| column(field)) {
            return Err(RegistryError::Field);
        }
        let mut header = Vec::new();
        recorder_wire::header(&mut header, fields);
        if fields.len() > usize::from(u8::MAX) || wrapped_bytes(Some(&routing), header.len()) > FRAME_BYTES {
            return Err(RegistryError::Wide);
        }
        if self.streams.iter().any(|stream| stream.routing == routing) {
            return Err(RegistryError::Duplicate);
        }
        let id = u16::try_from(self.streams.len()).map_err(|_| RegistryError::Full)?;
        self.streams.push(Stream {
            routing,
            header,
            decimate_us,
            durable,
        });
        Ok(StreamId(id))
    }

    /// The session prefix.
    pub fn session(&self) -> &str {
        &self.session
    }

    /// The stream `id` names, or None when this registry has no such stream.
    pub fn stream(&self, id: StreamId) -> Option<&Stream> {
        self.streams.get(usize::from(id.0))
    }
}

#[cfg(test)]
mod tests {
    use super::{Registry, RegistryError, StreamId};

    #[test]
    fn streams_route_under_the_session_and_the_index_routes_bare() {
        let mut registry = Registry::new("000123").unwrap();
        let index = registry.stream(StreamId::SESSION_INDEX).unwrap();
        assert_eq!(index.routing(), "session.csv");
        assert_eq!(
            index.header(),
            b"uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon"
        );
        assert!(index.durable() && index.decimate_us() < 0);
        let imu = registry
            .add("imu_lsm6dso32.csv", &["ax", "ay", "az"], 0, false)
            .unwrap();
        let events = registry.add("flight_events.csv", &["event"], -1, true).unwrap();
        assert_eq!((imu, events), (StreamId(1), StreamId(2)));
        let imu = registry.stream(imu).unwrap();
        assert_eq!(imu.routing(), "000123_imu_lsm6dso32.csv");
        assert_eq!(imu.header(), b"uptime;ax;ay;az");
        assert_eq!((imu.decimate_us(), imu.durable()), (0, false));
        assert_eq!(registry.streams.len(), 3);
        assert_eq!(registry.session(), "000123");
        assert_eq!(registry.stream(StreamId(3)), None);
    }

    #[test]
    fn a_name_the_mover_would_misfile_is_refused() {
        for session in ["", "a/b", "a b", "lab@el", "caf\u{e9}"] {
            assert_eq!(Registry::new(session), Err(RegistryError::Name), "{session:?}");
        }
        let mut registry = Registry::new("hitl-f15").unwrap();
        assert!(
            registry.add(&"x".repeat(64 - 9), &[], 0, false).is_ok(),
            "64 bytes with the prefix"
        );
        for file in [&"x".repeat(64 - 8), "a/b.csv", "a@b.csv", "a b.csv", "\u{e9}.csv"] {
            assert_eq!(registry.add(file, &[], 0, false), Err(RegistryError::Name), "{file:?}");
        }
    }

    #[test]
    fn a_field_that_breaks_the_columns_is_refused() {
        let mut registry = Registry::new("000001").unwrap();
        for field in ["", "a;b", "a b", "a\tb", "\u{e9}"] {
            assert_eq!(
                registry.add("s.csv", &["ok", field], 0, false),
                Err(RegistryError::Field),
                "{field:?}"
            );
        }
        assert!(registry.add("s.csv", &["ok", "dynamic_pressure"], 0, false).is_ok());
    }

    #[test]
    fn a_stream_no_row_or_header_of_could_be_recorded_is_refused() {
        let mut registry = Registry::new("000001").unwrap();
        let names: Vec<String> = (0..256).map(|index| std::format!("f{index}")).collect();
        let fields: Vec<&str> = names.iter().map(String::as_str).collect();
        assert_eq!(
            registry.add("many.csv", &fields, 0, false),
            Err(RegistryError::Wide),
            "a row counts its cells in a byte"
        );
        assert!(registry.add("many.csv", &fields[..255], 0, false).is_ok(), "255 fit");
        // routing 000001_wide.csv (15) + 2, `uptime;` (7), the wrapper (22) and the line end: 16 337 bytes of
        // fields and their `;` fit a 16 KiB frame
        let name = "x".repeat(100);
        let wide: Vec<&str> = std::iter::repeat_n(name.as_str(), 162).collect(); // 162 x 101 - 1 = 16 361
        assert_eq!(registry.add("wide.csv", &wide, 0, false), Err(RegistryError::Wide));
        let fitting = "x".repeat(16_337 - 161 * 101);
        let mut last = wide[..161].to_vec();
        last.push(&fitting);
        assert!(registry.add("wide.csv", &last, 0, false).is_ok(), "exactly a frame");
    }

    #[test]
    fn a_routing_is_registered_once() {
        let mut registry = Registry::new("000001").unwrap();
        registry.add("health.csv", &["temp"], 0, false).unwrap();
        assert_eq!(
            registry.add("health.csv", &["load"], 0, false),
            Err(RegistryError::Duplicate)
        );
    }

    #[test]
    fn ids_stop_at_sixteen_bits() {
        let mut registry = Registry::new("s").unwrap();
        registry.add("last.csv", &[], 0, false).unwrap();
        let filler = registry.streams[1].clone();
        registry.streams.resize(usize::from(u16::MAX), filler);
        assert_eq!(registry.add("fits.csv", &[], 0, false), Ok(StreamId(u16::MAX)));
        assert_eq!(registry.add("one-more.csv", &[], 0, false), Err(RegistryError::Full));
    }
}
