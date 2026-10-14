//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stream registry (doc/specs/avionics-port.md "Producer side"): every telemetry file a boot writes, with
//! its destination file name, its column list and its decimation. Bring-up builds it before any thread spawns
//! and hands it to the [`crate::Recorder`], which never changes it again: a producer names its stream by a 16-bit
//! id, and the record it queues carries the file name, never the session (the block header names that).
//!
//! Stream 0 is the shared session index, the bare `session.csv`: the ring_saver files its rows both in the
//! session's directory and in the global index every boot appends to. Every other stream's file prints as
//! `<session>_<file>`.
//!
//! The registry keeps each stream's column list, `uptime;<field>;...`, built once. A HEADER record carries it, so
//! that every flash block reads without the registry, and the writer repeats it about once per MiB by copying
//! these bytes: nobody formats a header on the flight path.
//!
//! A stream whose records could never reach the SD whole is refused here, at bring-up, rather than losing every
//! row later: a file name the ring_saver would not file (`[A-Za-z0-9._-]{3,64}`: its rows would land in
//! recorder.log), a field that would break the column list, more fields than a row counts, or a HEADER record
//! longer than a chunk holds.

use std::string::String;
use std::vec::Vec;

use recorder_wire::{SESSION_INDEX, SESSION_INDEX_HEADER};

use crate::record::{LONGEST, header_length, is_file_name, padded};

/// A stream's id: its index in the registry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct StreamId(pub(crate) u16);

impl StreamId {
    /// The shared session index, `session.csv`: always stream 0.
    pub const SESSION_INDEX: StreamId = StreamId(0);
}

/// One telemetry stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Stream {
    /// The destination file name: the stream's file without the session prefix, or the bare `session.csv`.
    file: String,
    /// The column list, `uptime;<field>;...`: the body of its HEADER records.
    columns: Vec<u8>,
    /// `> 0` the stream's own decimation window in us; `0` the global `recorder.telemetry_ms`, read at use;
    /// `< 0` never decimated (events).
    decimate_us: i32,
    /// Whether its producer wakes the writer for each record, so the record skips the tick (sequencer,
    /// separation, checkpoint, events, session.csv).
    durable: bool,
}

impl Stream {
    /// The destination file name its records carry.
    pub fn file(&self) -> &str {
        &self.file
    }

    /// Its column list, `uptime;<field>;...`, without a line end.
    pub fn columns(&self) -> &[u8] {
        &self.columns
    }

    /// Its decimation setting: own window (> 0), the global one (0), or none (< 0).
    pub fn decimate_us(&self) -> i32 {
        self.decimate_us
    }

    /// Whether its records wake the writer.
    pub fn durable(&self) -> bool {
        self.durable
    }
}

/// Why a stream cannot be registered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegistryError {
    /// The file name breaks the ring_saver's rule, `[A-Za-z0-9._-]{3,64}`.
    Name,
    /// A field name is empty, or holds `;`, a space or a byte outside printable ASCII: it would break the
    /// column list.
    Field,
    /// More fields than a row record counts (255), or a HEADER record longer than a chunk holds (4032 bytes):
    /// none of the stream's rows, or not its header, could ever reach flash.
    Wide,
    /// The file is registered already: two streams would splice one file.
    Duplicate,
    /// Every 16-bit id is taken.
    Full,
}

/// Whether `field` can be a column.
fn column(field: &str) -> bool {
    !field.is_empty() && field.bytes().all(|byte| byte.is_ascii_graphic() && byte != b';')
}

/// The streams of one boot.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Registry {
    /// The streams, by id.
    streams: Vec<Stream>,
}

impl Default for Registry {
    /// [`Registry::new`].
    fn default() -> Registry {
        Registry::new()
    }
}

impl Registry {
    /// A registry holding only stream 0, the session index `session.csv`, durable and never decimated.
    pub fn new() -> Registry {
        let index = Stream {
            file: SESSION_INDEX.into(),
            columns: SESSION_INDEX_HEADER.into(),
            decimate_us: -1,
            durable: true,
        };
        Registry { streams: vec![index] }
    }

    /// Registers the stream `file` with these fields (its columns after `uptime`).
    ///
    /// Args:
    /// * `file` - the destination file name, without the session prefix (`imu_lsm6dso32.csv`).
    /// * `fields` - the column names after `uptime`.
    /// * `decimate_us` - the stream's own decimation window in us when positive, 0 for the global
    ///   `recorder.telemetry_ms`, negative for none.
    /// * `durable` - whether each record wakes the writer (spec "Producer side": the durable streams).
    ///
    /// Returns the stream's id. Errors: `Name` for a file name the ring_saver would not file, `Field` for a field
    /// that would break the column list, `Wide` for more than 255 fields or a HEADER record no chunk holds,
    /// `Duplicate` when the file is taken, `Full` past 65 536 streams.
    pub fn add(
        &mut self,
        file: &str,
        fields: &[&str],
        decimate_us: i32,
        durable: bool,
    ) -> Result<StreamId, RegistryError> {
        if !is_file_name(file.as_bytes()) {
            return Err(RegistryError::Name);
        }
        if !fields.iter().all(|field| column(field)) {
            return Err(RegistryError::Field);
        }
        let mut columns = Vec::new();
        recorder_wire::header(&mut columns, fields);
        if fields.len() > usize::from(u8::MAX) || padded(header_length(file, &columns)) > LONGEST {
            return Err(RegistryError::Wide);
        }
        if self.find(file.as_bytes()).is_some() {
            return Err(RegistryError::Duplicate);
        }
        let id = u16::try_from(self.streams.len()).map_err(|_| RegistryError::Full)?;
        self.streams.push(Stream {
            file: file.into(),
            columns,
            decimate_us,
            durable,
        });
        Ok(StreamId(id))
    }

    /// The stream `id` names, or None when this registry has no such stream.
    pub fn stream(&self, id: StreamId) -> Option<&Stream> {
        self.streams.get(usize::from(id.0))
    }

    /// The stream whose records go to `file`, or None. A linear search: the writer asks only for HEADER records,
    /// a stream's first and its repeats.
    pub(crate) fn find(&self, file: &[u8]) -> Option<StreamId> {
        self.streams
            .iter()
            .position(|stream| stream.file.as_bytes() == file)
            .map(|index| StreamId(index as u16))
    }

    /// The streams, by id.
    pub(crate) fn streams(&self) -> &[Stream] {
        &self.streams
    }
}

#[cfg(test)]
mod tests {
    use super::{Registry, RegistryError, StreamId};
    use std::string::String;
    use std::vec::Vec;

    #[test]
    fn streams_keep_their_file_and_columns_and_the_index_is_stream_0() {
        let mut registry = Registry::new();
        let index = registry.stream(StreamId::SESSION_INDEX).unwrap();
        assert_eq!(index.file(), "session.csv");
        assert_eq!(
            index.columns(),
            b"uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon"
        );
        assert!(index.durable() && index.decimate_us() < 0);
        let imu = registry
            .add("imu_lsm6dso32.csv", &["ax", "ay", "az"], 0, false)
            .unwrap();
        let events = registry.add("flight_events.csv", &["event"], -1, true).unwrap();
        assert_eq!((imu, events), (StreamId(1), StreamId(2)));
        let imu = registry.stream(imu).unwrap();
        assert_eq!(
            (imu.file(), imu.columns()),
            ("imu_lsm6dso32.csv", b"uptime;ax;ay;az".as_slice())
        );
        assert_eq!((imu.decimate_us(), imu.durable()), (0, false));
        assert_eq!(registry.find(b"flight_events.csv"), Some(StreamId(2)));
        assert_eq!(registry.find(b"session.csv"), Some(StreamId::SESSION_INDEX));
        // negative: what was never registered
        assert_eq!(registry.find(b"health.csv"), None);
        assert_eq!(registry.stream(StreamId(3)), None);
        assert_eq!(Registry::default(), Registry::new());
    }

    #[test]
    fn a_name_the_saver_would_not_file_is_refused() {
        let mut registry = Registry::new();
        assert!(registry.add(&"x".repeat(64), &[], 0, false).is_ok(), "64 bytes");
        assert!(registry.add("a.b", &[], 0, false).is_ok(), "3 bytes");
        for file in [&"y".repeat(65), "ab", "a/b.csv", "a@b.csv", "a b.csv", "\u{e9}.csv", ""] {
            assert_eq!(registry.add(file, &[], 0, false), Err(RegistryError::Name), "{file:?}");
        }
    }

    #[test]
    fn a_field_that_breaks_the_columns_is_refused() {
        let mut registry = Registry::new();
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
    fn a_stream_whose_header_or_rows_no_chunk_holds_is_refused() {
        let mut registry = Registry::new();
        let names: Vec<String> = (0..256).map(|index| std::format!("f{index}")).collect();
        let fields: Vec<&str> = names.iter().map(String::as_str).collect();
        assert_eq!(
            registry.add("many.csv", &fields, 0, false),
            Err(RegistryError::Wide),
            "a row counts its cells in a byte"
        );
        assert!(registry.add("many.csv", &fields[..255], 0, false).is_ok(), "255 fit");
        // 16 + 8 (wide.csv) + `uptime;` (7) + the fields and their ';': 4001 bytes of fields fit a 4032-byte chunk
        let name = "x".repeat(99);
        let wide: Vec<&str> = std::iter::repeat_n(name.as_str(), 41).collect(); // 41 x 100 - 1 = 4099
        assert_eq!(registry.add("wide.csv", &wide, 0, false), Err(RegistryError::Wide));
        let fitting = "x"; // 4001 - 40 x 100 bytes
        let mut last = wide[..40].to_vec();
        last.push(fitting);
        assert!(registry.add("wide.csv", &last, 0, false).is_ok(), "exactly a chunk");
    }

    #[test]
    fn a_file_is_registered_once() {
        let mut registry = Registry::new();
        registry.add("health.csv", &["temp"], 0, false).unwrap();
        assert_eq!(
            registry.add("health.csv", &["load"], 0, false),
            Err(RegistryError::Duplicate)
        );
        assert_eq!(
            registry.add("session.csv", &["x"], 0, false),
            Err(RegistryError::Duplicate),
            "the index is stream 0's"
        );
    }

    #[test]
    fn ids_stop_at_sixteen_bits() {
        let mut registry = Registry::new();
        registry.add("last.csv", &[], 0, false).unwrap();
        let filler = registry.streams[1].clone();
        registry.streams.resize(usize::from(u16::MAX), filler);
        assert_eq!(registry.add("fits.csv", &[], 0, false), Ok(StreamId(u16::MAX)));
        assert_eq!(registry.add("one-more.csv", &[], 0, false), Err(RegistryError::Full));
    }
}
