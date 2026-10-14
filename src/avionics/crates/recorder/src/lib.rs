//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder (doc/specs/avionics-port.md "Storage design": "Producer side", "The ring_writer", "The
//! ring_saver" Decode, "Row, log and index text", "CC tees and the HTTP exits"), the port of src/glider
//! recorder.py with the UART to the Luckfox replaced by the flash ring:
//!
//! ```text
//! producers (any core) -- binary records --> PSRAM SPSC ring per thread --> Writer::step --> flash_ring::Ring
//!   Telemetry::push, Recorder::log             (+ misc ring, try_lock)        | verbatim, into chunks
//!   Recorder::index_session                                                   +--> CC tees (log, tlm)
//!
//! flash block --> Decoder::block --> wire lines `[@<routing>@]{OPEN};<payload>;<CLOSE>\n` (ring_saver, /ring, tools)
//!
//! flash_ring::Ring --> Saver::step: Walk (in sequence order, up to writer_position) --> Decoder --> batches
//!   --> Fs, the card's root: <session>/recordings/<session>_<file>, <session>/recordings/recorder.log,
//!       <session>/recordings/session.csv + sessions.csv (layout); fsync
//!   --> saver_position (the eraser erases behind it)
//! ```
//!
//! * [`Registry`]: the streams, built before any thread spawns and then fixed.
//! * [`Recorder`]: the shared state, one per boot (`&'static` on the board): the producer rings, the tees, the
//!   counters, `writer_position`, the flight flag, and [`Recorder::log`], [`Recorder::index_session`],
//!   [`Recorder::stats`], [`Recorder::inspect`] and [`Recorder::set_stats_ms`] (plan B's `update`).
//! * [`Telemetry`]: a stream's producer side, `due`/`push`, owned by its thread.
//! * [`Writer`]: the ring_writer's loop body, [`Writer::step`], generic over [`flash_ring::Flash`].
//! * [`Decoder`]: a flash block back to the lines recorderd wrote, for the ring_saver and the tools.
//! * [`Saver`]: the ring_saver's loop body, [`Saver::step`], over a [`Walk`] of the ring and the card's [`Fs`]
//!   ([`StdFs`] on the board and the host): the flash emptied to the SD in batches, `saver_position` published only
//!   once a batch is fsynced.
//! * [`layout`]: the one place that maps (session, kind) to a directory on the card, and the global index's name;
//!   the saver, the firmware's camera and the host tests all ask it.
//!
//! Nothing on a producer's path or in the writer prints a record: the records are binary from the producer to
//! flash, and only the decoder and a tee's drain make text, off the flight path (invariant 4).
//!
//! Portable and host-tested; it runs on the ESP32-P4 (rv32imafc) too, so it holds no 64-bit atomic (the P4 has
//! none) and no mutex on a producer's path. It needs std for the thread-local ring binding and the locks that
//! guard the shared `misc` ring and a tee's consumer side, neither on a hot path.

mod byte_ring;
mod decoder;
mod fs;
pub mod layout;
mod published;
mod record;
mod recorder;
mod registry;
mod relay;
mod rows;
mod saver;
mod tee;
mod telemetry;
mod writer;

pub use decoder::{BlockError, BlockSummary, DecodedLine, Decoder};
pub use fs::{Fs, StdFs};
pub use published::Stats;
pub use recorder::{AttachError, Attachment, Buffers, Clock, Identity, Inspection, Overflow, Recorder, Settings, Wake};
pub use recorder_wire::Cell;
pub use registry::{Registry, RegistryError, Stream, StreamId};
pub use relay::{BEHIND_BLOCKS, Relay};
pub use rows::{LogOnce, Rows};
pub use saver::{Ahead, BATCH_BLOCKS, BATCH_FILES, BATCH_MS, SaveError, Saved, Saver, TAIL_BYTES, WRITE_BYTES, Walk};
pub use tee::TeeBatch;
pub use telemetry::Telemetry;
pub use writer::{ANCHOR_AFTER_US, HEADER_REPEAT_BYTES, MERGE_HORIZON_US, Progress, TICK_US, Writer};
