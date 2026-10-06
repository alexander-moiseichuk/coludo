//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder's producer side and its flash writer (doc/specs/avionics-port.md "Storage design": "Producer
//! side", "The flash writer", "Row, log and index text", "CC tees and the HTTP exits"), the port of src/glider
//! recorder.py with the UART to the Luckfox replaced by the flash ring:
//!
//! ```text
//! producers (any core) -- binary records --> PSRAM SPSC ring per thread --> Writer::step --> flash_ring::Ring
//!   Telemetry::push, Recorder::log             (+ misc ring, try_lock)        | format, tee, wrap, cut
//!                                                                             +--> CC tees (log, tlm)
//! ```
//!
//! * [`Registry`]: the streams, built before any thread spawns and then fixed.
//! * [`Recorder`]: the shared state, one per boot (`&'static` on the board): the producer rings, the tees, the
//!   counters, and [`Recorder::log`], [`Recorder::index_session`], [`Recorder::stats`], [`Recorder::inspect`]
//!   and [`Recorder::set_stats_ms`] (plan B's `update`).
//! * [`Telemetry`]: a stream's producer side, `due`/`push`, owned by its thread.
//! * [`Writer`]: the flash writer's loop body, [`Writer::step`], generic over [`flash_ring::Flash`].
//!
//! Portable and host-tested; it runs on the ESP32-P4 (rv32imafc) too, so it holds no 64-bit atomic (the P4 has
//! none) and no mutex on a producer's path. It needs std for the thread-local ring binding and the locks that
//! guard the shared `misc` ring and a tee's consumer side, neither on a hot path.

mod byte_ring;
mod published;
mod record;
mod recorder;
mod registry;
mod tee;
mod telemetry;
mod writer;

pub use published::Stats;
pub use recorder::{AttachError, Attachment, Buffers, Clock, Identity, Inspection, Overflow, Recorder, Settings};
pub use recorder_wire::Cell;
pub use registry::{Registry, RegistryError, Stream, StreamId};
pub use tee::TeeBatch;
pub use telemetry::Telemetry;
pub use writer::{ANCHOR_AFTER_US, CUT_AGE_US, FRAME_BYTES, MERGE_HORIZON_US, Progress, Writer};
