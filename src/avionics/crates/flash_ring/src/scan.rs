//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The bring-up scan (doc/specs/avionics-port.md, "Bring-up: the scan, the positions and the session-start
//! record"). It reads the 56-byte header at each of the 6144 block starts (about 0.15 s, estimated) and classifies
//! each slot into the [`BlockTable`] the three ring tasks share. It runs in the ring_writer thread before the first
//! append, so the rest of boot goes on and the producers queue in PSRAM meanwhile.
//!
//! One header per block is enough to classify it, because a block is one erase sector and the writer programs its
//! header first. What the header cannot show (a cut erase that left an erased header over dirty bytes) the writer
//! catches with its full read of a block before it opens it.
//!
//! The ring positions in the `cfg` NVS are hints, written about once a second: the scan is the truth for what is
//! in flash and where the writer goes. A hint only keeps the block sequence from going back. The previous boot
//! may have opened blocks that are no longer in flash (saved and erased, or a header a cut tore), and a sequence
//! number is never given twice.
//!
//! A hint is trusted only within [`HINT_REACH`] of the newest block in flash. One beyond it is corrupt, or was
//! written in another build's encoding (a reboot in flight that came back on another image reads the positions
//! the other image left). Taken as it is, it would push the numbering up to u32::MAX, where no block can open.

use crate::header::{BlockHeader, HEADER_BYTES, VERSION};
use crate::{BlockTable, Error, Flash, Geometry, Position, Slot};

/// How far a hint may run ahead of the newest block in flash (of 0 when flash holds none) and still raise the
/// numbering: 2^31 blocks, 8 TiB of writing, some 17 years at the flight rate. A real hint is at most a few blocks
/// ahead (headers a cut tore or a failed program spent), or anywhere when the ring was erased whole; the reach
/// takes both, and refuses the top half of the u32 range that only a corrupt hint reaches.
pub const HINT_REACH: u32 = 1 << 31;

/// The ring positions a previous boot left in the `cfg` NVS, when there are any.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Hints {
    /// The writer's last published position.
    pub writer: Option<Position>,
    /// The saver's last published position. It also chooses where the saver starts
    /// ([`BlockTable::saver_start`]).
    pub saver: Option<Position>,
}

/// What the bring-up scan found.
#[derive(Debug)]
pub struct Scan {
    /// Every slot's state, for the writer, the eraser and the saver to share.
    pub table: BlockTable,
    /// The sequence the writer gives the first block it opens: one past the newest block in flash and every
    /// hint within [`HINT_REACH`], so a new boot never reuses a number, nor appends to an old block. u32::MAX
    /// when every number is spent: the writer then refuses to open a block ([`Error::Exhausted`]).
    pub next_sequence: u32,
    /// No block of any layout anywhere: an unformatted ring (spec "Bring-up", step 2). The firmware shows
    /// `degraded: ring-unformatted` while this holds and [`crate::Figures::foreign`] is above 0, that is while
    /// the eraser formats the foreign slots ahead of the writer. Only the scan can tell: once the writer has
    /// opened a block, the table no longer shows it.
    pub unformatted: bool,
}

/// What a slot holds, judged by the 56 bytes at its start: [`Slot::Erased`] when they are all 0xFF,
/// [`Slot::Valid`] or [`Slot::OtherVersion`] for a whole header by its version, else [`Slot::Foreign`].
/// The scan never finds a block open: a new boot opens a block of its own.
pub fn classify(header: &[u8]) -> Slot {
    if header
        .get(..HEADER_BYTES)
        .is_some_and(|header| header.iter().all(|&byte| byte == 0xFF))
    {
        return Slot::Erased;
    }
    match BlockHeader::decode(header) {
        Ok(header) if header.version == VERSION => Slot::Valid {
            block_sequence: header.block_sequence,
            flight: header.flight,
        },
        Ok(header) => Slot::OtherVersion {
            block_sequence: header.block_sequence,
            flight: header.flight,
            version: header.version,
        },
        Err(_) => Slot::Foreign,
    }
}

/// Scans the ring: one 56-byte read per slot, classified into a new [`BlockTable`]. The writer's cursor goes to
/// the newest block's slot (it opens the first erased slot after it), and [`Scan::next_sequence`] goes past the
/// newest block and every hint within [`HINT_REACH`] of it.
///
/// Args:
/// * `flash` - the `log` partition.
/// * `geometry` - the ring's shape.
/// * `hints` - the positions the previous boot left in the `cfg` NVS (none on a first boot).
///
/// Returns the scan. Errors: `Geometry` when the geometry is larger than the flash, and `Flash` when a read fails.
pub fn scan<F: Flash>(flash: &F, geometry: Geometry, hints: Hints) -> Result<Scan, Error<F::Error>> {
    if geometry.bytes() > flash.size() {
        return Err(Error::Geometry);
    }
    let table = BlockTable::new(geometry);
    let mut header = [0u8; HEADER_BYTES];
    let mut newest: Option<(u32, u32)> = None; // (block sequence, slot)
    for slot in 0..geometry.blocks() {
        flash
            .read(geometry.slot_offset(slot), &mut header)
            .map_err(Error::Flash)?;
        let state = classify(&header);
        if let Some(block_sequence) = state.block_sequence()
            && newest.is_none_or(|(newest, _)| block_sequence > newest)
        {
            newest = Some((block_sequence, slot));
        }
        table.set(slot, state);
    }
    if let Some((_, slot)) = newest {
        table.set_cursor(slot);
    }
    let reach = newest.map_or(0, |(newest, _)| newest).saturating_add(HINT_REACH);
    let past_hints = [hints.writer, hints.saver]
        .into_iter()
        .flatten()
        .map(|hint| hint.block_sequence)
        .filter(|&block_sequence| block_sequence <= reach)
        .map(|block_sequence| block_sequence.saturating_add(1));
    // saturating: past a block numbered u32::MAX, u32::MAX itself says every number is spent
    let next_sequence = newest
        .map(|(newest, _)| newest.saturating_add(1))
        .into_iter()
        .chain(past_hints)
        .max()
        .unwrap_or(0);
    Ok(Scan {
        table,
        next_sequence,
        unformatted: newest.is_none(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::Session;

    /// The header of `block_sequence` in layout `version`.
    fn header(block_sequence: u32, version: u8, flight: bool) -> [u8; HEADER_BYTES] {
        BlockHeader {
            version,
            flight,
            block_sequence,
            boot_id: 9,
            session: Session::new("000009").unwrap(),
        }
        .encode()
    }

    #[test]
    fn classifies_erased_valid_other_version_and_foreign() {
        assert_eq!(classify(&[0xFF; HEADER_BYTES]), Slot::Erased);
        assert_eq!(
            classify(&header(5, VERSION, true)),
            Slot::Valid {
                block_sequence: 5,
                flight: true
            }
        );
        assert_eq!(
            classify(&header(6, 2, false)),
            Slot::OtherVersion {
                block_sequence: 6,
                flight: false,
                version: 2
            }
        );
        assert_eq!(classify(&[0x00; HEADER_BYTES]), Slot::Foreign);
        let mut torn = header(5, VERSION, false);
        torn[30..].fill(0xFF);
        assert_eq!(classify(&torn), Slot::Foreign, "a torn header");
        let mut almost_erased = [0xFF; HEADER_BYTES];
        almost_erased[55] = 0xFE;
        assert_eq!(classify(&almost_erased), Slot::Foreign);
        assert_eq!(classify(&[0xFF; 20]), Slot::Foreign, "too short to be anything");
    }
}
