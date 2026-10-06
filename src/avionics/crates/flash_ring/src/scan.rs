//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The boot scan (doc/specs/avionics-port.md, "Boot scan and recovery"): it reads the 40 bytes at each block
//! start (384 reads, ~8 ms on the board) and classifies the slot. Three facts make one header per block enough:
//! * the writer programs a block in address order with its BOOT frame first;
//! * GC erases a block's sectors in reverse order, so the header sector goes last;
//! * a block that reads erased at offset 0 therefore holds nothing, and a block whose header is neither valid,
//!   nor erased, nor a whole header of another layout version holds at most a torn BOOT frame.
//!
//! The writer still reads a whole block before its first program into it ([`crate::Ring`]), because a torn
//! erase of the header sector can leave an erased header over a dirty sector.

use alloc::vec;
use alloc::vec::Vec;

use crate::frame::{FrameError, HEADER_BYTES, Header, Kind};
use crate::{Flash, Geometry};

/// What a slot holds, judged by its offset-0 header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    /// The header reads all 0xFF: free for the writer, after its full read proves the rest erased too.
    Erased,
    /// A BOOT header for a block that belongs in this slot: ring data, erased only once the mover has
    /// confirmed it.
    Valid {
        /// The block's sequence number.
        block_sequence: u32,
        /// The boot that opened it.
        boot_id: u32,
    },
    /// Anything else: a torn BOOT, MicroPython-era filesystem data, a bench pattern, or a dirty block the
    /// writer refused. It holds no recoverable ring data, so GC erases it without a confirmation.
    Foreign,
    /// A whole header (magic and header CRC good) of another layout version: ring data that a newer or older
    /// firmware wrote and this one cannot read. It is no torn BOOT, so it may still be unmoved, and the board
    /// never erases unconfirmed data on its own. GC idles at it, as at an unconfirmed block, until the
    /// operator's discard.
    OtherVersion,
}

impl Slot {
    /// The block sequence of a valid slot. Returns None for any other.
    pub fn block_sequence(self) -> Option<u32> {
        match self {
            Slot::Valid { block_sequence, .. } => Some(block_sequence),
            Slot::Erased | Slot::Foreign | Slot::OtherVersion => None,
        }
    }
}

/// Classifies slot `slot` by the bytes at its offset 0.
///
/// A header counts as valid only when it is a BOOT frame, the frame every block starts with, and its block
/// sequence maps to this slot. A block found in the wrong slot comes from another geometry. Trusting it would
/// break the FIFO the writer and GC walk. It stays foreign: the geometry is fixed for the partition, and a
/// header check could not keep another geometry's blocks anyway, since their starts fall between this ring's.
/// A version is different. Another layout version keeps the slots and the BOOT-first rule, so its header is
/// found where this ring looks, and it is kept ([`Slot::OtherVersion`]).
pub fn classify(slot: u32, header: &[u8], geometry: Geometry) -> Slot {
    if header.iter().all(|&byte| byte == 0xFF) {
        return Slot::Erased;
    }
    match Header::decode(header) {
        Ok(header) if header.kind == Kind::Boot && geometry.slot(header.block_sequence) == slot => Slot::Valid {
            block_sequence: header.block_sequence,
            boot_id: header.boot_id,
        },
        Err(FrameError::Version) => Slot::OtherVersion,
        _ => Slot::Foreign,
    }
}

/// Every slot of the ring, classified from its offset-0 header: one 40-byte read per block.
pub fn scan<F: Flash>(flash: &F, geometry: Geometry) -> Result<Vec<Slot>, F::Error> {
    let mut header = [0u8; HEADER_BYTES];
    let mut slots = vec![Slot::Erased; geometry.blocks() as usize];
    for (slot, state) in (0..geometry.blocks()).zip(slots.iter_mut()) {
        flash.read(geometry.slot_offset(slot), &mut header)?;
        *state = classify(slot, &header, geometry);
    }
    Ok(slots)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::Flags;

    /// A BOOT header for `block_sequence`.
    fn boot(block_sequence: u32) -> Vec<u8> {
        Header {
            kind: Kind::Boot,
            flags: Flags::default(),
            payload_length: 0,
            block_sequence,
            boot_id: 9,
            frame_sequence: 0,
            uptime_us: 1,
        }
        .encode(b"")
        .unwrap()
    }

    #[test]
    fn classifies_erased_valid_and_foreign() {
        let geometry = Geometry::new(4, 1).unwrap();
        assert_eq!(classify(1, &[0xFF; 40], geometry), Slot::Erased);
        let valid = Slot::Valid {
            block_sequence: 5,
            boot_id: 9,
        };
        assert_eq!(classify(1, &boot(5), geometry), valid);
        assert_eq!(valid.block_sequence(), Some(5));
        assert_eq!(Slot::Foreign.block_sequence(), None);
    }

    #[test]
    fn a_torn_wrong_kind_or_misplaced_header_is_foreign() {
        let geometry = Geometry::new(4, 1).unwrap();
        let mut torn = boot(5);
        torn[20..].fill(0xFF); // the program stopped inside the header
        assert_eq!(classify(1, &torn, geometry), Slot::Foreign);
        assert_eq!(
            classify(2, &boot(5), geometry),
            Slot::Foreign,
            "block 5 lives in slot 1 of 4"
        );
        let data = Header {
            kind: Kind::Data,
            ..Header::decode(&boot(5)).unwrap()
        }
        .encode(b"")
        .unwrap();
        assert_eq!(classify(1, &data, geometry), Slot::Foreign, "a block starts with BOOT");
        assert_eq!(classify(1, &[0x00; 40], geometry), Slot::Foreign);
        let mut almost_erased = [0xFF; 40];
        almost_erased[39] = 0xFE;
        assert_eq!(classify(1, &almost_erased, geometry), Slot::Foreign);
    }

    /// `header` re-stamped as layout `version`, with its header CRC recomputed: what another firmware writes.
    fn restamped(mut header: Vec<u8>, version: u8) -> Vec<u8> {
        header[4] = version;
        let header_crc = recorder_wire::crc32(0, &header[..36]);
        header[36..40].copy_from_slice(&header_crc.to_le_bytes());
        header
    }

    #[test]
    fn a_whole_header_of_another_version_is_kept_apart_from_foreign() {
        let geometry = Geometry::new(4, 1).unwrap();
        for version in [0, 2, 0xFF] {
            assert_eq!(classify(1, &restamped(boot(5), version), geometry), Slot::OtherVersion);
        }
        assert_eq!(Slot::OtherVersion.block_sequence(), None);
        let mut torn = restamped(boot(5), 2);
        torn[30..].fill(0xFF); // torn before its header CRC: nothing proves it whole
        assert_eq!(classify(1, &torn, geometry), Slot::Foreign);
        let mut stale = boot(5);
        stale[4] = 2; // the version changed, the CRC did not: a damaged header, not another layout
        assert_eq!(classify(1, &stale, geometry), Slot::Foreign);
    }
}
