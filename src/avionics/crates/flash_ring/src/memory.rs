//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! An in-memory NOR flash for the host: the tests run the ring over it, and a host tool can build a ring image
//! with it. It enforces NOR's rules, so a ring bug fails a test instead of passing on forgiving memory:
//! * a program refuses any 0 -> 1 bit change, and leaves the flash as it was;
//! * an erase takes a whole 4 KiB sector;
//! * a call out of range fails.
//!
//! It also cuts power on demand ([`MemoryFlash::cut_power`]). After a set number of program and erase calls, the
//! next one is torn: only its first bytes take effect, and every later call fails, as on a dead board. The
//! test then reboots by mounting a new ring over [`MemoryFlash::into_image`].

use alloc::vec;
use alloc::vec::Vec;
use core::cell::Cell;

use crate::{Flash, SECTOR_BYTES};

/// Why a [`MemoryFlash`] call failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemoryError {
    /// The call reaches past the flash's end.
    OutOfRange,
    /// An erase that does not start on a sector.
    Misaligned,
    /// A program that would set a programmed bit back to 1 at `offset`. NOR cannot, and the ring never may.
    ZeroToOne {
        /// The first byte that would change.
        offset: u32,
    },
    /// The power was cut: this call was torn, or came after the cut.
    PowerCut,
}

/// A scheduled power cut.
#[derive(Clone, Copy, Debug)]
struct Cut {
    /// Program and erase calls still to complete before the torn one.
    operations: u32,
    /// How many leading bytes of the torn call take effect.
    torn_bytes: usize,
}

/// A NOR flash in RAM, with operation counters and an optional power cut.
#[derive(Debug)]
pub struct MemoryFlash {
    /// The flash content.
    bytes: Vec<u8>,
    /// Read calls so far. A Cell, because [`Flash::read`] takes `&self`, as the board's does.
    reads: Cell<u32>,
    /// Program calls so far, a torn one included.
    programs: u32,
    /// Erase calls so far, a torn one included.
    erases: u32,
    /// The cut to come, if any.
    cut: Option<Cut>,
    /// Whether the cut has happened.
    dead: bool,
}

impl MemoryFlash {
    /// A fully erased flash of `size` bytes.
    pub fn new(size: u32) -> MemoryFlash {
        MemoryFlash::from_image(vec![0xFF; size as usize])
    }

    /// A flash holding `image`, such as the image of a flash whose power was cut, to boot again over.
    pub fn from_image(image: Vec<u8>) -> MemoryFlash {
        MemoryFlash {
            bytes: image,
            reads: Cell::new(0),
            programs: 0,
            erases: 0,
            cut: None,
            dead: false,
        }
    }

    /// The flash content.
    pub fn image(&self) -> &[u8] {
        &self.bytes
    }

    /// The flash content, to mount again or keep.
    pub fn into_image(self) -> Vec<u8> {
        self.bytes
    }

    /// Cuts the power on a later call. The next `operations` program or erase calls complete, then the
    /// following one is torn: only its first `torn_bytes` bytes are programmed (or erased to 0xFF), it fails
    /// with [`MemoryError::PowerCut`], and so does every call after it, reads included.
    pub fn cut_power(&mut self, operations: u32, torn_bytes: usize) {
        self.cut = Some(Cut { operations, torn_bytes });
    }

    /// Read calls so far.
    pub fn reads(&self) -> u32 {
        self.reads.get()
    }

    /// Program calls so far, a torn one included.
    pub fn programs(&self) -> u32 {
        self.programs
    }

    /// Erase calls so far, a torn one included.
    pub fn erases(&self) -> u32 {
        self.erases
    }

    /// The bytes `offset..offset + length`, or OutOfRange.
    fn range(&self, offset: u32, length: usize) -> Result<core::ops::Range<usize>, MemoryError> {
        let start = offset as usize;
        let end = start.checked_add(length).ok_or(MemoryError::OutOfRange)?;
        if end > self.bytes.len() {
            return Err(MemoryError::OutOfRange);
        }
        Ok(start..end)
    }

    /// Starts a program or erase. Returns how many leading bytes take effect when the cut lands on this call
    /// (None: the whole call), or PowerCut after the cut.
    fn begin(&mut self) -> Result<Option<usize>, MemoryError> {
        if self.dead {
            return Err(MemoryError::PowerCut);
        }
        match self.cut.as_mut() {
            Some(cut) if cut.operations == 0 => {
                self.dead = true;
                Ok(Some(cut.torn_bytes))
            }
            Some(cut) => {
                cut.operations -= 1;
                Ok(None)
            }
            None => Ok(None),
        }
    }
}

impl Flash for MemoryFlash {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        if self.dead {
            return Err(MemoryError::PowerCut);
        }
        let range = self.range(offset, buffer.len())?;
        buffer.copy_from_slice(&self.bytes[range]);
        self.reads.set(self.reads.get() + 1);
        Ok(())
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), MemoryError> {
        let range = self.range(offset, data.len())?;
        if let Some(index) = self.bytes[range.clone()]
            .iter()
            .zip(data)
            .position(|(&old, &new)| old & new != new)
        {
            return Err(MemoryError::ZeroToOne {
                offset: offset + index as u32,
            });
        }
        let torn = self.begin()?;
        self.programs += 1;
        let done = torn.unwrap_or(data.len()).min(data.len());
        for (cell, &new) in self.bytes[range.start..range.start + done].iter_mut().zip(data) {
            *cell &= new;
        }
        if torn.is_some() {
            Err(MemoryError::PowerCut)
        } else {
            Ok(())
        }
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        if !offset.is_multiple_of(SECTOR_BYTES) {
            return Err(MemoryError::Misaligned);
        }
        let range = self.range(offset, SECTOR_BYTES as usize)?;
        let torn = self.begin()?;
        self.erases += 1;
        let done = torn.unwrap_or(range.len()).min(range.len());
        self.bytes[range.start..range.start + done].fill(0xFF);
        if torn.is_some() {
            Err(MemoryError::PowerCut)
        } else {
            Ok(())
        }
    }

    fn size(&self) -> u32 {
        self.bytes.len() as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn programs_only_clear_bits() {
        let mut flash = MemoryFlash::new(8192);
        flash.program(10, &[0xF0, 0x0F]).unwrap();
        flash.program(10, &[0x30, 0x0F]).unwrap(); // clearing more bits is fine
        assert_eq!(flash.image()[9..13], [0xFF, 0x30, 0x0F, 0xFF]);
        assert_eq!(
            flash.program(10, &[0x30, 0x1F]),
            Err(MemoryError::ZeroToOne { offset: 11 })
        );
        assert_eq!(flash.image()[10..12], [0x30, 0x0F], "a refused program changes nothing");
        assert_eq!(flash.programs(), 2, "nor is it counted");
        let mut read = [0u8; 2];
        flash.read(10, &mut read).unwrap();
        assert_eq!((read, flash.reads()), ([0x30, 0x0F], 1));
    }

    #[test]
    fn erases_whole_aligned_sectors_in_range() {
        let mut flash = MemoryFlash::new(8192);
        flash.program(4095, &[0, 0]).unwrap();
        assert_eq!(flash.erase_sector(100), Err(MemoryError::Misaligned));
        assert_eq!(flash.erase_sector(8192), Err(MemoryError::OutOfRange));
        flash.erase_sector(0).unwrap();
        assert_eq!(
            flash.image()[4094..4098],
            [0xFF, 0xFF, 0x00, 0xFF],
            "only sector 0 is erased"
        );
        assert_eq!(flash.program(8191, &[0, 0]), Err(MemoryError::OutOfRange));
        assert_eq!(flash.read(8190, &mut [0; 4]), Err(MemoryError::OutOfRange));
    }

    #[test]
    fn a_cut_tears_one_call_and_kills_the_rest() {
        let mut flash = MemoryFlash::new(8192);
        flash.cut_power(1, 3);
        flash.program(0, &[0; 8]).unwrap();
        assert_eq!(flash.program(16, &[0; 8]), Err(MemoryError::PowerCut));
        assert_eq!(
            flash.image()[16..24],
            [0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            "a 3-byte prefix"
        );
        assert_eq!(flash.erase_sector(0), Err(MemoryError::PowerCut));
        assert_eq!(flash.read(0, &mut [0; 1]), Err(MemoryError::PowerCut));
        assert_eq!((flash.programs(), flash.erases()), (2, 0));

        let mut flash = MemoryFlash::from_image(vec![0u8; 8192]);
        flash.cut_power(0, 100);
        assert_eq!(flash.erase_sector(4096), Err(MemoryError::PowerCut));
        assert_eq!(flash.image()[4095..4097], [0x00, 0xFF]);
        assert_eq!(
            flash.image()[4195..4197],
            [0xFF, 0x00],
            "a 100-byte prefix of the sector"
        );
        assert_eq!(flash.into_image().len(), 8192);
    }
}
