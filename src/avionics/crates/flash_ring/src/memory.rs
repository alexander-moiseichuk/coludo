//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! An in-memory NOR flash for the host: the tests run the ring over it, and a host tool can build a ring image
//! with it. It enforces NOR's rules, so a ring bug fails a test instead of passing on forgiving memory:
//! * a program refuses any 0 -> 1 bit change, and leaves the flash as it was;
//! * an erase takes a whole 4 KiB sector;
//! * a call out of range fails.
//!
//! The writer and the eraser each hold their own handle on the board, so here both borrow one `MemoryFlash`:
//! [`Flash`] is implemented for `&MemoryFlash` as well, over interior mutability.
//!
//! It also fails on demand. [`MemoryFlash::cut_power`]: after a set number of program and erase calls, the next
//! one is torn (only its first bytes take effect) and every later call fails, as on a dead board; the test then
//! boots again over [`MemoryFlash::into_image`]. [`MemoryFlash::fail`] tears one call the same way but leaves the
//! flash alive, as a flash error the writer retries.

use alloc::vec;
use alloc::vec::Vec;
use core::cell::{Cell, RefCell};

use crate::{BLOCK_BYTES, Flash};

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
    /// This one call failed after its first bytes took effect ([`MemoryFlash::fail`]); later calls work.
    Failed,
}

/// A scheduled fault.
#[derive(Clone, Copy, Debug)]
struct Fault {
    /// Program and erase calls still to complete before the torn one.
    operations: u32,
    /// How many leading bytes of the torn call take effect.
    torn_bytes: usize,
    /// Whether the power goes with it, failing every later call.
    power: bool,
}

/// A NOR flash in RAM, with operation counters and an optional fault.
#[derive(Debug)]
pub struct MemoryFlash {
    /// The flash content.
    bytes: RefCell<Vec<u8>>,
    /// Read calls so far.
    reads: Cell<u32>,
    /// Program calls so far, a torn one included.
    programs: Cell<u32>,
    /// Erase calls so far, a torn one included.
    erases: Cell<u32>,
    /// The fault to come, if any.
    fault: Cell<Option<Fault>>,
    /// Whether the power was cut.
    dead: Cell<bool>,
}

impl MemoryFlash {
    /// A fully erased flash of `size` bytes.
    pub fn new(size: u32) -> MemoryFlash {
        MemoryFlash::from_image(vec![0xFF; size as usize])
    }

    /// A flash holding `image`, such as the image of a flash whose power was cut, to boot again over.
    pub fn from_image(image: Vec<u8>) -> MemoryFlash {
        MemoryFlash {
            bytes: RefCell::new(image),
            reads: Cell::new(0),
            programs: Cell::new(0),
            erases: Cell::new(0),
            fault: Cell::new(None),
            dead: Cell::new(false),
        }
    }

    /// A copy of the flash content.
    pub fn image(&self) -> Vec<u8> {
        self.bytes.borrow().clone()
    }

    /// The flash content, to boot again over or keep.
    pub fn into_image(self) -> Vec<u8> {
        self.bytes.into_inner()
    }

    /// Cuts the power on a later call. The next `operations` program or erase calls complete, then the following
    /// one is torn: only its first `torn_bytes` bytes are programmed (or erased to 0xFF), it fails with
    /// [`MemoryError::PowerCut`], and so does every call after it, reads included. A `torn_bytes` past the call's
    /// length lets it complete before the power goes: it was done but never returned.
    pub fn cut_power(&self, operations: u32, torn_bytes: usize) {
        self.fault.set(Some(Fault {
            operations,
            torn_bytes,
            power: true,
        }));
    }

    /// Fails one later call: as [`MemoryFlash::cut_power`], but it fails with [`MemoryError::Failed`] and the
    /// calls after it work, as a flash error that a retry gets past.
    pub fn fail(&self, operations: u32, torn_bytes: usize) {
        self.fault.set(Some(Fault {
            operations,
            torn_bytes,
            power: false,
        }));
    }

    /// Read calls so far.
    pub fn reads(&self) -> u32 {
        self.reads.get()
    }

    /// Program calls so far, a torn one included.
    pub fn programs(&self) -> u32 {
        self.programs.get()
    }

    /// Erase calls so far, a torn one included.
    pub fn erases(&self) -> u32 {
        self.erases.get()
    }

    /// The bytes `offset..offset + length`, or OutOfRange.
    fn range(&self, offset: u32, length: usize) -> Result<core::ops::Range<usize>, MemoryError> {
        let start = offset as usize;
        let end = start.checked_add(length).ok_or(MemoryError::OutOfRange)?;
        if end > self.bytes.borrow().len() {
            return Err(MemoryError::OutOfRange);
        }
        Ok(start..end)
    }

    /// Starts a program or erase. Returns the fault when it lands on this call (None: the whole call), or
    /// PowerCut after a cut.
    fn begin(&self) -> Result<Option<Fault>, MemoryError> {
        if self.dead.get() {
            return Err(MemoryError::PowerCut);
        }
        match self.fault.get() {
            Some(fault) if fault.operations == 0 => {
                self.fault.set(None);
                self.dead.set(fault.power);
                Ok(Some(fault))
            }
            Some(fault) => {
                self.fault.set(Some(Fault {
                    operations: fault.operations - 1,
                    ..fault
                }));
                Ok(None)
            }
            None => Ok(None),
        }
    }

    /// Ends a program or erase that `fault` (if any) tore.
    fn end(fault: Option<Fault>) -> Result<(), MemoryError> {
        match fault {
            Some(Fault { power: true, .. }) => Err(MemoryError::PowerCut),
            Some(Fault { power: false, .. }) => Err(MemoryError::Failed),
            None => Ok(()),
        }
    }

    /// [`Flash::read`], through a shared handle.
    fn read_at(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        if self.dead.get() {
            return Err(MemoryError::PowerCut);
        }
        let range = self.range(offset, buffer.len())?;
        buffer.copy_from_slice(&self.bytes.borrow()[range]);
        self.reads.set(self.reads.get() + 1);
        Ok(())
    }

    /// [`Flash::program`], through a shared handle.
    fn program_at(&self, offset: u32, data: &[u8]) -> Result<(), MemoryError> {
        let range = self.range(offset, data.len())?;
        if let Some(index) = self.bytes.borrow()[range.clone()]
            .iter()
            .zip(data)
            .position(|(&old, &new)| old & new != new)
        {
            return Err(MemoryError::ZeroToOne {
                offset: offset + index as u32,
            });
        }
        let fault = self.begin()?;
        self.programs.set(self.programs.get() + 1);
        let done = fault.map_or(data.len(), |fault| fault.torn_bytes.min(data.len()));
        let mut bytes = self.bytes.borrow_mut();
        for (cell, &new) in bytes[range.start..range.start + done].iter_mut().zip(data) {
            *cell &= new;
        }
        MemoryFlash::end(fault)
    }

    /// [`Flash::erase_sector`], through a shared handle.
    fn erase_at(&self, offset: u32) -> Result<(), MemoryError> {
        if !offset.is_multiple_of(BLOCK_BYTES) {
            return Err(MemoryError::Misaligned);
        }
        let range = self.range(offset, BLOCK_BYTES as usize)?;
        let fault = self.begin()?;
        self.erases.set(self.erases.get() + 1);
        let done = fault.map_or(range.len(), |fault| fault.torn_bytes.min(range.len()));
        self.bytes.borrow_mut()[range.start..range.start + done].fill(0xFF);
        MemoryFlash::end(fault)
    }
}

impl Flash for MemoryFlash {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        self.read_at(offset, buffer)
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), MemoryError> {
        self.program_at(offset, data)
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        self.erase_at(offset)
    }

    fn size(&self) -> u32 {
        self.bytes.borrow().len() as u32
    }
}

impl Flash for &MemoryFlash {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        self.read_at(offset, buffer)
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), MemoryError> {
        self.program_at(offset, data)
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        self.erase_at(offset)
    }

    fn size(&self) -> u32 {
        self.bytes.borrow().len() as u32
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
    fn two_handles_share_one_flash() {
        let flash = MemoryFlash::new(8192);
        let mut writer = &flash;
        let mut eraser = &flash;
        writer.program(4096, &[0x12]).unwrap();
        assert_eq!(flash.image()[4096], 0x12);
        eraser.erase_sector(4096).unwrap();
        assert_eq!(flash.image()[4096], 0xFF);
        assert_eq!((flash.programs(), flash.erases(), writer.size()), (1, 1, 8192));
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

    #[test]
    fn a_failure_tears_one_call_and_the_flash_goes_on() {
        let mut flash = MemoryFlash::new(8192);
        flash.fail(0, 2);
        assert_eq!(flash.program(0, &[0; 8]), Err(MemoryError::Failed));
        assert_eq!(flash.image()[..3], [0, 0, 0xFF], "a 2-byte prefix");
        flash.program(8, &[0; 8]).unwrap();
        flash.erase_sector(0).unwrap();
        assert_eq!((flash.programs(), flash.erases()), (2, 1));
        flash.fail(0, usize::MAX);
        assert_eq!(
            flash.program(0, &[1, 2]),
            Err(MemoryError::Failed),
            "it landed whole and still failed"
        );
        assert_eq!(flash.image()[..2], [1, 2]);
    }
}
