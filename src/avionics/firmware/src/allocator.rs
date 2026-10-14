//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The heap allocator: std's own (IDF's malloc), except that a thread whose stack lives in PSRAM allocates in PSRAM.
//!
//! Internal RAM is the P4's scarce memory (doc/specs/avionics-port.md §4.7): every byte of it is DMA-capable, and
//! esp_hosted asserts and reboots the board when it gets none for a Wi-Fi frame. IDF keeps every malloc of 4 KB or
//! less internal (SPIRAM_MALLOC_ALWAYSINTERNAL), and a parsed JSON document is thousands of such allocations: one
//! `set-config board` of a 7 KB config took the internal heap from ~70 KB free to 20 bytes, on the CC executor.
//!
//! The slow core-0 threads (the CC executor, the console, the supervisor, the file server) already run on PSRAM stacks
//! (they never call esp_mmu_map or esp_mmu_unmap, the one runtime path that turns the cache off). The stack pointer is
//! the marker: a Rust allocation made on a PSRAM stack goes to PSRAM, and falls back to std's path when PSRAM has none
//! (a fallback only while CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS stays off, IDF's default: with it a full PSRAM
//! aborts). Everything else -- the flight and ring threads, boot on main, interrupts -- allocates exactly as before.
//! C code (NVS, FatFs, esp_hosted) calls malloc itself and is untouched.
//!
//! What such a thread needs in internal RAM it asks for by name: [`InternalBuffer`] (a flash read's destination).
//!
//! It also counts the Rust allocations made on core 1, the flight core ([`flight_core_allocations`]): an allocation
//! there takes the heap's lock, which core 0 shares, so a tick that allocates can wait on core 0's malloc. The
//! control thread's `control` command reports the count, which should stop growing once the bring-up is over.

use std::alloc::{GlobalAlloc, Layout, System};
use std::ops::{Deref, DerefMut};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU32, Ordering};

use esp_idf_svc::sys;

/// The capabilities a PSRAM-stack thread's allocations ask for.
const PSRAM: u32 = sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT;

/// The alignment IDF's heap guarantees (std's MIN_ALIGN for espidf).
const HEAP_ALIGN: usize = 4;

/// std's allocator, with PSRAM for the threads whose stack is in PSRAM.
pub struct Allocator;

#[global_allocator]
static ALLOCATOR: Allocator = Allocator;

/// The Rust allocations (and reallocations) made on core 1 since boot.
static FLIGHT_CORE_ALLOCATIONS: AtomicU32 = AtomicU32::new(0);

/// The Rust allocations made on core 1, the flight core, since boot (the control thread, the I2C owners).
pub fn flight_core_allocations() -> u32 {
    FLIGHT_CORE_ALLOCATIONS.load(Ordering::Relaxed)
}

/// Counts an allocation when the calling code runs on core 1: the hart id CSR, one instruction.
#[inline(always)]
fn count_flight_core() {
    let hart: usize;
    // SAFETY: reads the hart id CSR (ESP-IDF runs in machine mode); no memory is touched.
    unsafe { core::arch::asm!("csrr {}, mhartid", out(reg) hart, options(nomem, nostack, preserves_flags)) };
    if hart == 1 {
        FLIGHT_CORE_ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Whether the calling thread's stack is in PSRAM: the stack pointer against the external RAM range.
#[inline(always)]
fn stack_in_psram() -> bool {
    let stack: usize;
    // SAFETY: reads the stack pointer register; no memory is touched.
    unsafe { core::arch::asm!("mv {}, sp", out(reg) stack, options(nomem, nostack, preserves_flags)) };
    (sys::SOC_EXTRAM_LOW as usize..sys::SOC_EXTRAM_HIGH as usize).contains(&stack)
}

/// A PSRAM block for `layout`, or null.
///
/// # Safety
/// As [`GlobalAlloc::alloc`].
unsafe fn psram_alloc(layout: Layout) -> *mut u8 {
    // SAFETY: plain heap calls; heap_caps_aligned_alloc takes any power-of-two alignment, which a Layout's is.
    unsafe {
        if layout.align() <= HEAP_ALIGN {
            sys::heap_caps_malloc(layout.size(), PSRAM) as *mut u8
        } else {
            sys::heap_caps_aligned_alloc(layout.align(), layout.size(), PSRAM) as *mut u8
        }
    }
}

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count_flight_core();
        if stack_in_psram() {
            // SAFETY: forwarded from the caller.
            let block = unsafe { psram_alloc(layout) };
            if !block.is_null() {
                return block;
            }
        }
        // SAFETY: forwarded from the caller.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count_flight_core();
        if stack_in_psram() {
            // SAFETY: forwarded from the caller; the block holds layout.size() bytes.
            let block = unsafe { psram_alloc(layout) };
            if !block.is_null() {
                // SAFETY: the block was just allocated with layout.size() bytes.
                unsafe { block.write_bytes(0, layout.size()) };
                return block;
            }
        }
        // SAFETY: forwarded from the caller.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, block: *mut u8, layout: Layout) {
        // SAFETY: free() releases a block from any heap, aligned ones included (heap_caps_free).
        unsafe { System.dealloc(block, layout) }
    }

    unsafe fn realloc(&self, block: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count_flight_core();
        if stack_in_psram() {
            // SAFETY: forwarded from the caller. heap_caps_realloc moves the block into PSRAM when it is not there
            // (its contents kept); an over-aligned block is moved by hand, as std's own realloc does.
            unsafe {
                if layout.align() <= HEAP_ALIGN {
                    let moved = sys::heap_caps_realloc(block as *mut _, new_size, PSRAM) as *mut u8;
                    if !moved.is_null() {
                        return moved;
                    }
                } else {
                    let grown = Layout::from_size_align_unchecked(new_size, layout.align());
                    let moved = psram_alloc(grown);
                    if !moved.is_null() {
                        moved.copy_from_nonoverlapping(block, layout.size().min(new_size));
                        System.dealloc(block, layout);
                        return moved;
                    }
                }
            }
        }
        // SAFETY: forwarded from the caller.
        unsafe { System.realloc(block, layout, new_size) }
    }
}

/// A zeroed byte buffer in internal RAM, whatever the calling thread's stack.
///
/// For a destination that must be internal on a PSRAM-stack thread: esp_flash_read fills internal RAM directly, and
/// reads into any other memory through an internal bounce buffer it allocates per call, after walking the internal
/// heap for its largest block with core 0's interrupts masked -- thousands of times per ring pull, and a failed
/// allocation fails the read.
pub struct InternalBuffer {
    /// The buffer, `length` bytes from heap_caps_calloc.
    pointer: NonNull<u8>,
    /// Its length in bytes, never zero.
    length: usize,
}

// SAFETY: the buffer is owned memory with no thread affinity; free() releases it from any thread.
unsafe impl Send for InternalBuffer {}

impl InternalBuffer {
    /// `length` (not zero) zeroed bytes of internal RAM; None when the internal heap has no such block.
    pub fn zeroed(length: usize) -> Option<Self> {
        if length == 0 {
            return None;
        }
        // SAFETY: a plain heap call; the block is owned by the returned value and freed on its drop.
        let block = unsafe { sys::heap_caps_calloc(1, length, sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT) };
        NonNull::new(block as *mut u8).map(|pointer| Self { pointer, length })
    }
}

impl Deref for InternalBuffer {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        // SAFETY: `length` initialised (zeroed) bytes, owned by self.
        unsafe { std::slice::from_raw_parts(self.pointer.as_ptr(), self.length) }
    }
}

impl DerefMut for InternalBuffer {
    fn deref_mut(&mut self) -> &mut [u8] {
        // SAFETY: as deref, and borrowed mutably through self.
        unsafe { std::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.length) }
    }
}

impl Drop for InternalBuffer {
    fn drop(&mut self) {
        // SAFETY: the block came from heap_caps_calloc and is freed once.
        unsafe { sys::heap_caps_free(self.pointer.as_ptr().cast()) };
    }
}
