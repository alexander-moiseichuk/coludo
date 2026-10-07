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
//! (they never call esp_mmu_map, the one runtime path that turns the cache off). The stack pointer is the marker: a
//! Rust allocation made on a PSRAM stack goes to PSRAM, and falls back to std's path when PSRAM has none. Everything
//! else -- the flight and ring threads, boot on main, interrupts -- allocates exactly as before. C code (NVS, FatFs,
//! esp_hosted) calls malloc itself and is untouched.

use std::alloc::{GlobalAlloc, Layout, System};

use esp_idf_svc::sys;

/// The capabilities a PSRAM-stack thread's allocations ask for.
const PSRAM: u32 = sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT;

/// The alignment IDF's heap guarantees (std's MIN_ALIGN for espidf).
const HEAP_ALIGN: usize = 4;

/// std's allocator, with PSRAM for the threads whose stack is in PSRAM.
pub struct Allocator;

#[global_allocator]
static ALLOCATOR: Allocator = Allocator;

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
        if stack_in_psram() {
            // SAFETY: forwarded from the caller; the block holds layout.size() bytes.
            let block = unsafe { psram_alloc(layout) };
            if !block.is_null() {
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
