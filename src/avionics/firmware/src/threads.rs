//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The firmware's threads: one way to spawn a std thread (a FreeRTOS task underneath) on a core, at a priority, with
//! its stack in the memory it needs (doc/specs/avionics-port.md §4.7, the internal RAM budget). esp-idf-svc takes a
//! thread's name, core, priority and stack memory from a thread-local spawn configuration, so every spawn sets it,
//! spawns, and puts the default back for the next plain `std::thread::spawn`.

use std::ffi::CStr;
use std::thread::JoinHandle;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};

/// Where a thread's stack lives (§4.7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stack {
    /// Internal RAM: a thread on the flight core, or one that calls the flash driver, which turns the cache (and so
    /// PSRAM) off.
    Internal,
    /// PSRAM, and so its Rust allocations too (crate::allocator): a slow core-0 thread that never touches the flash.
    Psram,
}

/// A thread to spawn.
#[derive(Clone, Copy, Debug)]
pub struct Spec {
    /// Its task name (FreeRTOS keeps up to 15 characters).
    pub name: &'static CStr,
    /// Its stack, bytes.
    pub stack_size: usize,
    /// Where the stack lives.
    pub stack: Stack,
    /// Its FreeRTOS priority.
    pub priority: u8,
    /// The core it is pinned to.
    pub core: Core,
}

/// Spawns `body` as `spec` describes, then puts the default spawn configuration back.
///
/// Errors: the configuration would not set, or the thread would not spawn.
pub fn spawn<T: Send + 'static>(
    spec: Spec,
    body: impl FnOnce() -> T + Send + 'static,
) -> anyhow::Result<JoinHandle<T>> {
    ThreadSpawnConfiguration {
        name: Some(spec.name),
        stack_size: spec.stack_size,
        priority: spec.priority,
        pin_to_core: Some(spec.core),
        stack_alloc_caps: match spec.stack {
            Stack::Internal => ThreadSpawnConfiguration::default().stack_alloc_caps,
            Stack::Psram => MallocCap::Spiram | MallocCap::Cap8bit,
        },
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(spec.stack_size).spawn(body);
    ThreadSpawnConfiguration::default().set()?;
    Ok(spawned?)
}
