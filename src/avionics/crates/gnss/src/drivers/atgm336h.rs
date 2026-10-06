//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! ATGM336H GNSS, GPS + BDS on a CASIC chip (src/glider drivers/atgm336h.py): the CASIC reconfiguration as bytes.
//!
//! RMC at `hz` (the position) plus GGA at ~1 Hz (the altitude and elevation, a baro backup): both fit 9600 baud
//! (~10 Hz RMC is ~700 B/s, ~1 Hz GGA ~70 B/s, under 960). Then the sky diagnostics every ~10 s: GSA, GSV and the
//! antenna text, ~0.5 kB a burst (~50 B/s); on every fix they would not fit (5 kB/s at 10 Hz). PCAS is the CASIC
//! command set; the PMTK pair goes out too, a fallback for MTK-variant modules (each side ignores the other's
//! sentences). The 2026-10-04 bench (src/gnss_bench) flew exactly this sequence and measured the diagnostics NOT to
//! disturb the flight rates: at hz 10, 250 RMC + 25 GGA in 25 s, the diagnostics at ~9.5 s and ~19.4 s.

use crate::{DIAGNOSTICS_S, SkySwitch, nmea};

/// The pause between two setup frames, in milliseconds (plan B's `_configure`): the module parses one sentence
/// before the next arrives.
pub const SEND_GAP_MS: u32 = 80;

/// The diagnostics divider's cap. The CASIC manual gives PCAS03 one digit per field, but the ATGM takes two
/// (measured: the GGA divider 10, and 99 on the bench), so 99: 9.9 s at 10 Hz.
const DIAGNOSTICS_CAP: i32 = 99;

/// PMTK314's GGA divider cap, the MTK fallback's own limit.
const MTK_DIVIDER_CAP: i32 = 5;

/// The setup sentences for `hz` fixes per second, framed, in send order (plan B `_commands`).
///
/// The flight init first (RMC every fix, GGA every `hz`th, the measurement period, the MTK pair), then the
/// diagnostics: the same PCAS03 with GSA, GSV and the antenna text raised from 0 to every Nth fix (N = `hz` x 10 s,
/// capped at 99). A rate below 1 counts as 1, so there is never a divide by zero.
pub fn commands(hz: i32) -> [Vec<u8>; 5] {
    let hz = hz.max(1);
    let period_ms = 1000 / hz; // 10 Hz -> 100 ms
    let gga_every = hz; // GGA once every `hz` fixes: ~1 Hz at any base rate (bandwidth)
    let diagnostics_every = hz.saturating_mul(DIAGNOSTICS_S).min(DIAGNOSTICS_CAP);
    [
        nmea(&format!("PCAS03,{gga_every},0,0,0,1,0,0,0,0,0,,,0,0")), // CASIC: GGA every Nth fix, RMC every fix
        nmea(&format!("PCAS02,{period_ms}")),                         // CASIC: the measurement period
        nmea(&format!(
            "PMTK314,0,1,0,{},0,0,0,0,0,0,0,0,0,0,0,0,0,0,0",
            gga_every.min(MTK_DIVIDER_CAP)
        )), // MTK fallback: RMC + GGA
        nmea(&format!("PMTK220,{period_ms}")),                        // MTK fallback: the update period
        // CASIC: the same mask, plus GSA, GSV and the antenna text (ANT) every Nth fix
        nmea(&format!(
            "PCAS03,{gga_every},0,{diagnostics_every},{diagnostics_every},1,0,0,{diagnostics_every},0,0,,,0,0"
        )),
    ]
}

/// The sky switch for `hz` (plan B `_configure`'s return): off is the flight init's own PCAS03 (RMC + GGA, nothing
/// else: the flight output as it was before the diagnostics), on is the diagnostics' PCAS03.
pub fn sky_switch(hz: i32) -> SkySwitch {
    let [off, _period, _mtk_selection, _mtk_period, on] = commands(hz);
    SkySwitch { off, on }
}
