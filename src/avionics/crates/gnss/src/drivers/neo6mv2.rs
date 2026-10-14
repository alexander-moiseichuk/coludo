//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! GY-NEO6MV2, a u-blox NEO-6M (src/glider drivers/neo6mv2.py): the u-blox reconfiguration as bytes. A drop-in
//! alternative to the ATGM336H on the same UART (lower `hz`: the NEO-6M tops out near 5 Hz).
//!
//! `$PUBX,40` selects RMC (the position) and GGA at ~1 Hz (the altitude and elevation) on the UART and silences the
//! rest; UBX-CFG-RATE sets the measurement period; UBX-CFG-NAV5 the airborne dynamic model; then `$PUBX,40` turns
//! GSA and GSV back on every ~10 s, the sky diagnostics (the NEO-6M reports no antenna status over NMEA). The
//! 2026-10-04 bench (src/gnss_bench) flew this sequence without the NAV5 frame and measured the diagnostics NOT to
//! disturb the flight rates: at hz 5, 125 RMC + 25 GGA in 25 s, GSA + GSV every ~10 s.

use crate::{DIAGNOSTICS_S, SkySwitch, nmea};

/// The pause between two setup frames, in milliseconds (plan B's `_configure`).
pub const SEND_GAP_MS: u32 = 40;

/// UBX-CFG, the configuration class.
const CONFIGURATION_CLASS: u8 = 0x06;

/// UBX-CFG-RATE: the measurement period.
const RATE_MESSAGE: u8 = 0x08;

/// UBX-CFG-NAV5: the navigation engine settings.
const NAVIGATION_MESSAGE: u8 = 0x24;

/// UBX-CFG-NAV5's payload length: 36 bytes, of which the mask and the dynamic model are set.
const NAVIGATION_PAYLOAD: usize = 36;

/// NAV5's mask bit 0: apply the dynamic model and ignore every other field.
const NAV5_DYNAMIC_MODEL_ONLY: u16 = 0x0001;

/// NAV5's dynamic model 8: airborne below 4 g. A u-blox receiver starts on 'portable', whose platform limits (the
/// Receiver Description's "Dynamic platform" table, vertical speed among them) are a walker's or a car's, and a
/// solution outside its model's limits is discarded. A rocket's boost and coast leave those far behind (TMS-7:
/// ~125 m/s, ~15 g), so the receiver would drop the fix exactly when it matters. Airborne <4g is the most permissive
/// model the NEO-6M and NEO-M8 share: the boost may still overrun it for a few seconds, and the fix returns in the
/// glide.
const NAV5_AIRBORNE_4G: u8 = 8;

/// A port's output rate is one byte (UBX-CFG-MSG), so the diagnostics divider caps at 255: only above 25 Hz, which
/// a module topping out near 5 Hz never runs.
const RATE_CAP: i32 = 255;

/// A UBX binary frame (plan B `_ubx`): the sync characters, the class, the id, the little-endian payload length, the
/// payload, and the 8-bit Fletcher checksum over class..payload.
///
/// Panics when the payload is longer than UBX's u16 length can say (65535 bytes); the frames here are 6 and 36.
pub fn ubx(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    crate::ubx::frame(class, id, payload)
}

/// The setup frames for `hz` fixes per second, in send order (plan B `_commands`).
///
/// The flight init first: the `$PUBX,40` selection (RMC every fix, GGA every `hz`th, GLL/GSA/GSV/VTG silenced to
/// stay within 9600 baud), UBX-CFG-RATE (the period; navRate 1, GPS time), UBX-CFG-NAV5 (airborne <4g). Then the
/// diagnostics: GSA and GSV every Nth fix (N = `hz` x 10 s, capped at 255). A rate below 1 counts as 1.
pub fn commands(hz: i32) -> [Vec<u8>; 10] {
    let hz = hz.max(1);
    let period_ms = u16::try_from(1000 / hz).expect("1000 / hz fits a u16 for hz >= 1");
    let gga_every = hz; // GGA once every `hz` fixes: ~1 Hz (bandwidth)
    let diagnostics_every = hz.saturating_mul(DIAGNOSTICS_S).min(RATE_CAP);
    // $PUBX,40,<msg>,rddc,rus1,rus2,rusb,rspi,res: the per-port output divider (rus1 is this UART)
    let selection = |message: &str, divider: i32| nmea(&format!("PUBX,40,{message},0,{divider},0,0,0,0"));
    let mut rate = Vec::with_capacity(6);
    for half in [period_ms, 1, 1] {
        rate.extend_from_slice(&half.to_le_bytes()); // measRate ms, navRate cycles, timeRef 1 (GPS)
    }
    let mut navigation = vec![0_u8; NAVIGATION_PAYLOAD];
    navigation[..2].copy_from_slice(&NAV5_DYNAMIC_MODEL_ONLY.to_le_bytes());
    navigation[2] = NAV5_AIRBORNE_4G;
    [
        selection("RMC", 1),
        selection("GGA", gga_every),
        selection("GLL", 0),
        selection("GSA", 0),
        selection("GSV", 0),
        selection("VTG", 0),
        ubx(CONFIGURATION_CLASS, RATE_MESSAGE, &rate),
        ubx(CONFIGURATION_CLASS, NAVIGATION_MESSAGE, &navigation),
        selection("GSA", diagnostics_every), // the sky diagnostics: fix mode + satellites used ...
        selection("GSV", diagnostics_every), // ... and satellites in view with their C/N0
    ]
}

/// The sky switch for `hz` (plan B `_configure`'s return): off is the flight init's own GSA and GSV at 0 (the flight
/// output as it was before the diagnostics), on is the diagnostics' GSA and GSV, each pair one write. u-blox parses
/// its input sentence by sentence, but this back-to-back pair (the init puts 40 ms between them) is not bench-checked
/// yet.
pub fn sky_switch(hz: i32) -> SkySwitch {
    let frames = commands(hz);
    SkySwitch {
        off: [frames[3].as_slice(), frames[4].as_slice()].concat(),
        on: [frames[8].as_slice(), frames[9].as_slice()].concat(),
    }
}
