//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The u-blox NEO-M8N (M16): binary delivery. Plan B had no driver for it (its NEO-M8N ran as the NEO-6M, on NMEA);
//! this one switches the module's UART to UBX only and reads the navigation solution as one UBX-NAV-PVT per epoch
//! (crate::ubx), so no epoch is half delivered and no text is parsed. Every frame is UBX:
//!
//! * UBX-CFG-PRT: UART1 at [`BAUD`] (8N1), UBX and NMEA taken in, UBX only out. 9600 baud carries a 100-byte NAV-PVT
//!   at most ~9 times a second with nothing else, so the port goes up first;
//! * UBX-CFG-RATE: the measurement period (the entry's `hz`; the M8N runs GPS + GLONASS at up to 10 Hz);
//! * UBX-CFG-NAV5: the airborne <4g dynamic model, the NEO-6M driver's (drivers::neo6mv2 says why);
//! * UBX-CFG-MSG: NAV-PVT on every solution, and NAV-SAT every `hz` x 10 s, the sky diagnostics (its switch).
//!
//! Each frame is answered with an ACK-ACK or an ACK-NAK, which the firmware logs: the NMEA path could only tell a
//! refusal from a `$GPTXT ... inv format`.
//!
//! **The baud after a P4 reboot (autobaud):** the module keeps the port settings until its own power cycle, so a P4
//! that reboots while the module stays powered finds it at [`BAUD`], not at its power-on 9600. The firmware writes
//! the CFG-PRT frame at both rates ([`port`] at [`BAUD`] first, then at the power-on rate) and talks at [`BAUD`]
//! after: whichever rate the module listens at, one copy lands, and the other is noise it drops.

use crate::DIAGNOSTICS_S;
use crate::SkySwitch;
use crate::ubx::{CONFIGURATION_CLASS, NAVIGATION_CLASS, PVT, SAT, frame};

/// The pause between two setup frames, ms: the module answers each with its ACK first.
pub const SEND_GAP_MS: u32 = 40;

/// The UART's rate after the configuration: NAV-PVT (100 bytes) at 10 Hz and a NAV-SAT (~300 bytes) every 10 s use
/// about a quarter of it.
pub const BAUD: u32 = 38_400;

/// The fastest rate configured: the M8N's limit with two constellations.
const MAXIMUM_HZ: i32 = 10;

/// UBX-CFG-PRT: the port's settings.
const PORT_MESSAGE: u8 = 0x00;
/// UBX-CFG-MSG: a message's output rate.
const MESSAGE_RATE: u8 = 0x01;
/// UBX-CFG-RATE: the measurement period.
const RATE_MESSAGE: u8 = 0x08;
/// UBX-CFG-NAV5: the navigation engine settings.
const NAVIGATION_MESSAGE: u8 = 0x24;

/// The UART port's id in CFG-PRT (UART1, the one on the module's pins).
const UART1: u8 = 1;
/// CFG-PRT's mode: 8 data bits, no parity, 1 stop bit.
const MODE_8N1: u32 = 0x0000_08D0;
/// CFG-PRT's protocol masks: UBX (bit 0), NMEA (bit 1).
const PROTOCOL_UBX: u16 = 0x0001;
const PROTOCOL_NMEA: u16 = 0x0002;

/// CFG-NAV5's payload: the mask applying the dynamic model alone, then the airborne <4g model (8).
const NAVIGATION_PAYLOAD: usize = 36;
const NAV5_DYNAMIC_MODEL_ONLY: u16 = 0x0001;
const NAV5_AIRBORNE_4G: u8 = 8;

/// A message's rate is one byte (UBX-CFG-MSG): solutions per output.
const RATE_CAP: i32 = 255;

/// The CFG-PRT frame: UART1 at `baud`, 8N1, UBX and NMEA in, UBX out.
pub fn port(baud: u32) -> Vec<u8> {
    let mut payload = [0_u8; 20];
    payload[0] = UART1;
    payload[4..8].copy_from_slice(&MODE_8N1.to_le_bytes());
    payload[8..12].copy_from_slice(&baud.to_le_bytes());
    payload[12..14].copy_from_slice(&(PROTOCOL_UBX | PROTOCOL_NMEA).to_le_bytes());
    payload[14..16].copy_from_slice(&PROTOCOL_UBX.to_le_bytes());
    frame(CONFIGURATION_CLASS, PORT_MESSAGE, &payload)
}

/// The CFG-MSG frame setting `class`/`id`'s rate on the port it arrives on: one output every `every` solutions, 0
/// off.
fn message_rate(class: u8, id: u8, every: i32) -> Vec<u8> {
    frame(
        CONFIGURATION_CLASS,
        MESSAGE_RATE,
        &[class, id, every.clamp(0, RATE_CAP) as u8],
    )
}

/// The sky diagnostics' divider for `hz`: a NAV-SAT every 10 s.
fn diagnostics_every(hz: i32) -> i32 {
    hz.saturating_mul(DIAGNOSTICS_S).min(RATE_CAP)
}

/// The rate `hz` as configured: 1 to [`MAXIMUM_HZ`].
fn clamp(hz: i32) -> i32 {
    hz.clamp(1, MAXIMUM_HZ)
}

/// The setup frames after the port's ([`port`]), at `hz` solutions per second, in send order: the period, the
/// dynamic model, NAV-PVT on every solution, NAV-SAT every 10 s.
pub fn commands(hz: i32) -> [Vec<u8>; 4] {
    let hz = clamp(hz);
    let period_ms = u16::try_from(1000 / hz).expect("1000 / hz fits a u16 for hz >= 1");
    let mut rate = Vec::with_capacity(6);
    for half in [period_ms, 1, 1] {
        rate.extend_from_slice(&half.to_le_bytes()); // measRate ms, navRate cycles, timeRef 1 (GPS)
    }
    let mut navigation = vec![0_u8; NAVIGATION_PAYLOAD];
    navigation[..2].copy_from_slice(&NAV5_DYNAMIC_MODEL_ONLY.to_le_bytes());
    navigation[2] = NAV5_AIRBORNE_4G;
    [
        frame(CONFIGURATION_CLASS, RATE_MESSAGE, &rate),
        frame(CONFIGURATION_CLASS, NAVIGATION_MESSAGE, &navigation),
        message_rate(NAVIGATION_CLASS, PVT, 1),
        message_rate(NAVIGATION_CLASS, SAT, diagnostics_every(hz)),
    ]
}

/// The sky switch for `hz`: NAV-SAT off for the flight, back every 10 s on the pad.
pub fn sky_switch(hz: i32) -> SkySwitch {
    let hz = clamp(hz);
    SkySwitch {
        off: message_rate(NAVIGATION_CLASS, SAT, 0),
        on: message_rate(NAVIGATION_CLASS, SAT, diagnostics_every(hz)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ubx::{Frames, Message};

    /// The one message in `bytes`.
    fn message(bytes: &[u8]) -> Message {
        let mut frames = Frames::new();
        let found: Vec<Message> = bytes.iter().filter_map(|byte| frames.push(*byte)).collect();
        assert_eq!(found.len(), 1, "one frame");
        found.into_iter().next().expect("one frame")
    }

    #[test]
    fn the_port_goes_to_ubx_only_at_the_baud() {
        let port = message(&port(BAUD));
        assert_eq!((port.class, port.id), (CONFIGURATION_CLASS, PORT_MESSAGE));
        let payload = port.payload;
        assert_eq!(payload.len(), 20);
        assert_eq!(payload[0], UART1);
        assert_eq!(u32::from_le_bytes(payload[4..8].try_into().unwrap()), MODE_8N1);
        assert_eq!(u32::from_le_bytes(payload[8..12].try_into().unwrap()), 38_400);
        assert_eq!(
            u16::from_le_bytes([payload[12], payload[13]]),
            0x0003,
            "UBX and NMEA in"
        );
        assert_eq!(u16::from_le_bytes([payload[14], payload[15]]), 0x0001, "UBX alone out");
    }

    #[test]
    fn the_setup_sets_the_rate_the_model_and_the_messages() {
        let [rate, navigation, pvt, sat] = commands(5).map(|bytes| message(&bytes));
        assert_eq!((rate.id, rate.payload), (RATE_MESSAGE, vec![200, 0, 1, 0, 1, 0]));
        assert_eq!(navigation.id, NAVIGATION_MESSAGE);
        assert_eq!(
            (navigation.payload.len(), navigation.payload[0], navigation.payload[2]),
            (36, 1, 8)
        );
        assert_eq!((pvt.id, pvt.payload), (MESSAGE_RATE, vec![NAVIGATION_CLASS, PVT, 1]));
        assert_eq!(
            (sat.id, sat.payload),
            (MESSAGE_RATE, vec![NAVIGATION_CLASS, SAT, 50]),
            "every 10 s at 5 Hz"
        );
    }

    #[test]
    fn the_rate_stays_within_the_module_s() {
        // NEGATIVE: 20 Hz is past the M8N's 10, 0 below 1
        assert_eq!(message(&commands(20)[0]).payload[..2], 100_u16.to_le_bytes());
        assert_eq!(message(&commands(0)[0]).payload[..2], 1000_u16.to_le_bytes());
        assert_eq!(
            message(&commands(20)[3]).payload[2],
            100,
            "the sky every 10 s at the clamped 10 Hz"
        );
    }

    #[test]
    fn the_sky_switch_turns_nav_sat_off_and_on() {
        let switch = sky_switch(5);
        assert_eq!(message(&switch.off).payload, [NAVIGATION_CLASS, SAT, 0]);
        assert_eq!(message(&switch.on).payload, [NAVIGATION_CLASS, SAT, 50]);
    }
}
