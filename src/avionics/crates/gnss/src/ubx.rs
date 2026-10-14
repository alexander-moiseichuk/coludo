//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! u-blox's binary protocol (UBX), the NEO-M8N's delivery (M16): a frame is `µb`, the class and the id, the
//! little-endian payload length, the payload, and the 8-bit Fletcher checksum over class..payload ([`frame`] builds
//! one, [`Frames`] finds them in the byte stream). What the receiver reads of it:
//! * UBX-NAV-PVT ([`Pvt`]): the whole navigation solution in one message per epoch -- the time, the fix type and
//!   flags, the satellites used, the position already in 1e-7 degree integers, the heights in mm, the NED velocity,
//!   the ground speed and the heading of motion, the position DOP. It replaces NMEA's RMC + GGA pair: one message, so
//!   no epoch is ever half delivered, and no text to parse;
//! * UBX-NAV-SAT ([`Sat`]): every satellite tracked, its C/N0 and whether the solution uses it: the sky diagnostics
//!   (NMEA's GSV + GSA);
//! * UBX-ACK-ACK / ACK-NAK: the module's answer to each configuration frame;
//! * UBX-MON-VER ([`Version`]): the module's own name for itself, its answer to the poll [`frame`] with no payload
//!   makes: proof that it hears the board, and which receiver it is (a clone shows here).
//!
//! The layouts are the u-blox 8 / M8 Receiver Description's (protocol 15 and later). Integers only: the position
//! never passes through a float (`Position`'s 1e-7 degree, as the NMEA path keeps it).

use control::navigation::Position;

/// UBX's two sync characters, `µb`.
pub const SYNC: [u8; 2] = [0xB5, 0x62];

/// UBX-NAV, the navigation results class.
pub const NAVIGATION_CLASS: u8 = 0x01;
/// UBX-NAV-PVT: the navigation solution.
pub const PVT: u8 = 0x07;
/// UBX-NAV-SAT: the satellites.
pub const SAT: u8 = 0x35;
/// UBX-ACK, the acknowledgement class.
pub const ACKNOWLEDGEMENT_CLASS: u8 = 0x05;
/// UBX-ACK-ACK: the configuration frame was taken.
pub const ACK: u8 = 0x01;
/// UBX-ACK-NAK: the configuration frame was refused.
pub const NAK: u8 = 0x00;
/// UBX-CFG, the configuration class.
pub const CONFIGURATION_CLASS: u8 = 0x06;
/// UBX-MON, the monitoring class.
pub const MONITOR_CLASS: u8 = 0x0A;
/// UBX-MON-VER: the receiver's software, hardware and protocol versions.
pub const VERSION: u8 = 0x04;

/// NAV-PVT's payload length.
const PVT_BYTES: usize = 92;
/// NAV-SAT's header length; each satellite adds [`SAT_BLOCK_BYTES`].
const SAT_HEADER_BYTES: usize = 8;
/// One satellite's block in NAV-SAT.
const SAT_BLOCK_BYTES: usize = 12;
/// The longest payload [`Frames`] keeps: NAV-SAT with 100 satellites is 1208 bytes; a longer length is noise.
pub const MAXIMUM_PAYLOAD: usize = 1280;

/// NAV-PVT's `flags` bit 0: gnssFixOK, a fix within the DOP and accuracy masks.
const FIX_OK: u8 = 0x01;
/// NAV-PVT's `flags` bit 1: diffSoln, differential corrections applied.
const DIFFERENTIAL: u8 = 0x02;
/// NAV-SAT's per-satellite `flags` bit 3: svUsed, used in the navigation solution.
const SATELLITE_USED: u32 = 0x08;

/// The 8-bit Fletcher checksum over `bytes` (class..payload).
fn checksum(bytes: &[u8]) -> [u8; 2] {
    let (first, second) = bytes.iter().fold((0_u8, 0_u8), |(first, second), byte| {
        let first = first.wrapping_add(*byte);
        (first, second.wrapping_add(first))
    });
    [first, second]
}

/// One UBX frame of `class`/`id` around `payload`.
///
/// Panics when the payload is longer than UBX's u16 length can say (65535 bytes).
pub fn frame(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let length = u16::try_from(payload.len()).expect("a UBX payload fits its u16 length");
    let mut frame = Vec::with_capacity(payload.len() + 8);
    frame.extend_from_slice(&SYNC);
    frame.extend_from_slice(&[class, id]);
    frame.extend_from_slice(&length.to_le_bytes());
    frame.extend_from_slice(payload);
    let check = checksum(&frame[SYNC.len()..]);
    frame.extend_from_slice(&check);
    frame
}

/// A frame's class, id and payload, its checksum passed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Message {
    /// The class.
    pub class: u8,
    /// The id within the class.
    pub id: u8,
    /// The payload.
    pub payload: Vec<u8>,
}

/// Where [`Frames`] is in a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// Looking for `µ`.
    Sync,
    /// `µ` seen: `b` next.
    Sync2,
    /// The class next.
    Class,
    /// The id next.
    Id,
    /// The length's low byte next.
    LengthLow,
    /// The length's high byte next.
    LengthHigh,
    /// The payload's bytes.
    Payload,
    /// The checksum's first byte.
    CheckFirst,
    /// The checksum's second byte.
    CheckSecond,
}

/// The UBX frames in a byte stream, found byte by byte: anything between frames (the module's NMEA before its
/// configuration took, line noise) is skipped, and a frame whose checksum fails is dropped and counted.
#[derive(Clone, Debug)]
pub struct Frames {
    /// Where in a frame.
    state: State,
    /// The frame so far: class, id, the length's two bytes, the payload.
    body: Vec<u8>,
    /// The payload's announced length.
    length: usize,
    /// The checksum's first byte, as received.
    check: u8,
    /// Frames dropped on their checksum or an impossible length.
    rejected: u32,
}

impl Default for Frames {
    fn default() -> Frames {
        Frames::new()
    }
}

impl Frames {
    /// Nothing heard yet.
    pub fn new() -> Frames {
        Frames {
            state: State::Sync,
            body: Vec::with_capacity(64),
            length: 0,
            check: 0,
            rejected: 0,
        }
    }

    /// One byte: the frame it completes, if any.
    pub fn push(&mut self, byte: u8) -> Option<Message> {
        match self.state {
            State::Sync => {
                if byte == SYNC[0] {
                    self.state = State::Sync2;
                }
            }
            State::Sync2 => {
                self.state = match byte {
                    _ if byte == SYNC[1] => {
                        self.body.clear();
                        State::Class
                    }
                    _ if byte == SYNC[0] => State::Sync2,
                    _ => State::Sync,
                };
            }
            State::Class | State::Id | State::LengthLow => {
                self.body.push(byte);
                self.state = match self.state {
                    State::Class => State::Id,
                    State::Id => State::LengthLow,
                    _ => State::LengthHigh,
                };
            }
            State::LengthHigh => {
                self.body.push(byte);
                self.length = usize::from(u16::from_le_bytes([self.body[2], self.body[3]]));
                self.state = if self.length > MAXIMUM_PAYLOAD {
                    self.rejected = self.rejected.wrapping_add(1);
                    State::Sync
                } else if self.length == 0 {
                    State::CheckFirst
                } else {
                    State::Payload
                };
            }
            State::Payload => {
                self.body.push(byte);
                if self.body.len() == 4 + self.length {
                    self.state = State::CheckFirst;
                }
            }
            State::CheckFirst => {
                self.check = byte;
                self.state = State::CheckSecond;
            }
            State::CheckSecond => {
                self.state = State::Sync;
                if checksum(&self.body) != [self.check, byte] {
                    self.rejected = self.rejected.wrapping_add(1);
                    return None;
                }
                return Some(Message {
                    class: self.body[0],
                    id: self.body[1],
                    payload: self.body[4..].to_vec(),
                });
            }
        }
        None
    }

    /// Frames dropped so far: a failed checksum, or a length past [`MAXIMUM_PAYLOAD`].
    pub fn rejected(&self) -> u32 {
        self.rejected
    }
}

/// The little-endian integers at `offset` of a payload whose length was checked.
fn u16_at(payload: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([payload[offset], payload[offset + 1]])
}

fn i32_at(payload: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes([
        payload[offset],
        payload[offset + 1],
        payload[offset + 2],
        payload[offset + 3],
    ])
}

fn u32_at(payload: &[u8], offset: usize) -> u32 {
    i32_at(payload, offset) as u32
}

/// One UBX-NAV-PVT: the navigation solution of one epoch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pvt {
    /// The GPS time of week of the epoch, ms.
    pub time_of_week_ms: u32,
    /// The fix type: 0 none, 1 dead reckoning only, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only.
    pub fix_type: u8,
    /// `flags`: bit 0 gnssFixOK, bit 1 diffSoln.
    pub flags: u8,
    /// Satellites used in the solution.
    pub satellites: u8,
    /// The position.
    pub position: Position,
    /// The height above mean sea level, mm.
    pub altitude_mm: i32,
    /// The horizontal accuracy estimate, mm.
    pub horizontal_accuracy_mm: u32,
    /// The ground speed, mm/s.
    pub ground_speed_mm_s: i32,
    /// The heading of motion, 1e-5 degree.
    pub heading_e5: i32,
    /// The position DOP, hundredths.
    pub position_dop_centi: u16,
}

impl Pvt {
    /// The payload of a NAV-PVT, None when it is not one (a short or long payload).
    pub fn decode(payload: &[u8]) -> Option<Pvt> {
        (payload.len() == PVT_BYTES).then(|| Pvt {
            time_of_week_ms: u32_at(payload, 0),
            fix_type: payload[20],
            flags: payload[21],
            satellites: payload[23],
            position: Position {
                latitude_e7: i32_at(payload, 28),
                longitude_e7: i32_at(payload, 24),
            },
            altitude_mm: i32_at(payload, 36),
            horizontal_accuracy_mm: u32_at(payload, 40),
            ground_speed_mm_s: i32_at(payload, 60),
            heading_e5: i32_at(payload, 64),
            position_dop_centi: u16_at(payload, 76),
        })
    }

    /// Whether the solution is a fix to use: gnssFixOK, and a 2D, 3D or GNSS + dead-reckoning fix (NMEA RMC's `A`).
    pub fn fixed(&self) -> bool {
        self.flags & FIX_OK != 0 && matches!(self.fix_type, 2..=4)
    }

    /// The fix quality as GGA spells it: 0 none, 1 GPS, 2 differential.
    pub fn quality(&self) -> i32 {
        match (self.fixed(), self.flags & DIFFERENTIAL != 0) {
            (false, _) => 0,
            (true, false) => 1,
            (true, true) => 2,
        }
    }

    /// The fix mode as GSA spells it: 1 none, 2 2D, 3 3D.
    pub fn mode(&self) -> i32 {
        match self.fix_type {
            2 => 2,
            3 | 4 => 3,
            _ => 1,
        }
    }
}

/// One UBX-NAV-SAT, folded: what the sky row reads of it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sat {
    /// Satellites the receiver knows of (GSV's in view).
    pub in_view: i32,
    /// Satellites the solution uses.
    pub used: i32,
    /// The four strongest C/N0 of the satellites heard, dB-Hz, strongest first (fewer when fewer were heard).
    pub strongest: Vec<i32>,
}

impl Sat {
    /// The payload of a NAV-SAT, None when its length does not match its count.
    pub fn decode(payload: &[u8]) -> Option<Sat> {
        let count = usize::from(*payload.get(5)?);
        if payload.len() != SAT_HEADER_BYTES + count * SAT_BLOCK_BYTES {
            return None;
        }
        let (blocks, _) = payload[SAT_HEADER_BYTES..].as_chunks::<SAT_BLOCK_BYTES>();
        let used = blocks
            .iter()
            .filter(|block| u32_at(block.as_slice(), 8) & SATELLITE_USED != 0)
            .count();
        // a C/N0 of 0 is a satellite known but not heard: no signal to rank, as GSV's empty field
        let mut strongest: Vec<i32> = blocks
            .iter()
            .map(|block| i32::from(block[2]))
            .filter(|cn0| *cn0 > 0)
            .collect();
        strongest.sort_unstable_by(|first, second| second.cmp(first));
        strongest.truncate(4);
        Some(Sat {
            in_view: count as i32,
            used: used as i32,
            strongest,
        })
    }
}

/// The name a configuration message (UBX-CFG `id`) goes by in the u-blox manual, for the log: `CFG-PRT`; a hex id for
/// one the drivers do not send.
pub fn configuration_name(id: u8) -> String {
    match id {
        0x00 => "CFG-PRT".to_string(),
        0x01 => "CFG-MSG".to_string(),
        0x08 => "CFG-RATE".to_string(),
        0x24 => "CFG-NAV5".to_string(),
        other => format!("CFG-{other:#04x}"),
    }
}

/// One UBX-MON-VER: the module's software and hardware versions and its extension strings (`PROTVER=18.00`,
/// `MOD=NEO-M8N`, ...).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Version {
    /// The software version (`ROM CORE 3.01 (107888)`).
    pub software: String,
    /// The hardware version (`00080000`).
    pub hardware: String,
    /// The extensions, in order.
    pub extensions: Vec<String>,
}

impl Version {
    /// The software field's bytes, the hardware's, and each extension's.
    const SOFTWARE_BYTES: usize = 30;
    const HARDWARE_BYTES: usize = 10;
    const EXTENSION_BYTES: usize = 30;

    /// The payload of a MON-VER, None when its length is not 40 plus whole extensions.
    pub fn decode(payload: &[u8]) -> Option<Version> {
        let head = Self::SOFTWARE_BYTES + Self::HARDWARE_BYTES;
        if payload.len() < head || !(payload.len() - head).is_multiple_of(Self::EXTENSION_BYTES) {
            return None;
        }
        // each field a NUL-terminated string in its fixed width
        let text = |bytes: &[u8]| {
            let end = bytes.iter().position(|byte| *byte == 0).unwrap_or(bytes.len());
            String::from_utf8_lossy(&bytes[..end]).trim().to_string()
        };
        Some(Version {
            software: text(&payload[..Self::SOFTWARE_BYTES]),
            hardware: text(&payload[Self::SOFTWARE_BYTES..head]),
            extensions: payload[head..]
                .as_chunks::<{ Self::EXTENSION_BYTES }>()
                .0
                .iter()
                .map(|extension| text(extension.as_slice()))
                .collect(),
        })
    }
}

/// The module's answer to a configuration frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Answer {
    /// Taken: the frame's class and id.
    Acknowledged(u8, u8),
    /// Refused: the frame's class and id.
    Refused(u8, u8),
}

impl Answer {
    /// The ACK class's message `id` with `payload`, None for anything else.
    pub fn decode(id: u8, payload: &[u8]) -> Option<Answer> {
        let [class, message] = payload.try_into().ok()?;
        match id {
            ACK => Some(Answer::Acknowledged(class, message)),
            NAK => Some(Answer::Refused(class, message)),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A NAV-PVT payload: `fix_type` with `flags`, 9 satellites, 48.1173 N 11.5167 E, 545.4 m MSL, 1.5 m/s heading
    /// 270.5 degrees, PDOP 1.25.
    pub(crate) fn pvt_payload(fix_type: u8, flags: u8) -> Vec<u8> {
        let mut payload = vec![0_u8; 92];
        payload[0..4].copy_from_slice(&123_456_u32.to_le_bytes());
        payload[20] = fix_type;
        payload[21] = flags;
        payload[23] = 9;
        payload[24..28].copy_from_slice(&115_167_000_i32.to_le_bytes());
        payload[28..32].copy_from_slice(&481_173_000_i32.to_le_bytes());
        payload[36..40].copy_from_slice(&545_400_i32.to_le_bytes());
        payload[40..44].copy_from_slice(&2_500_u32.to_le_bytes());
        payload[60..64].copy_from_slice(&1_500_i32.to_le_bytes());
        payload[64..68].copy_from_slice(&27_050_000_i32.to_le_bytes());
        payload[76..78].copy_from_slice(&125_u16.to_le_bytes());
        payload
    }

    /// A NAV-SAT payload of satellites (C/N0, used).
    pub(crate) fn sat_payload(satellites: &[(u8, bool)]) -> Vec<u8> {
        let mut payload = vec![0_u8; 8];
        payload[4] = 1; // version
        payload[5] = satellites.len() as u8;
        for (index, (cn0, used)) in satellites.iter().enumerate() {
            let mut block = [0_u8; 12];
            block[1] = index as u8 + 1;
            block[2] = *cn0;
            block[8..12].copy_from_slice(&(if *used { SATELLITE_USED | 0x7 } else { 0x1 }).to_le_bytes());
            payload.extend_from_slice(&block);
        }
        payload
    }

    /// Every message `bytes` holds, through one parser.
    fn messages(frames: &mut Frames, bytes: &[u8]) -> Vec<Message> {
        bytes.iter().filter_map(|byte| frames.push(*byte)).collect()
    }

    #[test]
    fn a_frame_comes_back_whole() {
        let bytes = frame(NAVIGATION_CLASS, PVT, &pvt_payload(3, 1));
        let mut frames = Frames::new();
        let found = messages(&mut frames, &bytes);
        assert_eq!(found.len(), 1);
        assert_eq!((found[0].class, found[0].id), (NAVIGATION_CLASS, PVT));
        assert_eq!(found[0].payload, pvt_payload(3, 1));
        assert_eq!(frames.rejected(), 0);
    }

    #[test]
    fn the_checksum_is_u_blox_s() {
        // UBX-CFG-RATE 1 Hz, as u-center sends it
        assert_eq!(
            frame(CONFIGURATION_CLASS, 0x08, &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]),
            [
                0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39
            ]
        );
    }

    #[test]
    fn nmea_and_noise_between_frames_are_skipped() {
        // the module's NMEA before its port took UBX only, a stray `µ`, then two frames back to back
        let mut bytes = b"$GPRMC,,V,,,,,,,,,,N*53\r\n\xB5\x00".to_vec();
        bytes.extend(frame(ACKNOWLEDGEMENT_CLASS, ACK, &[0x06, 0x00]));
        bytes.extend(frame(NAVIGATION_CLASS, SAT, &sat_payload(&[(40, true)])));
        let mut frames = Frames::new();
        let found = messages(&mut frames, &bytes);
        assert_eq!(found.iter().map(|message| message.id).collect::<Vec<_>>(), [ACK, SAT]);
    }

    #[test]
    fn a_damaged_frame_is_dropped_and_counted() {
        // NEGATIVE: one byte of the payload flipped; the parser finds the next frame all the same
        let mut damaged = frame(NAVIGATION_CLASS, PVT, &pvt_payload(3, 1));
        damaged[30] ^= 0x01;
        damaged.extend(frame(ACKNOWLEDGEMENT_CLASS, NAK, &[0x06, 0x01]));
        let mut frames = Frames::new();
        let found = messages(&mut frames, &damaged);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].id, NAK);
        assert_eq!(frames.rejected(), 1);
    }

    #[test]
    fn an_impossible_length_is_noise() {
        // NEGATIVE: a length past the largest NAV-SAT: dropped at once, the next frame found
        let mut bytes = vec![0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF];
        bytes.extend(frame(ACKNOWLEDGEMENT_CLASS, ACK, &[0x06, 0x08]));
        let mut frames = Frames::new();
        assert_eq!(messages(&mut frames, &bytes).len(), 1);
        assert_eq!(frames.rejected(), 1);
    }

    #[test]
    fn a_pvt_decodes_to_integers() {
        let pvt = Pvt::decode(&pvt_payload(3, FIX_OK)).expect("a NAV-PVT");
        assert_eq!(
            pvt.position,
            Position {
                latitude_e7: 481_173_000,
                longitude_e7: 115_167_000
            }
        );
        assert_eq!(
            (
                pvt.time_of_week_ms,
                pvt.satellites,
                pvt.altitude_mm,
                pvt.position_dop_centi
            ),
            (123_456, 9, 545_400, 125)
        );
        assert_eq!(
            (pvt.ground_speed_mm_s, pvt.heading_e5, pvt.horizontal_accuracy_mm),
            (1_500, 27_050_000, 2_500)
        );
        assert!(pvt.fixed());
        assert_eq!((pvt.quality(), pvt.mode()), (1, 3));
        let differential = Pvt::decode(&pvt_payload(3, FIX_OK | DIFFERENTIAL)).expect("a NAV-PVT");
        assert_eq!(differential.quality(), 2);
        assert_eq!(Pvt::decode(&pvt_payload(2, FIX_OK)).map(|pvt| pvt.mode()), Some(2));
    }

    #[test]
    fn a_pvt_without_gnss_fix_ok_is_no_fix() {
        // NEGATIVE: a 3D solution outside the masks (gnssFixOK clear), dead reckoning alone, time only
        for (fix_type, flags) in [(3, 0), (1, FIX_OK), (5, FIX_OK), (0, FIX_OK)] {
            let pvt = Pvt::decode(&pvt_payload(fix_type, flags)).expect("a NAV-PVT");
            assert!(!pvt.fixed(), "fix type {fix_type} flags {flags}");
            assert_eq!(pvt.quality(), 0);
        }
    }

    #[test]
    fn a_short_or_long_pvt_is_none() {
        // NEGATIVE: protocol 14's 84-byte PVT, and one byte too many
        assert_eq!(Pvt::decode(&[0; 84]), None);
        assert_eq!(Pvt::decode(&[0; 93]), None);
    }

    #[test]
    fn a_sat_counts_the_sky() {
        let sat = Sat::decode(&sat_payload(&[
            (40, true),
            (0, false),
            (35, true),
            (22, false),
            (44, false),
            (31, false),
        ]))
        .expect("a NAV-SAT");
        assert_eq!((sat.in_view, sat.used), (6, 2));
        assert_eq!(
            sat.strongest,
            [44, 40, 35, 31],
            "the four strongest heard, a silent one left out"
        );
        assert_eq!(Sat::decode(&sat_payload(&[])).map(|sat| sat.in_view), Some(0));
    }

    #[test]
    fn a_sat_whose_count_lies_is_none() {
        // NEGATIVE: three satellites announced, two carried; and no header at all
        let mut payload = sat_payload(&[(40, true), (30, false)]);
        payload[5] = 3;
        assert_eq!(Sat::decode(&payload), None);
        assert_eq!(Sat::decode(&[0; 4]), None);
    }

    #[test]
    fn a_mon_ver_names_the_module() {
        let mut payload = vec![0_u8; 100];
        payload[..21].copy_from_slice(b"ROM CORE 3.01 (107888)"[..21].as_ref());
        payload[30..38].copy_from_slice(b"00080000");
        payload[40..53].copy_from_slice(b"PROTVER=18.00");
        payload[70..80].copy_from_slice(b"MOD=NEO-M8");
        let version = Version::decode(&payload).expect("a MON-VER");
        assert_eq!(version.software, "ROM CORE 3.01 (107888");
        assert_eq!(version.hardware, "00080000");
        assert_eq!(version.extensions, ["PROTVER=18.00", "MOD=NEO-M8"]);
        // a u-blox 6 has no extensions
        assert_eq!(
            Version::decode(&[0; 40]).map(|version| version.extensions.len()),
            Some(0)
        );
        // NEGATIVE: a payload of no whole extension, and one too short for the versions
        assert_eq!(Version::decode(&[0; 41]), None);
        assert_eq!(Version::decode(&[0; 39]), None);
    }

    #[test]
    fn acks_name_the_frame_they_answer() {
        assert_eq!(
            Answer::decode(ACK, &[0x06, 0x00]),
            Some(Answer::Acknowledged(0x06, 0x00))
        );
        assert_eq!(Answer::decode(NAK, &[0x06, 0x24]), Some(Answer::Refused(0x06, 0x24)));
        // NEGATIVE: another id, a payload of the wrong size
        assert_eq!(Answer::decode(0x02, &[0x06, 0x00]), None);
        assert_eq!(Answer::decode(ACK, &[0x06]), None);
    }
}
