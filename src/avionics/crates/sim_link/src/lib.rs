//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The HITL sim-link's two frames (doc/specs/sim-link.md): SENSORS, the host's physics as the board's databoard
//! quantities, and FINS, the board's stage, arming and fins back. Fixed sizes, little-endian, a CRC-32 trailer;
//! encoding and decoding never allocate, so the board's endpoint runs them on its receive path as they are.
//!
//! A decode refuses a frame of the wrong length, magic, version or kind, a bad CRC, or a non-finite float in a
//! present field ([`Refusal`]); the sequence order ([`newer`]) is the receiver's to apply.

use recorder_wire::crc32;

/// Every frame's first four bytes.
pub const MAGIC: [u8; 4] = *b"CSL1";

/// The frame layout's version.
pub const VERSION: u8 = 1;

/// The SENSORS kind byte.
const KIND_SENSORS: u8 = 1;

/// The FINS kind byte.
const KIND_FINS: u8 = 2;

/// The header: magic, version, kind, presence flags.
const HEADER: usize = 8;

/// Why a frame was dropped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// Not the kind's size.
    Length,
    /// Not `CSL1`.
    Magic,
    /// Another layout's version.
    Version,
    /// Not the kind expected.
    Kind,
    /// The CRC-32 does not match.
    Crc,
    /// A present field holds a NaN or an infinity.
    NotFinite,
}

/// Whether sequence number `seq` is newer than `last` across the wrap: `(seq - last) mod 2^32` in 1..2^31.
pub fn newer(seq: u32, last: u32) -> bool {
    let ahead = seq.wrapping_sub(last);
    ahead != 0 && ahead < 1 << 31
}

/// One SENSORS frame: the host's simulated sensors, each None when the host does not send it (its flag clear).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sensors {
    /// +1 per frame, wrapping.
    pub seq: u32,
    /// The physics clock, ms.
    pub sim_time_ms: u32,
    /// g, body axes (x forward, y left, z up).
    pub accel: Option<[f32; 3]>,
    /// deg/s: roll, pitch, yaw.
    pub rate: Option<[f32; 3]>,
    /// degrees: heading, roll, pitch.
    pub attitude: Option<[f32; 3]>,
    /// Calibrated counts.
    pub mag: Option<[f32; 3]>,
    /// m MSL.
    pub altitude: Option<f32>,
    /// m above the pad.
    pub elevation: Option<f32>,
    /// m, the laser's.
    pub agl: Option<f32>,
    /// m/s, the pitot's.
    pub airspeed: Option<f32>,
    /// Pa.
    pub dynamic_pressure: Option<f32>,
    /// 1e-7 degree: latitude, longitude.
    pub position: Option<[i32; 2]>,
    /// m/s over ground.
    pub speed: Option<f32>,
    /// degrees.
    pub course: Option<f32>,
}

impl Sensors {
    /// The frame's size.
    pub const BYTES: usize = 104;

    /// The frame's bytes.
    pub fn encode(&self) -> [u8; Self::BYTES] {
        let mut frame = [0u8; Self::BYTES];
        let mut flags = 0u16;
        let mut put = Writer { frame: &mut frame };
        put.u32(8, self.seq);
        put.u32(12, self.sim_time_ms);
        for (bit, offset, value) in [
            (0, 16, self.accel),
            (1, 28, self.rate),
            (2, 40, self.attitude),
            (3, 52, self.mag),
        ] {
            if let Some(vector) = value {
                flags |= 1 << bit;
                for (index, component) in vector.into_iter().enumerate() {
                    put.f32(offset + 4 * index, component);
                }
            }
        }
        for (bit, offset, value) in [
            (4, 64, self.altitude),
            (5, 68, self.elevation),
            (6, 72, self.agl),
            (7, 76, self.airspeed),
            (8, 80, self.dynamic_pressure),
            (10, 92, self.speed),
            (11, 96, self.course),
        ] {
            if let Some(scalar) = value {
                flags |= 1 << bit;
                put.f32(offset, scalar);
            }
        }
        if let Some([latitude, longitude]) = self.position {
            flags |= 1 << 9;
            put.i32(84, latitude);
            put.i32(88, longitude);
        }
        seal(&mut frame, KIND_SENSORS, flags);
        frame
    }

    /// The frame in `bytes`.
    ///
    /// Errors: the [`Refusal`] that dropped it.
    pub fn decode(bytes: &[u8]) -> Result<Sensors, Refusal> {
        let flags = check(bytes, Self::BYTES, KIND_SENSORS)?;
        let get = Reader { frame: bytes };
        let present = |bit: u16| flags & (1 << bit) != 0;
        let vector = |bit: u16, offset: usize| -> Result<Option<[f32; 3]>, Refusal> {
            if !present(bit) {
                return Ok(None);
            }
            let value = [get.f32(offset), get.f32(offset + 4), get.f32(offset + 8)];
            finite(&value).map(|()| Some(value))
        };
        let scalar = |bit: u16, offset: usize| -> Result<Option<f32>, Refusal> {
            if !present(bit) {
                return Ok(None);
            }
            let value = get.f32(offset);
            finite(&[value]).map(|()| Some(value))
        };
        Ok(Sensors {
            seq: get.u32(8),
            sim_time_ms: get.u32(12),
            accel: vector(0, 16)?,
            rate: vector(1, 28)?,
            attitude: vector(2, 40)?,
            mag: vector(3, 52)?,
            altitude: scalar(4, 64)?,
            elevation: scalar(5, 68)?,
            agl: scalar(6, 72)?,
            airspeed: scalar(7, 76)?,
            dynamic_pressure: scalar(8, 80)?,
            position: present(9).then(|| [get.i32(84), get.i32(88)]),
            speed: scalar(10, 92)?,
            course: scalar(11, 96)?,
        })
    }
}

/// One FINS frame: the board's answer to a SENSORS frame it applied.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Fins {
    /// The SENSORS seq this answers.
    pub seq: u32,
    /// The board's clock at the answer, ms.
    pub uptime_ms: u32,
    /// The stage, 1 SETTING .. 5 DONE (crates/sequencer `Stage`).
    pub stage: u8,
    /// Armed.
    pub armed: bool,
    /// The launch was detected.
    pub launched: bool,
    /// Whole degrees, 90 neutral: left elevon, right elevon, yaw; None while the board drives no fins.
    pub fins: Option<[i16; 3]>,
}

impl Fins {
    /// The frame's size.
    pub const BYTES: usize = 32;

    /// The frame's bytes.
    pub fn encode(&self) -> [u8; Self::BYTES] {
        let mut frame = [0u8; Self::BYTES];
        let mut put = Writer { frame: &mut frame };
        put.u32(8, self.seq);
        put.u32(12, self.uptime_ms);
        put.frame[16] = self.stage;
        put.frame[17] = u8::from(self.armed);
        put.frame[18] = u8::from(self.launched);
        let mut flags = 0u16;
        if let Some(fins) = self.fins {
            flags |= 1;
            for (index, angle) in fins.into_iter().enumerate() {
                put.frame[20 + 2 * index..22 + 2 * index].copy_from_slice(&angle.to_le_bytes());
            }
        }
        seal(&mut frame, KIND_FINS, flags);
        frame
    }

    /// The frame in `bytes`.
    ///
    /// Errors: the [`Refusal`] that dropped it.
    pub fn decode(bytes: &[u8]) -> Result<Fins, Refusal> {
        let flags = check(bytes, Self::BYTES, KIND_FINS)?;
        let get = Reader { frame: bytes };
        let angle = |offset: usize| i16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        Ok(Fins {
            seq: get.u32(8),
            uptime_ms: get.u32(12),
            stage: bytes[16],
            armed: bytes[17] != 0,
            launched: bytes[18] != 0,
            fins: (flags & 1 != 0).then(|| [angle(20), angle(22), angle(24)]),
        })
    }
}

/// Writes little-endian fields into a frame.
struct Writer<'f> {
    /// The frame.
    frame: &'f mut [u8],
}

impl Writer<'_> {
    /// `value` at `offset`.
    fn u32(&mut self, offset: usize, value: u32) {
        self.frame[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    /// `value` at `offset`.
    fn i32(&mut self, offset: usize, value: i32) {
        self.frame[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    /// `value` at `offset`.
    fn f32(&mut self, offset: usize, value: f32) {
        self.frame[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
}

/// Reads little-endian fields from a frame whose length was checked.
struct Reader<'f> {
    /// The frame.
    frame: &'f [u8],
}

impl Reader<'_> {
    /// Four bytes at `offset`.
    fn word(&self, offset: usize) -> [u8; 4] {
        [
            self.frame[offset],
            self.frame[offset + 1],
            self.frame[offset + 2],
            self.frame[offset + 3],
        ]
    }

    /// The u32 at `offset`.
    fn u32(&self, offset: usize) -> u32 {
        u32::from_le_bytes(self.word(offset))
    }

    /// The i32 at `offset`.
    fn i32(&self, offset: usize) -> i32 {
        i32::from_le_bytes(self.word(offset))
    }

    /// The f32 at `offset`.
    fn f32(&self, offset: usize) -> f32 {
        f32::from_le_bytes(self.word(offset))
    }
}

/// Writes the header (`kind`, `flags`) and the CRC-32 trailer over everything before it.
fn seal(frame: &mut [u8], kind: u8, flags: u16) {
    frame[..4].copy_from_slice(&MAGIC);
    frame[4] = VERSION;
    frame[5] = kind;
    frame[6..HEADER].copy_from_slice(&flags.to_le_bytes());
    let body = frame.len() - 4;
    let crc = crc32(0, &frame[..body]);
    frame[body..].copy_from_slice(&crc.to_le_bytes());
}

/// The checks every decode makes, in the spec's order; the presence flags when the frame passes.
fn check(bytes: &[u8], size: usize, kind: u8) -> Result<u16, Refusal> {
    if bytes.len() != size {
        return Err(Refusal::Length);
    }
    if bytes[..4] != MAGIC {
        return Err(Refusal::Magic);
    }
    if bytes[4] != VERSION {
        return Err(Refusal::Version);
    }
    if bytes[5] != kind {
        return Err(Refusal::Kind);
    }
    let body = size - 4;
    if crc32(0, &bytes[..body]).to_le_bytes() != bytes[body..] {
        return Err(Refusal::Crc);
    }
    Ok(u16::from_le_bytes([bytes[6], bytes[7]]))
}

/// Every value finite, else [`Refusal::NotFinite`]: a NaN would poison every reading fused with it.
fn finite(values: &[f32]) -> Result<(), Refusal> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(Refusal::NotFinite)
    }
}

#[cfg(test)]
mod tests;
