//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! CRC-32 as zlib and Python's `binascii.crc32` compute it (IEEE 802.3, the reflected polynomial 0xEDB88320):
//! the wrapper's OPEN tag, and flash_ring's block and chunk checksums. A 256-entry table built at compile time
//! (1 KiB of read-only data) instead of a crate: the wire needs this one function and nothing else from a CRC
//! library.

/// The reflected IEEE 802.3 polynomial, the one zlib, `binascii` and plan B's `_crc_table` use.
const POLYNOMIAL: u32 = 0xEDB8_8320;

/// The byte-at-a-time lookup table, computed by the compiler so nothing builds it at boot.
const TABLE: [u32; 256] = table();

/// The table zlib builds: entry `n` is the register after shifting the byte `n` through eight rounds.
const fn table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut index = 0;
    while index < table.len() {
        let mut value = index as u32;
        let mut round = 0;
        while round < 8 {
            value = if value & 1 == 1 {
                (value >> 1) ^ POLYNOMIAL
            } else {
                value >> 1
            };
            round += 1;
        }
        table[index] = value;
        index += 1;
    }
    table
}

/// The CRC-32 of `bytes`, continuing from the CRC `previous` of the bytes before them, exactly as
/// `binascii.crc32(bytes, previous)` and zlib's `crc32(previous, bytes)`: start a fresh CRC with 0, and
/// `crc32(crc32(0, a), b) == crc32(0, a ++ b)`. Continuing is what lets OPEN cover `<routing>@<payload>`
/// without first copying the pieces into one buffer.
pub fn crc32(previous: u32, bytes: &[u8]) -> u32 {
    let register = bytes.iter().fold(!previous, |register, &byte| {
        TABLE[((register ^ u32::from(byte)) & 0xFF) as usize] ^ (register >> 8)
    });
    !register
}

#[cfg(test)]
mod tests {
    use super::crc32;

    #[test]
    fn the_check_value_matches_zlib() {
        assert_eq!(crc32(0, b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(0, b""), 0);
        assert_eq!(crc32(0, b"a"), 0xE8B7_BE43);
    }

    #[test]
    fn a_crc_continues_across_pieces() {
        let whole = crc32(0, b"000123_imu.csv@884029;0.98");
        assert_eq!(crc32(crc32(crc32(0, b"000123_imu.csv"), b"@"), b"884029;0.98"), whole);
        assert_eq!(crc32(whole, b""), whole, "nothing more leaves it as it is");
    }

    #[test]
    fn one_changed_byte_changes_the_crc() {
        assert_ne!(crc32(0, b"123456789"), crc32(0, b"123456788"));
        assert_ne!(crc32(0, b"12345678"), crc32(0, b"123456789"), "a truncation is caught");
        assert_ne!(crc32(1, b"123456789"), crc32(0, b"123456789"), "so is another start");
    }
}
