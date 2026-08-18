const POLYNOMIAL: u16 = 0x1021;
const INITIAL_VALUE: u16 = 0xffff;

/// CRC-16/CCITT-FALSE used by protocol version 1.
pub(crate) fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = INITIAL_VALUE;

    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ POLYNOMIAL
            } else {
                crc << 1
            };
        }
    }

    crc
}

#[cfg(test)]
mod tests {
    use super::crc16_ccitt_false;

    #[test]
    fn matches_ccitt_false_check_value() {
        assert_eq!(crc16_ccitt_false(b"123456789"), 0x29b1);
    }
}
