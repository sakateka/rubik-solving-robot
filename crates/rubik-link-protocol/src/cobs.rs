use crate::wire::WireError;

pub(crate) fn max_encoded_len(input_len: usize) -> usize {
    input_len + input_len / 254 + 1
}

pub(crate) fn encode(input: &[u8], output: &mut [u8]) -> Result<usize, WireError> {
    if output.len() < max_encoded_len(input.len()) {
        return Err(WireError::OutputTooSmall);
    }

    let mut read = 0;
    let mut write = 1;
    let mut code_index = 0;
    let mut code = 1u8;

    while read < input.len() {
        if input[read] == 0 {
            output[code_index] = code;
            code_index = write;
            write += 1;
            code = 1;
        } else {
            output[write] = input[read];
            write += 1;
            code += 1;

            if code == 0xff {
                output[code_index] = code;
                code_index = write;
                write += 1;
                code = 1;
            }
        }
        read += 1;
    }

    output[code_index] = code;
    Ok(write)
}

pub(crate) fn decode(input: &[u8], output: &mut [u8]) -> Result<usize, WireError> {
    if input.is_empty() {
        return Err(WireError::MalformedCobs);
    }

    let mut read = 0;
    let mut write = 0;

    while read < input.len() {
        let code = input[read];
        if code == 0 {
            return Err(WireError::MalformedCobs);
        }
        read += 1;

        let block_len = usize::from(code) - 1;
        if read + block_len > input.len() || write + block_len > output.len() {
            return Err(if read + block_len > input.len() {
                WireError::MalformedCobs
            } else {
                WireError::OutputTooSmall
            });
        }

        output[write..write + block_len].copy_from_slice(&input[read..read + block_len]);
        read += block_len;
        write += block_len;

        if code != 0xff && read < input.len() {
            if write == output.len() {
                return Err(WireError::OutputTooSmall);
            }
            output[write] = 0;
            write += 1;
        }
    }

    Ok(write)
}

#[cfg(test)]
mod tests {
    use super::{decode, encode, max_encoded_len};

    fn round_trip(input: &[u8]) {
        let mut encoded = [0u8; 520];
        let mut decoded = [0u8; 512];
        let encoded_len = encode(input, &mut encoded).unwrap();
        assert!(!encoded[..encoded_len].contains(&0));
        let decoded_len = decode(&encoded[..encoded_len], &mut decoded).unwrap();
        assert_eq!(&decoded[..decoded_len], input);
    }

    #[test]
    fn round_trips_empty_and_zero_heavy_inputs() {
        round_trip(&[]);
        round_trip(&[0]);
        round_trip(&[0, 0, 1, 0, 2, 0]);
    }

    #[test]
    fn round_trips_full_code_block() {
        let input = [0x55; 254];
        round_trip(&input);
        assert_eq!(max_encoded_len(input.len()), 256);
    }
}
