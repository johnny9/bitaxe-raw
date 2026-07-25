//! Pure host-side byte-pair decoding for the ASIC 9-bit UART.

#[derive(Default)]
pub struct NineBitPairDecoder {
    pending_low_byte: Option<u8>,
}

impl NineBitPairDecoder {
    pub const fn new() -> Self {
        Self { pending_low_byte: None }
    }

    /// Decode `[low byte, bit 8]` pairs into 9-bit words. A pair split across
    /// USB packets is retained. An invalid bit-8 byte becomes the next
    /// low-byte candidate so a dropped byte does not permanently shift the
    /// stream out of phase.
    pub fn decode(&mut self, input: &[u8], output: &mut [u16]) -> usize {
        let mut input_index = 0;
        let mut output_length = 0;

        if let Some(low_byte) = self.pending_low_byte.take() {
            if input.is_empty() || output.is_empty() {
                self.pending_low_byte = Some(low_byte);
                return 0;
            }
            if input[0] <= 1 {
                output[output_length] = low_byte as u16 | ((input[0] as u16) << 8);
                output_length += 1;
            } else {
                self.pending_low_byte = Some(input[0]);
            }
            input_index = 1;
        }

        while input_index < input.len() && output_length < output.len() {
            let low_byte = match self.pending_low_byte.take() {
                Some(low_byte) => low_byte,
                None => {
                    let low_byte = input[input_index];
                    input_index += 1;
                    if input_index == input.len() {
                        self.pending_low_byte = Some(low_byte);
                        break;
                    }
                    low_byte
                }
            };

            let bit_8 = input[input_index];
            input_index += 1;
            if bit_8 <= 1 {
                output[output_length] = low_byte as u16 | ((bit_8 as u16) << 8);
                output_length += 1;
            } else {
                self.pending_low_byte = Some(bit_8);
            }
        }

        if input_index + 1 == input.len() {
            self.pending_low_byte = Some(input[input_index]);
        }

        output_length
    }
}

/// BIRDS and Bonanza responses carry only the low eight ASIC data bits.
pub const fn response_byte(word: u16) -> u8 {
    word as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complete_pairs_decode_to_nine_bit_words() {
        let mut decoder = NineBitPairDecoder::new();
        let mut words = [0u16; 3];
        let count = decoder.decode(&[0x55, 1, 0xaa, 0, 0xff, 1], &mut words);

        assert_eq!(count, 3);
        assert_eq!(words, [0x155, 0x0aa, 0x1ff]);
    }

    #[test]
    fn split_pairs_are_preserved_across_usb_packets() {
        let mut decoder = NineBitPairDecoder::new();
        let mut words = [0u16; 2];

        assert_eq!(decoder.decode(&[0x12], &mut words), 0);
        assert_eq!(decoder.decode(&[1, 0x34, 0], &mut words), 2);
        assert_eq!(words, [0x112, 0x034]);
    }

    #[test]
    fn dropped_bytes_resynchronize_on_a_valid_bit_byte() {
        let mut decoder = NineBitPairDecoder::new();
        let mut words = [0u16; 3];

        let count = decoder.decode(&[0x55, 0xaa, 0, 0xff, 1], &mut words);
        assert_eq!(&words[..count], &[0x0aa, 0x1ff]);
    }

    #[test]
    fn responses_drop_only_the_ninth_bit() {
        assert_eq!(response_byte(0x1a5), 0xa5);
        assert_eq!(response_byte(0x0a5), 0xa5);
    }
}
