pub const MIN_REQUEST_FRAME_LEN: usize = 6;
pub const ERROR_TIMEOUT: u8 = 0x10;
pub const ERROR_INVALID: u8 = 0x11;
pub const ERROR_DENIED: u8 = 0x12;
pub const ERROR_FAULT: u8 = 0x13;
pub const ERROR_EXTENDED: u8 = 0xff;
pub const ERROR_BUFFER_OVERFLOW_DETAIL: &[u8] = b"Buf";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrameError {
    InvalidLength,
}

pub fn request_frame_length(frame: &[u8], maximum: usize) -> Result<Option<usize>, FrameError> {
    if frame.len() < 2 {
        return Ok(None);
    }

    let frame_length = u16::from_le_bytes([frame[0], frame[1]]) as usize;
    if !(MIN_REQUEST_FRAME_LEN..=maximum).contains(&frame_length) {
        return Err(FrameError::InvalidLength);
    }
    if frame.len() < frame_length {
        return Ok(None);
    }
    Ok(Some(frame_length))
}

pub fn response_frame_length(payload_length: usize) -> Option<u16> {
    payload_length.checked_add(3)?.try_into().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waits_for_a_fragmented_header_and_body() {
        assert_eq!(request_frame_length(&[], 4098), Ok(None));
        assert_eq!(request_frame_length(&[6], 4098), Ok(None));
        assert_eq!(request_frame_length(&[6, 0, 1], 4098), Ok(None));
        assert_eq!(request_frame_length(&[6, 0, 1, 0, 6, 2], 4098), Ok(Some(6)));
    }

    #[test]
    fn rejects_invalid_request_lengths() {
        assert_eq!(request_frame_length(&[5, 0], 4098), Err(FrameError::InvalidLength));
        assert_eq!(request_frame_length(&[3, 0], 4098), Err(FrameError::InvalidLength));
        assert_eq!(request_frame_length(&[0xff, 0xff], 4098), Err(FrameError::InvalidLength));
    }

    #[test]
    fn accepts_the_first_frame_from_a_coalesced_buffer() {
        let frames = [6, 0, 1, 0, 6, 2, 6, 0, 2, 0, 6, 3];
        assert_eq!(request_frame_length(&frames, 4098), Ok(Some(6)));
    }

    #[test]
    fn response_length_includes_length_and_id_fields() {
        assert_eq!(response_frame_length(0), Some(3));
        assert_eq!(response_frame_length(1), Some(4));
        assert_eq!(response_frame_length(256), Some(259));
        assert_eq!(response_frame_length(usize::MAX), None);
    }

    #[test]
    fn error_codes_match_bitaxe_raw_esp() {
        assert_eq!(ERROR_TIMEOUT, 0x10);
        assert_eq!(ERROR_INVALID, 0x11);
        assert_eq!(ERROR_DENIED, 0x12);
        assert_eq!(ERROR_FAULT, 0x13);
        assert_eq!([&[ERROR_EXTENDED][..], ERROR_BUFFER_OVERFLOW_DETAIL].concat(), b"\xffBuf");
    }
}
