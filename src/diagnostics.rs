pub const SYSTEM_GET_INFO: u8 = 0x01;
pub const SYSTEM_GET_RX_STATS: u8 = 0x02;
pub const SYSTEM_GET_SAFETY_STATUS: u8 = 0x10;

pub const INFO_SCHEMA_VERSION: u8 = 1;
pub const PROTOCOL_MAJOR: u8 = 1;
pub const PROTOCOL_MINOR: u8 = 0;
pub const FIRMWARE_VERSION: &str = concat!("bitaxe-birds-raw-", env!("CARGO_PKG_VERSION"));

pub const RX_STATS_SCHEMA_VERSION: u8 = 1;
pub const RX_STATS_LENGTH: usize = 9;

pub const SAFETY_STATUS_SCHEMA_VERSION: u8 = 1;
pub const SAFETY_STATUS_LENGTH: usize = 17;

const SAFETY_STAGE_BOOT_SAFE: u8 = 0;
const SAFETY_STATE_SAFE_OFF: u8 = 0;
const SAFETY_STATE_CONTROLLED: u8 = 1;
const SAFETY_FAULT_NONE: u8 = 0;
const RUNTIME_GOOD_SAFE_OFF: u8 = 0;
const RUNTIME_GOOD_CONTROLLED: u8 = 1;
const RUNTIME_BAD_TRIP_INPUT: u8 = 0x82;
const PRODUCTION_BAD_STAGE_DISABLED: u8 = 0x80;

const CAP_FIVE_VOLT_CONTROL: u16 = 1 << 0;
const CAP_ASIC_RESET_CONTROL: u16 = 1 << 1;
const CAP_FAN_FORCE_FULL: u16 = 1 << 2;
const CAP_TRIP_INPUT_SAMPLED: u16 = 1 << 3;
const CAPABILITIES: u16 = CAP_FIVE_VOLT_CONTROL | CAP_ASIC_RESET_CONTROL | CAP_FAN_FORCE_FULL | CAP_TRIP_INPUT_SAMPLED;

const EVIDENCE_OUTPUTS_SAFE: u16 = 1 << 0;
const EVIDENCE_LEASE_VALID: u16 = 1 << 1;
const EVIDENCE_TRIP_CLEAR: u16 = 1 << 2;
const EVIDENCE_FAULT_CLEAR: u16 = 1 << 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SafetySnapshot {
    pub five_volt_enabled: bool,
    pub asic_reset_asserted: bool,
    pub fan_percent: u8,
    pub trip_input_asserted: bool,
}

pub fn encode_info(output: &mut [u8]) -> Option<usize> {
    let length = FIRMWARE_VERSION.len().checked_add(4)?;
    if FIRMWARE_VERSION.len() > 63 || output.len() < length {
        return None;
    }

    output[..4].copy_from_slice(&[INFO_SCHEMA_VERSION, PROTOCOL_MAJOR, PROTOCOL_MINOR, FIRMWARE_VERSION.len() as u8]);
    output[4..length].copy_from_slice(FIRMWARE_VERSION.as_bytes());
    Some(length)
}

pub fn encode_rx_stats(pio_fifo_overflows: u32, software_ring_overflows: u32) -> [u8; RX_STATS_LENGTH] {
    let mut payload = [0; RX_STATS_LENGTH];
    payload[0] = RX_STATS_SCHEMA_VERSION;
    payload[1..5].copy_from_slice(&pio_fifo_overflows.to_le_bytes());
    payload[5..9].copy_from_slice(&software_ring_overflows.to_le_bytes());
    payload
}

pub fn encode_safety_status(snapshot: SafetySnapshot) -> [u8; SAFETY_STATUS_LENGTH] {
    let fan_percent = snapshot.fan_percent.min(100);
    let outputs_safe = !snapshot.five_volt_enabled && snapshot.asic_reset_asserted && fan_percent == 100;
    let state = if outputs_safe { SAFETY_STATE_SAFE_OFF } else { SAFETY_STATE_CONTROLLED };
    let runtime_verdict = if snapshot.trip_input_asserted {
        RUNTIME_BAD_TRIP_INPUT
    } else if outputs_safe {
        RUNTIME_GOOD_SAFE_OFF
    } else {
        RUNTIME_GOOD_CONTROLLED
    };

    let mut evidence = EVIDENCE_LEASE_VALID | EVIDENCE_FAULT_CLEAR;
    if outputs_safe {
        evidence |= EVIDENCE_OUTPUTS_SAFE;
    }
    if !snapshot.trip_input_asserted {
        evidence |= EVIDENCE_TRIP_CLEAR;
    }

    let mut payload = [0; SAFETY_STATUS_LENGTH];
    payload[..6].copy_from_slice(&[SAFETY_STATUS_SCHEMA_VERSION, SAFETY_STAGE_BOOT_SAFE, state, SAFETY_FAULT_NONE, runtime_verdict, PRODUCTION_BAD_STAGE_DISABLED]);
    payload[6..8].copy_from_slice(&CAPABILITIES.to_le_bytes());
    payload[8..10].copy_from_slice(&evidence.to_le_bytes());
    payload[10..14].copy_from_slice(&0u32.to_le_bytes());
    payload[14] = u8::from(snapshot.five_volt_enabled) | (u8::from(snapshot.asic_reset_asserted) << 1) | (u8::from(fan_percent == 100) << 2);
    payload[15] = fan_percent;
    payload[16] = u8::from(snapshot.trip_input_asserted);
    payload
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn info_matches_the_esp_schema() {
        let mut payload = [0; 67];
        let length = encode_info(&mut payload).unwrap();

        assert_eq!(&payload[..4], &[1, 1, 0, FIRMWARE_VERSION.len() as u8]);
        assert_eq!(&payload[4..length], FIRMWARE_VERSION.as_bytes());
    }

    #[test]
    fn rx_stats_match_the_esp_schema() {
        assert_eq!(encode_rx_stats(0x1234_5678, 0x90ab_cdef), [1, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x90]);
    }

    #[test]
    fn safe_status_is_coherent_and_lease_free() {
        let payload = encode_safety_status(SafetySnapshot {
            five_volt_enabled: false,
            asic_reset_asserted: true,
            fan_percent: 100,
            trip_input_asserted: false,
        });

        assert_eq!(payload, [1, 0, 0, 0, 0, 0x80, 0x0f, 0, 0x0f, 0, 0, 0, 0, 0, 0x06, 100, 0]);
    }

    #[test]
    fn active_outputs_report_a_controlled_state() {
        let payload = encode_safety_status(SafetySnapshot {
            five_volt_enabled: true,
            asic_reset_asserted: false,
            fan_percent: 50,
            trip_input_asserted: false,
        });

        assert_eq!(payload[2], SAFETY_STATE_CONTROLLED);
        assert_eq!(payload[4], RUNTIME_GOOD_CONTROLLED);
        assert_eq!(payload[8], 0x0e);
        assert_eq!(&payload[10..14], &[0, 0, 0, 0]);
        assert_eq!(&payload[14..], &[0x01, 50, 0]);
    }

    #[test]
    fn active_trip_is_visible_without_inventing_a_latched_fault() {
        let payload = encode_safety_status(SafetySnapshot {
            five_volt_enabled: false,
            asic_reset_asserted: true,
            fan_percent: 100,
            trip_input_asserted: true,
        });

        assert_eq!(payload[3], SAFETY_FAULT_NONE);
        assert_eq!(payload[4], RUNTIME_BAD_TRIP_INPUT);
        assert_eq!(payload[8], EVIDENCE_OUTPUTS_SAFE as u8 | EVIDENCE_LEASE_VALID as u8 | EVIDENCE_FAULT_CLEAR as u8);
        assert_eq!(payload[16], 1);
    }
}
