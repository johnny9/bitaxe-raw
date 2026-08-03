#![no_std]

pub mod bridge_owner;
pub mod control_protocol;

#[cfg(test)]
mod tests {
    use bonanza_bridge_fw::info;

    #[test]
    fn embedded_bridge_identity_is_protocol_1_0_beta_2() {
        let payload = info::firmware_info().unwrap();

        assert_eq!(&payload[..4], &[1, 1, 0, 12]);
        assert_eq!(&payload[4..], b"0.0.1-beta.2");
    }
}
