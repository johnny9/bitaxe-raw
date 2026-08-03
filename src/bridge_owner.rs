//! Transparent ownership of the embedded Bonanza Bridge safety policy.
//!
//! A normal BitaxeBonanza runs `bitaxe-raw-bonanza` on the ESP32-S3 and the
//! bridge policy on a separate RP2040. The BIRDS reference device has only the
//! RP2040, so this owner preserves the same lease, trip-latch, sequencing, and
//! fail-safe behavior inside one image.

use bonanza_bridge_fw::safety::{SafetyConfig, SafetyError, SafetyOutputs, SafetyPolicy, SafetyStatus};

pub const HEARTBEAT_INTERVAL_MS: u64 = 250;

#[derive(Debug)]
pub struct BridgeOwner {
    policy: SafetyPolicy,
    lease_owned: bool,
    next_heartbeat_ms: u64,
}

impl Default for BridgeOwner {
    fn default() -> Self {
        Self::new()
    }
}

impl BridgeOwner {
    pub const fn new() -> Self {
        Self {
            policy: SafetyPolicy::new(SafetyConfig::firmware()),
            lease_owned: false,
            next_heartbeat_ms: 0,
        }
    }

    pub fn service(&mut self, now_ms: u64, trip_input_asserted: bool) {
        self.policy.tick(now_ms, trip_input_asserted);

        if self.lease_owned && now_ms >= self.next_heartbeat_ms {
            if self.policy.heartbeat(now_ms).is_ok() {
                self.next_heartbeat_ms = now_ms.saturating_add(HEARTBEAT_INTERVAL_MS);
            } else {
                self.fail_safe();
            }
        }
    }

    pub const fn outputs(&self) -> SafetyOutputs {
        self.policy.outputs()
    }

    pub fn status(&self, now_ms: u64) -> SafetyStatus {
        self.policy.status(now_ms)
    }

    pub fn request_five_volt_enabled(&mut self, enabled: bool, now_ms: u64, trip_input_asserted: bool) -> Result<(), SafetyError> {
        self.service(now_ms, trip_input_asserted);
        if enabled {
            if let Err(error) = self.ensure_controlled(now_ms) {
                self.fail_safe();
                return Err(error);
            }
            if self.policy.outputs().five_volt_enabled {
                return Ok(());
            }
        }

        if let Err(error) = self.policy.request_five_volt_enabled(enabled, now_ms) {
            self.fail_safe();
            return Err(error);
        }

        // This mirrors the ESP raw firmware: disabling 5 V ends ownership of
        // the bridge and leaves every bridge-controlled output safe.
        if !enabled {
            self.fail_safe();
        }
        Ok(())
    }

    pub fn request_asic_reset_asserted(&mut self, asserted: bool, now_ms: u64, trip_input_asserted: bool) -> Result<(), SafetyError> {
        self.service(now_ms, trip_input_asserted);
        if !asserted {
            if let Err(error) = self.ensure_controlled(now_ms) {
                self.fail_safe();
                return Err(error);
            }
            if !self.policy.outputs().asic_reset_asserted {
                return Ok(());
            }
        }

        if let Err(error) = self.policy.request_asic_reset_asserted(asserted, now_ms) {
            self.fail_safe();
            return Err(error);
        }
        Ok(())
    }

    pub fn request_fan_percent(&mut self, percent: u8, now_ms: u64, trip_input_asserted: bool) -> Result<(), SafetyError> {
        self.service(now_ms, trip_input_asserted);
        if percent < 100 {
            if let Err(error) = self.ensure_controlled(now_ms) {
                self.fail_safe();
                return Err(error);
            }
        }

        if let Err(error) = self.policy.request_fan_percent(percent, now_ms) {
            self.fail_safe();
            return Err(error);
        }
        Ok(())
    }

    pub fn fail_safe(&mut self) {
        self.lease_owned = false;
        self.next_heartbeat_ms = 0;
        self.policy.disarm();
    }

    fn ensure_controlled(&mut self, now_ms: u64) -> Result<(), SafetyError> {
        let status = self.policy.status(now_ms);
        match status.state {
            bonanza_bridge_fw::safety::SafetyState::Controlled => {}
            bonanza_bridge_fw::safety::SafetyState::SafeOff => self.policy.arm(now_ms)?,
            bonanza_bridge_fw::safety::SafetyState::FaultLatched => {
                self.policy.clear_fault(now_ms)?;
                self.policy.arm(now_ms)?;
            }
        }

        // Do not postpone an already scheduled heartbeat when a client sends
        // frequent unsafe or idempotent commands. The two-chip raw firmware's
        // heartbeat ticker is independent of command traffic, and the combined
        // owner must preserve that property.
        if !self.lease_owned {
            self.lease_owned = true;
            self.next_heartbeat_ms = now_ms.saturating_add(HEARTBEAT_INTERVAL_MS);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bonanza_bridge_fw::safety::{FaultReason, SafetyState, CAP_FAN_CONTROLLED_SPEED};

    #[test]
    fn first_unsafe_request_transparently_owns_the_lease() {
        let mut owner = BridgeOwner::new();

        owner.request_five_volt_enabled(true, 100, false).unwrap();
        let status = owner.status(100);

        assert_eq!(status.state, SafetyState::Controlled);
        assert!(status.lease_remaining_ms > 0);
        assert!(status.outputs.five_volt_enabled);
        assert!(status.outputs.asic_reset_asserted);
    }

    #[test]
    fn latest_bridge_allows_controlled_fan_speed_while_powered() {
        let mut owner = BridgeOwner::new();

        owner.request_five_volt_enabled(true, 0, false).unwrap();
        owner.request_asic_reset_asserted(false, 0, false).unwrap();
        owner.request_fan_percent(40, 0, false).unwrap();

        let status = owner.status(0);
        assert_ne!(status.capabilities & CAP_FAN_CONTROLLED_SPEED, 0);
        assert_eq!(status.outputs.fan_percent, 40);
    }

    #[test]
    fn repeated_unsafe_levels_are_idempotent_after_reset_release() {
        let mut owner = BridgeOwner::new();
        owner.request_five_volt_enabled(true, 0, false).unwrap();
        owner.request_asic_reset_asserted(false, 0, false).unwrap();

        owner.request_five_volt_enabled(true, 1, false).unwrap();
        owner.request_asic_reset_asserted(false, 1, false).unwrap();

        let outputs = owner.outputs();
        assert!(outputs.five_volt_enabled);
        assert!(!outputs.asic_reset_asserted);
    }

    #[test]
    fn periodic_service_keeps_the_internal_lease_alive() {
        let mut owner = BridgeOwner::new();
        owner.request_five_volt_enabled(true, 0, false).unwrap();

        for now_ms in (250..=5_000).step_by(250) {
            owner.service(now_ms, false);
        }

        assert_eq!(owner.status(5_000).state, SafetyState::Controlled);
        assert!(owner.status(5_000).lease_remaining_ms > 0);
    }

    #[test]
    fn frequent_unsafe_commands_do_not_postpone_heartbeats() {
        let mut owner = BridgeOwner::new();
        owner.request_five_volt_enabled(true, 0, false).unwrap();

        for now_ms in (100..=5_000).step_by(100) {
            owner.request_five_volt_enabled(true, now_ms, false).unwrap();
        }

        assert_eq!(owner.status(5_000).state, SafetyState::Controlled);
        assert!(owner.status(5_000).lease_remaining_ms > 0);
    }

    #[test]
    fn missed_lease_and_trip_both_force_safe_outputs() {
        let mut expired = BridgeOwner::new();
        expired.request_five_volt_enabled(true, 0, false).unwrap();
        expired.service(2_001, false);
        assert_eq!(expired.status(2_001).fault, FaultReason::LeaseExpired);
        assert!(expired.outputs().is_safe());

        let mut tripped = BridgeOwner::new();
        tripped.request_five_volt_enabled(true, 0, false).unwrap();
        tripped.service(1, true);
        assert_eq!(tripped.status(1).fault, FaultReason::AsicTrip);
        assert!(tripped.outputs().is_safe());
    }

    #[test]
    fn disabling_five_volts_disarms_and_restores_safe_outputs() {
        let mut owner = BridgeOwner::new();
        owner.request_five_volt_enabled(true, 0, false).unwrap();
        owner.request_asic_reset_asserted(false, 0, false).unwrap();
        owner.request_fan_percent(50, 0, false).unwrap();

        owner.request_five_volt_enabled(false, 10, false).unwrap();

        assert_eq!(owner.status(10).state, SafetyState::SafeOff);
        assert_eq!(owner.outputs(), SafetyOutputs::SAFE);
    }
}
