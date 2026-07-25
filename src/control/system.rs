use heapless::Vec;

use crate::diagnostics::{self, SafetySnapshot};

use super::{CommandError, Controller, ControllerCommand};

#[derive(defmt::Format)]
pub enum Command {
    Info,
    RxStats,
    SafetyStatus,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        match buf {
            [diagnostics::SYSTEM_GET_INFO] => Ok(Self::Info),
            [diagnostics::SYSTEM_GET_RX_STATS] => Ok(Self::RxStats),
            [diagnostics::SYSTEM_GET_SAFETY_STATUS] => Ok(Self::SafetyStatus),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl ControllerCommand for Command {
    async fn handle(&self, controller: &mut Controller) -> Result<Vec<u8, 256>, CommandError> {
        match self {
            Self::Info => {
                let mut payload = [0; 67];
                let length = diagnostics::encode_info(&mut payload).ok_or(CommandError::Invalid)?;
                Vec::from_slice(&payload[..length]).map_err(|_| CommandError::BufferOverflow)
            }
            Self::RxStats => {
                // This direct RP2040 path has no intermediate UART or software
                // RX ring, so both ESP-compatible loss counters are zero.
                Vec::from_slice(&diagnostics::encode_rx_stats(0, 0)).map_err(|_| CommandError::BufferOverflow)
            }
            Self::SafetyStatus => {
                let snapshot = SafetySnapshot {
                    five_volt_enabled: bool::from(controller.gpio.v5_en.get_output_level()),
                    asic_reset_asserted: !bool::from(controller.gpio.asic_rst.get_output_level()),
                    fan_percent: controller.fan.percent,
                    trip_input_asserted: controller.gpio.asic_trip.is_high(),
                };
                Vec::from_slice(&diagnostics::encode_safety_status(snapshot)).map_err(|_| CommandError::BufferOverflow)
            }
        }
    }
}
