use heapless::Vec;

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
            [crate::bridge::SYSTEM_GET_INFO] => Ok(Self::Info),
            [crate::bridge::SYSTEM_GET_RX_STATS] => Ok(Self::RxStats),
            [crate::bridge::SYSTEM_GET_SAFETY_STATUS] => Ok(Self::SafetyStatus),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl ControllerCommand for Command {
    async fn handle(&self, _controller: &mut Controller) -> Result<Vec<u8, 256>, CommandError> {
        let command = match self {
            Self::Info => crate::bridge::SYSTEM_GET_INFO,
            Self::RxStats => crate::bridge::SYSTEM_GET_RX_STATS,
            Self::SafetyStatus => crate::bridge::SYSTEM_GET_SAFETY_STATUS,
        };
        crate::bridge::system(command).await.map_err(CommandError::from_bridge)
    }
}
