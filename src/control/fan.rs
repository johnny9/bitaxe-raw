use super::{CommandError, Controller, ControllerCommand};

#[derive(defmt::Format)]
pub enum Command {
    SetSpeed(u8), // 0-100 percent
    GetTach,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        match buf {
            [0x10, speed] if *speed <= 100 => Ok(Command::SetSpeed(*speed)),
            [0x20] => Ok(Command::GetTach),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl ControllerCommand for Command {
    async fn handle(&self, _controller: &mut Controller) -> Result<heapless::Vec<u8, 256>, CommandError> {
        match self {
            Command::SetSpeed(speed) => crate::bridge::fan(crate::bridge::FAN_SET_SPEED, Some(*speed)).await.map_err(CommandError::from_bridge),
            Command::GetTach => crate::bridge::fan(crate::bridge::FAN_GET_TACH, None).await.map_err(CommandError::from_bridge),
        }
    }
}
