use super::CommandError;
use heapless::Vec;

pub struct Pins<'d> {
    pub vr_en: embassy_rp::gpio::Output<'d>,
    pub vr_pgood: embassy_rp::gpio::Input<'d>,
}

#[derive(defmt::Format)]
pub enum Command {
    SetAsicResetn { level: bool },
    GetAsicResetn,
    Set5vEn { level: bool },
    Get5vEn,
    SetAsicRst { level: bool },
    GetAsicRst,
    GetAsicTrip,
    SetVrEn { level: bool },
    GetVrEn,
    GetVrPgood,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        match buf {
            [0x00] => Ok(Self::GetAsicResetn),
            [0x00, level] => Ok(Self::SetAsicResetn { level: *level > 0 }),
            [0x01] => Ok(Self::Get5vEn),
            [0x01, level] => Ok(Self::Set5vEn { level: *level > 0 }),
            [0x02] => Ok(Self::GetAsicRst),
            [0x02, level] => Ok(Self::SetAsicRst { level: *level > 0 }),
            [0x03] => Ok(Self::GetAsicTrip),
            [0x04] => Ok(Self::GetVrEn),
            [0x04, level] => Ok(Self::SetVrEn { level: *level > 0 }),
            [0x05] => Ok(Self::GetVrPgood),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl super::ControllerCommand for Command {
    async fn handle(&self, controller: &mut super::Controller) -> Result<Vec<u8, 256>, CommandError> {
        let level = match self {
            Command::GetAsicResetn | Command::GetAsicRst => return crate::bridge::gpio(crate::bridge::GPIO_ASIC_RESET, None).await.map_err(CommandError::from_bridge),
            Command::SetAsicResetn { level } | Command::SetAsicRst { level } => return crate::bridge::gpio(crate::bridge::GPIO_ASIC_RESET, Some(*level)).await.map_err(CommandError::from_bridge),
            Command::Get5vEn => return crate::bridge::gpio(crate::bridge::GPIO_5V_ENABLE, None).await.map_err(CommandError::from_bridge),
            Command::Set5vEn { level } => return crate::bridge::gpio(crate::bridge::GPIO_5V_ENABLE, Some(*level)).await.map_err(CommandError::from_bridge),
            Command::GetAsicTrip => return crate::bridge::gpio(crate::bridge::GPIO_ASIC_TRIP, None).await.map_err(CommandError::from_bridge),
            Command::GetVrEn => bool::from(controller.gpio.vr_en.get_output_level()),
            Command::SetVrEn { level } => {
                controller.gpio.vr_en.set_level((*level).into());
                bool::from(controller.gpio.vr_en.get_output_level())
            }
            Command::GetVrPgood => controller.gpio.vr_pgood.is_high(),
        };

        Ok(Vec::from_slice(&[level as u8]).unwrap())
    }
}
