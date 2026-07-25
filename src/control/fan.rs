use embassy_rp::{gpio, pwm};
use embassy_time::{Duration, Instant, Timer};

use super::{CommandError, Controller, ControllerCommand};

pub struct Pins<'d> {
    pub pwm: pwm::Pwm<'d>,
    pub tach: gpio::Input<'d>,
}

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

pub fn set_speed(pins: &mut Pins<'_>, speed: u8) {
    let duty = (1000u32 * speed.min(100) as u32 / 100) as u16;

    let mut config = pwm::Config::default();
    config.top = 1000;
    config.compare_a = duty;
    config.compare_b = 0;
    config.divider = 5.into();
    config.invert_a = true;
    config.phase_correct = false;
    config.enable = true;
    pins.pwm.set_config(&config);
}

impl ControllerCommand for Command {
    async fn handle(&self, controller: &mut Controller) -> Result<heapless::Vec<u8, 256>, CommandError> {
        match self {
            Command::SetSpeed(speed) => {
                set_speed(&mut controller.fan, *speed);

                let mut res = heapless::Vec::new();
                res.push(0x00).unwrap(); // Success
                Ok(res)
            }
            Command::GetTach => {
                // Measure fan tachometer pulses
                // Most fans output 2 pulses per revolution
                let rpm = measure_fan_rpm(&mut controller.fan.tach).await?;

                let mut res = heapless::Vec::new();
                res.extend_from_slice(&rpm.to_le_bytes()).unwrap();
                Ok(res)
            }
        }
    }
}

/// Measure fan RPM by counting tachometer pulses over a period
/// Most PC fans output 2 pulses per revolution
async fn measure_fan_rpm(tach: &mut gpio::Input<'_>) -> Result<u16, CommandError> {
    let measurement_time = Duration::from_millis(500);
    let start = Instant::now();
    let mut pulse_count = 0u32;
    let mut last_state = tach.is_high();

    // Count rising edges for 500ms
    while start.elapsed() < measurement_time {
        let current_state = tach.is_high();
        if current_state && !last_state {
            pulse_count += 1;
        }
        last_state = current_state;
        Timer::after_micros(100).await; // Sample at 10kHz
    }

    // Calculate RPM: (pulses / 2) * (60 / 0.5) = pulses * 60
    // Most fans have 2 pulses per revolution
    let rpm = (pulse_count * 60) as u16;

    Ok(rpm)
}
