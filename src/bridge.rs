use bonanza_bridge_fw::{
    info, rx_stats,
    safety::{fan_pwm_compare, SafetyError, SafetyOutputs},
    safety_timing::{fan_rpm_from_half_second_pulses, safety_service_due, MAX_SAFETY_SERVICE_INTERVAL_MS, TACH_MEASUREMENT_MS, TACH_SAMPLE_INTERVAL_US},
    uart_timing::asic_rx_forwarding_allowed,
};
use embassy_futures::select::{select, Either};
use embassy_rp::{gpio, pwm, watchdog::Watchdog};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::Vec;
use portable_atomic::{AtomicU8, Ordering};

use crate::{bridge_owner::BridgeOwner, pio_uart};

pub const SYSTEM_GET_INFO: u8 = 0x01;
pub const SYSTEM_GET_RX_STATS: u8 = 0x02;
pub const SYSTEM_GET_SAFETY_STATUS: u8 = 0x10;

pub const GPIO_5V_ENABLE: u8 = 0x01;
pub const GPIO_ASIC_RESET: u8 = 0x02;
pub const GPIO_ASIC_TRIP: u8 = 0x03;

pub const FAN_SET_SPEED: u8 = 0x10;
pub const FAN_GET_TACH: u8 = 0x20;

const REQUEST_QUEUE_LEN: usize = 8;

static NEXT_REQUEST_ID: AtomicU8 = AtomicU8::new(0);
static REQUEST_CHANNEL: Channel<CriticalSectionRawMutex, RequestEnvelope, REQUEST_QUEUE_LEN> = Channel::new();
static REPLY_CHANNEL: Channel<CriticalSectionRawMutex, ReplyEnvelope, REQUEST_QUEUE_LEN> = Channel::new();

pub struct Pins<'d> {
    pub v5_en: gpio::Output<'d>,
    pub asic_rst: gpio::Output<'d>,
    pub asic_trip: gpio::Input<'d>,
}

pub struct FanPins<'d> {
    pub pwm: pwm::Pwm<'d>,
    pub tach: gpio::Input<'d>,
}

pub fn fan_pwm_config(percent: u8) -> pwm::Config {
    let mut config = pwm::Config::default();
    config.top = 1000;
    config.compare_a = fan_pwm_compare(percent);
    config.compare_b = 0;
    config.divider = 5.into();
    // The BIRDS reference device uses the same active-low fan-control path as
    // bitaxeBonanza: a safe 100-percent request holds PWM low.
    config.invert_a = true;
    config.phase_correct = false;
    config.enable = true;
    config
}

#[derive(defmt::Format)]
enum Request {
    System { command: u8 },
    Gpio { command: u8, level: Option<bool> },
    Fan { command: u8, speed: Option<u8> },
    Shutdown,
}

#[derive(defmt::Format)]
struct RequestEnvelope {
    id: u8,
    request: Request,
}

struct ReplyEnvelope {
    id: u8,
    result: Result<Vec<u8, 256>, CommandError>,
}

#[derive(Clone, Copy, Debug, defmt::Format, Eq, PartialEq)]
pub enum CommandError {
    Invalid,
    Denied,
    Fault,
}

impl From<SafetyError> for CommandError {
    fn from(error: SafetyError) -> Self {
        match error {
            SafetyError::LeaseExpired | SafetyError::FaultLatched | SafetyError::TripActive => Self::Fault,
            SafetyError::LeaseRequired | SafetyError::InvalidSequence | SafetyError::FanNotSafe | SafetyError::InvalidFanPercent => Self::Denied,
        }
    }
}

async fn request(request: Request) -> Result<Vec<u8, 256>, CommandError> {
    let id = NEXT_REQUEST_ID.fetch_add(1, Ordering::Relaxed);
    REQUEST_CHANNEL.send(RequestEnvelope { id, request }).await;

    // IDs make cancellation safe. A response abandoned by a disconnected USB
    // client is discarded instead of becoming the next command's response.
    loop {
        let reply = REPLY_CHANNEL.receive().await;
        if reply.id == id {
            return reply.result;
        }
    }
}

pub async fn system(command: u8) -> Result<Vec<u8, 256>, CommandError> {
    request(Request::System { command }).await
}

pub async fn gpio(command: u8, level: Option<bool>) -> Result<Vec<u8, 256>, CommandError> {
    request(Request::Gpio { command, level }).await
}

pub async fn fan(command: u8, speed: Option<u8>) -> Result<Vec<u8, 256>, CommandError> {
    request(Request::Fan { command, speed }).await
}

pub async fn shutdown() {
    let _ = request(Request::Shutdown).await;
}

struct BridgeManager {
    gpio: Pins<'static>,
    fan: FanPins<'static>,
    owner: BridgeOwner,
    applied_outputs: Option<SafetyOutputs>,
    watchdog: Watchdog,
}

impl BridgeManager {
    fn new(gpio: Pins<'static>, fan: FanPins<'static>, watchdog: Watchdog) -> Self {
        Self {
            gpio,
            fan,
            owner: BridgeOwner::new(),
            applied_outputs: None,
            watchdog,
        }
    }

    fn now_ms() -> u64 {
        Instant::now().as_millis()
    }

    fn service_safety(&mut self) {
        let now_ms = Self::now_ms();
        self.owner.service(now_ms, self.gpio.asic_trip.is_high());
        self.apply_outputs();
        // The combined bridge task is the only watchdog feed site. If it can
        // no longer sample trip/lease state and apply outputs, the Pico resets
        // to its safe boot levels.
        self.watchdog.feed();
    }

    fn apply_outputs(&mut self) {
        let outputs = self.owner.outputs();
        if self.applied_outputs == Some(outputs) {
            return;
        }

        let forward_asic_rx = asic_rx_forwarding_allowed(outputs.five_volt_enabled, outputs.asic_reset_asserted);
        if !forward_asic_rx {
            pio_uart::set_buffered_rx_forwarding_enabled(false);
        }

        let intent = outputs.board_control_intent();
        self.gpio.v5_en.set_level(intent.five_volt_enable_high.into());
        self.gpio.asic_rst.set_level(intent.asic_reset_n_high.into());
        self.fan.pwm.set_config(&fan_pwm_config(intent.fan_percent));

        if forward_asic_rx {
            pio_uart::set_buffered_rx_forwarding_enabled(true);
        }
        self.applied_outputs = Some(outputs);
    }

    async fn handle(&mut self, request: Request) -> Result<Vec<u8, 256>, CommandError> {
        match request {
            Request::System { command } => self.handle_system(command),
            Request::Gpio { command, level } => self.handle_gpio(command, level),
            Request::Fan { command, speed } => self.handle_fan(command, speed).await,
            Request::Shutdown => {
                self.owner.fail_safe();
                self.apply_outputs();
                Ok(Vec::new())
            }
        }
    }

    fn handle_system(&mut self, command: u8) -> Result<Vec<u8, 256>, CommandError> {
        match command {
            SYSTEM_GET_INFO => info::firmware_info().map_err(|_| CommandError::Invalid),
            SYSTEM_GET_RX_STATS => {
                let (pio_fifo_overflows, software_ring_overflows) = pio_uart::buffered_rx_overflows();
                let payload = rx_stats::encode(pio_fifo_overflows, software_ring_overflows);
                Ok(Vec::from_slice(payload.as_slice()).unwrap())
            }
            SYSTEM_GET_SAFETY_STATUS => {
                let payload = self.owner.status(Self::now_ms()).encode();
                Ok(Vec::from_slice(payload.as_slice()).unwrap())
            }
            _ => Err(CommandError::Invalid),
        }
    }

    fn handle_gpio(&mut self, command: u8, level: Option<bool>) -> Result<Vec<u8, 256>, CommandError> {
        let trip_input_asserted = self.gpio.asic_trip.is_high();
        let now_ms = Self::now_ms();
        let level = match (command, level) {
            (GPIO_5V_ENABLE, None) => bool::from(self.gpio.v5_en.get_output_level()),
            (GPIO_5V_ENABLE, Some(level)) => {
                self.owner.request_five_volt_enabled(level, now_ms, trip_input_asserted).map_err(CommandError::from)?;
                self.apply_outputs();
                bool::from(self.gpio.v5_en.get_output_level())
            }
            (GPIO_ASIC_RESET, None) => bool::from(self.gpio.asic_rst.get_output_level()),
            (GPIO_ASIC_RESET, Some(level)) => {
                // The command carries the physical RST_N level. LOW means the
                // semantic reset request is asserted.
                self.owner.request_asic_reset_asserted(!level, now_ms, trip_input_asserted).map_err(CommandError::from)?;
                self.apply_outputs();
                bool::from(self.gpio.asic_rst.get_output_level())
            }
            (GPIO_ASIC_TRIP, None) => self.gpio.asic_trip.is_high(),
            _ => return Err(CommandError::Invalid),
        };

        Ok(Vec::from_slice(&[level as u8]).unwrap())
    }

    async fn handle_fan(&mut self, command: u8, speed: Option<u8>) -> Result<Vec<u8, 256>, CommandError> {
        match (command, speed) {
            (FAN_SET_SPEED, Some(speed)) if speed <= 100 => {
                self.owner.request_fan_percent(speed, Self::now_ms(), self.gpio.asic_trip.is_high()).map_err(CommandError::from)?;
                self.apply_outputs();
                Ok(Vec::from_slice(&[0]).unwrap())
            }
            (FAN_GET_TACH, None) => {
                let rpm = self.measure_fan_rpm().await?;
                Ok(Vec::from_slice(&rpm.to_le_bytes()).unwrap())
            }
            _ => Err(CommandError::Invalid),
        }
    }

    async fn measure_fan_rpm(&mut self) -> Result<u16, CommandError> {
        let start = Instant::now();
        let mut last_safety_service = Instant::now();
        let mut pulse_count = 0u32;
        let mut last_state = self.fan.tach.is_high();

        while start.elapsed() < Duration::from_millis(TACH_MEASUREMENT_MS) {
            let current_state = self.fan.tach.is_high();
            if current_state && !last_state {
                pulse_count += 1;
            }
            last_state = current_state;

            if safety_service_due(last_safety_service.elapsed().as_micros()) {
                self.service_safety();
                last_safety_service = Instant::now();
            }
            Timer::after_micros(TACH_SAMPLE_INTERVAL_US).await;
        }

        self.service_safety();
        fan_rpm_from_half_second_pulses(pulse_count).ok_or(CommandError::Invalid)
    }
}

#[embassy_executor::task]
pub async fn manager_task(gpio: Pins<'static>, fan: FanPins<'static>, watchdog: Watchdog) -> ! {
    let mut manager = BridgeManager::new(gpio, fan, watchdog);
    let mut ticker = Ticker::every(Duration::from_millis(MAX_SAFETY_SERVICE_INTERVAL_MS));
    manager.service_safety();

    loop {
        match select(REQUEST_CHANNEL.receive(), ticker.next()).await {
            Either::First(envelope) => {
                manager.service_safety();
                let result = manager.handle(envelope.request).await;
                manager.service_safety();
                REPLY_CHANNEL.send(ReplyEnvelope { id: envelope.id, result }).await;
            }
            Either::Second(()) => manager.service_safety(),
        }
    }
}
