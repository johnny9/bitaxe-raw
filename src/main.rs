#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::{
    adc::{self},
    bind_interrupts,
    flash::{self},
    gpio::{self},
    i2c::{self},
    interrupt,
    peripherals::{PIO1, USB},
    pio::{self},
    pwm::{self},
    usb::{self},
    Peripheral,
};
use embassy_time::Duration;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use static_cell::StaticCell;

use bonanza_bridge_fw::safety_timing::WATCHDOG_TIMEOUT_MS;

mod bridge;
mod bridge_owner;
mod control;
mod control_protocol;
mod pio_uart;
mod uart;

pub type I2cPeripheral = embassy_rp::peripherals::I2C1;
pub type I2cDriver = i2c::I2c<'static, I2cPeripheral, i2c::Async>;
pub type UsbPeripheral = embassy_rp::peripherals::USB;
pub type UsbDriver = usb::Driver<'static, UsbPeripheral>;
pub type UsbDevice = embassy_usb::UsbDevice<'static, UsbDriver>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    I2C1_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

const FLASH_SIZE: usize = 2 * 1024 * 1024;
const VERSION: u16 = 0x0001;
const USB_VID: u16 = 0xc0de;
const USB_PID: u16 = 0xb17d;

static MANUFACTURER: &str = "OSMU";
static PRODUCT: &str = "BitaxeBonanza";

/// Return a unique serial number for this device from its SPI flash unique ID.
fn serial_number() -> &'static str {
    let p = unsafe { embassy_rp::Peripherals::steal() };
    let flash = unsafe { p.FLASH.clone_unchecked() };
    // DMA channel 0 is reserved for the continuously draining ASIC RX ring.
    let mut flash = flash::Flash::<_, flash::Async, FLASH_SIZE>::new(flash, p.DMA_CH1);
    let mut unique_id = [0; 8];
    flash.blocking_unique_id(&mut unique_id).unwrap();

    static SERIAL_NUMBER_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let buf = SERIAL_NUMBER_BUF.init([0; 16]);
    hex::encode_to_slice(unique_id, &mut buf[..]).unwrap();
    unsafe { core::str::from_utf8_unchecked(buf) }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // ASIC RX uses DMA, but PIO remains highest priority for command TX. USB
    // comes next, followed by slower board-peripheral work.
    interrupt::PIO1_IRQ_0.set_priority(Priority::P0);
    interrupt::USBCTRL_IRQ.set_priority(Priority::P1);
    interrupt::I2C1_IRQ.set_priority(Priority::P2);
    interrupt::ADC_IRQ_FIFO.set_priority(Priority::P2);

    let mut watchdog = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
    watchdog.set_scratch(0, 0);
    watchdog.start(Duration::from_millis(WATCHDOG_TIMEOUT_MS));

    let usb_driver = usb::Driver::new(p.USB, Irqs);

    let usb_config = {
        let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
        config.device_release = VERSION;
        config.manufacturer = Some(MANUFACTURER);
        config.product = Some(PRODUCT);
        config.serial_number = Some(serial_number());
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config.device_class = 0xef;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        config
    };

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        embassy_usb::Builder::new(usb_driver, usb_config, CONFIG_DESCRIPTOR.init([0; 256]), BOS_DESCRIPTOR.init([0; 256]), &mut [], CONTROL_BUF.init([0; 64]))
    };

    let control_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let asic_uart_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let i2c = {
        let sda = p.PIN_14;
        let scl = p.PIN_15;
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, Default::default())
    };

    let gpio_pins = control::gpio::Pins {
        vr_en: gpio::Output::new(p.PIN_19, gpio::Level::Low),
        vr_pgood: gpio::Input::new(p.PIN_16, gpio::Pull::None),
    };

    let bridge_gpio_pins = bridge::Pins {
        v5_en: gpio::Output::new(p.PIN_18, gpio::Level::Low),
        // ASIC_RST is RST_N, so LOW is the semantic asserted/safe level.
        asic_rst: gpio::Output::new(p.PIN_11, gpio::Level::Low),
        asic_trip: gpio::Input::new(p.PIN_10, gpio::Pull::None),
    };

    let adc = adc::Adc::new(p.ADC, Irqs, Default::default());
    let adc_pins = control::adc::Pins {
        adc,
        domain1: adc::Channel::new_pin(p.PIN_26, gpio::Pull::None),
        domain2: adc::Channel::new_pin(p.PIN_27, gpio::Pull::None),
        domain3: adc::Channel::new_pin(p.PIN_28, gpio::Pull::None),
    };

    let fan_pins = {
        let pwm_config = bridge::fan_pwm_config(100);
        let pwm = pwm::Pwm::new_output_a(p.PWM_SLICE2, p.PIN_20, pwm_config.clone());
        let tach = gpio::Input::new(p.PIN_21, gpio::Pull::None);
        bridge::FanPins { pwm, tach }
    };

    let pio::Pio { mut common, sm0, sm1, .. } = pio::Pio::new(p.PIO1, Irqs);
    let asic_uart = pio_uart::PioUart::new(&mut common, sm0, sm1, p.PIN_8, p.PIN_9, 5_000_000);

    unwrap!(spawner.spawn(usb_task(builder.build())));
    unwrap!(spawner.spawn(bridge::manager_task(bridge_gpio_pins, fan_pins, watchdog)));
    unwrap!(spawner.spawn(control::usb_task(control_class, i2c, gpio_pins, adc_pins)));
    unwrap!(spawner.spawn(uart::usb_task(asic_uart_class, asic_uart, p.DMA_CH0)));

    core::future::pending::<()>().await;
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice) -> ! {
    usb.run().await
}
