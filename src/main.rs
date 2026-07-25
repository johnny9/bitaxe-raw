#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::{
    adc::{self},
    bind_interrupts,
    flash::{self},
    gpio::{self},
    i2c::{self},
    peripherals::{PIO1, USB},
    pio::{self},
    pwm::{self},
    usb::{self},
    Peripheral,
};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use static_cell::StaticCell;

mod control;
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

const FLASH_SIZE: usize = 4 * 1024 * 1024;
const VERSION: u16 = 0x0001;

static MANUFACTURER: &str = "OSMU";
static PRODUCT: &str = "BitaxeBonanza";

/// Return a unique serial number for this device by hashing its flash JEDEC ID.
fn serial_number() -> &'static str {
    let p = unsafe { embassy_rp::Peripherals::steal() };
    let flash = unsafe { p.FLASH.clone_unchecked() };
    let mut flash = flash::Flash::<_, flash::Async, FLASH_SIZE>::new(flash, p.DMA_CH0);
    static SERIAL_NUMBER_BUF: StaticCell<[u8; 8]> = StaticCell::new();
    let jedec_id = flash.blocking_jedec_id().unwrap();
    let sn = const_murmur3::murmur3_32(&jedec_id.to_le_bytes(), 0);
    let buf = SERIAL_NUMBER_BUF.init([0; 8]);
    hex::encode_to_slice(sn.to_le_bytes(), &mut buf[..]).unwrap();
    unsafe { core::str::from_utf8_unchecked(buf) }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut watchdog = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
    watchdog.set_scratch(0, 0);
    watchdog.feed();

    let usb_driver = usb::Driver::new(p.USB, Irqs);

    let usb_config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
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
        v5_en: gpio::Output::new(p.PIN_18, gpio::Level::Low),
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
        let mut pwm_config = pwm::Config::default();
        pwm_config.top = 1000; // 1000 steps for 0.1% resolution
        pwm_config.compare_a = 0; // Start at 0% duty cycle
        pwm_config.compare_b = 0;
        pwm_config.divider = 5.into(); // 125MHz / 5 / 1000 = 25kHz
        pwm_config.invert_a = false;
        pwm_config.phase_correct = false;
        pwm_config.enable = true; // Explicitly enable PWM

        let pwm = pwm::Pwm::new_output_a(p.PWM_SLICE2, p.PIN_20, pwm_config.clone());

        let tach = gpio::Input::new(p.PIN_21, gpio::Pull::None);
        control::fan::Pins { pwm, tach }
    };

    let pio::Pio { mut common, sm0, sm1, .. } = pio::Pio::new(p.PIO1, Irqs);
    let asic_uart = pio_uart::PioUart::new(&mut common, sm0, sm1, p.PIN_8, p.PIN_9, 115200);

    unwrap!(spawner.spawn(usb_task(builder.build())));
    unwrap!(spawner.spawn(control::usb_task(control_class, i2c, gpio_pins, adc_pins, fan_pins)));
    unwrap!(spawner.spawn(uart::usb_task(asic_uart_class, asic_uart)));

    loop {
        watchdog.feed();
        Timer::after_secs(2).await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice) -> ! {
    usb.run().await
}
