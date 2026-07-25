pub enum UartTaskError {
    Disconnected,
}

use embassy_futures::select::{select3, Either3};
use embassy_rp::usb::{self};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender},
    driver::EndpointError,
};

use crate::pio_uart::PioUart;
use crate::uart_codec::{response_byte, NineBitPairDecoder};

impl From<EndpointError> for UartTaskError {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => UartTaskError::Disconnected {},
        }
    }
}

#[embassy_executor::task]
pub async fn usb_task(class: CdcAcmClass<'static, super::UsbDriver>, mut uart: PioUart<'static, embassy_rp::peripherals::PIO1, 0, 1>) -> ! {
    let (mut tx, mut rx, mut ctrl) = class.split_with_control();

    loop {
        rx.wait_connection().await;
        let _ = pipe_uart(&mut tx, &mut rx, &mut ctrl, &mut uart).await;
    }
}

/// Handle ASIC UART <-> BMC USB TTY forwarding and baudrate changes.
///
/// Host-to-ASIC 9-bit serial data is encoded as pairs of bytes over USB:
/// - First byte: lower 8 bits of the 9-bit word
/// - Second byte: bit 8 (0 or 1)
///
/// ASIC responses return only their low eight bits, matching the BIRDS and
/// latest Bonanza Bridge host contract.
pub async fn pipe_uart<'d, T: usb::Instance + 'd>(
    usb_tx: &mut Sender<'d, usb::Driver<'d, T>>,
    usb_rx: &mut Receiver<'d, usb::Driver<'d, T>>,
    ctrl: &mut ControlChanged<'d>,
    uart: &mut PioUart<'static, embassy_rp::peripherals::PIO1, 0, 1>,
) -> Result<(), UartTaskError> {
    let mut usb_buf = [0u8; 64];
    let mut uart_buf = [0u8; 64];
    let mut uart_words = [0u16; 32];
    let mut decoder = NineBitPairDecoder::new();

    let baudrate = usb_rx.line_coding().data_rate();
    if baudrate != 0 {
        uart.set_baudrate(baudrate);
    }

    loop {
        let usb_read = usb_rx.read_packet(&mut usb_buf);
        let control_change = ctrl.control_changed();
        let uart_read = uart.read_u16();

        match select3(usb_read, control_change, uart_read).await {
            // Forward data from USB host to UART as 9-bit words
            // Expects pairs of bytes: [data_low, bit8, data_low, bit8, ...]
            Either3::First(result) => {
                let n = result?;
                let word_count = decoder.decode(&usb_buf[..n], &mut uart_words);
                for word in &uart_words[..word_count] {
                    uart.write_u16(*word).await;
                }
            }
            // Handle baudrate changes from USB CDC control requests
            Either3::Second(()) => {
                let line_coding = usb_rx.line_coding();
                let baudrate = line_coding.data_rate();
                uart.set_baudrate(baudrate);
            }
            // Forward BIRDS-compatible raw ASIC response bytes to USB.
            Either3::Third(word) => {
                let mut count = 0;

                uart_buf[count] = response_byte(word);
                count += 1;

                while count < uart_buf.len() {
                    if let Some(word) = uart.try_read() {
                        uart_buf[count] = response_byte(word);
                        count += 1;
                    } else {
                        break;
                    }
                }

                usb_tx.write_packet(&uart_buf[..count]).await?;
            }
        }
    }
}
