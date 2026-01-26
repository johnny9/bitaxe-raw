
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

impl From<EndpointError> for UartTaskError {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => UartTaskError::Disconnected {},
        }
    }
}

#[embassy_executor::task]
pub async fn usb_task(
    class: CdcAcmClass<'static, super::UsbDriver>,
    mut uart: PioUart<'static, embassy_rp::peripherals::PIO1, 0, 1>,
) -> ! {
    let (mut tx, mut rx, mut ctrl) = class.split_with_control();

    loop {
        rx.wait_connection().await;
        let _ = pipe_uart(&mut tx, &mut rx, &mut ctrl, &mut uart).await;
    }
}

/// Handle ASIC UART <-> BMC USB TTY forwarding and baudrate changes.
///
/// 9-bit serial data is encoded as pairs of bytes over USB for TX:
/// - First byte: lower 8 bits of the 9-bit word
/// - Second byte: bit 8 (0 or 1)
///
/// Received UART data is truncated to 8 bits and sent as single bytes.
pub async fn pipe_uart<'d, T: usb::Instance + 'd>(
    usb_tx: &mut Sender<'d, usb::Driver<'d, T>>,
    usb_rx: &mut Receiver<'d, usb::Driver<'d, T>>,
    ctrl: &mut ControlChanged<'d>,
    uart: &mut PioUart<'static, embassy_rp::peripherals::PIO1, 0, 1>,
) -> Result<(), UartTaskError> {
    let mut usb_buf = [0u8; 64];
    let mut uart_buf = [0u8; 64];
    let mut pending_byte: Option<u8> = None;

    loop {
        let usb_read = usb_rx.read_packet(&mut usb_buf);
        let control_change = ctrl.control_changed();
        let uart_read = uart.read_u8();

        match select3(usb_read, control_change, uart_read).await {
            // Forward data from USB host to UART as 9-bit words
            // Expects pairs of bytes: [data_low, bit8, data_low, bit8, ...]
            Either3::First(result) => {
                let n = result?;
                let data = &usb_buf[..n];

                let mut i = 0;
                // Process any pending byte from last packet
                if let Some(low_byte) = pending_byte {
                    if i < n {
                        let bit8 = data[i] & 0x01;
                        let word = (low_byte as u16) | ((bit8 as u16) << 8);
                        uart.write_u16(word).await;
                        i += 1;
                        pending_byte = None;
                    }
                }

                // Process pairs of bytes
                while i + 1 < n {
                    let low_byte = data[i];
                    let bit8 = data[i + 1] & 0x01;
                    let word = (low_byte as u16) | ((bit8 as u16) << 8);
                    uart.write_u16(word).await;
                    i += 2;
                }

                // Save any remaining byte for next packet
                if i < n {
                    pending_byte = Some(data[i]);
                }
            }
            // Handle baudrate changes from USB CDC control requests
            Either3::Second(()) => {
                let line_coding = usb_rx.line_coding();
                let baudrate = line_coding.data_rate();
                uart.set_baudrate(baudrate);
            }
            // Forward UART RX data to USB as single bytes
            Either3::Third(byte) => {
                let mut count = 0;
                
                // Add the first received byte
                uart_buf[count] = byte;
                count += 1;
                
                // Opportunistically drain any additional buffered data
                while count < uart_buf.len() {
                    if let Some(byte) = uart.try_read() {
                        uart_buf[count] = byte;
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
