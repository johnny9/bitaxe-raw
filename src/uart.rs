use bonanza_bridge_fw::uart_codec::NineBitPairDecoder;
use embassy_futures::select::{select, Either};
use embassy_rp::{
    peripherals::{DMA_CH0, PIO1},
    usb,
};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender},
    driver::EndpointError,
};

use crate::pio_uart::{receive_buffered_rx_chunk, PioUart, PioUartRx, PioUartTx};

pub enum UartTaskError {
    Disconnected,
}

impl From<EndpointError> for UartTaskError {
    fn from(error: EndpointError) -> Self {
        match error {
            EndpointError::BufferOverflow => panic!("buffer overflow"),
            EndpointError::Disabled => Self::Disconnected,
        }
    }
}

#[embassy_executor::task]
pub async fn usb_task(class: CdcAcmClass<'static, super::UsbDriver>, uart: PioUart<'static, PIO1, 0, 1>, dma: DMA_CH0) -> ! {
    let (mut usb_tx, mut usb_rx, mut ctrl) = class.split_with_control();
    let (mut uart_tx, mut uart_rx) = uart.split();

    // The state machine and channel are driven through PAC registers because
    // Embassy's finite DMA future does not expose RP2040 address-ring mode.
    // Keep both ownership guards for the lifetime of this task.
    let _dma = dma;

    loop {
        usb_rx.wait_connection().await;
        let _ = select(host_to_asic(&mut usb_rx, &mut ctrl, &mut uart_tx), asic_to_host(&mut usb_tx, &mut uart_rx)).await;
    }
}

/// Decode raw-protocol byte pairs from USB into fixed-rate 5 Mbaud 9N1 ASIC
/// words. Unlike the original BIRDS firmware, USB line coding does not change
/// the ASIC rate; this mirrors `bitaxe-raw-bonanza` plus protocol-1.0 Bridge.
async fn host_to_asic<'d, T: usb::Instance + 'd>(usb_rx: &mut Receiver<'d, usb::Driver<'d, T>>, ctrl: &mut ControlChanged<'d>, uart_tx: &mut PioUartTx<'static, PIO1, 0>) -> Result<(), UartTaskError> {
    let mut usb_buf = [0u8; 64];
    let mut words = [0u16; 32];
    let mut decoder = NineBitPairDecoder::new();

    loop {
        match select(usb_rx.read_packet(&mut usb_buf), ctrl.control_changed()).await {
            Either::First(result) => {
                let count = result?;
                let word_count = decoder.decode(&usb_buf[..count], &mut words);
                for word in &words[..word_count] {
                    uart_tx.write_u16(*word).await;
                }
            }
            Either::Second(()) if !usb_rx.dtr() => return Err(UartTaskError::Disconnected),
            Either::Second(()) => {}
        }
    }
}

/// Forward DMA-buffered BIRDS-compatible ASIC response bytes to USB. The PIO
/// receiver still samples the complete 9N1 word and requires its stop bit;
/// protocol 1.0 forwards only the low eight response bits.
async fn asic_to_host<'d, T: usb::Instance + 'd>(usb_tx: &mut Sender<'d, usb::Driver<'d, T>>, _uart_rx: &mut PioUartRx<'static, PIO1, 1>) -> Result<(), UartTaskError> {
    let mut bytes = [0u8; 64];

    loop {
        let count = receive_buffered_rx_chunk(&mut bytes).await;
        usb_tx.write_packet(&bytes[..count]).await?;
    }
}
