use core::{
    cell::UnsafeCell,
    sync::atomic::{compiler_fence, Ordering as CompilerOrdering},
};

use bonanza_bridge_fw::uart_codec::{asic_rx_word_to_u16, asic_rx_word_to_u8};
use bonanza_bridge_fw::uart_timing::{clock_divider_bits, dma_ring_window, ASIC_RX_DMA_RING_WORDS, ASIC_RX_DMA_TRANSFER_COUNT};
use embassy_rp::{
    clocks::clk_sys_freq,
    pac,
    pio::{Config, Direction, FifoJoin, Instance, PioPin, ShiftDirection, StateMachine},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use portable_atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

const ASIC_RX_SM: usize = 1;
const ASIC_RX_DMA_CHANNEL: usize = 0;
const ASIC_RX_DMA_RING_BYTES_LOG2: u8 = 12;
const ASIC_RX_POLL_INTERVAL_US: u64 = 250;

#[repr(C, align(4096))]
struct AlignedDmaRing(UnsafeCell<[u32; ASIC_RX_DMA_RING_WORDS]>);

// DMA owns writes and the receive task uses volatile reads only after observing
// TRANS_COUNT. Natural alignment is required by RP2040 address wrapping.
unsafe impl Sync for AlignedDmaRing {}

static ASIC_RX_DMA_RING: AlignedDmaRing = AlignedDmaRing(UnsafeCell::new([0; ASIC_RX_DMA_RING_WORDS]));
static ASIC_RX_GATE_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static ASIC_RX_FIFO_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static ASIC_RX_RING_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static ASIC_RX_ENABLED: AtomicBool = AtomicBool::new(false);
static ASIC_RX_PROGRAM_ORIGIN: AtomicU8 = AtomicU8::new(0);
static ASIC_RX_DMA_CONSUMED: AtomicU32 = AtomicU32::new(0);
static ASIC_RX_DMA_GENERATION: AtomicU32 = AtomicU32::new(0);

fn abort_buffered_rx_dma() {
    pac::DMA.chan_abort().modify(|mask| mask.set_chan_abort(1 << ASIC_RX_DMA_CHANNEL));
    while pac::DMA.ch(ASIC_RX_DMA_CHANNEL).ctrl_trig().read().busy() {}
}

fn start_buffered_rx_dma() {
    let channel = pac::DMA.ch(ASIC_RX_DMA_CHANNEL);
    channel.read_addr().write_value(pac::PIO1.rxf(ASIC_RX_SM).as_ptr() as u32);
    channel.write_addr().write_value(ASIC_RX_DMA_RING.0.get().cast::<u32>() as u32);
    channel.trans_count().write(|count| *count = ASIC_RX_DMA_TRANSFER_COUNT);

    compiler_fence(CompilerOrdering::SeqCst);
    channel.ctrl_trig().write(|control| {
        control.set_high_priority(true);
        control.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        control.set_incr_read(false);
        control.set_incr_write(true);
        control.set_ring_size(ASIC_RX_DMA_RING_BYTES_LOG2);
        control.set_ring_sel(true);
        control.set_chain_to(ASIC_RX_DMA_CHANNEL as u8);
        control.set_treq_sel(pac::dma::vals::TreqSel::PIO1_RX1);
        control.set_en(true);
    });
    compiler_fence(CompilerOrdering::SeqCst);
}

/// Gate ASIC receive forwarding with the effective bridge safety outputs.
///
/// Disabling synchronously stops DMA, drains the PIO FIFO, and restores the
/// receiver to its start-bit wait. Enabling arms DMA before the state machine,
/// so bytes from separate powered sessions cannot be mixed.
pub fn set_buffered_rx_forwarding_enabled(enabled: bool) {
    let pio = &pac::PIO1;
    let mask = 1u8 << ASIC_RX_SM;

    ASIC_RX_ENABLED.store(false, Ordering::Release);
    abort_buffered_rx_dma();
    pio.ctrl().modify(|w| w.set_sm_enable(w.sm_enable() & !mask));
    while pio.fstat().read().rxempty() & mask == 0 {
        let _ = pio.rxf(ASIC_RX_SM).read();
    }
    let fdebug = pio.fdebug();
    if fdebug.read().rxstall() & mask != 0 {
        fdebug.write(|w| w.set_rxstall(mask));
    }
    ASIC_RX_FIFO_OVERFLOWS.store(0, Ordering::Relaxed);
    ASIC_RX_RING_OVERFLOWS.store(0, Ordering::Relaxed);
    ASIC_RX_DMA_CONSUMED.store(0, Ordering::Relaxed);
    ASIC_RX_DMA_GENERATION.fetch_add(1, Ordering::AcqRel);

    if enabled {
        let origin = ASIC_RX_PROGRAM_ORIGIN.load(Ordering::Relaxed);
        // An unconditional JMP encodes the five-bit program address. Executing
        // it while disabled restores framing after a mid-byte safe transition.
        pio.sm(ASIC_RX_SM).instr().write(|w| w.set_instr(origin as u16));
        pio.ctrl().modify(|w| {
            w.set_sm_restart(w.sm_restart() | mask);
            w.set_clkdiv_restart(w.clkdiv_restart() | mask);
        });
        start_buffered_rx_dma();
        ASIC_RX_ENABLED.store(true, Ordering::Release);
        pio.ctrl().modify(|w| w.set_sm_enable(w.sm_enable() | mask));
    }
    ASIC_RX_GATE_CHANGED.signal(());
}

pub async fn receive_buffered_rx_chunk(output: &mut [u8]) -> usize {
    assert!(!output.is_empty());
    loop {
        while !ASIC_RX_ENABLED.load(Ordering::Acquire) {
            ASIC_RX_GATE_CHANGED.wait().await;
        }

        let channel = pac::DMA.ch(ASIC_RX_DMA_CHANNEL);
        if !channel.ctrl_trig().read().busy() {
            ASIC_RX_DMA_CONSUMED.store(0, Ordering::Relaxed);
            ASIC_RX_DMA_GENERATION.fetch_add(1, Ordering::AcqRel);
            start_buffered_rx_dma();
            Timer::after_micros(ASIC_RX_POLL_INTERVAL_US).await;
            continue;
        }

        let generation = ASIC_RX_DMA_GENERATION.load(Ordering::Acquire);
        let remaining = channel.trans_count().read();
        let produced = ASIC_RX_DMA_TRANSFER_COUNT.wrapping_sub(remaining);
        let consumed = ASIC_RX_DMA_CONSUMED.load(Ordering::Relaxed);
        let (consumed, available, dropped) = dma_ring_window(produced, consumed, ASIC_RX_DMA_RING_WORDS);
        ASIC_RX_RING_OVERFLOWS.fetch_add(dropped, Ordering::Relaxed);

        if available != 0 {
            let count = available.min(output.len());
            compiler_fence(CompilerOrdering::Acquire);
            for (offset, byte) in output[..count].iter_mut().enumerate() {
                let index = consumed.wrapping_add(offset as u32) as usize % ASIC_RX_DMA_RING_WORDS;
                let raw = unsafe { core::ptr::read_volatile(ASIC_RX_DMA_RING.0.get().cast::<u32>().add(index)) };
                *byte = asic_rx_word_to_u8(raw);
            }

            if generation == ASIC_RX_DMA_GENERATION.load(Ordering::Acquire) && ASIC_RX_ENABLED.load(Ordering::Acquire) {
                ASIC_RX_DMA_CONSUMED.store(consumed.wrapping_add(count as u32), Ordering::Release);
                return count;
            }
            continue;
        }

        Timer::after_micros(ASIC_RX_POLL_INTERVAL_US).await;
    }
}

pub fn buffered_rx_overflows() -> (u32, u32) {
    let pio = &pac::PIO1;
    let mask = 1u8 << ASIC_RX_SM;
    let fdebug = pio.fdebug();
    if fdebug.read().rxstall() & mask != 0 {
        fdebug.write(|w| w.set_rxstall(mask));
        ASIC_RX_FIFO_OVERFLOWS.fetch_add(1, Ordering::Relaxed);
    }
    (ASIC_RX_FIFO_OVERFLOWS.load(Ordering::Relaxed), ASIC_RX_RING_OVERFLOWS.load(Ordering::Relaxed))
}

/// PIO-based 9N1 UART (9 data bits, no parity, 1 stop bit)
pub struct PioUart<'d, PIO: Instance, const SM_TX: usize, const SM_RX: usize> {
    sm_tx: StateMachine<'d, PIO, SM_TX>,
    sm_rx: StateMachine<'d, PIO, SM_RX>,
}

#[allow(dead_code)]
impl<'d, PIO: Instance, const SM_TX: usize, const SM_RX: usize> PioUart<'d, PIO, SM_TX, SM_RX> {
    pub fn new(pio: &mut embassy_rp::pio::Common<'d, PIO>, mut sm_tx: StateMachine<'d, PIO, SM_TX>, mut sm_rx: StateMachine<'d, PIO, SM_RX>, tx_pin: impl PioPin, rx_pin: impl PioPin, baudrate: u32) -> Self {
        // PIO program for 9-bit UART TX
        // Sends 1 start bit, 9 data bits, 1 stop bit = 11 bits total
        // Each bit period is 8 cycles for correct baudrate timing
        let prg_tx = pio_proc::pio_asm!(
            ".side_set 1 opt"
            ".wrap_target"
            "pull       side 1 [7]",  // Pull data, idle high (8 cycles total, amortized)
            "set x, 8   side 0 [7]",  // Set counter, start bit (8 cycles)
            "bitloop:",
            "out pins, 1       [6]",  // Output data bit (7 cycles)
            "jmp x-- bitloop",        // Loop (1 cycle) = 7+1 = 8 cycles per bit
            "nop        side 1 [7]",  // Stop bit (8 cycles)
            ".wrap"
        );

        // Sample all nine data bits and require the actual stop/idle level
        // before looking for another start bit. This is the protocol-1.0
        // Bonanza receiver timing used at the fixed 5 Mbaud ASIC rate.
        let prg_rx = pio_proc::pio_asm!(
            ".wrap_target"
            "wait 0 pin 0",          // Wait for start bit (falling edge)
            "set x, 8     [10]",     // Center the first sample 1.5 bit periods after start detection
            "bitloop:",
            "in pins, 1   [6]",      // Sample and shift in 1 bit (7 cycles)
            "jmp x-- bitloop",       // Loop (1 cycle) = 8 cycles per bit
            "wait 1 pin 0",          // Require the stop/idle level
            ".wrap"                  // Autopush provides one complete word
        );

        // Install TX program
        let tx_pin = pio.make_pio_pin(tx_pin);
        sm_tx.set_pin_dirs(Direction::Out, &[&tx_pin]);
        let mut cfg_tx = Config::default();
        let prg_tx_loaded = pio.load_program(&prg_tx.program);
        cfg_tx.use_program(&prg_tx_loaded, &[&tx_pin]);
        cfg_tx.set_out_pins(&[&tx_pin]);
        cfg_tx.set_set_pins(&[&tx_pin]);
        cfg_tx.shift_out.direction = ShiftDirection::Right;
        cfg_tx.shift_out.auto_fill = false;
        cfg_tx.shift_out.threshold = 32;
        cfg_tx.fifo_join = FifoJoin::TxOnly;

        // Calculate clock divider for baudrate
        cfg_tx.clock_divider = Self::calculate_clk_div(baudrate);

        sm_tx.set_config(&cfg_tx);
        sm_tx.set_enable(true);

        // Install RX program
        let mut rx_pin = pio.make_pio_pin(rx_pin);
        // Enable pull-up on RX pin so it idles high when nothing is connected
        rx_pin.set_pull(embassy_rp::gpio::Pull::Up);
        sm_rx.set_pin_dirs(Direction::In, &[&rx_pin]);
        let mut cfg_rx = Config::default();
        let prg_rx_loaded = pio.load_program(&prg_rx.program);
        cfg_rx.use_program(&prg_rx_loaded, &[]);
        cfg_rx.set_in_pins(&[&rx_pin]);
        cfg_rx.shift_in.direction = ShiftDirection::Right; // Shift right (LSB first) like standard UART
        cfg_rx.shift_in.auto_fill = true; // Enable autopush
        cfg_rx.shift_in.threshold = 9; // Autopush after 9 bits
        cfg_rx.fifo_join = FifoJoin::RxOnly;
        cfg_rx.clock_divider = Self::calculate_clk_div(baudrate);
        ASIC_RX_PROGRAM_ORIGIN.store(prg_rx_loaded.origin, Ordering::Relaxed);

        sm_rx.set_config(&cfg_rx);
        // Safety ownership enables the receiver only after 5 V is present and
        // reset is released.
        sm_rx.set_enable(false);

        Self { sm_tx, sm_rx }
    }

    fn calculate_clk_div(baudrate: u32) -> fixed::FixedU32<fixed::types::extra::U8> {
        let bits = clock_divider_bits(clk_sys_freq(), baudrate).expect("PIO UART baudrate must produce a valid divider");
        fixed::FixedU32::from_bits(bits)
    }

    pub fn set_baudrate(&mut self, baudrate: u32) {
        let clk_div = Self::calculate_clk_div(baudrate);
        self.sm_tx.set_clock_divider(clk_div);
        self.sm_rx.set_clock_divider(clk_div);
    }

    /// Write a 9-bit value (blocking)
    pub async fn write_u16(&mut self, data: u16) {
        // Mask to 9 bits
        let data = data & 0x1FF;
        self.sm_tx.tx().wait_push(data as u32).await;
    }

    /// Read a 9-bit value (blocking)
    #[allow(dead_code)]
    pub async fn read_u16(&mut self) -> u16 {
        let data = self.sm_rx.rx().wait_pull().await;
        asic_rx_word_to_u16(data)
    }

    /// Check if TX FIFO is full
    #[allow(dead_code)]
    pub fn tx_is_full(&mut self) -> bool {
        self.sm_tx.tx().full()
    }

    /// Check if RX FIFO is empty
    pub fn rx_is_empty(&mut self) -> bool {
        self.sm_rx.rx().empty()
    }

    /// Try to write a 9-bit value (non-blocking)
    #[allow(dead_code)]
    pub fn try_write(&mut self, data: u16) -> bool {
        if !self.tx_is_full() {
            let data = data & 0x1FF;
            self.sm_tx.tx().push(data as u32);
            true
        } else {
            false
        }
    }

    /// Try to read a 9-bit value (non-blocking)
    pub fn try_read(&mut self) -> Option<u16> {
        self.sm_rx.rx().try_pull().map(asic_rx_word_to_u16)
    }

    /// Split into separate TX and RX handles
    #[allow(dead_code)]
    pub fn split(self) -> (PioUartTx<'d, PIO, SM_TX>, PioUartRx<'d, PIO, SM_RX>) {
        (PioUartTx { sm: self.sm_tx }, PioUartRx { sm: self.sm_rx })
    }
}

/// TX-only handle for split operation
#[allow(dead_code)]
pub struct PioUartTx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
}

#[allow(dead_code)]
impl<'d, PIO: Instance, const SM: usize> PioUartTx<'d, PIO, SM> {
    pub async fn write_u16(&mut self, data: u16) {
        let data = data & 0x1FF;
        self.sm.tx().wait_push(data as u32).await;
    }

    pub fn try_write(&mut self, data: u16) -> bool {
        if !self.is_full() {
            let data = data & 0x1FF;
            self.sm.tx().push(data as u32);
            true
        } else {
            false
        }
    }

    pub fn is_full(&mut self) -> bool {
        self.sm.tx().full()
    }
}

/// RX-only handle for split operation
#[allow(dead_code)]
pub struct PioUartRx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
}

#[allow(dead_code)]
impl<'d, PIO: Instance, const SM: usize> PioUartRx<'d, PIO, SM> {
    pub async fn read_u16(&mut self) -> u16 {
        let data = self.sm.rx().wait_pull().await;
        asic_rx_word_to_u16(data)
    }

    pub fn try_read(&mut self) -> Option<u16> {
        self.sm.rx().try_pull().map(asic_rx_word_to_u16)
    }

    pub fn is_empty(&mut self) -> bool {
        self.sm.rx().empty()
    }
}
