use embassy_rp::pio::{
    Config, Direction, FifoJoin, Instance, PioPin, ShiftDirection, StateMachine,
};

/// PIO-based 9N1 UART TX and 8-bit RX (drops 9th data bit)
pub struct PioUart<'d, PIO: Instance, const SM_TX: usize, const SM_RX: usize> {
    sm_tx: StateMachine<'d, PIO, SM_TX>,
    sm_rx: StateMachine<'d, PIO, SM_RX>,
}

impl<'d, PIO: Instance, const SM_TX: usize, const SM_RX: usize> PioUart<'d, PIO, SM_TX, SM_RX> {
    pub fn new(
        pio: &mut embassy_rp::pio::Common<'d, PIO>,
        mut sm_tx: StateMachine<'d, PIO, SM_TX>,
        mut sm_rx: StateMachine<'d, PIO, SM_RX>,
        tx_pin: impl PioPin,
        rx_pin: impl PioPin,
        baudrate: u32,
    ) -> Self {
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

        // PIO program for 9-bit UART RX, dropping the 9th bit
        // Receives 1 start bit, 9 data bits, 1 stop bit
        // Each bit period is 8 cycles
        let prg_rx = pio_proc::pio_asm!(
            ".wrap_target"
            "wait 0 pin 0",          // Wait for start bit (falling edge)
            "set x, 7     [11]",     // Set bit counter to 7 (8 bits), delay 1.5 bit periods to center of first data bit
            "bitloop:",
            "in pins, 1   [6]",      // Sample and shift in 1 bit (7 cycles)
            "jmp x-- bitloop",       // Loop (1 cycle) = 8 cycles per bit
            "nop        [7]",        // Wait out the 9th data bit time (drop addr/data flag)
            ".wrap"                  // Remove manual push, use autopush instead
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
        cfg_rx.shift_in.direction = ShiftDirection::Right;  // Shift right (LSB first) like standard UART
        cfg_rx.shift_in.auto_fill = true;   // Enable autopush
        cfg_rx.shift_in.threshold = 8;      // Autopush after 8 bits (9th bit dropped)
        cfg_rx.fifo_join = FifoJoin::RxOnly;
        cfg_rx.clock_divider = Self::calculate_clk_div(baudrate);
        
        sm_rx.set_config(&cfg_rx);
        sm_rx.set_enable(true);

        Self { sm_tx, sm_rx }
    }

    fn calculate_clk_div(baudrate: u32) -> fixed::FixedU32<fixed::types::extra::U8> {
        // RP2040 system clock is typically 125 MHz
        // Each UART bit should take the same amount of time
        // In our PIO program, each bit takes 8 cycles (1 instruction + 7 delay)
        // clk_div = sys_clk / (baudrate * cycles_per_bit)
        let sys_clk = 125_000_000u32;
        let cycles_per_bit = 8u32;
        
        // Calculate using fixed-point arithmetic (8.8 format)
        // clk_div = sys_clk / (baudrate * cycles_per_bit)
        let divisor = baudrate * cycles_per_bit;
        
        // Convert to 8.8 fixed point format
        // Multiply sys_clk by 256 to get fractional precision
        let clk_div_u32 = ((sys_clk as u64 * 256) / divisor as u64) as u32;
        
        fixed::FixedU32::from_bits(clk_div_u32)
    }

    pub fn set_baudrate(&mut self, baudrate: u32) {
        let clk_div = Self::calculate_clk_div(baudrate);
        self.sm_tx.set_clock_divider(clk_div);
        self.sm_rx.set_clock_divider(clk_div);
    }

    /// Write a 9-bit value (blocking)
    pub async fn write_u16(&mut self, data: u16) {
        let data = data & 0x1FF;
        self.sm_tx.tx().wait_push(data as u32).await;
    }

    /// Read an 8-bit value (blocking); 9th bit is dropped by the RX program
    #[allow(dead_code)]
    pub async fn read_u8(&mut self) -> u8 {
        let data = self.sm_rx.rx().wait_pull().await;
        ((data >> 24) & 0xFF) as u8
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

    /// Try to read an 8-bit value (non-blocking); 9th bit is dropped
    pub fn try_read(&mut self) -> Option<u8> {
        if let Some(data) = self.sm_rx.rx().try_pull() {
            // With autopush threshold=8 and shift_right, bits land in [31:24]
            Some(((data >> 24) & 0xFF) as u8)
        } else {
            None
        }
    }

    /// Split into separate TX and RX handles
    #[allow(dead_code)]
    pub fn split(self) -> (PioUartTx<'d, PIO, SM_TX>, PioUartRx<'d, PIO, SM_RX>) {
        (
            PioUartTx { sm: self.sm_tx },
            PioUartRx { sm: self.sm_rx },
        )
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
    pub async fn read_u8(&mut self) -> u8 {
        let data = self.sm.rx().wait_pull().await;
        ((data >> 24) & 0xFF) as u8
    }

    pub fn try_read(&mut self) -> Option<u8> {
        self.sm.rx().try_pull().map(|data| ((data >> 24) & 0xFF) as u8)
    }

    pub fn is_empty(&mut self) -> bool {
        self.sm.rx().empty()
    }
}
