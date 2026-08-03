# bitaxe BIRDS combined raw/Bridge firmware

This firmware makes the BIRDS reference device present the same host-facing product strings and raw protocol as `bitaxe-raw-bonanza` plus a protocol-1.0 `bonanza-bridge-fw`, while retaining its dedicated BIRDS USB PID. Both roles run in one image on the original RP2040 Raspberry Pi Pico; there is no ESP32 or internal ESP-to-Bridge UART. It targets `thumbv6m-none-eabi` and does not target the RP2350.

The combined image pins its embedded Bridge policy and protocol helpers to `bonanza-bridge-fw` revision `478160d` (release `0.0.1-beta.2`). That version includes trip-latched output safety, a two-second internally owned lease, DMA-buffered ASIC RX, real overflow counters, and controlled fan speed while powered.

The target hardware is the [`pico` branch of bitaxeBIRDS](https://github.com/bitaxeorg/bitaxeBIRDS/tree/pico). ASIC 9-bit UART is implemented on RP2040 PIO1.

## Developing

Install Rust:

```Shell
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

rustup target add thumbv6m-none-eabi

cargo install probe-rs-tools --locked
cargo install elf2uf2-rs --locked
cargo install cargo-binutils
```

For SWD-based development and debugging:

```Shell
# Build the latest firmware:
cargo build --release

# Build, program, and attach to the device with RTT for debugging:
cargo run --release

# Just flash the device, don't attach to RTT:
cargo flash --release --chip RP2040

# Erase all flash memory:
probe-rs erase --chip RP2040 --allow-erase-all
```

For UF2-based development:

```Shell
# Build the latest firmware:
cargo build --release

# Convert the ELF to an RP2040-compatible UF2 image:
elf2uf2-rs target/thumbv6m-none-eabi/release/bitaxe-birds-raw bitaxe-birds-raw.uf2

# Convert and deploy the UF2 image to an mounted RP2040:
elf2uf2-rs -d target/thumbv6m-none-eabi/release/bitaxe-birds-raw
```

## Running
The firmware creates the same two serial ports as `bitaxe-raw-bonanza`:

- `control serial`: I2C, GPIO, ADC, fan, and read-only Bridge diagnostics
- `data serial`: BIRDS-compatible ASIC traffic

I2C, ADC, and VR control remain local raw-firmware functions. A dedicated always-running Bridge task owns 5 V, ASIC reset, trip monitoring, fan control, ASIC RX gating, the safety lease, and the hardware watchdog.

The composite USB device uses VID/PID `c0de:b17d`, manufacturer `OSMU`, product `BitaxeBonanza`, and a 16-character serial derived from the RP2040 Pico's SPI flash unique ID. The distinct BIRDS product ID differentiates this RP2040-only hardware while preserving the Bonanza manufacturer and product strings used by clients.

| Function | RP2040 GPIO |
|----------|-------------|
| ASIC TX / RX | GP8 / GP9 |
| ASIC trip / reset | GP10 / GP11 |
| I2C SDA / SCL | GP14 / GP15 |
| VR PGOOD / enable | GP16 / GP19 |
| 5 V enable | GP18 |
| Fan PWM / tach | GP20 / GP21 |
| Domain ADC 1 / 2 / 3 | GP26 / GP27 / GP28 |

### Data Serial
- Second serial port
- **9-bit serial (9N1)**: 9 data bits, no parity, 1 stop bit
- All data is passed through bidirectionally
- The ASIC UART is fixed at 5 Mbaud, matching the current Bonanza Bridge. USB CDC line-coding changes are ignored.
- ASIC RX is drained continuously by DMA into a 1024-word ring while 5 V is enabled and reset is released. Safe transitions synchronously stop and reset the receive path so data from separate powered sessions cannot mix.

**9-bit Data Encoding over USB:**

Host-to-ASIC data is sent as pairs of bytes:
- **First byte**: Lower 8 bits of the 9-bit word (bits 0-7)
- **Second byte**: Bit 8 (only LSB is used, can be 0 or 1)

Examples:
- To send `0x155` (binary: `1_01010101`): Send bytes `[0x55, 0x01]`
- To send `0x0AA` (binary: `0_10101010`): Send bytes `[0xAA, 0x00]`
- ASIC-to-host responses return only bits 0 through 7 as raw bytes. The RP2040 still samples the complete 9N1 word before dropping bit 8. This matches the latest Bonanza Bridge and the BIRDS `bzmd` receive contract.

**Note:** The 9th bit can be used for addressing or protocol-specific purposes depending on your ASIC requirements.


### Control Serial
- First serial port
- baudrate does not matter

**Packet Format**

| 0      | 1      | 2  | 3   | 4    | 5   | 6... |
|--------|--------|----|-----|------|-----|------|
| LEN LO | LEN HI | ID | BUS | PAGE | CMD | DATA |

```
0. length low
1. length high
	- packet length is number of bytes of the whole packet. 
2. command id
	- Whatever byte you want. will be returned in the response 
3. command bus
	- always 0x00 
4. command page
	- I2C:  0x05
	- GPIO: 0x06
	- ADC:  0x07
	- Fan: 0x09
5. command 
	- varies by command page. See below
6. data
	- data to write. variable length. See below
```

**I2C**

Commands:

- write: 0x20
- read: 0x30
- readwrite: 0x40

Data:

- [I2C address, (bytes to write), (number of bytes to read)]

Example:

- write 0xDE to addr 0x4F: `08 00 01 00 05 20 4F DE`
- read one byte from addr 0x4C: `08 00 01 00 05 30 4C 01`
- readwrite two bytes from addr 0x32, reg 0xFE: `09 00 01 00 05 40 32 FE 02`

**GPIO**

Commands:

- `RST_N` compatibility alias: 0x00
- 5v_en: 0x01
- asic_rst: 0x02
- asic_trip (read-only): 0x03
- VR_EN: 0x04
- VR_PGOOD (read-only): 0x05

Data:

- [pin level] (omit for read operations)

Example:

- Set `RST_N` High: `07 00 00 00 06 00 01`
- Set 5v_en High: `07 00 00 00 06 01 01`
- Get asic_rst: `06 00 00 00 06 02`
- Get asic_trip: `06 00 00 00 06 03`
- Set VR_EN High: `07 00 00 00 06 04 01`
- Get VR_PGOOD: `06 00 00 00 06 05`

GPIO and fan commands use the embedded Bonanza Bridge safety policy. The combined firmware transparently arms and renews the local lease, so existing raw clients do not send lease commands. The policy requires full fan and asserted ASIC reset before enabling 5 V, and requires 5 V before releasing reset. Once controlled and powered, the latest Bridge policy permits a lower host-supervised fan target. Disabling 5 V, closing the control port, lease expiry, an ASIC trip, or a stalled Bridge task restores reset asserted, 5 V off, and full fan.

This intentionally follows the Bonanza GPIO numbering: command `0x00` is the `RST_N` compatibility alias and VR enable is command `0x04`. Clients written for the older BIRDS raw mapping, where `0x00` meant VR enable, must use `0x04`.

VR enable remains a raw-side BIRDS reference-device GPIO, matching the ESP-side responsibility in a two-chip BitaxeBonanza. Closing the control serial port also disables VR power.

**System diagnostics**

The read-only ESP-compatible diagnostic commands are available on page `0x00`:

- firmware and protocol info: `0x01`
- receive overflow counters: `0x02`
- safety status: `0x10`

The payload schemas and error encodings match `bitaxe-raw-bonanza`. `GET_INFO` reports Bridge protocol `1.0` and the pinned Bridge firmware identity. RX statistics report the real PIO FIFO and DMA software-ring loss counters. Safety status reports the production `TripLatch` stage, live lease time, fault state, effective outputs, trip input, and capability value `0x008f`, including controlled fan speed. Lease mutation commands remain unavailable to USB clients, matching the ESP raw interface.

Errors use the ESP diagnostic namespace: timeout `0x10`, invalid command `0x11`, denied `0x12`, fault `0x13`, and extended errors beginning with `0xff`. Buffer overflow is encoded as `ff 42 75 66` (`ff "Buf"`).

**ADC**

Commands:

- read domain1: 0x50
- read domain2: 0x51
- read domain3: 0x52

Example:

- read domain1: `06 00 00 00 07 50`
- read domain2: `06 00 00 00 07 51`

**Fan**

Commands:

- set speed: 0x10
- get tachometer: 0x20

Data:

- [speed percentage 0-100] (for set speed command)

Example:

- Set fan speed to 50%:  `07 00 00 00 09 10 32`
- Set fan speed to 100%: `07 00 00 00 09 10 64`
- Read fan tach (RPM):   `06 00 00 00 09 20`
