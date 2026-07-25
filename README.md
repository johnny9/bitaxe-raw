# bitaxe BIRDS raw USB firmware

This firmware makes a BIRDS development board present the same host-facing USB identity and raw protocol as a BitaxeBonanza, without an ESP32 or an intermediate Bridge control UART. It runs directly on the original RP2040 Raspberry Pi Pico and targets `thumbv6m-none-eabi`; it does not target the RP2350.

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
The usbserial firmware will create two serial ports. The first serial port is "control serial" for I2C, GPIO, and ADC. The second serial port is "data serial" and is pass through UART.

The composite USB device uses VID/PID `c0de:cafe`, manufacturer `bitaxeBIRDS`, product `BitaxeBonanza`, and a 16-character serial derived from the RP2040 Pico's SPI flash unique ID. Keeping the Bonanza product string preserves client compatibility, while the BIRDS manufacturer distinguishes this RP2040-only hardware in USB descriptors and persistent device paths.

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
- USB serial baudrate is mirrored to the 9-bit UART output. Baudrates up to 5Mbaud have been tested, and seem to work 🤞

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

GPIO and fan commands operate directly on the RP2040 pins. There is no safety-lease protocol on this developer firmware; the host owns power and reset sequencing.

This intentionally follows the Bonanza GPIO numbering: command `0x00` is the `RST_N` compatibility alias and VR enable is command `0x04`. Clients written for the older BIRDS raw mapping, where `0x00` meant VR enable, must use `0x04`.

Closing the control serial port immediately asserts ASIC reset, disables 5 V and VR power, and drives the fan to full speed. This requires no lease acquisition, renewal, or keepalive command.

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
