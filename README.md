# SEN5x Rust Driver

An idiomatic, zero-allocation Rust driver for the Sensirion SEN5x environmental sensor family.

This project is a Rust port of Sensirion's reference C driver, with a focus on:

- `no_std` embedded compatibility
- `embedded-hal` based I2C abstraction
- Safe Rust APIs
- Correct protocol implementation
- Differential testing against the original C implementation

The goal is to provide a reliable Rust-native driver while maintaining behavioral compatibility with Sensirion's official reference implementation.

## Supported Sensors

The driver targets the Sensirion SEN5x family:

- SEN50
- SEN54
- SEN55

The SEN5x family provides measurements for:

- Particulate matter:
  - PM1.0
  - PM2.5
  - PM4.0
  - PM10
- Relative humidity
- Temperature
- VOC index
- NOx index

## Project Structure

```
sen55-rust/
|
├── sen5x-conversion/
│   └── Shared sentinel + scale-factor logic (single source of truth)
│
├── sen5x-rs/
│   └── Pure Rust embedded driver
│
├── sen5x-c-wrapper/
│   ├── C reference driver wrapper
│   ├── Mock I2C HAL
│   └── Validation tests
│
└── README.md
```

## Design Goals

### Embedded Friendly

The Rust driver is designed for embedded environments:

- No heap allocation
- No operating system dependency
- Compatible with `no_std`
- Uses `embedded-hal` I2C traits

Example platforms:

- STM32
- ESP32
- nRF52
- RP2040
- Other embedded Rust targets

## Example Usage

```rust
use sen5x_rs::Sen5xDriver;

// `i2c` and `delay` come from your platform's `embedded-hal-async` support.
// On Embassy, `embassy_time::Delay` implements the required `DelayNs` trait.
let mut sensor = Sen5xDriver::new(i2c, delay);

// All driver methods are async; run this from an async context that
// returns a `Result` (e.g. `async fn main() -> Result<(), ...>`).
sensor.start_measurement().await?;

let measurements = sensor.read_measurements().await?;

// Each field is `Option<f32>`: PM values are in µg/m³, and `None` means the
// channel has no data yet (e.g. PM before the first sample, NOx on a SEN54).
if let Some(pm2_5) = measurements.pm2_5 {
    // use the PM2.5 concentration in µg/m³
}
```

## Measurement Conversion

The SEN5x protocol returns fixed-point values.

The driver automatically converts raw values:

| Measurement | Raw scaling | Rust representation |
|-------------|-------------|---------------------|
| PM values | ×10 | µg/m³ |
| Humidity | ×100 | %RH |
| Temperature | ×200 | °C |
| VOC index | ×10 | index value |
| NOx index | ×10 | index value |

Example:

```
Raw PM2.5 value: 250

Converted value:

250 / 10 = 25.0 µg/m³
```

## Error Handling

The driver exposes explicit error types:

```rust
pub enum Sen5xError<E> {
    I2c(E),
    Crc,
    InvalidArgument,
}
```

Errors include:

- I2C communication failures
- CRC validation failures
- Invalid arguments rejected before I2C access

## Validation Strategy

The project uses a differential testing approach.

The Rust implementation is continuously compared against Sensirion's original C reference implementation.

The validation tests verify:

- Correct measurement parsing
- CRC handling
- Invalid value handling
- Signed temperature conversion
- Scaling calculations

Example workflow:

```
Mock SEN5x response buffer
            |
            |
      +-------------+
      |             |
      v             v
 C reference     Rust driver
 implementation  implementation
      |             |
      +-------------+
            |
            v

     Compare outputs
```

This ensures the Rust implementation does not diverge from the proven C driver behavior.

## Mock Hardware Layer

The project contains a desktop mock I2C implementation.

It allows testing the driver without physical hardware by injecting raw SEN5x responses.

Example:

```rust
sensirion_i2c_hal_mock_set_read_buffer(
    response.as_ptr(),
    response.len()
);
```

The mock layer simulates:

- Sensor reads
- CRC failures
- Invalid values
- Edge cases

## Running Tests

Run normal tests:

```bash
cargo test
```

Run tests using the mock HAL (all workspace crates):

```bash
cargo test --workspace --features mock
```

For tests that share the global C mock state:

```bash
cargo test --workspace --features mock -- --test-threads=1
```

## Current Status

Implemented:

- Device reset command
- Start measurement command (full and without-PM modes)
- Stop measurement command
- Data-ready polling command (`read_data_ready`)
- Measurement read command (measured values, raw values, extended PM block)
- Device status read and read-and-clear
- Product name and serial number
- Version read
- Configuration commands: temperature offset, warm start, VOC/NOx algorithm
  tuning, RH/T acceleration mode, VOC algorithm state, fan cleaning
  (manual start and auto-cleaning interval)
- CRC validation
- Fixed-point conversion
- Invalid value handling
- Differential testing against C reference

## Public API Stability

The driver is currently published as version `0.x`, so breaking API changes may be introduced in minor releases before `1.0`. We will nevertheless aim to keep the API coherent and document migration guidance for intentional breaking changes.

The following are part of the public API and may evolve before `1.0`:

- `Sen5xDriver` constructors and methods
- `Sen5xError` and its variants
- `SensorString` and its accessors
- Measurement, raw-measurement, extended-PM, status, and configuration types
- Public constants such as `DEFAULT_I2C_ADDRESS`

The wire protocol behavior is intended to remain compatible with Sensirion's reference C implementation. Changes to protocol support, conversion semantics, or supported sensor variants will be documented in the changelog and covered by the differential test suite.

Once the API and protocol coverage are considered stable, the project can target a `1.0` release with stronger compatibility guarantees. Until then, pin a compatible crate version if your application requires a stable API.

## Future Work

Possible improvements:

- Add more exhaustive protocol tests
- Add hardware integration tests
- Add examples for popular embedded platforms
- Improve error diagnostics

## License

This project is licensed under the same license terms as the original Sensirion reference implementation unless otherwise specified.

Sensirion's original C driver remains the reference implementation for protocol behavior.
