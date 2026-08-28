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

- Particulate matter: PM1.0, PM2.5, PM4.0, and PM10
- Relative humidity
- Temperature
- VOC index
- NOx index

## Project Structure

```
sen55-rust/
|
├── sen5x-rs/
│   └── Pure Rust embedded driver and shared conversion logic
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

Example platforms include STM32, ESP32, nRF52, RP2040, and other embedded Rust targets.

## Example Usage

```rust
use sen5x_rs::Sen5xDriver;

// `i2c` and `delay` come from your platform's `embedded-hal-async` support.
let mut sensor = Sen5xDriver::new(i2c, delay);
sensor.start_measurement().await?;

let measurements = sensor.read_measurements().await?;

if let Some(pm2_5) = measurements.pm2_5 {
    // PM2.5 is returned in µg/m³.
}
```

## Measurement Conversion

The driver automatically converts the SEN5x fixed-point values:

| Measurement | Raw scaling | Rust representation |
|-------------|-------------|---------------------|
| PM values | ×10 | µg/m³ |
| Humidity | ×100 | %RH |
| Temperature | ×200 | °C |
| VOC index | ×10 | index value |
| NOx index | ×10 | index value |

## Error Handling

The driver exposes explicit error types:

```rust
pub enum Sen5xError<E> {
    I2c(E),
    Crc,
    InvalidArgument,
}
```

Errors include I2C communication failures, CRC validation failures, and invalid arguments rejected before I2C access.

## Running Tests

```bash
cargo test --workspace --features mock
```

## Current Status

The driver supports the SEN5x command surface, including measurement, status, identification, version, configuration, fan-cleaning, CRC validation, fixed-point conversion, invalid-value handling, and differential testing against the C reference.

## Public API Stability

The driver is currently published as version `0.x`, so breaking API changes may be introduced in minor releases before `1.0`. We will nevertheless aim to keep the API coherent and document migration guidance for intentional breaking changes.

The wire protocol behavior is intended to remain compatible with Sensirion's reference C implementation. Changes to protocol support, conversion semantics, or supported sensor variants will be documented and covered by the differential test suite.

## License

This project is licensed under the same license terms as the original Sensirion reference implementation unless otherwise specified.
