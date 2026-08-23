use crc8::Crc8;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{Error as I2cError, ErrorKind, ErrorType, I2c, Operation};
use sen5x_rs::Sen5xDriver as PureRustDriver;
use sen5x_rust::Sen5xDriver;
use serial_test::serial;

fn crc8_of(data: &[u8]) -> u8 {
    let mut engine = Crc8::create_msb(49);
    engine.calc(data, data.len() as i32, 0xFF)
}

/// Appends one 2-byte word plus its CRC byte to the wire payload.
fn word(value: u16, payload: &mut Vec<u8>) {
    payload.extend_from_slice(&value.to_be_bytes());
    let crc = crc8_of(&payload[payload.len() - 2..]);
    payload.push(crc);
}

/// Builds the 24-byte wire response of the 0x03C4 read-measured-values command.
///
/// `pm` holds the 4 unsigned PM words; `env` the 4 signed words (humidity,
/// temperature, VOC index, NOx index). Values are passed as the raw fixed-point
/// integers the sensor would emit.
fn measurements_payload(pm: [u16; 4], env: [i16; 4]) -> Vec<u8> {
    let mut response = Vec::new();

    for &val in &pm {
        word(val, &mut response);
    }
    for &val in &env {
        word(val as u16, &mut response);
    }

    response
}

fn generate_mock_payload() -> Vec<u8> {
    // 4 Unsigned PM values (scaled * 10)
    let pm_values = [100u16, 250, 400, 1000];
    // 4 Signed Environmental/Index values (scaled * 100, * 200, * 10, * 10)
    let env_values = [5000i16, 4000i16, 150i16, 20i16];

    measurements_payload(pm_values, env_values)
}

fn generate_corrupted_payload() -> Vec<u8> {
    let mut response = generate_mock_payload();

    // Corrupt the CRC byte of the first PM1.0 word.
    // Format is: [MSB][LSB][CRC], repeated for each measurement.
    response[2] ^= 0xFF;

    response
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn verify_c_reference_implementation() {
    let response = generate_mock_payload();

    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            response.as_ptr(),
            response.len() as u16,
        );
    }

    let mut c_pm1_0: u16 = 0;
    let mut c_pm2_5: u16 = 0;
    let mut c_pm4_0: u16 = 0;
    let mut c_pm10_0: u16 = 0;
    let mut c_humidity: i16 = 0;
    let mut c_temp: i16 = 0;
    let mut c_voc: i16 = 0;
    let mut c_nox: i16 = 0;

    let c_error = unsafe {
        sen5x_rust::ffi::sen5x_read_measured_values(
            &mut c_pm1_0,
            &mut c_pm2_5,
            &mut c_pm4_0,
            &mut c_pm10_0,
            &mut c_humidity,
            &mut c_temp,
            &mut c_voc,
            &mut c_nox,
        )
    };
    assert_eq!(
        c_error, 0,
        "C reference implementation failed to parse payload"
    );

    assert_eq!((c_pm1_0 as f32) / 10.0, 10.0);
    assert_eq!((c_pm2_5 as f32) / 10.0, 25.0);
    assert_eq!((c_humidity as f32) / 100.0, 50.0);
    assert_eq!((c_temp as f32) / 200.0, 20.0);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn verify_rust_driver_implementation() {
    let response = generate_mock_payload();

    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            response.as_ptr(),
            response.len() as u16,
        );
    }

    let mut rust_driver = Sen5xDriver::new();
    let rust_readings = rust_driver.read_measurements().expect("Rust driver failed");

    assert_eq!(rust_readings.pm1_0, Some(10.0));
    assert_eq!(rust_readings.pm2_5, Some(25.0));
    assert_eq!(rust_readings.pm4_0, Some(40.0));
    assert_eq!(rust_readings.pm10_0, Some(100.0));

    assert_eq!(rust_readings.humidity, Some(50.0));
    assert_eq!(rust_readings.temperature, Some(20.0));
    assert_eq!(rust_readings.voc_index, Some(15.0));
    assert_eq!(rust_readings.nox_index, Some(2.0));
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn verify_rust_driver_handles_pm_sentinel() {
    // A raw 0xFFFF on a PM channel means "no data"; the driver must map it
    // to None instead of reporting 6553.5 µg/m³.
    let mut response = generate_mock_payload();
    response[0] = 0xFF;
    response[1] = 0xFF;
    let mut crc8_engine = Crc8::create_msb(49);
    response[2] = crc8_engine.calc(&[0xFF, 0xFF], 2, 0xFF);

    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            response.as_ptr(),
            response.len() as u16,
        );
    }

    let mut rust_driver = Sen5xDriver::new();
    let rust_readings = rust_driver.read_measurements().expect("Rust driver failed");

    assert_eq!(rust_readings.pm1_0, None);
    assert_eq!(rust_readings.pm2_5, Some(25.0));
    assert_eq!(rust_readings.pm4_0, Some(40.0));
    assert_eq!(rust_readings.pm10_0, Some(100.0));
    assert_eq!(rust_readings.humidity, Some(50.0));
    assert_eq!(rust_readings.temperature, Some(20.0));
    assert_eq!(rust_readings.voc_index, Some(15.0));
    assert_eq!(rust_readings.nox_index, Some(2.0));
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn verify_crc_failure_is_detected() {
    let response = generate_corrupted_payload();

    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            response.as_ptr(),
            response.len() as u16,
        );
    }

    let mut rust_driver = Sen5xDriver::new();

    let result = rust_driver.read_measurements();

    assert!(
        result.is_err(),
        "Driver should reject payload with invalid CRC"
    );
}

/// An in-memory I2C bus feeding a fixed response payload to the pure Rust
/// driver, mirroring the mock used in `sen5x-rs`' own test suite.
#[derive(Debug, Default)]
struct MockI2c {
    response: Vec<u8>,
    offset: usize,
}

impl MockI2c {
    fn with_response(response: Vec<u8>) -> Self {
        Self {
            response,
            offset: 0,
        }
    }
}

#[derive(Debug)]
struct MockError;

impl I2cError for MockError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for MockI2c {
    type Error = MockError;
}

impl I2c for MockI2c {
    async fn transaction(
        &mut self,
        _address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), MockError> {
        for operation in operations {
            match operation {
                Operation::Write(_) => {}
                Operation::Read(buffer) => {
                    let end = self.offset + buffer.len();
                    if end > self.response.len() {
                        return Err(MockError);
                    }
                    buffer.copy_from_slice(&self.response[self.offset..end]);
                    self.offset = end;
                }
            }
        }
        Ok(())
    }
}

/// No-op delay provider for the pure Rust driver's async API.
#[derive(Debug, Default)]
struct MockDelay;

impl DelayNs for MockDelay {
    async fn delay_ns(&mut self, _ns: u32) {}
}

/// The core differential guarantee: the exact same raw wire payload is fed to
/// the C reference driver (through the FFI mock HAL and the wrapper's
/// fixed-point to float conversion) and to the pure Rust driver, and their
/// outputs must be identical field for field.
#[cfg(feature = "mock")]
fn assert_c_and_rust_drivers_agree(payload: Vec<u8>) {
    // C reference path
    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            payload.as_ptr(),
            payload.len() as u16,
        );
    }
    let mut c_driver = Sen5xDriver::new();
    let c = c_driver
        .read_measurements()
        .expect("C reference driver failed");

    // Pure Rust path
    let rust = pollster::block_on(async {
        let mut rust_driver =
            PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
        rust_driver
            .read_measurements()
            .await
            .expect("pure Rust driver failed")
    });

    assert_eq!(c.pm1_0, rust.pm1_0, "PM1.0 diverges between drivers");
    assert_eq!(c.pm2_5, rust.pm2_5, "PM2.5 diverges between drivers");
    assert_eq!(c.pm4_0, rust.pm4_0, "PM4.0 diverges between drivers");
    assert_eq!(c.pm10_0, rust.pm10_0, "PM10.0 diverges between drivers");
    assert_eq!(c.humidity, rust.humidity, "humidity diverges between drivers");
    assert_eq!(
        c.temperature, rust.temperature,
        "temperature diverges between drivers"
    );
    assert_eq!(c.voc_index, rust.voc_index, "VOC index diverges between drivers");
    assert_eq!(c.nox_index, rust.nox_index, "NOx index diverges between drivers");
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_normal_values() {
    let payload = measurements_payload(
        [100u16, 250, 400, 1000],
        [5000i16, 4000, 150, 20],
    );
    assert_c_and_rust_drivers_agree(payload);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_negative_temperature() {
    // Raw -100 as temperature = -0.5 °C; both drivers must scale the signed
    // word identically (a positive-only test would miss a sign bug).
    let payload = measurements_payload(
        [100u16, 250, 400, 1000],
        [5000i16, -100, 150, 20],
    );
    assert_c_and_rust_drivers_agree(payload);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_all_channels_unavailable() {
    // Every channel reports "no data": PM → 0xFFFF, environmental → 0x7FFF.
    let payload = measurements_payload([0xFFFFu16; 4], [0x7FFFi16; 4]);
    assert_c_and_rust_drivers_agree(payload);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_mixed_sentinels_and_edge_values() {
    // 0x7FFF on a PM channel is valid data (3276.7 µg/m³), not a sentinel;
    // 0xFFFF as humidity and -1 as VOC are likewise just data. Both drivers
    // must classify these edge words identically.
    let payload = measurements_payload(
        [0x7FFFu16, 0xFFFF, 0, 1000],
        [0xFFFFu16 as i16, 0x7FFF, -1, 20],
    );
    assert_c_and_rust_drivers_agree(payload);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn verify_c_reference_rejects_bad_crc() {
    let response = generate_corrupted_payload();

    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            response.as_ptr(),
            response.len() as u16,
        );
    }

    let mut c_pm1_0 = 0u16;
    let mut c_pm2_5 = 0u16;
    let mut c_pm4_0 = 0u16;
    let mut c_pm10_0 = 0u16;
    let mut c_humidity = 0i16;
    let mut c_temp = 0i16;
    let mut c_voc = 0i16;
    let mut c_nox = 0i16;

    let error = unsafe {
        sen5x_rust::ffi::sen5x_read_measured_values(
            &mut c_pm1_0,
            &mut c_pm2_5,
            &mut c_pm4_0,
            &mut c_pm10_0,
            &mut c_humidity,
            &mut c_temp,
            &mut c_voc,
            &mut c_nox,
        )
    };

    assert_ne!(
        error, 0,
        "C reference driver unexpectedly accepted corrupted CRC"
    );
}
