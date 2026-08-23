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

/// Builds the 12-byte wire response of the 0x03D2 read-raw-values command.
fn raw_payload(humidity: i16, temperature: i16, voc: u16, nox: u16) -> Vec<u8> {
    let mut payload = Vec::new();
    word(humidity as u16, &mut payload);
    word(temperature as u16, &mut payload);
    word(voc, &mut payload);
    word(nox, &mut payload);
    payload
}

/// Builds the 30-byte wire response of the 0x0413 read-PM-values command.
fn pm_payload(values: [u16; 10]) -> Vec<u8> {
    let mut payload = Vec::new();
    for value in values {
        word(value, &mut payload);
    }
    payload
}

/// Builds the 12-byte wire response of the 0xD100 get-version command.
///
/// The reference C driver clocks out four words and uses the first seven
/// data bytes, so the payload is four CRC-protected words whose first seven
/// data bytes carry the version fields.
fn version_payload(bytes: [u8; 8]) -> Vec<u8> {
    let mut payload = Vec::new();
    for chunk in bytes.chunks_exact(2) {
        word(u16::from_be_bytes([chunk[0], chunk[1]]), &mut payload);
    }
    payload
}

/// Builds the 6-byte wire response of the 0xD210 read-and-clear-status
/// command (two words holding the 32-bit status register).
fn device_status_payload(status: u32) -> Vec<u8> {
    let mut payload = Vec::new();
    word((status >> 16) as u16, &mut payload);
    word(status as u16, &mut payload);
    payload
}

/// Builds the 9-byte wire response of the 0x60B2 get-temperature-offset
/// command (temp offset, slope, time constant).
fn temp_offset_payload(temp_offset: i16, slope: i16, time_constant: u16) -> Vec<u8> {
    let mut payload = Vec::new();
    word(temp_offset as u16, &mut payload);
    word(slope as u16, &mut payload);
    word(time_constant, &mut payload);
    payload
}

/// Builds the 3-byte wire response of the 0x60C6 get-warm-start command.
fn warm_start_payload(warm_start: u16) -> Vec<u8> {
    let mut payload = Vec::new();
    word(warm_start, &mut payload);
    payload
}

/// Builds the 18-byte wire response of the 0x60D0/0x60E1 get-tuning-parameters
/// commands (six signed words).
fn tuning_payload(values: [i16; 6]) -> Vec<u8> {
    let mut payload = Vec::new();
    for value in values {
        word(value as u16, &mut payload);
    }
    payload
}

/// Builds the 3-byte wire response of the 0x60F7 get-RHT-acceleration command.
fn rht_acceleration_payload(mode: u16) -> Vec<u8> {
    let mut payload = Vec::new();
    word(mode, &mut payload);
    payload
}

/// Builds the 12-byte wire response of the 0x6181 get-VOC-state command
/// (four words = eight state bytes).
fn voc_state_payload(state: [u8; 8]) -> Vec<u8> {
    let mut payload = Vec::new();
    for chunk in state.chunks_exact(2) {
        word(u16::from_be_bytes([chunk[0], chunk[1]]), &mut payload);
    }
    payload
}

/// Builds the 6-byte wire response of the 0x8004 get-fan-cleaning-interval
/// command (two words holding the 32-bit interval).
fn fan_interval_payload(interval: u32) -> Vec<u8> {
    let mut payload = Vec::new();
    word((interval >> 16) as u16, &mut payload);
    word(interval as u16, &mut payload);
    payload
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
/// driver, mirroring the mock used in `sen5x-rs`' own test suite. It also
/// records every write frame so set-commands can be compared against the
/// C driver's written bytes.
#[derive(Debug, Default)]
struct MockI2c {
    response: Vec<u8>,
    offset: usize,
    commands: Vec<Vec<u8>>,
}

impl MockI2c {
    fn with_response(response: Vec<u8>) -> Self {
        Self {
            response,
            offset: 0,
            commands: Vec::new(),
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
                Operation::Write(bytes) => self.commands.push(bytes.to_vec()),
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

/// The core differential harness: the exact same raw wire payload is fed to
/// the C reference driver (through the FFI mock HAL) and to the pure Rust
/// driver, and both decoded results are returned for comparison.
#[cfg(feature = "mock")]
fn run_c_and_rust<C, R>(
    payload: Vec<u8>,
    c_read: impl FnOnce() -> C,
    rust_read: impl FnOnce(Vec<u8>) -> R,
) -> (C, R) {
    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            payload.as_ptr(),
            payload.len() as u16,
        );
    }
    let c = c_read();
    let rust = rust_read(payload);
    (c, rust)
}

/// Differential guarantee for the 0x03C4 measured-values command: the exact
/// same raw wire payload is fed to the C reference driver (through the FFI
/// mock HAL and the wrapper's fixed-point to float conversion) and to the
/// pure Rust driver, and their outputs must be identical field for field.
#[cfg(feature = "mock")]
fn assert_c_and_rust_drivers_agree(payload: Vec<u8>) {
    let (c, rust) = run_c_and_rust(
        payload,
        || {
            let mut c_driver = Sen5xDriver::new();
            c_driver
                .read_measurements()
                .expect("C reference driver failed")
        },
        |payload| {
            pollster::block_on(async {
                let mut rust_driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                rust_driver
                    .read_measurements()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );

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
fn differential_test_raw_measurements() {
    let (c, rust) = run_c_and_rust(
        raw_payload(5000, 4000, 150, 20),
        || {
            let mut raw_humidity = 0i16;
            let mut raw_temperature = 0i16;
            let mut raw_voc = 0u16;
            let mut raw_nox = 0u16;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_read_measured_raw_values(
                    &mut raw_humidity,
                    &mut raw_temperature,
                    &mut raw_voc,
                    &mut raw_nox,
                )
            };
            assert_eq!(rc, 0, "C reference failed to parse raw values");
            sen5x_rs::Sen5xRawMeasurements {
                humidity: raw_humidity,
                temperature: raw_temperature,
                voc: raw_voc,
                nox: raw_nox,
            }
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .read_measured_raw_values()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );

    assert_eq!(c, rust, "raw values diverge between drivers");
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_pm_values() {
    let values = [100u16, 250, 400, 1000, 50, 120, 300, 500, 900, 3200];
    let (c, rust) = run_c_and_rust(
        pm_payload(values),
        || {
            let mut mass_concentration_pm1p0 = 0u16;
            let mut mass_concentration_pm2p5 = 0u16;
            let mut mass_concentration_pm4p0 = 0u16;
            let mut mass_concentration_pm10p0 = 0u16;
            let mut number_concentration_pm0p5 = 0u16;
            let mut number_concentration_pm1p0 = 0u16;
            let mut number_concentration_pm2p5 = 0u16;
            let mut number_concentration_pm4p0 = 0u16;
            let mut number_concentration_pm10p0 = 0u16;
            let mut typical_particle_size = 0u16;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_read_measured_pm_values(
                    &mut mass_concentration_pm1p0,
                    &mut mass_concentration_pm2p5,
                    &mut mass_concentration_pm4p0,
                    &mut mass_concentration_pm10p0,
                    &mut number_concentration_pm0p5,
                    &mut number_concentration_pm1p0,
                    &mut number_concentration_pm2p5,
                    &mut number_concentration_pm4p0,
                    &mut number_concentration_pm10p0,
                    &mut typical_particle_size,
                )
            };
            assert_eq!(rc, 0, "C reference failed to parse PM values");
            sen5x_rs::Sen5xPmMeasurements {
                mass_pm1_0: mass_concentration_pm1p0,
                mass_pm2_5: mass_concentration_pm2p5,
                mass_pm4_0: mass_concentration_pm4p0,
                mass_pm10_0: mass_concentration_pm10p0,
                number_pm0_5: number_concentration_pm0p5,
                number_pm1_0: number_concentration_pm1p0,
                number_pm2_5: number_concentration_pm2p5,
                number_pm4_0: number_concentration_pm4p0,
                number_pm10_0: number_concentration_pm10p0,
                typical_particle_size,
            }
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .read_measured_pm_values()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );

    assert_eq!(c, rust, "PM values diverge between drivers");
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_version() {
    // firmware 1.2, debug off, hardware 3.0, protocol 4.5
    let (c, rust) = run_c_and_rust(
        version_payload([1u8, 2, 0, 3, 0, 4, 5, 0]),
        || {
            let mut firmware_major = 0u8;
            let mut firmware_minor = 0u8;
            let mut firmware_debug = false;
            let mut hardware_major = 0u8;
            let mut hardware_minor = 0u8;
            let mut protocol_major = 0u8;
            let mut protocol_minor = 0u8;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_get_version(
                    &mut firmware_major,
                    &mut firmware_minor,
                    &mut firmware_debug,
                    &mut hardware_major,
                    &mut hardware_minor,
                    &mut protocol_major,
                    &mut protocol_minor,
                )
            };
            assert_eq!(rc, 0, "C reference failed to parse version");
            sen5x_rs::Sen5xVersion {
                firmware_major,
                firmware_minor,
                firmware_debug,
                hardware_major,
                hardware_minor,
                protocol_major,
                protocol_minor,
            }
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_version()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );

    assert_eq!(c, rust, "version diverges between drivers");
    assert_eq!(rust.firmware_major, 1);
    assert_eq!(rust.firmware_minor, 2);
    assert_eq!(rust.hardware_major, 3);
    assert_eq!(rust.protocol_major, 4);
    assert_eq!(rust.protocol_minor, 5);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_read_and_clear_device_status() {
    let (c, rust) = run_c_and_rust(
        device_status_payload(0x0001_0203),
        || {
            let mut status = 0u32;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_read_and_clear_device_status(&mut status)
            };
            assert_eq!(rc, 0, "C reference failed to read status");
            status
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .read_and_clear_device_status()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );

    assert_eq!(c, rust, "device status diverges between drivers");
    assert_eq!(rust, 0x0001_0203);
}

/// Differential guarantee for write-only (set) commands: the exact frame the
/// C driver puts on the bus (captured by the mock HAL) must be byte-identical
/// to the frame the pure Rust driver writes.
#[cfg(feature = "mock")]
fn assert_c_and_rust_write_frames_agree(
    c_write: impl FnOnce() -> i16,
    rust_write: impl FnOnce(&mut PureRustDriver<MockI2c, MockDelay>),
) -> Vec<u8> {
    let c_rc = c_write();
    assert_eq!(c_rc, 0, "C reference write failed");

    let mut c_frame = [0u8; 64];
    let mut c_len = 0u16;
    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_get_write_buffer(
            c_frame.as_mut_ptr(),
            &mut c_len,
        );
    }

    let rust_frame = pollster::block_on(async {
        let mut driver = PureRustDriver::new(MockI2c::default(), MockDelay);
        rust_write(&mut driver);
        let (i2c, _) = driver.destroy();
        i2c.commands[0].clone()
    });

    assert_eq!(
        &c_frame[..c_len as usize],
        rust_frame.as_slice(),
        "write frame diverges between drivers"
    );

    rust_frame
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_start_fan_cleaning() {
    let frame = assert_c_and_rust_write_frames_agree(
        || unsafe { sen5x_rust::ffi::sen5x_start_fan_cleaning() },
        |driver| {
            pollster::block_on(driver.start_fan_cleaning()).expect("pure Rust driver failed")
        },
    );
    assert_eq!(frame, vec![0x56, 0x07]);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_set_temperature_offset_parameters() {
    let frame = assert_c_and_rust_write_frames_agree(
        || unsafe { sen5x_rust::ffi::sen5x_set_temperature_offset_parameters(-100, 50, 300) },
        |driver| {
            pollster::block_on(driver.set_temperature_offset_parameters(-100, 50, 300))
                .expect("pure Rust driver failed")
        },
    );
    assert_eq!(&frame[..2], &[0x60, 0xB2]);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_get_temperature_offset_parameters() {
    let (c, rust) = run_c_and_rust(
        temp_offset_payload(-100, 50, 300),
        || {
            let mut temp_offset = 0i16;
            let mut slope = 0i16;
            let mut time_constant = 0u16;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_get_temperature_offset_parameters(
                    &mut temp_offset,
                    &mut slope,
                    &mut time_constant,
                )
            };
            assert_eq!(rc, 0, "C reference failed to read temp offset");
            sen5x_rs::TemperatureOffsetParameters {
                temp_offset,
                slope,
                time_constant,
            }
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_temperature_offset_parameters()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );
    assert_eq!(c, rust, "temperature offset diverges between drivers");
    assert_eq!(rust.temp_offset, -100);
    assert_eq!(rust.time_constant, 300);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_set_warm_start_parameter() {
    let frame = assert_c_and_rust_write_frames_agree(
        || unsafe { sen5x_rust::ffi::sen5x_set_warm_start_parameter(3333) },
        |driver| {
            pollster::block_on(driver.set_warm_start_parameter(3333))
                .expect("pure Rust driver failed")
        },
    );
    assert_eq!(&frame[..2], &[0x60, 0xC6]);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_get_warm_start_parameter() {
    let (c, rust) = run_c_and_rust(
        warm_start_payload(3333),
        || {
            let mut warm_start = 0u16;
            let rc = unsafe { sen5x_rust::ffi::sen5x_get_warm_start_parameter(&mut warm_start) };
            assert_eq!(rc, 0, "C reference failed to read warm start");
            warm_start
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_warm_start_parameter()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );
    assert_eq!(c, rust, "warm start diverges between drivers");
    assert_eq!(rust, 3333);
}

fn tuning_values() -> [i16; 6] {
    [-100, 12, 24, 180, 50, 230]
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_set_voc_algorithm_tuning_parameters() {
    let [index_offset, lto, ltg, gating, std_initial, gain] = tuning_values();
    let frame = assert_c_and_rust_write_frames_agree(
        || {
            unsafe {
                sen5x_rust::ffi::sen5x_set_voc_algorithm_tuning_parameters(
                    index_offset, lto, ltg, gating, std_initial, gain,
                )
            }
        },
        |driver| {
            pollster::block_on(driver.set_voc_algorithm_tuning_parameters(
                sen5x_rs::TuningParameters {
                    index_offset,
                    learning_time_offset_hours: lto,
                    learning_time_gain_hours: ltg,
                    gating_max_duration_minutes: gating,
                    std_initial,
                    gain_factor: gain,
                },
            ))
            .expect("pure Rust driver failed")
        },
    );
    assert_eq!(&frame[..2], &[0x60, 0xD0]);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_get_voc_algorithm_tuning_parameters() {
    let values = tuning_values();
    let (c, rust) = run_c_and_rust(
        tuning_payload(values),
        || {
            let mut index_offset = 0i16;
            let mut lto = 0i16;
            let mut ltg = 0i16;
            let mut gating = 0i16;
            let mut std_initial = 0i16;
            let mut gain = 0i16;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_get_voc_algorithm_tuning_parameters(
                    &mut index_offset, &mut lto, &mut ltg, &mut gating, &mut std_initial,
                    &mut gain,
                )
            };
            assert_eq!(rc, 0, "C reference failed to read VOC tuning");
            sen5x_rs::TuningParameters {
                index_offset,
                learning_time_offset_hours: lto,
                learning_time_gain_hours: ltg,
                gating_max_duration_minutes: gating,
                std_initial,
                gain_factor: gain,
            }
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_voc_algorithm_tuning_parameters()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );
    assert_eq!(c, rust, "VOC tuning diverges between drivers");
    assert_eq!(rust.index_offset, -100);
    assert_eq!(rust.gain_factor, 230);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_set_nox_algorithm_tuning_parameters() {
    let [index_offset, lto, ltg, gating, std_initial, gain] = tuning_values();
    let frame = assert_c_and_rust_write_frames_agree(
        || {
            unsafe {
                sen5x_rust::ffi::sen5x_set_nox_algorithm_tuning_parameters(
                    index_offset, lto, ltg, gating, std_initial, gain,
                )
            }
        },
        |driver| {
            pollster::block_on(driver.set_nox_algorithm_tuning_parameters(
                sen5x_rs::TuningParameters {
                    index_offset,
                    learning_time_offset_hours: lto,
                    learning_time_gain_hours: ltg,
                    gating_max_duration_minutes: gating,
                    std_initial,
                    gain_factor: gain,
                },
            ))
            .expect("pure Rust driver failed")
        },
    );
    assert_eq!(&frame[..2], &[0x60, 0xE1]);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_get_nox_algorithm_tuning_parameters() {
    let values = tuning_values();
    let (c, rust) = run_c_and_rust(
        tuning_payload(values),
        || {
            let mut index_offset = 0i16;
            let mut lto = 0i16;
            let mut ltg = 0i16;
            let mut gating = 0i16;
            let mut std_initial = 0i16;
            let mut gain = 0i16;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_get_nox_algorithm_tuning_parameters(
                    &mut index_offset, &mut lto, &mut ltg, &mut gating, &mut std_initial,
                    &mut gain,
                )
            };
            assert_eq!(rc, 0, "C reference failed to read NOx tuning");
            sen5x_rs::TuningParameters {
                index_offset,
                learning_time_offset_hours: lto,
                learning_time_gain_hours: ltg,
                gating_max_duration_minutes: gating,
                std_initial,
                gain_factor: gain,
            }
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_nox_algorithm_tuning_parameters()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );
    assert_eq!(c, rust, "NOx tuning diverges between drivers");
    assert_eq!(rust.index_offset, -100);
    assert_eq!(rust.gain_factor, 230);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_set_rht_acceleration_mode() {
    let frame = assert_c_and_rust_write_frames_agree(
        || unsafe { sen5x_rust::ffi::sen5x_set_rht_acceleration_mode(2) },
        |driver| {
            pollster::block_on(driver.set_rht_acceleration_mode(2))
                .expect("pure Rust driver failed")
        },
    );
    assert_eq!(&frame[..2], &[0x60, 0xF7]);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_get_rht_acceleration_mode() {
    let (c, rust) = run_c_and_rust(
        rht_acceleration_payload(2),
        || {
            let mut mode = 0u16;
            let rc = unsafe { sen5x_rust::ffi::sen5x_get_rht_acceleration_mode(&mut mode) };
            assert_eq!(rc, 0, "C reference failed to read RHT mode");
            mode
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_rht_acceleration_mode()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );
    assert_eq!(c, rust, "RHT acceleration mode diverges between drivers");
    assert_eq!(rust, 2);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_set_voc_algorithm_state() {
    let state = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04];
    let frame = assert_c_and_rust_write_frames_agree(
        || unsafe { sen5x_rust::ffi::sen5x_set_voc_algorithm_state(state.as_ptr(), 8) },
        |driver| {
            pollster::block_on(driver.set_voc_algorithm_state(&state))
                .expect("pure Rust driver failed")
        },
    );
    assert_eq!(&frame[..2], &[0x61, 0x81]);
    assert_eq!(frame.len(), 14); // 2 command + 8 state bytes + 4 CRCs
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_get_voc_algorithm_state() {
    let state = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04];
    let (c, rust) = run_c_and_rust(
        voc_state_payload(state),
        || {
            let mut out = [0u8; 8];
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_get_voc_algorithm_state(out.as_mut_ptr(), 8)
            };
            assert_eq!(rc, 0, "C reference failed to read VOC state");
            out
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_voc_algorithm_state()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );
    assert_eq!(c, rust, "VOC state diverges between drivers");
    assert_eq!(rust, state);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_set_fan_auto_cleaning_interval() {
    let interval = 0x0102_0304u32;
    let frame = assert_c_and_rust_write_frames_agree(
        || unsafe { sen5x_rust::ffi::sen5x_set_fan_auto_cleaning_interval(interval) },
        |driver| {
            pollster::block_on(driver.set_fan_auto_cleaning_interval(interval))
                .expect("pure Rust driver failed")
        },
    );
    assert_eq!(&frame[..2], &[0x80, 0x04]);
}

#[test]
#[cfg(feature = "mock")]
#[serial]
fn differential_test_get_fan_auto_cleaning_interval() {
    let interval = 0x0102_0304u32;
    let (c, rust) = run_c_and_rust(
        fan_interval_payload(interval),
        || {
            let mut out = 0u32;
            let rc = unsafe {
                sen5x_rust::ffi::sen5x_get_fan_auto_cleaning_interval(&mut out)
            };
            assert_eq!(rc, 0, "C reference failed to read fan interval");
            out
        },
        |payload| {
            pollster::block_on(async {
                let mut driver =
                    PureRustDriver::new(MockI2c::with_response(payload), MockDelay);
                driver
                    .get_fan_auto_cleaning_interval()
                    .await
                    .expect("pure Rust driver failed")
            })
        },
    );
    assert_eq!(c, rust, "fan cleaning interval diverges between drivers");
    assert_eq!(rust, interval);
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
