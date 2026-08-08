use crc8::Crc8;
use sen5x_rust::Sen5xDriver;
use serial_test::serial;

fn generate_mock_payload() -> Vec<u8> {
    let mut response = Vec::new();

    // 4 Unsigned PM values (scaled * 10)
    let pm_values = [100u16, 250, 400, 1000];
    // 4 Signed Environmental/Index values (scaled * 100, * 200, * 10, * 10)
    let env_values = [5000i16, 4000i16, 150i16, 20i16];

    let mut crc8_engine = Crc8::create_msb(49);

    for &val in &pm_values {
        let bytes = val.to_be_bytes();
        response.push(bytes[0]);
        response.push(bytes[1]);
        let crc_byte = crc8_engine.calc(&bytes, 2, 0xFF);
        response.push(crc_byte);
    }

    for &val in &env_values {
        let bytes = val.to_be_bytes();
        response.push(bytes[0]);
        response.push(bytes[1]);
        let crc_byte = crc8_engine.calc(&bytes, 2, 0xFF);
        response.push(crc_byte);
    }

    response
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
