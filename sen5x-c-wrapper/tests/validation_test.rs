use crc8::Crc8;
use sen5x_rust::Sen5xDriver;

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

#[test]
#[cfg(feature = "mock")]
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

    assert_eq!(rust_readings.pm1_0, 10.0);
    assert_eq!(rust_readings.pm2_5, 25.0);
    assert_eq!(rust_readings.pm4_0, 40.0);
    assert_eq!(rust_readings.pm10_0, 100.0);

    assert_eq!(rust_readings.humidity, Some(50.0));
    assert_eq!(rust_readings.temperature, Some(20.0));
    assert_eq!(rust_readings.voc_index, Some(15.0));
    assert_eq!(rust_readings.nox_index, Some(2.0));
}
