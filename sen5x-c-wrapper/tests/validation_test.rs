use crc8::Crc8;
use sen5x_rust::Sen5xDriver;

#[test]
#[cfg(feature = "mock")]
fn validate_measurement_parsing_against_reference() {
    let mut response = Vec::new();

    append_word(&mut response, 100);
    append_word(&mut response, 250);
    append_word(&mut response, 400);
    append_word(&mut response, 1000);

    append_word(&mut response, 5000);
    append_word(&mut response, 4000);

    append_word(&mut response, 150);
    append_word(&mut response, 20);

    unsafe {
        sen5x_rust::ffi::sensirion_i2c_hal_mock_set_read_buffer(
            response.as_ptr(),
            response.len() as u16,
        );
    }

    let mut driver = Sen5xDriver::new();

    let readings = driver.read_measurements().expect("Rust driver failed");

    assert_eq!(readings.pm1_0, 10.0);
    assert_eq!(readings.pm2_5, 25.0);
    assert_eq!(readings.pm4_0, 40.0);
    assert_eq!(readings.pm10_0, 100.0);

    assert_eq!(readings.humidity, Some(50.0));
    assert_eq!(readings.temperature, Some(20.0));

    assert_eq!(readings.voc_index, Some(15.0));
    assert_eq!(readings.nox_index, Some(2.0));
}

fn append_word(buffer: &mut Vec<u8>, value: u16) {
    let bytes = value.to_be_bytes();

    buffer.push(bytes[0]);
    buffer.push(bytes[1]);

    let mut crc8_engine = Crc8::create_msb(49);
    let crc_byte = crc8_engine.calc(&bytes, 2, 0xFF);

    buffer.push(crc_byte);
}
