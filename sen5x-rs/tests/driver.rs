use crc8::Crc8;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{Error as I2cError, ErrorKind, ErrorType, I2c, Operation};
use sen5x_rs::{Sen5xDriver, Sen5xError};

#[derive(Debug)]
struct MockError;

impl I2cError for MockError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// An in-memory I2C bus. Reads past the configured response payload fail,
/// mimicking a real bus clocking garbage / NACKing on over-reads, so any
/// driver that reads more bytes than the sensor would produce is caught.
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

/// Records the delays the driver asked for (in microseconds), so the
/// datasheet-mandated command/response timing is verified, not just compiled.
#[derive(Debug, Default)]
struct MockDelay {
    calls_us: Vec<u32>,
}

impl DelayNs for MockDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.calls_us.push(ns / 1000);
    }
}

fn crc8_of(data: &[u8]) -> u8 {
    let mut engine = Crc8::create_msb(49);
    engine.calc(data, data.len() as i32, 0xFF)
}

/// Appends one 2-byte word plus its CRC byte to the response payload.
fn word(value: u16, payload: &mut Vec<u8>) {
    payload.extend_from_slice(&value.to_be_bytes());
    let crc = crc8_of(&payload[payload.len() - 2..]);
    payload.push(crc);
}

/// Builds the 24-byte wire response of the 0x03C4 read-measured-values command.
fn measurements_payload() -> Vec<u8> {
    let mut payload = Vec::new();
    for value in [100u16, 250, 400, 1000] {
        word(value, &mut payload);
    }
    for value in [5000i16, 4000i16, 150i16, 20i16] {
        word(value as u16, &mut payload);
    }
    payload
}

/// Builds the 48-byte wire response of the 0xD014/0xD033 string commands.
fn string_payload(text: &str) -> Vec<u8> {
    let mut padded = [0u8; 32];
    padded[..text.len()].copy_from_slice(text.as_bytes());

    let mut payload = Vec::new();
    for chunk in padded.chunks_exact(2) {
        word(u16::from_be_bytes([chunk[0], chunk[1]]), &mut payload);
    }
    payload
}

#[test]
fn read_measurements_parses_and_scales_all_values() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(
            MockI2c::with_response(measurements_payload()),
            MockDelay::default(),
        );

        let readings = driver
            .read_measurements()
            .await
            .expect("read_measurements failed");

        assert_eq!(readings.pm1_0, Some(10.0));
        assert_eq!(readings.pm2_5, Some(25.0));
        assert_eq!(readings.pm4_0, Some(40.0));
        assert_eq!(readings.pm10_0, Some(100.0));
        assert_eq!(readings.humidity, Some(50.0));
        assert_eq!(readings.temperature, Some(20.0));
        assert_eq!(readings.voc_index, Some(15.0));
        assert_eq!(readings.nox_index, Some(2.0));

        driver.destroy()
    });

    // The driver must clock out exactly the 24 wire bytes the sensor returns;
    // anything more would read garbage beyond the response. The command is
    // sent first and the response is awaited for the mandated 20 ms.
    assert_eq!(i2c.commands, vec![vec![0x03, 0xC4]]);
    assert_eq!(i2c.offset, 24);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn read_measurements_marks_unsupported_fields_as_none() {
    // 0x7FFF signals "value not available" (e.g. NOx on a SEN54).
    let mut payload = Vec::new();
    for value in [100u16, 250, 400, 1000] {
        word(value, &mut payload);
    }
    for value in [5000i16, 4000i16, 0x7FFFi16, 0x7FFFi16] {
        word(value as u16, &mut payload);
    }

    let readings = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        driver
            .read_measurements()
            .await
            .expect("read_measurements failed")
    });

    assert_eq!(readings.voc_index, None);
    assert_eq!(readings.nox_index, None);
}

#[test]
fn read_measurements_marks_pm_sentinel_as_none() {
    // 0xFFFF signals "no data" for PM channels (e.g. measurement not running).
    let mut payload = Vec::new();
    for value in [0xFFFFu16, 250, 400, 1000] {
        word(value, &mut payload);
    }
    for value in [5000i16, 4000i16, 150i16, 20i16] {
        word(value as u16, &mut payload);
    }

    let readings = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        driver
            .read_measurements()
            .await
            .expect("read_measurements failed")
    });

    assert_eq!(readings.pm1_0, None);
    assert_eq!(readings.pm2_5, Some(25.0));
    assert_eq!(readings.pm4_0, Some(40.0));
    assert_eq!(readings.pm10_0, Some(100.0));
    assert_eq!(readings.humidity, Some(50.0));
    assert_eq!(readings.temperature, Some(20.0));
    assert_eq!(readings.voc_index, Some(15.0));
    assert_eq!(readings.nox_index, Some(2.0));
}

#[test]
fn read_measurements_rejects_corrupted_crc() {
    let mut payload = measurements_payload();
    payload[2] ^= 0xFF; // corrupt the CRC of the first PM1.0 word

    let result = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        driver.read_measurements().await
    });

    assert!(matches!(result, Err(Sen5xError::Crc)));
}

#[test]
fn read_device_status_returns_full_u32() {
    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        word(0x0001, &mut payload);
        word(0x0203, &mut payload);

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());

        let status = driver
            .read_device_status()
            .await
            .expect("read_device_status failed");
        assert_eq!(status, 0x0001_0203);

        driver.destroy()
    });

    // Device status is 2 words = 6 wire bytes; reading more would fail.
    assert_eq!(i2c.commands, vec![vec![0xD2, 0x06]]);
    assert_eq!(i2c.offset, 6);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn read_product_name_returns_sensor_string() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(
            MockI2c::with_response(string_payload("SEN55")),
            MockDelay::default(),
        );

        let name = driver
            .read_product_name()
            .await
            .expect("read_product_name failed");
        assert_eq!(name.as_str(), Ok("SEN55"));

        driver.destroy()
    });

    // Product name responses are 48 wire bytes (32 ASCII + 16 CRC).
    assert_eq!(i2c.commands, vec![vec![0xD0, 0x14]]);
    assert_eq!(i2c.offset, 48);
    assert_eq!(delay.calls_us, vec![50_000]);
}

#[test]
fn read_serial_number_returns_sensor_string() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(
            MockI2c::with_response(string_payload("123456789")),
            MockDelay::default(),
        );

        let serial = driver
            .read_serial_number()
            .await
            .expect("read_serial_number failed");
        assert_eq!(serial.as_str(), Ok("123456789"));

        driver.destroy()
    });

    assert_eq!(i2c.commands, vec![vec![0xD0, 0x33]]);
    assert_eq!(i2c.offset, 48);
    assert_eq!(delay.calls_us, vec![50_000]);
}

#[test]
fn read_data_ready_response_polls_the_flag() {
    let (i2c, delay) = pollster::block_on(async {
        let mut ready_payload = Vec::new();
        word(0x0001, &mut ready_payload); // data ready

        let mut driver =
            Sen5xDriver::new(MockI2c::with_response(ready_payload), MockDelay::default());
        assert!(driver
            .read_data_ready_response()
            .await
            .expect("read failed"));

        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x02, 0x02]]);
    assert_eq!(delay.calls_us, vec![20_000]);

    let not_ready = pollster::block_on(async {
        let mut not_ready_payload = Vec::new();
        word(0x0000, &mut not_ready_payload); // no fresh sample

        let mut driver =
            Sen5xDriver::new(MockI2c::with_response(not_ready_payload), MockDelay::default());
        driver
            .read_data_ready_response()
            .await
            .expect("read failed")
    });
    assert!(!not_ready);
}

#[test]
fn request_data_ready_and_write_only_commands_send_expected_bytes() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());

        driver.request_data_ready().await.unwrap();
        driver.device_reset().await.unwrap();
        driver.start_measurement().await.unwrap();
        driver.stop_measurement().await.unwrap();

        driver.destroy()
    });

    assert_eq!(
        i2c.commands,
        vec![
            vec![0x02, 0x02],
            vec![0xD3, 0x04],
            vec![0x00, 0x21],
            vec![0x01, 0x04],
        ]
    );
    // request_data_ready sleeps nothing; reset/start/stop honor the reference
    // driver's 200 ms / 50 ms / 200 ms recovery times.
    assert_eq!(delay.calls_us, vec![200_000, 50_000, 200_000]);
}
