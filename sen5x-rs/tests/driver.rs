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
    let mut crc: u8 = 0xFF;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
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
fn read_measurements_handles_negative_temperature() {
    // Raw -100 → -100 / 200 = -0.5 °C. Positive-only tests can't catch a
    // sign bug: if the driver used unsigned decode, -100 would wrap to 65436
    // and yield 327.18 °C instead of -0.5.
    let mut payload = Vec::new();
    for value in [100u16, 250, 400, 1000] {
        word(value, &mut payload);
    }
    // humidity=5000 (50 %), temperature=-100 (-0.5 °C), voc=150, nox=20
    for value in [5000i16, -100i16, 150i16, 20i16] {
        word(value as u16, &mut payload);
    }

    let readings = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        driver
            .read_measurements()
            .await
            .expect("read_measurements failed")
    });

    assert_eq!(readings.temperature, Some(-0.5));
    assert_eq!(readings.humidity, Some(50.0));
}

#[test]
fn read_measurements_all_sentinels_yield_none() {
    // Every channel at its "no data" sentinel: PM → 0xFFFF, environment →
    // 0x7FFF. Both drivers must map all eight to None, covering the sentinel
    // classification for every channel in one shot.
    let mut payload = Vec::new();
    for value in [0xFFFFu16; 4] {
        word(value, &mut payload);
    }
    for value in [0x7FFFi16; 4] {
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
    assert_eq!(readings.pm2_5, None);
    assert_eq!(readings.pm4_0, None);
    assert_eq!(readings.pm10_0, None);
    assert_eq!(readings.humidity, None);
    assert_eq!(readings.temperature, None);
    assert_eq!(readings.voc_index, None);
    assert_eq!(readings.nox_index, None);
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
fn read_measured_raw_values_parses_unscaled_ticks() {
    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        word(5000u16, &mut payload); // raw humidity
        word(4000u16, &mut payload); // raw temperature
        word(150u16, &mut payload); // raw VOC
        word(20u16, &mut payload); // raw NOx

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());

        let raw = driver
            .read_measured_raw_values()
            .await
            .expect("read_measured_raw_values failed");
        assert_eq!(raw.humidity, 5000);
        assert_eq!(raw.temperature, 4000);
        assert_eq!(raw.voc, 150);
        assert_eq!(raw.nox, 20);

        driver.destroy()
    });

    // Raw values are 4 words = 12 wire bytes.
    assert_eq!(i2c.commands, vec![vec![0x03, 0xD2]]);
    assert_eq!(i2c.offset, 12);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn read_measured_pm_values_parses_all_words() {
    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        for value in [100u16, 250, 400, 1000, 50, 120, 300, 500, 900, 3200] {
            word(value, &mut payload);
        }

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());

        let pm = driver
            .read_measured_pm_values()
            .await
            .expect("read_measured_pm_values failed");
        assert_eq!(pm.mass_pm1_0, 100);
        assert_eq!(pm.mass_pm2_5, 250);
        assert_eq!(pm.mass_pm4_0, 400);
        assert_eq!(pm.mass_pm10_0, 1000);
        assert_eq!(pm.number_pm0_5, 50);
        assert_eq!(pm.number_pm1_0, 120);
        assert_eq!(pm.number_pm2_5, 300);
        assert_eq!(pm.number_pm4_0, 500);
        assert_eq!(pm.number_pm10_0, 900);
        assert_eq!(pm.typical_particle_size, 3200);

        driver.destroy()
    });

    // The extended PM block is 10 words = 30 wire bytes.
    assert_eq!(i2c.commands, vec![vec![0x04, 0x13]]);
    assert_eq!(i2c.offset, 30);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn get_version_parses_all_fields() {
    let (i2c, delay) = pollster::block_on(async {
        // Four words; the first seven data bytes carry the version fields.
        let mut payload = Vec::new();
        word(0x0102, &mut payload); // firmware 1.2
        word(0x0003, &mut payload); // debug off, hardware 3
        word(0x0004, &mut payload); // hardware minor 0, protocol 4
        word(0x0500, &mut payload); // protocol 5, unused byte

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());

        let version = driver
            .get_version()
            .await
            .expect("get_version failed");
        assert_eq!(version.firmware_major, 1);
        assert_eq!(version.firmware_minor, 2);
        assert!(!version.firmware_debug);
        assert_eq!(version.hardware_major, 3);
        assert_eq!(version.hardware_minor, 0);
        assert_eq!(version.protocol_major, 4);
        assert_eq!(version.protocol_minor, 5);

        driver.destroy()
    });

    // Mirrors the reference C driver, which reads 4 words = 12 wire bytes.
    assert_eq!(i2c.commands, vec![vec![0xD1, 0x00]]);
    assert_eq!(i2c.offset, 12);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn read_and_clear_device_status_returns_full_u32() {
    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        word(0x0001, &mut payload);
        word(0x0203, &mut payload);

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());

        let status = driver
            .read_and_clear_device_status()
            .await
            .expect("read_and_clear_device_status failed");
        assert_eq!(status, 0x0001_0203);

        driver.destroy()
    });

    assert_eq!(i2c.commands, vec![vec![0xD2, 0x10]]);
    assert_eq!(i2c.offset, 6);
    assert_eq!(delay.calls_us, vec![20_000]);
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
fn read_data_ready_polls_the_flag() {
    let (i2c, delay) = pollster::block_on(async {
        let mut ready_payload = Vec::new();
        word(0x0001, &mut ready_payload); // data ready

        let mut driver =
            Sen5xDriver::new(MockI2c::with_response(ready_payload), MockDelay::default());
        assert!(driver
            .read_data_ready()
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
            .read_data_ready()
            .await
            .expect("read failed")
    });
    assert!(!not_ready);
}

#[test]
fn request_data_ready_sends_command_without_delay() {
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

/// Builds the exact write frame (command + words + CRCs) the driver should
/// emit for a set command, mirroring the reference C driver's buffer layout.
fn set_frame(command: [u8; 2], words: &[u16]) -> Vec<u8> {
    let mut frame = vec![command[0], command[1]];
    for &w in words {
        word(w, &mut frame);
    }
    frame
}

#[test]
fn start_fan_cleaning_sends_command() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver.start_fan_cleaning().await.unwrap();
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x56, 0x07]]);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn start_measurement_without_pm_sends_command() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver.start_measurement_without_pm().await.unwrap();
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x00, 0x37]]);
    assert_eq!(delay.calls_us, vec![50_000]);
}

#[test]
fn set_and_get_temperature_offset_parameters() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver
            .set_temperature_offset_parameters(-100, 50, 300)
            .await
            .unwrap();
        driver.destroy()
    });
    assert_eq!(
        i2c.commands,
        vec![set_frame([0x60, 0xB2], &[(-100i16) as u16, 50, 300])]
    );
    assert_eq!(delay.calls_us, vec![20_000]);

    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        word((-100i16) as u16, &mut payload);
        word(50u16, &mut payload);
        word(300u16, &mut payload);

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        let params = driver
            .get_temperature_offset_parameters()
            .await
            .unwrap();
        assert_eq!(params.temp_offset, -100);
        assert_eq!(params.slope, 50);
        assert_eq!(params.time_constant, 300);
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x60, 0xB2]]);
    assert_eq!(i2c.offset, 9);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn set_and_get_warm_start_parameter() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver.set_warm_start_parameter(3333).await.unwrap();
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![set_frame([0x60, 0xC6], &[3333])]);
    assert_eq!(delay.calls_us, vec![20_000]);

    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        word(3333u16, &mut payload);

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        assert_eq!(driver.get_warm_start_parameter().await.unwrap(), 3333);
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x60, 0xC6]]);
    assert_eq!(i2c.offset, 3);
    assert_eq!(delay.calls_us, vec![20_000]);
}

fn tuning_params() -> sen5x_rs::TuningParameters {
    sen5x_rs::TuningParameters {
        index_offset: -100,
        learning_time_offset_hours: 12,
        learning_time_gain_hours: 24,
        gating_max_duration_minutes: 180,
        std_initial: 50,
        gain_factor: 230,
    }
}

fn tuning_words(params: &sen5x_rs::TuningParameters) -> [u16; 6] {
    [
        params.index_offset as u16,
        params.learning_time_offset_hours as u16,
        params.learning_time_gain_hours as u16,
        params.gating_max_duration_minutes as u16,
        params.std_initial as u16,
        params.gain_factor as u16,
    ]
}

#[test]
fn set_and_get_voc_algorithm_tuning_parameters() {
    let params = tuning_params();
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver
            .set_voc_algorithm_tuning_parameters(params)
            .await
            .unwrap();
        driver.destroy()
    });
    assert_eq!(
        i2c.commands,
        vec![set_frame([0x60, 0xD0], &tuning_words(&params))]
    );
    assert_eq!(delay.calls_us, vec![20_000]);

    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        for &w in &tuning_words(&params) {
            word(w, &mut payload);
        }

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        assert_eq!(driver.get_voc_algorithm_tuning_parameters().await.unwrap(), params);
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x60, 0xD0]]);
    assert_eq!(i2c.offset, 18);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn set_and_get_nox_algorithm_tuning_parameters() {
    let params = tuning_params();
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver
            .set_nox_algorithm_tuning_parameters(params)
            .await
            .unwrap();
        driver.destroy()
    });
    assert_eq!(
        i2c.commands,
        vec![set_frame([0x60, 0xE1], &tuning_words(&params))]
    );
    assert_eq!(delay.calls_us, vec![20_000]);

    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        for &w in &tuning_words(&params) {
            word(w, &mut payload);
        }

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        assert_eq!(driver.get_nox_algorithm_tuning_parameters().await.unwrap(), params);
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x60, 0xE1]]);
    assert_eq!(i2c.offset, 18);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn set_and_get_rht_acceleration_mode() {
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver.set_rht_acceleration_mode(2).await.unwrap();
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![set_frame([0x60, 0xF7], &[2])]);
    assert_eq!(delay.calls_us, vec![20_000]);

    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        word(2u16, &mut payload);

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        assert_eq!(driver.get_rht_acceleration_mode().await.unwrap(), 2);
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x60, 0xF7]]);
    assert_eq!(i2c.offset, 3);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn set_and_get_voc_algorithm_state() {
    let state = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04];
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver.set_voc_algorithm_state(&state).await.unwrap();
        driver.destroy()
    });
    let mut expected = vec![0x61, 0x81];
    for chunk in state.chunks_exact(2) {
        word(u16::from_be_bytes([chunk[0], chunk[1]]), &mut expected);
    }
    assert_eq!(i2c.commands, vec![expected]);
    assert_eq!(delay.calls_us, vec![20_000]);

    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        for chunk in state.chunks_exact(2) {
            word(u16::from_be_bytes([chunk[0], chunk[1]]), &mut payload);
        }

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        assert_eq!(driver.get_voc_algorithm_state().await.unwrap(), state);
        driver.destroy()
    });
    // Mirrors the reference C driver, which reads 4 words = 12 wire bytes.
    assert_eq!(i2c.commands, vec![vec![0x61, 0x81]]);
    assert_eq!(i2c.offset, 12);
    assert_eq!(delay.calls_us, vec![20_000]);
}

#[test]
fn set_voc_algorithm_state_rejects_odd_length() {
    let result = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver.set_voc_algorithm_state(&[1, 2, 3]).await
    });
    assert!(matches!(result, Err(Sen5xError::InvalidArgument)));
}

#[test]
fn set_and_get_fan_auto_cleaning_interval() {
    let interval = 0x0102_0304u32;
    let (i2c, delay) = pollster::block_on(async {
        let mut driver = Sen5xDriver::new(MockI2c::with_response(Vec::new()), MockDelay::default());
        driver
            .set_fan_auto_cleaning_interval(interval)
            .await
            .unwrap();
        driver.destroy()
    });
    assert_eq!(
        i2c.commands,
        vec![set_frame([0x80, 0x04], &[(interval >> 16) as u16, interval as u16])]
    );
    assert_eq!(delay.calls_us, vec![20_000]);

    let (i2c, delay) = pollster::block_on(async {
        let mut payload = Vec::new();
        word((interval >> 16) as u16, &mut payload);
        word(interval as u16, &mut payload);

        let mut driver = Sen5xDriver::new(MockI2c::with_response(payload), MockDelay::default());
        assert_eq!(driver.get_fan_auto_cleaning_interval().await.unwrap(), interval);
        driver.destroy()
    });
    assert_eq!(i2c.commands, vec![vec![0x80, 0x04]]);
    assert_eq!(i2c.offset, 6);
    assert_eq!(delay.calls_us, vec![20_000]);
}
