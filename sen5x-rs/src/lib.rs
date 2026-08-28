#![no_std]

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use sen5x_conversion::{humidity_reading, index_reading, pm_reading, temperature_reading};

const SEN5X_I2C_ADDRESS: u8 = 0x69;
const MAX_RESPONSE_WORDS: usize = 16;

// Datasheet-mandated response preparation times (mirrored from the reference
// C driver): after a command is sent, the SEN5x needs this long before its
// response can be clocked out. The sensor does not clock-stretch.
const DELAY_20MS_US: u32 = 20_000;
const DELAY_50MS_US: u32 = 50_000;
const DELAY_200MS_US: u32 = 200_000;

/// Sensirion CRC-8: polynomial 0x31, MSB-first, init 0xFF.
///
/// Inlined here so the crate stays `no_std`; the `crc8` crate does not
/// declare `#![no_std]`.
fn crc8(data: &[u8]) -> u8 {
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

#[derive(Debug)]
pub enum Sen5xError<E> {
    I2c(E),
    Crc,
    InvalidArgument,
}

impl<E> From<E> for Sen5xError<E> {
    fn from(error: E) -> Self {
        Self::I2c(error)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SensorString {
    bytes: [u8; 32],
    len: usize,
}

impl SensorString {
    fn from_bytes(bytes: [u8; 32]) -> Self {
        let len = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());

        Self { bytes, len }
    }

    pub fn as_str(&self) -> Result<&str, core::str::Utf8Error> {
        core::str::from_utf8(&self.bytes[..self.len])
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Sen5xMeasurements {
    /// PM concentrations in µg/m³, `None` when the sensor reports no data (0xFFFF)
    pub pm1_0: Option<f32>,
    pub pm2_5: Option<f32>,
    pub pm4_0: Option<f32>,
    pub pm10_0: Option<f32>,

    /// Relative humidity in % × 100
    pub humidity: Option<f32>,

    /// Temperature in °C × 200
    pub temperature: Option<f32>,

    /// VOC index × 10
    pub voc_index: Option<f32>,

    /// NOx index × 10
    pub nox_index: Option<f32>,
}

/// Raw (unscaled) ticks from the 0x03D2 command: humidity (÷ 100 for %RH)
/// and temperature (÷ 200 for °C) share the ambient scale factors; VOC and
/// NOx have no scale factor.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Sen5xRawMeasurements {
    pub humidity: i16,
    pub temperature: i16,
    pub voc: u16,
    pub nox: u16,
}

/// Extended PM block from the 0x0413 command. The `mass_pm*` fields are mass
/// concentrations (÷ 10 for µg/m³); the `number_pm*` fields are number
/// concentrations and `typical_particle_size` is unscaled. All values are
/// the raw fixed-point words the sensor emits.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Sen5xPmMeasurements {
    pub mass_pm1_0: u16,
    pub mass_pm2_5: u16,
    pub mass_pm4_0: u16,
    pub mass_pm10_0: u16,
    pub number_pm0_5: u16,
    pub number_pm1_0: u16,
    pub number_pm2_5: u16,
    pub number_pm4_0: u16,
    pub number_pm10_0: u16,
    pub typical_particle_size: u16,
}

/// Firmware, hardware and protocol version from the 0xD100 command.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Sen5xVersion {
    pub firmware_major: u8,
    pub firmware_minor: u8,
    pub firmware_debug: bool,
    pub hardware_major: u8,
    pub hardware_minor: u8,
    pub protocol_major: u8,
    pub protocol_minor: u8,
}

/// Temperature offset parameters from the 0x60B2 command: a constant offset
/// (÷ 200 for °C), a normalized slope (÷ 10000) and a time constant in
/// seconds.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TemperatureOffsetParameters {
    pub temp_offset: i16,
    pub slope: i16,
    pub time_constant: u16,
}

/// VOC/NOx algorithm tuning parameters from the 0x60D0/0x60E1 commands.
/// Both algorithms share the same six fields, all scaled as raw integers.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TuningParameters {
    pub index_offset: i16,
    pub learning_time_offset_hours: i16,
    pub learning_time_gain_hours: i16,
    pub gating_max_duration_minutes: i16,
    pub std_initial: i16,
    pub gain_factor: i16,
}

pub struct Sen5xDriver<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D, E> Sen5xDriver<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    /// Creates a driver over the given I2C bus and delay provider.
    ///
    /// The delay is used to honor the command/response timing the SEN5x
    /// requires between sending a command and reading its response. On
    /// Embassy, `embassy_time::Delay` implements `embedded_hal_async::delay::DelayNs`,
    /// or pass a small wrapper implementing `delay_ns` (the only required
    /// method) that awaits `Timer::after_micros`; `delay_us`/`delay_ms` are
    /// provided by the trait.
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self { i2c, delay }
    }

    pub fn destroy(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    pub async fn device_reset(&mut self) -> Result<(), Sen5xError<E>> {
        self.write_command([0xD3, 0x04]).await?;

        // The device reboots after the reset; give it time before the next
        // command is issued.
        self.delay.delay_us(DELAY_200MS_US).await;

        Ok(())
    }

    pub async fn read_product_name(&mut self) -> Result<SensorString, Sen5xError<E>> {
        self.read_string32([0xD0, 0x14], DELAY_50MS_US).await
    }

    pub async fn read_serial_number(&mut self) -> Result<SensorString, Sen5xError<E>> {
        self.read_string32([0xD0, 0x33], DELAY_50MS_US).await
    }

    /// Reads the 32-bit device status register.
    ///
    /// The status flags are bit-encoded in a 32-bit integer (see the SEN5x
    /// datasheet for the flag bit assignments).
    pub async fn read_device_status(&mut self) -> Result<u32, Sen5xError<E>> {
        let words = self.read_words::<2>([0xD2, 0x06], DELAY_20MS_US).await?;

        Ok(((words[0] as u32) << 16) | words[1] as u32)
    }

    pub async fn start_measurement(&mut self) -> Result<(), Sen5xError<E>> {
        self.write_command([0x00, 0x21]).await?;

        self.delay.delay_us(DELAY_50MS_US).await;

        Ok(())
    }

    /// Starts the measurement in a power-reduced mode without the PM fan and
    /// laser (0x0037), for sensor variants without particle measurement.
    pub async fn start_measurement_without_pm(&mut self) -> Result<(), Sen5xError<E>> {
        self.write_command([0x00, 0x37]).await?;

        self.delay.delay_us(DELAY_50MS_US).await;

        Ok(())
    }

    pub async fn stop_measurement(&mut self) -> Result<(), Sen5xError<E>> {
        self.write_command([0x01, 0x04]).await?;

        self.delay.delay_us(DELAY_200MS_US).await;

        Ok(())
    }

    /// Triggers the data-ready state check sequence.
    ///
    /// This is optional: `read_data_ready_response` sends the command itself
    /// and handles the required wait internally. This method only lets the
    /// caller fire the command early.
    pub async fn request_data_ready(&mut self) -> Result<(), Sen5xError<E>> {
        self.write_command([0x02, 0x02]).await
    }

    /// Polls the data-ready flag by sending the 0x0202 command, waiting the
    /// datasheet-mandated 20 ms, then reading back the 3-byte status packet.
    ///
    /// Mirrors the reference C driver, which assigns the raw second data byte
    /// to the output (`*data_ready = buffer[1]`): any nonzero value reports
    /// ready, not just exactly 1.
    pub async fn read_data_ready_response(&mut self) -> Result<bool, Sen5xError<E>> {
        let data = self.read_bytes::<2>([0x02, 0x02], DELAY_20MS_US).await?;

        Ok(data[1] != 0)
    }

    /// Reads out the 24-byte data block containing air metrics and converts
    /// them into physical units.
    ///
    /// The `0xFFFF`/`0x7FFF` "no data" sentinels and the scale factors are
    /// defined once in [`sen5x_conversion`], the same crate the C reference
    /// facade uses, so the two drivers cannot drift on this logic.
    pub async fn read_measurements(&mut self) -> Result<Sen5xMeasurements, Sen5xError<E>> {
        let words = self.read_words::<8>([0x03, 0xC4], DELAY_20MS_US).await?;

        Ok(Sen5xMeasurements {
            pm1_0: pm_reading(words[0]),
            pm2_5: pm_reading(words[1]),
            pm4_0: pm_reading(words[2]),
            pm10_0: pm_reading(words[3]),
            humidity: humidity_reading(words[4] as i16),
            temperature: temperature_reading(words[5] as i16),
            voc_index: index_reading(words[6] as i16),
            nox_index: index_reading(words[7] as i16),
        })
    }

    /// Reads the raw, unscaled humidity, temperature, VOC and NOx ticks.
    ///
    /// Raw humidity and temperature share the ambient scale factors
    /// (÷ 100 for RH%, ÷ 200 for °C); raw VOC and NOx are plain ticks with
    /// no documented scale factor.
    pub async fn read_measured_raw_values(
        &mut self,
    ) -> Result<Sen5xRawMeasurements, Sen5xError<E>> {
        let words = self.read_words::<4>([0x03, 0xD2], DELAY_20MS_US).await?;

        Ok(Sen5xRawMeasurements {
            humidity: words[0] as i16,
            temperature: words[1] as i16,
            voc: words[2],
            nox: words[3],
        })
    }

    /// Reads the extended PM block: mass concentrations (÷ 10 for µg/m³),
    /// number concentrations and typical particle size, all as raw words.
    pub async fn read_measured_pm_values(
        &mut self,
    ) -> Result<Sen5xPmMeasurements, Sen5xError<E>> {
        let words = self.read_words::<10>([0x04, 0x13], DELAY_20MS_US).await?;

        Ok(Sen5xPmMeasurements {
            mass_pm1_0: words[0],
            mass_pm2_5: words[1],
            mass_pm4_0: words[2],
            mass_pm10_0: words[3],
            number_pm0_5: words[4],
            number_pm1_0: words[5],
            number_pm2_5: words[6],
            number_pm4_0: words[7],
            number_pm10_0: words[8],
            typical_particle_size: words[9],
        })
    }

    /// Reads the firmware, hardware and protocol version.
    ///
    /// Mirrors the reference C driver, which clocks out four words (12 wire
    /// bytes) for this command and uses the first seven data bytes; the
    /// protocol minor version is the low byte of the fourth word.
    pub async fn get_version(&mut self) -> Result<Sen5xVersion, Sen5xError<E>> {
        let words = self.read_words::<4>([0xD1, 0x00], DELAY_20MS_US).await?;

        let byte = |index: usize| {
            let word = words[index / 2];
            if index % 2 == 0 {
                (word >> 8) as u8
            } else {
                (word & 0xFF) as u8
            }
        };

        Ok(Sen5xVersion {
            firmware_major: byte(0),
            firmware_minor: byte(1),
            firmware_debug: byte(2) != 0,
            hardware_major: byte(3),
            hardware_minor: byte(4),
            protocol_major: byte(5),
            protocol_minor: byte(6),
        })
    }

    /// Reads and clears the 32-bit device status register, like
    /// [`read_device_status`](Self::read_device_status) but resetting the
    /// flags after the read.
    pub async fn read_and_clear_device_status(&mut self) -> Result<u32, Sen5xError<E>> {
        let words = self.read_words::<2>([0xD2, 0x10], DELAY_20MS_US).await?;

        Ok(((words[0] as u32) << 16) | words[1] as u32)
    }

    /// Starts the fan cleaning manually. Only available in measure mode with
    /// PM measurement enabled; does nothing otherwise.
    pub async fn start_fan_cleaning(&mut self) -> Result<(), Sen5xError<E>> {
        self.write_command([0x56, 0x07]).await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Sets the temperature offset parameters (0x60B2).
    pub async fn set_temperature_offset_parameters(
        &mut self,
        temp_offset: i16,
        slope: i16,
        time_constant: u16,
    ) -> Result<(), Sen5xError<E>> {
        self.write_command_with_words(
            [0x60, 0xB2],
            &[temp_offset as u16, slope as u16, time_constant],
        )
        .await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Reads the currently configured temperature offset parameters.
    pub async fn get_temperature_offset_parameters(
        &mut self,
    ) -> Result<TemperatureOffsetParameters, Sen5xError<E>> {
        let words = self.read_words::<3>([0x60, 0xB2], DELAY_20MS_US).await?;

        Ok(TemperatureOffsetParameters {
            temp_offset: words[0] as i16,
            slope: words[1] as i16,
            time_constant: words[2],
        })
    }

    /// Sets the warm start behavior (0x60C6): 0 = cold start, 1 = warm start.
    pub async fn set_warm_start_parameter(&mut self, warm_start: u16) -> Result<(), Sen5xError<E>> {
        self.write_command_with_words([0x60, 0xC6], &[warm_start]).await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Reads the configured warm start behavior.
    pub async fn get_warm_start_parameter(&mut self) -> Result<u16, Sen5xError<E>> {
        let words = self.read_words::<1>([0x60, 0xC6], DELAY_20MS_US).await?;

        Ok(words[0])
    }

    /// Sets the VOC algorithm tuning parameters (0x60D0).
    pub async fn set_voc_algorithm_tuning_parameters(
        &mut self,
        params: TuningParameters,
    ) -> Result<(), Sen5xError<E>> {
        self.write_command_with_words(
            [0x60, 0xD0],
            &[
                params.index_offset as u16,
                params.learning_time_offset_hours as u16,
                params.learning_time_gain_hours as u16,
                params.gating_max_duration_minutes as u16,
                params.std_initial as u16,
                params.gain_factor as u16,
            ],
        )
        .await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Reads the VOC algorithm tuning parameters.
    pub async fn get_voc_algorithm_tuning_parameters(
        &mut self,
    ) -> Result<TuningParameters, Sen5xError<E>> {
        let words = self.read_words::<6>([0x60, 0xD0], DELAY_20MS_US).await?;

        Ok(TuningParameters {
            index_offset: words[0] as i16,
            learning_time_offset_hours: words[1] as i16,
            learning_time_gain_hours: words[2] as i16,
            gating_max_duration_minutes: words[3] as i16,
            std_initial: words[4] as i16,
            gain_factor: words[5] as i16,
        })
    }

    /// Sets the NOx algorithm tuning parameters (0x60E1).
    pub async fn set_nox_algorithm_tuning_parameters(
        &mut self,
        params: TuningParameters,
    ) -> Result<(), Sen5xError<E>> {
        self.write_command_with_words(
            [0x60, 0xE1],
            &[
                params.index_offset as u16,
                params.learning_time_offset_hours as u16,
                params.learning_time_gain_hours as u16,
                params.gating_max_duration_minutes as u16,
                params.std_initial as u16,
                params.gain_factor as u16,
            ],
        )
        .await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Reads the NOx algorithm tuning parameters.
    pub async fn get_nox_algorithm_tuning_parameters(
        &mut self,
    ) -> Result<TuningParameters, Sen5xError<E>> {
        let words = self.read_words::<6>([0x60, 0xE1], DELAY_20MS_US).await?;

        Ok(TuningParameters {
            index_offset: words[0] as i16,
            learning_time_offset_hours: words[1] as i16,
            learning_time_gain_hours: words[2] as i16,
            gating_max_duration_minutes: words[3] as i16,
            std_initial: words[4] as i16,
            gain_factor: words[5] as i16,
        })
    }

    /// Sets the RH/T acceleration mode (0x60F7): 0 = slow, 1 = medium,
    /// 2 = fast.
    pub async fn set_rht_acceleration_mode(&mut self, mode: u16) -> Result<(), Sen5xError<E>> {
        self.write_command_with_words([0x60, 0xF7], &[mode]).await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Reads the configured RH/T acceleration mode.
    pub async fn get_rht_acceleration_mode(&mut self) -> Result<u16, Sen5xError<E>> {
        let words = self.read_words::<1>([0x60, 0xF7], DELAY_20MS_US).await?;

        Ok(words[0])
    }

    /// Restores a previously captured VOC algorithm state (0x6181). Only
    /// applied on the next measurement start. The state must be an even
    /// number of bytes (each word is CRC-protected on the wire).
    pub async fn set_voc_algorithm_state(&mut self, state: &[u8]) -> Result<(), Sen5xError<E>> {
        self.write_command_with_payload([0x61, 0x81], state).await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Captures the current VOC algorithm state (8 bytes) so it can be
    /// restored later.
    ///
    /// Mirrors the reference C driver, which sends the *set* command code
    /// 0x6181 and reads back 4 words.
    pub async fn get_voc_algorithm_state(&mut self) -> Result<[u8; 8], Sen5xError<E>> {
        let words = self.read_words::<4>([0x61, 0x81], DELAY_20MS_US).await?;

        let mut state = [0u8; 8];
        for (index, word) in words.iter().enumerate() {
            state[index * 2] = (word >> 8) as u8;
            state[index * 2 + 1] = (word & 0xFF) as u8;
        }

        Ok(state)
    }

    /// Sets the fan auto cleaning interval in seconds (0x8004).
    pub async fn set_fan_auto_cleaning_interval(
        &mut self,
        interval: u32,
    ) -> Result<(), Sen5xError<E>> {
        self.write_command_with_words(
            [0x80, 0x04],
            &[(interval >> 16) as u16, interval as u16],
        )
        .await?;

        self.delay.delay_us(DELAY_20MS_US).await;

        Ok(())
    }

    /// Reads the configured fan auto cleaning interval in seconds.
    pub async fn get_fan_auto_cleaning_interval(&mut self) -> Result<u32, Sen5xError<E>> {
        let words = self.read_words::<2>([0x80, 0x04], DELAY_20MS_US).await?;

        Ok(((words[0] as u32) << 16) | words[1] as u32)
    }

    async fn write_command(&mut self, command: [u8; 2]) -> Result<(), Sen5xError<E>> {
        self.i2c.write(SEN5X_I2C_ADDRESS, &command).await?;

        Ok(())
    }

    /// Writes a command followed by data words, each CRC-protected, in one
    /// I2C frame like the reference C driver's `sensirion_i2c_write_data`.
    async fn write_command_with_words(
        &mut self,
        command: [u8; 2],
        words: &[u16],
    ) -> Result<(), Sen5xError<E>> {
        if words.len() > MAX_RESPONSE_WORDS {
            return Err(Sen5xError::InvalidArgument);
        }

        let mut payload = [0u8; MAX_RESPONSE_WORDS * 2];
        for (index, &word) in words.iter().enumerate() {
            payload[index * 2..index * 2 + 2].copy_from_slice(&word.to_be_bytes());
        }

        self.write_command_with_payload(command, &payload[..words.len() * 2])
            .await
    }

    /// Writes a command followed by an even number of raw data bytes, each
    /// 2-byte word CRC-protected, in one I2C frame. Mirrors the reference C
    /// driver, which rejects odd payload lengths with `BYTE_NUM_ERROR`.
    async fn write_command_with_payload(
        &mut self,
        command: [u8; 2],
        payload: &[u8],
    ) -> Result<(), Sen5xError<E>> {
        if payload.len() % 2 != 0 || payload.len() / 2 > MAX_RESPONSE_WORDS {
            return Err(Sen5xError::InvalidArgument);
        }

        let mut buffer = [0u8; MAX_RESPONSE_WORDS * 3];
        buffer[0] = command[0];
        buffer[1] = command[1];

        let mut len = 2;
        for chunk in payload.chunks_exact(2) {
            buffer[len] = chunk[0];
            buffer[len + 1] = chunk[1];
            buffer[len + 2] = crc8(&buffer[len..len + 2]);
            len += 3;
        }

        self.i2c.write(SEN5X_I2C_ADDRESS, &buffer[..len]).await?;

        Ok(())
    }

    fn check_crc(&self, data: &[u8], expected_crc: u8) -> Result<(), Sen5xError<E>> {
        if crc8(data) != expected_crc {
            Err(Sen5xError::Crc)
        } else {
            Ok(())
        }
    }

    /// Reads a response consisting of `N` data bytes, where every two data
    /// bytes are followed by a CRC byte. `delay_us` is the wait required
    /// between sending the command and reading the response.
    async fn read_bytes<const N: usize>(
        &mut self,
        command: [u8; 2],
        delay_us: u32,
    ) -> Result<[u8; N], Sen5xError<E>> {
        if N % 2 != 0 || N / 2 > MAX_RESPONSE_WORDS {
            return Err(Sen5xError::InvalidArgument);
        }

        let mut buffer = [0u8; MAX_RESPONSE_WORDS * 3];
        let response_len = (N / 2) * 3;
        debug_assert!(
            response_len <= buffer.len(),
            "response length exceeds MAX_RESPONSE_WORDS"
        );
        let response = &mut buffer[..response_len];

        self.i2c.write(SEN5X_I2C_ADDRESS, &command).await?;
        self.delay.delay_us(delay_us).await;
        self.i2c.read(SEN5X_I2C_ADDRESS, response).await?;

        let mut bytes = [0u8; N];

        for (index, chunk) in response.chunks_exact(3).enumerate() {
            self.check_crc(&chunk[..2], chunk[2])?;

            bytes[index * 2] = chunk[0];
            bytes[index * 2 + 1] = chunk[1];
        }

        Ok(bytes)
    }

    /// Reads a response of `N` words (2 data bytes + 1 CRC byte per word),
    /// clocking out exactly `N * 3` bytes from the bus.
    async fn read_words<const N: usize>(
        &mut self,
        command: [u8; 2],
        delay_us: u32,
    ) -> Result<[u16; N], Sen5xError<E>> {
        if N > MAX_RESPONSE_WORDS {
            return Err(Sen5xError::InvalidArgument);
        }

        let mut buffer = [0u8; MAX_RESPONSE_WORDS * 3];
        let response_len = N * 3;
        debug_assert!(
            response_len <= buffer.len(),
            "response length exceeds MAX_RESPONSE_WORDS"
        );
        let response = &mut buffer[..response_len];

        self.i2c.write(SEN5X_I2C_ADDRESS, &command).await?;
        self.delay.delay_us(delay_us).await;
        self.i2c.read(SEN5X_I2C_ADDRESS, response).await?;

        let mut words = [0u16; N];

        for (index, chunk) in response.chunks_exact(3).enumerate() {
            self.check_crc(&chunk[..2], chunk[2])?;

            words[index] = u16::from_be_bytes([chunk[0], chunk[1]]);
        }

        Ok(words)
    }

    async fn read_string32(
        &mut self,
        command: [u8; 2],
        delay_us: u32,
    ) -> Result<SensorString, Sen5xError<E>> {
        let bytes = self.read_bytes::<32>(command, delay_us).await?;

        Ok(SensorString::from_bytes(bytes))
    }
}

#[cfg(test)]
mod tests {
    use super::{Sen5xDriver, Sen5xError, MAX_RESPONSE_WORDS};
    use embedded_hal_async::delay::DelayNs;
    use embedded_hal_async::i2c::{Error as I2cError, ErrorKind, ErrorType, I2c, Operation};

    #[derive(Debug)]
    struct MockError;

    impl I2cError for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    #[derive(Debug, Default)]
    struct MockI2c;

    impl ErrorType for MockI2c {
        type Error = MockError;
    }

    impl I2c for MockI2c {
        async fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), MockError> {
            Ok(())
        }
    }

    #[derive(Debug, Default)]
    struct MockDelay;

    impl DelayNs for MockDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn oversized_write_payload_is_rejected_before_i2c() {
        let result = pollster::block_on(async {
            let mut driver = Sen5xDriver::new(MockI2c, MockDelay);
            let words = [0u16; MAX_RESPONSE_WORDS + 1];
            driver.write_command_with_words([0, 0], &words).await
        });

        assert!(matches!(result, Err(Sen5xError::InvalidArgument)));
    }

    #[test]
    fn oversized_and_odd_write_payloads_are_rejected_before_i2c() {
        let result = pollster::block_on(async {
            let mut driver = Sen5xDriver::new(MockI2c, MockDelay);
            let oversized = [0u8; MAX_RESPONSE_WORDS * 2 + 2];
            let odd = [0u8; 1];

            let oversized_result = driver
                .write_command_with_payload([0, 0], &oversized)
                .await;
            let odd_result = driver.write_command_with_payload([0, 0], &odd).await;

            (oversized_result, odd_result)
        });

        assert!(matches!(result.0, Err(Sen5xError::InvalidArgument)));
        assert!(matches!(result.1, Err(Sen5xError::InvalidArgument)));
    }

    #[test]
    fn invalid_read_sizes_are_rejected_before_i2c() {
        let odd = pollster::block_on(async {
            let mut driver = Sen5xDriver::new(MockI2c, MockDelay);
            driver.read_bytes::<1>([0, 0], 0).await
        });
        let oversized_bytes = pollster::block_on(async {
            let mut driver = Sen5xDriver::new(MockI2c, MockDelay);
            driver.read_bytes::<{ MAX_RESPONSE_WORDS * 2 + 2 }>([0, 0], 0).await
        });
        let oversized_words = pollster::block_on(async {
            let mut driver = Sen5xDriver::new(MockI2c, MockDelay);
            driver.read_words::<{ MAX_RESPONSE_WORDS + 1 }>([0, 0], 0).await
        });

        assert!(matches!(odd, Err(Sen5xError::InvalidArgument)));
        assert!(matches!(
            oversized_bytes,
            Err(Sen5xError::InvalidArgument)
        ));
        assert!(matches!(
            oversized_words,
            Err(Sen5xError::InvalidArgument)
        ));
    }
}
