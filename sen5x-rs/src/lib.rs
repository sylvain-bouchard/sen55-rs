#![no_std]

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

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
    pub async fn read_data_ready_response(&mut self) -> Result<bool, Sen5xError<E>> {
        let data = self.read_bytes::<2>([0x02, 0x02], DELAY_20MS_US).await?;

        Ok(data[1] == 1)
    }

    /// Reads out the 24-byte data block containing air metrics and converts
    /// them into physical units.
    pub async fn read_measurements(&mut self) -> Result<Sen5xMeasurements, Sen5xError<E>> {
        let words = self.read_words::<8>([0x03, 0xC4], DELAY_20MS_US).await?;

        // PM channels report 0xFFFF when no data is available (e.g. before the
        // first sample); map it to `None` like the 0x7FFF sentinels below.
        let pm = |raw: u16| match raw {
            0xFFFF => None,
            value => Some(value as f32 / 10.0),
        };

        let pm1_0 = pm(words[0]);
        let pm2_5 = pm(words[1]);
        let pm4_0 = pm(words[2]);
        let pm10_0 = pm(words[3]);

        let humidity_raw = words[4] as i16;
        let temperature_raw = words[5] as i16;
        let voc_raw = words[6] as i16;
        let nox_raw = words[7] as i16;

        let humidity = match humidity_raw {
            0x7FFF => None,
            value => Some(value as f32 / 100.0),
        };

        let temperature = match temperature_raw {
            0x7FFF => None,
            value => Some(value as f32 / 200.0),
        };

        let voc_index = match voc_raw {
            0x7FFF => None,
            value => Some(value as f32 / 10.0),
        };

        let nox_index = match nox_raw {
            0x7FFF => None,
            value => Some(value as f32 / 10.0),
        };

        Ok(Sen5xMeasurements {
            pm1_0,
            pm2_5,
            pm4_0,
            pm10_0,
            humidity,
            temperature,
            voc_index,
            nox_index,
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

    async fn write_command(&mut self, command: [u8; 2]) -> Result<(), Sen5xError<E>> {
        self.i2c.write(SEN5X_I2C_ADDRESS, &command).await?;

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
