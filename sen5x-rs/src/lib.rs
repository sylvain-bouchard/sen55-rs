#![no_std]

use crc8::Crc8;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

const SEN5X_I2C_ADDRESS: u8 = 0x69;
const CRC_INIT: u8 = 0xFF;
const MAX_RESPONSE_WORDS: usize = 16;

// Datasheet-mandated response preparation times (mirrored from the reference
// C driver): after a command is sent, the SEN5x needs this long before its
// response can be clocked out. The sensor does not clock-stretch.
const DELAY_20MS_US: u32 = 20_000;
const DELAY_50MS_US: u32 = 50_000;
const DELAY_200MS_US: u32 = 200_000;

#[derive(Debug)]
pub enum Sen5xError<E> {
    I2c(E),
    Crc,
    InvalidData,
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

pub struct Sen5xDriver<I2C, D> {
    i2c: I2C,
    delay: D,
    crc: Crc8,
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
        Self {
            i2c,
            delay,
            crc: Crc8::create_msb(49),
        }
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

    async fn write_command(&mut self, command: [u8; 2]) -> Result<(), Sen5xError<E>> {
        self.i2c.write(SEN5X_I2C_ADDRESS, &command).await?;

        Ok(())
    }

    fn check_crc(&mut self, data: &[u8], expected_crc: u8) -> Result<(), Sen5xError<E>> {
        let calculated_crc = self.crc.calc(data, data.len() as i32, CRC_INIT);

        if calculated_crc != expected_crc {
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
