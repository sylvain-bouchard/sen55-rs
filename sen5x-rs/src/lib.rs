#![no_std]

use crc8::Crc8;
use embedded_hal::i2c::I2c;

const SEN5X_I2C_ADDRESS: u8 = 0x69;
const CRC_INIT: u8 = 0xFF;
const MAX_RESPONSE_WORDS: usize = 16;

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
    /// PM concentrations in µg/m³ × 10
    pub pm1_0: f32,
    pub pm2_5: f32,
    pub pm4_0: f32,
    pub pm10_0: f32,

    /// Relative humidity in % × 100
    pub humidity: Option<f32>,

    /// Temperature in °C × 200
    pub temperature: Option<f32>,

    /// VOC index × 10
    pub voc_index: Option<f32>,

    /// NOx index × 10
    pub nox_index: Option<f32>,
}

pub struct Sen5xDriver<I2C> {
    i2c: I2C,
    crc: Crc8,
}

impl<I2C, E> Sen5xDriver<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            crc: Crc8::create_msb(49),
        }
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }

    pub fn device_reset(&mut self) -> Result<(), Sen5xError<E>> {
        let command = [0xD3, 0x04];

        self.i2c.write(SEN5X_I2C_ADDRESS, &command)?;

        Ok(())
    }

    pub fn read_product_name(&mut self) -> Result<SensorString, Sen5xError<E>> {
        self.read_string32([0xD0, 0x14])
    }

    pub fn read_serial_number(&mut self) -> Result<SensorString, Sen5xError<E>> {
        self.read_string32([0xD0, 0x33])
    }

    pub fn read_device_status(&mut self) -> Result<u16, Sen5xError<E>> {
        let words = self.read_words::<1>([0xD2, 0x06])?;

        Ok(words[0])
    }

    pub fn start_measurement(&mut self) -> Result<(), Sen5xError<E>> {
        let command = [0x00, 0x21];

        self.i2c.write(SEN5X_I2C_ADDRESS, &command)?;

        Ok(())
    }

    pub fn stop_measurement(&mut self) -> Result<(), Sen5xError<E>> {
        let command = [0x01, 0x04];

        self.i2c.write(SEN5X_I2C_ADDRESS, &command)?;

        Ok(())
    }

    /// Triggers the data-ready state check sequence.
    /// **NOTE:** The calling runtime MUST wait at least 20ms before calling `read_data_ready_response`.
    pub fn request_data_ready(&mut self) -> Result<(), Sen5xError<E>> {
        let command = [0x02, 0x02];

        self.i2c.write(SEN5X_I2C_ADDRESS, &command)?;

        Ok(())
    }

    /// Reads back and validates the 3-byte data-ready status packet from the sensor.
    pub fn read_data_ready_response(&mut self) -> Result<bool, Sen5xError<E>> {
        let data = self.read_bytes::<2>([0x02, 0x02])?;

        Ok(data[1] == 1)
    }

    /// Reads out the 24-byte data block containing air metrics and converts them into physical units.
    pub fn read_measurements(&mut self) -> Result<Sen5xMeasurements, Sen5xError<E>> {
        let words = self.read_words::<8>([0x03, 0xC4])?;

        let pm1_0 = words[0] as f32 / 10.0;
        let pm2_5 = words[1] as f32 / 10.0;
        let pm4_0 = words[2] as f32 / 10.0;
        let pm10_0 = words[3] as f32 / 10.0;

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

    fn check_crc(&mut self, data: &[u8], expected_crc: u8) -> Result<(), Sen5xError<E>> {
        let calculated_crc = self.crc.calc(data, data.len() as i32, CRC_INIT);

        if calculated_crc != expected_crc {
            Err(Sen5xError::Crc)
        } else {
            Ok(())
        }
    }

    fn read_bytes<const N: usize>(&mut self, command: [u8; 2]) -> Result<[u8; N], Sen5xError<E>> {
        let mut buffer = [0u8; MAX_RESPONSE_WORDS * 3];

        let response_len = (N / 2) * 3;
        let response = &mut buffer[..response_len];

        self.i2c.write_read(SEN5X_I2C_ADDRESS, &command, response)?;

        let mut bytes = [0u8; N];

        for (index, chunk) in response.chunks_exact(3).enumerate() {
            self.check_crc(&chunk[..2], chunk[2])?;

            bytes[index * 2] = chunk[0];
            bytes[index * 2 + 1] = chunk[1];
        }

        Ok(bytes)
    }

    fn read_words<const N: usize>(&mut self, command: [u8; 2]) -> Result<[u16; N], Sen5xError<E>> {
        const MAX_WORD_BYTES: usize = MAX_RESPONSE_WORDS * 2;

        let bytes = self.read_bytes::<MAX_WORD_BYTES>(command)?;

        let mut words = [0u16; N];

        for (index, chunk) in bytes[..N * 2].chunks_exact(2).enumerate() {
            words[index] = u16::from_be_bytes([chunk[0], chunk[1]]);
        }

        Ok(words)
    }

    fn read_string32(&mut self, command: [u8; 2]) -> Result<SensorString, Sen5xError<E>> {
        let bytes = self.read_bytes::<32>(command)?;

        Ok(SensorString::from_bytes(bytes))
    }
}
