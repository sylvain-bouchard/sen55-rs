#![no_std]

use crc8::Crc8;
use embedded_hal::i2c::I2c;

const SEN5X_I2C_ADDRESS: u8 = 0x69;

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

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Sen5xMeasurements {
    pub pm1_0: f32,
    pub pm2_5: f32,
    pub pm4_0: f32,
    pub pm10_0: f32,
    pub humidity: Option<f32>,
    pub temperature: Option<f32>,
    pub voc_index: Option<f32>,
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
        let mut buffer = [0u8; 3];
        self.i2c.read(SEN5X_I2C_ADDRESS, &mut buffer)?;

        let calculated_crc = self.crc.calc(&buffer[0..2], 2, 0xFF);
        if calculated_crc != buffer[2] {
            return Err(Sen5xError::Crc);
        }

        // The sensor returns 0x01 in the second byte if a fresh sample is ready
        Ok(buffer[1] == 1)
    }

    /// Reads out the 24-byte data block containing air metrics and converts them into physical units.
    pub fn read_measurements(&mut self) -> Result<Sen5xMeasurements, Sen5xError<E>> {
        let command = [0x03, 0xC4];
        let mut buffer = [0u8; 24]; // 8 telemetry fields * (2 data bytes + 1 CRC byte)

        self.i2c
            .write_read(SEN5X_I2C_ADDRESS, &command, &mut buffer)?;

        // Validate the CRC-8 byte appended to every 2-byte word chunk
        for chunk in buffer.chunks_exact(3) {
            let calculated_crc = self.crc.calc(&chunk[0..2], 2, 0xFF);
            if calculated_crc != chunk[2] {
                return Err(Sen5xError::Crc);
            }
        }

        // Helper closures to parse big-endian words out of the raw response
        let read_u16 =
            |idx: usize| -> u16 { u16::from_be_bytes([buffer[idx * 3], buffer[idx * 3 + 1]]) };
        let read_i16 =
            |idx: usize| -> i16 { i16::from_be_bytes([buffer[idx * 3], buffer[idx * 3 + 1]]) };

        // PM concentrations are scaled by 10.0
        let pm1_0 = (read_u16(0) as f32) / 10.0;
        let pm2_5 = (read_u16(1) as f32) / 10.0;
        let pm4_0 = (read_u16(2) as f32) / 10.0;
        let pm10_0 = (read_u16(3) as f32) / 10.0;

        // Sensirion sets unused or unsupported metrics (e.g., NOx on a SEN54) to 0x7FFF
        let humidity = match read_i16(4) {
            0x7FFF => None,
            val => Some((val as f32) / 100.0),
        };

        let temperature = match read_i16(5) {
            0x7FFF => None,
            val => Some((val as f32) / 200.0),
        };

        let voc_index = match read_i16(6) {
            0x7FFF => None,
            val => Some((val as f32) / 10.0),
        };

        let nox_index = match read_i16(7) {
            0x7FFF => None,
            val => Some((val as f32) / 10.0),
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
}
