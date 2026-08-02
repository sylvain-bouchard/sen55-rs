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

pub struct Sen5xDriver<I2C> {
    i2c: I2C,
    crc: Crc8,
}

impl<I2C, E> Sen5xDriver<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, crc: Crc8::create_msb(49), }
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
}
