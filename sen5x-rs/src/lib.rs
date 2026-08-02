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

    pub fn is_data_ready(&mut self) -> Result<bool, Sen5xError<E>> {
        let command = [0x02, 0x02];
        let mut buffer = [0u8; 3]; // 2 data bytes + 1 CRC byte

        // Write the command and read back the 3-byte status packet
        self.i2c
            .write_read(SEN5X_I2C_ADDRESS, &command, &mut buffer)?;

        // The crc8 crate's .calc() method takes a mutable reference to self because it updates
        // internal bit tracking fields during execution, so we use self.crc directly.
        let calculated_crc = self.crc.calc(&buffer[0..2], 2, 0xFF);

        // Validate the integrity of the data bytes against the received CRC byte
        if calculated_crc != buffer[2] {
            return Err(Sen5xError::Crc);
        }

        // The sensor returns 0x01 in the second byte (buffer[1]) if a fresh sample is ready
        Ok(buffer[1] == 1)
    }
}
