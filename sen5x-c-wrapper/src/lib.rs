#![no_std]

pub mod ffi;

use sen5x_conversion::{humidity_reading, index_reading, pm_reading, temperature_reading};

/// Custom error type wrapping the raw C status return values
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Sen5xError {
    DriverError(i16),
}

/// Labeled sensor measurement data packet parsed out into clean types
#[derive(Debug, Clone, PartialEq)]
pub struct SensorReadings {
    pub pm1_0: Option<f32>,       // µg/m³ (None when unavailable, raw 0xFFFF)
    pub pm2_5: Option<f32>,       // µg/m³ (None when unavailable, raw 0xFFFF)
    pub pm4_0: Option<f32>,       // µg/m³ (None when unavailable, raw 0xFFFF)
    pub pm10_0: Option<f32>,      // µg/m³ (None when unavailable, raw 0xFFFF)
    pub humidity: Option<f32>,    // %RH (Option because some variants omit it)
    pub temperature: Option<f32>, // °C
    pub voc_index: Option<f32>,   // 1-500 point index
    pub nox_index: Option<f32>,   // 1-500 point index
}

/// The safe Rust driver abstraction for the SEN5x air quality module
pub struct Sen5xDriver;

impl Sen5xDriver {
    /// Initializer mimicking the device handle lifecycle
    pub fn new() -> Self {
        Self
    }
}

impl Default for Sen5xDriver {
    fn default() -> Self {
        Self
    }
}

impl Sen5xDriver {
    /// Triggers a hard command reset on the remote SEN5x module over the I2C bus
    pub fn device_reset(&mut self) -> Result<(), Sen5xError> {
        let rc = unsafe { ffi::sen5x_device_reset() };
        if rc == 0 {
            Ok(())
        } else {
            Err(Sen5xError::DriverError(rc))
        }
    }

    pub fn is_data_ready(&mut self) -> Result<bool, Sen5xError> {
        let mut ready = false;
        let rc = unsafe { ffi::sen5x_read_data_ready(&mut ready) };
        
        if rc == 0 {
            Ok(ready)
        } else {
            Err(Sen5xError::DriverError(rc))
        }
    }

    /// Puts the sensor into continuous sampling mode
    pub fn start_measurement(&mut self) -> Result<(), Sen5xError> {
        let rc = unsafe { ffi::sen5x_start_measurement() };
        if rc == 0 {
            Ok(())
        } else {
            Err(Sen5xError::DriverError(rc))
        }
    }

    /// Stops the internal fan and laser diode sampling loop to conserve system power
    pub fn stop_measurement(&mut self) -> Result<(), Sen5xError> {
        let rc = unsafe { ffi::sen5x_stop_measurement() };
        if rc == 0 {
            Ok(())
        } else {
            Err(Sen5xError::DriverError(rc))
        }
    }

    /// Fetches the latest metrics snapshot and automatically converts raw
    /// fixed-point C integer scalings into floating point types.
    ///
    /// The sentinel checks and scale factors come from `sen5x-conversion`,
    /// the same single source of truth the pure Rust driver uses, so the two
    /// implementations cannot drift on this logic.
    pub fn read_measurements(&mut self) -> Result<SensorReadings, Sen5xError> {
        let mut raw_pm1_0 = 0u16;
        let mut raw_pm2_5 = 0u16;
        let mut raw_pm4_0 = 0u16;
        let mut raw_pm10_0 = 0u16;
        let mut raw_humidity = 0i16;
        let mut raw_temperature = 0i16;
        let mut raw_voc = 0i16;
        let mut raw_nox = 0i16;

        let rc = unsafe {
            ffi::sen5x_read_measured_values(
                &mut raw_pm1_0,
                &mut raw_pm2_5,
                &mut raw_pm4_0,
                &mut raw_pm10_0,
                &mut raw_humidity,
                &mut raw_temperature,
                &mut raw_voc,
                &mut raw_nox,
            )
        };

        if rc != 0 {
            return Err(Sen5xError::DriverError(rc));
        }

        // The 0xFFFF (PM) / 0x7FFF (environmental) "no data" sentinels and the
        // /10, /100, /200 scale factors live in sen5x-conversion — the same
        // crate the pure Rust driver uses — so both drivers share one
        // definition and cannot drift.
        Ok(SensorReadings {
            // PM values arrive scaled by 10 (e.g. 125 = 12.5 µg/m³).
            pm1_0: pm_reading(raw_pm1_0),
            pm2_5: pm_reading(raw_pm2_5),
            pm4_0: pm_reading(raw_pm4_0),
            pm10_0: pm_reading(raw_pm10_0),

            // Environmental data scales by 100 (humidity) and 200 (temperature)
            humidity: humidity_reading(raw_humidity),
            temperature: temperature_reading(raw_temperature),

            // Indices scale by 10
            voc_index: index_reading(raw_voc),
            nox_index: index_reading(raw_nox),
        })
    }
}
