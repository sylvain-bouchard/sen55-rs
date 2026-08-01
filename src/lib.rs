#![no_std]

// Ensure we link the static C library when building our core crate
#[link(name = "sensirion_c", kind = "static")]
extern "C" {
    // Exact signatures from the compiled Sensirion headers
    fn sen5x_device_reset() -> i16;
    fn sen5x_start_measurement() -> i16;
    fn sen5x_stop_measurement() -> i16;
    fn sen5x_read_measured_values(
        mass_concentration_pm1p0: &mut u16,
        mass_concentration_pm2p5: &mut u16,
        mass_concentration_pm4p0: &mut u16,
        mass_concentration_pm10p0: &mut u16,
        ambient_humidity: &mut i16,
        ambient_temperature: &mut i16,
        voc_index: &mut i16,
        nox_index: &mut i16,
    ) -> i16;
}

/// Custom error type wrapping the raw C status return values
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Sen5xError {
    DriverError(i16),
}

/// Labeled sensor measurement data packet parsed out into clean types
#[derive(Debug, Clone, PartialEq)]
pub struct SensorReadings {
    pub pm1_0: f32,               // µg/m³
    pub pm2_5: f32,               // µg/m³
    pub pm4_0: f32,               // µg/m³
    pub pm10_0: f32,              // µg/m³
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

    /// Triggers a hard command reset on the remote SEN5x module over the I2C bus
    pub fn device_reset(&mut self) -> Result<(), Sen5xError> {
        let rc = unsafe { sen5x_device_reset() };
        if rc == 0 {
            Ok(())
        } else {
            Err(Sen5xError::DriverError(rc))
        }
    }

    /// Puts the sensor into continuous sampling mode
    pub fn start_measurement(&mut self) -> Result<(), Sen5xError> {
        let rc = unsafe { sen5x_start_measurement() };
        if rc == 0 {
            Ok(())
        } else {
            Err(Sen5xError::DriverError(rc))
        }
    }

    /// Stops the internal fan and laser diode sampling loop to conserve system power
    pub fn stop_measurement(&mut self) -> Result<(), Sen5xError> {
        let rc = unsafe { sen5x_stop_measurement() };
        if rc == 0 {
            Ok(())
        } else {
            Err(Sen5xError::DriverError(rc))
        }
    }

    /// Fetches the latest metrics snapshot and automatically converts raw
    /// fixed-point C integer scalings into floating point types.
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
            sen5x_read_measured_values(
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

        // Sensirion internal C driver defaults unmeasured/unavailable entries
        // to `0x7FFF` (transposed to 32767) for invalid fields.
        let is_invalid = |val: i16| val == 32767;

        Ok(SensorReadings {
            // PM values arrive scaled by 10 (e.g. 125 = 12.5 µg/m³)
            pm1_0: (raw_pm1_0 as f32) / 10.0,
            pm2_5: (raw_pm2_5 as f32) / 10.0,
            pm4_0: (raw_pm4_0 as f32) / 10.0,
            pm10_0: (raw_pm10_0 as f32) / 10.0,

            // Environmental data scales by 100 (humidity) and 200 (temperature)
            humidity: if is_invalid(raw_humidity) {
                None
            } else {
                Some((raw_humidity as f32) / 100.0)
            },
            temperature: if is_invalid(raw_temperature) {
                None
            } else {
                Some((raw_temperature as f32) / 200.0)
            },

            // Indices scale by 10
            voc_index: if is_invalid(raw_voc) {
                None
            } else {
                Some((raw_voc as f32) / 10.0)
            },
            nox_index: if is_invalid(raw_nox) {
                None
            } else {
                Some((raw_nox as f32) / 10.0)
            },
        })
    }
}
