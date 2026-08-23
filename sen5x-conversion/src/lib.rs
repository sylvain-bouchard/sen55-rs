#![no_std]

//! Shared raw-word → physical-unit conversion for the SEN5x drivers.
//!
//! The pure Rust driver (`sen5x-rs`) and the C reference facade
//! (`sen5x-c-wrapper`) must agree on which raw words mean "no data" and how
//! fixed-point words scale to physical units. This crate is the single
//! source of truth for both, so the two drivers cannot drift: changing a
//! sentinel or scale factor here changes both implementations at once.
//!
//! The drivers keep their own wire-level decoding (byte offsets, endianness,
//! CRC) independent — only the interpretation of already-decoded raw words is
//! shared.

/// Raw PM channel word meaning "no data" (measurement not running, or before
/// the first sample).
pub const PM_SENTINEL: u16 = 0xFFFF;

/// Raw environmental channel word meaning "no data". The sensor reports
/// `0x7FFF` (32767 as `i16`) for unavailable humidity/temperature/VOC/NOx
/// fields.
pub const ENV_SENTINEL: i16 = 0x7FFF;

/// PM mass concentrations: raw word / 10 → µg/m³.
pub const PM_MASS_SCALE: f32 = 10.0;

/// Relative humidity: raw word / 100 → %RH.
pub const HUMIDITY_SCALE: f32 = 100.0;

/// Temperature: raw word / 200 → °C.
pub const TEMPERATURE_SCALE: f32 = 200.0;

/// VOC/NOx index: raw word / 10 → index.
pub const INDEX_SCALE: f32 = 10.0;

/// Whether a PM channel word carries valid data (not the `0xFFFF` sentinel).
#[inline]
pub const fn is_pm_available(raw: u16) -> bool {
    raw != PM_SENTINEL
}

/// Whether an environmental word carries valid data (not the `0x7FFF`
/// sentinel).
#[inline]
pub const fn is_env_available(raw: i16) -> bool {
    raw != ENV_SENTINEL
}

/// Converts a PM mass word to µg/m³, or `None` when the channel reports the
/// `0xFFFF` sentinel.
#[inline]
pub fn pm_reading(raw: u16) -> Option<f32> {
    is_pm_available(raw).then(|| raw as f32 / PM_MASS_SCALE)
}

/// Converts a humidity word to %RH, or `None` when the channel reports the
/// `0x7FFF` sentinel.
#[inline]
pub fn humidity_reading(raw: i16) -> Option<f32> {
    is_env_available(raw).then(|| raw as f32 / HUMIDITY_SCALE)
}

/// Converts a temperature word to °C, or `None` when the channel reports the
/// `0x7FFF` sentinel.
#[inline]
pub fn temperature_reading(raw: i16) -> Option<f32> {
    is_env_available(raw).then(|| raw as f32 / TEMPERATURE_SCALE)
}

/// Converts a VOC/NOx index word to the 1–500 index, or `None` when the
/// channel reports the `0x7FFF` sentinel.
#[inline]
pub fn index_reading(raw: i16) -> Option<f32> {
    is_env_available(raw).then(|| raw as f32 / INDEX_SCALE)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pm_sentinel_yields_none() {
        assert_eq!(pm_reading(PM_SENTINEL), None);
    }

    #[test]
    fn env_sentinel_yields_none() {
        assert_eq!(humidity_reading(ENV_SENTINEL), None);
        assert_eq!(temperature_reading(ENV_SENTINEL), None);
        assert_eq!(index_reading(ENV_SENTINEL), None);
    }

    #[test]
    fn pm_values_scale_by_ten() {
        assert_eq!(pm_reading(125), Some(12.5));
        assert_eq!(pm_reading(0), Some(0.0));
    }

    #[test]
    fn env_values_scale_by_documented_factors() {
        assert_eq!(humidity_reading(5625), Some(56.25));
        assert_eq!(temperature_reading(4000), Some(20.0));
        assert_eq!(index_reading(1234), Some(123.4));
    }

    #[test]
    fn negative_temperature_scales_correctly() {
        assert_eq!(temperature_reading(-100), Some(-0.5));
    }

    // The sentinels are only sentinels on their own channel types — the
    // overlapping bit patterns below are where classification bugs live.

    #[test]
    fn pm_sentinel_value_is_valid_environmental_data() {
        // 0xFFFF as i16 is -1, not the 0x7FFF sentinel.
        assert_eq!(temperature_reading(-1), Some(-0.005));
        assert_eq!(humidity_reading(-1), Some(-0.01));
    }

    #[test]
    fn env_sentinel_value_is_valid_pm_data() {
        // 0x7FFF = 32767 µg/m³ is a legitimate (very high) PM reading.
        assert_eq!(pm_reading(0x7FFF), Some(3276.7));
    }
}
