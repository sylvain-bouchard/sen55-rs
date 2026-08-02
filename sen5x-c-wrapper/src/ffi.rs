#[link(name = "sensirion_c", kind = "static")]
unsafe extern "C" {
    pub fn sen5x_device_reset() -> i16;

    pub fn sen5x_read_data_ready(data_ready: &mut bool) -> i16;

    pub fn sen5x_start_measurement() -> i16;

    pub fn sen5x_stop_measurement() -> i16;

    pub fn sen5x_read_measured_values(
        mass_concentration_pm1p0: &mut u16,
        mass_concentration_pm2p5: &mut u16,
        mass_concentration_pm4p0: &mut u16,
        mass_concentration_pm10p0: &mut u16,
        ambient_humidity: &mut i16,
        ambient_temperature: &mut i16,
        voc_index: &mut i16,
        nox_index: &mut i16,
    ) -> i16;

    #[cfg(feature = "mock")]
    pub fn sensirion_i2c_hal_mock_set_read_buffer(data: *const u8, length: u16);
}
