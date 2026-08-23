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

    pub fn sen5x_read_measured_raw_values(
        raw_humidity: &mut i16,
        raw_temperature: &mut i16,
        raw_voc: &mut u16,
        raw_nox: &mut u16,
    ) -> i16;

    pub fn sen5x_read_measured_pm_values(
        mass_concentration_pm1p0: &mut u16,
        mass_concentration_pm2p5: &mut u16,
        mass_concentration_pm4p0: &mut u16,
        mass_concentration_pm10p0: &mut u16,
        number_concentration_pm0p5: &mut u16,
        number_concentration_pm1p0: &mut u16,
        number_concentration_pm2p5: &mut u16,
        number_concentration_pm4p0: &mut u16,
        number_concentration_pm10p0: &mut u16,
        typical_particle_size: &mut u16,
    ) -> i16;

    pub fn sen5x_get_version(
        firmware_major: &mut u8,
        firmware_minor: &mut u8,
        firmware_debug: &mut bool,
        hardware_major: &mut u8,
        hardware_minor: &mut u8,
        protocol_major: &mut u8,
        protocol_minor: &mut u8,
    ) -> i16;

    pub fn sen5x_read_and_clear_device_status(device_status: &mut u32) -> i16;

    #[cfg(feature = "mock")]
    pub fn sensirion_i2c_hal_mock_set_read_buffer(data: *const u8, length: u16);
}
