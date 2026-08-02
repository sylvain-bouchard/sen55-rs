#include "sensirion_i2c_hal.h"

// Stubs out the physical microsecond hardware timers so desktop tests run instantly.
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    // Left empty intentionally. Desktop test loops run immediately without delays.
}

// Stubs out the baseline board initialization routines
void sensirion_i2c_hal_init(void) {
    // No physical hardware registers to map out on desktop systems.
}

// Stubs out the low-level I2C hardware write routine
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count) {
    // Return 0 to pretend the write transaction succeeded perfectly
    return 0;
}

// Stubs out the low-level I2C hardware read routine
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count) {
    // Return 0 to pretend the read transaction succeeded perfectly
    return 0;
}