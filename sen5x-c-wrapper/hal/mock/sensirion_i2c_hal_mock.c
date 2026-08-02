#include "sensirion_i2c_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t mock_read_buffer[32];
static uint16_t mock_read_length = 0;
static uint16_t mock_read_offset = 0;

void sensirion_i2c_hal_mock_set_read_buffer(
    const uint8_t* data,
    uint16_t length)
{
    memcpy(mock_read_buffer, data, length);
    mock_read_length = length;
    mock_read_offset = 0;
}

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
    if ((mock_read_offset + count) > mock_read_length) {
        return -1;
    }

    memcpy(
        data,
        &mock_read_buffer[mock_read_offset],
        count
    );

    mock_read_offset += count;

    return 0;
}