idf_component_register(SRCS "sensirion_common.c" "sensirion_i2c_hal.c" "sensirion_i2c.c" "sen5x_i2c.c"
                       INCLUDE_DIRS "."
                       REQUIRES "driver")
