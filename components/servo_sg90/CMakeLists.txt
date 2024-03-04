idf_component_register(SRCS "servo_sg90.c"
                        INCLUDE_DIRS "."
                        PRIV_INCLUDE_DIRS ""
                        PRIV_REQUIRES "driver"
                        REQUIRES "esp32_i2c")
