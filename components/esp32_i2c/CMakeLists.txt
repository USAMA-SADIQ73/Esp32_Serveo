idf_component_register(SRCS "esp32_i2c.c"
                        INCLUDE_DIRS "."
                        PRIV_INCLUDE_DIRS ""
                        PRIV_REQUIRES "driver"
                        REQUIRES "Interrupt_Gpio")
