# STM32F407xx_spi_driver

MCU Specific header file(stm32F407xx.h), GPIO driver header file(stm32f407xx_gpio_driver.h) and SPI driver header file(STM32F407xx_spi_driver) are prepared for only this project.

If you want to use this driver with other peripherals you must update MCU Specific header file and prepare a driver for your peripherals.

SPI driver is not support multimaster configuration.

For test the SPI driver STM32F4DISCOVERY Board and ARDUNIO UNO R3 boards are used.

# CONNECTIONS

                                        STM32F4DISCOVERY                  ARDUNIO UNO

                                              PB12  ----------------------->  10
                                              PB13  ----------------------->  13
                                              PB14  ----------------------->  12
                                              PB15  ----------------------->  15

                                              PA3   ----------------------->  3

                                              GND   ----------------------->  GND
