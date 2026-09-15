# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)

zephyr_library_sources_ifdef(CONFIG_CB_STEPPER_TMC_SPI adi_tmc_spi.c)
zephyr_library_sources_ifdef(CONFIG_CB_STEPPER_TMC_UART adi_tmc_uart.c)
