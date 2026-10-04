# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32_ptp_offload)

target_sources(app PRIVATE
  src/main.c
  src/driver.c
)
target_include_directories(app PRIVATE "${ZEPHYR_BASE}/drivers/ethernet")
if(CONFIG_TEST_STM32_TX_ASYNC)
  target_compile_definitions(app PRIVATE CONFIG_ETH_STM32_HAL_TX_ASYNC=1)
endif()
