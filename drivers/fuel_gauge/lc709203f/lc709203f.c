/*
 * Copyright (c) 2025 Philipp Steiner <philipp.steiner1987@gmailcom>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr driver for LC709203F Battery Monitor
 *
 * This driver implements the sensor API for the LC709203F battery monitor,
 * providing battery voltage, state-of-charge (SOC), and temperature measurements.
 *
 * Note:
 * - The LC709203F is assumed to be connected via I2C.
 * - The register addresses and conversion factors used here are based on
 *   common LC709203F implementations. Consult your datasheet and adjust as needed.
 * - To use this driver, create a matching device tree node (with a “compatible”
 *   string, I2C bus, and register address) so that the DT_INST_* macros can pick it up.
 */

#define DT_DRV_COMPAT maxim_max17048

#include "lc709203f.h"

#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lc709203f, CONFIG_SENSOR_LOG_LEVEL);

/* Default I2C address if not overridden by device tree */
#define LC709203F_DEFAULT_I2C_ADDR 0x0B // TODO: Remove if not needed anymore
