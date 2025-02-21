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

#define LC709203F_CRC_POLYNOMIAL 0x07 /// Polynomial to calculate CRC-8-ATM

/*
 * The LC709203F outputs a raw voltage value that must be multiplied by ~1.1 to
 * obtain a millivolt reading. Similarly, temperature and SOC conversion
 * factors may be required.
 *
 * In this example:
 * - Voltage conversion: voltage_mv = (raw_value * 11 + 5) / 10.
 * - SOC is assumed to be reported as an integer percentage.
 * - Temperature is assumed to be in deci-degrees Celsius (0.1°C per LSB).
 */

struct lc709203f_data {
	/* Last sensor readings (raw values as returned from the chip) */
	uint16_t voltage;     /* battery voltage */
	uint16_t rsoc;        /* relative state-of-charge (percentage ) */
	int16_t temp_celsius; /* cell temperature (assumed in deci-deg Celsius) */
	bool charging;        /* true if battery charging, false if discharging */
};

static uint8_t lc709203f_calc_crc(uint8_t *data, size_t data_len)
{
	uint8_t crc = 0;

	for (size_t j = data_len; j; --j) {
		crc ^= *data++;

		for (size_t i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ LC709203F_CRC_POLYNOMIAL : (crc << 1);
		}
	}

	return crc;
}

/*
 * Read a 16-bit register value (with CRC check).
 *
 * The LC709203F expects the following transaction:
 *   Write: [reg]
 *   Read:  [LSB, MSB, CRC]
 *
 * The CRC is computed over:
 *   [I2C_addr (write), reg, I2C_addr (read), LSB, MSB]
 */
static int lc709203f_read16(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct lc709203f_config *config = dev->config;
	uint8_t buf[3];
	int ret;

	ret = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), buf, sizeof(buf));
	if (ret) {
		LOG_ERR("i2c_write_read failed (reg 0x%02x): %d", reg, ret);
		return ret;
	}

	/* Build buffer for CRC calculation */
	uint8_t crc_buf[5];
	crc_buf[0] = config->i2c.addr << 1;
	crc_buf[1] = reg;
	crc_buf[2] = (config->i2c.addr << 1) | 0x01;
	crc_buf[3] = buf[0]; /* LSB */
	crc_buf[4] = buf[1]; /* MSB */

	if (lc709203f_calc_crc(crc_buf, sizeof(crc_buf)) != buf[2]) {
		LOG_ERR("CRC mismatch on reg 0x%02x", reg);
		return -EIO;
	}

	if (value) {
		*value = buf[0] | (buf[1] << 8);
	}

	return 0;
}

/*
 * Write a 16-bit word to a register (with CRC appended).
 *
 * The transaction is:
 *   Write: [reg, LSB, MSB, CRC]
 *
 * The CRC is computed over:
 *   [I2C_addr (write), reg, LSB, MSB]
 */
static int lc709203f_write16(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct lc709203f_config *config = dev->config;
	uint8_t temp[4];
	uint8_t buf[4];

	temp[0] = config->i2c.addr << 1;
	temp[1] = reg;
	temp[2] = value & 0xFF;
	temp[3] = value >> 8;

	buf[0] = reg;
	buf[1] = temp[2];
	buf[2] = temp[3];
	buf[3] = lc709203f_calc_crc(temp, sizeof(temp));

	return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

/*
 * Sensor API: sample_fetch
 *
 * Fetch the latest measurements from the LC709203F. All three channels
 * (voltage, state-of-charge, and temperature) are updated.
 */
static int lc709203f_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct lc709203f_data *data = dev->data;
	uint16_t raw;
	int ret;

	/* Fetch battery voltage (in mV) */
	ret = lc709203f_read16(dev, LC709203F_REG_CELL_VOLT, &raw);
	if (ret < 0) {
		return ret;
	}
	data->voltage = raw;

	/* Fetch battery state-of-charge (RSOC, in %) */
	ret = lc709203f_read16(dev, LC709203F_REG_RSOC, &raw);
	if (ret < 0) {
		return ret;
	}
	data->rsoc = raw;

	/* Fetch temperature (in 0.1 K) and convert to °C */
	ret = lc709203f_read16(dev, LC709203F_REG_CELL_TEMP, &raw);
	if (ret < 0) {
		return ret;
	}
	/* Convert deciKelvin to Celsius */
	data->temp_celsius = (raw / 10.0f) - 273.15f;

	return 0;
}
