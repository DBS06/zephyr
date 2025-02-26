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
#define LC709203F_DEFAULT_I2C_ADDR 0x0B   // TODO: Remove if not needed anymore
#define LC709203F_INIT_RSOC_VAL    0xAA55 // RSOC initialization value

#define LC709203F_CRC_POLYNOMIAL 0x07 // Polynomial to calculate CRC-8-ATM

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

int lc709203f_before_rsoc(const struct device *dev);
int lc709203f_initial_rsoc(const struct device *dev);
int lc709203f_get_alarm_low_rsoc(const struct device *dev, uint8_t *rsoc);
int lc709203f_get_alarm_low_voltage(const struct device *dev, uint16_t *voltage);
int lc709203f_get_apa(const struct device *dev, uint8_t *apa);
int lc709203f_get_apt(const struct device *dev, uint16_t *apt);
int lc709203f_get_battery_profile(const struct device *dev, lc709203f_battery_profile_t *profile);
int lc709203f_get_battery_profile_code(const struct device *dev, uint16_t *code);
int lc709203f_get_cell_ite(const struct device *dev, uint16_t *ite);
int lc709203f_get_cell_temperature(const struct device *dev, float *temperature);
int lc709203f_get_cell_temperature_celsius(const struct device *dev, float *temperature);
int lc709203f_get_cell_voltage(const struct device *dev, uint16_t *voltage);
int lc709203f_get_direction(const struct device *dev, lc709203f_direction_t *direction);
int lc709203f_get_ic_version(const struct device *dev, uint16_t *ic_version);
int lc709203f_get_power_mode(const struct device *dev, lc709203f_power_mode_t *mode);
int lc709203f_get_rsoc(const struct device *dev, uint16_t *rsoc);
int lc709203f_get_temp_mode(const struct device *dev, lc709203f_temp_mode_t *mode);
int lc709203f_get_thermistor_b(const struct device *dev, uint16_t *value);
int lc709203f_set_alarm_low_rsoc(const struct device *dev, uint8_t rsoc);
int lc709203f_set_alarm_low_voltage(const struct device *dev, uint16_t voltage);
int lc709203f_set_apa(const struct device *dev, uint8_t apa);
int lc709203f_set_apt(const struct device *dev, uint16_t apt);
int lc709203f_set_battery_profile(const struct device *dev, lc709203f_battery_profile_t profile);
int lc709203f_set_cell_temperature(const struct device *dev, float temperature);
int lc709203f_set_cell_temperature_celsius(const struct device *dev, float temperature);
int lc709203f_set_current_direction(const struct device *dev, lc709203f_direction_t direction);
int lc709203f_set_power_mode(const struct device *dev, lc709203f_power_mode_t mode);
int lc709203f_set_temp_mode(const struct device *dev, lc709203f_temp_mode_t mode);
int lc709203f_set_thermistor_b(const struct device *dev, uint16_t value);

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
static int lc709203f_read_word(const struct device *dev, uint8_t reg, uint16_t *val)
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
static int lc709203f_write_word(const struct device *dev, uint8_t reg, uint16_t value)
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
	ret = lc709203f_read_word(dev, LC709203F_REG_CELL_VOLT, &raw);
	if (ret < 0) {
		return ret;
	}
	data->voltage = raw;

	/* Fetch battery state-of-charge (RSOC, in %) */
	ret = lc709203f_read_word(dev, LC709203F_REG_RSOC, &raw);
	if (ret < 0) {
		return ret;
	}
	data->rsoc = raw;

	/* Fetch temperature (in 0.1 K) and convert to °C */
	ret = lc709203f_read_word(dev, LC709203F_REG_CELL_TEMP, &raw);
	if (ret < 0) {
		return ret;
	}
	/* Convert deciKelvin to Celsius */
	data->temp_celsius = (raw / 10.0f) - 273.15f;

	return 0;
}

/*
 * Optional: A “quickstart” routine to reinitialize RSOC measurement.
 * (For example, writing 0xAA55 to LC709203F_REG_INITIAL_RSOC.)
 */
static int lc709203f_quickstart(const struct device *dev)
{
	return lc709203f_write16(dev, LC709203F_REG_INITIAL_RSOC, LC709203F_INIT_RSOC_VAL);
}

/*
 * Device initialization function.
 */
static int lc709203f_init(const struct device *dev)
{
	const struct lc709203f_config *config = dev->config;
	int ret = 0;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = lc709203f_set_power_mode(dev, LC709203F_POWER_OPERATE);
	if (ret) {
		LOG_ERR("Failed to set power mode: %d", ret);
	}

	ret = lc709203f_set_apt(dev, LC709203F_APA_500MAH);
	if (ret) {
		LOG_ERR("Failed to set battery pack: %d", ret);
	}

	// use 4.2V profile
	ret = lc709203f_set_battery_profile(dev, 0x1);
	if (ret) {
		LOG_ERR("Failed to set battery profile: %d", ret);
	}

	lc709203f_set_temp_mode(dev, LC709203F_TEMPERATURE_THERMISTOR);
	if (ret) {
		LOG_ERR("Failed to set temperature mode: %d", ret);
	}

	// Optionally, perform a quickstart to initialize RSOC measurement:
	ret = lc709203f_quickstart(dev);
	if (ret) {
		LOG_ERR("Quickstart failed: %d", ret);
		return ret;
	}

	return 0;
}

int lc709203f_before_rsoc(const struct device *dev)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_BEFORE_RSOC, LC709203F_INIT_RSOC_VAL);
}

int lc709203f_initial_rsoc(const struct device *dev)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_INITIAL_RSOC, LC709203F_INIT_RSOC_VAL);
}

int lc709203f_get_alarm_low_rsoc(const struct device *dev, uint8_t *rsoc)
{
	uint16_t tmp;
	int ret;

	if (!dev || !rsoc) {
		return -EINVAL;
	}

	ret = lc709203f_read_word(dev, LC709203F_REG_ALARM_LOW_RSOC, &tmp);
	if (ret) {
		return ret;
	}

	*rsoc = (uint8_t)tmp;
	return 0;
}

int lc709203f_get_alarm_low_voltage(const struct device *dev, uint16_t *voltage)
{
	if (!dev || !voltage) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_ALARM_LOW_VOLTAGE, voltage);
}

int lc709203f_get_apa(const struct device *dev, uint8_t *apa)
{
	uint16_t tmp;
	int ret;

	if (!dev || !apa) {
		return -EINVAL;
	}

	ret = lc709203f_read_word(dev, LC709203F_REG_APA, &tmp);
	if (ret) {
		return ret;
	}

	*apa = (uint8_t)tmp;
	return 0;
}

int lc709203f_get_apt(const struct device *dev, lc709203f_battery_profile_t *apt)
{
	if (!dev || !apt) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_APT, apt);
}

int lc709203f_get_battery_profile(const struct device *dev, uint16_t *profile)
{
	uint16_t tmp;
	int ret;

	if (!dev || !profile) {
		return -EINVAL;
	}

	ret = lc709203f_read_word(dev, LC709203F_REG_BAT_PROFILE, &tmp);
	if (ret) {
		return ret;
	}

	*profile = (lc709203f_battery_profile_t)tmp;
	return 0;
}

int lc709203f_get_battery_profile_code(const struct device *dev, uint16_t *code)
{
	if (!dev || !code) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_NUM_PARAMETER, code);
}

int lc709203f_get_cell_ite(const struct device *dev, uint16_t *ite)
{
	if (!dev || !ite) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_CELL_ITE, ite);
}

int lc709203f_get_cell_temperature(const struct device *dev, float *temperature)
{
	uint16_t temp_val;
	int ret;

	if (!dev || !temperature) {
		return -EINVAL;
	}

	ret = lc709203f_read_word(dev, LC709203F_REG_CELL_TEMPERATURE, &temp_val);
	if (ret) {
		return ret;
	}

	/* The device returns temperature in deciKelvin */
	*temperature = temp_val / 10.0f;
	return 0;
}

int lc709203f_get_cell_temperature_celsius(const struct device *dev, float *temperature)
{
	int ret;
	float temp_kelvin;

	ret = lc709203f_get_cell_temperature(dev, &temp_kelvin);
	if (ret) {
		return ret;
	}
	*temperature = temp_kelvin - 273.15f;
	return 0;
}

int lc709203f_get_cell_voltage(const struct device *dev, uint16_t *voltage)
{
	if (!dev || !voltage) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_CELL_VOLTAGE, voltage);
}

int lc709203f_get_direction(const struct device *dev, lc709203f_direction_t *direction)
{
	uint16_t tmp;
	int ret;

	if (!dev || !direction) {
		return -EINVAL;
	}

	/* Note: The original ESP-IDF driver appears to have a copy-paste error.
	 * Here we use the proper register for current direction.
	 */
	ret = lc709203f_read_word(dev, LC709203F_REG_CURRENT_DIRECTION, &tmp);
	if (ret) {
		return ret;
	}

	*direction = (lc709203f_direction_t)tmp;
	return 0;
}

int lc709203f_get_ic_version(const struct device *dev, uint16_t *ic_version)
{
	if (!dev || !ic_version) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_IC_VERSION, ic_version);
}

int lc709203f_get_power_mode(const struct device *dev, lc709203f_power_mode_t *mode)
{
	uint16_t tmp;
	int ret;

	if (!dev || !mode) {
		return -EINVAL;
	}

	ret = lc709203f_read_word(dev, LC709203F_REG_IC_POWER_MODE, &tmp);
	if (ret) {
		return ret;
	}

	*mode = (lc709203f_power_mode_t)tmp;
	return 0;
}

int lc709203f_get_rsoc(const struct device *dev, uint16_t *rsoc)
{
	if (!dev || !rsoc) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_RSOC, rsoc);
}

int lc709203f_get_temp_mode(const struct device *dev, lc709203f_temp_mode_t *mode)
{
	uint16_t tmp;
	int ret;

	if (!dev || !mode) {
		return -EINVAL;
	}

	ret = lc709203f_read_word(dev, LC709203F_REG_STATUS_BIT, &tmp);
	if (ret) {
		return ret;
	}

	*mode = (lc709203f_temp_mode_t)tmp;
	return 0;
}

int lc709203f_get_thermistor_b(const struct device *dev, uint16_t *value)
{
	if (!dev || !value) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_THERMISTOR_B, value);
}

int lc709203f_set_alarm_low_rsoc(const struct device *dev, uint8_t rsoc)
{
	if (!dev) {
		return -EINVAL;
	}
	if (rsoc > 100) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_ALARM_LOW_RSOC, rsoc);
}

int lc709203f_set_alarm_low_voltage(const struct device *dev, uint16_t voltage)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_ALARM_LOW_VOLTAGE, voltage);
}

int lc709203f_set_apa(const struct device *dev, uint8_t apa)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_APA, apa);
}

int lc709203f_set_apt(const struct device *dev, lc709203f_battery_profile_t apt)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_APT, (uint16_t)apt);
}

int lc709203f_set_battery_profile(const struct device *dev, uint16_t profile)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_BAT_PROFILE, profile);
}

int lc709203f_set_cell_temperature(const struct device *dev, float temperature)
{
	uint16_t temp;
	/*
	 * The device expects temperature in deciKelvin.
	 * For example, 25°C corresponds roughly to 298.15K -> 2982 (0.1K units).
	 */
	temp = (uint16_t)(temperature * 10.0f);

	if (!dev) {
		return -EINVAL;
	}
	/* Check that the temperature is within a valid range.
	 * (These limits come from the original driver; adjust as needed.)
	 */
	if (temp < 0x09E4 || temp > 0x0D04) {
		return -EINVAL;
	}

	return lc709203f_write_word(dev, LC709203F_REG_CELL_TEMPERATURE, temp);
}

int lc709203f_set_cell_temperature_celsius(const struct device *dev, float temperature)
{
	/* Convert Celsius to Kelvin */
	return lc709203f_set_cell_temperature(dev, temperature + 273.15f);
}

int lc709203f_set_current_direction(const struct device *dev, lc709203f_direction_t direction)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_CURRENT_DIRECTION, (uint16_t)direction);
}

int lc709203f_set_power_mode(const struct device *dev, lc709203f_power_mode_t mode)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_POWER_MODE, (uint16_t)mode);
}

int lc709203f_set_temp_mode(const struct device *dev, lc709203f_temp_mode_t mode)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_STATUS_BIT, (uint16_t)mode);
}

int lc709203f_set_thermistor_b(const struct device *dev, uint16_t value)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_THERMISTOR_B, value);
}

static DEVICE_API(fuel_gauge, bq27z746_driver_api) = {
	.get_property = &bq27z746_get_prop,
	.set_property = &bq27z746_set_prop,
	.get_buffer_property = &bq27z746_get_buffer_prop,
};

#define BQ27Z746_INIT(index)                                                                       \
                                                                                                   \
	static const struct bq27z746_config bq27z746_config_##index = {                            \
		.i2c = I2C_DT_SPEC_INST_GET(index),                                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &bq27z746_init, NULL, NULL, &bq27z746_config_##index,         \
			      POST_KERNEL, CONFIG_FUEL_GAUGE_INIT_PRIORITY, &bq27z746_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ27Z746_INIT)
