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
 * - The LC chip works best when queried every few seconds at the fastest. Don't disconnect the LiPo
 *   battery, it is used to power the LC chip!
 */

#define DT_DRV_COMPAT onsemi_lc709203f

#include "lc709203f.h"

#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lc709203f);

#define LC709203F_INIT_RSOC_VAL  0xAA55 // RSOC initialization value
#define LC709203F_CRC_POLYNOMIAL 0x07   // Polynomial to calculate CRC-8-ATM

static int lc709203f_read_word(const struct device *dev, uint8_t reg, uint16_t *value);
static int lc709203f_write_word(const struct device *dev, uint8_t reg, uint16_t value);
static uint8_t lc709203f_calc_crc(uint8_t *data, size_t data_len);

int lc709203f_before_rsoc(const struct device *dev);
int lc709203f_initial_rsoc(const struct device *dev);

int lc709203f_get_alarm_low_rsoc(const struct device *dev, uint8_t *rsoc);
int lc709203f_get_alarm_low_voltage(const struct device *dev, uint16_t *voltage);
int lc709203f_get_apa(const struct device *dev, lc709203f_battery_apa_t *apa);
int lc709203f_get_apt(const struct device *dev, uint16_t *apt);
int lc709203f_get_battery_profile(const struct device *dev, lc709203f_battery_profile_t *profile);
int lc709203f_get_battery_profile_code(const struct device *dev, uint16_t *code);
int lc709203f_get_cell_ite(const struct device *dev, uint16_t *ite);
int lc709203f_get_cell_temperature(const struct device *dev, uint16_t *temperature);
int lc709203f_get_cell_voltage(const struct device *dev, uint16_t *voltage);
int lc709203f_get_direction(const struct device *dev, lc709203f_direction_t *direction);
int lc709203f_get_ic_version(const struct device *dev, uint16_t *ic_version);
int lc709203f_get_power_mode(const struct device *dev, lc709203f_power_mode_t *mode);
int lc709203f_get_rsoc(const struct device *dev, uint16_t *rsoc);
int lc709203f_get_temp_mode(const struct device *dev, lc709203f_temp_mode_t *mode);
int lc709203f_get_thermistor_b(const struct device *dev, uint16_t *value);

int lc709203f_set_alarm_low_rsoc(const struct device *dev, uint8_t rsoc);
int lc709203f_set_alarm_low_voltage(const struct device *dev, uint16_t voltage);
int lc709203f_set_apa(const struct device *dev, lc709203f_battery_apa_t apa);
int lc709203f_set_apt(const struct device *dev, uint16_t apt);
int lc709203f_set_battery_profile(const struct device *dev, lc709203f_battery_profile_t profile);
int lc709203f_set_current_direction(const struct device *dev, lc709203f_direction_t direction);
int lc709203f_set_power_mode(const struct device *dev, lc709203f_power_mode_t mode);
int lc709203f_set_temp_mode(const struct device *dev, lc709203f_temp_mode_t mode);
int lc709203f_set_thermistor_b(const struct device *dev, uint16_t value);

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
static int lc709203f_read_word(const struct device *dev, uint8_t reg, uint16_t *value)
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
 * Calculate the CRC-8 checksum using polynomial 0x07 (CRC-8-ATM).
 */
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

int lc709203f_get_apa(const struct device *dev, lc709203f_battery_apa_t *apa)
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

	*apa = (lc709203f_battery_apa_t)tmp;
	return 0;
}

int lc709203f_get_apt(const struct device *dev, uint16_t *apt)
{
	if (!dev || !apt) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_APT, apt);
}

int lc709203f_get_battery_profile(const struct device *dev, lc709203f_battery_profile_t *profile)
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

int lc709203f_get_cell_temperature(const struct device *dev, uint16_t *temperature)
{
	if (!dev || !temperature) {
		return -EINVAL;
	}
	return lc709203f_read_word(dev, LC709203F_REG_CELL_TEMPERATURE, temperature);
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

int lc709203f_set_apa(const struct device *dev, lc709203f_battery_apa_t apa)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_APA, (uint16_t)apa);
}

int lc709203f_set_apt(const struct device *dev, uint16_t apt)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_APT, apt);
}

int lc709203f_set_battery_profile(const struct device *dev, lc709203f_battery_profile_t profile)
{
	if (!dev) {
		return -EINVAL;
	}
	return lc709203f_write_word(dev, LC709203F_REG_BAT_PROFILE, (uint16_t)profile);
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
	return lc709203f_write_word(dev, LC709203F_REG_IC_POWER_MODE, (uint16_t)mode);
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

lc709203f_battery_apa_t lc709203f_string_to_apa(const char *apa_string)
{
	static const char *const apas[] = {"100mAh",  "200mAh",  "500mAh",
					   "1000mAh", "2000mAh", "3000mAh"};

	static const lc709203f_battery_apa_t apa_values[] = {
		LC709203F_APA_100MAH,  LC709203F_APA_200MAH,  LC709203F_APA_500MAH,
		LC709203F_APA_1000MAH, LC709203F_APA_2000MAH, LC709203F_APA_3000MAH};

	// Check if the string is NULL or empty
	for (size_t i = 0; i < ARRAY_SIZE(apas); i++) {
		if (strncmp(apa_string, apas[i], strlen(apas[i])) == 0) {
			return apa_values[i];
		}
	}
	LOG_ERR("Invalid apa_string: %s, returning default: %d", apa_string, LC709203F_APA_100MAH);
	return LC709203F_APA_100MAH;
}

/*
 * Device initialization function.
 */
int lc709203f_init(const struct device *dev)
{
	const struct lc709203f_config *config = dev->config;
	int ret = 0;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	lc709203f_power_mode_t mode;
	LOG_INF("Get power mode");
	ret = lc709203f_get_power_mode(dev, &mode);
	if (ret) {
		LOG_ERR("Failed to get power mode: %d", ret);
	}

	LOG_INF("Power mode: %d", mode);
	if (mode == LC709203F_POWER_MODE_SLEEP) {
		LOG_INF("Set Power mode");
		ret = lc709203f_set_power_mode(dev, LC709203F_POWER_MODE_OPERATIONAL);
		if (ret) {
			LOG_ERR("Failed to set power mode: %d", ret);
		}
	}

	LOG_INF("Set battery pack: %s", config->battery_apa);
	ret = lc709203f_set_apa(dev, lc709203f_string_to_apa(config->battery_apa));
	if (ret) {
		LOG_ERR("Failed to set battery pack: %d", ret);
	}

	LOG_INF("Set battery profile: %d", config->battery_profile);
	ret = lc709203f_set_battery_profile(dev, config->battery_profile);
	if (ret) {
		LOG_ERR("Failed to set battery profile: %d", ret);
	}

	if (config->thermistor) {
		LOG_INF("Set temperature mode: %d", config->thermistor_mode);
		lc709203f_set_temp_mode(dev, config->thermistor_mode);
		if (ret) {
			LOG_ERR("Failed to set temperature mode: %d", ret);
		}

		LOG_INF("Set thermistor B value: %d", config->thermistor_b_value);
		ret = lc709203f_set_thermistor_b(dev, config->thermistor_b_value);
		if (ret) {
			LOG_ERR("Failed to set thermistor B value: %d", ret);
		}

		LOG_INF("Set thermistor APT: %d", config->thermistor_apt);
		ret = lc709203f_set_apt(dev, config->thermistor_apt);
		if (ret) {
			LOG_ERR("Failed to set thermistor APT: %d", ret);
		}
	}

	if (config->initial_rsoc) {
		LOG_INF("lc709203f_initial_rsoc");
		ret = lc709203f_initial_rsoc(dev);
		if (ret) {
			LOG_ERR("Quickstart failed: %d", ret);
			return ret;
		}
	}

	LOG_INF("initialized");
	return 0;
}

static int lc709203f_get_prop(const struct device *dev, fuel_gauge_prop_t prop,
			      union fuel_gauge_prop_val *val)
{
	int rc = 0;
	uint16_t tmp_val = 0;
	const struct lc709203f_config *config = dev->config;

	switch (prop) {
	case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE:
		rc = lc709203f_get_rsoc(dev, &tmp_val);
		val->relative_state_of_charge = tmp_val;
		break;
	case FUEL_GAUGE_VOLTAGE:
		rc = lc709203f_get_cell_voltage(dev, &tmp_val);
		val->voltage = tmp_val * 1000;
		break;
	case FUEL_GAUGE_TEMPERATURE:
		if (!config->thermistor) {
			LOG_ERR("Thermistor not enabled");
			return -ENOTSUP;
		}
		rc = lc709203f_get_cell_temperature(dev, &tmp_val);
		val->temperature = tmp_val;
		break;
	case FUEL_GAUGE_DESIGN_CAPACITY:
		lc709203f_battery_apa_t apa;
		rc = lc709203f_get_apa(dev, &apa);

		switch (apa) {
		case LC709203F_APA_100MAH:
			val->design_cap = 100;
			break;
		case LC709203F_APA_200MAH:
			val->design_cap = 200;
			break;
		case LC709203F_APA_500MAH:
			val->design_cap = 500;
			break;
		case LC709203F_APA_1000MAH:
			val->design_cap = 1000;
			break;
		case LC709203F_APA_2000MAH:
			val->design_cap = 2000;
			break;
		case LC709203F_APA_3000MAH:
			val->design_cap = 3000;
			break;
		default:
			LOG_ERR("Invalid battery capacity: %d", apa);
			return -EINVAL;
		}
		break;
	}

	return rc;
}

static int lc709203f_set_prop(const struct device *dev, fuel_gauge_prop_t prop,
			      union fuel_gauge_prop_val val)
{
	int rc = 0;

	switch (prop) {
	case FUEL_GAUGE_DESIGN_CAPACITY:
		rc = lc709203f_set_apa(dev, (lc709203f_battery_apa_t)val.design_cap);
		break;
	}

	return rc;
}

static DEVICE_API(fuel_gauge, lc709203f_driver_api) = {
	.get_property = &lc709203f_get_prop,
	.set_property = &lc709203f_set_prop,
};

#define LC709203F_INIT(inst)                                                                       \
                                                                                                   \
	static const struct lc709203f_config lc709203f_config_##inst = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.initial_rsoc = DT_INST_PROP(inst, initial_rsoc),                                  \
		.battery_apa = DT_INST_PROP(inst, apa),                                            \
		.battery_profile = DT_INST_PROP(inst, battery_profile),                            \
		.thermistor = DT_INST_PROP(inst, thermistor),                                      \
		.thermistor_b_value = DT_INST_PROP(inst, thermistor_b_value),                      \
		.thermistor_apt = DT_INST_PROP(inst, apt),                                         \
		.thermistor_mode = DT_INST_PROP(inst, thermistor_mode),                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &lc709203f_init, NULL, NULL, &lc709203f_config_##inst,         \
			      POST_KERNEL, CONFIG_FUEL_GAUGE_INIT_PRIORITY,                        \
			      &lc709203f_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LC709203F_INIT)
