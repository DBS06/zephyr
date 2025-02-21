/*
 * LC709203F fuel gauge driver
 *
 * This driver is written in a style similar to the Zephyr fuel gauge drivers
 * (such as bq27z746 and max17048). It implements the sensor API so that the
 * battery voltage, state‐of‐charge (RSOC), and temperature (in °C) can be
 * fetched via sensor_sample_fetch() and sensor_channel_get().
 *
 * The LC709203F communicates via I2C and uses a CRC‐8 (polynomial 0x07)
 * for data integrity.
 *
 * SPDX-License-Identifier: ISC
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "lc709203f.h"

LOG_MODULE_REGISTER(lc709203f, CONFIG_SENSOR_LOG_LEVEL);

/* LC709203F register definitions */
#define LC709203F_REG_CELL_TEMPERATURE 0x08 /* Read/write battery temperature */
#define LC709203F_REG_CELL_VOLTAGE     0x09 /* Read battery voltage */
#define LC709203F_REG_RSOC             0x0D /* Read state of charge */
#define LC709203F_REG_INITIAL_RSOC     0x07 /* RSOC initialization command */
#define LC709203F_INIT_RSOC_VAL        0xAA55

/* Default I2C address for LC709203F */
#define LC709203F_I2C_ADDR 0x0B

/*
 * The LC709203F returns temperature in deciKelvin (0.1 K units).
 * To convert to Celsius: Celsius = (deciKelvin / 10.0) - 273.15
 */

/* Calculate CRC-8 using polynomial 0x07 (CRC-8-ATM) */
static uint8_t lc709203f_calc_crc(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
		}
	}

	return crc;
}

/* Driver configuration structure */
struct lc709203f_config {
	struct i2c_dt_spec i2c;
};

/* Driver runtime data */
struct lc709203f_data {
	/* Cached sensor readings */
	uint16_t voltage;   /* in mV */
	uint16_t rsoc;      /* in % */
	float temp_celsius; /* in °C */
};

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
 * Sensor API: sample_fetch
 *
 * Fetch the latest measurements from the LC709203F. All three channels
 * (voltage, state-of-charge, and temperature) are updated.
 */
static int lc709203f_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct lc709203f_data *data = dev->data;
	int ret;
	uint16_t raw;

	/* Fetch battery voltage (in mV) */
	ret = lc709203f_read_word(dev, LC709203F_REG_CELL_VOLTAGE, &raw);
	if (ret) {
		return ret;
	}
	data->voltage = raw;

	/* Fetch battery state-of-charge (RSOC, in %) */
	ret = lc709203f_read_word(dev, LC709203F_REG_RSOC, &raw);
	if (ret) {
		return ret;
	}
	data->rsoc = raw;

	/* Fetch temperature (in 0.1 K) and convert to °C */
	ret = lc709203f_read_word(dev, LC709203F_REG_CELL_TEMPERATURE, &raw);
	if (ret) {
		return ret;
	}
	/* Convert deciKelvin to Celsius */
	data->temp_celsius = (raw / 10.0f) - 273.15f;

	return 0;
}

/*
 * Sensor API: channel_get
 *
 * Return the last fetched measurement for the selected channel.
 */
static int lc709203f_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct lc709203f_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		/* Voltage is read in mV; convert to V for sensor_value */
		sensor_value_set_double(val, data->voltage / 1000.0);
		break;
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		/* RSOC in percent */
		sensor_value_set_double(val, data->rsoc);
		break;
	case SENSOR_CHAN_AMBIENT_TEMP:
		sensor_value_set_double(val, data->temp_celsius);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Optional: A “quickstart” routine to reinitialize RSOC measurement.
 * (For example, writing 0xAA55 to LC709203F_REG_INITIAL_RSOC.)
 */
static int lc709203f_quickstart(const struct device *dev)
{
	return lc709203f_write_word(dev, LC709203F_REG_INITIAL_RSOC, LC709203F_INIT_RSOC_VAL);
}

/*
 * Sensor driver API structure
 */
static const struct sensor_driver_api lc709203f_driver_api = {
	.sample_fetch = lc709203f_sample_fetch, .channel_get = lc709203f_channel_get,
	/* If desired, you can add trigger_set or other API functions here. */
};

/*
 * Device initialization function.
 */
static int lc709203f_init(const struct device *dev)
{
	const struct lc709203f_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Optionally, perform a quickstart to initialize RSOC measurement:
	 *
	 * int ret = lc709203f_quickstart(dev);
	 * if (ret) {
	 *     LOG_ERR("Quickstart failed: %d", ret);
	 *     return ret;
	 * }
	 */

	return 0;
}

/*
 * Device instance macro.
 *
 * This macro instantiates a device instance for each LC709203F node in the devicetree.
 * Ensure your devicetree binding uses:
 *
 *   compatible = "lc709203f";
 *   reg = <0x0B>;
 *
 * and is connected to an I2C bus.
 */
#define LC709203F_INIT(inst)                                                                       \
	static struct lc709203f_data lc709203f_data_##inst;                                        \
	static const struct lc709203f_config lc709203f_config_##inst = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, lc709203f_init, NULL, &lc709203f_data_##inst,                  \
			      &lc709203f_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  \
			      &lc709203f_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LC709203F_INIT)
