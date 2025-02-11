/*
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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>

LOG_MODULE_REGISTER(lc709203f, CONFIG_SENSOR_LOG_LEVEL);

/* Default I2C address if not overridden by device tree */
#define LC709203F_DEFAULT_I2C_ADDR 0x0B

/* LC709203F Register Addresses (adjust as needed) */
#define LC709203F_REG_VOLTAGE 0x09 /* Battery voltage register */
#define LC709203F_REG_SOC     0x0A /* State-of-charge register */
#define LC709203F_REG_TEMP    0x0C /* Battery temperature register */

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

struct lc709203f_config {
	const struct device *i2c; /* I2C bus device */
	uint16_t i2c_addr;        /* I2C address of the LC709203F */
};

struct lc709203f_data {
	/* Last sensor readings (raw values as returned from the chip) */
	uint16_t voltage; /* Raw battery voltage */
	uint16_t soc;     /* Raw state-of-charge (percentage) */
	int16_t temp;     /* Raw temperature (assumed in deci-deg Celsius) */
};

/**
 * @brief Read a 16‐bit register from the LC709203F.
 *
 * This helper reads two consecutive bytes starting at register @p reg,
 * returning the 16‐bit big–endian value in @p val.
 */
static int lc709203f_read_reg(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct lc709203f_config *cfg = dev->config;
	uint8_t buf[2];
	int ret;

	ret = i2c_burst_read(cfg->i2c, cfg->i2c_addr, reg, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read reg 0x%x: %d", reg, ret);
		return ret;
	}

	*val = (buf[0] << 8) | buf[1];
	return 0;
}

/**
 * @brief Fetch a new sample from the sensor.
 *
 * This function reads the voltage, SOC, and temperature registers from the LC709203F.
 */
static int lc709203f_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct lc709203f_data *data = dev->data;
	uint16_t raw;
	int ret;

	/* Read battery voltage */
	ret = lc709203f_read_reg(dev, LC709203F_REG_VOLTAGE, &raw);
	if (ret < 0) {
		return ret;
	}
	data->voltage = raw;

	/* Read state-of-charge */
	ret = lc709203f_read_reg(dev, LC709203F_REG_SOC, &raw);
	if (ret < 0) {
		return ret;
	}
	data->soc = raw;

	/* Read temperature */
	ret = lc709203f_read_reg(dev, LC709203F_REG_TEMP, &raw);
	if (ret < 0) {
		return ret;
	}
	data->temp = (int16_t)raw;

	return 0;
}

/**
 * @brief Get a sensor value for a given channel.
 *
 * The supported channels in this driver are:
 *
 * - SENSOR_CHAN_GAUGE_VOLTAGE: Battery voltage (in V)
 * - SENSOR_CHAN_GAUGE_STATE_OF_CHARGE: State-of-charge (percentage)
 * - SENSOR_CHAN_AMBIENT_TEMP: Battery temperature (in °C)
 *
 * Conversion from raw values to sensor_value is performed as follows:
 * - Voltage: raw * 1.1 gives mV; then converted to V.
 * - SOC: assumed to be already in percent.
 * - Temperature: raw value is assumed to be in deci-degrees Celsius.
 */
static int lc709203f_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct lc709203f_data *data = dev->data;
	int32_t tmp;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE: {
		/* Convert raw voltage to millivolts.
		 * voltage_mv = (raw * 11 + 5) / 10 rounds the value.
		 */
		uint32_t voltage_mv = ((uint32_t)data->voltage * 11 + 5) / 10;
		/* Convert mV to V in sensor_value:
		 *   val1 = integer part (volts)
		 *   val2 = fractional part (nanovolts)
		 */
		val->val1 = voltage_mv / 1000;
		val->val2 = (voltage_mv % 1000) * 1000000;
		break;
	}
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE: {
		/* SOC is provided as an integer percentage */
		val->val1 = data->soc;
		val->val2 = 0;
		break;
	}
	case SENSOR_CHAN_AMBIENT_TEMP: {
		/* Convert temperature from deci-deg Celsius to deg Celsius.
		 * For example, if data->temp is 253 then the temperature is 25.3°C.
		 */
		tmp = data->temp;
		val->val1 = tmp / 10;
		/* Multiply the remainder by 100,000,000 (0.1°C = 100,000,000 n°C) */
		val->val2 = (tmp % 10) * 100000000;
		break;
	}
	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Initialize the LC709203F sensor.
 *
 * This function verifies that the I2C bus is ready and performs any additional
 * initialization required by the sensor.
 */
static int lc709203f_init(const struct device *dev)
{
	const struct lc709203f_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c)) {
		LOG_ERR("I2C bus device %s not ready", cfg->i2c->name);
		return -ENODEV;
	}

	/* Any additional sensor initialization (e.g. configuration, calibration)
	 * can be added here.
	 */

	LOG_INF("LC709203F initialized on I2C bus %s (addr 0x%x)", cfg->i2c->name, cfg->i2c_addr);

	return 0;
}

/* Define the sensor driver API structure */
static const struct sensor_driver_api lc709203f_driver_api = {
	.sample_fetch = lc709203f_sample_fetch, .channel_get = lc709203f_channel_get,
	/* .trigger_set can be added here if using interrupts/triggers */
};

#ifdef CONFIG_LC709203F_TRIGGER
/* Implement trigger support if needed */
#endif

/*
 * Device instantiation using device tree macros.
 *
 * For each LC709203F node in your device tree (with matching "compatible"
 * string), this macro instantiates a driver instance.
 */
#define LC709203F_INIT(inst)                                                                       \
	static struct lc709203f_data lc709203f_data_##inst;                                        \
                                                                                                   \
	static const struct lc709203f_config lc709203f_config_##inst = {                           \
		.i2c = DEVICE_DT_GET(DT_INST_BUS(inst)),                                           \
		.i2c_addr = DT_INST_REG_ADDR(inst),                                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, lc709203f_init, NULL, &lc709203f_data_##inst,                  \
			      &lc709203f_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  \
			      &lc709203f_driver_api);

/* This will instantiate the driver for every matching device tree instance */
DT_INST_FOREACH(LC709203F_INIT)
