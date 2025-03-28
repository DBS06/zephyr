/*
 * Copyright (c) 2025 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LC709203F_LC709203F_H_
#define ZEPHYR_DRIVERS_SENSOR_LC709203F_LC709203F_H_

#include <zephyr/drivers/i2c.h>

enum lc709203f_regs {
	LC709203F_REG_BEFORE_RSOC = 0x04,       // [ ] Initialize before RSOC
	LC709203F_REG_THERMISTOR_B = 0x06,      // [x] Read/write thermistor B
	LC709203F_REG_INITIAL_RSOC = 0x07,      // [x] Initialize RSOC calculation
	LC709203F_REG_CELL_TEMPERATURE = 0x08,  // [x] Read/write cell temperature
	LC709203F_REG_CELL_VOLTAGE = 0x09,      // [x] Read batt voltage
	LC709203F_REG_CURRENT_DIRECTION = 0x0A, // [x] Read/write current direction
	LC709203F_REG_APA = 0x0B,               // [x] Adjustment Pack Application
	LC709203F_REG_APT = 0x0C,               // [x] Read/write Adjustment Pack Thermistor
	LC709203F_REG_RSOC = 0x0D,              // [x] Read state of charge; 1% 0−100 scale
	LC709203F_REG_CELL_ITE = 0x0F,          // [ ] Read batt indicator to empty
	LC709203F_REG_IC_VERSION = 0x11,        // [ ] Read IC version
	LC709203F_REG_BAT_PROFILE = 0x12,       // [x] Set the battery profile
	LC709203F_REG_ALARM_LOW_RSOC = 0x13,    // [x] Alarm on percent threshold
	LC709203F_REG_ALARM_LOW_VOLTAGE = 0x14, // [x] Alarm on voltage threshold
	LC709203F_REG_IC_POWER_MODE = 0x15,     // [x] Sets sleep/power mode
	LC709203F_REG_STATUS_BIT = 0x16,        // [x] Temperature obtaining method
	LC709203F_REG_NUM_PARAMETER = 0x1A      // [ ] Batt profile code
};

/* Battery temperature source */
enum lc709203f_temp_mode {
	LC709203F_TEMPERATURE_I2C = 0x0000,
	LC709203F_TEMPERATURE_THERMISTOR = 0x0001,
};

/* Chip power state */
enum lc709203f_power_mode {
	LC709203F_POWER_MODE_OPERATIONAL = 0x0001,
	LC709203F_POWER_MODE_SLEEP = 0x0002,
};

/* Current Direction Auto/Charge/Discharge mode */
enum lc709203f_current_direction {
	LC709203F_DIRECTION_AUTO = 0x0000,
	LC709203F_DIRECTION_CHARGE = 0x0001,
	LC709203F_DIRECTION_DISCHARGE = 0xFFFF,
};

/* Selects a battery profile */
enum lc709203f_battery_profile {
	LC709203F_BATTERY_PROFILE_0 = 0x0000,
	LC709203F_BATTERY_PROFILE_1 = 0x0001,
};

/* Approx battery pack size. Pick the closest of the following values for your
   battery size. */
enum lc709203f_battery_apa {
	LC709203F_APA_100MAH = 0x08,
	LC709203F_APA_200MAH = 0x0B,
	LC709203F_APA_500MAH = 0x10,
	LC709203F_APA_1000MAH = 0x19,
	LC709203F_APA_2000MAH = 0x2D,
	LC709203F_APA_3000MAH = 0x36,
};

struct lc709203f_config {
	struct i2c_dt_spec i2c;
	bool initial_rsoc;
	char *battery_apa;
	enum lc709203f_battery_profile battery_profile;
	bool thermistor;
	int thermistor_b_value;
	int thermistor_apt;
	enum lc709203f_temp_mode thermistor_mode;
};
#endif /* ZEPHYR_DRIVERS_SENSOR_LC709203F_LC709203F_H_ */
