/*
 * Copyright (c) 2025 Philipp Steiner <philipp.steiner1987@gmailcom>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LC709203F_LC709203F_H_
#define ZEPHYR_DRIVERS_SENSOR_LC709203F_LC709203F_H_

#include <zephyr/drivers/i2c.h>

#define REGISTER_THERMISTORB   0x06 ///< Read/write thermistor B
#define REGISTER_INITIAL_RSOC  0x07 ///< Initialize RSOC calculation
#define REGISTER_CELL_TEMP     0x08 ///< Read/write batt temperature
#define REGISTER_CELL_VOLT     0x09 ///< Read batt voltage
#define REGISTER_APA           0x0B ///< Adjustment Pack Application
#define REGISTER_RSOC          0x0D ///< Read state of charge
#define REGISTER_CELLITE       0x0F ///< Read batt indicator to empty
#define REGISTER_IC_VERSION    0x11 ///< Read IC version
#define REGISTER_BAT_PROFILE   0x12 ///< Set the battery profile
#define REGISTER_ALRM_LOW_RSOC 0x13 ///< Alarm on percent threshold
#define REGISTER_ALRM_LOW_VOLT 0x14 ///< Alarm on voltage threshold
#define REGISTER_POWER_MODE    0x15 ///< Sets sleep/power mode
#define REGISTER_STATUSBIT     0x16 ///< Temperature obtaining method
#define REGISTER_PARAMETER     0x1A ///< Batt profile code

struct lc709203f_config {
	struct i2c_dt_spec i2c;
};

// TODO: Below Enums needs to be moved to KCONFIG
/* Battery temperature source */
typedef enum {
	LC709203F_TEMPERATURE_I2C = 0x0000,
	LC709203F_TEMPERATURE_THERMISTOR = 0x0001,
} lc709203_tempmode_t;

/* Chip power state */
typedef enum {
	LC709203F_POWER_OPERATE = 0x0001,
	LC709203F_POWER_SLEEP = 0x0002,
} lc709203_powermode_t;

/* Approx battery pack size */
typedef enum {
	LC709203F_APA_100MAH = 0x08,
	LC709203F_APA_200MAH = 0x0B,
	LC709203F_APA_500MAH = 0x10,
	LC709203F_APA_1000MAH = 0x19,
	LC709203F_APA_2000MAH = 0x2D,
	LC709203F_APA_3000MAH = 0x36,
} lc709203_adjustment_t;

#endif /* ZEPHYR_DRIVERS_SENSOR_LC709203F_LC709203F_H_ */
