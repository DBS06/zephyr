/*
 * Copyright (c) 2023 Alvaro Garcia Gomez <maxpowel@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/fuel_gauge.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(onsemi_lc709203f);
	int ret = 0;

	if (dev == NULL) {
		LOG_ERR("no device found.");
		return 0;
	}

	if (!device_is_ready(dev)) {
		LOG_ERR("Error: Device \"%s\" is not ready; check the driver initialization logs "
			"for errors.",
			dev->name);
		return 0;
	}

	LOG_INF("Found device \"%s\", getting fuel gauge data", dev->name);

	if (dev == NULL) {
		return 0;
	}

	while (1) {

		fuel_gauge_prop_t props[] = {
			FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE,
			FUEL_GAUGE_VOLTAGE,
		};

		union fuel_gauge_prop_val vals[ARRAY_SIZE(props)];

		ret = fuel_gauge_get_props(dev, props, vals, ARRAY_SIZE(props));

		if (ret < 0) {
			LOG_ERR("Error: cannot get properties");
		} else {
			LOG_INF("Fuel gauge data: Charge: %d%%, Voltage: %dmV",
				vals[0].relative_state_of_charge, vals[1].voltage / 1000);
		}

		k_sleep(K_MSEC(5000));
	}
	return 0;
}
