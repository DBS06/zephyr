/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Adapter from PTP clock drivers to precision timing clocks.
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_TIMING_PRECISION_PTP_CLOCK_H_
#define ZEPHYR_INCLUDE_ZEPHYR_TIMING_PRECISION_PTP_CLOCK_H_

#include <zephyr/device.h>
#include <zephyr/timing/precision_timing.h>

#ifdef __cplusplus
extern "C" {
#endif

struct precision_ptp_clock_adapter {
	struct precision_clock clock;
	const struct device *ptp_clock;
	struct precision_clock_caps caps;
};

void precision_ptp_clock_init(struct precision_ptp_clock_adapter *adapter,
			      const struct device *ptp_clock,
			      struct precision_time_domain domain);

static inline const struct precision_clock *
precision_ptp_clock_get(const struct precision_ptp_clock_adapter *adapter)
{
	return &adapter->clock;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_TIMING_PRECISION_PTP_CLOCK_H_ */
