/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Adjustable software precision clock backed by monotonic kernel time.
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_SOFTWARE_CLOCK_H_
#define ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_SOFTWARE_CLOCK_H_

#include <zephyr/kernel.h>
#include <zephyr/precision_timing/precision_clock.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Adjustable software precision clock
 * @defgroup precision_software_clock Adjustable software precision clock
 * @since 4.5
 * @version 0.1.0
 * @ingroup precision_timing
 * @{
 */

/** Caller-owned state for an adjustable software precision clock.
 *
 * Initialize the structure with precision_software_clock_init() and do not
 * copy or inspect it afterward. Access the clock through
 * precision_software_clock_get().
 */
struct precision_software_clock {
	/** Lock protecting the clock anchor and rate. */
	struct k_mutex lock;
	/** Precision clock interface exposed to callers. */
	struct precision_clock clock;
	/** Software time at the monotonic anchor. */
	precision_time_t anchor_time_ns;
	/** Monotonic time at which the software clock was last anchored. */
	precision_time_t anchor_monotonic_ns;
	/** Current rate adjustment in parts per billion. */
	int32_t rate_ppb;
	/** Whether initialization completed successfully. */
	bool initialized;
};

/**
 * @brief Initialize an adjustable monotonic-backed software clock.
 *
 * The software clock is independent of kernel monotonic time and
 * @ref SYS_CLOCK_REALTIME. Explicit set and phase operations may move its
 * value backward. A stable 64-bit system cycle counter provides the backing
 * time when available; otherwise the clock uses kernel uptime ticks.
 *
 * @param clock Caller-owned software clock state.
 * @param output_domain Valid domain exposed by the clock.
 * @param initial_time Initial clock value in nanoseconds.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p clock or @p output_domain is invalid.
 * @retval -ERANGE if the monotonic anchor cannot be represented.
 * @return A kernel mutex initialization error on failure.
 */
int precision_software_clock_init(struct precision_software_clock *clock,
				  struct precision_time_domain output_domain,
				  precision_time_t initial_time);

/**
 * @brief Get the precision clock interface of a software clock.
 *
 * @param clock Initialized software clock state.
 *
 * @return Precision clock interface, or null if @p clock is null or was not
 * initialized successfully.
 */
const struct precision_clock *
precision_software_clock_get(const struct precision_software_clock *clock);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_SOFTWARE_CLOCK_H_ */
