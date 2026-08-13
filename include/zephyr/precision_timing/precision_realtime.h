/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Explicit bridge from UTC precision clocks to system real-time.
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_REALTIME_H_
#define ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_REALTIME_H_

#include <zephyr/precision_timing/precision_clock.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Precision Real-Time Bridge
 * @defgroup precision_realtime Precision Real-Time Bridge
 * @since 4.5
 * @version 0.1.0
 * @ingroup precision_timing
 * @{
 */

/**
 * @brief Step system real-time from a UTC-domain precision clock.
 *
 * This performs one explicit, potentially discontinuous update of
 * @ref SYS_CLOCK_REALTIME. It does not schedule future updates, establish
 * ownership of the global clock, or convert from another time scale. The
 * caller is responsible for those policies.
 *
 * The clock value is interpreted as signed nanoseconds from the POSIX epoch.
 * Values before the epoch are accepted when they are representable by
 * @c time_t.
 *
 * @param utc_clock Readable precision clock whose domain type is
 *                  @ref PRECISION_TIME_DOMAIN_UTC.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p utc_clock is null, invalid, or not UTC-domain.
 * @retval -ENOTSUP if the clock cannot be read.
 * @retval -ERANGE if the clock value cannot be represented by system real-time.
 * @return A clock-provider or system-clock error on failure.
 */
int precision_clock_step_realtime(const struct precision_clock *utc_clock);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_REALTIME_H_ */
