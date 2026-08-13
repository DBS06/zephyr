/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <time.h>

#include <zephyr/precision_timing/precision_realtime.h>
#include <zephyr/precision_timing/precision_time.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/timeutil.h>

int precision_clock_step_realtime(const struct precision_clock *utc_clock)
{
	struct precision_time_point utc_time;
	struct timespec realtime;
	precision_time_t seconds;
	precision_time_t nanoseconds;
	int ret;

	if (utc_clock == NULL || utc_clock->domain.type != PRECISION_TIME_DOMAIN_UTC) {
		return -EINVAL;
	}

	ret = precision_clock_read(utc_clock, &utc_time);
	if (ret < 0) {
		return ret;
	}

	seconds = utc_time.time / NSEC_PER_SEC;
	nanoseconds = utc_time.time % NSEC_PER_SEC;
	if (nanoseconds < 0) {
		seconds--;
		nanoseconds += NSEC_PER_SEC;
	}

	if (seconds < (precision_time_t)SYS_TIME_T_MIN ||
	    seconds > (precision_time_t)SYS_TIME_T_MAX) {
		return -ERANGE;
	}
	realtime.tv_sec = (time_t)seconds;
	realtime.tv_nsec = (long)nanoseconds;

	return sys_clock_settime(SYS_CLOCK_REALTIME, &realtime);
}
