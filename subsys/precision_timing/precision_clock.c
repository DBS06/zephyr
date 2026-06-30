/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/timing/precision_timing.h>

int precision_clock_read(const struct precision_clock *clock, struct precision_time_point *time)
{
	int ret;

	if (clock == NULL || time == NULL || clock->api == NULL) {
		return -EINVAL;
	}

	if (clock->api->read == NULL) {
		return -ENOTSUP;
	}

	ret = clock->api->read(clock, time);
	if (ret == 0 && time->domain.type == PRECISION_TIME_DOMAIN_INVALID) {
		time->domain = clock->domain;
	}

	return ret;
}

int precision_clock_set(const struct precision_clock *clock,
			const struct precision_time_point *time)
{
	if (clock == NULL || time == NULL || clock->api == NULL) {
		return -EINVAL;
	}

	if (!precision_time_domain_equal(&clock->domain, &time->domain)) {
		return -EINVAL;
	}

	if (clock->api->set == NULL) {
		return -ENOTSUP;
	}

	return clock->api->set(clock, time);
}

int precision_clock_adjust_phase(const struct precision_clock *clock, precision_time_t phase_ns)
{
	if (clock == NULL || clock->api == NULL) {
		return -EINVAL;
	}

	if (clock->api->adjust_phase == NULL) {
		return -ENOTSUP;
	}

	return clock->api->adjust_phase(clock, phase_ns);
}

int precision_clock_adjust_rate(const struct precision_clock *clock, int32_t rate_ppb)
{
	if (clock == NULL || clock->api == NULL) {
		return -EINVAL;
	}

	if (clock->api->adjust_rate == NULL) {
		return -ENOTSUP;
	}

	return clock->api->adjust_rate(clock, rate_ppb);
}

int precision_clock_get_caps(const struct precision_clock *clock, struct precision_clock_caps *caps)
{
	if (clock == NULL || caps == NULL || clock->api == NULL) {
		return -EINVAL;
	}

	if (clock->api->get_caps == NULL) {
		return -ENOTSUP;
	}

	return clock->api->get_caps(clock, caps);
}
