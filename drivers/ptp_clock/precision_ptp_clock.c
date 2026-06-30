/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <limits.h>

#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/timing/precision_ptp_clock.h>

static int precision_ptp_read(const struct precision_clock *clock,
			      struct precision_time_point *time)
{
	struct precision_ptp_clock_adapter *adapter =
		(struct precision_ptp_clock_adapter *)clock->user_data;
	const struct ptp_clock_driver_api *api;
	struct net_ptp_time ptp_time;
	int ret;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->get == NULL) {
		return -ENOTSUP;
	}

	ret = api->get(adapter->ptp_clock, &ptp_time);
	if (ret < 0) {
		return ret;
	}

	ret = precision_time_from_u64_sec_nsec(ptp_time.second, ptp_time.nanosecond,
					       &time->time);
	if (ret < 0) {
		return ret;
	}

	time->domain = clock->domain;

	return 0;
}

static int precision_ptp_set(const struct precision_clock *clock,
			     const struct precision_time_point *time)
{
	struct precision_ptp_clock_adapter *adapter =
		(struct precision_ptp_clock_adapter *)clock->user_data;
	const struct ptp_clock_driver_api *api;
	struct net_ptp_time ptp_time;
	int ret;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->set == NULL) {
		return -ENOTSUP;
	}

	ret = precision_time_to_u64_sec_nsec(time->time, &ptp_time.second,
					     &ptp_time.nanosecond);
	if (ret < 0) {
		return ret;
	}

	return api->set(adapter->ptp_clock, &ptp_time);
}

static int precision_ptp_adjust_phase(const struct precision_clock *clock,
				      precision_time_t phase_ns)
{
	struct precision_ptp_clock_adapter *adapter =
		(struct precision_ptp_clock_adapter *)clock->user_data;
	const struct ptp_clock_driver_api *api;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	if (phase_ns < INT_MIN || phase_ns > INT_MAX) {
		return -ERANGE;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->adjust == NULL) {
		return -ENOTSUP;
	}

	return api->adjust(adapter->ptp_clock, (int)phase_ns);
}

static int precision_ptp_adjust_rate(const struct precision_clock *clock, int32_t rate_ppb)
{
	struct precision_ptp_clock_adapter *adapter =
		(struct precision_ptp_clock_adapter *)clock->user_data;
	const struct ptp_clock_driver_api *api;
	double ratio;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->rate_adjust == NULL) {
		return -ENOTSUP;
	}

	if (rate_ppb <= -1000000000) {
		return -ERANGE;
	}

	ratio = 1.0 + ((double)rate_ppb / 1000000000.0);

	return api->rate_adjust(adapter->ptp_clock, ratio);
}

static int precision_ptp_get_caps(const struct precision_clock *clock,
				  struct precision_clock_caps *caps)
{
	struct precision_ptp_clock_adapter *adapter =
		(struct precision_ptp_clock_adapter *)clock->user_data;

	if (adapter == NULL || caps == NULL) {
		return -EINVAL;
	}

	*caps = adapter->caps;

	return 0;
}

static const struct precision_clock_api precision_ptp_api = {
	.read = precision_ptp_read,
	.set = precision_ptp_set,
	.adjust_phase = precision_ptp_adjust_phase,
	.adjust_rate = precision_ptp_adjust_rate,
	.get_caps = precision_ptp_get_caps,
};

void precision_ptp_clock_init(struct precision_ptp_clock_adapter *adapter,
			      const struct device *ptp_clock,
			      struct precision_time_domain domain)
{
	const struct ptp_clock_driver_api *api = NULL;
	uint32_t flags = 0U;

	if (adapter == NULL) {
		return;
	}

	if (ptp_clock != NULL) {
		api = DEVICE_API_GET(ptp_clock, ptp_clock);
	}

	if (api != NULL) {
		if (api->get != NULL) {
			flags |= PRECISION_CLOCK_CAP_READ;
		}
		if (api->set != NULL) {
			flags |= PRECISION_CLOCK_CAP_SET;
		}
		if (api->adjust != NULL) {
			flags |= PRECISION_CLOCK_CAP_ADJUST_PHASE;
		}
		if (api->rate_adjust != NULL) {
			flags |= PRECISION_CLOCK_CAP_ADJUST_RATE;
		}
	}

	adapter->ptp_clock = ptp_clock;
	adapter->caps = (struct precision_clock_caps) {
		.flags = flags,
		.resolution_ns = 1,
		.max_phase_adjust_ns = INT_MAX,
		.min_rate_ppb = -999999999,
		.max_rate_ppb = INT32_MAX,
	};
	adapter->clock = (struct precision_clock) {
		.api = &precision_ptp_api,
		.user_data = adapter,
		.domain = domain,
	};
}
