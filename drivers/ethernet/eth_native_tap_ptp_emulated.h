/*
 * Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_NATIVE_TAP_PTP_EMULATED_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_NATIVE_TAP_PTP_EMULATED_H_

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/net/ptp_time.h>

struct eth_native_tap_ptp_state {
	int64_t anchor_host_ns;
	int64_t anchor_phc_ns;
	double rate_ratio;
	bool initialized;
};

static inline int eth_native_tap_ptp_time_to_ns(const struct net_ptp_time *tm, int64_t *ns)
{
	uint64_t second_limit = (uint64_t)(INT64_MAX / NSEC_PER_SEC);
	uint64_t second_remainder = (uint64_t)(INT64_MAX % NSEC_PER_SEC);

	if (tm == NULL || ns == NULL) {
		return -EINVAL;
	}

	if (tm->nanosecond >= NSEC_PER_SEC) {
		return -EINVAL;
	}

	if (tm->second > second_limit ||
	    (tm->second == second_limit && tm->nanosecond > second_remainder)) {
		return -ERANGE;
	}

	*ns = ((int64_t)tm->second * NSEC_PER_SEC) + tm->nanosecond;

	return 0;
}

static inline int eth_native_tap_ptp_ns_to_time(int64_t ns, struct net_ptp_time *tm)
{
	if (tm == NULL) {
		return -EINVAL;
	}

	if (ns < 0) {
		return -ERANGE;
	}

	tm->second = (uint64_t)(ns / NSEC_PER_SEC);
	tm->nanosecond = (uint32_t)(ns % NSEC_PER_SEC);

	return 0;
}

static inline int64_t eth_native_tap_ptp_scale_ns(int64_t delta_ns, double ratio)
{
	double scaled = (double)delta_ns * ratio;

	return (int64_t)(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

static inline void eth_native_tap_ptp_seed(struct eth_native_tap_ptp_state *state,
					   int64_t host_now_ns)
{
	state->anchor_host_ns = host_now_ns;
	state->anchor_phc_ns = host_now_ns;
	state->rate_ratio = 1.0;
	state->initialized = true;
}

static inline int64_t eth_native_tap_ptp_current_ns(const struct eth_native_tap_ptp_state *state,
						    int64_t host_now_ns)
{
	return state->anchor_phc_ns +
		eth_native_tap_ptp_scale_ns(host_now_ns - state->anchor_host_ns,
					    state->rate_ratio);
}

static inline int eth_native_tap_ptp_read_time(struct eth_native_tap_ptp_state *state,
					       bool emulated,
					       int64_t host_now_ns,
					       struct net_ptp_time *tm)
{
	int64_t time_ns = host_now_ns;

	if (emulated) {
		if (state == NULL) {
			return -EINVAL;
		}

		if (!state->initialized) {
			eth_native_tap_ptp_seed(state, host_now_ns);
		}

		time_ns = eth_native_tap_ptp_current_ns(state, host_now_ns);
	}

	return eth_native_tap_ptp_ns_to_time(time_ns, tm);
}

static inline int eth_native_tap_ptp_set_time(struct eth_native_tap_ptp_state *state,
					      int64_t host_now_ns,
					      const struct net_ptp_time *tm)
{
	double rate_ratio = 1.0;
	int64_t phc_ns;
	int ret;

	if (state == NULL) {
		return -EINVAL;
	}

	ret = eth_native_tap_ptp_time_to_ns(tm, &phc_ns);
	if (ret < 0) {
		return ret;
	}

	if (state->initialized) {
		rate_ratio = state->rate_ratio;
	}

	state->anchor_host_ns = host_now_ns;
	state->anchor_phc_ns = phc_ns;
	state->rate_ratio = rate_ratio;
	state->initialized = true;

	return 0;
}

static inline int eth_native_tap_ptp_adjust_time(struct eth_native_tap_ptp_state *state,
						 int64_t host_now_ns,
						 int increment)
{
	double rate_ratio = 1.0;
	int64_t phc_ns;

	if (state == NULL) {
		return -EINVAL;
	}

	if (!state->initialized) {
		eth_native_tap_ptp_seed(state, host_now_ns);
	} else {
		rate_ratio = state->rate_ratio;
	}

	phc_ns = eth_native_tap_ptp_current_ns(state, host_now_ns);
	phc_ns += increment;

	state->anchor_host_ns = host_now_ns;
	state->anchor_phc_ns = phc_ns;
	state->rate_ratio = rate_ratio;
	state->initialized = true;

	return 0;
}

static inline int eth_native_tap_ptp_rate_adjust(struct eth_native_tap_ptp_state *state,
						 int64_t host_now_ns,
						 double ratio)
{
	int64_t phc_ns;

	if (state == NULL) {
		return -EINVAL;
	}

	if (ratio <= 0.0) {
		return -EINVAL;
	}

	if (!state->initialized) {
		eth_native_tap_ptp_seed(state, host_now_ns);
	}

	phc_ns = eth_native_tap_ptp_current_ns(state, host_now_ns);

	state->anchor_host_ns = host_now_ns;
	state->anchor_phc_ns = phc_ns;
	state->rate_ratio = ratio;
	state->initialized = true;

	return 0;
}

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_NATIVE_TAP_PTP_EMULATED_H_ */
