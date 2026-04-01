/*
 * Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "eth_native_tap_ptp_emulated.h"

ZTEST(native_tap_ptp_clock, test_legacy_timestamp_passthrough)
{
	struct eth_native_tap_ptp_state state = { 0 };
	struct net_ptp_time tm;
	int64_t host_now_ns = 123456789LL;

	zassert_ok(eth_native_tap_ptp_read_time(&state, false, host_now_ns, &tm));
	zassert_equal(tm.second, 0U);
	zassert_equal(tm.nanosecond, 123456789U);
	zassert_false(state.initialized);
}

ZTEST(native_tap_ptp_clock, test_emulated_read_seeds_from_host_time)
{
	struct eth_native_tap_ptp_state state = { 0 };
	struct net_ptp_time tm;
	int64_t host_now_ns = (42LL * NSEC_PER_SEC) + 17;

	zassert_ok(eth_native_tap_ptp_read_time(&state, true, host_now_ns, &tm));
	zassert_true(state.initialized);
	zassert_equal(state.anchor_host_ns, host_now_ns);
	zassert_equal(state.anchor_phc_ns, host_now_ns);
	zassert_equal(state.rate_ratio, 1.0);
	zassert_equal(tm.second, 42U);
	zassert_equal(tm.nanosecond, 17U);
}

ZTEST(native_tap_ptp_clock, test_set_rebases_without_changing_rate)
{
	struct eth_native_tap_ptp_state state = { 0 };
	struct net_ptp_time set_time = {
		.second = 5,
		.nanosecond = 10,
	};
	struct net_ptp_time tm;
	int64_t host_now_ns = 1000;

	zassert_ok(eth_native_tap_ptp_rate_adjust(&state, host_now_ns, 2.0));
	zassert_ok(eth_native_tap_ptp_set_time(&state, host_now_ns + 50, &set_time));
	zassert_equal(state.rate_ratio, 2.0);
	zassert_ok(eth_native_tap_ptp_read_time(&state, true, host_now_ns + 150, &tm));
	zassert_equal(tm.second, 5U);
	zassert_equal(tm.nanosecond, 210U);
}

ZTEST(native_tap_ptp_clock, test_adjust_applies_increment_without_rate_change)
{
	struct eth_native_tap_ptp_state state = { 0 };
	struct net_ptp_time tm;

	zassert_ok(eth_native_tap_ptp_read_time(&state, true, 1000, &tm));
	zassert_ok(eth_native_tap_ptp_adjust_time(&state, 1100, 250));
	zassert_equal(state.rate_ratio, 1.0);
	zassert_ok(eth_native_tap_ptp_read_time(&state, true, 1200, &tm));
	zassert_equal(tm.second, 0U);
	zassert_equal(tm.nanosecond, 1450U);
}

ZTEST(native_tap_ptp_clock, test_rate_adjust_changes_slope_without_time_jump)
{
	struct eth_native_tap_ptp_state state = { 0 };
	struct net_ptp_time tm;

	zassert_ok(eth_native_tap_ptp_read_time(&state, true, 1000, &tm));
	zassert_ok(eth_native_tap_ptp_rate_adjust(&state, 1500, 2.0));

	zassert_ok(eth_native_tap_ptp_read_time(&state, true, 1500, &tm));
	zassert_equal(tm.second, 0U);
	zassert_equal(tm.nanosecond, 1500U);

	zassert_ok(eth_native_tap_ptp_read_time(&state, true, 1800, &tm));
	zassert_equal(tm.second, 0U);
	zassert_equal(tm.nanosecond, 2100U);
}

ZTEST_SUITE(native_tap_ptp_clock, NULL, NULL, NULL, NULL, NULL);
