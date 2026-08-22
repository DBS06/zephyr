/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/ztest.h>

#include "ptp_clock_nxp_enet_qos_output.h"

static struct precision_clock_output_raw_waveform_config valid_config(void)
{
	return (struct precision_clock_output_raw_waveform_config){
		.first_rising_time = 5 * (precision_time_t)NSEC_PER_SEC,
		.period_ns = NSEC_PER_SEC,
		.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT,
	};
}

ZTEST(nxp_enet_qos_output, test_fixed_caps_and_validation)
{
	struct precision_clock_output_caps caps;
	struct precision_clock_output_raw_waveform_config config = valid_config();

	zassert_ok(ptp_clock_nxp_enet_qos_output_caps(0, &caps));
	zassert_equal(caps.flags, PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM);
	zassert_equal(caps.channel_count, 1U);
	zassert_equal(caps.resolution_ns, NSEC_PER_SEC);
	zassert_equal(caps.min_lead_time_ns, 20 * NSEC_PER_MSEC);
	zassert_equal(caps.min_period_ns, NSEC_PER_SEC);
	zassert_equal(caps.max_period_ns, NSEC_PER_SEC);
	zassert_equal(caps.min_pulse_width_ns, 0);
	zassert_equal(caps.max_pulse_width_ns, 0);
	zassert_equal(ptp_clock_nxp_enet_qos_output_caps(1, &caps), -ENOTSUP);
	zassert_equal(ptp_clock_nxp_enet_qos_output_caps(0, NULL), -EINVAL);

	zassert_ok(ptp_clock_nxp_enet_qos_output_validate(0, &config));
	config.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_EXACT;
	config.pulse_width_ns = 200 * NSEC_PER_MSEC;
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, &config), -ENOTSUP);
	config.width_policy = (enum precision_clock_output_width_policy)99;
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, &config), -EINVAL);
	config = valid_config();
	config.period_ns = 2 * (precision_time_t)NSEC_PER_SEC;
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, &config), -ERANGE);
	config = valid_config();
	config.first_rising_time++;
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, &config), -ERANGE);
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(1, &config), -ENOTSUP);
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, NULL), -EINVAL);
}

ZTEST(nxp_enet_qos_output, test_gate_window_and_clock_movement)
{
	const precision_time_t target = 5 * (precision_time_t)NSEC_PER_SEC;
	struct ptp_clock_nxp_enet_qos_output_gate_decision decision;

	decision = ptp_clock_nxp_enet_qos_output_gate_decide(4 * NSEC_PER_SEC, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_WAIT);
	zassert_equal(decision.delay_ns, 250 * NSEC_PER_MSEC);

	decision =
		ptp_clock_nxp_enet_qos_output_gate_decide(target - 500 * NSEC_PER_MSEC - 1, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_WAIT);
	zassert_equal(decision.delay_ns, 1);

	decision = ptp_clock_nxp_enet_qos_output_gate_decide(target - 500 * NSEC_PER_MSEC, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_ROUTE);

	/* A minimum-lead request is routed synchronously by start_waveform(). */
	decision = ptp_clock_nxp_enet_qos_output_gate_decide(target - 20 * NSEC_PER_MSEC, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_ROUTE);

	decision = ptp_clock_nxp_enet_qos_output_gate_decide(target - 1, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_MISSED);

	decision = ptp_clock_nxp_enet_qos_output_gate_decide(
		target - PTP_CLOCK_NXP_ENET_QOS_OUTPUT_ROUTE_LEAD_NS, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_ROUTE);
	decision = ptp_clock_nxp_enet_qos_output_gate_decide(
		target - PTP_CLOCK_NXP_ENET_QOS_OUTPUT_ROUTE_LEAD_NS + 1, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_MISSED);

	decision = ptp_clock_nxp_enet_qos_output_gate_decide(target, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_MISSED);

	/* A backward PHC step returns an armed route to bounded polling. */
	decision = ptp_clock_nxp_enet_qos_output_gate_decide(target - 750 * NSEC_PER_MSEC, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_WAIT);
	zassert_equal(decision.delay_ns, 250 * NSEC_PER_MSEC);

	/* A forward step beyond the target is treated as a missed activation. */
	decision = ptp_clock_nxp_enet_qos_output_gate_decide(target + 1, target);
	zassert_equal(decision.action, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_MISSED);
}

ZTEST(nxp_enet_qos_output, test_seconds_counter_range)
{
	struct precision_clock_output_raw_waveform_config config = valid_config();

	config.first_rising_time = (precision_time_t)UINT32_MAX * NSEC_PER_SEC;
	zassert_ok(ptp_clock_nxp_enet_qos_output_validate(0, &config));
	config.first_rising_time += NSEC_PER_SEC;
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, &config), -ERANGE);
	config.first_rising_time += NSEC_PER_SEC;
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, &config), -ERANGE);
	config.first_rising_time = -(precision_time_t)NSEC_PER_SEC;
	zassert_equal(ptp_clock_nxp_enet_qos_output_validate(0, &config), -ERANGE);
}

ZTEST_SUITE(nxp_enet_qos_output, NULL, NULL, NULL, NULL, NULL);
