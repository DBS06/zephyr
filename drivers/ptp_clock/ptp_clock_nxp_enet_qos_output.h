/*
 * SPDX-FileCopyrightText: Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PTP_CLOCK_PTP_CLOCK_NXP_ENET_QOS_OUTPUT_H_
#define ZEPHYR_DRIVERS_PTP_CLOCK_PTP_CLOCK_NXP_ENET_QOS_OUTPUT_H_

#include <errno.h>

#include <zephyr/drivers/precision_clock_output.h>
#include <zephyr/sys/util.h>

/*
 * Valid targets are integral seconds. Routing halfway through the preceding
 * period avoids exposing that pulse while leaving ample system-workqueue slack.
 */
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_PERIOD_NS      ((precision_time_t)NSEC_PER_SEC)
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_MIN_LEAD_NS    (20 * (precision_time_t)NSEC_PER_MSEC)
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_LEAD_NS   (500 * (precision_time_t)NSEC_PER_MSEC)
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_MAX_RECHECK_NS (250 * (precision_time_t)NSEC_PER_MSEC)

enum ptp_clock_nxp_enet_qos_output_gate_action {
	PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_WAIT,
	PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_ROUTE,
	PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_MISSED,
};

struct ptp_clock_nxp_enet_qos_output_gate_decision {
	enum ptp_clock_nxp_enet_qos_output_gate_action action;
	precision_time_t delay_ns;
};

static inline int ptp_clock_nxp_enet_qos_output_caps(
	uint32_t channel, struct precision_clock_output_caps *caps)
{
	if (caps == NULL) {
		return -EINVAL;
	}
	if (channel != 0U) {
		return -ENOTSUP;
	}

	*caps = (struct precision_clock_output_caps){
		.flags = PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM,
		.channel_count = 1U,
		.resolution_ns = PTP_CLOCK_NXP_ENET_QOS_OUTPUT_PERIOD_NS,
		.min_lead_time_ns = PTP_CLOCK_NXP_ENET_QOS_OUTPUT_MIN_LEAD_NS,
		.min_period_ns = PTP_CLOCK_NXP_ENET_QOS_OUTPUT_PERIOD_NS,
		.max_period_ns = PTP_CLOCK_NXP_ENET_QOS_OUTPUT_PERIOD_NS,
	};

	return 0;
}

static inline int ptp_clock_nxp_enet_qos_output_validate(
	uint32_t channel, const struct precision_clock_output_raw_waveform_config *config)
{
	if (config == NULL) {
		return -EINVAL;
	}
	if (channel != 0U) {
		return -ENOTSUP;
	}
	if (config->width_policy != PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT) {
		return config->width_policy == PRECISION_CLOCK_OUTPUT_WIDTH_EXACT ? -ENOTSUP
									  : -EINVAL;
	}
	if (config->period_ns != PTP_CLOCK_NXP_ENET_QOS_OUTPUT_PERIOD_NS ||
	    (config->first_rising_time % PTP_CLOCK_NXP_ENET_QOS_OUTPUT_PERIOD_NS) != 0) {
		return -ERANGE;
	}

	return 0;
}

static inline struct ptp_clock_nxp_enet_qos_output_gate_decision
ptp_clock_nxp_enet_qos_output_gate_decide(precision_time_t now_ns,
					  precision_time_t first_rising_time)
{
	precision_time_t gate_time_ns =
		first_rising_time - PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_LEAD_NS;
	struct ptp_clock_nxp_enet_qos_output_gate_decision decision = {0};

	if (now_ns >= first_rising_time) {
		decision.action = PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_MISSED;
	} else if (now_ns < gate_time_ns) {
		decision.action = PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_WAIT;
		decision.delay_ns = MIN(gate_time_ns - now_ns,
					PTP_CLOCK_NXP_ENET_QOS_OUTPUT_MAX_RECHECK_NS);
	} else {
		decision.action = PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_ROUTE;
	}

	return decision;
}

#endif /* ZEPHYR_DRIVERS_PTP_CLOCK_PTP_CLOCK_NXP_ENET_QOS_OUTPUT_H_ */
