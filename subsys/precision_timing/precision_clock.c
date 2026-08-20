/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/precision_timing/precision_clock.h>

int precision_clock_read(const struct precision_clock *precision_clk,
			 struct precision_time_point *tp)
{
	int ret;

	if (precision_clk == NULL || tp == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (precision_clk->api->read == NULL) {
		return -ENOTSUP;
	}

	ret = precision_clk->api->read(precision_clk, tp);
	if (ret == 0 && tp->domain.type == PRECISION_TIME_DOMAIN_INVALID) {
		tp->domain = precision_clk->domain;
	}

	return ret;
}

int precision_clock_set(const struct precision_clock *precision_clk,
			const struct precision_time_point *tp)
{
	if (precision_clk == NULL || tp == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (!precision_time_domain_equal(&precision_clk->domain, &tp->domain)) {
		return -EINVAL;
	}

	if (precision_clk->api->set == NULL) {
		return -ENOTSUP;
	}

	return precision_clk->api->set(precision_clk, tp);
}

int precision_clock_adjust_phase(const struct precision_clock *precision_clk,
				 precision_time_t phase_ns)
{
	if (precision_clk == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (precision_clk->api->adjust_phase == NULL) {
		return -ENOTSUP;
	}

	return precision_clk->api->adjust_phase(precision_clk, phase_ns);
}

int precision_clock_adjust_rate(const struct precision_clock *precision_clk, int32_t rate_ppb)
{
	if (precision_clk == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (precision_clk->api->adjust_rate == NULL) {
		return -ENOTSUP;
	}

	return precision_clk->api->adjust_rate(precision_clk, rate_ppb);
}

int precision_clock_get_caps(const struct precision_clock *precision_clk,
			     struct precision_clock_caps *caps)
{
	if (precision_clk == NULL || caps == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (precision_clk->api->get_caps == NULL) {
		return -ENOTSUP;
	}

	return precision_clk->api->get_caps(precision_clk, caps);
}

#ifdef CONFIG_PRECISION_CLOCK_OUTPUT

int precision_clock_output_get_caps(const struct precision_clock *precision_clk, uint32_t channel,
				    struct precision_clock_output_caps *caps)
{
	int ret;

	if (precision_clk == NULL || caps == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (precision_clk->api->get_output_caps == NULL) {
		return -ENOTSUP;
	}

	ret = precision_clk->api->get_output_caps(precision_clk, channel, caps);
	if (ret != 0) {
		return ret;
	}

	if (channel >= caps->channel_count) {
		return -ENOTSUP;
	}

	return 0;
}

static int precision_clock_output_check_base_caps(const struct precision_clock_output_caps *caps)
{
	if (caps->resolution_ns <= 0 || caps->min_lead_time_ns < 0) {
		return -ERANGE;
	}

	return 0;
}

static int precision_clock_output_check_period_caps(const struct precision_clock_output_caps *caps)
{
	if (caps->min_period_ns <= 0 || caps->max_period_ns < caps->min_period_ns) {
		return -ERANGE;
	}

	return 0;
}

static int precision_clock_output_check_width_caps(const struct precision_clock_output_caps *caps)
{
	if (caps->min_pulse_width_ns <= 0 || caps->max_pulse_width_ns < caps->min_pulse_width_ns) {
		return -ERANGE;
	}

	return 0;
}

static int
precision_clock_output_check_event_config(const struct precision_clock_output_caps *caps,
					  const struct precision_clock_output_event_config *config)
{
	uint32_t edge_cap;
	int ret;

	if ((caps->flags & PRECISION_CLOCK_OUTPUT_CAP_EVENT) == 0U) {
		return -ENOTSUP;
	}

	switch (config->edge) {
	case PRECISION_CLOCK_OUTPUT_EDGE_RISING:
		edge_cap = PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING;
		break;
	case PRECISION_CLOCK_OUTPUT_EDGE_FALLING:
		edge_cap = PRECISION_CLOCK_OUTPUT_CAP_EDGE_FALLING;
		break;
	default:
		return -EINVAL;
	}

	if ((caps->flags & edge_cap) == 0U) {
		return -ENOTSUP;
	}

	ret = precision_clock_output_check_base_caps(caps);
	if (ret != 0) {
		return ret;
	}

	if ((config->target_time.time % caps->resolution_ns) != 0) {
		return -ERANGE;
	}

	return 0;
}

static int precision_clock_output_check_waveform_config(
	const struct precision_clock_output_caps *caps,
	const struct precision_clock_output_waveform_config *config)
{
	bool exact_width;
	int ret;

	if ((caps->flags & PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM) == 0U) {
		return -ENOTSUP;
	}

	switch (config->width_policy) {
	case PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT:
		exact_width = false;
		break;
	case PRECISION_CLOCK_OUTPUT_WIDTH_EXACT:
		exact_width = true;
		break;
	default:
		return -EINVAL;
	}

	if (config->period_ns <= 0 ||
	    (exact_width &&
	     (config->pulse_width_ns <= 0 || config->pulse_width_ns >= config->period_ns))) {
		return -EINVAL;
	}

	if (exact_width && (caps->flags & PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH) == 0U) {
		return -ENOTSUP;
	}

	ret = precision_clock_output_check_base_caps(caps);
	if (ret != 0) {
		return ret;
	}

	ret = precision_clock_output_check_period_caps(caps);
	if (ret != 0) {
		return ret;
	}

	if (config->period_ns < caps->min_period_ns || config->period_ns > caps->max_period_ns ||
	    (config->period_ns % caps->resolution_ns) != 0 ||
	    (config->first_rising_time.time % caps->resolution_ns) != 0) {
		return -ERANGE;
	}

	if (!exact_width) {
		return 0;
	}

	ret = precision_clock_output_check_width_caps(caps);
	if (ret != 0) {
		return ret;
	}

	if (config->pulse_width_ns < caps->min_pulse_width_ns ||
	    config->pulse_width_ns > caps->max_pulse_width_ns ||
	    (config->pulse_width_ns % caps->resolution_ns) != 0) {
		return -ERANGE;
	}

	return 0;
}

static int precision_clock_output_check_target(const struct precision_clock *precision_clk,
					       const struct precision_clock_output_caps *caps,
					       precision_time_t target_ns)
{
	struct precision_time_point now;
	precision_time_t earliest;
	int ret;

	ret = precision_clock_read(precision_clk, &now);
	if (ret != 0) {
		return ret;
	}

	if (target_ns < now.time) {
		return -ETIME;
	}

	ret = precision_time_add(now.time, caps->min_lead_time_ns, &earliest);
	if (ret != 0) {
		return ret;
	}

	if (target_ns < earliest) {
		return -ETIME;
	}

	return 0;
}

static int precision_clock_output_check_status(const struct precision_clock *precision_clk,
					       const struct precision_clock_output_caps *caps,
					       const struct precision_clock_output_status *status)
{
	int ret;

	if (status->hardware_active_valid &&
	    (caps->flags & PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE) == 0U) {
		return -EINVAL;
	}

	if (!status->configured) {
		return 0;
	}

	switch (status->kind) {
	case PRECISION_CLOCK_OUTPUT_KIND_EVENT:
		if (!precision_time_domain_equal(&precision_clk->domain,
						 &status->config.event.target_time.domain)) {
			return -EINVAL;
		}
		ret = precision_clock_output_check_event_config(caps, &status->config.event);
		break;
	case PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM:
		if (!precision_time_domain_equal(
			    &precision_clk->domain,
			    &status->config.waveform.first_rising_time.domain)) {
			return -EINVAL;
		}
		ret = precision_clock_output_check_waveform_config(caps, &status->config.waveform);
		break;
	default:
		return -EINVAL;
	}

	return ret == 0 ? 0 : -EINVAL;
}

int precision_clock_output_next_start_time(const struct precision_time_point *now,
					   precision_time_t period_ns,
					   precision_time_t min_lead_time_ns,
					   struct precision_time_point *start_time)
{
	precision_time_t boundary;
	precision_time_t remainder;
	int ret;

	if (now == NULL || start_time == NULL || period_ns <= 0 || min_lead_time_ns < 0) {
		return -EINVAL;
	}

	ret = precision_time_add(now->time, min_lead_time_ns, &boundary);
	if (ret != 0) {
		return ret;
	}

	remainder = boundary % period_ns;
	if (remainder > 0) {
		ret = precision_time_add(boundary, period_ns - remainder, &boundary);
	} else if (remainder < 0) {
		ret = precision_time_sub(boundary, remainder, &boundary);
	}
	if (ret != 0) {
		return ret;
	}

	*start_time = (struct precision_time_point){
		.time = boundary,
		.domain = now->domain,
	};

	return 0;
}

int precision_clock_output_schedule_event(const struct precision_clock *precision_clk,
					  uint32_t channel,
					  const struct precision_clock_output_event_config *config)
{
	struct precision_clock_output_caps caps;
	int ret;

	if (precision_clk == NULL || config == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (!precision_time_domain_equal(&precision_clk->domain, &config->target_time.domain)) {
		return -EINVAL;
	}

	if (precision_clk->api->output_schedule_event == NULL) {
		return -ENOTSUP;
	}

	ret = precision_clock_output_get_caps(precision_clk, channel, &caps);
	if (ret != 0) {
		return ret;
	}

	ret = precision_clock_output_check_event_config(&caps, config);
	if (ret != 0) {
		return ret;
	}

	ret = precision_clock_output_check_target(precision_clk, &caps, config->target_time.time);
	if (ret != 0) {
		return ret;
	}

	return precision_clk->api->output_schedule_event(precision_clk, channel, config);
}

int precision_clock_output_start_waveform(
	const struct precision_clock *precision_clk, uint32_t channel,
	const struct precision_clock_output_waveform_config *config)
{
	struct precision_clock_output_caps caps;
	int ret;

	if (precision_clk == NULL || config == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (!precision_time_domain_equal(&precision_clk->domain,
					 &config->first_rising_time.domain)) {
		return -EINVAL;
	}

	if (precision_clk->api->output_start_waveform == NULL) {
		return -ENOTSUP;
	}

	ret = precision_clock_output_get_caps(precision_clk, channel, &caps);
	if (ret != 0) {
		return ret;
	}

	ret = precision_clock_output_check_waveform_config(&caps, config);
	if (ret != 0) {
		return ret;
	}

	ret = precision_clock_output_check_target(precision_clk, &caps,
						  config->first_rising_time.time);
	if (ret != 0) {
		return ret;
	}

	return precision_clk->api->output_start_waveform(precision_clk, channel, config);
}

int precision_clock_output_stop(const struct precision_clock *precision_clk, uint32_t channel)
{
	if (precision_clk == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (precision_clk->api->output_stop == NULL) {
		return -ENOTSUP;
	}

	return precision_clk->api->output_stop(precision_clk, channel);
}

int precision_clock_output_get_status(const struct precision_clock *precision_clk, uint32_t channel,
				      struct precision_clock_output_status *status)
{
	struct precision_clock_output_caps caps;
	int ret;

	if (precision_clk == NULL || status == NULL || precision_clk->api == NULL) {
		return -EINVAL;
	}

	if (precision_clk->api->get_output_status == NULL) {
		return -ENOTSUP;
	}

	ret = precision_clock_output_get_caps(precision_clk, channel, &caps);
	if (ret != 0) {
		return ret;
	}

	ret = precision_clk->api->get_output_status(precision_clk, channel, status);
	if (ret != 0) {
		return ret;
	}

	return precision_clock_output_check_status(precision_clk, &caps, status);
}

#endif /* CONFIG_PRECISION_CLOCK_OUTPUT */
