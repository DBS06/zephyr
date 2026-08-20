/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <limits.h>

#include <zephyr/drivers/precision_clock_output.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/sys/check.h>
#include <zephyr/precision_timing/precision_clock_ptp.h>

/* Keep the rate ratio passed to ptp_clock_rate_adjust() strictly positive. */
#define PRECISION_CLOCK_PTP_MIN_RATE_PPB (-999999999)

static int precision_clock_ptp_read(const struct precision_clock *precision_clk,
				    struct precision_time_point *tp)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct ptp_clock_driver_api *api;
	struct net_ptp_time ptp_time;
	int ret;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_READ)) {
		return -ENOTSUP;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->get == NULL) {
		return -ENOTSUP;
	}

	ret = api->get(adapter->ptp_clock, &ptp_time);
	if (ret < 0) {
		return ret;
	}

	ret = precision_time_from_u64_sec_nsec(ptp_time.second, ptp_time.nanosecond, &tp->time);
	if (ret < 0) {
		return ret;
	}

	tp->domain = precision_clk->domain;

	return 0;
}

static int precision_clock_ptp_set(const struct precision_clock *precision_clk,
				   const struct precision_time_point *tp)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct ptp_clock_driver_api *api;
	struct net_ptp_time ptp_time;
	int ret;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_SET)) {
		return -ENOTSUP;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->set == NULL) {
		return -ENOTSUP;
	}

	ret = precision_time_to_u64_sec_nsec(tp->time, &ptp_time.second, &ptp_time.nanosecond);
	if (ret < 0) {
		return ret;
	}

	return api->set(adapter->ptp_clock, &ptp_time);
}

static int precision_clock_ptp_adjust_phase(const struct precision_clock *precision_clk,
					    precision_time_t phase_ns)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct ptp_clock_driver_api *api;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_ADJUST_PHASE)) {
		return -ENOTSUP;
	}

	if (phase_ns < INT_MIN || phase_ns > INT_MAX) {
		return -ERANGE;
	}

	if (adapter->caps.max_phase_adjust_ns > 0 &&
	    (phase_ns < -adapter->caps.max_phase_adjust_ns ||
	     phase_ns > adapter->caps.max_phase_adjust_ns)) {
		return -ERANGE;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->adjust == NULL) {
		return -ENOTSUP;
	}

	return api->adjust(adapter->ptp_clock, (int)phase_ns);
}

static int precision_clock_ptp_adjust_rate(const struct precision_clock *precision_clk,
					   int32_t rate_ppb)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct ptp_clock_driver_api *api;
	double ratio;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_ADJUST_RATE)) {
		return -ENOTSUP;
	}

	if (rate_ppb < adapter->caps.min_rate_ppb || rate_ppb > adapter->caps.max_rate_ppb) {
		return -ERANGE;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL || api->rate_adjust == NULL) {
		return -ENOTSUP;
	}

	ratio = 1.0 + ((double)rate_ppb / 1000000000.0);

	return api->rate_adjust(adapter->ptp_clock, ratio);
}

static int precision_clock_ptp_get_caps(const struct precision_clock *precision_clk,
					struct precision_clock_caps *caps)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;

	if (adapter == NULL || caps == NULL) {
		return -EINVAL;
	}

	*caps = adapter->caps;

	return 0;
}

#ifdef CONFIG_PRECISION_CLOCK_OUTPUT
static const struct precision_clock_output_provider *
precision_clock_ptp_output_provider(const struct precision_clock_ptp_adapter *adapter)
{
	const struct ptp_clock_driver_api *api;

	if (adapter == NULL || adapter->ptp_clock == NULL) {
		return NULL;
	}

	api = DEVICE_API_GET(ptp_clock, adapter->ptp_clock);
	if (api == NULL) {
		return NULL;
	}

	return api->output;
}

static bool precision_clock_ptp_has_output(const struct precision_clock_output_provider *provider)
{
	return provider != NULL && provider->get_caps != NULL && provider->stop != NULL &&
	       provider->get_status != NULL &&
	       (provider->schedule_event != NULL || provider->start_waveform != NULL);
}

static int
precision_clock_ptp_output_available(const struct device *ptp_clock,
				     const struct precision_clock_output_provider *provider,
				     bool *available)
{
	struct precision_clock_output_caps output_caps;
	int ret;

	*available = false;
	if (!precision_clock_ptp_has_output(provider)) {
		return 0;
	}

	ret = provider->get_caps(ptp_clock, 0U, &output_caps);
	if (ret == -ENOTSUP) {
		return 0;
	}
	if (ret < 0) {
		return ret;
	}

	*available = output_caps.channel_count > 0U;

	return 0;
}

static int precision_clock_ptp_get_output_caps(const struct precision_clock *precision_clk,
					       uint32_t channel,
					       struct precision_clock_output_caps *caps)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct precision_clock_output_provider *provider;

	if (adapter == NULL || caps == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT)) {
		return -ENOTSUP;
	}

	provider = precision_clock_ptp_output_provider(adapter);
	if (provider == NULL || provider->get_caps == NULL) {
		return -ENOTSUP;
	}

	return provider->get_caps(adapter->ptp_clock, channel, caps);
}

static int
precision_clock_ptp_output_schedule_event(const struct precision_clock *precision_clk,
					  uint32_t channel,
					  const struct precision_clock_output_event_config *config)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct precision_clock_output_provider *provider;
	struct precision_clock_output_raw_event_config raw_config;

	if (adapter == NULL || config == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT)) {
		return -ENOTSUP;
	}

	provider = precision_clock_ptp_output_provider(adapter);
	if (provider == NULL || provider->schedule_event == NULL) {
		return -ENOTSUP;
	}

	raw_config = (struct precision_clock_output_raw_event_config){
		.target_time = config->target_time.time,
		.edge = config->edge,
	};

	return provider->schedule_event(adapter->ptp_clock, channel, &raw_config);
}

static int precision_clock_ptp_output_start_waveform(
	const struct precision_clock *precision_clk, uint32_t channel,
	const struct precision_clock_output_waveform_config *config)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct precision_clock_output_provider *provider;
	struct precision_clock_output_raw_waveform_config raw_config;

	if (adapter == NULL || config == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT)) {
		return -ENOTSUP;
	}

	provider = precision_clock_ptp_output_provider(adapter);
	if (provider == NULL || provider->start_waveform == NULL) {
		return -ENOTSUP;
	}

	raw_config = (struct precision_clock_output_raw_waveform_config){
		.first_rising_time = config->first_rising_time.time,
		.period_ns = config->period_ns,
		.width_policy = config->width_policy,
		.pulse_width_ns = config->pulse_width_ns,
	};

	return provider->start_waveform(adapter->ptp_clock, channel, &raw_config);
}

static int precision_clock_ptp_output_stop(const struct precision_clock *precision_clk,
					   uint32_t channel)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct precision_clock_output_provider *provider;

	if (adapter == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT)) {
		return -ENOTSUP;
	}

	provider = precision_clock_ptp_output_provider(adapter);
	if (provider == NULL || provider->stop == NULL) {
		return -ENOTSUP;
	}

	return provider->stop(adapter->ptp_clock, channel);
}

static int precision_clock_ptp_get_output_status(const struct precision_clock *precision_clk,
						 uint32_t channel,
						 struct precision_clock_output_status *status)
{
	struct precision_clock_ptp_adapter *adapter =
		(struct precision_clock_ptp_adapter *)precision_clk->adapter;
	const struct precision_clock_output_provider *provider;
	struct precision_clock_output_raw_status raw_status;
	int ret;

	if (adapter == NULL || status == NULL) {
		return -EINVAL;
	}

	if (!(adapter->caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT)) {
		return -ENOTSUP;
	}

	provider = precision_clock_ptp_output_provider(adapter);
	if (provider == NULL || provider->get_status == NULL) {
		return -ENOTSUP;
	}

	ret = provider->get_status(adapter->ptp_clock, channel, &raw_status);
	if (ret < 0) {
		return ret;
	}

	*status = (struct precision_clock_output_status){
		.configured = raw_status.configured,
		.hardware_active_valid = raw_status.hardware_active_valid,
		.hardware_active = raw_status.hardware_active,
	};

	if (!raw_status.configured) {
		return 0;
	}

	status->kind = raw_status.kind;

	switch (raw_status.kind) {
	case PRECISION_CLOCK_OUTPUT_KIND_EVENT:
		status->config.event.target_time.time = raw_status.config.event.target_time;
		status->config.event.target_time.domain = precision_clk->domain;
		status->config.event.edge = raw_status.config.event.edge;
		break;
	case PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM:
		status->config.waveform.first_rising_time.time =
			raw_status.config.waveform.first_rising_time;
		status->config.waveform.first_rising_time.domain = precision_clk->domain;
		status->config.waveform.period_ns = raw_status.config.waveform.period_ns;
		status->config.waveform.width_policy = raw_status.config.waveform.width_policy;
		status->config.waveform.pulse_width_ns = raw_status.config.waveform.pulse_width_ns;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_PRECISION_CLOCK_OUTPUT */

static uint32_t precision_clock_ptp_cap_flags(uint32_t ptp_flags)
{
	uint32_t flags = 0U;

	if (ptp_flags & PTP_CLOCK_CAP_READ) {
		flags |= PRECISION_CLOCK_CAP_READ;
	}
	if (ptp_flags & PTP_CLOCK_CAP_SET) {
		flags |= PRECISION_CLOCK_CAP_SET;
	}
	if (ptp_flags & PTP_CLOCK_CAP_ADJUST) {
		flags |= PRECISION_CLOCK_CAP_ADJUST_PHASE;
	}
	if (ptp_flags & PTP_CLOCK_CAP_RATE_ADJUST) {
		flags |= PRECISION_CLOCK_CAP_ADJUST_RATE;
	}

	return flags;
}

/*
 * Derive capabilities for a driver that does not implement the optional
 * capability callback. The set, get, adjust, and rate_adjust callbacks are
 * mandatory in the PTP clock driver API, so a non-null callback is the same
 * guarantee that ptp_clock_set(), ptp_clock_get(), ptp_clock_adjust(), and
 * ptp_clock_rate_adjust() already rely on. Rate adjustment limits stay generic
 * because a legacy driver cannot describe them. Phase adjustment is bounded by
 * the int argument accepted by ptp_clock_adjust(). Scheduled output is derived
 * separately from the optional output provider extension.
 */
static uint32_t precision_clock_ptp_legacy_cap_flags(const struct ptp_clock_driver_api *api)
{
	uint32_t flags = 0U;

	if (api == NULL) {
		return flags;
	}

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

	return flags;
}

static const struct precision_clock_api precision_clock_ptp_api = {
	.read = precision_clock_ptp_read,
	.set = precision_clock_ptp_set,
	.adjust_phase = precision_clock_ptp_adjust_phase,
	.adjust_rate = precision_clock_ptp_adjust_rate,
	.get_caps = precision_clock_ptp_get_caps,
#ifdef CONFIG_PRECISION_CLOCK_OUTPUT
	.get_output_caps = precision_clock_ptp_get_output_caps,
	.output_schedule_event = precision_clock_ptp_output_schedule_event,
	.output_start_waveform = precision_clock_ptp_output_start_waveform,
	.output_stop = precision_clock_ptp_output_stop,
	.get_output_status = precision_clock_ptp_get_output_status,
#endif /* CONFIG_PRECISION_CLOCK_OUTPUT */
};

int precision_clock_ptp_init(struct precision_clock_ptp_adapter *adapter,
			     const struct device *ptp_clock, struct precision_time_domain domain)
{
	const struct ptp_clock_driver_api *api;
	struct ptp_clock_caps ptp_caps;
	struct precision_clock_caps caps;
#ifdef CONFIG_PRECISION_CLOCK_OUTPUT
	bool output_available;
#endif
	int ret;

	CHECKIF((adapter == NULL) || (ptp_clock == NULL)) {
		return -EINVAL;
	}

	api = DEVICE_API_GET(ptp_clock, ptp_clock);

	caps = (struct precision_clock_caps){
		.flags = precision_clock_ptp_legacy_cap_flags(api),
		.resolution_ns = 1,
		.max_phase_adjust_ns = 0,
		.min_rate_ppb = 0,
		.max_rate_ppb = 0,
	};

	if ((caps.flags & PRECISION_CLOCK_CAP_ADJUST_PHASE) != 0U) {
		caps.max_phase_adjust_ns = INT_MAX;
	}

	if ((caps.flags & PRECISION_CLOCK_CAP_ADJUST_RATE) != 0U) {
		/* A legacy driver does not report its rate range, so leave the
		 * upper bound to the driver while keeping the derived ratio valid.
		 */
		caps.min_rate_ppb = PRECISION_CLOCK_PTP_MIN_RATE_PPB;
		caps.max_rate_ppb = INT32_MAX;
	}

	if ((api != NULL) && (api->get_caps != NULL)) {
		ret = api->get_caps(ptp_clock, &ptp_caps);
		if (ret == 0) {
			caps = (struct precision_clock_caps){
				.flags = precision_clock_ptp_cap_flags(ptp_caps.flags),
				.resolution_ns = ptp_caps.resolution_ns,
				.max_phase_adjust_ns = ptp_caps.max_adjust_ns,
				.min_rate_ppb = ptp_caps.min_rate_ppb,
				.max_rate_ppb = ptp_caps.max_rate_ppb,
			};
		} else if (ret != -ENOTSUP) {
			return ret;
		}
	}

#ifdef CONFIG_PRECISION_CLOCK_OUTPUT
	ret = precision_clock_ptp_output_available(ptp_clock, api != NULL ? api->output : NULL,
						   &output_available);
	if (ret < 0) {
		return ret;
	}
	if (output_available) {
		caps.flags |= PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT;
	}
#endif /* CONFIG_PRECISION_CLOCK_OUTPUT */

	adapter->ptp_clock = ptp_clock;
	adapter->caps = caps;
	adapter->clock = (struct precision_clock){
		.api = &precision_clock_ptp_api,
		.adapter = adapter,
		.domain = domain,
	};

	return 0;
}
