/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/precision_clock_output.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/precision_timing/precision_clock.h>
#include <zephyr/precision_timing/precision_clock_ptp.h>
#include <zephyr/ztest.h>

static const struct precision_time_domain output_domain = {
	.type = PRECISION_TIME_DOMAIN_PHC,
	.id = 7,
};

struct ptp_output_fake {
	struct precision_clock_output_caps caps;
	struct precision_clock_output_raw_status status;
	struct precision_clock_output_raw_event_config event_config;
	struct precision_clock_output_raw_waveform_config waveform_config;
	struct net_ptp_time now;
	uint32_t last_channel;
	uint32_t schedule_calls;
	uint32_t waveform_calls;
	uint32_t stop_calls;
	int caps_error;
	int schedule_error;
	int waveform_error;
	int stop_error;
	int status_error;
};

static struct ptp_output_fake ptp_output_fake;

static void ptp_output_fake_mark_active(void)
{
	ptp_output_fake.status.configured = true;
	ptp_output_fake.status.hardware_active_valid =
		(ptp_output_fake.caps.flags & PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE) != 0U;
	ptp_output_fake.status.hardware_active = true;
}

static int ptp_output_fake_get(const struct device *dev, struct net_ptp_time *tm)
{
	ARG_UNUSED(dev);

	*tm = ptp_output_fake.now;

	return 0;
}

static int ptp_output_fake_get_clock_caps(const struct device *dev, struct ptp_clock_caps *caps)
{
	ARG_UNUSED(dev);

	*caps = (struct ptp_clock_caps){
		.flags = PTP_CLOCK_CAP_READ,
		.resolution_ns = 1,
	};

	return 0;
}

static int ptp_output_fake_get_caps(const struct device *dev, uint32_t channel,
				    struct precision_clock_output_caps *caps)
{
	ARG_UNUSED(dev);

	ptp_output_fake.last_channel = channel;
	if (ptp_output_fake.caps_error != 0) {
		return ptp_output_fake.caps_error;
	}

	*caps = ptp_output_fake.caps;

	return 0;
}

static int ptp_output_fake_schedule_event(const struct device *dev, uint32_t channel,
					  const struct precision_clock_output_raw_event_config *cfg)
{
	ARG_UNUSED(dev);

	ptp_output_fake.last_channel = channel;
	ptp_output_fake.schedule_calls++;
	if (ptp_output_fake.schedule_error != 0) {
		return ptp_output_fake.schedule_error;
	}

	ptp_output_fake.event_config = *cfg;
	ptp_output_fake_mark_active();
	ptp_output_fake.status.kind = PRECISION_CLOCK_OUTPUT_KIND_EVENT;
	ptp_output_fake.status.config.event = *cfg;

	return 0;
}

static int
ptp_output_fake_start_waveform(const struct device *dev, uint32_t channel,
			       const struct precision_clock_output_raw_waveform_config *cfg)
{
	ARG_UNUSED(dev);

	ptp_output_fake.last_channel = channel;
	ptp_output_fake.waveform_calls++;
	if (ptp_output_fake.waveform_error != 0) {
		return ptp_output_fake.waveform_error;
	}

	ptp_output_fake.waveform_config = *cfg;
	ptp_output_fake_mark_active();
	ptp_output_fake.status.kind = PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM;
	ptp_output_fake.status.config.waveform = *cfg;

	return 0;
}

static int ptp_output_fake_stop(const struct device *dev, uint32_t channel)
{
	ARG_UNUSED(dev);

	ptp_output_fake.last_channel = channel;
	ptp_output_fake.stop_calls++;
	if (ptp_output_fake.stop_error != 0) {
		return ptp_output_fake.stop_error;
	}

	ptp_output_fake.status.configured = false;

	return 0;
}

static int ptp_output_fake_get_status(const struct device *dev, uint32_t channel,
				      struct precision_clock_output_raw_status *status)
{
	ARG_UNUSED(dev);

	ptp_output_fake.last_channel = channel;
	if (ptp_output_fake.status_error != 0) {
		return ptp_output_fake.status_error;
	}

	*status = ptp_output_fake.status;

	return 0;
}

static const struct precision_clock_output_provider ptp_output_full_provider = {
	.get_caps = ptp_output_fake_get_caps,
	.schedule_event = ptp_output_fake_schedule_event,
	.start_waveform = ptp_output_fake_start_waveform,
	.stop = ptp_output_fake_stop,
	.get_status = ptp_output_fake_get_status,
};

static const struct precision_clock_output_provider ptp_output_event_only_provider = {
	.get_caps = ptp_output_fake_get_caps,
	.schedule_event = ptp_output_fake_schedule_event,
	.stop = ptp_output_fake_stop,
	.get_status = ptp_output_fake_get_status,
};

static const struct precision_clock_output_provider ptp_output_waveform_only_provider = {
	.get_caps = ptp_output_fake_get_caps,
	.start_waveform = ptp_output_fake_start_waveform,
	.stop = ptp_output_fake_stop,
	.get_status = ptp_output_fake_get_status,
};

static const struct precision_clock_output_provider ptp_output_no_modes_provider = {
	.get_caps = ptp_output_fake_get_caps,
	.stop = ptp_output_fake_stop,
	.get_status = ptp_output_fake_get_status,
};

static const struct precision_clock_output_provider ptp_output_missing_caps_provider = {
	.schedule_event = ptp_output_fake_schedule_event,
	.start_waveform = ptp_output_fake_start_waveform,
	.stop = ptp_output_fake_stop,
	.get_status = ptp_output_fake_get_status,
};

static const struct precision_clock_output_provider ptp_output_missing_stop_provider = {
	.get_caps = ptp_output_fake_get_caps,
	.schedule_event = ptp_output_fake_schedule_event,
	.start_waveform = ptp_output_fake_start_waveform,
	.get_status = ptp_output_fake_get_status,
};

static const struct precision_clock_output_provider ptp_output_missing_status_provider = {
	.get_caps = ptp_output_fake_get_caps,
	.schedule_event = ptp_output_fake_schedule_event,
	.start_waveform = ptp_output_fake_start_waveform,
	.stop = ptp_output_fake_stop,
};

static DEVICE_API(ptp_clock, ptp_output_full_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
	.output = &ptp_output_full_provider,
};

DEVICE_DEFINE(ptp_output_full, "ptp_output_full", NULL, NULL, NULL, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_full_api);

static DEVICE_API(ptp_clock, ptp_output_event_only_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
	.output = &ptp_output_event_only_provider,
};

DEVICE_DEFINE(ptp_output_event_only, "ptp_output_event_only", NULL, NULL, NULL, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_event_only_api);

static DEVICE_API(ptp_clock, ptp_output_waveform_only_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
	.output = &ptp_output_waveform_only_provider,
};

DEVICE_DEFINE(ptp_output_waveform_only, "ptp_output_waveform_only", NULL, NULL, NULL, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_waveform_only_api);

static DEVICE_API(ptp_clock, ptp_output_no_extension_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
};

DEVICE_DEFINE(ptp_output_no_extension, "ptp_output_no_extension", NULL, NULL, NULL, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_no_extension_api);

static DEVICE_API(ptp_clock, ptp_output_no_modes_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
	.output = &ptp_output_no_modes_provider,
};

DEVICE_DEFINE(ptp_output_no_modes, "ptp_output_no_modes", NULL, NULL, NULL, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_no_modes_api);

static DEVICE_API(ptp_clock, ptp_output_missing_caps_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
	.output = &ptp_output_missing_caps_provider,
};

DEVICE_DEFINE(ptp_output_missing_caps, "ptp_output_missing_caps", NULL, NULL, NULL, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_missing_caps_api);

static DEVICE_API(ptp_clock, ptp_output_missing_stop_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
	.output = &ptp_output_missing_stop_provider,
};

DEVICE_DEFINE(ptp_output_missing_stop, "ptp_output_missing_stop", NULL, NULL, NULL, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_missing_stop_api);

static DEVICE_API(ptp_clock, ptp_output_missing_status_api) = {
	.get = ptp_output_fake_get,
	.get_caps = ptp_output_fake_get_clock_caps,
	.output = &ptp_output_missing_status_provider,
};

DEVICE_DEFINE(ptp_output_missing_status, "ptp_output_missing_status", NULL, NULL, NULL, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ptp_output_missing_status_api);

static void ptp_output_fake_init(void)
{
	ptp_output_fake = (struct ptp_output_fake){0};
	ptp_output_fake.caps = (struct precision_clock_output_caps){
		.flags = PRECISION_CLOCK_OUTPUT_CAP_EVENT | PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM |
			 PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH |
			 PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING |
			 PRECISION_CLOCK_OUTPUT_CAP_EDGE_FALLING |
			 PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE,
		.channel_count = 2,
		.resolution_ns = 10,
		.min_lead_time_ns = 100,
		.min_period_ns = 100,
		.max_period_ns = 2000000000LL,
		.min_pulse_width_ns = 20,
		.max_pulse_width_ns = 1000000000LL,
	};
	ptp_output_fake.now = (struct net_ptp_time){
		.second = 1,
		.nanosecond = 20,
	};
}

static struct precision_clock_output_event_config valid_event_config(void)
{
	struct precision_clock_output_event_config config = {
		.edge = PRECISION_CLOCK_OUTPUT_EDGE_RISING,
	};

	config.target_time.time = 3LL * NSEC_PER_SEC + 40;
	config.target_time.domain = output_domain;

	return config;
}

static struct precision_clock_output_waveform_config valid_waveform_config(void)
{
	struct precision_clock_output_waveform_config config = {
		.period_ns = NSEC_PER_SEC,
		.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_EXACT,
		.pulse_width_ns = 100 * NSEC_PER_MSEC,
	};

	config.first_rising_time.time = 3LL * NSEC_PER_SEC + 40;
	config.first_rising_time.domain = output_domain;

	return config;
}

ZTEST(precision_timing, test_ptp_clock_output_discovers_and_converts)
{
	struct precision_clock_ptp_adapter adapter;
	struct precision_clock_output_caps caps;
	struct precision_clock_output_status status;
	struct precision_clock_caps clock_caps;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();
	const struct precision_clock *clock;

	ptp_output_fake_init();
	event.edge = PRECISION_CLOCK_OUTPUT_EDGE_FALLING;
	zassert_ok(precision_clock_ptp_init(&adapter, DEVICE_GET(ptp_output_full), output_domain));
	clock = precision_clock_ptp_get(&adapter);

	zassert_ok(precision_clock_get_caps(clock, &clock_caps));
	zassert_true(clock_caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT);

	zassert_ok(precision_clock_output_get_caps(clock, 1, &caps));
	zassert_equal(ptp_output_fake.last_channel, 1);
	zassert_equal(caps.flags, ptp_output_fake.caps.flags);
	zassert_equal(caps.channel_count, ptp_output_fake.caps.channel_count);
	zassert_equal(caps.resolution_ns, ptp_output_fake.caps.resolution_ns);
	zassert_equal(caps.min_lead_time_ns, ptp_output_fake.caps.min_lead_time_ns);
	zassert_equal(caps.max_period_ns, ptp_output_fake.caps.max_period_ns);

	zassert_ok(precision_clock_output_schedule_event(clock, 1, &event));
	zassert_equal(ptp_output_fake.schedule_calls, 1);
	zassert_equal(ptp_output_fake.last_channel, 1);
	zassert_equal(ptp_output_fake.event_config.target_time, 3LL * NSEC_PER_SEC + 40);
	zassert_equal(ptp_output_fake.event_config.edge, PRECISION_CLOCK_OUTPUT_EDGE_FALLING);

	zassert_ok(precision_clock_output_get_status(clock, 1, &status));
	zassert_true(status.configured);
	zassert_equal(status.kind, PRECISION_CLOCK_OUTPUT_KIND_EVENT);
	zassert_true(precision_time_domain_equal(&status.config.event.target_time.domain,
						 &output_domain));
	zassert_equal(status.config.event.target_time.time, 3LL * NSEC_PER_SEC + 40);
	zassert_equal(status.config.event.edge, PRECISION_CLOCK_OUTPUT_EDGE_FALLING);
	zassert_true(status.hardware_active_valid);
	zassert_true(status.hardware_active);

	zassert_ok(precision_clock_output_stop(clock, 1));
	zassert_equal(ptp_output_fake.stop_calls, 1);

	zassert_ok(precision_clock_output_start_waveform(clock, 0, &waveform));
	zassert_equal(ptp_output_fake.waveform_calls, 1);
	zassert_equal(ptp_output_fake.waveform_config.first_rising_time, 3LL * NSEC_PER_SEC + 40);
	zassert_equal(ptp_output_fake.waveform_config.period_ns, NSEC_PER_SEC);
	zassert_equal(ptp_output_fake.waveform_config.width_policy,
		      PRECISION_CLOCK_OUTPUT_WIDTH_EXACT);
	zassert_equal(ptp_output_fake.waveform_config.pulse_width_ns, 100 * NSEC_PER_MSEC);

	zassert_ok(precision_clock_output_get_status(clock, 0, &status));
	zassert_equal(status.kind, PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM);
	zassert_true(precision_time_domain_equal(&status.config.waveform.first_rising_time.domain,
						 &output_domain));
	zassert_equal(status.config.waveform.first_rising_time.time, 3LL * NSEC_PER_SEC + 40);
	zassert_equal(status.config.waveform.period_ns, NSEC_PER_SEC);
	zassert_equal(status.config.waveform.pulse_width_ns, 100 * NSEC_PER_MSEC);
}

ZTEST(precision_timing, test_ptp_clock_output_event_only_provider)
{
	struct precision_clock_ptp_adapter adapter;
	struct precision_clock_caps clock_caps;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();
	const struct precision_clock *clock;

	ptp_output_fake_init();
	ptp_output_fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_EVENT |
				     PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING |
				     PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE;
	zassert_ok(precision_clock_ptp_init(&adapter, DEVICE_GET(ptp_output_event_only),
					    output_domain));
	clock = precision_clock_ptp_get(&adapter);

	zassert_ok(precision_clock_get_caps(clock, &clock_caps));
	zassert_true(clock_caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT);

	zassert_ok(precision_clock_output_schedule_event(clock, 0, &event));
	zassert_equal(ptp_output_fake.schedule_calls, 1);
	zassert_equal(precision_clock_output_start_waveform(clock, 0, &waveform), -ENOTSUP);
	zassert_equal(ptp_output_fake.waveform_calls, 0);
}

ZTEST(precision_timing, test_ptp_clock_output_waveform_only_provider)
{
	struct precision_clock_ptp_adapter adapter;
	struct precision_clock_caps clock_caps;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();
	const struct precision_clock *clock;

	ptp_output_fake_init();
	ptp_output_fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM |
				     PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH |
				     PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE;
	zassert_ok(precision_clock_ptp_init(&adapter, DEVICE_GET(ptp_output_waveform_only),
					    output_domain));
	clock = precision_clock_ptp_get(&adapter);

	zassert_ok(precision_clock_get_caps(clock, &clock_caps));
	zassert_true(clock_caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT);

	zassert_ok(precision_clock_output_start_waveform(clock, 0, &waveform));
	zassert_equal(ptp_output_fake.waveform_calls, 1);
	zassert_equal(precision_clock_output_schedule_event(clock, 0, &event), -ENOTSUP);
	zassert_equal(ptp_output_fake.schedule_calls, 0);
}

ZTEST(precision_timing, test_ptp_clock_output_propagates_errors)
{
	struct precision_clock_ptp_adapter adapter;
	struct precision_clock_output_caps caps;
	struct precision_clock_output_status status;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();
	const struct precision_clock *clock;

	ptp_output_fake_init();
	zassert_ok(precision_clock_ptp_init(&adapter, DEVICE_GET(ptp_output_full), output_domain));
	clock = precision_clock_ptp_get(&adapter);

	ptp_output_fake.caps_error = -EIO;
	zassert_equal(precision_clock_output_get_caps(clock, 0, &caps), -EIO);
	ptp_output_fake.caps_error = 0;

	ptp_output_fake.schedule_error = -EACCES;
	zassert_equal(precision_clock_output_schedule_event(clock, 0, &event), -EACCES);
	ptp_output_fake.schedule_error = 0;

	ptp_output_fake.waveform_error = -ENOSPC;
	zassert_equal(precision_clock_output_start_waveform(clock, 0, &waveform), -ENOSPC);
	ptp_output_fake.waveform_error = 0;

	ptp_output_fake.stop_error = -EIO;
	zassert_equal(precision_clock_output_stop(clock, 0), -EIO);
	ptp_output_fake.stop_error = 0;

	ptp_output_fake.status_error = -EAGAIN;
	zassert_equal(precision_clock_output_get_status(clock, 0, &status), -EAGAIN);
}

ZTEST(precision_timing, test_ptp_clock_output_requires_usable_channel)
{
	struct precision_clock_ptp_adapter adapter;
	struct precision_clock_output_caps output_caps;
	struct precision_clock_caps caps;
	const struct precision_clock *clock;

	ptp_output_fake_init();
	ptp_output_fake.caps.channel_count = 0U;
	zassert_ok(precision_clock_ptp_init(&adapter, DEVICE_GET(ptp_output_full), output_domain));
	clock = precision_clock_ptp_get(&adapter);
	zassert_ok(precision_clock_get_caps(clock, &caps));
	zassert_false(caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT);
	zassert_equal(precision_clock_output_get_caps(clock, 0, &output_caps), -ENOTSUP);

	ptp_output_fake_init();
	ptp_output_fake.caps_error = -EIO;
	zassert_equal(
		precision_clock_ptp_init(&adapter, DEVICE_GET(ptp_output_full), output_domain),
		-EIO);
}

static void assert_no_scheduled_output(const struct device *dev)
{
	struct precision_clock_ptp_adapter adapter;
	struct precision_clock_output_caps output_caps;
	struct precision_clock_output_status status;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();
	struct precision_clock_caps caps;
	const struct precision_clock *clock;

	zassert_ok(precision_clock_ptp_init(&adapter, dev, output_domain));
	clock = precision_clock_ptp_get(&adapter);
	zassert_ok(precision_clock_get_caps(clock, &caps));
	zassert_false(caps.flags & PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT);
	zassert_equal(precision_clock_output_get_caps(clock, 0, &output_caps), -ENOTSUP);
	zassert_equal(precision_clock_output_schedule_event(clock, 0, &event), -ENOTSUP);
	zassert_equal(precision_clock_output_start_waveform(clock, 0, &waveform), -ENOTSUP);
	zassert_equal(precision_clock_output_stop(clock, 0), -ENOTSUP);
	zassert_equal(precision_clock_output_get_status(clock, 0, &status), -ENOTSUP);
}

ZTEST(precision_timing, test_ptp_clock_output_requires_common_callbacks)
{
	ptp_output_fake_init();
	assert_no_scheduled_output(DEVICE_GET(ptp_output_no_extension));
	assert_no_scheduled_output(DEVICE_GET(ptp_output_no_modes));
	assert_no_scheduled_output(DEVICE_GET(ptp_output_missing_caps));
	assert_no_scheduled_output(DEVICE_GET(ptp_output_missing_stop));
	assert_no_scheduled_output(DEVICE_GET(ptp_output_missing_status));
}
