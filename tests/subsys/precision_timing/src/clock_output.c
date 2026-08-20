/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/precision_timing/precision_clock.h>
#include <zephyr/ztest.h>

static const struct precision_time_domain output_domain = {
	.type = PRECISION_TIME_DOMAIN_PHC,
	.id = 7,
};

struct output_fake {
	struct precision_clock clock;
	struct precision_clock_output_caps caps;
	struct precision_clock_output_status status;
	precision_time_t now_ns;
	uint32_t last_channel;
	uint32_t schedule_calls;
	uint32_t waveform_calls;
	uint32_t stop_calls;
	bool provider_configured;
	int caps_error;
	int schedule_error;
	int waveform_error;
	int stop_error;
	int status_error;
};

static int output_fake_read(const struct precision_clock *clock, struct precision_time_point *tp)
{
	struct output_fake *fake = (struct output_fake *)clock->adapter;

	tp->time = fake->now_ns;
	tp->domain = clock->domain;

	return 0;
}

static int output_fake_get_clock_caps(const struct precision_clock *clock,
				      struct precision_clock_caps *caps)
{
	ARG_UNUSED(clock);

	*caps = (struct precision_clock_caps){
		.flags = PRECISION_CLOCK_CAP_READ | PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT,
		.resolution_ns = 1,
	};

	return 0;
}

static int output_fake_get_caps(const struct precision_clock *clock, uint32_t channel,
				struct precision_clock_output_caps *caps)
{
	struct output_fake *fake = (struct output_fake *)clock->adapter;

	fake->last_channel = channel;
	if (fake->caps_error != 0) {
		return fake->caps_error;
	}

	*caps = fake->caps;

	return 0;
}

static void output_fake_mark_active(struct output_fake *fake)
{
	fake->provider_configured = true;
	fake->status.configured = true;
	fake->status.hardware_active_valid =
		(fake->caps.flags & PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE) != 0U;
	fake->status.hardware_active = true;
}

static int output_fake_schedule_event(const struct precision_clock *clock, uint32_t channel,
				      const struct precision_clock_output_event_config *config)
{
	struct output_fake *fake = (struct output_fake *)clock->adapter;

	fake->last_channel = channel;
	fake->schedule_calls++;
	if (fake->schedule_error != 0) {
		return fake->schedule_error;
	}
	if (fake->provider_configured) {
		return -EBUSY;
	}

	output_fake_mark_active(fake);
	fake->status.kind = PRECISION_CLOCK_OUTPUT_KIND_EVENT;
	fake->status.config.event = *config;

	return 0;
}

static int output_fake_start_waveform(const struct precision_clock *clock, uint32_t channel,
				      const struct precision_clock_output_waveform_config *config)
{
	struct output_fake *fake = (struct output_fake *)clock->adapter;

	fake->last_channel = channel;
	fake->waveform_calls++;
	if (fake->waveform_error != 0) {
		return fake->waveform_error;
	}
	if (fake->provider_configured) {
		return -EBUSY;
	}

	output_fake_mark_active(fake);
	fake->status.kind = PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM;
	fake->status.config.waveform = *config;

	return 0;
}

static int output_fake_stop(const struct precision_clock *clock, uint32_t channel)
{
	struct output_fake *fake = (struct output_fake *)clock->adapter;

	fake->last_channel = channel;
	if (channel >= fake->caps.channel_count) {
		return -ENOTSUP;
	}

	fake->stop_calls++;
	if (fake->stop_error != 0) {
		return fake->stop_error;
	}

	fake->provider_configured = false;
	fake->status.configured = false;

	return 0;
}

static int output_fake_get_status(const struct precision_clock *clock, uint32_t channel,
				  struct precision_clock_output_status *status)
{
	struct output_fake *fake = (struct output_fake *)clock->adapter;

	fake->last_channel = channel;
	if (fake->status_error != 0) {
		return fake->status_error;
	}

	*status = fake->status;

	return 0;
}

static const struct precision_clock_api output_fake_api = {
	.read = output_fake_read,
	.get_caps = output_fake_get_clock_caps,
	.get_output_caps = output_fake_get_caps,
	.output_schedule_event = output_fake_schedule_event,
	.output_start_waveform = output_fake_start_waveform,
	.output_stop = output_fake_stop,
	.get_output_status = output_fake_get_status,
};

static const struct precision_clock_api output_unsupported_api = {
	.read = output_fake_read,
	.get_caps = output_fake_get_clock_caps,
};

static void output_fake_init(struct output_fake *fake)
{
	*fake = (struct output_fake){0};
	fake->clock = (struct precision_clock){
		.api = &output_fake_api,
		.adapter = fake,
		.domain = output_domain,
	};
	fake->caps = (struct precision_clock_output_caps){
		.flags = PRECISION_CLOCK_OUTPUT_CAP_EVENT | PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM |
			 PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH |
			 PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING |
			 PRECISION_CLOCK_OUTPUT_CAP_EDGE_FALLING |
			 PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE,
		.channel_count = 2,
		.resolution_ns = 10,
		.min_lead_time_ns = 100,
		.min_period_ns = 100,
		.max_period_ns = 10000,
		.min_pulse_width_ns = 20,
		.max_pulse_width_ns = 5000,
	};
	fake->now_ns = 1000;
}

static struct precision_clock_output_event_config valid_event_config(void)
{
	struct precision_clock_output_event_config config = {
		.edge = PRECISION_CLOCK_OUTPUT_EDGE_RISING,
	};

	config.target_time.time = 2000;
	config.target_time.domain = output_domain;

	return config;
}

static struct precision_clock_output_waveform_config valid_waveform_config(void)
{
	struct precision_clock_output_waveform_config config = {
		.period_ns = 1000,
		.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_EXACT,
		.pulse_width_ns = 100,
	};

	config.first_rising_time.time = 2000;
	config.first_rising_time.domain = output_domain;

	return config;
}

ZTEST(precision_timing, test_clock_output_rejects_invalid_arguments)
{
	struct output_fake fake;
	struct precision_clock_output_caps caps;
	struct precision_clock_output_status status;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();
	struct precision_clock invalid_clock = {0};

	output_fake_init(&fake);
	zassert_equal(precision_clock_output_get_caps(NULL, 0, &caps), -EINVAL);
	zassert_equal(precision_clock_output_get_caps(&fake.clock, 0, NULL), -EINVAL);
	zassert_equal(precision_clock_output_get_caps(&invalid_clock, 0, &caps), -EINVAL);
	zassert_equal(precision_clock_output_schedule_event(NULL, 0, &event), -EINVAL);
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, NULL), -EINVAL);
	zassert_equal(precision_clock_output_start_waveform(NULL, 0, &waveform), -EINVAL);
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, NULL), -EINVAL);
	zassert_equal(precision_clock_output_stop(NULL, 0), -EINVAL);
	zassert_equal(precision_clock_output_get_status(NULL, 0, &status), -EINVAL);
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, NULL), -EINVAL);
}

ZTEST(precision_timing, test_clock_output_reports_unsupported_operations)
{
	struct output_fake fake;
	struct precision_clock_output_caps caps;
	struct precision_clock_output_status status;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	fake.clock.api = &output_unsupported_api;
	zassert_equal(precision_clock_output_get_caps(&fake.clock, 0, &caps), -ENOTSUP);
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ENOTSUP);
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ENOTSUP);
	zassert_equal(precision_clock_output_stop(&fake.clock, 0), -ENOTSUP);
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -ENOTSUP);
}

ZTEST(precision_timing, test_clock_output_calculates_next_aligned_start)
{
	struct precision_time_point start;
	struct precision_time_point now = {
		.time = 1999LL * NSEC_PER_MSEC,
		.domain = output_domain,
	};

	zassert_ok(
		precision_clock_output_next_start_time(&now, NSEC_PER_SEC, NSEC_PER_MSEC, &start));
	zassert_equal(start.time, 2LL * NSEC_PER_SEC);
	zassert_true(precision_time_domain_equal(&start.domain, &output_domain));

	now.time++;
	zassert_ok(
		precision_clock_output_next_start_time(&now, NSEC_PER_SEC, NSEC_PER_MSEC, &start));
	zassert_equal(start.time, 3LL * NSEC_PER_SEC);

	now.time = -1500LL * NSEC_PER_MSEC;
	zassert_ok(precision_clock_output_next_start_time(&now, NSEC_PER_SEC, 0, &start));
	zassert_equal(start.time, -(precision_time_t)NSEC_PER_SEC);
	now.time = -2LL * NSEC_PER_SEC;
	zassert_ok(precision_clock_output_next_start_time(&now, NSEC_PER_SEC, 0, &start));
	zassert_equal(start.time, now.time);

	zassert_equal(precision_clock_output_next_start_time(NULL, NSEC_PER_SEC, 0, &start),
		      -EINVAL);
	zassert_equal(precision_clock_output_next_start_time(&now, 0, 0, &start), -EINVAL);
	zassert_equal(precision_clock_output_next_start_time(&now, NSEC_PER_SEC, -1, &start),
		      -EINVAL);
	zassert_equal(precision_clock_output_next_start_time(&now, NSEC_PER_SEC, 0, NULL), -EINVAL);

	now.time = PRECISION_TIME_MAX;
	zassert_equal(precision_clock_output_next_start_time(&now, NSEC_PER_SEC, 1, &start),
		      -ERANGE);
	zassert_equal(precision_clock_output_next_start_time(&now, NSEC_PER_SEC, 0, &start),
		      -ERANGE);
}

ZTEST(precision_timing, test_clock_output_queries_per_channel_capabilities)
{
	struct output_fake fake;
	struct precision_clock_output_caps caps;

	output_fake_init(&fake);
	zassert_ok(precision_clock_output_get_caps(&fake.clock, 1, &caps));
	zassert_equal(fake.last_channel, 1);
	zassert_equal(caps.channel_count, 2);
	zassert_equal(caps.resolution_ns, 10);
	zassert_equal(precision_clock_output_get_caps(&fake.clock, 2, &caps), -ENOTSUP);

	fake.caps_error = -EIO;
	zassert_equal(precision_clock_output_get_caps(&fake.clock, 0, &caps), -EIO);
}

ZTEST(precision_timing, test_clock_output_schedules_event)
{
	struct output_fake fake;
	struct precision_clock_output_status status;
	struct precision_clock_output_event_config event = valid_event_config();

	output_fake_init(&fake);
	event.edge = PRECISION_CLOCK_OUTPUT_EDGE_FALLING;
	zassert_ok(precision_clock_output_schedule_event(&fake.clock, 1, &event));
	zassert_equal(fake.schedule_calls, 1);
	zassert_equal(fake.last_channel, 1);
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 1, &event), -EBUSY);
	zassert_equal(fake.schedule_calls, 2);

	zassert_ok(precision_clock_output_get_status(&fake.clock, 1, &status));
	zassert_true(status.configured);
	zassert_equal(status.kind, PRECISION_CLOCK_OUTPUT_KIND_EVENT);
	zassert_equal(status.config.event.target_time.time, event.target_time.time);
	zassert_equal(status.config.event.edge, PRECISION_CLOCK_OUTPUT_EDGE_FALLING);
	zassert_true(status.hardware_active_valid);
	zassert_true(status.hardware_active);
}

ZTEST(precision_timing, test_clock_output_event_only_provider)
{
	struct output_fake fake;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_EVENT | PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING;
	zassert_ok(precision_clock_output_schedule_event(&fake.clock, 0, &event));
	zassert_equal(fake.schedule_calls, 1);
	zassert_ok(precision_clock_output_stop(&fake.clock, 0));

	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ENOTSUP);
	zassert_equal(fake.waveform_calls, 0);
}

ZTEST(precision_timing, test_clock_output_rejects_unsupported_edge)
{
	struct output_fake fake;
	struct precision_clock_output_event_config event = valid_event_config();

	output_fake_init(&fake);
	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_EVENT | PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING;
	event.edge = PRECISION_CLOCK_OUTPUT_EDGE_FALLING;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ENOTSUP);

	event = valid_event_config();
	event.edge = (enum precision_clock_output_edge)9;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -EINVAL);

	event = valid_event_config();
	fake.caps.flags =
		PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM | PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ENOTSUP);
	zassert_equal(fake.schedule_calls, 0);
}

ZTEST(precision_timing, test_clock_output_starts_waveform_exact_width)
{
	struct output_fake fake;
	struct precision_clock_output_status status;
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	zassert_ok(precision_clock_output_start_waveform(&fake.clock, 1, &waveform));
	zassert_equal(fake.waveform_calls, 1);
	zassert_equal(fake.last_channel, 1);
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 1, &waveform), -EBUSY);

	zassert_ok(precision_clock_output_get_status(&fake.clock, 1, &status));
	zassert_true(status.configured);
	zassert_equal(status.kind, PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM);
	zassert_equal(status.config.waveform.first_rising_time.time,
		      waveform.first_rising_time.time);
	zassert_equal(status.config.waveform.period_ns, waveform.period_ns);
	zassert_equal(status.config.waveform.width_policy, PRECISION_CLOCK_OUTPUT_WIDTH_EXACT);
	zassert_equal(status.config.waveform.pulse_width_ns, waveform.pulse_width_ns);
}

ZTEST(precision_timing, test_clock_output_starts_waveform_provider_default_width)
{
	struct output_fake fake;
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM;
	fake.caps.min_pulse_width_ns = 0;
	fake.caps.max_pulse_width_ns = 0;
	waveform.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT;
	waveform.pulse_width_ns = 0;
	zassert_ok(precision_clock_output_start_waveform(&fake.clock, 0, &waveform));
	zassert_equal(fake.waveform_calls, 1);
	zassert_ok(precision_clock_output_stop(&fake.clock, 0));

	waveform = valid_waveform_config();
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ENOTSUP);
	zassert_equal(fake.waveform_calls, 1);
}

ZTEST(precision_timing, test_clock_output_rejects_invalid_waveform_arguments)
{
	struct output_fake fake;
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	waveform.first_rising_time.domain.id++;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -EINVAL);
	waveform = valid_waveform_config();
	waveform.width_policy = (enum precision_clock_output_width_policy)5;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -EINVAL);
	waveform = valid_waveform_config();
	waveform.period_ns = 0;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -EINVAL);
	waveform = valid_waveform_config();
	waveform.pulse_width_ns = 0;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -EINVAL);
	waveform = valid_waveform_config();
	waveform.pulse_width_ns = waveform.period_ns;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -EINVAL);
	zassert_equal(fake.waveform_calls, 0);
}

ZTEST(precision_timing, test_clock_output_validates_waveform_limits_and_resolution)
{
	struct output_fake fake;
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	waveform.period_ns = fake.caps.min_period_ns - fake.caps.resolution_ns;
	waveform.pulse_width_ns = fake.caps.min_pulse_width_ns;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	waveform = valid_waveform_config();
	waveform.period_ns = fake.caps.max_period_ns + fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	waveform = valid_waveform_config();
	waveform.pulse_width_ns = fake.caps.min_pulse_width_ns - fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	waveform = valid_waveform_config();
	waveform.period_ns = 6000;
	waveform.pulse_width_ns = fake.caps.max_pulse_width_ns + fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);

	waveform = valid_waveform_config();
	waveform.first_rising_time.time++;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	waveform = valid_waveform_config();
	waveform.period_ns++;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	waveform = valid_waveform_config();
	waveform.pulse_width_ns++;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	zassert_equal(fake.waveform_calls, 0);
}

ZTEST(precision_timing, test_clock_output_rejects_malformed_provider_caps)
{
	struct output_fake fake;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	fake.caps.resolution_ns = 0;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ERANGE);
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	fake.caps.resolution_ns = 10;
	fake.caps.min_lead_time_ns = -1;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ERANGE);
	fake.caps.min_lead_time_ns = 100;
	fake.caps.max_period_ns = fake.caps.min_period_ns - 1;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
	fake.caps.max_period_ns = 10000;
	fake.caps.max_pulse_width_ns = fake.caps.min_pulse_width_ns - 1;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
}

ZTEST(precision_timing, test_clock_output_enforces_minimum_lead_time)
{
	struct output_fake fake;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	event.target_time.time = fake.now_ns - fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ETIME);
	event.target_time.time = fake.now_ns + fake.caps.min_lead_time_ns - fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ETIME);
	event.target_time.time = fake.now_ns + fake.caps.min_lead_time_ns;
	zassert_ok(precision_clock_output_schedule_event(&fake.clock, 0, &event));

	output_fake_init(&fake);
	waveform.first_rising_time.time =
		fake.now_ns + fake.caps.min_lead_time_ns - fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ETIME);
	waveform.first_rising_time.time = fake.now_ns + fake.caps.min_lead_time_ns;
	zassert_ok(precision_clock_output_start_waveform(&fake.clock, 0, &waveform));

	output_fake_init(&fake);
	fake.now_ns = PRECISION_TIME_MAX - 57;
	event.target_time.time = PRECISION_TIME_MAX - 7;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -ERANGE);
	waveform = valid_waveform_config();
	waveform.first_rising_time.time = PRECISION_TIME_MAX - 7;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -ERANGE);
}

ZTEST(precision_timing, test_clock_output_validates_status_consistency)
{
	struct output_fake fake;
	struct precision_clock_output_status status;

	output_fake_init(&fake);
	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM;
	fake.status.configured = true;
	fake.status.kind = PRECISION_CLOCK_OUTPUT_KIND_EVENT;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	output_fake_init(&fake);
	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_EVENT;
	fake.status.configured = false;
	fake.status.hardware_active_valid = true;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	output_fake_init(&fake);
	fake.status.configured = true;
	fake.status.kind = (enum precision_clock_output_kind)7;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	output_fake_init(&fake);
	fake.status.configured = false;
	fake.status.hardware_active_valid = true;
	fake.status.hardware_active = false;
	zassert_ok(precision_clock_output_get_status(&fake.clock, 0, &status));
	zassert_false(status.configured);
	zassert_true(status.hardware_active_valid);
	zassert_false(status.hardware_active);
}

ZTEST(precision_timing, test_clock_output_validates_event_status_configuration)
{
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_status status;
	struct output_fake fake;

	output_fake_init(&fake);
	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_EVENT | PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING;
	fake.now_ns = event.target_time.time + NSEC_PER_SEC;
	fake.status.configured = true;
	fake.status.kind = PRECISION_CLOCK_OUTPUT_KIND_EVENT;
	fake.status.config.event = event;
	zassert_ok(precision_clock_output_get_status(&fake.clock, 0, &status));

	fake.status.config.event.edge = PRECISION_CLOCK_OUTPUT_EDGE_FALLING;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.event = event;
	fake.status.config.event.edge = (enum precision_clock_output_edge)7;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.event = event;
	fake.status.config.event.target_time.time++;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.event = event;
	fake.status.config.event.target_time.domain.id++;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);
}

ZTEST(precision_timing, test_clock_output_validates_waveform_status_configuration)
{
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();
	struct precision_clock_output_status status;
	struct output_fake fake;

	output_fake_init(&fake);
	fake.now_ns = waveform.first_rising_time.time + NSEC_PER_SEC;
	fake.status.configured = true;
	fake.status.kind = PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM;
	fake.status.config.waveform = waveform;
	zassert_ok(precision_clock_output_get_status(&fake.clock, 0, &status));

	fake.status.config.waveform.period_ns = 0;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.period_ns++;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.period_ns = fake.caps.min_period_ns - fake.caps.resolution_ns;
	fake.status.config.waveform.pulse_width_ns = fake.caps.min_pulse_width_ns;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.period_ns = fake.caps.max_period_ns + fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.width_policy = (enum precision_clock_output_width_policy)7;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.caps.flags &= ~PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.caps.flags |= PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH;
	fake.status.config.waveform.pulse_width_ns = waveform.period_ns;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.pulse_width_ns =
		fake.caps.min_pulse_width_ns - fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.period_ns = 6000;
	fake.status.config.waveform.pulse_width_ns =
		fake.caps.max_pulse_width_ns + fake.caps.resolution_ns;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.pulse_width_ns++;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.first_rising_time.time++;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.status.config.waveform = waveform;
	fake.status.config.waveform.first_rising_time.domain.id++;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EINVAL);

	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM;
	fake.caps.min_pulse_width_ns = 0;
	fake.caps.max_pulse_width_ns = 0;
	fake.status.config.waveform = waveform;
	fake.status.config.waveform.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT;
	fake.status.config.waveform.pulse_width_ns = 0;
	zassert_ok(precision_clock_output_get_status(&fake.clock, 0, &status));
}

ZTEST(precision_timing, test_clock_output_stop_and_propagates_errors)
{
	struct output_fake fake;
	struct precision_clock_output_status status;
	struct precision_clock_output_event_config event = valid_event_config();
	struct precision_clock_output_waveform_config waveform = valid_waveform_config();

	output_fake_init(&fake);
	zassert_ok(precision_clock_output_stop(&fake.clock, 0));
	zassert_equal(fake.stop_calls, 1);
	zassert_equal(precision_clock_output_stop(&fake.clock, 2), -ENOTSUP);
	zassert_equal(precision_clock_output_get_status(&fake.clock, 2, &status), -ENOTSUP);

	fake.schedule_error = -EACCES;
	zassert_equal(precision_clock_output_schedule_event(&fake.clock, 0, &event), -EACCES);
	fake.waveform_error = -EIO;
	zassert_equal(precision_clock_output_start_waveform(&fake.clock, 0, &waveform), -EIO);
	fake.stop_error = -EBUSY;
	zassert_equal(precision_clock_output_stop(&fake.clock, 0), -EBUSY);
	fake.status_error = -EAGAIN;
	zassert_equal(precision_clock_output_get_status(&fake.clock, 0, &status), -EAGAIN);
}
