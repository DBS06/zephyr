/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/precision_timing/precision_clock_sync.h>
#include <zephyr/ztest.h>

#include "precision_clock_sync_internal.h"

#define SYNC_FAKE_MAX_READS 16U

static const struct precision_time_domain sync_source_domain = {
	.type = PRECISION_TIME_DOMAIN_PHC,
	.id = 10,
};

static const struct precision_time_domain sync_sink_domain = {
	.type = PRECISION_TIME_DOMAIN_PHC,
	.id = 11,
};

struct sync_fake_clock {
	struct precision_clock clock;
	struct precision_clock_caps caps;
	precision_time_t read_values[SYNC_FAKE_MAX_READS];
	int read_errors[SYNC_FAKE_MAX_READS];
	precision_time_t fallback_time;
	size_t read_count;
	size_t read_index;
	uint32_t read_calls;
	uint32_t set_calls;
	uint32_t phase_calls;
	uint32_t rate_calls;
	precision_time_t last_set_time;
	precision_time_t last_phase_ns;
	int32_t last_rate_ppb;
	int get_caps_error;
	int set_error;
	int phase_error;
	int rate_error;
	char control_operations[8];
	uint8_t control_operation_count;
};

static struct sync_fake_clock *sync_fake_from_clock(const struct precision_clock *clock)
{
	return (struct sync_fake_clock *)clock->adapter;
}

static void sync_fake_record_control(struct sync_fake_clock *fake, char operation)
{
	if (fake->control_operation_count < ARRAY_SIZE(fake->control_operations)) {
		fake->control_operations[fake->control_operation_count++] = operation;
	}
}

static int sync_fake_read(const struct precision_clock *clock, struct precision_time_point *tp)
{
	struct sync_fake_clock *fake = sync_fake_from_clock(clock);
	size_t index = fake->read_index++;
	int ret = 0;

	fake->read_calls++;
	if (index < fake->read_count) {
		ret = fake->read_errors[index];
		if (ret == 0) {
			tp->time = fake->read_values[index];
		}
	} else {
		tp->time = fake->fallback_time;
	}

	if (ret == 0) {
		tp->domain = clock->domain;
	}

	return ret;
}

static int sync_fake_set(const struct precision_clock *clock, const struct precision_time_point *tp)
{
	struct sync_fake_clock *fake = sync_fake_from_clock(clock);

	fake->set_calls++;
	sync_fake_record_control(fake, 'S');
	if (fake->set_error < 0) {
		return fake->set_error;
	}

	fake->last_set_time = tp->time;
	fake->fallback_time = tp->time;

	return 0;
}

static int sync_fake_adjust_phase(const struct precision_clock *clock, precision_time_t phase_ns)
{
	struct sync_fake_clock *fake = sync_fake_from_clock(clock);

	fake->phase_calls++;
	sync_fake_record_control(fake, 'P');
	if (fake->phase_error < 0) {
		return fake->phase_error;
	}

	fake->last_phase_ns = phase_ns;
	(void)precision_time_add(fake->fallback_time, phase_ns, &fake->fallback_time);

	return 0;
}

static int sync_fake_adjust_rate(const struct precision_clock *clock, int32_t rate_ppb)
{
	struct sync_fake_clock *fake = sync_fake_from_clock(clock);

	fake->rate_calls++;
	sync_fake_record_control(fake, 'R');
	if (fake->rate_error < 0) {
		return fake->rate_error;
	}

	fake->last_rate_ppb = rate_ppb;

	return 0;
}

static int sync_fake_get_caps(const struct precision_clock *clock,
			      struct precision_clock_caps *caps)
{
	struct sync_fake_clock *fake = sync_fake_from_clock(clock);

	if (fake->get_caps_error < 0) {
		return fake->get_caps_error;
	}

	*caps = fake->caps;

	return 0;
}

static const struct precision_clock_api sync_fake_clock_api = {
	.read = sync_fake_read,
	.set = sync_fake_set,
	.adjust_phase = sync_fake_adjust_phase,
	.adjust_rate = sync_fake_adjust_rate,
	.get_caps = sync_fake_get_caps,
};

static void sync_fake_init(struct sync_fake_clock *fake, struct precision_time_domain domain,
			   bool sink)
{
	memset(fake, 0, sizeof(*fake));
	fake->clock = (struct precision_clock){
		.api = &sync_fake_clock_api,
		.adapter = fake,
		.domain = domain,
	};
	fake->caps = (struct precision_clock_caps){
		.flags = sink ? PRECISION_CLOCK_CAP_READ | PRECISION_CLOCK_CAP_SET |
					 PRECISION_CLOCK_CAP_ADJUST_PHASE |
					 PRECISION_CLOCK_CAP_ADJUST_RATE
			      : PRECISION_CLOCK_CAP_READ,
		.resolution_ns = 1,
		.max_phase_adjust_ns = PRECISION_TIME_MAX,
		.min_rate_ppb = -1000000000,
		.max_rate_ppb = 1000000000,
	};
}

static void sync_fake_queue_reads(struct sync_fake_clock *fake, const precision_time_t *values,
				  const int *errors, size_t count)
{
	zassert_true(count <= SYNC_FAKE_MAX_READS);
	fake->read_count = count;
	fake->read_index = 0U;
	memset(fake->read_values, 0, sizeof(fake->read_values));
	memset(fake->read_errors, 0, sizeof(fake->read_errors));

	for (size_t i = 0U; i < count; i++) {
		fake->read_values[i] = values != NULL ? values[i] : 0;
		fake->read_errors[i] = errors != NULL ? errors[i] : 0;
	}
}

static uint32_t sync_fake_start(struct precision_clock_sync *sync,
				const struct precision_clock_sync_config *config)
{
	zassert_ok(precision_clock_sync_init(sync, config));
	zassert_ok(precision_clock_sync_start(sync));

	return sync->generation;
}

ZTEST(precision_timing, test_clock_sync_defaults_and_preflight_capabilities)
{
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync sync = {0};

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);

	zassert_equal(precision_clock_sync_config_default(NULL, &source.clock, &sink.clock),
		      -EINVAL);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	zassert_equal(config.update_interval_ns, NSEC_PER_SEC);
	zassert_equal(config.readings_per_update, 5U);
	zassert_equal(config.pi.step_threshold_ns, NSEC_PER_SEC);
	zassert_equal(config.pi.lock_threshold_ns, 10 * NSEC_PER_MSEC);
	zassert_equal(config.pi.outlier_threshold_ns, 100 * NSEC_PER_MSEC);
	zassert_equal(config.pi.max_uncertainty_ns, 10 * NSEC_PER_MSEC + 2);
	zassert_equal(config.pi.source_timeout_ns, 3 * NSEC_PER_SEC);
	zassert_equal(config.pi.holdover_ns, 3 * NSEC_PER_SEC);
	zassert_equal(config.pi.lock_sample_count, 3U);
	zassert_equal(config.pi.outlier_sample_count, 2U);
	zassert_equal(config.pi.kp_num, CONFIG_PRECISION_TIMING_PI_KP);
	zassert_equal(config.pi.ki_num, CONFIG_PRECISION_TIMING_PI_KI);
	zassert_equal(config.pi.gain_den, PRECISION_PI_GAIN_DEN);

	config.pi.max_uncertainty_ns = 1;
	zassert_equal(precision_clock_sync_init(&sync, &config), -ERANGE);
	config.pi.max_uncertainty_ns = 10 * NSEC_PER_MSEC + 2;

	config.sink = config.source;
	zassert_equal(precision_clock_sync_init(&sync, &config), -EINVAL);
	config.sink = &sink.clock;

	sink.clock.domain = source.clock.domain;
	config.pi.local_domain = sink.clock.domain;
	zassert_equal(precision_clock_sync_init(&sync, &config), -EINVAL);
	sink.clock.domain = sync_sink_domain;
	config.pi.local_domain = sync_sink_domain;

	source.caps.flags = 0U;
	zassert_equal(precision_clock_sync_init(&sync, &config), -ENOTSUP);
	source.caps.flags = PRECISION_CLOCK_CAP_READ;

	sink.caps.flags = PRECISION_CLOCK_CAP_READ | PRECISION_CLOCK_CAP_ADJUST_RATE;
	zassert_equal(precision_clock_sync_init(&sync, &config), -ENOTSUP);
	config.pi.step_threshold_ns = 0;

	config.pi.min_rate_ppb = 100;
	config.pi.max_rate_ppb = 200;
	sink.caps.min_rate_ppb = -50;
	sink.caps.max_rate_ppb = 50;
	zassert_equal(precision_clock_sync_init(&sync, &config), -ERANGE);
	config.pi.min_rate_ppb = INT32_MIN;
	config.pi.max_rate_ppb = INT32_MAX;
	zassert_ok(precision_clock_sync_init(&sync, &config));
}

ZTEST(precision_timing, test_clock_sync_selects_fastest_bracket_and_midpoint)
{
	static const precision_time_t source_reads[] = {1100, 1100, 1100, 1100, 1100};
	static const precision_time_t sink_reads[] = {
		900, 1100, 950, 1050, 990, 1010, 800, 1200, 980, 1020,
	};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	source.caps.resolution_ns = 3;
	sink.caps.resolution_ns = 2;
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	generation = sync_fake_start(&sync, &config);

	zassert_ok(precision_clock_sync_run_once(&sync, generation));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(source.read_calls, 5U);
	zassert_equal(sink.read_calls, 10U);
	zassert_equal(status.sampling_uncertainty_ns, 15);
	zassert_equal(status.offset_ns, 100);
	zassert_equal(status.applied_rate_ppb, 100);
	zassert_equal(status.accepted_observations, 1U);
	zassert_equal(status.rejected_observations, 0U);
	zassert_equal(status.source_age_ns, 0);
	zassert_equal(status.state, PRECISION_SYNC_ACQUIRING);
}

ZTEST(precision_timing, test_clock_sync_intersects_rate_limits_and_offset_sign)
{
	static const precision_time_t source_reads[] = {900};
	static const precision_time_t sink_reads[] = {1000, 1000};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	sink.caps.min_rate_ppb = -25;
	sink.caps.max_rate_ppb = 25;
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	config.pi.target_offset_ns = -100;
	config.pi.min_rate_ppb = -100;
	config.pi.max_rate_ppb = 100;
	generation = sync_fake_start(&sync, &config);

	zassert_equal(sync.config.pi.min_rate_ppb, -25);
	zassert_equal(sync.config.pi.max_rate_ppb, 25);
	zassert_ok(precision_clock_sync_run_once(&sync, generation));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.offset_ns, -200);
	zassert_equal(status.applied_rate_ppb, -25);
	zassert_equal(sink.last_rate_ppb, -25);
}

ZTEST(precision_timing, test_clock_sync_converging_samples_reach_lock)
{
	static const precision_time_t offsets[] = {100, 8, 5, 2};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	config.pi.lock_threshold_ns = 10;
	config.pi.lock_sample_count = 3U;
	config.pi.outlier_threshold_ns = 0;
	config.pi.max_uncertainty_ns = 0;
	generation = sync_fake_start(&sync, &config);

	for (size_t i = 0U; i < ARRAY_SIZE(offsets); i++) {
		precision_time_t source_value = 1000 + (precision_time_t)i * 1000 + offsets[i];
		precision_time_t sink_values[] = {
			1000 + (precision_time_t)i * 1000,
			1000 + (precision_time_t)i * 1000,
		};

		sync_fake_queue_reads(&source, &source_value, NULL, 1U);
		sync_fake_queue_reads(&sink, sink_values, NULL, ARRAY_SIZE(sink_values));
		zassert_ok(precision_clock_sync_run_once(&sync, generation));
	}

	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.offset_ns, 2);
	zassert_equal(status.state, PRECISION_SYNC_LOCKED);
	zassert_equal(status.accepted_observations, ARRAY_SIZE(offsets));
}

ZTEST(precision_timing, test_clock_sync_rejects_excessive_uncertainty)
{
	static const precision_time_t source_reads[] = {1000};
	static const precision_time_t sink_reads[] = {990, 1010};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	source.caps.resolution_ns = 3;
	sink.caps.resolution_ns = 2;
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.max_uncertainty_ns = 14;
	generation = sync_fake_start(&sync, &config);

	zassert_equal(precision_clock_sync_run_once(&sync, generation), -ESTALE);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.sampling_uncertainty_ns, 15);
	zassert_equal(status.rejected_observations, 1U);
	zassert_equal(status.accepted_observations, 0U);
	zassert_equal(sink.rate_calls, 0U);
}

ZTEST(precision_timing, test_clock_sync_steps_with_phase_then_reacquires)
{
	static const precision_time_t source_reads[] = {1000};
	static const precision_time_t sink_reads[] = {0, 0};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	precision_time_t sink_value;
	int source_error;
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	sink.caps.max_phase_adjust_ns = 2000;
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 100;
	generation = sync_fake_start(&sync, &config);

	zassert_ok(precision_clock_sync_run_once_at(&sync, generation, 1000));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(sink.control_operation_count, 2U);
	zassert_equal(sink.control_operations[0], 'R');
	zassert_equal(sink.control_operations[1], 'P');
	zassert_equal(sink.last_rate_ppb, 0);
	zassert_equal(sink.last_phase_ns, 1000);
	zassert_equal(sink.set_calls, 0U);
	zassert_equal(status.last_action, PRECISION_DISCIPLINE_STEP);
	zassert_equal(status.applied_rate_ppb, 0);
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);

	source_error = -EIO;
	sink_value = 1001;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, &sink_value, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 3000), -EIO);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.source_age_ns, 2000);
	zassert_equal(status.accepted_observations, 1U);
	zassert_equal(status.applied_rate_ppb, 0);
}

ZTEST(precision_timing, test_clock_sync_falls_back_to_absolute_set)
{
	static const precision_time_t source_reads[] = {1000};
	static const precision_time_t sink_reads[] = {0, 0, 10};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	sink.caps.max_phase_adjust_ns = 50;
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 100;
	generation = sync_fake_start(&sync, &config);

	zassert_ok(precision_clock_sync_run_once(&sync, generation));
	zassert_equal(sink.control_operation_count, 2U);
	zassert_equal(sink.control_operations[0], 'R');
	zassert_equal(sink.control_operations[1], 'S');
	zassert_equal(sink.phase_calls, 0U);
	zassert_equal(sink.last_set_time, 1010);
}

ZTEST(precision_timing, test_clock_sync_falls_back_when_phase_is_rejected)
{
	static const precision_time_t source_reads[] = {1000};
	static const precision_time_t sink_reads[] = {0, 0, 10};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	sink.caps.max_phase_adjust_ns = 0;
	sink.phase_error = -ERANGE;
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 100;
	generation = sync_fake_start(&sync, &config);

	zassert_ok(precision_clock_sync_run_once(&sync, generation));
	zassert_equal(sink.control_operation_count, 3U);
	zassert_equal(sink.control_operations[0], 'R');
	zassert_equal(sink.control_operations[1], 'P');
	zassert_equal(sink.control_operations[2], 'S');
	zassert_equal(sink.phase_calls, 1U);
	zassert_equal(sink.last_set_time, 1010);
}

ZTEST(precision_timing, test_clock_sync_bounds_holdover_and_reacquires_fresh)
{
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	precision_time_t source_value;
	precision_time_t sink_values[2];
	int source_error;
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	config.pi.lock_sample_count = 0U;
	config.pi.outlier_threshold_ns = 0;
	config.pi.max_uncertainty_ns = 0;
	config.pi.source_timeout_ns = 3000;
	config.pi.holdover_ns = 3000;
	generation = sync_fake_start(&sync, &config);

	source_value = 1100;
	sink_values[0] = 1000;
	sink_values[1] = 1000;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once_at(&sync, generation, 1000));
	zassert_equal(sink.last_rate_ppb, 100);

	source_error = -EIO;
	sink_values[0] = 2000;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 2000), -EIO);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_LOCKED);
	zassert_equal(status.source_age_ns, 1000);
	zassert_equal(status.applied_rate_ppb, 100);

	sink_values[0] = 5000;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 5000), -EIO);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_HOLDOVER);
	zassert_equal(status.applied_rate_ppb, 100);

	sink_values[0] = 8000;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 8000), -EIO);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);
	zassert_equal(status.last_action, PRECISION_DISCIPLINE_RESET);
	zassert_equal(status.applied_rate_ppb, 0);
	zassert_equal(status.source_read_failures, 3U);

	source_value = 9100;
	sink_values[0] = 9000;
	sink_values[1] = 9000;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once_at(&sync, generation, 9000));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_LOCKED);
	zassert_equal(status.applied_rate_ppb, 100);
	zassert_equal(status.accepted_observations, 2U);
	zassert_equal(status.last_error, 0);
}

ZTEST(precision_timing, test_clock_sync_rejected_recovery_preserves_timeout_state)
{
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	precision_time_t source_value;
	precision_time_t sink_values[2];
	int source_error = -EIO;
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	config.pi.lock_sample_count = 0U;
	config.pi.outlier_threshold_ns = 0;
	config.pi.max_uncertainty_ns = 2;
	config.pi.source_timeout_ns = 3000;
	config.pi.holdover_ns = 3000;
	generation = sync_fake_start(&sync, &config);

	source_value = 1100;
	sink_values[0] = 1000;
	sink_values[1] = 1000;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once_at(&sync, generation, 1000));

	sink_values[0] = 2000;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 2000), -EIO);

	source_value = 3100;
	sink_values[0] = 3000;
	sink_values[1] = 3002;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 3000), -ESTALE);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_LOCKED);
	zassert_equal(status.applied_rate_ppb, 100);
	zassert_equal(status.source_age_ns, 2000);
	zassert_equal(status.rejected_observations, 1U);

	sink_values[0] = 4000;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 8000), -EIO);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);
	zassert_equal(status.last_action, PRECISION_DISCIPLINE_RESET);
	zassert_equal(status.applied_rate_ppb, 0);
	zassert_equal(status.source_age_ns, 7000);
	zassert_equal(status.accepted_observations, 1U);
	zassert_equal(status.source_read_failures, 2U);
}

ZTEST(precision_timing, test_clock_sync_rejected_observations_expire_source)
{
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	precision_time_t source_value;
	precision_time_t sink_values[2];
	int source_error;
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	config.pi.lock_sample_count = 0U;
	config.pi.outlier_threshold_ns = 0;
	config.pi.max_uncertainty_ns = 2;
	config.pi.source_timeout_ns = 3000;
	config.pi.holdover_ns = 3000;
	generation = sync_fake_start(&sync, &config);

	source_value = 1100;
	sink_values[0] = 1000;
	sink_values[1] = 1000;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once_at(&sync, generation, 1000));

	source_value = 2100;
	sink_values[0] = 2000;
	sink_values[1] = 2002;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 5000), -ESTALE);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_HOLDOVER);
	zassert_equal(status.applied_rate_ppb, 100);
	zassert_equal(status.source_age_ns, 4000);

	source_value = 3100;
	sink_values[0] = 3000;
	sink_values[1] = 3002;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 8000), -ESTALE);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);
	zassert_equal(status.last_action, PRECISION_DISCIPLINE_RESET);
	zassert_equal(status.applied_rate_ppb, 0);
	zassert_equal(status.source_age_ns, 7000);
	zassert_equal(status.rejected_observations, 2U);
	zassert_equal(status.source_read_failures, 0U);

	source_error = -EIO;
	sink_values[0] = 4000;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 9000), -EIO);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);
	zassert_equal(status.applied_rate_ppb, 0);
	zassert_equal(status.source_age_ns, 8000);
	zassert_equal(status.source_read_failures, 1U);
}

ZTEST(precision_timing, test_clock_sync_source_timeout_uses_monotonic_uptime)
{
	static const precision_time_t source_value = 1;
	static const int source_error = -EIO;
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	precision_time_t sink_values[2];
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	config.pi.lock_sample_count = 0U;
	config.pi.outlier_threshold_ns = 0;
	config.pi.max_uncertainty_ns = 0;
	config.pi.source_timeout_ns = 3000;
	config.pi.holdover_ns = 3000;
	generation = sync_fake_start(&sync, &config);

	sink_values[0] = 999999999;
	sink_values[1] = 999999999;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once_at(&sync, generation, 1000));
	zassert_true(sink.last_rate_ppb < -999999990);

	sink_values[0] = 1000000000;
	sync_fake_queue_reads(&source, NULL, &source_error, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 1U);
	zassert_equal(precision_clock_sync_run_once_at(&sync, generation, 8000), -EIO);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);
	zassert_equal(status.last_action, PRECISION_DISCIPLINE_RESET);
	zassert_equal(status.applied_rate_ppb, 0);
	zassert_equal(status.source_age_ns, 7000);
}

ZTEST(precision_timing, test_clock_sync_rejects_locked_outliers)
{
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	precision_time_t source_value;
	precision_time_t sink_values[2];
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	config.pi.lock_sample_count = 0U;
	config.pi.outlier_threshold_ns = 50;
	config.pi.outlier_sample_count = 2U;
	config.pi.max_uncertainty_ns = 0;
	generation = sync_fake_start(&sync, &config);

	source_value = 1000;
	sink_values[0] = 1000;
	sink_values[1] = 1000;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once(&sync, generation));

	source_value = 2100;
	sink_values[0] = 2000;
	sink_values[1] = 2000;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once(&sync, generation));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.last_action, PRECISION_DISCIPLINE_IGNORE);
	zassert_equal(status.rejected_observations, 1U);

	source_value = 3100;
	sink_values[0] = 3000;
	sink_values[1] = 3000;
	sync_fake_queue_reads(&source, &source_value, NULL, 1U);
	sync_fake_queue_reads(&sink, sink_values, NULL, 2U);
	zassert_ok(precision_clock_sync_run_once(&sync, generation));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.last_action, PRECISION_DISCIPLINE_RESET);
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);
	zassert_equal(status.rejected_observations, 2U);
}

ZTEST(precision_timing, test_clock_sync_transient_sink_read_failure_is_retried)
{
	static const int sink_errors[] = {-EIO, 0, 0};
	static const precision_time_t source_reads[] = {1000};
	static const precision_time_t sink_reads[] = {0, 1000, 1000};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, sink_errors, ARRAY_SIZE(sink_errors));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 2U;
	generation = sync_fake_start(&sync, &config);

	zassert_ok(precision_clock_sync_run_once(&sync, generation));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_true(status.running);
	zassert_equal(status.sink_read_failures, 1U);
	zassert_equal(status.accepted_observations, 1U);
	zassert_equal(status.control_failures, 0U);
	zassert_equal(sink.rate_calls, 1U);
}

ZTEST(precision_timing, test_clock_sync_control_failure_faults)
{
	static const precision_time_t source_reads[] = {1100};
	static const precision_time_t sink_reads[] = {1000, 1000};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	sink.rate_error = -EACCES;
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	config.pi.step_threshold_ns = 0;
	generation = sync_fake_start(&sync, &config);

	zassert_equal(precision_clock_sync_run_once(&sync, generation), -EACCES);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_FAULT);
	zassert_false(status.running);
	zassert_equal(status.control_failures, 1U);
	zassert_equal(status.last_error, -EACCES);
}

ZTEST(precision_timing, test_clock_sync_rejects_sampling_overflow)
{
	static const precision_time_t source_reads[] = {0};
	static const precision_time_t sink_reads[] = {PRECISION_TIME_MIN, PRECISION_TIME_MAX};
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync = {0};
	uint32_t generation;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	sync_fake_queue_reads(&source, source_reads, NULL, ARRAY_SIZE(source_reads));
	sync_fake_queue_reads(&sink, sink_reads, NULL, ARRAY_SIZE(sink_reads));
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	config.readings_per_update = 1U;
	generation = sync_fake_start(&sync, &config);

	zassert_equal(precision_clock_sync_run_once(&sync, generation), -ERANGE);
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_equal(status.state, PRECISION_SYNC_UNSYNCED);
	zassert_true(status.running);
	zassert_equal(status.rejected_observations, 1U);
	zassert_equal(status.last_error, -ERANGE);
	zassert_equal(sink.rate_calls, 0U);
}

ZTEST(precision_timing, test_clock_sync_basic_lifecycle_is_idempotent)
{
	struct sync_fake_clock source;
	struct sync_fake_clock sink;
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_sync sync;

	sync_fake_init(&source, sync_source_domain, false);
	sync_fake_init(&sink, sync_sink_domain, true);
	zassert_ok(precision_clock_sync_config_default(&config, &source.clock, &sink.clock));
	zassert_ok(precision_clock_sync_init(&sync, &config));
	zassert_ok(precision_clock_sync_start(&sync));
	zassert_equal(precision_clock_sync_start(&sync), -EALREADY);
	zassert_ok(precision_clock_sync_stop(&sync));
	zassert_ok(precision_clock_sync_stop(&sync));
	zassert_ok(precision_clock_sync_get_status(&sync, &status));
	zassert_false(status.running);
	zassert_ok(precision_clock_sync_reset(&sync));
	zassert_equal(sink.last_rate_ppb, 0);
}
