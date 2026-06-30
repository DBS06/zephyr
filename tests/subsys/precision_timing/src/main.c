/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/timing/precision_timing.h>
#include <zephyr/ztest.h>

static const struct precision_time_domain source_domain = {
	.type = PRECISION_TIME_DOMAIN_PTP,
	.id = 1,
};

static const struct precision_time_domain local_domain = {
	.type = PRECISION_TIME_DOMAIN_PHC,
	.id = 2,
};

static struct precision_time_observation observation(precision_time_t source,
						     precision_time_t local)
{
	return (struct precision_time_observation) {
		.source = {
			.time = source,
			.domain = source_domain,
		},
		.local = {
			.time = local,
			.domain = local_domain,
		},
		.flags = PRECISION_OBSERVATION_SOURCE_VALID |
			 PRECISION_OBSERVATION_LOCAL_VALID,
	};
}

static struct precision_time_mapping build_offset_mapping(void)
{
	struct precision_time_mapping mapping;
	struct precision_time_observation obs1 = observation(0, 100);
	struct precision_time_observation obs2 = observation(1000, 1100);

	precision_time_mapping_init(&mapping, source_domain, local_domain);
	(void)precision_time_mapping_update(&mapping, &obs1);
	(void)precision_time_mapping_update(&mapping, &obs2);

	return mapping;
}

ZTEST(precision_timing, test_scalar_conversions_check_bounds)
{
	precision_time_t time;
	uint64_t sec;
	uint32_t nsec;

	zassert_ok(precision_time_from_u64_sec_nsec(12, 34, &time));
	zassert_equal(time, 12LL * NSEC_PER_SEC + 34);
	zassert_ok(precision_time_to_u64_sec_nsec(time, &sec, &nsec));
	zassert_equal(sec, 12);
	zassert_equal(nsec, 34);
	zassert_equal(precision_time_from_u64_sec_nsec(UINT64_MAX, 0, &time), -ERANGE);
	zassert_equal(precision_time_add(PRECISION_TIME_MAX, 1, &time), -ERANGE);
	zassert_equal(precision_time_sub(PRECISION_TIME_MIN, 1, &time), -ERANGE);
}

ZTEST(precision_timing, test_domain_mapping_converts_and_invalidates)
{
	struct precision_time_mapping mapping;
	struct precision_time_observation obs = observation(-100, 50);
	struct precision_time_point source = {
		.time = -90,
		.domain = source_domain,
	};
	struct precision_time_point local;

	precision_time_mapping_init(&mapping, source_domain, local_domain);
	zassert_equal(precision_time_mapping_source_to_local(&mapping, &source, &local),
		      -EAGAIN);

	zassert_ok(precision_time_mapping_update(&mapping, &obs));
	zassert_ok(precision_time_mapping_source_to_local(&mapping, &source, &local));
	zassert_equal(local.time, 60);
	zassert_true(precision_time_domain_equal(&local.domain, &local_domain));

	precision_time_mapping_invalidate(&mapping);
	zassert_equal(precision_time_mapping_source_to_local(&mapping, &source, &local),
		      -EAGAIN);
}

ZTEST(precision_timing, test_domain_mapping_inverts_local_to_source)
{
	struct precision_time_mapping mapping;
	struct precision_time_observation obs = observation(-100, 50);
	struct precision_time_point local = {
		.time = 60,
		.domain = local_domain,
	};
	struct precision_time_point source;

	precision_time_mapping_init(&mapping, source_domain, local_domain);
	zassert_equal(precision_time_mapping_local_to_source(&mapping, &local, &source),
		      -EAGAIN);

	zassert_ok(precision_time_mapping_update(&mapping, &obs));
	zassert_ok(precision_time_mapping_local_to_source(&mapping, &local, &source));
	zassert_equal(source.time, -90);
	zassert_true(precision_time_domain_equal(&source.domain, &source_domain));

	local.domain = source_domain;
	zassert_equal(precision_time_mapping_local_to_source(&mapping, &local, &source),
		      -EINVAL);
}

ZTEST(precision_timing, test_pi_adjusts_rate_and_locks)
{
	struct precision_pi_config config = {
		.source_domain = source_domain,
		.local_domain = local_domain,
		.step_threshold_ns = NSEC_PER_SEC,
		.lock_threshold_ns = 10,
		.lock_sample_count = 2,
		.min_rate_ppb = INT32_MIN,
		.max_rate_ppb = INT32_MAX,
		.kp_num = 7,
		.ki_num = 3,
		.gain_den = 10,
	};
	struct precision_pi_discipline discipline;
	struct precision_discipline_result result;
	struct precision_time_observation obs;

	zassert_ok(precision_pi_init(&discipline, &config));

	obs = observation(900, 1000);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.action, PRECISION_DISCIPLINE_ADJUST_RATE);
	zassert_equal(result.offset_ns, -100);
	zassert_equal(result.rate_ppb, -100);
	zassert_equal(result.state, PRECISION_SYNC_ACQUIRING);

	obs = observation(2000, 2000);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.state, PRECISION_SYNC_ACQUIRING);

	obs = observation(3000, 3000);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.state, PRECISION_SYNC_LOCKED);
}

ZTEST(precision_timing, test_pi_steps_and_rejects_locked_outliers)
{
	struct precision_pi_config config = {
		.source_domain = source_domain,
		.local_domain = local_domain,
		.step_threshold_ns = NSEC_PER_SEC,
		.lock_threshold_ns = 10,
		.outlier_threshold_ns = 100,
		.lock_sample_count = 1,
		.outlier_sample_count = 2,
		.min_rate_ppb = INT32_MIN,
		.max_rate_ppb = INT32_MAX,
		.kp_num = 7,
		.ki_num = 3,
		.gain_den = 10,
	};
	struct precision_pi_discipline discipline;
	struct precision_discipline_result result;
	struct precision_time_observation obs;

	zassert_ok(precision_pi_init(&discipline, &config));

	obs = observation(2 * NSEC_PER_SEC, 0);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.action, PRECISION_DISCIPLINE_STEP);
	zassert_equal(result.phase_correction_ns, 2 * NSEC_PER_SEC);

	obs = observation(10, 10);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.state, PRECISION_SYNC_LOCKED);

	obs = observation(20, 200);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.action, PRECISION_DISCIPLINE_IGNORE);
	zassert_equal(discipline.outlier_samples, 1);

	obs = observation(30, 220);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.action, PRECISION_DISCIPLINE_RESET);
	zassert_equal(discipline.state, PRECISION_SYNC_UNSYNCED);
}

ZTEST(precision_timing, test_pi_source_timeout_enters_holdover_then_unsyncs)
{
	struct precision_pi_config config = {
		.source_domain = source_domain,
		.local_domain = local_domain,
		.step_threshold_ns = NSEC_PER_SEC,
		.lock_threshold_ns = 10,
		.source_timeout_ns = 1000,
		.holdover_ns = 5000,
		.lock_sample_count = 0,
		.min_rate_ppb = INT32_MIN,
		.max_rate_ppb = INT32_MAX,
		.kp_num = 7,
		.ki_num = 3,
		.gain_den = 10,
	};
	struct precision_pi_discipline discipline;
	struct precision_discipline_result result;
	struct precision_time_observation obs;

	zassert_ok(precision_pi_init(&discipline, &config));

	/* No observation yet: nothing to time out. */
	zassert_equal(precision_pi_check_source_timeout(&discipline, 0, &result), -EAGAIN);

	obs = observation(1000, 1000);
	zassert_ok(precision_pi_process(&discipline, &obs, &result));
	zassert_equal(result.state, PRECISION_SYNC_LOCKED);

	/* Within the source timeout the lock is preserved. */
	zassert_ok(precision_pi_check_source_timeout(&discipline, 1500, &result));
	zassert_equal(result.state, PRECISION_SYNC_LOCKED);

	/* Past the source timeout but inside holdover. */
	zassert_equal(precision_pi_check_source_timeout(&discipline, 2500, &result), -ESTALE);
	zassert_equal(discipline.state, PRECISION_SYNC_HOLDOVER);

	/* Past the holdover window resets the discipline to unsynced. */
	zassert_equal(precision_pi_check_source_timeout(&discipline, 7500, &result), -ESTALE);
	zassert_equal(discipline.state, PRECISION_SYNC_UNSYNCED);
	zassert_false(discipline.has_last_update);
}

struct fake_clock_data {
	precision_time_t time;
	int32_t rate_ppb;
};

static int fake_clock_read(const struct precision_clock *clock,
			   struct precision_time_point *time)
{
	struct fake_clock_data *data = (struct fake_clock_data *)clock->user_data;

	time->time = data->time;
	time->domain = clock->domain;

	return 0;
}

static int fake_clock_set(const struct precision_clock *clock,
			  const struct precision_time_point *time)
{
	struct fake_clock_data *data = (struct fake_clock_data *)clock->user_data;

	data->time = time->time;

	return 0;
}

static int fake_clock_adjust_rate(const struct precision_clock *clock, int32_t rate_ppb)
{
	struct fake_clock_data *data = (struct fake_clock_data *)clock->user_data;

	data->rate_ppb = rate_ppb;

	return 0;
}

static const struct precision_clock_api fake_clock_api = {
	.read = fake_clock_read,
	.set = fake_clock_set,
	.adjust_rate = fake_clock_adjust_rate,
};

ZTEST(precision_timing, test_precision_clock_checks_domains_and_unsupported_ops)
{
	struct fake_clock_data data = {
		.time = 100,
	};
	struct precision_clock clock = {
		.api = &fake_clock_api,
		.user_data = &data,
		.domain = local_domain,
	};
	struct precision_time_point time = {
		.time = 200,
		.domain = source_domain,
	};

	zassert_ok(precision_clock_read(&clock, &time));
	zassert_equal(time.time, 100);
	zassert_true(precision_time_domain_equal(&time.domain, &local_domain));

	zassert_equal(precision_clock_set(&clock, &time), 0);
	zassert_equal(data.time, 100);

	time.domain = source_domain;
	zassert_equal(precision_clock_set(&clock, &time), -EINVAL);
	zassert_equal(precision_clock_adjust_phase(&clock, 1), -ENOTSUP);
	zassert_ok(precision_clock_adjust_rate(&clock, 1234));
	zassert_equal(data.rate_ppb, 1234);
}

ZTEST(precision_timing, test_pi_init_rejects_outlier_without_sample_count)
{
	struct precision_pi_config config = {
		.source_domain = source_domain,
		.local_domain = local_domain,
		.step_threshold_ns = NSEC_PER_SEC,
		.outlier_threshold_ns = 100,
		.outlier_sample_count = 0,
		.min_rate_ppb = INT32_MIN,
		.max_rate_ppb = INT32_MAX,
		.kp_num = 7,
		.ki_num = 3,
		.gain_den = 10,
	};
	struct precision_pi_discipline discipline;

	/* Outlier rejection enabled but no sample count would reset the servo
	 * on the first outlier, so the configuration is rejected.
	 */
	zassert_equal(precision_pi_init(&discipline, &config), -EINVAL);

	/* A non-zero sample count makes the configuration valid. */
	config.outlier_sample_count = 2;
	zassert_ok(precision_pi_init(&discipline, &config));

	/* Disabled outlier rejection does not require a sample count. */
	config.outlier_threshold_ns = 0;
	config.outlier_sample_count = 0;
	zassert_ok(precision_pi_init(&discipline, &config));
}

ZTEST(precision_timing, test_domain_mapping_is_trivially_copyable)
{
	struct precision_time_point src = {
		.time = 500,
		.domain = source_domain,
	};
	struct precision_time_mapping first = build_offset_mapping();
	struct precision_time_mapping second = build_offset_mapping();
	struct precision_time_point from_first;
	struct precision_time_point from_second;

	/* A mapping returned by value must stay usable: its timeutil sync
	 * state must not reference storage from the builder's stack frame.
	 */
	zassert_ok(precision_time_mapping_source_to_local(&first, &src, &from_first));
	zassert_ok(precision_time_mapping_source_to_local(&second, &src, &from_second));
	zassert_equal(from_first.time, 600);
	zassert_equal(from_first.time, from_second.time);

	/* Copies evolve independently. */
	precision_time_mapping_invalidate(&first);
	zassert_equal(precision_time_mapping_source_to_local(&first, &src, &from_first),
		      -EAGAIN);
	zassert_ok(precision_time_mapping_source_to_local(&second, &src, &from_second));
	zassert_equal(from_second.time, 600);
}

ZTEST(precision_timing, test_pi_matches_float_reference_servo)
{
	/*
	 * Offsets (source - local) that keep the servo in the rate-adjust
	 * regime while exercising fixed-point truncation against a floating
	 * point reference of the PI servo the shared engine replaces.
	 */
	static const int64_t offsets[] = {
		47000, -31000, 22345, -15678, 9013, -6007, 4111, -2733,
		1777, -1201, 811, -509, 337, -211, 143, -89, 61, -37, 23, -13,
	};
	struct precision_pi_config config = {
		.source_domain = source_domain,
		.local_domain = local_domain,
		.step_threshold_ns = NSEC_PER_SEC,
		.lock_sample_count = 0,
		.min_rate_ppb = INT32_MIN,
		.max_rate_ppb = INT32_MAX,
		.kp_num = 7,
		.ki_num = 3,
		.gain_den = 10,
	};
	struct precision_pi_discipline discipline;
	struct precision_discipline_result result;
	struct precision_time_observation obs;
	const double kp = 0.7;
	const double ki = 0.3;
	double ref_drift = 0.0;
	int64_t local = 1000;

	zassert_ok(precision_pi_init(&discipline, &config));

	for (int i = 0; i < (int)ARRAY_SIZE(offsets); i++) {
		double ref_ppb;
		double err;

		obs = observation(local + offsets[i], local);
		zassert_ok(precision_pi_process(&discipline, &obs, &result));
		zassert_equal(result.action, PRECISION_DISCIPLINE_ADJUST_RATE);

		ref_drift += ki * (double)offsets[i];
		ref_ppb = kp * (double)offsets[i] + ref_drift;

		err = ref_ppb - (double)result.rate_ppb;
		if (err < 0) {
			err = -err;
		}

		/*
		 * Fixed-point truncation stays within one part-per-billion for
		 * the proportional term plus one per accumulated integral
		 * sample, so the deviation from the float servo is bounded by
		 * i + 2 after i + 1 processed samples.
		 */
		zassert_true(err <= (double)(i + 2),
			     "sample %d: fixed=%d float=%d deviates too far",
			     i, result.rate_ppb, (int)ref_ppb);

		local += 1000;
	}

	zassert_equal(result.state, PRECISION_SYNC_LOCKED);
}

ZTEST_SUITE(precision_timing, NULL, NULL, NULL, NULL, NULL);
