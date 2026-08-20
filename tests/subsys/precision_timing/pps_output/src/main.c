/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/precision_timing/precision_clock.h>
#include <zephyr/precision_timing/precision_pps_output.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#define CALLBACK_TIMEOUT      K_MSEC(2000)
#define WAIT_STATE_TIMEOUT_US 2000000

/* Fake protocol-neutral clock and output-channel provider, guarded by fake_lock. */
struct fake_clock {
	struct precision_clock clock;
	struct precision_clock_output_caps caps;
	struct precision_clock_output_status status;
	precision_time_t offset_ns;
	precision_time_t stop_advance_ns;
	int read_error;
	int read_error_after_stop;
	int caps_error;
	int start_error;
	int stop_error;
	int status_error;
	uint32_t caps_calls;
	uint32_t start_calls;
	uint32_t stop_calls;
	uint32_t status_calls;
	bool provider_configured;
	bool clear_status_error_on_stop;
	struct k_sem *read_entered;
	struct k_sem *read_release;
};

static K_MUTEX_DEFINE(fake_lock);
static struct fake_clock fake;
static struct fake_clock fake_other;
static struct k_sem *active_read_release;

/* Per-instance callback bookkeeping so concurrent instances stay independent. */
struct cb_record {
	struct k_sem sem;
	struct precision_pps_output_state state;
	k_tid_t thread;
	uint32_t events;
	uint32_t calls;
	bool attempt_self_stop;
	int self_stop_result;
};

static struct cb_record rec_a;
static struct cb_record rec_b;

#define MAX_TRACKED_PPS_INSTANCES 6

static struct precision_pps_output *tracked_pps_instances[MAX_TRACKED_PPS_INSTANCES];
static size_t tracked_pps_instance_count;

static void track_pps_instance(struct precision_pps_output *instance)
{
	zassert_true(tracked_pps_instance_count < ARRAY_SIZE(tracked_pps_instances));
	tracked_pps_instances[tracked_pps_instance_count++] = instance;
}

/*
 * Give every test distinct storage while retaining static lifetime in case a
 * failed assertion leaves delayed work pending for after_each() to cancel.
 */
#define TEST_PPS_INSTANCE(name)                                                                    \
	static struct precision_pps_output name;                                                   \
	track_pps_instance(&name)

static precision_time_t fake_monotonic_ns(void)
{
	return (precision_time_t)k_ticks_to_ns_floor64(k_uptime_ticks());
}

static int fake_read(const struct precision_clock *clock, precision_time_t *time)
{
	struct fake_clock *f = clock->data;
	struct k_sem *entered;
	struct k_sem *release;
	precision_time_t offset;
	int error;

	k_mutex_lock(&fake_lock, K_FOREVER);
	entered = f->read_entered;
	release = f->read_release;
	f->read_entered = NULL;
	f->read_release = NULL;
	if (release != NULL) {
		active_read_release = release;
	}
	k_mutex_unlock(&fake_lock);

	if (entered != NULL) {
		k_sem_give(entered);
	}
	if (release != NULL) {
		(void)k_sem_take(release, K_FOREVER);
	}

	k_mutex_lock(&fake_lock, K_FOREVER);
	if (active_read_release == release) {
		active_read_release = NULL;
	}
	error = f->read_error;
	offset = f->offset_ns;
	k_mutex_unlock(&fake_lock);

	if (error < 0) {
		return error;
	}

	*time = fake_monotonic_ns() + offset;
	return 0;
}

static int fake_get_output_caps(const struct precision_clock *clock, uint32_t channel,
				struct precision_clock_output_caps *caps)
{
	struct fake_clock *f = clock->data;
	int error;

	ARG_UNUSED(channel);

	k_mutex_lock(&fake_lock, K_FOREVER);
	f->caps_calls++;
	error = f->caps_error;
	*caps = f->caps;
	k_mutex_unlock(&fake_lock);

	return error;
}

static void fake_mark_active(struct fake_clock *f,
			     const struct precision_clock_output_waveform_config *config)
{
	f->provider_configured = true;
	f->status.configured = true;
	f->status.kind = PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM;
	f->status.config.waveform = *config;
	f->status.hardware_active_valid =
		(f->caps.flags & PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE) != 0U;
	f->status.hardware_active = f->status.hardware_active_valid;
}

static int fake_output_start_waveform(const struct precision_clock *clock, uint32_t channel,
				      const struct precision_clock_output_waveform_config *config)
{
	struct fake_clock *f = clock->data;
	int error;

	ARG_UNUSED(channel);

	k_mutex_lock(&fake_lock, K_FOREVER);
	f->start_calls++;
	error = f->start_error;
	if (error == 0) {
		if (f->provider_configured) {
			error = -EBUSY;
		} else {
			fake_mark_active(f, config);
		}
	}
	k_mutex_unlock(&fake_lock);

	return error;
}

static int fake_output_stop(const struct precision_clock *clock, uint32_t channel)
{
	struct fake_clock *f = clock->data;
	int error;

	ARG_UNUSED(channel);

	k_mutex_lock(&fake_lock, K_FOREVER);
	f->stop_calls++;
	f->offset_ns += f->stop_advance_ns;
	error = f->stop_error;
	if (error == 0) {
		f->provider_configured = false;
		f->read_error = f->read_error_after_stop;
		f->status.configured = false;
		if (f->clear_status_error_on_stop) {
			f->status_error = 0;
		}
	}
	k_mutex_unlock(&fake_lock);

	return error;
}

static int fake_get_output_status(const struct precision_clock *clock, uint32_t channel,
				  struct precision_clock_output_status *status)
{
	struct fake_clock *f = clock->data;
	int error;

	ARG_UNUSED(channel);

	k_mutex_lock(&fake_lock, K_FOREVER);
	f->status_calls++;
	error = f->status_error;
	*status = f->status;
	k_mutex_unlock(&fake_lock);

	return error;
}

static const struct precision_clock_api fake_api = {
	.read = fake_read,
	.get_output_caps = fake_get_output_caps,
	.output_start_waveform = fake_output_start_waveform,
	.output_stop = fake_output_stop,
	.get_output_status = fake_get_output_status,
};

static void fake_init(struct fake_clock *f)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	memset(f, 0, sizeof(*f));
	f->clock.api = &fake_api;
	f->clock.data = f;
	f->caps = (struct precision_clock_output_caps){
		.flags = PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM |
			 PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH,
		.channel_count = 1,
		.resolution_ns = 1,
		.min_lead_time_ns = NSEC_PER_MSEC,
		.min_period_ns = NSEC_PER_MSEC,
		.max_period_ns = 2LL * NSEC_PER_SEC,
		.min_pulse_width_ns = 1,
		.max_pulse_width_ns = 999999999,
	};
	k_mutex_unlock(&fake_lock);
}

static struct precision_pps_output_config default_config(void)
{
	return (struct precision_pps_output_config){
		.channel = 0,
		.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT,
		.pulse_width_ns = 0,
		.start_guard_ns = 10LL * NSEC_PER_MSEC,
		.step_limit_ns = 100LL * NSEC_PER_MSEC,
		.poll_interval_ms = 20,
	};
}

static struct precision_pps_output_config exact_config(void)
{
	struct precision_pps_output_config config = default_config();

	config.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_EXACT;
	config.pulse_width_ns = 200LL * NSEC_PER_MSEC;
	return config;
}

static void bump_offset(struct fake_clock *f, precision_time_t delta_ns)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->offset_ns += delta_ns;
	k_mutex_unlock(&fake_lock);
}

static void set_configured(struct fake_clock *f, bool configured)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->status.configured = configured;
	k_mutex_unlock(&fake_lock);
}

static void set_hardware_active(struct fake_clock *f, bool active)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->status.hardware_active = active;
	k_mutex_unlock(&fake_lock);
}

static void mismatch_active_output(struct fake_clock *f)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->status.config.waveform.first_rising_time += 1;
	k_mutex_unlock(&fake_lock);
}

static void set_read_error(struct fake_clock *f, int error)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->read_error = error;
	k_mutex_unlock(&fake_lock);
}

static void set_status_error(struct fake_clock *f, int error)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->status_error = error;
	k_mutex_unlock(&fake_lock);
}

static void set_start_error(struct fake_clock *f, int error)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->start_error = error;
	k_mutex_unlock(&fake_lock);
}

static void set_stop_error(struct fake_clock *f, int error)
{
	k_mutex_lock(&fake_lock, K_FOREVER);
	f->stop_error = error;
	k_mutex_unlock(&fake_lock);
}

static void test_callback(struct precision_pps_output *instance, uint32_t events,
			  const struct precision_pps_output_state *state, void *user_data)
{
	struct cb_record *rec = user_data;

	rec->thread = k_current_get();
	rec->events = events;
	rec->state = *state;
	rec->calls++;
	if (rec->attempt_self_stop) {
		rec->attempt_self_stop = false;
		rec->self_stop_result = precision_pps_output_stop(instance);
	}
	k_sem_give(&rec->sem);
}

static void cb_record_reset(struct cb_record *rec)
{
	k_sem_init(&rec->sem, 0, 1);
	memset(&rec->state, 0, sizeof(rec->state));
	rec->thread = NULL;
	rec->events = PRECISION_PPS_OUTPUT_EVENT_NONE;
	rec->calls = 0;
	rec->attempt_self_stop = false;
	rec->self_stop_result = 0;
}

static void wait_callback(struct cb_record *rec)
{
	zassert_equal(k_sem_take(&rec->sem, CALLBACK_TIMEOUT), 0,
		      "timed out waiting for an instance callback");
}

static void start_instance(struct precision_pps_output *instance, struct fake_clock *f,
			   const struct precision_pps_output_config *config, struct cb_record *rec)
{
	zassert_ok(precision_pps_output_init(instance, &f->clock, config, test_callback, rec));
	zassert_ok(precision_pps_output_start(instance));
}

static bool poll_advanced(struct precision_pps_output *instance, precision_time_t after_phc_ns,
			  struct precision_pps_output_state *out)
{
	return precision_pps_output_state_get(instance, out) == 0 &&
	       out->phc_time_ns > after_phc_ns;
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);

	tracked_pps_instance_count = 0;
	cb_record_reset(&rec_a);
	cb_record_reset(&rec_b);
}

static void release_parked_reads(void)
{
	struct k_sem *active;
	struct k_sem *pending;
	struct k_sem *pending_other;

	k_mutex_lock(&fake_lock, K_FOREVER);
	active = active_read_release;
	pending = fake.read_release;
	pending_other = fake_other.read_release;
	active_read_release = NULL;
	fake.read_entered = NULL;
	fake.read_release = NULL;
	fake_other.read_entered = NULL;
	fake_other.read_release = NULL;
	k_mutex_unlock(&fake_lock);

	if (active != NULL) {
		k_sem_give(active);
	}
	if (pending != NULL && pending != active) {
		k_sem_give(pending);
	}
	if (pending_other != NULL && pending_other != active && pending_other != pending) {
		k_sem_give(pending_other);
	}
}

static void after_each(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Release a fake provider read before synchronously cancelling its poll. */
	release_parked_reads();
	set_stop_error(&fake, 0);
	set_stop_error(&fake_other, 0);
	for (size_t i = 0; i < tracked_pps_instance_count; i++) {
		if (tracked_pps_instances[i]->started) {
			(void)precision_pps_output_stop(tracked_pps_instances[i]);
		}
	}
}

ZTEST(precision_timing_pps_output, test_init_rejects_invalid_arguments)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_config bad;
	struct precision_pps_output_state state;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);

	zassert_equal(precision_pps_output_init(NULL, &fake.clock, &config, test_callback, &rec_a),
		      -EINVAL);
	zassert_equal(precision_pps_output_init(&pps, NULL, &config, test_callback, &rec_a),
		      -EINVAL);
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, NULL, test_callback, &rec_a),
		      -EINVAL);
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, &config, NULL, &rec_a), -EINVAL);

	bad = config;
	bad.width_policy = (enum precision_clock_output_width_policy)5;
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, &bad, test_callback, &rec_a),
		      -EINVAL);

	bad = exact_config();
	bad.pulse_width_ns = NSEC_PER_SEC;
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, &bad, test_callback, &rec_a),
		      -EINVAL);

	bad = exact_config();
	bad.pulse_width_ns = 0;
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, &bad, test_callback, &rec_a),
		      -EINVAL);

	bad = config;
	bad.start_guard_ns = -1;
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, &bad, test_callback, &rec_a),
		      -EINVAL);

	bad = config;
	bad.step_limit_ns = -1;
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, &bad, test_callback, &rec_a),
		      -EINVAL);

	bad = config;
	bad.poll_interval_ms = 0;
	zassert_equal(precision_pps_output_init(&pps, &fake.clock, &bad, test_callback, &rec_a),
		      -EINVAL);

	zassert_equal(precision_pps_output_state_get(NULL, &state), -EINVAL);
	zassert_equal(precision_pps_output_start(NULL), -EINVAL);
	zassert_equal(precision_pps_output_stop(NULL), -EINVAL);

	/* A valid but not-started instance rejects observation and stop. */
	zassert_ok(precision_pps_output_init(&pps, &fake.clock, &config, test_callback, &rec_a));
	zassert_equal(precision_pps_output_state_get(&pps, NULL), -EINVAL);
	zassert_equal(precision_pps_output_state_get(&pps, &state), -EINVAL);
	zassert_equal(precision_pps_output_stop(&pps), -EALREADY);
	zassert_equal(fake.caps_calls, 0, "no clock query before start");
}

ZTEST(precision_timing_pps_output, test_start_validates_caps_against_policy)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_config exact = exact_config();
	struct precision_pps_output_state state;

	TEST_PPS_INSTANCE(no_waveform);
	TEST_PPS_INSTANCE(bad_resolution);
	TEST_PPS_INSTANCE(fixed_width);
	TEST_PPS_INSTANCE(bad_width_range);
	TEST_PPS_INSTANCE(bad_period_range);
	TEST_PPS_INSTANCE(caps_error);

	fake_init(&fake);
	fake.caps.flags = 0;
	zassert_ok(precision_pps_output_init(&no_waveform, &fake.clock, &config, test_callback,
					     &rec_a));
	zassert_equal(precision_pps_output_start(&no_waveform), -ENOTSUP);
	zassert_equal(precision_pps_output_state_get(&no_waveform, &state), -EINVAL);

	fake_init(&fake);
	fake.caps.resolution_ns = 3;
	zassert_ok(precision_pps_output_init(&bad_resolution, &fake.clock, &config, test_callback,
					     &rec_a));
	zassert_equal(precision_pps_output_start(&bad_resolution), -ERANGE);

	/* Exact width demands programmable width support. */
	fake_init(&fake);
	fake.caps.flags = PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM;
	zassert_ok(precision_pps_output_init(&fixed_width, &fake.clock, &exact, test_callback,
					     &rec_a));
	zassert_equal(precision_pps_output_start(&fixed_width), -ENOTSUP);

	/* Exact width outside the reported pulse-width range. */
	fake_init(&fake);
	fake.caps.max_pulse_width_ns = exact.pulse_width_ns - 1;
	zassert_ok(precision_pps_output_init(&bad_width_range, &fake.clock, &exact, test_callback,
					     &rec_a));
	zassert_equal(precision_pps_output_start(&bad_width_range), -ERANGE);

	fake_init(&fake);
	fake.caps.min_period_ns = 2LL * NSEC_PER_SEC;
	zassert_ok(precision_pps_output_init(&bad_period_range, &fake.clock, &config, test_callback,
					     &rec_a));
	zassert_equal(precision_pps_output_start(&bad_period_range), -ERANGE);

	fake_init(&fake);
	fake.caps_error = -EIO;
	zassert_ok(precision_pps_output_init(&caps_error, &fake.clock, &config, test_callback,
					     &rec_a));
	zassert_equal(precision_pps_output_start(&caps_error), -EIO);
	zassert_equal(precision_pps_output_state_get(&caps_error, &state), -EINVAL);
}

ZTEST(precision_timing_pps_output, test_start_rejects_double_start)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_equal(precision_pps_output_start(&pps), -EALREADY);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_initial_arm_stops_then_starts_fresh_output)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_true(rec_a.state.active);
	zassert_false(rec_a.state.rearm_pending);
	zassert_equal(rec_a.state.generation, 1);
	zassert_equal(rec_a.state.rearm_count, 0);
	zassert_equal(rec_a.state.last_error, 0);
	zassert_equal(rec_a.state.config.period_ns, (precision_time_t)NSEC_PER_SEC);
	zassert_equal(rec_a.state.config.width_policy,
		      PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT);
	zassert_equal(rec_a.state.config.first_rising_time % (precision_time_t)NSEC_PER_SEC, 0);
	zassert_equal(fake.start_calls, 1);
	zassert_equal(fake.stop_calls, 1);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_stop_crosses_target_boundary)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	/* Model PHC time advancing while the synchronous provider stop blocks. */
	fake.stop_advance_ns = 2 * (precision_time_t)NSEC_PER_SEC;
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_true(rec_a.state.active);
	zassert_true(rec_a.state.config.first_rising_time >
		     fake_monotonic_ns() + fake.offset_ns);
	zassert_equal(fake.start_calls, 1);
	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_read_failure_after_stop_retries)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	fake.read_error_after_stop = -EIO;
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_READ_ERROR) != 0);
	zassert_false(rec_a.state.phc_read_valid);
	zassert_true(rec_a.state.rearm_pending);
	zassert_equal(rec_a.state.last_error, -EIO);
	zassert_equal(fake.start_calls, 0);
	k_mutex_lock(&fake_lock, K_FOREVER);
	fake.read_error_after_stop = 0;
	fake.read_error = 0;
	k_mutex_unlock(&fake_lock);
	wait_callback(&rec_a);
	zassert_true(rec_a.state.active);
	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_exact_width_is_programmed_and_reported)
{
	struct precision_pps_output_config config = exact_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_equal(rec_a.state.config.width_policy, PRECISION_CLOCK_OUTPUT_WIDTH_EXACT);
	zassert_equal(rec_a.state.config.pulse_width_ns, config.pulse_width_ns);

	k_mutex_lock(&fake_lock, K_FOREVER);
	zassert_equal(fake.status.config.waveform.width_policy, PRECISION_CLOCK_OUTPUT_WIDTH_EXACT);
	zassert_equal(fake.status.config.waveform.pulse_width_ns, config.pulse_width_ns);
	k_mutex_unlock(&fake_lock);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_adopts_already_active_matching_output)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	fake.provider_configured = true;
	fake.status.configured = true;
	fake.status.kind = PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM;
	fake.status.config.waveform = (struct precision_clock_output_waveform_config){
		.first_rising_time = 0,
		.period_ns = NSEC_PER_SEC,
		.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT,
		.pulse_width_ns = 0,
	};

	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_equal(rec_a.events, PRECISION_PPS_OUTPUT_EVENT_ARMED);
	zassert_true(rec_a.state.active);
	zassert_equal(rec_a.state.generation, 1);
	zassert_equal(rec_a.state.rearm_count, 0);
	zassert_equal(fake.start_calls, 0);
	zassert_equal(fake.stop_calls, 0);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_configured_only_status_stays_active)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state before;
	struct precision_pps_output_state after;
	uint32_t calls_before;

	TEST_PPS_INSTANCE(pps);

	/* Default caps do not advertise hardware-active: validity stays false. */
	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);
	zassert_ok(precision_pps_output_state_get(&pps, &before));

	k_mutex_lock(&fake_lock, K_FOREVER);
	zassert_false(fake.status.hardware_active_valid,
		      "hardware activity must be reported as unobservable");
	k_mutex_unlock(&fake_lock);

	calls_before = rec_a.calls;
	k_msleep(4 * config.poll_interval_ms + 10);

	zassert_equal(rec_a.calls, calls_before,
		      "a configured, matching, unobservable output must not rearm");
	zassert_ok(precision_pps_output_state_get(&pps, &after));
	zassert_true(after.active);
	zassert_equal(after.generation, before.generation);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_hardware_active_false_recovers)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	fake.caps.flags |= PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE;
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	k_mutex_lock(&fake_lock, K_FOREVER);
	zassert_true(fake.status.hardware_active_valid);
	zassert_true(fake.status.hardware_active);
	k_mutex_unlock(&fake_lock);

	set_hardware_active(&fake, false);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_OUTPUT_INACTIVE) != 0);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_RECOVERED) != 0);
	zassert_false((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_true(rec_a.state.active);
	zassert_equal(rec_a.state.generation, 2);
	zassert_equal(rec_a.state.rearm_count, 1);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_normal_drift_does_not_rearm)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state before;
	struct precision_pps_output_state after;
	uint32_t calls_before;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);
	zassert_ok(precision_pps_output_state_get(&pps, &before));

	calls_before = rec_a.calls;
	bump_offset(&fake, 1000);

	zassert_true(WAIT_FOR(poll_advanced(&pps, before.phc_time_ns, &after),
			      WAIT_STATE_TIMEOUT_US, k_msleep(1)));
	zassert_equal(after.continuity_error_ns, 1000);
	zassert_equal(after.generation, before.generation);
	zassert_equal(rec_a.calls, calls_before, "a bounded drift must not produce an event");

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_forward_hard_step_rearms)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	bump_offset(&fake, 400LL * NSEC_PER_MSEC);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_HARD_STEP) != 0);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_RECOVERED) != 0);
	zassert_false((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_equal(rec_a.state.continuity_error_ns, 400LL * NSEC_PER_MSEC);
	zassert_equal(rec_a.state.generation, 2);
	zassert_equal(rec_a.state.rearm_count, 1);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_backward_hard_step_rearms)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	bump_offset(&fake, -150LL * NSEC_PER_MSEC);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_HARD_STEP) != 0);
	zassert_false((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_equal(rec_a.state.continuity_error_ns, -150LL * NSEC_PER_MSEC);
	zassert_equal(rec_a.state.generation, 2);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_inactive_output_recovers)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	set_configured(&fake, false);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_OUTPUT_INACTIVE) != 0);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_RECOVERED) != 0);
	zassert_false((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_true(rec_a.state.active);
	zassert_equal(rec_a.state.generation, 2);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_mismatched_output_recovers)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	mismatch_active_output(&fake);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_OUTPUT_INACTIVE) != 0);
	zassert_false((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_true(rec_a.state.active);
	zassert_equal(rec_a.state.generation, 2);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_read_and_status_errors_preserve_output)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state before;
	uint32_t stop_calls_before;
	uint32_t start_calls_before;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);
	zassert_ok(precision_pps_output_state_get(&pps, &before));

	stop_calls_before = fake.stop_calls;
	start_calls_before = fake.start_calls;
	set_read_error(&fake, -EIO);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_READ_ERROR) != 0);
	zassert_true(rec_a.state.active);
	zassert_equal(fake.stop_calls, stop_calls_before);

	set_read_error(&fake, 0);
	set_status_error(&fake, -EAGAIN);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_READ_RECOVERED) != 0);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_STATUS_ERROR) != 0);
	zassert_true(rec_a.state.active);
	zassert_equal(fake.stop_calls, stop_calls_before);
	zassert_equal(fake.start_calls, start_calls_before);

	set_status_error(&fake, 0);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_STATUS_RECOVERED) != 0);
	zassert_equal(rec_a.state.last_error, 0);
	zassert_equal(rec_a.state.generation, before.generation);
	zassert_equal(rec_a.state.rearm_count, before.rearm_count);
	zassert_equal(fake.stop_calls, stop_calls_before);
	zassert_equal(fake.start_calls, start_calls_before);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_combined_errors_keep_most_recent_code)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	set_read_error(&fake, -EIO);
	set_status_error(&fake, -EAGAIN);
	wait_callback(&rec_a);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_READ_ERROR) != 0U);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_STATUS_ERROR) != 0U);
	zassert_equal(rec_a.state.last_error, -EAGAIN);

	set_read_error(&fake, 0);
	set_status_error(&fake, 0);
	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_persistent_status_errors_stop_and_rearm)
{
	struct precision_pps_output_config config = default_config();
	uint32_t stop_calls_before;
	uint32_t start_calls_before;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	stop_calls_before = fake.stop_calls;
	start_calls_before = fake.start_calls;
	k_mutex_lock(&fake_lock, K_FOREVER);
	fake.status_error = -EIO;
	fake.clear_status_error_on_stop = true;
	k_mutex_unlock(&fake_lock);

	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_STATUS_ERROR) != 0);
	zassert_equal(fake.stop_calls, stop_calls_before);
	zassert_equal(fake.start_calls, start_calls_before);

	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_RECOVERED) != 0);
	zassert_false((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_true(rec_a.state.active);
	zassert_false(rec_a.state.rearm_pending);
	zassert_equal(rec_a.state.generation, 2);
	zassert_equal(rec_a.state.rearm_count, 1);
	zassert_equal(fake.stop_calls, stop_calls_before + 1);
	zassert_equal(fake.start_calls, start_calls_before + 1);

	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_STATUS_RECOVERED) != 0);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_start_error_retries_next_poll)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	fake.start_error = -ETIME;
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_START_ERROR) != 0);
	zassert_equal(fake.start_calls, 1);
	zassert_true(rec_a.state.rearm_pending);
	zassert_false(rec_a.state.active);

	set_start_error(&fake, 0);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_equal(fake.start_calls, 2);
	zassert_true(rec_a.state.active);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_stop_error_retries_before_start)
{
	struct precision_pps_output_config config = default_config();

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	fake.stop_error = -EIO;
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_STOP_ERROR) != 0);
	zassert_equal(fake.stop_calls, 1);
	zassert_equal(fake.start_calls, 0);

	set_stop_error(&fake, 0);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_equal(fake.stop_calls, 2);
	zassert_equal(fake.start_calls, 1);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_callback_runs_on_dedicated_queue)
{
	extern struct k_work_q k_sys_work_q;
	struct precision_pps_output_config config = default_config();
	const char *name;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_not_null(rec_a.thread);
	zassert_not_equal(rec_a.thread, k_current_get(), "callback ran on the caller thread");
	zassert_not_equal(rec_a.thread, k_sys_work_q.thread_id,
			  "callback ran on the system workqueue");
	name = k_thread_name_get(rec_a.thread);
	zassert_not_null(name);
	zassert_equal(strcmp(name, "pps_output"), 0,
		      "callback did not run on the dedicated PPS output workqueue");

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_callback_snapshot_matches_state_get)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state queried;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	zassert_ok(precision_pps_output_state_get(&pps, &queried));
	zassert_equal(queried.generation, rec_a.state.generation);
	zassert_equal(queried.rearm_count, rec_a.state.rearm_count);
	zassert_equal(queried.active, rec_a.state.active);
	zassert_equal(queried.phc_time_ns, rec_a.state.phc_time_ns);
	zassert_equal(queried.last_error, rec_a.state.last_error);
	zassert_equal(queried.config.first_rising_time, rec_a.state.config.first_rising_time);
	zassert_equal(queried.config.period_ns, rec_a.state.config.period_ns);
	zassert_equal(queried.config.width_policy, rec_a.state.config.width_policy);

	zassert_ok(precision_pps_output_stop(&pps));
	zassert_equal(precision_pps_output_state_get(&pps, &queried), -EINVAL);
}

ZTEST(precision_timing_pps_output, test_stop_stops_output_and_blocks_further_calls)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state state;
	uint32_t calls_before;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	calls_before = rec_a.calls;
	zassert_ok(precision_pps_output_stop(&pps));
	zassert_true(fake.stop_calls >= 1);
	zassert_equal(precision_pps_output_state_get(&pps, &state), -EINVAL);

	/* Cancellation is synchronous: no poll can be pending or running afterward. */
	zassert_equal(k_sem_take(&rec_a.sem, K_NO_WAIT), -EBUSY);
	zassert_equal(rec_a.calls, calls_before);

	/* A stopped instance restarts without reinitialization. */
	zassert_ok(precision_pps_output_start(&pps));
	wait_callback(&rec_a);
	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_stop_failure_retains_started)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state state;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	set_stop_error(&fake, -EIO);
	zassert_equal(precision_pps_output_stop(&pps), -EIO);

	/* Started state is retained so the caller cannot release the clock. */
	zassert_ok(precision_pps_output_state_get(&pps, &state));

	/* Polling resumed: a forced hard step still produces a callback. */
	bump_offset(&fake, 500LL * NSEC_PER_MSEC);
	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_HARD_STEP) != 0);

	set_stop_error(&fake, 0);
	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_self_stop_from_callback_returns_deadlock)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state state;

	TEST_PPS_INSTANCE(pps);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	rec_a.attempt_self_stop = true;
	bump_offset(&fake, 400LL * NSEC_PER_MSEC);
	wait_callback(&rec_a);

	zassert_equal(rec_a.self_stop_result, -EDEADLK);

	/* The rejected recursive call must not have torn down the instance. */
	zassert_ok(precision_pps_output_state_get(&pps, &state));
	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_counters_saturate)
{
	struct precision_pps_output_config config = default_config();
	static struct k_sem read_entered;
	static struct k_sem read_release;

	TEST_PPS_INSTANCE(pps);

	k_sem_init(&read_entered, 0, 1);
	k_sem_init(&read_release, 0, 1);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	/* Park the next poll inside the clock read to preset the counters. */
	k_mutex_lock(&fake_lock, K_FOREVER);
	fake.read_entered = &read_entered;
	fake.read_release = &read_release;
	k_mutex_unlock(&fake_lock);
	zassert_ok(k_sem_take(&read_entered, K_SECONDS(2)));

	/*
	 * White-box preset: the struct is exposed by value, so the counters can
	 * be primed near saturation. The give/take pair below orders these
	 * writes before the parked poll resumes and reads them.
	 */
	pps.state.generation = UINT32_MAX;
	pps.state.rearm_count = UINT32_MAX - 1;
	bump_offset(&fake, 500LL * NSEC_PER_MSEC);
	k_sem_give(&read_release);

	wait_callback(&rec_a);
	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_RECOVERED) != 0);
	zassert_equal(rec_a.state.generation, UINT32_MAX, "generation must saturate");
	zassert_equal(rec_a.state.rearm_count, UINT32_MAX, "rearm count must saturate");

	/* A further rearm must not wrap past the maximum. */
	bump_offset(&fake, 500LL * NSEC_PER_MSEC);
	wait_callback(&rec_a);
	zassert_equal(rec_a.state.generation, UINT32_MAX);
	zassert_equal(rec_a.state.rearm_count, UINT32_MAX);

	zassert_ok(precision_pps_output_stop(&pps));
}

ZTEST(precision_timing_pps_output, test_two_instances_run_and_stop_independently)
{
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_config exact = exact_config();
	struct precision_pps_output_state state;

	TEST_PPS_INSTANCE(pps);
	TEST_PPS_INSTANCE(pps_other);

	fake_init(&fake);
	fake_init(&fake_other);

	start_instance(&pps, &fake, &config, &rec_a);
	start_instance(&pps_other, &fake_other, &exact, &rec_b);
	wait_callback(&rec_a);
	wait_callback(&rec_b);

	zassert_true((rec_a.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_true((rec_b.events & PRECISION_PPS_OUTPUT_EVENT_ARMED) != 0);
	zassert_ok(precision_pps_output_state_get(&pps, &state));
	zassert_equal(state.config.width_policy, PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT);
	zassert_ok(precision_pps_output_state_get(&pps_other, &state));
	zassert_equal(state.config.width_policy, PRECISION_CLOCK_OUTPUT_WIDTH_EXACT);

	/* Stopping the first instance leaves the second running. */
	zassert_ok(precision_pps_output_stop(&pps));
	zassert_equal(precision_pps_output_state_get(&pps, &state), -EINVAL);

	bump_offset(&fake_other, 400LL * NSEC_PER_MSEC);
	wait_callback(&rec_b);
	zassert_true((rec_b.events & PRECISION_PPS_OUTPUT_EVENT_HARD_STEP) != 0);
	zassert_ok(precision_pps_output_state_get(&pps_other, &state));
	zassert_true(state.active);

	zassert_ok(precision_pps_output_stop(&pps_other));
}

struct stop_thread_ctx {
	struct precision_pps_output *instance;
	atomic_t started;
	atomic_t done;
	int result;
};

#define STOP_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(stop_thread_stack, STOP_STACK_SIZE);
static struct k_thread stop_thread_data;

static void stop_thread_fn(void *ctx_ptr, void *unused1, void *unused2)
{
	struct stop_thread_ctx *ctx = ctx_ptr;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	atomic_set(&ctx->started, 1);
	ctx->result = precision_pps_output_stop(ctx->instance);
	atomic_set(&ctx->done, 1);
}

ZTEST(precision_timing_pps_output, test_stop_waits_for_in_flight_poll)
{
	static struct precision_pps_output pps;
	static struct stop_thread_ctx ctx;
	static struct k_sem read_entered;
	static struct k_sem read_release;
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state state;

	track_pps_instance(&pps);
	ctx = (struct stop_thread_ctx){.instance = &pps};

	k_sem_init(&read_entered, 0, 1);
	k_sem_init(&read_release, 0, 1);

	fake_init(&fake);
	start_instance(&pps, &fake, &config, &rec_a);
	wait_callback(&rec_a);

	k_mutex_lock(&fake_lock, K_FOREVER);
	fake.read_entered = &read_entered;
	fake.read_release = &read_release;
	k_mutex_unlock(&fake_lock);
	zassert_ok(k_sem_take(&read_entered, K_SECONDS(2)));

	k_thread_create(&stop_thread_data, stop_thread_stack,
			K_THREAD_STACK_SIZEOF(stop_thread_stack), stop_thread_fn, &ctx, NULL, NULL,
			K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

	zassert_true(WAIT_FOR(atomic_get(&ctx.started) != 0, 1000000, k_msleep(1)));
	k_msleep(20);
	zassert_false(atomic_get(&ctx.done),
		      "stop returned while the in-flight poll was still blocked");

	k_sem_give(&read_release);
	zassert_ok(k_thread_join(&stop_thread_data, K_SECONDS(2)));
	zassert_ok(ctx.result);

	(void)k_sem_take(&rec_a.sem, K_NO_WAIT);
	zassert_equal(precision_pps_output_state_get(&pps, &state), -EINVAL);
}

ZTEST(precision_timing_pps_output, test_stop_other_instance_while_poll_is_blocked)
{
	static struct precision_pps_output pps;
	static struct precision_pps_output pps_other;
	static struct stop_thread_ctx ctx;
	static struct k_sem read_entered;
	static struct k_sem read_release;
	struct precision_pps_output_config config = default_config();
	struct precision_pps_output_state state;

	track_pps_instance(&pps);
	track_pps_instance(&pps_other);
	ctx = (struct stop_thread_ctx){.instance = &pps};

	k_sem_init(&read_entered, 0, 1);
	k_sem_init(&read_release, 0, 1);

	fake_init(&fake);
	fake_init(&fake_other);
	start_instance(&pps, &fake, &config, &rec_a);
	start_instance(&pps_other, &fake_other, &config, &rec_b);
	wait_callback(&rec_a);
	wait_callback(&rec_b);

	/* Park the first instance's poll in its clock read. */
	k_mutex_lock(&fake_lock, K_FOREVER);
	fake.read_entered = &read_entered;
	fake.read_release = &read_release;
	k_mutex_unlock(&fake_lock);
	zassert_ok(k_sem_take(&read_entered, K_SECONDS(2)));

	/* Stopping the parked instance must wait for its own poll to finish. */
	k_thread_create(&stop_thread_data, stop_thread_stack,
			K_THREAD_STACK_SIZEOF(stop_thread_stack), stop_thread_fn, &ctx, NULL, NULL,
			K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	zassert_true(WAIT_FOR(atomic_get(&ctx.started) != 0, 1000000, k_msleep(1)));
	k_msleep(20);
	zassert_false(atomic_get(&ctx.done), "stop of the parked instance returned early");

	/* The other instance is cancelled independently without waiting. */
	zassert_ok(precision_pps_output_stop(&pps_other));
	zassert_equal(precision_pps_output_state_get(&pps_other, &state), -EINVAL);

	/* Release the parked poll and let its stop complete. */
	k_sem_give(&read_release);
	zassert_ok(k_thread_join(&stop_thread_data, K_SECONDS(2)));
	zassert_ok(ctx.result);
	zassert_equal(precision_pps_output_state_get(&pps, &state), -EINVAL);
}

ZTEST_SUITE(precision_timing_pps_output, NULL, NULL, before_each, after_each, NULL);
