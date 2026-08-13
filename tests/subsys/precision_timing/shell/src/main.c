/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/precision_timing/precision_timing.h>
#include <zephyr/precision_timing/precision_timing_shell.h>
#include <zephyr/ztest.h>

struct fake_clock {
	struct precision_clock clock;
	struct precision_clock_caps caps;
	precision_time_t time;
	int read_error;
	int caps_error;
	int adjust_rate_error;
	int32_t last_rate_ppb;
	uint32_t adjust_rate_calls;
	struct k_sem *read_entered;
	struct k_sem *read_release;
};

static const struct shell *test_shell;

static struct fake_clock *fake_clock_from_precision(const struct precision_clock *clock)
{
	return (struct fake_clock *)clock->adapter;
}

static int fake_clock_read(const struct precision_clock *clock,
			   struct precision_time_point *time_point)
{
	struct fake_clock *fake = fake_clock_from_precision(clock);

	if (fake->read_entered != NULL) {
		k_sem_give(fake->read_entered);
	}
	if (fake->read_release != NULL) {
		(void)k_sem_take(fake->read_release, K_FOREVER);
	}
	if (fake->read_error < 0) {
		return fake->read_error;
	}

	*time_point = (struct precision_time_point){
		.time = fake->time,
		.domain = clock->domain,
	};
	return 0;
}

static int fake_clock_set(const struct precision_clock *clock,
			  const struct precision_time_point *time_point)
{
	struct fake_clock *fake = fake_clock_from_precision(clock);

	fake->time = time_point->time;
	return 0;
}

static int fake_clock_adjust_phase(const struct precision_clock *clock, precision_time_t phase_ns)
{
	struct fake_clock *fake = fake_clock_from_precision(clock);

	fake->time += phase_ns;
	return 0;
}

static int fake_clock_adjust_rate(const struct precision_clock *clock, int32_t rate_ppb)
{
	struct fake_clock *fake = fake_clock_from_precision(clock);

	fake->adjust_rate_calls++;
	fake->last_rate_ppb = rate_ppb;
	return fake->adjust_rate_error;
}

static int fake_clock_get_caps(const struct precision_clock *clock,
			       struct precision_clock_caps *caps)
{
	struct fake_clock *fake = fake_clock_from_precision(clock);

	if (fake->caps_error < 0) {
		return fake->caps_error;
	}

	*caps = fake->caps;
	return 0;
}

static const struct precision_clock_api fake_clock_api = {
	.read = fake_clock_read,
	.set = fake_clock_set,
	.adjust_phase = fake_clock_adjust_phase,
	.adjust_rate = fake_clock_adjust_rate,
	.get_caps = fake_clock_get_caps,
};

static void fake_clock_init(struct fake_clock *fake, enum precision_time_domain_type domain_type,
			    uint32_t domain_id, precision_time_t time, uint32_t caps_flags)
{
	memset(fake, 0, sizeof(*fake));
	fake->clock.api = &fake_clock_api;
	fake->clock.adapter = fake;
	fake->clock.domain.type = domain_type;
	fake->clock.domain.id = domain_id;
	fake->caps = (struct precision_clock_caps){
		.flags = caps_flags,
		.resolution_ns = 7,
		.max_phase_adjust_ns = 123456,
		.min_rate_ppb = -100000,
		.max_rate_ppb = 200000,
	};
	fake->time = time;
}

static int execute(const char *command)
{
	shell_backend_dummy_clear_output(test_shell);
	return shell_execute_cmd(test_shell, command);
}

static const char *output_get(void)
{
	size_t output_size;

	return shell_backend_dummy_get_output(test_shell, &output_size);
}

static void assert_output_contains(const char *output, const char *expected)
{
	zassert_not_null(strstr(output, expected), "output '%s' does not contain '%s'", output,
			 expected);
}

static void assert_output_error(const char *output, const char *name, int error)
{
	char expected[64];
	int length;

	length = snprintk(expected, sizeof(expected), "precision clock '%s': %d", name, error);
	zassert_true(length > 0 && length < sizeof(expected));
	assert_output_contains(output, expected);
}

#if defined(CONFIG_PRECISION_CLOCK_SYNC_SERVICE)
static void assert_output_last_error(const char *output, int error)
{
	char expected[32];
	int length;

	length = snprintk(expected, sizeof(expected), "last_error=%d", error);
	zassert_true(length > 0 && length < sizeof(expected));
	assert_output_contains(output, expected);
}
#endif

static void registry_cleanup(void)
{
	static const char *const names[] = {
		"Alpha",
		"alpha",
		"beta",
		"clock",
		"concurrent",
		"sync",
		"1234567890123456789012345678901",
	};

	for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
		(void)precision_timing_shell_unregister(names[i]);
	}
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);

	registry_cleanup();
	shell_backend_dummy_clear_output(test_shell);
}

static void *setup(void)
{
	test_shell = shell_backend_dummy_get_ptr();
	WAIT_FOR(shell_ready(test_shell), 20000, k_msleep(1));
	zassert_true(shell_ready(test_shell), "timed out waiting for dummy shell backend");

	return NULL;
}

ZTEST(precision_timing_shell, test_registration_validation_capacity_and_sorted_list)
{
	struct fake_clock fake;
	char copied_name[] = "beta";
	const char *output;
	const char *alpha_position;
	const char *beta_position;
	static const char max_name[] = "1234567890123456789012345678901";

	fake_clock_init(&fake, PRECISION_TIME_DOMAIN_RAW, 1, 0, PRECISION_CLOCK_CAP_READ);

	zassert_equal(precision_timing_shell_register(NULL, &fake.clock, NULL), -EINVAL);
	zassert_equal(precision_timing_shell_register("", &fake.clock, NULL), -EINVAL);
	zassert_equal(precision_timing_shell_register("bad name", &fake.clock, NULL), -EINVAL);
	zassert_equal(precision_timing_shell_register("bad/name", &fake.clock, NULL), -EINVAL);
	zassert_equal(precision_timing_shell_register("12345678901234567890123456789012",
						      &fake.clock, NULL),
		      -EINVAL);
	zassert_equal(precision_timing_shell_register("clock", NULL, NULL), -EINVAL);
	zassert_equal(precision_timing_shell_unregister(NULL), -EINVAL);

	zassert_ok(precision_timing_shell_register(copied_name, &fake.clock, NULL));
	zassert_equal(precision_timing_shell_register("beta", &fake.clock, NULL), -EEXIST);
	copied_name[0] = 'z';
	zassert_ok(precision_timing_shell_register("Alpha", &fake.clock, NULL));
	zassert_equal(precision_timing_shell_register("alpha", &fake.clock, NULL), -ENOSPC);

	zassert_ok(execute("precision_clock list"));
	output = output_get();
	alpha_position = strstr(output, "Alpha");
	beta_position = strstr(output, "beta");
	zassert_not_null(alpha_position);
	zassert_not_null(beta_position);
	zassert_true(alpha_position < beta_position, "registry list is not sorted: %s", output);

	zassert_equal(precision_timing_shell_unregister("unknown"), -ENOENT);
	zassert_ok(precision_timing_shell_unregister("beta"));
	zassert_ok(precision_timing_shell_register(max_name, &fake.clock, NULL));
	zassert_ok(precision_timing_shell_unregister(max_name));
	zassert_ok(precision_timing_shell_register("alpha", &fake.clock, NULL));
	zassert_ok(precision_timing_shell_unregister("Alpha"));
	zassert_ok(precision_timing_shell_unregister("alpha"));
}

ZTEST(precision_timing_shell, test_get_caps_clock_only_and_provider_errors)
{
	struct fake_clock fake;
	const char *output;

	fake_clock_init(&fake, PRECISION_TIME_DOMAIN_UTC, 42, -123,
			PRECISION_CLOCK_CAP_READ | PRECISION_CLOCK_CAP_SET |
				PRECISION_CLOCK_CAP_ADJUST_PHASE | PRECISION_CLOCK_CAP_ADJUST_RATE);
	zassert_ok(precision_timing_shell_register("clock", &fake.clock, NULL));

	zassert_ok(execute("precision_clock get clock"));
	output = output_get();
	assert_output_contains(output, "time_ns=-123");
	assert_output_contains(output, "domain=utc");
	assert_output_contains(output, "domain_id=42");

	zassert_ok(execute("precision_clock caps clock"));
	output = output_get();
	assert_output_contains(output, "read=yes set=yes adjust_phase=yes adjust_rate=yes");
	assert_output_contains(output, "resolution_ns=7");
	assert_output_contains(output, "max_phase_adjust_ns=123456");
	assert_output_contains(output, "min_rate_ppb=-100000 max_rate_ppb=200000");

	fake.read_error = -EIO;
	zassert_equal(execute("precision_clock get clock"), -EIO);
	assert_output_error(output_get(), "clock", -EIO);
	fake.read_error = 0;
	fake.caps_error = -ERANGE;
	zassert_equal(execute("precision_clock caps clock"), -ERANGE);
	assert_output_error(output_get(), "clock", -ERANGE);

	zassert_equal(execute("precision_clock get missing"), -ENOENT);
	assert_output_error(output_get(), "missing", -ENOENT);
	zassert_equal(execute("precision_clock caps missing"), -ENOENT);

#if defined(CONFIG_PRECISION_CLOCK_SYNC_SERVICE)
	zassert_equal(execute("precision_clock status clock"), -ENOTSUP);
	assert_output_error(output_get(), "clock", -ENOTSUP);
#endif

	zassert_ok(precision_timing_shell_unregister("clock"));
}

#if defined(CONFIG_PRECISION_CLOCK_SYNC_SERVICE)
static void sync_init(struct precision_clock_sync *sync, struct fake_clock *source,
		      struct fake_clock *sink)
{
	struct precision_clock_sync_config config;

	fake_clock_init(source, PRECISION_TIME_DOMAIN_PHC, 11, 1000000, PRECISION_CLOCK_CAP_READ);
	fake_clock_init(sink, PRECISION_TIME_DOMAIN_RAW, 12, 900000,
			PRECISION_CLOCK_CAP_READ | PRECISION_CLOCK_CAP_SET |
				PRECISION_CLOCK_CAP_ADJUST_PHASE | PRECISION_CLOCK_CAP_ADJUST_RATE);
	zassert_ok(precision_clock_sync_config_default(&config, &source->clock, &sink->clock));
	zassert_ok(precision_clock_sync_init(sync, &config));
}

ZTEST(precision_timing_shell, test_status_and_lifecycle_return_values)
{
	struct precision_clock_sync sync = {0};
	struct fake_clock source;
	struct fake_clock sink;
	const char *output;

	sync_init(&sync, &source, &sink);
	(void)k_mutex_lock(&sync.lock, K_FOREVER);
	sync.status = (struct precision_clock_sync_status){
		.running = false,
		.source_domain = {.type = PRECISION_TIME_DOMAIN_PHC, .id = 11},
		.sink_domain = {.type = PRECISION_TIME_DOMAIN_RAW, .id = 12},
		.state = PRECISION_SYNC_LOCKED,
		.last_action = PRECISION_DISCIPLINE_ADJUST_RATE,
		.offset_ns = -1234,
		.applied_rate_ppb = -55,
		.source_age_ns = 666,
		.sampling_uncertainty_ns = 77,
		.accepted_observations = 8,
		.rejected_observations = 9,
		.source_read_failures = 10,
		.sink_read_failures = 11,
		.control_failures = 12,
		.last_error = -ESTALE,
	};
	(void)k_mutex_unlock(&sync.lock);
	zassert_ok(precision_timing_shell_register("sync", &sink.clock, &sync));

	zassert_ok(execute("precision_clock status sync"));
	output = output_get();
	assert_output_contains(output, "running=no");
	assert_output_contains(output, "source_domain=phc source_domain_id=11");
	assert_output_contains(output, "sink_domain=raw sink_domain_id=12");
	assert_output_contains(output, "state=LOCKED last_action=ADJUST_RATE");
	assert_output_contains(output, "offset_ns=-1234 applied_rate_ppb=-55");
	assert_output_contains(output, "source_age_ns=666 sampling_uncertainty_ns=77");
	assert_output_contains(output, "accepted_observations=8 rejected_observations=9");
	assert_output_contains(output,
			       "source_read_failures=10 sink_read_failures=11 control_failures=12");
	assert_output_last_error(output, -ESTALE);

	zassert_ok(execute("precision_clock start sync"));
	zassert_equal(execute("precision_clock start sync"), -EALREADY);
	assert_output_error(output_get(), "sync", -EALREADY);
	zassert_ok(execute("precision_clock stop sync"));
	zassert_ok(execute("precision_clock stop sync"));
	zassert_ok(execute("precision_clock reset sync"));
	zassert_equal(sink.last_rate_ppb, 0);

	sink.adjust_rate_error = -EIO;
	zassert_equal(execute("precision_clock reset sync"), -EIO);
	assert_output_error(output_get(), "sync", -EIO);
	zassert_ok(precision_timing_shell_unregister("sync"));
}

struct concurrent_context {
	atomic_t command_done;
	atomic_t unregister_started;
	atomic_t unregister_done;
	int command_result;
	int unregister_result;
};

#define CONCURRENT_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(command_stack, CONCURRENT_STACK_SIZE);
K_THREAD_STACK_DEFINE(unregister_stack, CONCURRENT_STACK_SIZE);
static struct k_thread command_thread;
static struct k_thread unregister_thread;

static void command_thread_fn(void *context_ptr, void *unused1, void *unused2)
{
	struct concurrent_context *context = context_ptr;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	context->command_result = execute("precision_clock get concurrent");
	atomic_set(&context->command_done, 1);
}

static void unregister_thread_fn(void *context_ptr, void *unused1, void *unused2)
{
	struct concurrent_context *context = context_ptr;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	atomic_set(&context->unregister_started, 1);
	context->unregister_result = precision_timing_shell_unregister("concurrent");
	atomic_set(&context->unregister_done, 1);
}

ZTEST(precision_timing_shell, test_unregister_waits_for_active_command)
{
	struct concurrent_context context = {0};
	struct k_sem read_entered;
	struct k_sem read_release;
	struct fake_clock fake;

	k_sem_init(&read_entered, 0, 1);
	k_sem_init(&read_release, 0, 1);
	fake_clock_init(&fake, PRECISION_TIME_DOMAIN_RAW, 99, 123, PRECISION_CLOCK_CAP_READ);
	fake.read_entered = &read_entered;
	fake.read_release = &read_release;
	zassert_ok(precision_timing_shell_register("concurrent", &fake.clock, NULL));

	k_thread_create(&command_thread, command_stack, K_THREAD_STACK_SIZEOF(command_stack),
			command_thread_fn, &context, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&read_entered, K_SECONDS(1)));
	k_thread_create(&unregister_thread, unregister_stack,
			K_THREAD_STACK_SIZEOF(unregister_stack), unregister_thread_fn, &context,
			NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	WAIT_FOR(atomic_get(&context.unregister_started) != 0, 1000, k_msleep(1));
	zassert_true(atomic_get(&context.unregister_started) != 0);
	k_msleep(20);
	zassert_false(atomic_get(&context.unregister_done),
		      "unregister returned while the clock provider was still active");

	k_sem_give(&read_release);
	zassert_ok(k_thread_join(&command_thread, K_SECONDS(1)));
	zassert_ok(k_thread_join(&unregister_thread, K_SECONDS(1)));
	zassert_ok(context.command_result);
	zassert_ok(context.unregister_result);
	zassert_true(atomic_get(&context.command_done));
	zassert_true(atomic_get(&context.unregister_done));
	zassert_equal(precision_timing_shell_unregister("concurrent"), -ENOENT);
}
#else
ZTEST(precision_timing_shell, test_sync_registration_requires_service)
{
	struct precision_clock_sync sync = {0};
	struct fake_clock fake;

	fake_clock_init(&fake, PRECISION_TIME_DOMAIN_RAW, 1, 0, PRECISION_CLOCK_CAP_READ);
	zassert_equal(precision_timing_shell_register("sync", &fake.clock, &sync), -ENOTSUP);
}
#endif /* CONFIG_PRECISION_CLOCK_SYNC_SERVICE */

ZTEST_SUITE(precision_timing_shell, NULL, setup, before_each, NULL, NULL);
