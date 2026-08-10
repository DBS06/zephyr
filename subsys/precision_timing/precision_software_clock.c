/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/precision_timing/precision_software_clock.h>

#define SOFTWARE_CLOCK_MIN_RATE_PPB (-999999999)
#define SOFTWARE_CLOCK_MAX_RATE_PPB INT32_MAX

/* Converting an absolute cycle count assumes that its frequency is stable. */
#if defined(CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER) &&                                               \
	!defined(CONFIG_SYSTEM_CLOCK_HW_CYCLES_PER_SEC_RUNTIME_UPDATE)
#define SOFTWARE_CLOCK_USE_CYCLE_COUNTER 1
#else
#define SOFTWARE_CLOCK_USE_CYCLE_COUNTER 0
#endif

static int software_clock_monotonic_now(precision_time_t *now_ns)
{
	uint64_t converted_ns;

	if (now_ns == NULL) {
		return -EINVAL;
	}

#if SOFTWARE_CLOCK_USE_CYCLE_COUNTER
	converted_ns = k_cyc_to_ns_floor64(k_cycle_get_64());
#else
	int64_t ticks = k_uptime_ticks();

	if (ticks < 0) {
		return -ERANGE;
	}

	converted_ns = k_ticks_to_ns_floor64((uint64_t)ticks);
#endif
	if (converted_ns > (uint64_t)PRECISION_TIME_MAX) {
		return -ERANGE;
	}

	*now_ns = (precision_time_t)converted_ns;

	return 0;
}

static precision_time_t software_clock_resolution_ns(void)
{
#if SOFTWARE_CLOCK_USE_CYCLE_COUNTER
	return (precision_time_t)k_cyc_to_ns_ceil64(1);
#else
	return (precision_time_t)k_ticks_to_ns_ceil64(1);
#endif
}

static int software_clock_scale_ppb(precision_time_t elapsed_ns, int32_t rate_ppb,
				    precision_time_t *adjustment_ns)
{
	int64_t quotient;
	int64_t remainder;
	int64_t whole;
	int64_t fraction;

	if (elapsed_ns < 0 || adjustment_ns == NULL) {
		return -EINVAL;
	}

	quotient = elapsed_ns / NSEC_PER_SEC;
	remainder = elapsed_ns % NSEC_PER_SEC;

	if (rate_ppb > 0 && quotient > INT64_MAX / rate_ppb) {
		return -ERANGE;
	}

	whole = quotient * (int64_t)rate_ppb;
	/* The remainder is below 1e9 and the rate fits in int32_t. */
	fraction = (remainder * (int64_t)rate_ppb) / NSEC_PER_SEC;

	return precision_time_add(whole, fraction, adjustment_ns);
}

static int software_clock_value_locked(const struct precision_software_clock *clock,
				       precision_time_t monotonic_now_ns,
				       precision_time_t *value_ns)
{
	precision_time_t elapsed_ns;
	precision_time_t adjustment_ns;
	precision_time_t adjusted_elapsed_ns;
	int ret;

	ret = precision_time_sub(monotonic_now_ns, clock->anchor_monotonic_ns, &elapsed_ns);
	if (ret < 0) {
		return ret;
	}

	if (elapsed_ns < 0) {
		return -ERANGE;
	}

	ret = software_clock_scale_ppb(elapsed_ns, clock->rate_ppb, &adjustment_ns);
	if (ret < 0) {
		return ret;
	}

	ret = precision_time_add(elapsed_ns, adjustment_ns, &adjusted_elapsed_ns);
	if (ret < 0) {
		return ret;
	}

	return precision_time_add(clock->anchor_time_ns, adjusted_elapsed_ns, value_ns);
}

static struct precision_software_clock *
software_clock_from_precision(const struct precision_clock *precision_clk)
{
	return (struct precision_software_clock *)precision_clk->adapter;
}

static int software_clock_read(const struct precision_clock *precision_clk,
			       struct precision_time_point *tp)
{
	struct precision_software_clock *clock = software_clock_from_precision(precision_clk);
	precision_time_t monotonic_now_ns;
	precision_time_t value_ns;
	int ret;

	if (clock == NULL || !clock->initialized) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&clock->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	ret = software_clock_monotonic_now(&monotonic_now_ns);
	if (ret == 0) {
		ret = software_clock_value_locked(clock, monotonic_now_ns, &value_ns);
	}

	(void)k_mutex_unlock(&clock->lock);

	if (ret == 0) {
		tp->time = value_ns;
		tp->domain = precision_clk->domain;
	}

	return ret;
}

static int software_clock_set(const struct precision_clock *precision_clk,
			      const struct precision_time_point *tp)
{
	struct precision_software_clock *clock = software_clock_from_precision(precision_clk);
	precision_time_t monotonic_now_ns;
	int ret;

	if (clock == NULL || !clock->initialized) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&clock->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	ret = software_clock_monotonic_now(&monotonic_now_ns);
	if (ret == 0) {
		clock->anchor_time_ns = tp->time;
		clock->anchor_monotonic_ns = monotonic_now_ns;
	}

	(void)k_mutex_unlock(&clock->lock);

	return ret;
}

static int software_clock_adjust_phase(const struct precision_clock *precision_clk,
				       precision_time_t phase_ns)
{
	struct precision_software_clock *clock = software_clock_from_precision(precision_clk);
	precision_time_t monotonic_now_ns;
	precision_time_t value_ns;
	precision_time_t adjusted_ns;
	int ret;

	if (clock == NULL || !clock->initialized) {
		return -EINVAL;
	}

	if (phase_ns == PRECISION_TIME_MIN) {
		return -ERANGE;
	}

	ret = k_mutex_lock(&clock->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	ret = software_clock_monotonic_now(&monotonic_now_ns);
	if (ret == 0) {
		ret = software_clock_value_locked(clock, monotonic_now_ns, &value_ns);
	}
	if (ret == 0) {
		ret = precision_time_add(value_ns, phase_ns, &adjusted_ns);
	}
	if (ret == 0) {
		clock->anchor_time_ns = adjusted_ns;
		clock->anchor_monotonic_ns = monotonic_now_ns;
	}

	(void)k_mutex_unlock(&clock->lock);

	return ret;
}

static int software_clock_adjust_rate(const struct precision_clock *precision_clk, int32_t rate_ppb)
{
	struct precision_software_clock *clock = software_clock_from_precision(precision_clk);
	precision_time_t monotonic_now_ns;
	precision_time_t value_ns;
	int ret;

	if (clock == NULL || !clock->initialized) {
		return -EINVAL;
	}

	if (rate_ppb < SOFTWARE_CLOCK_MIN_RATE_PPB) {
		return -ERANGE;
	}

	ret = k_mutex_lock(&clock->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	ret = software_clock_monotonic_now(&monotonic_now_ns);
	if (ret == 0) {
		ret = software_clock_value_locked(clock, monotonic_now_ns, &value_ns);
	}
	if (ret == 0) {
		clock->anchor_time_ns = value_ns;
		clock->anchor_monotonic_ns = monotonic_now_ns;
		clock->rate_ppb = rate_ppb;
	}

	(void)k_mutex_unlock(&clock->lock);

	return ret;
}

static int software_clock_get_caps(const struct precision_clock *precision_clk,
				   struct precision_clock_caps *caps)
{
	struct precision_software_clock *clock = software_clock_from_precision(precision_clk);

	if (clock == NULL || !clock->initialized) {
		return -EINVAL;
	}

	*caps = (struct precision_clock_caps){
		.flags = PRECISION_CLOCK_CAP_READ | PRECISION_CLOCK_CAP_SET |
			 PRECISION_CLOCK_CAP_ADJUST_PHASE | PRECISION_CLOCK_CAP_ADJUST_RATE,
		.resolution_ns = software_clock_resolution_ns(),
		.max_phase_adjust_ns = PRECISION_TIME_MAX,
		.min_rate_ppb = SOFTWARE_CLOCK_MIN_RATE_PPB,
		.max_rate_ppb = SOFTWARE_CLOCK_MAX_RATE_PPB,
	};

	return 0;
}

static const struct precision_clock_api software_clock_api = {
	.read = software_clock_read,
	.set = software_clock_set,
	.adjust_phase = software_clock_adjust_phase,
	.adjust_rate = software_clock_adjust_rate,
	.get_caps = software_clock_get_caps,
};

int precision_software_clock_init(struct precision_software_clock *clock,
				  struct precision_time_domain output_domain,
				  precision_time_t initial_time)
{
	precision_time_t monotonic_now_ns;
	int ret;

	if (clock == NULL || output_domain.type == PRECISION_TIME_DOMAIN_INVALID) {
		return -EINVAL;
	}

	memset(clock, 0, sizeof(*clock));

	ret = k_mutex_init(&clock->lock);
	if (ret < 0) {
		return ret;
	}

	ret = software_clock_monotonic_now(&monotonic_now_ns);
	if (ret < 0) {
		return ret;
	}

	clock->clock.api = &software_clock_api;
	clock->clock.adapter = clock;
	clock->clock.domain = output_domain;
	clock->anchor_time_ns = initial_time;
	clock->anchor_monotonic_ns = monotonic_now_ns;
	clock->rate_ppb = 0;
	clock->initialized = true;

	return 0;
}

const struct precision_clock *
precision_software_clock_get(const struct precision_software_clock *clock)
{
	if (clock == NULL || !clock->initialized) {
		return NULL;
	}

	return &clock->clock;
}
