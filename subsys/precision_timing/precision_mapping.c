/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/timing/precision_timing.h>

/*
 * All domain mappings run timeutil_sync in nanosecond ticks, so the sync
 * configuration is a shared constant. Keeping it in static storage (rather than
 * embedding it in each mapping and self-referencing it from state.cfg) makes
 * struct precision_time_mapping trivially copyable.
 */
static const struct timeutil_sync_config precision_mapping_sync_cfg = {
	.ref_Hz = NSEC_PER_SEC,
	.local_Hz = NSEC_PER_SEC,
};

int precision_time_add(precision_time_t a, precision_time_t b, precision_time_t *result)
{
	if (result == NULL) {
		return -EINVAL;
	}

	if ((b > 0 && a > PRECISION_TIME_MAX - b) ||
	    (b < 0 && a < PRECISION_TIME_MIN - b)) {
		return -ERANGE;
	}

	*result = a + b;

	return 0;
}

int precision_time_sub(precision_time_t a, precision_time_t b, precision_time_t *result)
{
	if (b == PRECISION_TIME_MIN) {
		return -ERANGE;
	}

	return precision_time_add(a, -b, result);
}

int precision_time_from_u64_sec_nsec(uint64_t sec, uint32_t nsec, precision_time_t *result)
{
	precision_time_t sec_ns;

	if (result == NULL || nsec >= NSEC_PER_SEC ||
	    sec > (uint64_t)(PRECISION_TIME_MAX / NSEC_PER_SEC)) {
		return -ERANGE;
	}

	sec_ns = (precision_time_t)sec * NSEC_PER_SEC;

	return precision_time_add(sec_ns, (precision_time_t)nsec, result);
}

int precision_time_to_u64_sec_nsec(precision_time_t time, uint64_t *sec, uint32_t *nsec)
{
	if (sec == NULL || nsec == NULL || time < 0) {
		return -ERANGE;
	}

	*sec = (uint64_t)(time / NSEC_PER_SEC);
	*nsec = (uint32_t)(time % NSEC_PER_SEC);

	return 0;
}

static int precision_time_to_timeutil(precision_time_t time, precision_time_t bias, uint64_t *out)
{
	precision_time_t biased;
	int ret;

	ret = precision_time_add(time, bias, &biased);
	if (ret < 0 || biased < 0) {
		return -ERANGE;
	}

	*out = (uint64_t)biased;

	return 0;
}

static int precision_time_bias_for(precision_time_t time, precision_time_t *bias)
{
	if (bias == NULL || time == PRECISION_TIME_MIN) {
		return -ERANGE;
	}

	*bias = time <= 0 ? 1 - time : 0;

	return 0;
}

void precision_time_mapping_init(struct precision_time_mapping *mapping,
				 struct precision_time_domain source_domain,
				 struct precision_time_domain local_domain)
{
	if (mapping == NULL) {
		return;
	}

	memset(mapping, 0, sizeof(*mapping));
	mapping->source_domain = source_domain;
	mapping->local_domain = local_domain;
	mapping->state.cfg = &precision_mapping_sync_cfg;
}

void precision_time_mapping_invalidate(struct precision_time_mapping *mapping)
{
	struct precision_time_domain source_domain;
	struct precision_time_domain local_domain;

	if (mapping == NULL) {
		return;
	}

	source_domain = mapping->source_domain;
	local_domain = mapping->local_domain;
	precision_time_mapping_init(mapping, source_domain, local_domain);
}

int precision_time_mapping_update(struct precision_time_mapping *mapping,
				  const struct precision_time_observation *observation)
{
	struct timeutil_sync_instant instant;
	int ret;

	if (mapping == NULL || observation == NULL ||
	    (observation->flags & (PRECISION_OBSERVATION_SOURCE_VALID |
				   PRECISION_OBSERVATION_LOCAL_VALID)) !=
		    (PRECISION_OBSERVATION_SOURCE_VALID | PRECISION_OBSERVATION_LOCAL_VALID)) {
		return -EINVAL;
	}

	if (!precision_time_domain_equal(&mapping->source_domain, &observation->source.domain) ||
	    !precision_time_domain_equal(&mapping->local_domain, &observation->local.domain)) {
		return -EINVAL;
	}

	if (!mapping->valid) {
		ret = precision_time_bias_for(observation->source.time, &mapping->source_bias);
		if (ret < 0) {
			return ret;
		}

		ret = precision_time_bias_for(observation->local.time, &mapping->local_bias);
		if (ret < 0) {
			return ret;
		}
	}

	ret = precision_time_to_timeutil(observation->source.time, mapping->source_bias,
					 &instant.ref);
	if (ret < 0) {
		return ret;
	}

	ret = precision_time_to_timeutil(observation->local.time, mapping->local_bias,
					 &instant.local);
	if (ret < 0) {
		return ret;
	}

	ret = timeutil_sync_state_update(&mapping->state, &instant);
	if (ret < 0) {
		return ret;
	}

	mapping->valid = true;

	if (ret > 0) {
		float skew = timeutil_sync_estimate_skew(&mapping->state);

		if (skew > 0.0f) {
			(void)timeutil_sync_state_set_skew(&mapping->state, skew, NULL);
		}
	}

	return 0;
}

int precision_time_mapping_source_to_local(const struct precision_time_mapping *mapping,
					   const struct precision_time_point *source,
					   struct precision_time_point *local)
{
	uint64_t source_biased;
	int64_t local_biased;
	precision_time_t local_time;
	int ret;

	if (mapping == NULL || source == NULL || local == NULL) {
		return -EINVAL;
	}

	if (!mapping->valid) {
		return -EAGAIN;
	}

	if (!precision_time_domain_equal(&mapping->source_domain, &source->domain)) {
		return -EINVAL;
	}

	ret = precision_time_to_timeutil(source->time, mapping->source_bias, &source_biased);
	if (ret < 0) {
		return ret;
	}

	ret = timeutil_sync_local_from_ref(&mapping->state, source_biased, &local_biased);
	if (ret < 0) {
		return ret;
	}

	ret = precision_time_sub((precision_time_t)local_biased, mapping->local_bias, &local_time);
	if (ret < 0) {
		return ret;
	}

	local->time = local_time;
	local->domain = mapping->local_domain;

	return 0;
}

int precision_time_mapping_local_to_source(const struct precision_time_mapping *mapping,
					   const struct precision_time_point *local,
					   struct precision_time_point *source)
{
	uint64_t local_biased;
	uint64_t source_biased;
	precision_time_t source_time;
	int ret;

	if (mapping == NULL || source == NULL || local == NULL) {
		return -EINVAL;
	}

	if (!mapping->valid) {
		return -EAGAIN;
	}

	if (!precision_time_domain_equal(&mapping->local_domain, &local->domain)) {
		return -EINVAL;
	}

	ret = precision_time_to_timeutil(local->time, mapping->local_bias, &local_biased);
	if (ret < 0) {
		return ret;
	}

	ret = timeutil_sync_ref_from_local(&mapping->state, local_biased, &source_biased);
	if (ret < 0) {
		return ret;
	}

	if (source_biased > (uint64_t)PRECISION_TIME_MAX) {
		return -ERANGE;
	}

	ret = precision_time_sub((precision_time_t)source_biased, mapping->source_bias,
				 &source_time);
	if (ret < 0) {
		return ret;
	}

	source->time = source_time;
	source->domain = mapping->source_domain;

	return 0;
}
