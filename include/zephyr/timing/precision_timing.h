/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Experimental protocol-neutral precision timing APIs.
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_TIMING_PRECISION_TIMING_H_
#define ZEPHYR_INCLUDE_ZEPHYR_TIMING_PRECISION_TIMING_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys_clock.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int64_t precision_time_t;

#define PRECISION_TIME_MAX INT64_MAX
#define PRECISION_TIME_MIN INT64_MIN

enum precision_time_domain_type {
	PRECISION_TIME_DOMAIN_INVALID = 0,
	PRECISION_TIME_DOMAIN_TAI,
	PRECISION_TIME_DOMAIN_UTC,
	PRECISION_TIME_DOMAIN_MONOTONIC,
	PRECISION_TIME_DOMAIN_PHC,
	PRECISION_TIME_DOMAIN_PTP,
	PRECISION_TIME_DOMAIN_GPTP,
	PRECISION_TIME_DOMAIN_RAW,
};

struct precision_time_domain {
	enum precision_time_domain_type type;
	uint32_t id;
};

struct precision_time_point {
	precision_time_t time;
	struct precision_time_domain domain;
};

enum precision_observation_flags {
	PRECISION_OBSERVATION_SOURCE_VALID = BIT(0),
	PRECISION_OBSERVATION_LOCAL_VALID = BIT(1),
};

struct precision_time_observation {
	struct precision_time_point source;
	struct precision_time_point local;
	precision_time_t uncertainty_ns;
	uint32_t flags;
};

struct precision_time_mapping {
	struct precision_time_domain source_domain;
	struct precision_time_domain local_domain;
	struct timeutil_sync_state state;
	precision_time_t source_bias;
	precision_time_t local_bias;
	bool valid;
};

enum precision_clock_caps_flags {
	PRECISION_CLOCK_CAP_READ = BIT(0),
	PRECISION_CLOCK_CAP_SET = BIT(1),
	PRECISION_CLOCK_CAP_ADJUST_PHASE = BIT(2),
	PRECISION_CLOCK_CAP_ADJUST_RATE = BIT(3),
};

struct precision_clock_caps {
	uint32_t flags;
	precision_time_t resolution_ns;
	precision_time_t max_phase_adjust_ns;
	int32_t min_rate_ppb;
	int32_t max_rate_ppb;
};

struct precision_clock;

struct precision_clock_api {
	int (*read)(const struct precision_clock *clock, struct precision_time_point *time);
	int (*set)(const struct precision_clock *clock, const struct precision_time_point *time);
	int (*adjust_phase)(const struct precision_clock *clock, precision_time_t phase_ns);
	int (*adjust_rate)(const struct precision_clock *clock, int32_t rate_ppb);
	int (*get_caps)(const struct precision_clock *clock, struct precision_clock_caps *caps);
};

struct precision_clock {
	const struct precision_clock_api *api;
	const void *user_data;
	struct precision_time_domain domain;
};

enum precision_sync_state {
	PRECISION_SYNC_UNSYNCED = 0,
	PRECISION_SYNC_ACQUIRING,
	PRECISION_SYNC_LOCKED,
	PRECISION_SYNC_HOLDOVER,
	PRECISION_SYNC_FAULT,
};

enum precision_discipline_action {
	PRECISION_DISCIPLINE_IGNORE = 0,
	PRECISION_DISCIPLINE_STEP,
	PRECISION_DISCIPLINE_ADJUST_RATE,
	PRECISION_DISCIPLINE_RESET,
};

struct precision_pi_config {
	struct precision_time_domain source_domain;
	struct precision_time_domain local_domain;
	precision_time_t step_threshold_ns;
	precision_time_t lock_threshold_ns;
	precision_time_t outlier_threshold_ns;
	precision_time_t max_uncertainty_ns;
	precision_time_t source_timeout_ns;
	precision_time_t holdover_ns;
	uint8_t lock_sample_count;
	uint8_t outlier_sample_count;
	int32_t min_rate_ppb;
	int32_t max_rate_ppb;
	int32_t kp_num;
	int32_t ki_num;
	uint32_t gain_den;
};

struct precision_pi_discipline {
	struct precision_pi_config config;
	enum precision_sync_state state;
	int64_t drift_ppb;
	int64_t last_offset_ns;
	int32_t frequency_correction_ppb;
	uint32_t rejected_observations;
	precision_time_t last_update_ns;
	uint8_t lock_samples;
	uint8_t outlier_samples;
	bool has_last_update;
};

struct precision_discipline_result {
	enum precision_discipline_action action;
	enum precision_sync_state state;
	precision_time_t offset_ns;
	precision_time_t phase_correction_ns;
	int32_t rate_ppb;
	uint32_t rejected_observations;
};

static inline bool precision_time_domain_equal(const struct precision_time_domain *a,
					       const struct precision_time_domain *b)
{
	return a != NULL && b != NULL && a->type == b->type && a->id == b->id;
}

int precision_time_add(precision_time_t a, precision_time_t b, precision_time_t *result);
int precision_time_sub(precision_time_t a, precision_time_t b, precision_time_t *result);
int precision_time_from_u64_sec_nsec(uint64_t sec, uint32_t nsec, precision_time_t *result);
int precision_time_to_u64_sec_nsec(precision_time_t time, uint64_t *sec, uint32_t *nsec);

void precision_time_mapping_init(struct precision_time_mapping *mapping,
				 struct precision_time_domain source_domain,
				 struct precision_time_domain local_domain);
void precision_time_mapping_invalidate(struct precision_time_mapping *mapping);
int precision_time_mapping_update(struct precision_time_mapping *mapping,
				  const struct precision_time_observation *observation);
int precision_time_mapping_source_to_local(const struct precision_time_mapping *mapping,
					   const struct precision_time_point *source,
					   struct precision_time_point *local);
int precision_time_mapping_local_to_source(const struct precision_time_mapping *mapping,
					   const struct precision_time_point *local,
					   struct precision_time_point *source);

int precision_clock_read(const struct precision_clock *clock, struct precision_time_point *time);
int precision_clock_set(const struct precision_clock *clock,
			const struct precision_time_point *time);
int precision_clock_adjust_phase(const struct precision_clock *clock, precision_time_t phase_ns);
int precision_clock_adjust_rate(const struct precision_clock *clock, int32_t rate_ppb);
int precision_clock_get_caps(const struct precision_clock *clock,
			     struct precision_clock_caps *caps);

int precision_pi_init(struct precision_pi_discipline *discipline,
		      const struct precision_pi_config *config);
void precision_pi_reset(struct precision_pi_discipline *discipline);
int precision_pi_process(struct precision_pi_discipline *discipline,
			 const struct precision_time_observation *observation,
			 struct precision_discipline_result *result);
int precision_pi_check_source_timeout(struct precision_pi_discipline *discipline,
				      precision_time_t now_local_ns,
				      struct precision_discipline_result *result);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_TIMING_PRECISION_TIMING_H_ */
