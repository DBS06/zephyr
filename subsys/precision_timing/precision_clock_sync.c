/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/precision_timing/precision_clock_sync.h>

#include "precision_clock_sync_internal.h"

#define CLOCK_SYNC_DEFAULT_UPDATE_NS             (1LL * NSEC_PER_SEC)
#define CLOCK_SYNC_DEFAULT_READINGS              5U
#define CLOCK_SYNC_DEFAULT_STEP_NS               (1LL * NSEC_PER_SEC)
#define CLOCK_SYNC_DEFAULT_LOCK_NS               (10LL * NSEC_PER_MSEC)
#define CLOCK_SYNC_DEFAULT_OUTLIER_NS            (100LL * NSEC_PER_MSEC)
#define CLOCK_SYNC_DEFAULT_UNCERTAINTY_MARGIN_NS (10LL * NSEC_PER_MSEC)
#define CLOCK_SYNC_DEFAULT_SOURCE_TIMEOUT_NS     (3LL * NSEC_PER_SEC)
#define CLOCK_SYNC_DEFAULT_HOLDOVER_NS           (3LL * NSEC_PER_SEC)
#define CLOCK_SYNC_DEFAULT_LOCK_SAMPLES          3U
#define CLOCK_SYNC_DEFAULT_OUTLIER_SAMPLES       2U

/*
 * Longest accepted update interval. The bound keeps the configured interval
 * well inside the range that the kernel timeout conversion can represent, and
 * a synchronization instance that samples less than once per hour cannot hold
 * a useful correction anyway.
 */
#define CLOCK_SYNC_MAX_UPDATE_NS (3600LL * NSEC_PER_SEC)

struct clock_sync_sampling_result {
	struct precision_time_observation observation;
	precision_time_t bracket_ns;
	int error;
	uint32_t source_read_failures;
	uint32_t sink_read_failures;
	bool valid;
	bool source_unavailable;
};

struct clock_sync_source_state {
	precision_time_t last_update_uptime_ns;
	bool has_last_update_uptime;
	bool lost;
};

static void clock_sync_work_handler(struct k_work *work);

static bool clock_sync_domain_valid(const struct precision_time_domain *domain)
{
	return domain != NULL && domain->type != PRECISION_TIME_DOMAIN_INVALID;
}

static precision_time_t clock_sync_abs_sat(precision_time_t value)
{
	if (value == PRECISION_TIME_MIN) {
		return PRECISION_TIME_MAX;
	}

	return value < 0 ? -value : value;
}

static int clock_sync_uptime_now(precision_time_t *now_ns)
{
	int64_t ticks = k_uptime_ticks();
	uint64_t converted_ns;

	if (ticks < 0 || now_ns == NULL) {
		return -ERANGE;
	}

	converted_ns = k_ticks_to_ns_floor64((uint64_t)ticks);
	if (converted_ns > (uint64_t)PRECISION_TIME_MAX) {
		return -ERANGE;
	}

	*now_ns = (precision_time_t)converted_ns;

	return 0;
}

static void clock_sync_status_init(struct precision_clock_sync_status *status,
				   const struct precision_clock_sync_config *config, bool running)
{
	*status = (struct precision_clock_sync_status){
		.running = running,
		.source_domain = config->pi.source_domain,
		.sink_domain = config->pi.local_domain,
		.state = PRECISION_SYNC_UNSYNCED,
		.last_action = PRECISION_DISCIPLINE_IGNORE,
	};
}

static int clock_sync_validate_caps(const struct precision_clock_caps *caps)
{
	if (caps->resolution_ns < 0 || caps->max_phase_adjust_ns < 0 ||
	    caps->min_rate_ppb > caps->max_rate_ppb) {
		return -EINVAL;
	}

	return 0;
}

static int clock_sync_min_uncertainty(const struct precision_clock_caps *source_caps,
				      const struct precision_clock_caps *sink_caps,
				      precision_time_t *min_uncertainty_ns)
{
	return precision_time_add(source_caps->resolution_ns, sink_caps->resolution_ns,
				  min_uncertainty_ns);
}

static int clock_sync_default_uncertainty(const struct precision_clock *source,
					  const struct precision_clock *sink,
					  precision_time_t *max_uncertainty_ns)
{
	struct precision_clock_caps source_caps;
	struct precision_clock_caps sink_caps;
	precision_time_t min_uncertainty_ns;
	int ret;

	ret = precision_clock_get_caps(source, &source_caps);
	if (ret < 0) {
		return ret;
	}

	ret = precision_clock_get_caps(sink, &sink_caps);
	if (ret < 0) {
		return ret;
	}

	ret = clock_sync_validate_caps(&source_caps);
	if (ret == 0) {
		ret = clock_sync_validate_caps(&sink_caps);
	}
	if (ret == 0) {
		ret = clock_sync_min_uncertainty(&source_caps, &sink_caps, &min_uncertainty_ns);
	}
	if (ret == 0) {
		ret = precision_time_add(min_uncertainty_ns,
					 CLOCK_SYNC_DEFAULT_UNCERTAINTY_MARGIN_NS,
					 max_uncertainty_ns);
	}

	return ret;
}

static int clock_sync_preflight(struct precision_clock_sync_config *config,
				struct precision_clock_caps *source_caps,
				struct precision_clock_caps *sink_caps)
{
	uint32_t required_sink_caps = PRECISION_CLOCK_CAP_READ | PRECISION_CLOCK_CAP_ADJUST_RATE;
	precision_time_t min_uncertainty_ns;
	int32_t min_rate_ppb;
	int32_t max_rate_ppb;
	int ret;

	if (config->source == NULL || config->sink == NULL || config->source == config->sink ||
	    config->update_interval_ns <= 0 ||
	    config->update_interval_ns > CLOCK_SYNC_MAX_UPDATE_NS ||
	    config->readings_per_update == 0U ||
	    !clock_sync_domain_valid(&config->source->domain) ||
	    !clock_sync_domain_valid(&config->sink->domain) ||
	    precision_time_domain_equal(&config->source->domain, &config->sink->domain) ||
	    !precision_time_domain_equal(&config->pi.source_domain, &config->source->domain) ||
	    !precision_time_domain_equal(&config->pi.local_domain, &config->sink->domain)) {
		return -EINVAL;
	}

	ret = precision_clock_get_caps(config->source, source_caps);
	if (ret < 0) {
		return ret;
	}

	ret = precision_clock_get_caps(config->sink, sink_caps);
	if (ret < 0) {
		return ret;
	}

	ret = clock_sync_validate_caps(source_caps);
	if (ret < 0) {
		return ret;
	}

	ret = clock_sync_validate_caps(sink_caps);
	if (ret < 0) {
		return ret;
	}

	ret = clock_sync_min_uncertainty(source_caps, sink_caps, &min_uncertainty_ns);
	if (ret < 0) {
		return ret;
	}
	if (config->pi.max_uncertainty_ns > 0 &&
	    config->pi.max_uncertainty_ns < min_uncertainty_ns) {
		return -ERANGE;
	}

	if ((source_caps->flags & PRECISION_CLOCK_CAP_READ) == 0U ||
	    (sink_caps->flags & required_sink_caps) != required_sink_caps) {
		return -ENOTSUP;
	}

	if (config->pi.step_threshold_ns > 0 &&
	    (sink_caps->flags & (PRECISION_CLOCK_CAP_ADJUST_PHASE | PRECISION_CLOCK_CAP_SET)) ==
		    0U) {
		return -ENOTSUP;
	}

	min_rate_ppb = MAX(config->pi.min_rate_ppb, sink_caps->min_rate_ppb);
	max_rate_ppb = MIN(config->pi.max_rate_ppb, sink_caps->max_rate_ppb);
	if (min_rate_ppb > max_rate_ppb) {
		return -ERANGE;
	}

	config->pi.min_rate_ppb = min_rate_ppb;
	config->pi.max_rate_ppb = max_rate_ppb;

	return 0;
}

static int clock_sync_sample(const struct precision_clock_sync_config *config,
			     const struct precision_clock_caps *source_caps,
			     const struct precision_clock_caps *sink_caps,
			     struct clock_sync_sampling_result *sample)
{
	uint32_t source_successes = 0U;

	memset(sample, 0, sizeof(*sample));
	sample->error = -EAGAIN;

	for (uint8_t i = 0U; i < config->readings_per_update; i++) {
		struct precision_time_point sink_before;
		struct precision_time_point sink_after;
		struct precision_time_point source;
		precision_time_t half_bracket_ns;
		precision_time_t uncertainty_ns;
		precision_time_t midpoint_ns;
		precision_time_t bracket_ns;
		int ret;

		ret = precision_clock_read(config->sink, &sink_before);
		if (ret < 0) {
			sample->error = ret;
			sample->sink_read_failures++;
			continue;
		}

		ret = precision_clock_read(config->source, &source);
		if (ret < 0) {
			sample->source_read_failures++;
			sample->error = ret;
			continue;
		}
		source_successes++;

		ret = precision_clock_read(config->sink, &sink_after);
		if (ret < 0) {
			sample->error = ret;
			sample->sink_read_failures++;
			continue;
		}

		ret = precision_time_sub(sink_after.time, sink_before.time, &bracket_ns);
		if (ret < 0 || bracket_ns < 0) {
			sample->error = ret < 0 ? ret : -ESTALE;
			continue;
		}

		half_bracket_ns = bracket_ns / 2;
		ret = precision_time_add(sink_before.time, half_bracket_ns, &midpoint_ns);
		if (ret < 0) {
			sample->error = ret;
			continue;
		}

		ret = precision_time_add(half_bracket_ns, source_caps->resolution_ns,
					 &uncertainty_ns);
		if (ret == 0) {
			ret = precision_time_add(uncertainty_ns, sink_caps->resolution_ns,
						 &uncertainty_ns);
		}
		if (ret < 0) {
			sample->error = ret;
			continue;
		}

		if (!sample->valid || bracket_ns < sample->bracket_ns) {
			sample->observation = (struct precision_time_observation){
				.source = source,
				.uncertainty_ns = uncertainty_ns,
				.flags = PRECISION_OBSERVATION_SOURCE_VALID |
					 PRECISION_OBSERVATION_LOCAL_VALID,
			};
			sample->observation.local.time = midpoint_ns;
			sample->observation.local.domain = config->pi.local_domain;
			sample->bracket_ns = bracket_ns;
			sample->valid = true;
			sample->error = 0;
		}
	}

	if (!sample->valid && source_successes == 0U && sample->source_read_failures > 0U) {
		sample->source_unavailable = true;
	}

	return sample->error;
}

static int clock_sync_source_age(const struct clock_sync_source_state *source_state,
				 precision_time_t now_uptime_ns, precision_time_t *age_ns)
{
	int ret;

	if (!source_state->has_last_update_uptime) {
		*age_ns = 0;
		return 0;
	}

	ret = precision_time_sub(now_uptime_ns, source_state->last_update_uptime_ns, age_ns);
	if (ret < 0) {
		*age_ns = PRECISION_TIME_MAX;
		return ret;
	}

	if (*age_ns < 0) {
		*age_ns = 0;
	}

	return 0;
}

static int clock_sync_check_source_timeout(struct precision_pi_discipline *discipline,
					   precision_time_t source_age_ns,
					   struct precision_discipline_result *result)
{
	struct precision_pi_discipline normalized;
	precision_time_t last_update_ns;
	int ret;

	if (!discipline->has_last_update) {
		return precision_pi_check_source_timeout(discipline, 0, result);
	}

	/* Source freshness is measured on the monotonic uptime timeline. Normalize
	 * the PI timestamp so its timeout policy can consume the already-computed
	 * age without combining the sink and uptime time domains.
	 */
	normalized = *discipline;
	last_update_ns = normalized.last_update_ns;
	normalized.last_update_ns = 0;
	ret = precision_pi_check_source_timeout(&normalized, source_age_ns, result);
	if (normalized.has_last_update) {
		normalized.last_update_ns = last_update_ns;
	}
	*discipline = normalized;

	return ret;
}

static void clock_sync_fault(struct precision_pi_discipline *discipline,
			     struct precision_clock_sync_status *status, int error)
{
	precision_pi_fault(discipline);
	status->state = PRECISION_SYNC_FAULT;
	status->running = false;
	status->last_error = error;
	status->control_failures++;
}

static bool clock_sync_phase_is_supported(const struct precision_clock_caps *sink_caps,
					  precision_time_t phase_ns)
{
	if ((sink_caps->flags & PRECISION_CLOCK_CAP_ADJUST_PHASE) == 0U ||
	    phase_ns == PRECISION_TIME_MIN) {
		return false;
	}

	return sink_caps->max_phase_adjust_ns == 0 ||
	       clock_sync_abs_sat(phase_ns) <= sink_caps->max_phase_adjust_ns;
}

static int clock_sync_step_sink(const struct precision_clock_sync_config *config,
				const struct precision_clock_caps *sink_caps,
				precision_time_t phase_ns,
				struct precision_clock_sync_status *status)
{
	struct precision_time_point sink_time;
	int phase_ret = -ERANGE;
	int ret;

	ret = precision_clock_adjust_rate(config->sink, 0);
	if (ret < 0) {
		return ret;
	}
	status->applied_rate_ppb = 0;

	if (clock_sync_phase_is_supported(sink_caps, phase_ns)) {
		phase_ret = precision_clock_adjust_phase(config->sink, phase_ns);
		if (phase_ret == 0 || (phase_ret != -ENOTSUP && phase_ret != -ERANGE)) {
			return phase_ret;
		}
	}

	if ((sink_caps->flags & PRECISION_CLOCK_CAP_SET) == 0U) {
		return phase_ret;
	}

	ret = precision_clock_read(config->sink, &sink_time);
	if (ret < 0) {
		status->sink_read_failures++;
		return ret;
	}

	ret = precision_time_add(sink_time.time, phase_ns, &sink_time.time);
	if (ret < 0) {
		return ret;
	}

	return precision_clock_set(config->sink, &sink_time);
}

static int clock_sync_apply_result(const struct precision_clock_sync_config *config,
				   const struct precision_clock_caps *sink_caps,
				   struct precision_pi_discipline *discipline,
				   const struct precision_discipline_result *result,
				   struct precision_clock_sync_status *status)
{
	struct precision_pi_status pi_status;
	int ret = 0;

	status->last_action = result->action;
	status->offset_ns = result->offset_ns;

	switch (result->action) {
	case PRECISION_DISCIPLINE_STEP:
		ret = clock_sync_step_sink(config, sink_caps, result->phase_correction_ns, status);
		if (ret == 0) {
			precision_pi_reset(discipline);
		}
		break;
	case PRECISION_DISCIPLINE_ADJUST_RATE:
		ret = precision_clock_adjust_rate(config->sink, result->rate_ppb);
		if (ret == 0) {
			status->applied_rate_ppb = result->rate_ppb;
		}
		break;
	case PRECISION_DISCIPLINE_RESET:
		ret = precision_clock_adjust_rate(config->sink, 0);
		if (ret == 0) {
			status->applied_rate_ppb = 0;
			precision_pi_reset(discipline);
		}
		break;
	case PRECISION_DISCIPLINE_IGNORE:
		break;
	}

	if (ret < 0) {
		clock_sync_fault(discipline, status, ret);
		return ret;
	}

	ret = precision_pi_get_status(discipline, &pi_status);
	if (ret < 0) {
		clock_sync_fault(discipline, status, ret);
		return ret;
	}
	status->state = pi_status.state;

	return 0;
}

static bool clock_sync_generation_is_current(struct precision_clock_sync *sync, uint32_t generation)
{
	bool current;

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	current = sync->initialized && sync->status.running && sync->generation == generation;
	(void)k_mutex_unlock(&sync->lock);

	return current;
}

static bool clock_sync_commit(struct precision_clock_sync *sync, uint32_t generation,
			      const struct precision_pi_discipline *discipline,
			      const struct precision_clock_sync_status *status,
			      const struct clock_sync_source_state *source_state)
{
	bool committed = false;

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (sync->initialized && sync->status.running && sync->generation == generation) {
		sync->discipline = *discipline;
		sync->status = *status;
		sync->last_update_uptime_ns = source_state->last_update_uptime_ns;
		sync->has_last_update_uptime = source_state->has_last_update_uptime;
		sync->source_lost = source_state->lost;
		committed = true;
	}
	(void)k_mutex_unlock(&sync->lock);

	return committed;
}

static int clock_sync_reschedule_locked(struct precision_clock_sync *sync, k_timeout_t delay)
{
	int ret;

	ret = k_work_reschedule(&sync->work, delay);
	if (ret < 0) {
		sync->status.running = false;
		sync->status.last_error = ret;
		sync->generation++;
		return ret;
	}

	return 0;
}

static int clock_sync_process_source_timeout(const struct precision_clock_sync_config *config,
					     const struct precision_clock_caps *sink_caps,
					     precision_time_t now_uptime_ns, int reported_error,
					     struct precision_pi_discipline *discipline,
					     struct precision_clock_sync_status *status,
					     struct clock_sync_source_state *source_state)
{
	struct precision_discipline_result result;
	struct precision_pi_status pi_status;
	int age_ret;
	int ret;

	age_ret = clock_sync_source_age(source_state, now_uptime_ns, &status->source_age_ns);
	ret = age_ret < 0
		      ? age_ret
		      : clock_sync_check_source_timeout(discipline, status->source_age_ns, &result);
	if (age_ret < 0 || ret == -ERANGE) {
		precision_pi_reset(discipline);
		result = (struct precision_discipline_result){
			.action = PRECISION_DISCIPLINE_RESET,
			.state = PRECISION_SYNC_UNSYNCED,
			.offset_ns = status->offset_ns,
		};
		ret = -ESTALE;
	}

	if (ret == -ESTALE) {
		int control_ret;

		status->last_action = result.action;
		status->offset_ns = result.offset_ns;
		control_ret =
			clock_sync_apply_result(config, sink_caps, discipline, &result, status);
		if (control_ret < 0) {
			return control_ret;
		}
	} else if (ret != 0 && ret != -EAGAIN) {
		return ret;
	}

	if (precision_pi_get_status(discipline, &pi_status) == 0) {
		status->state = pi_status.state;
	}

	status->last_error = reported_error;

	return reported_error;
}

static int clock_sync_process_observation(const struct precision_clock_sync_config *config,
					  const struct precision_clock_caps *sink_caps,
					  const struct precision_time_observation *observation,
					  precision_time_t now_uptime_ns,
					  struct precision_pi_discipline *discipline,
					  struct precision_clock_sync_status *status,
					  struct clock_sync_source_state *source_state)
{
	struct precision_discipline_result result;
	struct precision_pi_discipline recovered_discipline;
	struct precision_pi_discipline *candidate = discipline;
	bool recovering = source_state->lost;
	bool accepted;
	int ret;

	if (recovering) {
		recovered_discipline = *discipline;
		precision_pi_reset(&recovered_discipline);
		candidate = &recovered_discipline;
	}

	status->sampling_uncertainty_ns = observation->uncertainty_ns;
	ret = precision_pi_process(candidate, observation, &result);
	status->last_action = result.action;
	status->offset_ns = result.offset_ns;
	accepted = ret == 0 && (result.action == PRECISION_DISCIPLINE_ADJUST_RATE ||
				result.action == PRECISION_DISCIPLINE_STEP);

	if (accepted) {
		if (recovering) {
			*discipline = recovered_discipline;
			source_state->lost = false;
		}

		status->accepted_observations++;
		status->source_age_ns = 0;
		ret = clock_sync_apply_result(config, sink_caps, discipline, &result, status);
		if (ret == 0) {
			status->last_error = 0;
			source_state->last_update_uptime_ns = now_uptime_ns;
			source_state->has_last_update_uptime = true;
		}

		return ret;
	}

	status->rejected_observations++;
	if (!recovering && ret == 0) {
		ret = clock_sync_apply_result(config, sink_caps, discipline, &result, status);
		if (ret < 0) {
			return ret;
		}
	}

	return clock_sync_process_source_timeout(config, sink_caps, now_uptime_ns, ret, discipline,
						 status, source_state);
}

static void clock_sync_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct precision_clock_sync *sync = CONTAINER_OF(dwork, struct precision_clock_sync, work);
	precision_time_t update_interval_ns;
	uint32_t generation;

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (!sync->initialized || !sync->status.running) {
		(void)k_mutex_unlock(&sync->lock);
		return;
	}
	generation = sync->generation;
	update_interval_ns = sync->config.update_interval_ns;
	(void)k_mutex_unlock(&sync->lock);

	(void)precision_clock_sync_run_once(sync, generation);

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (sync->initialized && sync->status.running && sync->generation == generation) {
		(void)clock_sync_reschedule_locked(sync, K_NSEC(update_interval_ns));
	}
	(void)k_mutex_unlock(&sync->lock);
}

int precision_clock_sync_config_default(struct precision_clock_sync_config *config,
					const struct precision_clock *source,
					const struct precision_clock *sink)
{
	precision_time_t max_uncertainty_ns;
	int ret;

	if (config == NULL || source == NULL || sink == NULL) {
		return -EINVAL;
	}

	ret = clock_sync_default_uncertainty(source, sink, &max_uncertainty_ns);
	if (ret < 0) {
		return ret;
	}

	*config = (struct precision_clock_sync_config){
		.source = source,
		.sink = sink,
		.update_interval_ns = CLOCK_SYNC_DEFAULT_UPDATE_NS,
		.readings_per_update = CLOCK_SYNC_DEFAULT_READINGS,
	};
	config->pi = (struct precision_pi_config){
		.source_domain = source->domain,
		.local_domain = sink->domain,
		.target_offset_ns = 0,
		.step_threshold_ns = CLOCK_SYNC_DEFAULT_STEP_NS,
		.lock_threshold_ns = CLOCK_SYNC_DEFAULT_LOCK_NS,
		.outlier_threshold_ns = CLOCK_SYNC_DEFAULT_OUTLIER_NS,
		.max_uncertainty_ns = max_uncertainty_ns,
		.source_timeout_ns = CLOCK_SYNC_DEFAULT_SOURCE_TIMEOUT_NS,
		.holdover_ns = CLOCK_SYNC_DEFAULT_HOLDOVER_NS,
		.lock_sample_count = CLOCK_SYNC_DEFAULT_LOCK_SAMPLES,
		.outlier_sample_count = CLOCK_SYNC_DEFAULT_OUTLIER_SAMPLES,
		.min_rate_ppb = INT32_MIN,
		.max_rate_ppb = INT32_MAX,
		.kp_num = CONFIG_PRECISION_TIMING_PI_KP,
		.ki_num = CONFIG_PRECISION_TIMING_PI_KI,
		.gain_den = PRECISION_PI_GAIN_DEN,
	};

	return 0;
}

/*
 * Clear everything an initialized instance owns, except the embedded locks
 * and the lifecycle incarnation.
 *
 * The locks must survive re-initialization because a lifecycle call may still
 * be blocked on the lifecycle lock. A k_mutex embeds a wait queue whose empty
 * state is a self-referencing list head, so it can be neither wiped nor copied
 * while a waiter exists.
 */
static void clock_sync_reset_state(struct precision_clock_sync *sync)
{
	memset(&sync->work, 0, sizeof(sync->work));
	memset(&sync->config, 0, sizeof(sync->config));
	memset(&sync->source_caps, 0, sizeof(sync->source_caps));
	memset(&sync->sink_caps, 0, sizeof(sync->sink_caps));
	memset(&sync->discipline, 0, sizeof(sync->discipline));
	memset(&sync->status, 0, sizeof(sync->status));
	sync->last_update_uptime_ns = 0;
	sync->generation = 0;
	sync->source_lost = false;
	sync->has_last_update_uptime = false;
}

/*
 * Capture the instance incarnation before waiting for the lifecycle lock.
 * A call that is descheduled in that window must not resume on an instance
 * that has since been released and initialized again.
 */
static int clock_sync_lock_lifecycle(struct precision_clock_sync *sync, uint32_t *incarnation)
{
	int ret;

	ret = k_mutex_lock(&sync->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	*incarnation = sync->incarnation;
	(void)k_mutex_unlock(&sync->lock);

	return k_mutex_lock(&sync->lifecycle_lock, K_FOREVER);
}

/*
 * A clock adapter is invoked by the synchronization work handler. A lifecycle
 * call made recursively by such an adapter cannot wait for that same handler
 * to finish.
 */
static bool clock_sync_cancel_would_deadlock(struct precision_clock_sync *sync)
{
	return k_current_get() == k_sys_work_q.thread_id &&
	       (k_work_delayable_busy_get(&sync->work) & K_WORK_RUNNING) != 0U;
}

/*
 * Synchronously cancel without a stack-allocated k_work_sync. Such an object
 * is not coherent on the thread stack when CONFIG_KERNEL_COHERENCE is set.
 * The lifecycle lock and cleared running flag prevent any new scheduling
 * while the busy state is polled.
 */
static void clock_sync_cancel_work(struct precision_clock_sync *sync)
{
	(void)k_work_cancel_delayable(&sync->work);

	while (k_work_delayable_busy_get(&sync->work) != 0) {
		k_sleep(K_TICKS(1));
	}
}

int precision_clock_sync_init(struct precision_clock_sync *sync,
			      const struct precision_clock_sync_config *config)
{
	struct precision_clock_sync_config effective_config;
	struct precision_clock_caps source_caps;
	struct precision_clock_caps sink_caps;
	struct precision_pi_discipline discipline;
	bool reinit;
	int ret;

	if (sync == NULL || config == NULL) {
		return -EINVAL;
	}

	/*
	 * Re-initialization has to serialize against lifecycle calls that are
	 * already blocked on the lifecycle lock, and must never wipe the locks
	 * themselves while a thread is waiting on them. The first
	 * initialization cannot take the lock because it does not exist yet;
	 * callers own that window, as they do for any zero-initialized object.
	 */
	reinit = sync->locks_ready;
	if (reinit) {
		ret = k_mutex_lock(&sync->lifecycle_lock, K_FOREVER);
		if (ret < 0) {
			return ret;
		}
	}

	if (sync->initialized) {
		if (reinit) {
			(void)k_mutex_unlock(&sync->lifecycle_lock);
		}
		return -EBUSY;
	}

	effective_config = *config;
	ret = clock_sync_preflight(&effective_config, &source_caps, &sink_caps);
	if (ret == 0) {
		ret = precision_pi_init(&discipline, &effective_config.pi);
	}
	if (ret < 0) {
		if (reinit) {
			(void)k_mutex_unlock(&sync->lifecycle_lock);
		}
		return ret;
	}

	if (reinit) {
		clock_sync_reset_state(sync);
	} else {
		memset(sync, 0, sizeof(*sync));

		ret = k_mutex_init(&sync->lifecycle_lock);
		if (ret == 0) {
			ret = k_mutex_init(&sync->lock);
		}
		if (ret < 0) {
			return ret;
		}

		sync->locks_ready = true;
	}

	sync->config = effective_config;
	sync->source_caps = source_caps;
	sync->sink_caps = sink_caps;
	sync->discipline = discipline;
	clock_sync_status_init(&sync->status, &effective_config, false);
	k_work_init_delayable(&sync->work, clock_sync_work_handler);

	/*
	 * Publish the instance under the state lock the readers use. The
	 * release pairs with their acquire, so a reader that observes the
	 * flag also observes the state written above it on targets that do
	 * not order stores.
	 */
	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	sync->incarnation++;
	sync->initialized = true;
	(void)k_mutex_unlock(&sync->lock);

	if (reinit) {
		(void)k_mutex_unlock(&sync->lifecycle_lock);
	}

	return 0;
}

int precision_clock_sync_start(struct precision_clock_sync *sync)
{
	uint32_t incarnation;
	int ret;

	if (sync == NULL || !sync->locks_ready) {
		return -EINVAL;
	}

	ret = clock_sync_lock_lifecycle(sync, &incarnation);
	if (ret < 0) {
		return ret;
	}

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (!sync->initialized || sync->incarnation != incarnation) {
		ret = -EINVAL;
	} else if (sync->status.running) {
		ret = -EALREADY;
	} else if (sync->status.state == PRECISION_SYNC_FAULT) {
		ret = -EIO;
	} else {
		sync->generation++;
		sync->status.running = true;
		ret = clock_sync_reschedule_locked(sync, K_NO_WAIT);
	}
	(void)k_mutex_unlock(&sync->lock);
	(void)k_mutex_unlock(&sync->lifecycle_lock);

	return ret;
}

int precision_clock_sync_stop(struct precision_clock_sync *sync)
{
	uint32_t incarnation;
	int ret;

	if (sync == NULL || !sync->locks_ready) {
		return -EINVAL;
	}

	ret = clock_sync_lock_lifecycle(sync, &incarnation);
	if (ret < 0) {
		return ret;
	}

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (!sync->initialized || sync->incarnation != incarnation) {
		(void)k_mutex_unlock(&sync->lock);
		(void)k_mutex_unlock(&sync->lifecycle_lock);
		return -EINVAL;
	}
	if (clock_sync_cancel_would_deadlock(sync)) {
		(void)k_mutex_unlock(&sync->lock);
		(void)k_mutex_unlock(&sync->lifecycle_lock);
		return -EDEADLK;
	}

	if (sync->status.running) {
		sync->status.running = false;
		sync->generation++;
	}
	(void)k_mutex_unlock(&sync->lock);
	clock_sync_cancel_work(sync);
	(void)k_mutex_unlock(&sync->lifecycle_lock);

	return 0;
}

int precision_clock_sync_deinit(struct precision_clock_sync *sync)
{
	uint32_t incarnation;
	int ret;

	if (sync == NULL) {
		return -EINVAL;
	}

	if (!sync->locks_ready) {
		return 0;
	}

	ret = clock_sync_lock_lifecycle(sync, &incarnation);
	if (ret < 0) {
		return ret;
	}

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (!sync->initialized || sync->incarnation != incarnation) {
		(void)k_mutex_unlock(&sync->lock);
		(void)k_mutex_unlock(&sync->lifecycle_lock);
		return 0;
	}
	if (clock_sync_cancel_would_deadlock(sync)) {
		(void)k_mutex_unlock(&sync->lock);
		(void)k_mutex_unlock(&sync->lifecycle_lock);
		return -EDEADLK;
	}

	/*
	 * Clear the initialized flag before the cancel, which can block for a
	 * whole update. A lifecycle call already waiting on the lifecycle lock
	 * then observes the release and fails instead of resuming work on an
	 * instance the caller considers gone.
	 */
	sync->initialized = false;
	sync->status.running = false;
	sync->generation++;
	(void)k_mutex_unlock(&sync->lock);

	clock_sync_cancel_work(sync);

	(void)k_mutex_unlock(&sync->lifecycle_lock);

	return 0;
}

int precision_clock_sync_reset(struct precision_clock_sync *sync)
{
	uint32_t incarnation;
	bool was_running;
	int ret;

	if (sync == NULL || !sync->locks_ready) {
		return -EINVAL;
	}

	ret = clock_sync_lock_lifecycle(sync, &incarnation);
	if (ret < 0) {
		return ret;
	}

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (!sync->initialized || sync->incarnation != incarnation) {
		(void)k_mutex_unlock(&sync->lock);
		(void)k_mutex_unlock(&sync->lifecycle_lock);
		return -EINVAL;
	}
	if (clock_sync_cancel_would_deadlock(sync)) {
		(void)k_mutex_unlock(&sync->lock);
		(void)k_mutex_unlock(&sync->lifecycle_lock);
		return -EDEADLK;
	}

	was_running = sync->status.running;
	sync->status.running = false;
	sync->generation++;
	(void)k_mutex_unlock(&sync->lock);
	clock_sync_cancel_work(sync);

	ret = precision_clock_adjust_rate(sync->config.sink, 0);

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (ret < 0) {
		clock_sync_fault(&sync->discipline, &sync->status, ret);
	} else {
		precision_pi_reset(&sync->discipline);
		clock_sync_status_init(&sync->status, &sync->config, was_running);
		sync->last_update_uptime_ns = 0;
		sync->has_last_update_uptime = false;
		sync->source_lost = false;
		sync->generation++;
		if (was_running) {
			ret = clock_sync_reschedule_locked(sync, K_NO_WAIT);
		}
	}
	(void)k_mutex_unlock(&sync->lock);
	(void)k_mutex_unlock(&sync->lifecycle_lock);

	return ret;
}

int precision_clock_sync_get_status(struct precision_clock_sync *sync,
				    struct precision_clock_sync_status *status)
{
	int ret;

	if (sync == NULL || status == NULL || !sync->locks_ready) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&sync->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	if (!sync->initialized) {
		(void)k_mutex_unlock(&sync->lock);
		return -EINVAL;
	}

	*status = sync->status;
	(void)k_mutex_unlock(&sync->lock);

	return 0;
}

int precision_clock_sync_run_once(struct precision_clock_sync *sync, uint32_t generation)
{
	precision_time_t now_uptime_ns;
	int ret;

	ret = clock_sync_uptime_now(&now_uptime_ns);
	if (ret < 0) {
		return ret;
	}

	return precision_clock_sync_run_once_at(sync, generation, now_uptime_ns);
}

int precision_clock_sync_run_once_at(struct precision_clock_sync *sync, uint32_t generation,
				     precision_time_t now_uptime_ns)
{
	struct precision_clock_sync_config config;
	struct precision_clock_sync_status status;
	struct precision_clock_caps source_caps;
	struct precision_clock_caps sink_caps;
	struct precision_pi_discipline discipline;
	struct clock_sync_sampling_result sample;
	struct clock_sync_source_state source_state;
	int ret;

	if (sync == NULL || !sync->locks_ready || now_uptime_ns < 0) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&sync->lock, K_FOREVER);
	if (!sync->initialized) {
		(void)k_mutex_unlock(&sync->lock);
		return -EINVAL;
	}
	if (!sync->status.running || sync->generation != generation) {
		(void)k_mutex_unlock(&sync->lock);
		return -ECANCELED;
	}
	config = sync->config;
	status = sync->status;
	source_caps = sync->source_caps;
	sink_caps = sync->sink_caps;
	discipline = sync->discipline;
	source_state = (struct clock_sync_source_state){
		.last_update_uptime_ns = sync->last_update_uptime_ns,
		.has_last_update_uptime = sync->has_last_update_uptime,
		.lost = sync->source_lost,
	};
	(void)k_mutex_unlock(&sync->lock);

	ret = clock_sync_sample(&config, &source_caps, &sink_caps, &sample);
	status.source_read_failures += sample.source_read_failures;
	status.sink_read_failures += sample.sink_read_failures;

	if (sample.valid) {
		if (!clock_sync_generation_is_current(sync, generation)) {
			return -ECANCELED;
		}
		ret = clock_sync_process_observation(&config, &sink_caps, &sample.observation,
						     now_uptime_ns, &discipline, &status,
						     &source_state);
	} else if (sample.source_unavailable) {
		if (!clock_sync_generation_is_current(sync, generation)) {
			return -ECANCELED;
		}
		source_state.lost = true;
		status.last_action = PRECISION_DISCIPLINE_IGNORE;
		ret = clock_sync_process_source_timeout(&config, &sink_caps, now_uptime_ns,
							sample.error, &discipline, &status,
							&source_state);
	} else {
		if (!clock_sync_generation_is_current(sync, generation)) {
			return -ECANCELED;
		}
		status.rejected_observations++;
		status.last_action = PRECISION_DISCIPLINE_IGNORE;
		ret = clock_sync_process_source_timeout(&config, &sink_caps, now_uptime_ns, ret,
							&discipline, &status, &source_state);
	}

	if (!clock_sync_commit(sync, generation, &discipline, &status, &source_state)) {
		return -ECANCELED;
	}

	return ret;
}
