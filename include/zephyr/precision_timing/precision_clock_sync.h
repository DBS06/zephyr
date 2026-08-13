/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Fixed source-to-sink precision clock synchronization.
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_CLOCK_SYNC_H_
#define ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_CLOCK_SYNC_H_

#include <zephyr/kernel.h>
#include <zephyr/precision_timing/precision_clock.h>
#include <zephyr/precision_timing/precision_pi.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Precision clock synchronization
 * @defgroup precision_clock_sync Precision clock synchronization
 * @since 4.5
 * @version 0.1.0
 * @ingroup precision_timing
 * @{
 */

/** Configuration of a fixed precision clock synchronization instance. */
struct precision_clock_sync_config {
	/** Clock that supplies synchronization time. */
	const struct precision_clock *source;
	/** Clock exclusively controlled by this synchronization instance. */
	const struct precision_clock *sink;
	/** PI policy, domains, target offset, and adjustment limits. */
	struct precision_pi_config pi;
	/** Interval between synchronization updates in nanoseconds. */
	precision_time_t update_interval_ns;
	/** Number of bracketed readings attempted per update. */
	uint8_t readings_per_update;
};

/** Observable status of a precision clock synchronization instance. */
struct precision_clock_sync_status {
	/** Whether periodic synchronization is active. */
	bool running;
	/** Fixed source clock domain. */
	struct precision_time_domain source_domain;
	/** Fixed sink clock domain. */
	struct precision_time_domain sink_domain;
	/** Current discipline state. */
	enum precision_sync_state state;
	/** Most recent discipline action. */
	enum precision_discipline_action last_action;
	/** Most recent target-adjusted source-to-sink offset. */
	precision_time_t offset_ns;
	/** Rate adjustment most recently applied to the sink. */
	int32_t applied_rate_ppb;
	/** Monotonic age of the last accepted source observation. */
	precision_time_t source_age_ns;
	/** Uncertainty of the most recent selected sample. */
	precision_time_t sampling_uncertainty_ns;
	/** Number of accepted observations. */
	uint32_t accepted_observations;
	/** Number of rejected observations. */
	uint32_t rejected_observations;
	/** Number of failed source reads. */
	uint32_t source_read_failures;
	/** Number of failed sink reads. */
	uint32_t sink_read_failures;
	/** Number of failed sink control operations. */
	uint32_t control_failures;
	/** Most recent operation error, or zero after a successful update. */
	int last_error;
};

/** Caller-owned state for a precision clock synchronization instance.
 *
 * Zero-initialize the structure before its first precision_clock_sync_init()
 * call, then do not copy or inspect it afterward. Use the synchronization
 * service APIs to control the instance and retrieve status.
 */
struct precision_clock_sync {
	/** Lock serializing lifecycle operations. */
	struct k_mutex lifecycle_lock;
	/** Lock protecting mutable engine state and status. */
	struct k_mutex lock;
	/** Delayable system-workqueue item used by the service. */
	struct k_work_delayable work;
	/** Configuration copied and bounded during initialization. */
	struct precision_clock_sync_config config;
	/** Source capabilities captured during initialization. */
	struct precision_clock_caps source_caps;
	/** Sink capabilities captured during initialization. */
	struct precision_clock_caps sink_caps;
	/** PI discipline controlled by the synchronization engine. */
	struct precision_pi_discipline discipline;
	/** Status snapshot protected by @c lock. */
	struct precision_clock_sync_status status;
	/** Monotonic uptime of the last accepted observation. */
	precision_time_t last_update_uptime_ns;
	/** Lifecycle incarnation used to reject calls stale across reinitialization. */
	uint32_t incarnation;
	/** Generation used to discard stale work. */
	uint32_t generation;
	/** Whether source recovery must start with fresh PI state. */
	bool source_lost;
	/** Whether @c last_update_uptime_ns is valid. */
	bool has_last_update_uptime;
	/** Whether initialization completed successfully. */
	bool initialized;
	/**
	 * Whether the embedded locks have been constructed.
	 *
	 * Set by the first successful initialization and never cleared, so
	 * that re-initialization can serialize against lifecycle calls that
	 * are already blocked on @c lifecycle_lock.
	 */
	bool locks_ready;
};

/**
 * @brief Populate the default fixed-clock synchronization policy.
 *
 * The default policy uses one-second updates, five bracketed readings, a
 * one-second step threshold, the configured PI gains, and bounded source
 * holdover. Its maximum sampling uncertainty is the sum of the advertised
 * source and sink resolutions plus a 10-millisecond sampling margin. The
 * desired sink time is source time plus @c config.pi.target_offset_ns, which
 * defaults to zero.
 *
 * @param config Configuration to populate.
 * @param source Fixed source clock.
 * @param sink Fixed sink clock controlled exclusively by the service.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument is null.
 * @retval -ERANGE if clock capabilities cannot represent the default policy.
 * @return A clock-provider error on failure.
 */
int precision_clock_sync_config_default(struct precision_clock_sync_config *config,
					const struct precision_clock *source,
					const struct precision_clock *sink);

/**
 * @brief Initialize a fixed-clock synchronization instance.
 *
 * The synchronization state must be zero-initialized before its first call.
 * The configuration is copied. Initialization validates clock identities and
 * capabilities and intersects PI rate limits with the sink limits.
 *
 * Release an initialized instance with precision_clock_sync_deinit() before
 * initializing it again with a different configuration.
 * Do not invoke another synchronization-service API on @p sync until its
 * first initialization call has returned. Subsequent lifecycle operations
 * and reinitialization are serialized by the instance.
 *
 * Call this function from thread context.
 *
 * @param sync Caller-owned synchronization state.
 * @param config Configuration to copy and validate.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument or policy is invalid.
 * @retval -EBUSY if @p sync was already initialized.
 * @retval -ENOTSUP if a required clock operation is unavailable.
 * @retval -ERANGE if configured and supported rate ranges do not intersect.
 * @return A clock-provider or kernel initialization error on failure.
 */
int precision_clock_sync_init(struct precision_clock_sync *sync,
			      const struct precision_clock_sync_config *config);

/**
 * @brief Start periodic clock synchronization.
 *
 * Call this function from thread context.
 *
 * @param sync Initialized synchronization instance.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p sync is invalid.
 * @retval -EALREADY if the instance is already running.
 * @retval -EIO if the instance is faulted and must be reset.
 * @return A work scheduling error on failure.
 */
int precision_clock_sync_start(struct precision_clock_sync *sync);

/**
 * @brief Stop periodic clock synchronization synchronously.
 *
 * Stopping an already stopped instance succeeds. The last applied sink rate
 * is retained.
 *
 * Call this function from thread context.
 *
 * @param sync Initialized synchronization instance.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p sync is invalid.
 * @retval -EDEADLK if called recursively by a clock adapter from the service
 * work handler.
 */
int precision_clock_sync_stop(struct precision_clock_sync *sync);

/**
 * @brief Release a synchronization instance so that it can be reconfigured.
 *
 * Stops periodic synchronization, waits for any in-flight update to complete,
 * and marks the instance uninitialized so that
 * precision_clock_sync_init() can be called again with a new configuration.
 * Deinitializing an instance that was never initialized succeeds.
 *
 * The sink is left untouched and retains its last applied rate. Restore the
 * nominal rate with precision_clock_sync_reset() before this call when the
 * sink must not keep the last correction.
 *
 * A start, stop, or reset that is already in progress on another thread is
 * allowed to complete first; a call that arrives during the release fails
 * with -EINVAL rather than resuming work on a released instance.
 *
 * Call this function from thread context.
 *
 * @param sync Synchronization instance to release.
 *
 * @retval 0 on success, including when @p sync was never initialized or has
 * already been released.
 * @retval -EINVAL if @p sync is null.
 * @retval -EDEADLK if called recursively by a clock adapter from the service
 * work handler.
 */
int precision_clock_sync_deinit(struct precision_clock_sync *sync);

/**
 * @brief Reset synchronization and restore the sink to nominal rate.
 *
 * A running instance resumes acquisition after the synchronous reset.
 *
 * Call this function from thread context.
 *
 * @param sync Initialized synchronization instance.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p sync is invalid.
 * @retval -EDEADLK if called recursively by a clock adapter from the service
 * work handler.
 * @return A sink control or work scheduling error on failure.
 */
int precision_clock_sync_reset(struct precision_clock_sync *sync);

/**
 * @brief Copy synchronization status.
 *
 * Call this function from thread context.
 *
 * @param sync Initialized synchronization instance.
 * @param status Destination for the coherent status snapshot.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument is invalid.
 */
int precision_clock_sync_get_status(struct precision_clock_sync *sync,
				    struct precision_clock_sync_status *status);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_CLOCK_SYNC_H_ */
