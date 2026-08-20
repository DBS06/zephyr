/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Protocol-neutral precision clock abstraction.
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_CLOCK_H_
#define ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_CLOCK_H_

#include <stdint.h>

#include <zephyr/precision_timing/precision_time.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Precision Clock
 * @defgroup precision_clock Precision Clock
 * @since 4.5
 * @version 0.1.0
 * @ingroup precision_timing
 * @{
 */

/** Precision clock capability flags. */
enum precision_clock_caps_flags {
	/** Clock can be read. */
	PRECISION_CLOCK_CAP_READ = BIT(0),
	/** Clock can be set to an absolute time. */
	PRECISION_CLOCK_CAP_SET = BIT(1),
	/** Clock supports phase adjustment. */
	PRECISION_CLOCK_CAP_ADJUST_PHASE = BIT(2),
	/** Clock supports rate adjustment. */
	PRECISION_CLOCK_CAP_ADJUST_RATE = BIT(3),
	/** Clock exposes at least one usable scheduled-output channel. */
	PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT = BIT(4),
};

/** Capabilities and adjustment limits of a precision clock. */
struct precision_clock_caps {
	/** Combination of @ref precision_clock_caps_flags. */
	uint32_t flags;
	/** Smallest representable clock increment in nanoseconds. */
	precision_time_t resolution_ns;
	/** Largest supported absolute phase adjustment in nanoseconds. */
	precision_time_t max_phase_adjust_ns;
	/** Minimum supported rate adjustment in parts per billion. */
	int32_t min_rate_ppb;
	/** Maximum supported rate adjustment in parts per billion. */
	int32_t max_rate_ppb;
};

/** Precision clock output edge action. */
enum precision_clock_output_edge {
	/** Drive a rising edge at the scheduled time. */
	PRECISION_CLOCK_OUTPUT_EDGE_RISING = 0,
	/** Drive a falling edge at the scheduled time. */
	PRECISION_CLOCK_OUTPUT_EDGE_FALLING = 1,
};

/** Precision clock output high-pulse width policy for periodic waveforms. */
enum precision_clock_output_width_policy {
	/** Use the provider's native default high-pulse width. */
	PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT = 0,
	/** Use the exact high-pulse width supplied in the configuration. */
	PRECISION_CLOCK_OUTPUT_WIDTH_EXACT = 1,
};

/** Kind of scheduled output configured on a precision clock output channel. */
enum precision_clock_output_kind {
	/** One-shot scheduled edge event. */
	PRECISION_CLOCK_OUTPUT_KIND_EVENT = 0,
	/** Periodic waveform. */
	PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM = 1,
};

/** Precision clock output capability flags. */
enum precision_clock_output_caps_flags {
	/** Channel can schedule one-shot output events. */
	PRECISION_CLOCK_OUTPUT_CAP_EVENT = BIT(0),
	/** Channel can generate periodic output waveforms. */
	PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM = BIT(1),
	/** Channel supports an exact programmable high-pulse width. */
	PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH = BIT(2),
	/** Channel can drive a scheduled rising edge. */
	PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING = BIT(3),
	/** Channel can drive a scheduled falling edge. */
	PRECISION_CLOCK_OUTPUT_CAP_EDGE_FALLING = BIT(4),
	/** Channel reports physical hardware-active state in its status. */
	PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE = BIT(5),
};

/**
 * @brief Capabilities and limits of a precision clock output channel.
 *
 * The period limits are meaningful only when @ref PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM
 * is advertised. The pulse-width limits are meaningful only when
 * @ref PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH is advertised. An
 * event-only channel leaves the mode-specific limits unset.
 */
struct precision_clock_output_caps {
	/** Combination of @ref precision_clock_output_caps_flags. */
	uint32_t flags;
	/** Number of output channels exposed by the clock. */
	uint32_t channel_count;
	/** Smallest representable output time increment in nanoseconds. */
	precision_time_t resolution_ns;
	/** Minimum required delay from the current time to the first edge. */
	precision_time_t min_lead_time_ns;
	/** Minimum supported waveform period in nanoseconds. */
	precision_time_t min_period_ns;
	/** Maximum supported waveform period in nanoseconds. */
	precision_time_t max_period_ns;
	/** Minimum supported programmable high-pulse width in nanoseconds. */
	precision_time_t min_pulse_width_ns;
	/** Maximum supported programmable high-pulse width in nanoseconds. */
	precision_time_t max_pulse_width_ns;
};

/** Configuration of a one-shot scheduled clock output event. */
struct precision_clock_output_event_config {
	/** Absolute time of the scheduled edge in the owning clock's domain. */
	struct precision_time_point target_time;
	/** Edge action to drive at @ref target_time. */
	enum precision_clock_output_edge edge;
};

/** Configuration of a periodic clock output waveform. */
struct precision_clock_output_waveform_config {
	/** Absolute time of the first rising edge in the owning clock's domain. */
	struct precision_time_point first_rising_time;
	/** Interval between rising edges in nanoseconds. */
	precision_time_t period_ns;
	/** High-pulse width policy applied to each period. */
	enum precision_clock_output_width_policy width_policy;
	/**
	 * High-pulse width in nanoseconds. Used only when @ref width_policy is
	 * @ref PRECISION_CLOCK_OUTPUT_WIDTH_EXACT.
	 */
	precision_time_t pulse_width_ns;
};

/** Accepted configuration held by a scheduled clock output channel. */
union precision_clock_output_configured_config {
	/** Valid when the configured kind is @ref PRECISION_CLOCK_OUTPUT_KIND_EVENT. */
	struct precision_clock_output_event_config event;
	/** Valid when the configured kind is @ref PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM. */
	struct precision_clock_output_waveform_config waveform;
};

/** Observable status of a precision clock output channel. */
struct precision_clock_output_status {
	/** Whether the provider currently holds a configuration for the channel. */
	bool configured;
	/** Configured output kind. Valid only when @ref configured is true. */
	enum precision_clock_output_kind kind;
	/** Accepted configuration. Valid only when @ref configured is true. */
	union precision_clock_output_configured_config config;
	/** Whether @ref hardware_active reflects a real hardware observation. */
	bool hardware_active_valid;
	/** Physical output activity. Meaningful only when @ref hardware_active_valid is true. */
	bool hardware_active;
};

struct precision_clock;

/** Operations implemented by a precision clock adapter. */
struct precision_clock_api {
	/** Read the current clock value. */
	int (*read)(const struct precision_clock *precision_clk, struct precision_time_point *tp);
	/** Set the current clock value. */
	int (*set)(const struct precision_clock *precision_clk,
		   const struct precision_time_point *tp);
	/** Apply a signed phase adjustment in nanoseconds. */
	int (*adjust_phase)(const struct precision_clock *precision_clk, precision_time_t phase_ns);
	/** Apply a signed rate adjustment in parts per billion. */
	int (*adjust_rate)(const struct precision_clock *precision_clk, int32_t rate_ppb);
	/** Query clock capabilities and limits. */
	int (*get_caps)(const struct precision_clock *precision_clk,
			struct precision_clock_caps *caps);
	/** Query scheduled-output channel capabilities and limits. */
	int (*get_output_caps)(const struct precision_clock *precision_clk, uint32_t channel,
			       struct precision_clock_output_caps *caps);
	/** Schedule a one-shot output event on a channel. */
	int (*output_schedule_event)(const struct precision_clock *precision_clk, uint32_t channel,
				     const struct precision_clock_output_event_config *config);
	/** Start a periodic output waveform on a channel. */
	int (*output_start_waveform)(const struct precision_clock *precision_clk, uint32_t channel,
				     const struct precision_clock_output_waveform_config *config);
	/** Stop a scheduled-output channel and validate @p channel. */
	int (*output_stop)(const struct precision_clock *precision_clk, uint32_t channel);
	/** Query scheduled-output channel status and validate @p channel. */
	int (*get_output_status)(const struct precision_clock *precision_clk, uint32_t channel,
				 struct precision_clock_output_status *status);
};

/** Protocol-neutral precision clock instance. */
struct precision_clock {
	/** Adapter operations. */
	const struct precision_clock_api *api;
	/** Adapter instance passed back to @ref precision_clock_api operations. */
	const void *adapter;
	/** Domain produced and consumed by this clock. */
	struct precision_time_domain domain;
};

/**
 * @brief Read a precision clock.
 *
 * @param precision_clk Clock to read.
 * @param tp Destination for the clock value and domain.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument or clock API is invalid.
 * @retval -ENOTSUP if the clock does not support reading.
 * @return An adapter-specific negative error code on failure.
 */
int precision_clock_read(const struct precision_clock *precision_clk,
			 struct precision_time_point *tp);

/**
 * @brief Set a precision clock to an absolute time.
 *
 * @param precision_clk Clock to set.
 * @param tp New clock value in the clock's domain.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument, API, or time domain is invalid.
 * @retval -ENOTSUP if the clock does not support setting.
 * @return An adapter-specific negative error code on failure.
 */
int precision_clock_set(const struct precision_clock *precision_clk,
			const struct precision_time_point *tp);

/**
 * @brief Apply a phase adjustment to a precision clock.
 *
 * @param precision_clk Clock to adjust.
 * @param phase_ns Signed phase adjustment in nanoseconds.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the clock or API is invalid.
 * @retval -ENOTSUP if phase adjustment is unsupported.
 * @retval -ERANGE if @p phase_ns exceeds adapter limits.
 * @return An adapter-specific negative error code on failure.
 */
int precision_clock_adjust_phase(const struct precision_clock *precision_clk,
				 precision_time_t phase_ns);

/**
 * @brief Apply a rate adjustment to a precision clock.
 *
 * @param precision_clk Clock to adjust.
 * @param rate_ppb Signed rate adjustment in parts per billion.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the clock or API is invalid.
 * @retval -ENOTSUP if rate adjustment is unsupported.
 * @retval -ERANGE if @p rate_ppb exceeds adapter limits.
 * @return An adapter-specific negative error code on failure.
 */
int precision_clock_adjust_rate(const struct precision_clock *precision_clk, int32_t rate_ppb);

/**
 * @brief Query precision clock capabilities and limits.
 *
 * @param precision_clk Clock to query.
 * @param caps Destination for capabilities and limits.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument or clock API is invalid.
 * @retval -ENOTSUP if capability reporting is unsupported.
 * @return An adapter-specific negative error code on failure.
 */
int precision_clock_get_caps(const struct precision_clock *precision_clk,
			     struct precision_clock_caps *caps);

/**
 * @brief Query the capabilities and limits of a clock output channel.
 *
 * Call from thread context. The provider operation may block.
 *
 * @param precision_clk Clock whose output channel is queried.
 * @param channel Zero-based output channel index.
 * @param caps Destination for channel capabilities and limits.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument or clock API is invalid.
 * @retval -ENOTSUP if scheduled output or @p channel is unsupported.
 * @return A provider-specific negative error code on failure.
 */
int precision_clock_output_get_caps(const struct precision_clock *precision_clk, uint32_t channel,
				    struct precision_clock_output_caps *caps);

/**
 * @brief Calculate the next aligned clock-output start time.
 *
 * Finds the earliest integer multiple of @p period_ns at or after the sum of
 * @p now and @p min_lead_time_ns. Negative clock values are supported.
 *
 * @param now Current time and domain of the output clock.
 * @param period_ns Positive output period in nanoseconds.
 * @param min_lead_time_ns Non-negative time required before the first edge.
 * @param start_time Destination for the aligned start time in @p now's domain.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument, period, or lead time is invalid.
 * @retval -ERANGE if calculating the aligned start time overflows.
 */
int precision_clock_output_next_start_time(const struct precision_time_point *now,
					   precision_time_t period_ns,
					   precision_time_t min_lead_time_ns,
					   struct precision_time_point *start_time);

/**
 * @brief Schedule a one-shot clock output event.
 *
 * Drives a single edge action selected by @p config edge at @p config
 * target_time. The channel must advertise @ref PRECISION_CLOCK_OUTPUT_CAP_EVENT
 * and the capability matching the requested edge. The target time must be
 * exactly representable at the channel resolution and meet the minimum lead
 * time. Scheduling an already configured channel is rejected; stop the channel
 * before reconfiguring it.
 *
 * Call from thread context. The provider operation may block.
 *
 * @param precision_clk Clock that owns the output channel.
 * @param channel Zero-based output channel index.
 * @param config Absolute target time and edge action.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument, domain, or edge value is invalid.
 * @retval -ENOTSUP if one-shot events, the requested edge, or @p channel is unsupported.
 * @retval -ERANGE if the target time is outside channel limits or not representable.
 * @retval -ETIME if the requested edge does not meet the minimum lead time.
 * @retval -EBUSY if the channel is already configured.
 * @return A provider-specific negative error code on failure.
 */
int precision_clock_output_schedule_event(const struct precision_clock *precision_clk,
					  uint32_t channel,
					  const struct precision_clock_output_event_config *config);

/**
 * @brief Start a periodic clock output waveform.
 *
 * The first rising edge is generated at @p config first_rising_time and
 * subsequent rising edges are separated by @p config period_ns. The channel
 * must advertise @ref PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM. When @p config
 * width_policy is @ref PRECISION_CLOCK_OUTPUT_WIDTH_EXACT the channel must also
 * advertise @ref PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH and the pulse
 * width must be positive and shorter than the period; otherwise the provider
 * default width is used and @p config pulse_width_ns is ignored. Configuration
 * values must be exactly representable at the channel resolution. Starting an
 * already configured channel is rejected; stop the channel before
 * reconfiguring it.
 *
 * A provider failure after hardware programming may conservatively retain the
 * channel configuration. Call @ref precision_clock_output_stop before retrying
 * or releasing the underlying device; a negative return does not guarantee
 * that the channel is unconfigured.
 *
 * Call from thread context. The provider operation may block.
 *
 * @param precision_clk Clock that owns the output channel.
 * @param channel Zero-based output channel index.
 * @param config First rising edge, period, and width policy.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument, domain, width policy, or interval relationship is invalid.
 * @retval -ENOTSUP if periodic waveforms, programmable width, or @p channel is unsupported.
 * @retval -ERANGE if a value is outside channel limits or not representable.
 * @retval -ETIME if the first rising edge does not meet the minimum lead time.
 * @retval -EBUSY if the channel is already configured.
 * @return A provider-specific negative error code on failure.
 */
int precision_clock_output_start_waveform(
	const struct precision_clock *precision_clk, uint32_t channel,
	const struct precision_clock_output_waveform_config *config);

/**
 * @brief Stop a scheduled clock output.
 *
 * Stopping a channel that is already unconfigured is idempotent.
 * The operation is delegated directly to the provider so a capability-query
 * failure cannot prevent an active output from being stopped.
 *
 * Call from thread context. The provider operation may block.
 *
 * @param precision_clk Clock that owns the output channel.
 * @param channel Zero-based output channel index.
 *
 * @retval 0 on success or if the channel is already unconfigured.
 * @retval -EINVAL if the clock or clock API is invalid.
 * @retval -ENOTSUP if scheduled output or @p channel is unsupported.
 * @return A provider-specific negative error code on failure.
 */
int precision_clock_output_stop(const struct precision_clock *precision_clk, uint32_t channel);

/**
 * @brief Query scheduled clock output status.
 *
 * Reports whether the provider holds a configuration for the channel, the
 * configured kind and accepted configuration, and the physical output state
 * when observable. The generic layer queries the channel capabilities to
 * validate @p channel and to reject a status that is inconsistent with the
 * advertised capabilities.
 *
 * Call from thread context. The provider operation may block.
 *
 * @param precision_clk Clock that owns the output channel.
 * @param channel Zero-based output channel index.
 * @param status Destination for the configuration and hardware-active state.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument or clock API is invalid, or the provider reports
 *	   a status inconsistent with its capabilities.
 * @retval -ENOTSUP if status reporting or @p channel is unsupported.
 * @return A provider-specific negative error code on failure.
 */
int precision_clock_output_get_status(const struct precision_clock *precision_clk, uint32_t channel,
				      struct precision_clock_output_status *status);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_PRECISION_TIMING_PRECISION_CLOCK_H_ */
