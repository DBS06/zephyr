/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Protocol-neutral scheduled clock output provider extension.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PRECISION_CLOCK_OUTPUT_H_
#define ZEPHYR_INCLUDE_DRIVERS_PRECISION_CLOCK_OUTPUT_H_

#include <zephyr/device.h>
#include <zephyr/precision_timing/precision_clock.h>
#include <zephyr/precision_timing/precision_time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Precision clock output provider extension
 * @defgroup precision_clock_output_provider Precision clock output provider
 * @since 4.5
 * @version 0.1.0
 * @ingroup precision_timing
 * @{
 *
 * Driver-facing contract that lets a device expose scheduled clock outputs to
 * a protocol-neutral precision clock adapter. All times are raw values in the
 * owning clock's domain, so the extension carries no @ref precision_time_domain
 * and no protocol-specific prefix. An adapter such as the PTP clock adapter
 * discovers the extension, attaches the clock domain, and bridges it onto the
 * domain-qualified public API in @ref precision_clock.
 *
 * The capability flags, mode-specific limits, edge actions, width policy, and
 * configured kind are shared with the public API through @ref precision_clock.
 * A provider may implement only one-shot events, only periodic waveforms, or
 * both; the mode it does not implement leaves the matching callback null and
 * the matching capability flag clear.
 *
 * Provider callbacks are invoked only from thread context and may block. A
 * provider may document additional, more restrictive context requirements.
 */

/** Raw clock-domain configuration of a one-shot scheduled output event. */
struct precision_clock_output_raw_event_config {
	/** Absolute target time of the scheduled edge in the clock's own domain. */
	precision_time_t target_time;
	/** Edge action to drive at @ref target_time. */
	enum precision_clock_output_edge edge;
};

/** Raw clock-domain configuration of a periodic output waveform. */
struct precision_clock_output_raw_waveform_config {
	/** Absolute time of the first rising edge in the clock's own domain. */
	precision_time_t first_rising_time;
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

/** Accepted raw configuration held by a scheduled output channel. */
union precision_clock_output_raw_configured_config {
	/** Valid when the configured kind is @ref PRECISION_CLOCK_OUTPUT_KIND_EVENT. */
	struct precision_clock_output_raw_event_config event;
	/** Valid when the configured kind is @ref PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM. */
	struct precision_clock_output_raw_waveform_config waveform;
};

/** Raw clock-domain status of a scheduled output channel. */
struct precision_clock_output_raw_status {
	/** Whether the provider currently holds a configuration for the channel. */
	bool configured;
	/** Configured output kind. Valid only when @ref configured is true. */
	enum precision_clock_output_kind kind;
	/** Accepted configuration. Valid only when @ref configured is true. */
	union precision_clock_output_raw_configured_config config;
	/** Whether @ref hardware_active reflects a real hardware observation. */
	bool hardware_active_valid;
	/** Physical output activity. Meaningful only when @ref hardware_active_valid is true. */
	bool hardware_active;
};

/**
 * @brief Query raw output channel capabilities and limits.
 *
 * @param dev Device that owns the output channel.
 * @param channel Zero-based output channel index.
 * @param caps Destination for the channel capabilities and limits.
 *
 * @return 0 on success or a negative error code on failure.
 */
typedef int (*precision_clock_output_get_caps_t)(const struct device *dev, uint32_t channel,
						 struct precision_clock_output_caps *caps);

/**
 * @brief Schedule a raw one-shot output event.
 *
 * @param dev Device that owns the output channel.
 * @param channel Zero-based output channel index.
 * @param config Raw target time and edge action.
 *
 * @return 0 on success or a negative error code on failure.
 */
typedef int (*precision_clock_output_schedule_event_t)(
	const struct device *dev, uint32_t channel,
	const struct precision_clock_output_raw_event_config *config);

/**
 * @brief Start a raw periodic output waveform.
 *
 * A failure after hardware programming may leave the channel conservatively
 * configured. The caller must stop the channel before retrying or releasing
 * the owning device; a negative return does not guarantee an unconfigured
 * channel.
 *
 * @param dev Device that owns the output channel.
 * @param channel Zero-based output channel index.
 * @param config Raw first rising edge, period, and width policy.
 *
 * @return 0 on success or a negative error code on failure.
 */
typedef int (*precision_clock_output_start_waveform_t)(
	const struct device *dev, uint32_t channel,
	const struct precision_clock_output_raw_waveform_config *config);

/**
 * @brief Stop a scheduled output channel.
 *
 * Stopping a channel that is already unconfigured is idempotent.
 *
 * @param dev Device that owns the output channel.
 * @param channel Zero-based output channel index.
 *
 * @return 0 on success or a negative error code on failure.
 */
typedef int (*precision_clock_output_stop_t)(const struct device *dev, uint32_t channel);

/**
 * @brief Query raw output channel status.
 *
 * @param dev Device that owns the output channel.
 * @param channel Zero-based output channel index.
 * @param status Destination for the raw channel status.
 *
 * @return 0 on success or a negative error code on failure.
 */
typedef int (*precision_clock_output_get_status_t)(
	const struct device *dev, uint32_t channel,
	struct precision_clock_output_raw_status *status);

/**
 * @brief Scheduled clock output provider extension.
 *
 * A device advertises scheduled output support by referencing an instance of
 * this structure from its driver API. The @ref get_caps, @ref stop, and
 * @ref get_status callbacks are required. At least one of @ref schedule_event
 * and @ref start_waveform must be present; the unused mode leaves its callback
 * null and clears its capability flag. A protocol adapter advertises clock-level
 * scheduled-output support only when channel zero exists and @ref get_caps reports
 * a positive channel count.
 */
struct precision_clock_output_provider {
	/** Query channel capabilities and limits. */
	precision_clock_output_get_caps_t get_caps;
	/** Schedule a one-shot output event. Null when events are unsupported. */
	precision_clock_output_schedule_event_t schedule_event;
	/** Start a periodic output waveform. Null when waveforms are unsupported. */
	precision_clock_output_start_waveform_t start_waveform;
	/** Stop a scheduled output channel. */
	precision_clock_output_stop_t stop;
	/** Query channel status. */
	precision_clock_output_get_status_t get_status;
};

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PRECISION_CLOCK_OUTPUT_H_ */
