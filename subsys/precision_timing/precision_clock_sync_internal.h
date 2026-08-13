/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_PRECISION_TIMING_PRECISION_CLOCK_SYNC_INTERNAL_H_
#define ZEPHYR_SUBSYS_PRECISION_TIMING_PRECISION_CLOCK_SYNC_INTERNAL_H_

#include <stdint.h>

#include <zephyr/precision_timing/precision_clock_sync.h>

int precision_clock_sync_run_once(struct precision_clock_sync *sync, uint32_t generation);

int precision_clock_sync_run_once_at(struct precision_clock_sync *sync, uint32_t generation,
				     precision_time_t now_uptime_ns);

#endif /* ZEPHYR_SUBSYS_PRECISION_TIMING_PRECISION_CLOCK_SYNC_INTERNAL_H_ */
