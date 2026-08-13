/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <zephyr/precision_timing/precision_timing_shell.h>

struct precision_timing_shell_entry {
	char name[PRECISION_TIMING_SHELL_NAME_MAX + 1];
	const struct precision_clock *clock;
	struct precision_clock_sync *sync;
	bool unregistering;
};

struct precision_timing_shell_operation {
	const struct precision_clock *clock;
	struct precision_clock_sync *sync;
};

static struct precision_timing_shell_entry registry[CONFIG_PRECISION_TIMING_SHELL_MAX_INSTANCES];
static size_t registry_count;
static size_t active_operations;
static K_MUTEX_DEFINE(registry_lock);
static K_CONDVAR_DEFINE(registry_idle);

static bool name_is_valid(const char *name)
{
	size_t length;

	if (name == NULL) {
		return false;
	}

	length = strlen(name);
	if (length == 0 || length > PRECISION_TIMING_SHELL_NAME_MAX) {
		return false;
	}

	for (size_t i = 0; i < length; i++) {
		unsigned char character = name[i];

		if (character > 0x7f || (!isalnum(character) && character != '.' &&
					 character != '-' && character != '_')) {
			return false;
		}
	}

	return true;
}

static size_t registry_lower_bound(const char *name, bool *found)
{
	size_t left = 0;
	size_t right = registry_count;

	while (left < right) {
		size_t middle = left + (right - left) / 2;
		int comparison = strcmp(registry[middle].name, name);

		if (comparison < 0) {
			left = middle + 1;
		} else {
			right = middle;
		}
	}

	*found = left < registry_count && strcmp(registry[left].name, name) == 0;
	return left;
}

int precision_timing_shell_register(const char *name, const struct precision_clock *clock,
				    struct precision_clock_sync *sync)
{
	struct precision_timing_shell_entry *entry;
	size_t name_length;
	size_t index;
	bool found;

	if (!name_is_valid(name) || clock == NULL) {
		return -EINVAL;
	}

	if (sync != NULL && !IS_ENABLED(CONFIG_PRECISION_CLOCK_SYNC_SERVICE)) {
		return -ENOTSUP;
	}

	k_mutex_lock(&registry_lock, K_FOREVER);
	index = registry_lower_bound(name, &found);
	if (found) {
		k_mutex_unlock(&registry_lock);
		return -EEXIST;
	}

	if (registry_count == ARRAY_SIZE(registry)) {
		k_mutex_unlock(&registry_lock);
		return -ENOSPC;
	}

	memmove(&registry[index + 1], &registry[index],
		(registry_count - index) * sizeof(registry[0]));
	entry = &registry[index];
	name_length = strlen(name);
	memcpy(entry->name, name, name_length + 1);
	entry->clock = clock;
	entry->sync = sync;
	entry->unregistering = false;
	registry_count++;
	k_mutex_unlock(&registry_lock);

	return 0;
}

int precision_timing_shell_unregister(const char *name)
{
	size_t index;
	bool found;

	if (!name_is_valid(name)) {
		return -EINVAL;
	}

	k_mutex_lock(&registry_lock, K_FOREVER);
	index = registry_lower_bound(name, &found);
	if (!found || registry[index].unregistering) {
		k_mutex_unlock(&registry_lock);
		return -ENOENT;
	}

	registry[index].unregistering = true;
	while (active_operations != 0) {
		(void)k_condvar_wait(&registry_idle, &registry_lock, K_FOREVER);
	}

	index = registry_lower_bound(name, &found);
	__ASSERT_NO_MSG(found && registry[index].unregistering);
	registry_count--;
	memmove(&registry[index], &registry[index + 1],
		(registry_count - index) * sizeof(registry[0]));
	memset(&registry[registry_count], 0, sizeof(registry[0]));
	k_mutex_unlock(&registry_lock);

	return 0;
}

static int operation_acquire(const char *name, bool require_sync,
			     struct precision_timing_shell_operation *operation)
{
	size_t index;
	bool found;

	k_mutex_lock(&registry_lock, K_FOREVER);
	index = registry_lower_bound(name, &found);
	if (!found || registry[index].unregistering) {
		k_mutex_unlock(&registry_lock);
		return -ENOENT;
	}

	if (require_sync && registry[index].sync == NULL) {
		k_mutex_unlock(&registry_lock);
		return -ENOTSUP;
	}

	operation->clock = registry[index].clock;
	operation->sync = registry[index].sync;
	active_operations++;
	k_mutex_unlock(&registry_lock);

	return 0;
}

static void operation_release(void)
{
	k_mutex_lock(&registry_lock, K_FOREVER);
	active_operations--;
	if (active_operations == 0) {
		k_condvar_broadcast(&registry_idle);
	}
	k_mutex_unlock(&registry_lock);
}

static const char *domain_type_name(enum precision_time_domain_type type)
{
	switch (type) {
	case PRECISION_TIME_DOMAIN_INVALID:
		return "invalid";
	case PRECISION_TIME_DOMAIN_TAI:
		return "tai";
	case PRECISION_TIME_DOMAIN_UTC:
		return "utc";
	case PRECISION_TIME_DOMAIN_MONOTONIC:
		return "monotonic";
	case PRECISION_TIME_DOMAIN_PHC:
		return "phc";
	case PRECISION_TIME_DOMAIN_PTP:
		return "ptp";
	case PRECISION_TIME_DOMAIN_GPTP:
		return "gptp";
	case PRECISION_TIME_DOMAIN_RAW:
		return "raw";
	default:
		return "unknown";
	}
}

#if defined(CONFIG_PRECISION_CLOCK_SYNC_SERVICE)
static const char *sync_state_name(enum precision_sync_state state)
{
	switch (state) {
	case PRECISION_SYNC_UNSYNCED:
		return "UNSYNCED";
	case PRECISION_SYNC_ACQUIRING:
		return "ACQUIRING";
	case PRECISION_SYNC_LOCKED:
		return "LOCKED";
	case PRECISION_SYNC_HOLDOVER:
		return "HOLDOVER";
	case PRECISION_SYNC_FAULT:
		return "FAULT";
	default:
		return "UNKNOWN";
	}
}

static const char *action_name(enum precision_discipline_action action)
{
	switch (action) {
	case PRECISION_DISCIPLINE_IGNORE:
		return "IGNORE";
	case PRECISION_DISCIPLINE_STEP:
		return "STEP";
	case PRECISION_DISCIPLINE_ADJUST_RATE:
		return "ADJUST_RATE";
	case PRECISION_DISCIPLINE_RESET:
		return "RESET";
	default:
		return "UNKNOWN";
	}
}
#endif /* CONFIG_PRECISION_CLOCK_SYNC_SERVICE */

static int report_error(const struct shell *shell, const char *name, int error)
{
	shell_error(shell, "precision clock '%s': %d", name, error);
	return error;
}

static int cmd_list(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_mutex_lock(&registry_lock, K_FOREVER);
	for (size_t i = 0; i < registry_count; i++) {
		if (!registry[i].unregistering) {
			shell_print(shell, "%s", registry[i].name);
		}
	}
	k_mutex_unlock(&registry_lock);

	return 0;
}

static int cmd_get(const struct shell *shell, size_t argc, char **argv)
{
	struct precision_timing_shell_operation operation;
	struct precision_time_point time_point;
	int ret;

	ARG_UNUSED(argc);

	ret = operation_acquire(argv[1], false, &operation);
	if (ret == 0) {
		ret = precision_clock_read(operation.clock, &time_point);
		operation_release();
	}
	if (ret < 0) {
		return report_error(shell, argv[1], ret);
	}

	shell_print(shell, "time_ns=%lld domain=%s domain_id=%u", (long long)time_point.time,
		    domain_type_name(time_point.domain.type), time_point.domain.id);
	return 0;
}

static int cmd_caps(const struct shell *shell, size_t argc, char **argv)
{
	struct precision_timing_shell_operation operation;
	struct precision_clock_caps caps;
	int ret;

	ARG_UNUSED(argc);

	ret = operation_acquire(argv[1], false, &operation);
	if (ret == 0) {
		ret = precision_clock_get_caps(operation.clock, &caps);
		operation_release();
	}
	if (ret < 0) {
		return report_error(shell, argv[1], ret);
	}

	shell_print(shell, "read=%s set=%s adjust_phase=%s adjust_rate=%s",
		    (caps.flags & PRECISION_CLOCK_CAP_READ) != 0 ? "yes" : "no",
		    (caps.flags & PRECISION_CLOCK_CAP_SET) != 0 ? "yes" : "no",
		    (caps.flags & PRECISION_CLOCK_CAP_ADJUST_PHASE) != 0 ? "yes" : "no",
		    (caps.flags & PRECISION_CLOCK_CAP_ADJUST_RATE) != 0 ? "yes" : "no");
	shell_print(shell, "resolution_ns=%lld max_phase_adjust_ns=%lld",
		    (long long)caps.resolution_ns, (long long)caps.max_phase_adjust_ns);
	shell_print(shell, "min_rate_ppb=%d max_rate_ppb=%d", caps.min_rate_ppb, caps.max_rate_ppb);
	return 0;
}

#if defined(CONFIG_PRECISION_CLOCK_SYNC_SERVICE)
static int cmd_status(const struct shell *shell, size_t argc, char **argv)
{
	struct precision_timing_shell_operation operation;
	struct precision_clock_sync_status status;
	int ret;

	ARG_UNUSED(argc);

	ret = operation_acquire(argv[1], true, &operation);
	if (ret == 0) {
		ret = precision_clock_sync_get_status(operation.sync, &status);
		operation_release();
	}
	if (ret < 0) {
		return report_error(shell, argv[1], ret);
	}

	shell_print(shell, "running=%s", status.running ? "yes" : "no");
	shell_print(shell, "source_domain=%s source_domain_id=%u",
		    domain_type_name(status.source_domain.type), status.source_domain.id);
	shell_print(shell, "sink_domain=%s sink_domain_id=%u",
		    domain_type_name(status.sink_domain.type), status.sink_domain.id);
	shell_print(shell, "state=%s last_action=%s", sync_state_name(status.state),
		    action_name(status.last_action));
	shell_print(shell, "offset_ns=%lld applied_rate_ppb=%d", (long long)status.offset_ns,
		    status.applied_rate_ppb);
	shell_print(shell, "source_age_ns=%lld sampling_uncertainty_ns=%lld",
		    (long long)status.source_age_ns, (long long)status.sampling_uncertainty_ns);
	shell_print(shell, "accepted_observations=%u rejected_observations=%u",
		    status.accepted_observations, status.rejected_observations);
	shell_print(shell, "source_read_failures=%u sink_read_failures=%u control_failures=%u",
		    status.source_read_failures, status.sink_read_failures,
		    status.control_failures);
	shell_print(shell, "last_error=%d", status.last_error);
	return 0;
}

static int cmd_sync_operation(const struct shell *shell, const char *name,
			      int (*operation_fn)(struct precision_clock_sync *sync))
{
	struct precision_timing_shell_operation operation;
	int ret;

	ret = operation_acquire(name, true, &operation);
	if (ret == 0) {
		ret = operation_fn(operation.sync);
		operation_release();
	}
	if (ret < 0) {
		return report_error(shell, name, ret);
	}

	return 0;
}

static int cmd_start(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	return cmd_sync_operation(shell, argv[1], precision_clock_sync_start);
}

static int cmd_stop(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	return cmd_sync_operation(shell, argv[1], precision_clock_sync_stop);
}

static int cmd_reset(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	return cmd_sync_operation(shell, argv[1], precision_clock_sync_reset);
}
#endif /* CONFIG_PRECISION_CLOCK_SYNC_SERVICE */

SHELL_STATIC_SUBCMD_SET_CREATE(
	precision_clock_commands,
	SHELL_CMD_ARG(list, NULL, "List registered precision clocks.", cmd_list, 1, 0),
	SHELL_CMD_ARG(get, NULL, "Read a precision clock. Usage: get <name>", cmd_get, 2, 0),
	SHELL_CMD_ARG(caps, NULL, "Show clock capabilities. Usage: caps <name>", cmd_caps, 2, 0),
#if defined(CONFIG_PRECISION_CLOCK_SYNC_SERVICE)
	SHELL_CMD_ARG(status, NULL, "Show synchronization status. Usage: status <name>", cmd_status,
		      2, 0),
	SHELL_CMD_ARG(start, NULL, "Start synchronization. Usage: start <name>", cmd_start, 2, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop synchronization. Usage: stop <name>", cmd_stop, 2, 0),
	SHELL_CMD_ARG(reset, NULL, "Reset synchronization. Usage: reset <name>", cmd_reset, 2, 0),
#endif /* CONFIG_PRECISION_CLOCK_SYNC_SERVICE */
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(precision_clock, &precision_clock_commands,
		   "Inspect and control registered precision clocks.", NULL);
