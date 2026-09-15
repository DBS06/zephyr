/*
 * Copyright (c) 2024 BayLibre SAS
 * Copyright (c) 2026 Philipp Steiner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_ptp_sample, LOG_LEVEL_DBG);

#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/precision_timing/precision_clock_ptp.h>
#include <zephyr/precision_timing/precision_timing_shell.h>

#include <zephyr/kernel.h>

#include <errno.h>
#include <stdlib.h>

#include "ptp/clock.h"
#include "ptp/port.h"

static int run_duration = CONFIG_NET_SAMPLE_RUN_DURATION;
static struct k_work_delayable stop_sample;
static struct k_sem quit_lock;

static void stop_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&quit_lock);
}

static int get_current_status(void)
{
	struct ptp_port *port;
	sys_slist_t *ports_list = ptp_clock_ports_list();

	if (!ports_list || sys_slist_len(ports_list) == 0) {
		return -EINVAL;
	}

	port = CONTAINER_OF(sys_slist_peek_head(ports_list), struct ptp_port, node);

	if (!port) {
		return -EINVAL;
	}

	switch (ptp_port_state(port)) {
	case PTP_PS_INITIALIZING:
	case PTP_PS_FAULTY:
	case PTP_PS_DISABLED:
	case PTP_PS_LISTENING:
	case PTP_PS_PRE_TIME_TRANSMITTER:
	case PTP_PS_PASSIVE:
	case PTP_PS_UNCALIBRATED:
		LOG_INF("FAIL");
		return 0;
	case PTP_PS_TIME_RECEIVER:
		LOG_INF("TIME RECEIVER");
		return 2;
	case PTP_PS_TIME_TRANSMITTER:
	case PTP_PS_GRAND_MASTER:
		LOG_INF("TIME TRANSMITTER");
		return 1;
	}

	return -1;
}

void init_testing(void)
{
	uint32_t uptime = k_uptime_get_32();
	int ret;

	if (run_duration == 0) {
		LOG_INF("Runs forever");
		return;
	}

	LOG_INF("Stopping after %u seconds", run_duration);

	k_sem_init(&quit_lock, 0, K_SEM_MAX_LIMIT);

	k_work_init_delayable(&stop_sample, stop_handler);
	k_work_reschedule(&stop_sample, K_SECONDS(run_duration));

	k_sem_take(&quit_lock, K_FOREVER);

	LOG_INF("Stopped after %u seconds", (k_uptime_get_32() - uptime) / 1000);

	/* Try to figure out what is the sync state.
	 * Return:
	 *  <0 - configuration error
	 *   0 - not time sync
	 *   1 - we are TimeTransmitter
	 *   2 - we are TimeReceiver
	 */
	ret = get_current_status();

	/* sleep gives deferred logs time to flush */
	k_msleep(2000);
	exit(ret);
}

#ifdef CONFIG_PRECISION_TIMING_SHELL
#define PHC_WAIT_ATTEMPTS 100U
#define PHC_WAIT_INTERVAL K_MSEC(100)

static struct precision_clock_ptp_adapter phc_adapter;

static int register_precision_clock(void)
{
	const struct device *ptp_clock;

	for (uint32_t attempt = 0U; attempt < PHC_WAIT_ATTEMPTS; attempt++) {
		struct net_if *iface = net_if_get_default();

		if (iface != NULL) {
			ptp_clock = net_eth_get_ptp_clock(iface);
			if (ptp_clock != NULL && device_is_ready(ptp_clock)) {
				precision_clock_ptp_init(&phc_adapter, ptp_clock);
				return precision_timing_shell_register(
					"phc", precision_clock_ptp_get(&phc_adapter));
			}
		}

		k_sleep(PHC_WAIT_INTERVAL);
	}

	return -ENODEV;
}
#endif

int main(void)
{
#ifdef CONFIG_PRECISION_TIMING_SHELL
	int ret = register_precision_clock();

	if (ret < 0) {
		LOG_ERR("Failed to register precision clock: %d", ret);
		return ret;
	}
#endif

	init_testing();
	return 0;
}
