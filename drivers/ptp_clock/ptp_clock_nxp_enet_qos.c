/*
 * SPDX-FileCopyrightText: Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PTP clock driver for NXP ENET QoS.
 *
 * The ENET QoS PTP subsystem operates in fine-update mode.
 * A 32-bit accumulator is incremented by ADDEND each reference-clock
 * cycle; every time it overflows the system-time sub-second register
 * is bumped by SNSINC nanoseconds.  Rate discipline is achieved by
 * adjusting ADDEND via the TSADDREG mechanism.
 *
 * Register macros (ENET_MAC_*) come from the MCXN CMSIS device header
 * (PERI_ENET.h) included transitively via eth_nxp_enet_qos.h.
 * The fsl_enet_qos SDK HAL is intentionally NOT used here.
 */

#define DT_DRV_COMPAT nxp_enet_qos_ptp_clock

#include <errno.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/drivers/precision_clock_output.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/ethernet/eth_nxp_enet_qos.h>
#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mux.h>
#include <zephyr/drivers/pinctrl.h>

#include "ptp_clock_nxp_enet_qos_output.h"
#endif
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ptp_clock_nxp_enet_qos);

/*
 * Desired PTP system-time clock frequency in Hz.
 * Must be strictly less than the reference (bus) clock so that
 * ADDEND = 2^32 * ptpClk / refClk fits in a 32-bit register.
 * 50 MHz matches ENET_QOS_SYSTIME_REQUIRED_CLK_MHZ in the NXP HAL.
 */
#define PTP_CLOCK_NXP_ENET_QOS_PTPCLK_HZ 50000000U

struct ptp_clock_nxp_enet_qos_config {
	const struct device *enet_qos_dev;	/* device of the parent ENET QoSmodule */
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
	const struct pinctrl_dev_config *pincfg;
	struct gpio_dt_spec output_gpio;
	const struct device *mux_dev;
	const struct mux_state *mux_state;
#endif
};

struct ptp_clock_nxp_enet_qos_data {
	enet_qos_t *base;	/* base address of the parent ENET QoS module */
	struct k_mutex ptp_mutex;
	uint32_t nominal_addend;
	uint32_t ref_clk_hz;	/* the actual PTP clock frequency */
#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
	const struct device *dev;
	struct k_mutex output_lifecycle_mutex;
	struct k_work_delayable output_work;
	struct precision_clock_output_raw_waveform_config output_config;
	/* Cleanup failed; get_status() surfaces this until stop succeeds. */
	int output_fault_error;
	bool output_configured;
	bool output_stopping;
#endif
};

static int ptp_clock_nxp_enet_qos_get_unlocked(const struct device *dev,
					       struct net_ptp_time *tm);

#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
static int ptp_clock_nxp_enet_qos_output_gate_low(const struct device *dev)
{
	const struct ptp_clock_nxp_enet_qos_config *config = dev->config;

	return gpio_pin_configure_dt(&config->output_gpio, GPIO_OUTPUT_LOW);
}

static void ptp_clock_nxp_enet_qos_output_clear(struct ptp_clock_nxp_enet_qos_data *data)
{
	data->output_fault_error = 0;
	data->output_configured = false;
	data->output_config = (struct precision_clock_output_raw_waveform_config){0};
}

static int ptp_clock_nxp_enet_qos_output_route(const struct device *dev)
{
	const struct ptp_clock_nxp_enet_qos_config *config = dev->config;

	return pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
}

static void ptp_clock_nxp_enet_qos_output_fail(struct ptp_clock_nxp_enet_qos_data *data,
					       int error)
{
	int ret;

	ret = ptp_clock_nxp_enet_qos_output_gate_low(data->dev);
	if (ret < 0) {
		data->output_fault_error = error;
		LOG_ERR("Scheduled output faulted: arming failed (%d), holding low failed (%d)",
			error, ret);
		return;
	}

	ptp_clock_nxp_enet_qos_output_clear(data);
}

static int ptp_clock_nxp_enet_qos_output_reschedule(
	struct ptp_clock_nxp_enet_qos_data *data, precision_time_t delay_ns)
{
	int ret;

	delay_ns = CLAMP(delay_ns, 0, PTP_CLOCK_NXP_ENET_QOS_OUTPUT_MAX_RECHECK_NS);
	ret = k_work_reschedule(&data->output_work, K_NSEC(delay_ns));

	return ret < 0 ? ret : 0;
}

static void ptp_clock_nxp_enet_qos_output_cancel_work_sync(
	struct ptp_clock_nxp_enet_qos_data *data)
{
	(void)k_work_cancel_delayable(&data->output_work);

	while (k_work_delayable_busy_get(&data->output_work) != 0U) {
		k_sleep(K_TICKS(1));
	}
}

static void ptp_clock_nxp_enet_qos_output_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ptp_clock_nxp_enet_qos_data *data =
		CONTAINER_OF(dwork, struct ptp_clock_nxp_enet_qos_data, output_work);
	struct net_ptp_time tm;
	precision_time_t now_ns;
	precision_time_t delay_ns;
	struct ptp_clock_nxp_enet_qos_output_gate_decision decision;
	int ret;

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);
	if (!data->output_configured || data->output_stopping) {
		k_mutex_unlock(&data->ptp_mutex);
		return;
	}

	ret = ptp_clock_nxp_enet_qos_get_unlocked(data->dev, &tm);
	if (ret < 0) {
		LOG_ERR("Failed to read PHC while arming PPS output: %d", ret);
		goto fail;
	}
	ret = precision_time_from_u64_sec_nsec(tm.second, tm.nanosecond, &now_ns);
	if (ret < 0) {
		LOG_ERR("PHC time is out of range while arming PPS output");
		goto fail;
	}
	decision = ptp_clock_nxp_enet_qos_output_gate_decide(
		now_ns, data->output_config.first_rising_time);
	if (decision.action == PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_MISSED) {
		LOG_WRN("Missed PPS output activation time");
		ret = -ETIME;
		goto fail;
	}

	if (decision.action == PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_WAIT) {
		delay_ns = decision.delay_ns;
		ret = ptp_clock_nxp_enet_qos_output_reschedule(data, delay_ns);
		if (ret < 0) {
			LOG_ERR("Failed to reschedule PPS output gate: %d", ret);
			goto fail;
		}
		k_mutex_unlock(&data->ptp_mutex);
		return;
	}

	ret = ptp_clock_nxp_enet_qos_output_route(data->dev);
	if (ret < 0) {
		LOG_ERR("Failed to route PPS output pin: %d", ret);
		goto fail;
	}
	k_mutex_unlock(&data->ptp_mutex);
	return;

fail:
	ptp_clock_nxp_enet_qos_output_fail(data, ret);
	k_mutex_unlock(&data->ptp_mutex);
}

static int ptp_clock_nxp_enet_qos_output_get_caps(const struct device *dev, uint32_t channel,
						  struct precision_clock_output_caps *caps)
{
	ARG_UNUSED(dev);
	return ptp_clock_nxp_enet_qos_output_caps(channel, caps);
}

static int ptp_clock_nxp_enet_qos_output_start_waveform(
	const struct device *dev, uint32_t channel,
	const struct precision_clock_output_raw_waveform_config *output_config)
{
	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	struct net_ptp_time tm;
	struct ptp_clock_nxp_enet_qos_output_gate_decision decision;
	precision_time_t now_ns;
	int ret;

	ret = ptp_clock_nxp_enet_qos_output_validate(channel, output_config);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->output_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&data->ptp_mutex, K_FOREVER);
	if (data->output_configured || data->output_stopping) {
		ret = -EBUSY;
		goto out;
	}

	ret = ptp_clock_nxp_enet_qos_get_unlocked(dev, &tm);
	if (ret < 0) {
		goto out;
	}
	ret = precision_time_from_u64_sec_nsec(tm.second, tm.nanosecond, &now_ns);
	if (ret < 0) {
		goto out;
	}
	if (output_config->first_rising_time < now_ns ||
	    output_config->first_rising_time - now_ns <
		    PTP_CLOCK_NXP_ENET_QOS_OUTPUT_MIN_LEAD_NS) {
		ret = -ETIME;
		goto out;
	}

	ret = ptp_clock_nxp_enet_qos_output_gate_low(dev);
	if (ret < 0) {
		goto out;
	}
	data->output_fault_error = 0;

	data->output_config = *output_config;
	data->output_configured = true;
	decision = ptp_clock_nxp_enet_qos_output_gate_decide(
		now_ns, output_config->first_rising_time);
	if (decision.action == PTP_CLOCK_NXP_ENET_QOS_OUTPUT_GATE_ROUTE) {
		ret = ptp_clock_nxp_enet_qos_output_route(dev);
	} else {
		ret = ptp_clock_nxp_enet_qos_output_reschedule(data, decision.delay_ns);
	}
	if (ret < 0) {
		ptp_clock_nxp_enet_qos_output_fail(data, ret);
	}

out:
	k_mutex_unlock(&data->ptp_mutex);
	k_mutex_unlock(&data->output_lifecycle_mutex);
	return ret;
}

static int ptp_clock_nxp_enet_qos_output_stop(const struct device *dev, uint32_t channel)
{
	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	int ret;

	if (channel != 0U) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->output_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&data->ptp_mutex, K_FOREVER);
	data->output_stopping = true;
	k_mutex_unlock(&data->ptp_mutex);

	ptp_clock_nxp_enet_qos_output_cancel_work_sync(data);
	ret = ptp_clock_nxp_enet_qos_output_gate_low(dev);

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);
	if (ret == 0) {
		ptp_clock_nxp_enet_qos_output_clear(data);
	} else {
		data->output_fault_error = ret;
	}
	data->output_stopping = false;
	k_mutex_unlock(&data->ptp_mutex);
	k_mutex_unlock(&data->output_lifecycle_mutex);

	return ret;
}

static int ptp_clock_nxp_enet_qos_output_get_status(
	const struct device *dev, uint32_t channel,
	struct precision_clock_output_raw_status *status)
{
	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	int ret;

	if (status == NULL) {
		return -EINVAL;
	}
	if (channel != 0U) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);
	*status = (struct precision_clock_output_raw_status){
		.configured = data->output_configured,
		.kind = PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM,
		.config.waveform = data->output_config,
		.hardware_active_valid = false,
	};
	ret = data->output_fault_error;
	k_mutex_unlock(&data->ptp_mutex);

	return ret;
}

static const struct precision_clock_output_provider ptp_clock_nxp_enet_qos_output_provider = {
	.get_caps = ptp_clock_nxp_enet_qos_output_get_caps,
	.start_waveform = ptp_clock_nxp_enet_qos_output_start_waveform,
	.stop = ptp_clock_nxp_enet_qos_output_stop,
	.get_status = ptp_clock_nxp_enet_qos_output_get_status,
};
#endif /* CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT */

static int ptp_clock_nxp_enet_qos_set(const struct device *dev, struct net_ptp_time *tm)
{
	LOG_DBG("PTP set time: %u s, %u ns", (unsigned int)tm->second, tm->nanosecond);

	struct ptp_clock_nxp_enet_qos_data *data = dev->data;

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	data->base->MAC_SYSTEM_TIME_SECONDS_UPDATE = (uint32_t)tm->second;
	/* ADDSUB = 0: absolute initialise */
	data->base->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE =
		tm->nanosecond & ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK;

	/* Use TSINIT (absolute load), valid because the accumulator is already
	 * running so it self-clears on the next accumulator overflow (~20 ns).
	 */
	data->base->MAC_TIMESTAMP_CONTROL |= ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK;
	while (data->base->MAC_TIMESTAMP_CONTROL & ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK) {
	}

	k_mutex_unlock(&data->ptp_mutex);
	return 0;
}

/* The caller must hold ptp_mutex to serialize this observation against PHC updates. */
static int ptp_clock_nxp_enet_qos_get_unlocked(const struct device *dev,
					       struct net_ptp_time *tm)
{
	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	uint32_t ns1, ns2, sec;

	/*
	 * Guard against a seconds roll-over between the two nanosecond reads:
	 * re-read if nanoseconds decreased (wrap occurred).
	 */
	do {
		ns1 = data->base->MAC_SYSTEM_TIME_NANOSECONDS &
		      ENET_MAC_SYSTEM_TIME_NANOSECONDS_TSSS_MASK;
		sec = data->base->MAC_SYSTEM_TIME_SECONDS;
		ns2 = data->base->MAC_SYSTEM_TIME_NANOSECONDS &
		      ENET_MAC_SYSTEM_TIME_NANOSECONDS_TSSS_MASK;
	} while (ns2 < ns1);

	tm->second = sec;
	tm->nanosecond = ns2;
	return 0;
}

static int ptp_clock_nxp_enet_qos_get(const struct device *dev, struct net_ptp_time *tm)
{
	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);
	ret = ptp_clock_nxp_enet_qos_get_unlocked(dev, tm);
	k_mutex_unlock(&data->ptp_mutex);

	return ret;
}

/**
 * adjust PTP clock by increment nanoseconds, positive or negative.
 */
static int ptp_clock_nxp_enet_qos_adjust(const struct device *dev, int increment)
{
	LOG_DBG("PTP rate adjust increment: %d", increment);

	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	uint32_t ns_update;

	if ((increment <= (-(int32_t)NSEC_PER_SEC)) || (increment >= (int32_t)NSEC_PER_SEC)) {
		return -EINVAL;
	}

	if (increment >= 0) {
		/* ADDSUB = 0: add */
		ns_update = (uint32_t)increment & ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK;
	} else {
		/* ADDSUB = 1: subtract */
		ns_update = ((uint32_t)(-increment) &
			     ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK) |
			    ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB_MASK;
	}

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	data->base->MAC_SYSTEM_TIME_SECONDS_UPDATE = 0;
	data->base->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE = ns_update;
	data->base->MAC_TIMESTAMP_CONTROL |= ENET_MAC_TIMESTAMP_CONTROL_TSUPDT_MASK;
	while (data->base->MAC_TIMESTAMP_CONTROL & ENET_MAC_TIMESTAMP_CONTROL_TSUPDT_MASK) {
	}

	k_mutex_unlock(&data->ptp_mutex);
	return 0;
}

static int ptp_clock_nxp_enet_qos_rate_adjust(const struct device *dev, double ratio)
{
	LOG_DBG("PTP rate adjust ratio: %f", ratio);

	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	uint32_t new_addend;

	/* No meaningful change */
	if ((ratio > 1.0 && ratio - 1.0 < 1e-9) || (ratio < 1.0 && 1.0 - ratio < 1e-9)) {
		return 0;
	}

	new_addend = (uint32_t)((double)data->nominal_addend * ratio);

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	data->base->MAC_TIMESTAMP_ADDEND = new_addend;
	data->base->MAC_TIMESTAMP_CONTROL |= ENET_MAC_TIMESTAMP_CONTROL_TSADDREG_MASK;
	while (data->base->MAC_TIMESTAMP_CONTROL & ENET_MAC_TIMESTAMP_CONTROL_TSADDREG_MASK) {
	}

	k_mutex_unlock(&data->ptp_mutex);
	return 0;
}

static int ptp_clock_nxp_enet_qos_get_caps(const struct device *dev, struct ptp_clock_caps *caps)
{
	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	double max_rate_ppb = INT32_MAX;

	if (caps == NULL) {
		return -EINVAL;
	}

	if (data->nominal_addend > 0U) {
		max_rate_ppb =
			(((double)UINT32_MAX / (double)data->nominal_addend) - 1.0) * 1000000000.0;
	}

	*caps = (struct ptp_clock_caps){
		.flags = PTP_CLOCK_CAP_READ | PTP_CLOCK_CAP_SET | PTP_CLOCK_CAP_ADJUST |
			 PTP_CLOCK_CAP_RATE_ADJUST,
		.resolution_ns = NSEC_PER_SEC / PTP_CLOCK_NXP_ENET_QOS_PTPCLK_HZ,
		.max_adjust_ns = NSEC_PER_SEC - 1,
		.min_rate_ppb = -999999999,
		.max_rate_ppb = (int32_t)MIN(max_rate_ppb, (double)INT32_MAX),
	};

	return 0;
}

static int ptp_clock_nxp_enet_qos_init(const struct device *dev)
{
	LOG_INF("Initializing NXP ENET QoS PTP clock on device %s", dev->name);
	const struct ptp_clock_nxp_enet_qos_config *config = dev->config;
	struct ptp_clock_nxp_enet_qos_data *data = dev->data;
	struct nxp_enet_qos_config *module_cfg = ENET_QOS_MODULE_CFG(config->enet_qos_dev);
	const uint32_t snsinc = NSEC_PER_SEC / PTP_CLOCK_NXP_ENET_QOS_PTPCLK_HZ;
	uint32_t clk_rate;
	int ret;

	data->base = module_cfg->base;
	k_mutex_init(&data->ptp_mutex);

#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
	data->dev = dev;
	k_mutex_init(&data->output_lifecycle_mutex);
	k_work_init_delayable(&data->output_work, ptp_clock_nxp_enet_qos_output_work);

	if (!device_is_ready(config->mux_dev) || !gpio_is_ready_dt(&config->output_gpio)) {
		return -ENODEV;
	}

	ret = mux_state_apply(config->mux_dev, config->mux_state);
	if (ret < 0) {
		LOG_ERR("Failed to route ENET PPS0 to EXT_TRIG0: %d", ret);
		return ret;
	}

	ret = ptp_clock_nxp_enet_qos_output_gate_low(dev);
	if (ret < 0) {
		LOG_ERR("Failed to hold PPS output low: %d", ret);
		return ret;
	}
#endif

	LOG_INF("Starting NXP ENET QoS PTP hardware");

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clk_rate);
	if (ret) {
		LOG_ERR("Failed to get PTP clock");
		return ret;
	}

	data->ref_clk_hz = clk_rate;
	LOG_INF("PTP reference clock %u Hz", data->ref_clk_hz);
	__ASSERT(data->ref_clk_hz != 0, "Failed to get PTP clock");

	data->nominal_addend =
		(uint32_t)((double)(1ULL << 32) * (double)PTP_CLOCK_NXP_ENET_QOS_PTPCLK_HZ /
			   (double)data->ref_clk_hz);
	LOG_INF("PTP accumulator addend %u nsec", data->nominal_addend);

	/*
	 * Step 1: Enable timestamping in COARSE update mode (no TSCFUPDT yet).
	 * In coarse mode TSINIT self-clears on the next mac_ptp_ref_clk edge,
	 * not on an accumulator overflow, so it completes immediately.
	 * nanosecond rollover in digital logic
	 */
	data->base->MAC_TIMESTAMP_CONTROL =
		ENET_MAC_TIMESTAMP_CONTROL_TSENA_MASK | ENET_MAC_TIMESTAMP_CONTROL_TSIPV4ENA_MASK |
		ENET_MAC_TIMESTAMP_CONTROL_TSIPV6ENA_MASK |
		ENET_MAC_TIMESTAMP_CONTROL_TSENALL_MASK |
		ENET_MAC_TIMESTAMP_CONTROL_TSEVNTENA_MASK |
		ENET_MAC_TIMESTAMP_CONTROL_SNAPTYPSEL_MASK |
		ENET_MAC_TIMESTAMP_CONTROL_TSCTRLSSR(1) |
		ENET_MAC_TIMESTAMP_CONTROL_TSVER2ENA_MASK | ENET_MAC_TIMESTAMP_CONTROL_TSIPENA_MASK;

	/* Step 2: initialize system time to zero (coarse mode — completes quickly) */
	data->base->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE = 0;
	data->base->MAC_SYSTEM_TIME_SECONDS_UPDATE = 0;
	data->base->MAC_TIMESTAMP_CONTROL |= ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK;
	while (data->base->MAC_TIMESTAMP_CONTROL & ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK) {
	}

	/* Step 3: switch to fine update mode */
	data->base->MAC_TIMESTAMP_CONTROL |= ENET_MAC_TIMESTAMP_CONTROL_TSCFUPDT_MASK;

	/* Step 4: set sub-second increment for 50 MHz PTP clock → 20 ns/tick */
	data->base->MAC_SUB_SECOND_INCREMENT = ENET_MAC_SUB_SECOND_INCREMENT_SNSINC(snsinc);

	/*
	 * Step 5: load the nominal addend into the fine accumulator.
	 * TSENA has already propagated to mac_ptp_ref_clk domain (step 1),
	 * so TSADDREG self-clears promptly.
	 */
	data->base->MAC_TIMESTAMP_ADDEND = data->nominal_addend;
	data->base->MAC_TIMESTAMP_CONTROL |= ENET_MAC_TIMESTAMP_CONTROL_TSADDREG_MASK;
	while (data->base->MAC_TIMESTAMP_CONTROL & ENET_MAC_TIMESTAMP_CONTROL_TSADDREG_MASK) {
	}

#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
	/* MCXN947 implements only the legacy PPS frequency field. Zero selects 1 Hz. */
	data->base->MAC_PPS_CONTROL &= ~ENET_MAC_PPS_CONTROL_PPSCTRL_PPSCMD_MASK;
#endif

	/* Allow the timestamp configuration to propagate to the MAC clock domain. */
	k_busy_wait(10);

	return 0;
}

static DEVICE_API(ptp_clock, ptp_clock_nxp_enet_qos_api) = {
	.set = ptp_clock_nxp_enet_qos_set,
	.get = ptp_clock_nxp_enet_qos_get,
	.adjust = ptp_clock_nxp_enet_qos_adjust,
	.rate_adjust = ptp_clock_nxp_enet_qos_rate_adjust,
	.get_caps = ptp_clock_nxp_enet_qos_get_caps,
#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
	.output = &ptp_clock_nxp_enet_qos_output_provider,
#endif
};

#if defined(CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT)
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_DEFINE(n)                                                \
	PINCTRL_DT_INST_DEFINE(n);                                                               \
	MUX_STATE_DT_SPEC_DEFINE(DT_DRV_INST(n));
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_CONFIG(n)                                                \
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                            \
	.output_gpio = GPIO_DT_SPEC_INST_GET(n, output_gpios),                                  \
	.mux_dev = MUX_STATE_DT_DEV_GET(DT_DRV_INST(n)),                                        \
	.mux_state = MUX_STATE_DT_GET(DT_DRV_INST(n)),
#else
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_DEFINE(n)
#define PTP_CLOCK_NXP_ENET_QOS_OUTPUT_CONFIG(n)
#endif

#define PTP_CLOCK_NXP_ENET_QOS_INIT(n)                                                             \
	PTP_CLOCK_NXP_ENET_QOS_OUTPUT_DEFINE(n)                                                    \
	static const struct ptp_clock_nxp_enet_qos_config ptp_clock_nxp_enet_qos_##n##_config = {  \
		.enet_qos_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                  \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(DT_INST_PARENT(n), ptp)),        \
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL_BY_NAME(                    \
			DT_INST_PARENT(n), ptp, name),                                             \
		PTP_CLOCK_NXP_ENET_QOS_OUTPUT_CONFIG(n)                                         \
	};                                                                                         \
                                                                                                   \
	static struct ptp_clock_nxp_enet_qos_data ptp_clock_nxp_enet_qos_##n##_data;               \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &ptp_clock_nxp_enet_qos_init, NULL,                               \
			      &ptp_clock_nxp_enet_qos_##n##_data,                                  \
			      &ptp_clock_nxp_enet_qos_##n##_config, POST_KERNEL,                   \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &ptp_clock_nxp_enet_qos_api);

DT_INST_FOREACH_STATUS_OKAY(PTP_CLOCK_NXP_ENET_QOS_INIT)
