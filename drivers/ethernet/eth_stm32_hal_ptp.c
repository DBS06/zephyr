/*
 * Copyright (c) 2017 Erwin Rol <erwin@erwinrol.com>
 * Copyright (c) 2020 Alexander Kozhinov <ak.alexander.kozhinov@gmail.com>
 * Copyright (c) 2021 Carbon Robotics
 * Copyright (c) 2025 STMicroelectronics
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <soc.h>

#include <stdbool.h>

#include "eth_stm32_hal_priv.h"

#define DT_DRV_COMPAT snps_dwmac_ptp_clock

LOG_MODULE_REGISTER(eth_stm32_hal_ptp, CONFIG_ETHERNET_LOG_LEVEL);

#if defined(CONFIG_PTP_CLOCK_STM32_HAL_OUTPUT) && defined(ETH_MACHWF2R_PPSOUTNUM_Msk) &&           \
	defined(ETH_MACHWF2R_PPSOUTNUM_Pos) && defined(ETH_MACPPSCR_PPSCTRL_Msk) &&                \
	defined(ETH_MACPPSCR_PPSEN0_Msk) && defined(ETH_MACPPSCR_TRGTMODSEL0_Msk) &&               \
	defined(ETH_MACPPSTTNR_TRGTBUSY0_Msk)
#define ETH_STM32_HAS_FLEX_PPS 1
#else
#define ETH_STM32_HAS_FLEX_PPS 0
#endif

#define ETH_STM32_PPS_OUTPUT_CHANNELS_IMPLEMENTED 1U
#define ETH_STM32_PPS_MIN_LEAD_TIME_NS            1000000ULL
#define ETH_STM32_PPS_BUSY_TIMEOUT_US             1000U
#define ETH_STM32_PPS_START_PULSE_TRAIN           2U
#define ETH_STM32_PPS_CANCEL_START                3U
#define ETH_STM32_PPS_STOP_PULSE_TRAIN_IMMEDIATE  5U
#define ETH_STM32_PPS_TARGET_MODE_OUTPUT_ONLY     3U

#if ETH_STM32_HAS_FLEX_PPS
struct ptp_clock_stm32_data {
	struct k_mutex output_lock;
	struct ptp_clock_output_status output_status;
	uint32_t output_channel_count;
};
#endif

/* Naming of the  ETH PTP Config Status changes depending on the stm32 series */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
#define ETH_STM32_PTP_CONFIGURED HAL_ETH_PTP_CONFIGURATED
#define ETH_STM32_PTP_NOT_CONFIGURED HAL_ETH_PTP_NOT_CONFIGURATED
#else
#define ETH_STM32_PTP_CONFIGURED HAL_ETH_PTP_CONFIGURED
#define ETH_STM32_PTP_NOT_CONFIGURED HAL_ETH_PTP_NOT_CONFIGURED
#endif /* stm32F7x or sm32F4x */

void HAL_ETH_TxPtpCallback(uint32_t *buff, ETH_TimeStampTypeDef *timestamp)
{
	struct eth_stm32_tx_context *ctx = (struct eth_stm32_tx_context *)buff;

	ctx->pkt->timestamp.second = timestamp->TimeStampHigh;
	ctx->pkt->timestamp.nanosecond = timestamp->TimeStampLow;

	net_if_add_tx_timestamp(ctx->pkt);
}

const struct device *eth_stm32_get_ptp_clock(const struct device *dev,
					     struct net_if *iface __unused)
{
	struct eth_stm32_hal_dev_data *dev_data = dev->data;

	return dev_data->ptp_clock;
}

static int ptp_clock_stm32_set(const struct device *dev,
			      struct net_ptp_time *tm)
{
	const struct device *eth_dev = dev->config;
	struct eth_stm32_hal_dev_data *eth_dev_data = eth_dev->data;
	ETH_HandleTypeDef *heth = &eth_dev_data->heth;
	unsigned int key;

	key = irq_lock();

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACSTSUR = tm->second;
	heth->Instance->MACSTNUR = tm->nanosecond;
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSINIT;
	while (heth->Instance->MACTSCR & ETH_MACTSCR_TSINIT_Msk) {
		/* spin lock */
	}
#else
	heth->Instance->PTPTSHUR = tm->second;
	heth->Instance->PTPTSLUR = tm->nanosecond;
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSSTI;
	while (heth->Instance->PTPTSCR & ETH_PTPTSCR_TSSTI_Msk) {
		/* spin lock */
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */

	irq_unlock(key);

	return 0;
}

static int ptp_clock_stm32_get(const struct device *dev,
			      struct net_ptp_time *tm)
{
	const struct device *eth_dev = dev->config;
	struct eth_stm32_hal_dev_data *eth_dev_data = eth_dev->data;
	ETH_HandleTypeDef *heth = &eth_dev_data->heth;
	unsigned int key;
	uint32_t second_2;

	key = irq_lock();

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	tm->second = heth->Instance->MACSTSR;
	tm->nanosecond = heth->Instance->MACSTNR;
	second_2 = heth->Instance->MACSTSR;
#else
	tm->second = heth->Instance->PTPTSHR;
	tm->nanosecond = heth->Instance->PTPTSLR;
	second_2 = heth->Instance->PTPTSHR;
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */

	irq_unlock(key);

	if (tm->second != second_2 && tm->nanosecond < NSEC_PER_SEC / 2) {
		/* Second rollover has happened during first measurement: second register
		 * was read before second boundary and nanosecond register was read after.
		 * We will use second_2 as a new second value.
		 */
		tm->second = second_2;
	}

	return 0;
}

static int ptp_clock_stm32_adjust(const struct device *dev, int increment)
{
	const struct device *eth_dev = dev->config;
	struct eth_stm32_hal_dev_data *eth_dev_data = eth_dev->data;
	ETH_HandleTypeDef *heth = &eth_dev_data->heth;
	int key, ret;

	if ((increment <= (int32_t)(-NSEC_PER_SEC)) ||
			(increment >= (int32_t)NSEC_PER_SEC)) {
		ret = -EINVAL;
	} else {
		key = irq_lock();

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
		heth->Instance->MACSTSUR = 0;
		if (increment >= 0) {
			heth->Instance->MACSTNUR = increment;
		} else {
			heth->Instance->MACSTNUR = ETH_MACSTNUR_ADDSUB | (NSEC_PER_SEC + increment);
		}
		heth->Instance->MACTSCR |= ETH_MACTSCR_TSUPDT;
		while (heth->Instance->MACTSCR & ETH_MACTSCR_TSUPDT_Msk) {
			/* spin lock */
		}
#else
		heth->Instance->PTPTSHUR = 0;
		if (increment >= 0) {
			heth->Instance->PTPTSLUR = increment;
		} else {
			heth->Instance->PTPTSLUR = ETH_PTPTSLUR_TSUPNS | (-increment);
		}
		heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSSTU;
		while (heth->Instance->PTPTSCR & ETH_PTPTSCR_TSSTU_Msk) {
			/* spin lock */
		}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */

		ret = 0;
		irq_unlock(key);
	}

	return ret;
}

static int ptp_clock_stm32_rate_adjust(const struct device *dev, double ratio)
{
	const struct device *eth_dev = dev->config;
	struct eth_stm32_hal_dev_data *eth_dev_data = eth_dev->data;
	ETH_HandleTypeDef *heth = &eth_dev_data->heth;
	int key, ret;
	uint32_t addend_val;

	key = irq_lock();

	/* Limit possible ratio */
	if (ratio * 100 < CONFIG_ETH_STM32_HAL_PTP_CLOCK_ADJ_MIN_PCT ||
			ratio * 100 > CONFIG_ETH_STM32_HAL_PTP_CLOCK_ADJ_MAX_PCT) {
		ret = -EINVAL;
		goto error;
	}

	/* Update addend register */
	addend_val = UINT32_MAX * (double)eth_dev_data->clk_ratio * ratio;

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACTSAR = addend_val;
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSADDREG;
	while (heth->Instance->MACTSCR & ETH_MACTSCR_TSADDREG_Msk) {
		/* spin lock */
	}
#else
	heth->Instance->PTPTSAR = addend_val;
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSARU;
	while (heth->Instance->PTPTSCR & ETH_PTPTSCR_TSARU_Msk) {
		/* spin lock */
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */

	ret = 0;

error:
	irq_unlock(key);

	return ret;
}

#if ETH_STM32_HAS_FLEX_PPS
static uint32_t ptp_clock_stm32_output_resolution_ns(void)
{
	return NSEC_PER_SEC / CONFIG_ETH_STM32_HAL_PTP_CLOCK_SRC_HZ;
}

static int ptp_clock_stm32_output_channel_validate(const struct device *dev, uint32_t channel)
{
	struct ptp_clock_stm32_data *data = dev->data;

	if (channel >= data->output_channel_count) {
		return -ENOTSUP;
	}

	return 0;
}

static int ptp_clock_stm32_output_wait_ready(ETH_HandleTypeDef *heth)
{
	for (uint32_t elapsed_us = 0U; elapsed_us < ETH_STM32_PPS_BUSY_TIMEOUT_US; elapsed_us++) {
		uint32_t control = heth->Instance->MACPPSCR;
		bool command_busy = (control & ETH_MACPPSCR_PPSEN0_Msk) != 0U &&
				    (control & ETH_MACPPSCR_PPSCTRL_Msk) != 0U;

		if ((heth->Instance->MACPPSTTNR & ETH_MACPPSTTNR_TRGTBUSY0_Msk) == 0U &&
		    !command_busy) {
			return 0;
		}

		k_busy_wait(1U);
	}

	return -EBUSY;
}

static void ptp_clock_stm32_output_command(ETH_HandleTypeDef *heth, uint32_t command,
					   uint32_t target_mode)
{
	uint32_t control = heth->Instance->MACPPSCR;

	control &= ~(ETH_MACPPSCR_PPSCTRL_Msk | ETH_MACPPSCR_PPSEN0_Msk |
		     ETH_MACPPSCR_TRGTMODSEL0_Msk);
#ifdef ETH_MACPPSCR_MCGREN0_Pos
	/* Some STM32Cube headers define MCGREN0_Msk with the unrelated TIMESEL position. */
	control &= ~BIT(ETH_MACPPSCR_MCGREN0_Pos);
#endif
	control |= (command << ETH_MACPPSCR_PPSCTRL_Pos) & ETH_MACPPSCR_PPSCTRL_Msk;
	control |= (target_mode << ETH_MACPPSCR_TRGTMODSEL0_Pos) & ETH_MACPPSCR_TRGTMODSEL0_Msk;
	control |= ETH_MACPPSCR_PPSEN0_Msk;
	heth->Instance->MACPPSCR = control;
}

static int ptp_clock_stm32_output_disable(ETH_HandleTypeDef *heth)
{
	int ret;

	ret = ptp_clock_stm32_output_wait_ready(heth);
	if (ret < 0) {
		return ret;
	}

	/* Cancel an armed start before stopping a pulse train which already began. */
	ptp_clock_stm32_output_command(heth, ETH_STM32_PPS_CANCEL_START,
				       ETH_STM32_PPS_TARGET_MODE_OUTPUT_ONLY);
	ret = ptp_clock_stm32_output_wait_ready(heth);
	if (ret < 0) {
		return ret;
	}

	ptp_clock_stm32_output_command(heth, ETH_STM32_PPS_STOP_PULSE_TRAIN_IMMEDIATE,
				       ETH_STM32_PPS_TARGET_MODE_OUTPUT_ONLY);

	return ptp_clock_stm32_output_wait_ready(heth);
}

static int ptp_clock_stm32_output_ticks(uint64_t duration_ns, uint64_t min_ticks,
					uint32_t resolution_ns, uint32_t *register_value)
{
	uint64_t ticks;

	if ((duration_ns % resolution_ns) != 0U) {
		return -ERANGE;
	}

	ticks = duration_ns / resolution_ns;
	if (ticks < min_ticks || ticks > UINT32_MAX) {
		return -ERANGE;
	}

	*register_value = (uint32_t)(ticks - 1U);

	return 0;
}

static bool ptp_clock_stm32_output_has_lead_time(const struct net_ptp_time *now,
						 const struct net_ptp_time *start)
{
	uint64_t second_delta;
	uint64_t lead_time_ns;

	if (start->second < now->second) {
		return false;
	}

	second_delta = start->second - now->second;
	if (second_delta > 1U) {
		return true;
	}

	if (second_delta == 0U) {
		if (start->nanosecond <= now->nanosecond) {
			return false;
		}

		lead_time_ns = start->nanosecond - now->nanosecond;
	} else {
		lead_time_ns = NSEC_PER_SEC - now->nanosecond + start->nanosecond;
	}

	return lead_time_ns >= ETH_STM32_PPS_MIN_LEAD_TIME_NS;
}

static int ptp_clock_stm32_get_output_caps(const struct device *dev, uint32_t channel,
					   struct ptp_clock_output_caps *caps)
{
	uint32_t resolution_ns = ptp_clock_stm32_output_resolution_ns();
	uint64_t max_duration_ns = (uint64_t)UINT32_MAX * resolution_ns;
	int ret;

	if (caps == NULL) {
		return -EINVAL;
	}

	ret = ptp_clock_stm32_output_channel_validate(dev, channel);
	if (ret < 0) {
		return ret;
	}

	*caps = (struct ptp_clock_output_caps){
		.flags = PTP_CLOCK_OUTPUT_CAP_PERIODIC | PTP_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH,
		.channel_count = ((struct ptp_clock_stm32_data *)dev->data)->output_channel_count,
		.resolution_ns = resolution_ns,
		.min_lead_time_ns = ETH_STM32_PPS_MIN_LEAD_TIME_NS,
		.min_period_ns = 2U * resolution_ns,
		.max_period_ns = max_duration_ns,
		.min_pulse_width_ns = resolution_ns,
		.max_pulse_width_ns = max_duration_ns,
	};

	return 0;
}

static int ptp_clock_stm32_output_start(const struct device *dev, uint32_t channel,
					const struct ptp_clock_output_config *config)
{
	const struct device *eth_dev = dev->config;
	struct eth_stm32_hal_dev_data *eth_dev_data = eth_dev->data;
	struct ptp_clock_stm32_data *data = dev->data;
	ETH_HandleTypeDef *heth = &eth_dev_data->heth;
	uint32_t resolution_ns = ptp_clock_stm32_output_resolution_ns();
	struct net_ptp_time now;
	uint32_t interval;
	uint32_t width;
	unsigned int key;
	int ret;

	if (config == NULL) {
		return -EINVAL;
	}

	ret = ptp_clock_stm32_output_channel_validate(dev, channel);
	if (ret < 0) {
		return ret;
	}

	if (config->period_ns == 0U || config->pulse_width_ns == 0U ||
	    config->pulse_width_ns >= config->period_ns) {
		return -EINVAL;
	}
	if (config->start_time.nanosecond >= NSEC_PER_SEC) {
		return -EINVAL;
	}
	if (config->start_time.second > UINT32_MAX) {
		return -ERANGE;
	}

	if ((config->start_time.nanosecond % resolution_ns) != 0U) {
		return -ERANGE;
	}

	ret = ptp_clock_stm32_output_ticks(config->period_ns, 2U, resolution_ns, &interval);
	if (ret < 0) {
		return ret;
	}

	ret = ptp_clock_stm32_output_ticks(config->pulse_width_ns, 1U, resolution_ns, &width);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->output_lock, K_FOREVER);

	if (data->output_status.active) {
		ret = -EBUSY;
		goto unlock;
	}

	ret = ptp_clock_stm32_output_wait_ready(heth);
	if (ret < 0) {
		goto unlock;
	}

	/* Keep the final lead-time check and command issue atomic with PHC updates. */
	key = irq_lock();
	ret = ptp_clock_stm32_get(dev, &now);
	if (ret < 0) {
		irq_unlock(key);
		goto unlock;
	}
	if (!ptp_clock_stm32_output_has_lead_time(&now, &config->start_time)) {
		ret = -ETIME;
		irq_unlock(key);
		goto unlock;
	}

	/* RM0481 requires target nanoseconds first and the control command last. */
	heth->Instance->MACPPSTTNR = config->start_time.nanosecond;
	heth->Instance->MACPPSTTSR = (uint32_t)config->start_time.second;
	heth->Instance->MACPPSIR = interval;
	heth->Instance->MACPPSWR = width;

	/* Activate pulse-train generation only after every parameter is valid. */
	ptp_clock_stm32_output_command(heth, ETH_STM32_PPS_START_PULSE_TRAIN,
				       ETH_STM32_PPS_TARGET_MODE_OUTPUT_ONLY);
	irq_unlock(key);
	ret = ptp_clock_stm32_output_wait_ready(heth);
	data->output_status = (struct ptp_clock_output_status){
		.active = true,
		.config = *config,
	};
	if (ret < 0) {
		/* The pending command can still arm the output; keep status conservative. */
		goto unlock;
	}

unlock:
	k_mutex_unlock(&data->output_lock);

	return ret;
}

static int ptp_clock_stm32_output_stop(const struct device *dev, uint32_t channel)
{
	const struct device *eth_dev = dev->config;
	struct eth_stm32_hal_dev_data *eth_dev_data = eth_dev->data;
	struct ptp_clock_stm32_data *data = dev->data;
	ETH_HandleTypeDef *heth = &eth_dev_data->heth;
	int ret;

	ret = ptp_clock_stm32_output_channel_validate(dev, channel);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->output_lock, K_FOREVER);

	if (!data->output_status.active) {
		ret = 0;
		goto unlock;
	}

	ret = ptp_clock_stm32_output_disable(heth);
	if (ret < 0) {
		goto unlock;
	}
	data->output_status.active = false;

unlock:
	k_mutex_unlock(&data->output_lock);

	return ret;
}

static int ptp_clock_stm32_get_output_status(const struct device *dev, uint32_t channel,
					     struct ptp_clock_output_status *status)
{
	struct ptp_clock_stm32_data *data = dev->data;
	int ret;

	if (status == NULL) {
		return -EINVAL;
	}

	ret = ptp_clock_stm32_output_channel_validate(dev, channel);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->output_lock, K_FOREVER);
	*status = data->output_status;
	k_mutex_unlock(&data->output_lock);

	return 0;
}
#endif /* ETH_STM32_HAS_FLEX_PPS */

static int ptp_clock_stm32_get_caps(const struct device *dev, struct ptp_clock_caps *caps)
{
#if ETH_STM32_HAS_FLEX_PPS
	struct ptp_clock_stm32_data *data = dev->data;
#endif
	int64_t min_rate_ppb;
	int64_t max_rate_ppb;
	uint32_t flags;

	if (caps == NULL) {
		return -EINVAL;
	}

	min_rate_ppb =
		((int64_t)CONFIG_ETH_STM32_HAL_PTP_CLOCK_ADJ_MIN_PCT - 100) * (NSEC_PER_SEC / 100);
	max_rate_ppb =
		((int64_t)CONFIG_ETH_STM32_HAL_PTP_CLOCK_ADJ_MAX_PCT - 100) * (NSEC_PER_SEC / 100);
	flags = PTP_CLOCK_CAP_READ | PTP_CLOCK_CAP_SET | PTP_CLOCK_CAP_ADJUST |
		PTP_CLOCK_CAP_RATE_ADJUST;
#if ETH_STM32_HAS_FLEX_PPS
	if (data->output_channel_count > 0U) {
		flags |= PTP_CLOCK_CAP_SCHEDULED_OUTPUT;
	}
#else
	ARG_UNUSED(dev);
#endif

	*caps = (struct ptp_clock_caps){
		.flags = flags,
		.resolution_ns = NSEC_PER_SEC / CONFIG_ETH_STM32_HAL_PTP_CLOCK_SRC_HZ,
		.max_adjust_ns = NSEC_PER_SEC - 1,
		.min_rate_ppb = (int32_t)CLAMP(min_rate_ppb, INT32_MIN, INT32_MAX),
		.max_rate_ppb = (int32_t)CLAMP(max_rate_ppb, INT32_MIN, INT32_MAX),
	};

	return 0;
}

static void eth_stm32_ptp_enable_timestamping(ETH_HandleTypeDef *heth)
{
	/* Mask the Timestamp Trigger interrupt and enable timestamping */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACIER &= ~(ETH_MACIER_TSIE);
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSENA;
#else
	heth->Instance->MACIMR &= ~(ETH_MACIMR_TSTIM);
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSE;
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */
}

static void eth_stm32_ptp_set_addend(ETH_HandleTypeDef *heth, uint32_t addend_val)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACTSAR = addend_val;
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSADDREG;
	while (heth->Instance->MACTSCR & ETH_MACTSCR_TSADDREG_Msk) {
		k_yield();
	}
#else
	heth->Instance->PTPTSAR = addend_val;
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSARU;
	while (heth->Instance->PTPTSCR & ETH_PTPTSCR_TSARU_Msk) {
		k_yield();
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */
}

static void eth_stm32_ptp_enable_fine_timestamp_update(ETH_HandleTypeDef *heth)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSCFUPDT;
#else
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSFCU;
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */
}

static void eth_stm32_ptp_enable_nsec_rollover(ETH_HandleTypeDef *heth)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSCTRLSSR;
#else
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSSSR;
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */
}

static void eth_stm32_ptp_init_timestamp(ETH_HandleTypeDef *heth)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACSTSUR = 0;
	heth->Instance->MACSTNUR = 0;
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSINIT;
	while (heth->Instance->MACTSCR & ETH_MACTSCR_TSINIT_Msk) {
		k_yield();
	}
#else
	heth->Instance->PTPTSHUR = 0;
	heth->Instance->PTPTSLUR = 0;
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSSTI;
	while (heth->Instance->PTPTSCR & ETH_PTPTSCR_TSSTI_Msk) {
		k_yield();
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */
}

static DEVICE_API(ptp_clock, api) = {
	.set = ptp_clock_stm32_set,
	.get = ptp_clock_stm32_get,
	.adjust = ptp_clock_stm32_adjust,
	.rate_adjust = ptp_clock_stm32_rate_adjust,
	.get_caps = ptp_clock_stm32_get_caps,
#if ETH_STM32_HAS_FLEX_PPS
	.get_output_caps = ptp_clock_stm32_get_output_caps,
	.output_start = ptp_clock_stm32_output_start,
	.output_stop = ptp_clock_stm32_output_stop,
	.get_output_status = ptp_clock_stm32_get_output_status,
#endif
};

BUILD_ASSERT(NSEC_PER_SEC % CONFIG_ETH_STM32_HAL_PTP_CLOCK_SRC_HZ == 0,
	     "PTP clock period must be an integer nanosecond value");

BUILD_ASSERT(NSEC_PER_SEC / CONFIG_ETH_STM32_HAL_PTP_CLOCK_SRC_HZ <= UINT8_MAX,
	     "PTP clock period is more than 255 nanoseconds");

static int ptp_stm32_init(const struct device *dev)
{
	const struct device *const eth_dev = dev->config;
	struct eth_stm32_hal_dev_data *eth_dev_data = eth_dev->data;
#if ETH_STM32_HAS_FLEX_PPS
	struct ptp_clock_stm32_data *data = dev->data;
#endif
	const struct eth_stm32_hal_dev_cfg *eth_cfg = eth_dev->config;
	ETH_HandleTypeDef *heth = &eth_dev_data->heth;
	int ret;
	uint32_t ptp_clk_rate;
	uint32_t ss_incr_ns = NSEC_PER_SEC / CONFIG_ETH_STM32_HAL_PTP_CLOCK_SRC_HZ;
	uint32_t addend_val;

#if ETH_STM32_HAS_FLEX_PPS
	k_mutex_init(&data->output_lock);
	data->output_status = (struct ptp_clock_output_status){0};
	data->output_channel_count = 0U;
#endif

	eth_dev_data->ptp_clock = dev;

	eth_stm32_ptp_enable_timestamping(heth);

	/* Query the MAC timestamp reference clock rate */
	clock_control_subsys_t rate_clk = (void *)&eth_cfg->pclken[eth_cfg->rate_pclken_idx];

	ret = clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE), rate_clk,
				     &ptp_clk_rate);
	if (ret) {
		LOG_ERR("Failed to query PTP reference clock");
		return -EIO;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACSSIR = ss_incr_ns << ETH_MACMACSSIR_SSINC_Pos;
#else
	heth->Instance->PTPSSIR = ss_incr_ns;
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */

	/* Program timestamp addend register */
	eth_dev_data->clk_ratio =
		((double)CONFIG_ETH_STM32_HAL_PTP_CLOCK_SRC_HZ) / ((double)ptp_clk_rate);
	/*
	 * clk_ratio is the ratio between the desired PTP clock frequency and the
	 * MAC timestamp reference clock. Because that reference is derived from a
	 * physical oscillator, it might drift due to manufacturing tolerances and
	 * environmental effects (e.g. temperature). It gets adjusted by calling
	 * ptp_clock_stm32_rate_adjust().
	 */
	addend_val =
		UINT32_MAX * eth_dev_data->clk_ratio;

	eth_stm32_ptp_set_addend(heth, addend_val);

	eth_stm32_ptp_enable_fine_timestamp_update(heth);

	eth_stm32_ptp_enable_nsec_rollover(heth);

	eth_stm32_ptp_init_timestamp(heth);

#if ETH_STM32_HAS_FLEX_PPS
	data->output_channel_count = MIN((heth->Instance->MACHWF2R & ETH_MACHWF2R_PPSOUTNUM_Msk) >>
						 ETH_MACHWF2R_PPSOUTNUM_Pos,
					 ETH_STM32_PPS_OUTPUT_CHANNELS_IMPLEMENTED);
	if (data->output_channel_count > 0U) {
		ret = ptp_clock_stm32_output_disable(heth);
		if (ret < 0) {
			LOG_WRN("Disabling scheduled output: PPS channel did not become idle");
			data->output_channel_count = 0U;
		}
	}
#endif

	/* Set PTP Configuration done */
	heth->IsPtpConfigured = ETH_STM32_PTP_CONFIGURED;

	return 0;
}

#if ETH_STM32_HAS_FLEX_PPS
#define PTP_CLOCK_STM32_DATA_DEFINE(n) static struct ptp_clock_stm32_data ptp_clock_stm32_data_##n;
#define PTP_CLOCK_STM32_DATA_GET(n)    &ptp_clock_stm32_data_##n
#else
#define PTP_CLOCK_STM32_DATA_DEFINE(n)
#define PTP_CLOCK_STM32_DATA_GET(n) NULL
#endif

#define PTP_CLOCK_STM32_INIT(n)                                                                    \
	PTP_CLOCK_STM32_DATA_DEFINE(n)                                                             \
	DEVICE_DT_INST_DEFINE(n, ptp_stm32_init, NULL, PTP_CLOCK_STM32_DATA_GET(n),                \
			      DEVICE_DT_GET(DT_INST_PARENT(n)), POST_KERNEL,                       \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(PTP_CLOCK_STM32_INIT)
