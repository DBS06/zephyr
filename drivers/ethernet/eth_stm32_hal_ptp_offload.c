/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/net/net_core.h>

#include "eth_stm32_hal_priv.h"
#include "eth_stm32_hal_ptp_offload.h"

LOG_MODULE_DECLARE(eth_stm32_hal, CONFIG_ETHERNET_LOG_LEVEL);

#define TX_TIMEOUT            K_MSEC(20)
/* RM0481 MACTSCR bit 28, absent from the STM32H563 CMSIS header. */
#define STM32_PTP_AV8021ASMEN BIT(28)
#define TS_MODE_MASK                                                                               \
	(ETH_MACTSCR_SNAPTYPSEL | ETH_MACTSCR_TSMSTRENA | ETH_MACTSCR_TSEVNTENA |                  \
	 STM32_PTP_AV8021ASMEN)

BUILD_ASSERT(ETH_TX_DESC_CNT >= 4, "PTP TX needs context, packet and a free ring slot");
BUILD_ASSERT(STM32_PTP_OSTC == ETH_DMATXCDESC_OSTC);

static uint16_t next_desc(uint16_t index)
{
	return (index + 1U) % ETH_TX_DESC_CNT;
}

void eth_stm32_ptp_offload_fault(struct eth_stm32_hal_dev_data *data)
{
	/* Also callable from the Ethernet error ISR. No mutex or clock reset. */
	data->heth.Instance->MACPOCR = 0;
	for (size_t i = 0; i < ETH_TX_DESC_CNT; i++) {
		data->ptp_tx_abandoned[i] = true;
	}
	atomic_set(&data->ptp_fault, 1);
}

/* Called with ptp_lock held. Each frame has one context and one data descriptor. */
static void reap_tx(struct eth_stm32_hal_dev_data *data)
{
	ETH_DMADescTypeDef *ring = data->heth.Init.TxDesc;

	while (data->ptp_tx_used != 0U) {
		uint16_t first = data->ptp_tx_tail;
		uint16_t last = next_desc(first);
		struct net_pkt *pkt = data->ptp_tx_pkt[last];
		uint32_t status = ring[last].DESC3;

		if ((status & ETH_DMATXNDESCRF_OWN) != 0U) {
			if (k_uptime_get() - data->ptp_tx_started[last] >= 20) {
				data->ptp_tx_abandoned[last] = true;
				eth_stm32_ptp_offload_fault(data);
			}
			break;
		}
		__DMB();
		if (!data->ptp_tx_abandoned[last] &&
		    (ring[first].DESC3 & ETH_DMATXCDESC_CDE) == 0U &&
		    (status & ETH_DMATXNDESCWBF_ES) == 0U &&
		    (status & ETH_DMATXNDESCWBF_TTSS) != 0U && net_pkt_is_tx_timestamping(pkt)) {
			pkt->timestamp.second = ring[last].DESC1;
			pkt->timestamp.nanosecond = ring[last].DESC0;
			if (pkt->timestamp.nanosecond < NSEC_PER_SEC) {
				net_if_add_tx_timestamp(pkt);
			}
		}
		if ((status & ETH_DMATXNDESCWBF_ES) != 0U ||
		    (ring[first].DESC3 & ETH_DMATXCDESC_CDE) != 0U) {
			eth_stm32_ptp_offload_fault(data);
		}
		data->ptp_tx_pkt[last] = NULL;
		net_pkt_unref(pkt);
		data->ptp_tx_tail = next_desc(last);
		data->ptp_tx_used -= 2U;
	}
}

static void tx_work(struct k_work *work)
{
	struct eth_stm32_hal_dev_data *data = CONTAINER_OF(
		k_work_delayable_from_work(work), struct eth_stm32_hal_dev_data, ptp_tx_work);

	if (k_mutex_lock(&data->ptp_lock, K_NO_WAIT) != 0) {
		k_work_reschedule(&data->ptp_tx_work, K_MSEC(1));
		return;
	}
	reap_tx(data);
	if (data->ptp_tx_used != 0U) {
		k_work_reschedule(&data->ptp_tx_work,
				  atomic_get(&data->ptp_fault) != 0 ? K_MSEC(20) : K_MSEC(1));
	}
	k_mutex_unlock(&data->ptp_lock);
}

void eth_stm32_ptp_offload_init(const struct device *dev)
{
	struct eth_stm32_hal_dev_data *data = dev->data;

	k_mutex_init(&data->ptp_lock);
	k_work_init_delayable(&data->ptp_tx_work, tx_work);
	data->ptp_config.capabilities = STM32_PTP_CAPS;
	data->ptp_config.generation = 1;
	data->ptp_stopping = true;
}

int eth_stm32_ptp_offload_stop(const struct device *dev)
{
	struct eth_stm32_hal_dev_data *data = dev->data;
	ETH_TypeDef *eth = data->heth.Instance;
	ETH_DMADescTypeDef *ring = data->heth.Init.TxDesc;
	k_timepoint_t deadline = sys_timepoint_calc(TX_TIMEOUT);

	/* Called with ptp_lock held after HAL has stopped both DMA directions. */
	while ((eth->DMADSR & (ETH_DMADSR_RPS | ETH_DMADSR_TPS)) != 0U) {
		if (sys_timepoint_expired(deadline)) {
			eth_stm32_ptp_offload_fault(data);
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(1));
	}
	for (size_t i = 0; i < ETH_TX_DESC_CNT; i++) {
		if (data->ptp_tx_pkt[i] != NULL) {
			net_pkt_unref(data->ptp_tx_pkt[i]);
			data->ptp_tx_pkt[i] = NULL;
		}
		ring[i].DESC3 = 0;
	}
	data->ptp_tx_head = 0;
	data->ptp_tx_tail = 0;
	data->ptp_tx_used = 0;
	eth->DMACTDLAR = (uint32_t)(uintptr_t)ring;
	eth->DMACTDTPR = (uint32_t)(uintptr_t)ring;
	return 0;
}

int eth_stm32_ptp_offload_tx(const struct device *dev, struct net_pkt *pkt)
{
	const struct eth_stm32_hal_dev_cfg *hw = dev->config;
	struct eth_stm32_hal_dev_data *data = dev->data;
	ETH_TypeDef *eth = data->heth.Instance;
	ETH_DMADescTypeDef *ring = data->heth.Init.TxDesc;
	size_t len = net_pkt_get_len(pkt);
	uint32_t context[4];
	uint16_t first, last, next;
	unsigned int key;
	k_timepoint_t deadline = sys_timepoint_calc(TX_TIMEOUT);
	int ret;

	if (len > ETH_STM32_TX_BUF_SIZE) {
		return -EMSGSIZE;
	}
	ret = k_mutex_lock(&data->ptp_lock, TX_TIMEOUT);
	if (ret != 0) {
		return ret;
	}
	while (data->ptp_tx_used + 2U >= ETH_TX_DESC_CNT) {
		reap_tx(data);
		if (sys_timepoint_expired(deadline)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		k_sem_take(&data->tx_int_sem, K_MSEC(1));
	}
	if ((atomic_get(&data->ptp_fault) != 0 &&
	     (pkt->ptp.flags & NET_PTP_PACKET_ONE_STEP) != 0U) || data->ptp_stopping ||
	    data->heth.gState != HAL_ETH_STATE_STARTED) {
		ret = -EIO;
		goto out;
	}
	first = data->ptp_tx_head;
	last = next_desc(first);
	next = next_desc(last);
	ret = net_pkt_read(pkt, hw->dma_buf->tx_buf[last], len);
	if (ret != 0) {
		goto out;
	}
	ret = stm32_ptp_context(&data->ptp_config, &pkt->ptp, hw->dma_buf->tx_buf[last], len,
				context);
	if (ret != 0) {
		goto out;
	}
	key = irq_lock();
	if (atomic_get(&data->ptp_fault) != 0 &&
	    (pkt->ptp.flags & NET_PTP_PACKET_ONE_STEP) != 0U) {
		irq_unlock(key);
		ret = -EIO;
		goto out;
	}
	ring[first].DESC0 = context[0];
	ring[first].DESC1 = context[1];
	ring[first].DESC2 = context[2];
	ring[first].DESC3 = context[3];
	ring[last].DESC0 = (uint32_t)(uintptr_t)hw->dma_buf->tx_buf[last];
	ring[last].DESC1 = 0;
	ring[last].DESC2 = len | ETH_DMATXNDESCRF_IOC;
	if ((pkt->ptp.flags & NET_PTP_PACKET_ONE_STEP) == 0U && net_pkt_is_tx_timestamping(pkt)) {
		ring[last].DESC2 |= ETH_DMATXNDESCRF_TTSE;
	}
	ring[last].DESC3 = len | ETH_DMATXNDESCRF_FD | ETH_DMATXNDESCRF_LD;
	data->ptp_tx_pkt[last] = net_pkt_ref(pkt);
	data->ptp_tx_started[last] = k_uptime_get();
	data->ptp_tx_abandoned[last] = false;
	data->ptp_tx_used += 2U;
	data->ptp_tx_head = next;
	/* Publish the first descriptor last, after the entire chain is visible. */
	__DMB();
	ring[last].DESC3 |= ETH_DMATXNDESCRF_OWN;
	__DMB();
	ring[first].DESC3 |= ETH_DMATXNDESCRF_OWN;
	__DSB();
	eth->DMACSR = ETH_DMACSR_TBU;
	eth->DMACTDTPR = (uint32_t)(uintptr_t)&ring[next];
	irq_unlock(key);
	k_work_reschedule(&data->ptp_tx_work, K_MSEC(1));
	ret = 0;
	if (!IS_ENABLED(CONFIG_ETH_STM32_HAL_TX_ASYNC)) {
		while ((ring[last].DESC3 & ETH_DMATXNDESCRF_OWN) != 0U) {
			if (sys_timepoint_expired(deadline)) {
				/* DMA still owns the buffer: retain it until completion. */
				data->ptp_tx_abandoned[last] = true;
				eth_stm32_ptp_offload_fault(data);
				ret = -ETIMEDOUT;
				break;
			}
			k_sem_take(&data->tx_int_sem, K_MSEC(1));
		}
		if (ret == 0 && ((ring[last].DESC3 & ETH_DMATXNDESCWBF_ES) != 0U ||
				 (ring[first].DESC3 & ETH_DMATXCDESC_CDE) != 0U)) {
			ret = -EIO;
		}
		reap_tx(data);
	}
out:
	k_mutex_unlock(&data->ptp_lock);
	return ret;
}

void eth_stm32_ptp_offload_rx(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_stm32_hal_dev_data *data = dev->data;
	uint8_t header[68];
	size_t len = net_pkt_get_len(pkt);

	pkt->ptp.flags = NET_PTP_PACKET_RX_VALID;
	pkt->ptp.generation = data->ptp_config.generation;
	net_pkt_cursor_init(pkt);
	if (len >= 48U && net_pkt_read(pkt, header, MIN(len, sizeof(header))) == 0 &&
	    stm32_ptp_response_owned(&data->ptp_config, header, len, data->mac_addr)) {
		pkt->ptp.flags |= NET_PTP_PACKET_RX_RESPONDED;
	}
	net_pkt_cursor_init(pkt);
}

int eth_stm32_ptp_offload_get(const struct device *dev, struct net_if *iface,
			      enum ethernet_config_type type, struct ethernet_config *config)
{
	struct eth_stm32_hal_dev_data *data = dev->data;

	ARG_UNUSED(iface);
	if (type != ETHERNET_CONFIG_TYPE_PTP) {
		return -ENOTSUP;
	}
	k_mutex_lock(&data->ptp_lock, K_FOREVER);
	config->ptp = data->ptp_config;
	if (atomic_get(&data->ptp_fault) != 0 || data->ptp_stopping ||
	    data->heth.gState != HAL_ETH_STATE_STARTED) {
		/* Report the old ownership until configure drains its RX generation. */
		config->ptp.capabilities = 0;
	}
	k_mutex_unlock(&data->ptp_lock);
	return 0;
}

int eth_stm32_ptp_offload_configure(const struct device *dev,
				    const struct ethernet_ptp_config *config)
{
	struct eth_stm32_hal_dev_data *data = dev->data;
	ETH_TypeDef *eth = data->heth.Instance;
	k_timepoint_t deadline = sys_timepoint_calc(TX_TIMEOUT);
	uint32_t ops = config->operations;
	uint32_t receiver, dma_receiver, mode;
	uint32_t control = (uint32_t)config->domain << ETH_MACPOCR_DN_Pos;
	unsigned int key;
	int ret = stm32_ptp_validate(config);

	if (ret != 0) {
		return ret;
	}
	ret = k_mutex_lock(&data->ptp_lock, TX_TIMEOUT);
	if (ret != 0) {
		return ret;
	}
	if (data->heth.gState != HAL_ETH_STATE_STARTED &&
	    (data->ptp_config.operations & STM32_PTP_AUTO) != 0U) {
		/* HAL cannot drain RX in its error state. Keep the old generation
		 * until device recovery, rather than relabeling queued requests.
		 */
		eth_stm32_ptp_offload_fault(data);
		ret = -EIO;
		goto out;
	}
	if (atomic_get(&data->ptp_fault) != 0 && ops != 0U) {
		ret = -EIO;
		goto out;
	}
	if (ops != 0U && (data->ptp_stopping || data->heth.gState != HAL_ETH_STATE_STARTED)) {
		ret = -ENETDOWN;
		goto out;
	}
	receiver = eth->MACCR & ETH_MACCR_RE;
	dma_receiver = eth->DMACRCR & ETH_DMACRCR_SR;
	eth->MACCR &= ~ETH_MACCR_RE;
	/* Stop periodic generation, but preserve response ownership while draining. */
	eth->MACPOCR &= ~ETH_MACPOCR_ASYNCEN;
	while (true) {
		struct net_pkt *pkt = eth_stm32_rx_locked(dev);

		if (pkt != NULL && net_recv_data(data->iface, pkt) < 0) {
			net_pkt_unref(pkt);
		}
		reap_tx(data);
		if (pkt == NULL && data->ptp_tx_used == 0U &&
		    (eth->MACDR & (ETH_MACDR_RPESTS | ETH_MACDR_TPESTS)) == 0U &&
		    (eth->MTLRQDR & (ETH_MTLRQDR_PRXQ | ETH_MTLRQDR_RXQSTS)) == 0U &&
		    (eth->MTLTQDR & ETH_MTLTQDR_TXQSTS) == 0U) {
			/* FIFO empty is not sufficient: wait for the RX descriptor and
			 * timestamp write-backs, then drain the final completed packet.
			 */
			eth->DMACRCR &= ~ETH_DMACRCR_SR;
			__DSB();
			if ((eth->DMADSR & ETH_DMADSR_RPS) == 0U) {
				pkt = eth_stm32_rx_locked(dev);
				if (pkt == NULL && !eth_stm32_rx_pending(dev)) {
					break;
				}
				if (pkt != NULL && net_recv_data(data->iface, pkt) < 0) {
					net_pkt_unref(pkt);
				}
			}
		}
		if (sys_timepoint_expired(deadline)) {
			eth_stm32_ptp_offload_fault(data);
			ret = -ETIMEDOUT;
			goto restore_rx;
		}
		k_sleep(K_USEC(100));
	}
	key = irq_lock();
	if ((atomic_get(&data->ptp_fault) != 0 && ops != 0U) ||
	    (data->heth.gState != HAL_ETH_STATE_STARTED &&
	     (data->ptp_config.operations & STM32_PTP_AUTO) != 0U)) {
		irq_unlock(key);
		ret = -EIO;
		goto restore_rx;
	}
	eth->MACPOCR = 0;
	mode = ETH_MACTSCR_TSVER2ENA | ETH_MACTSCR_TSIPENA | ETH_MACTSCR_TSENMACADDR;
	if (ops != 0U) {
		mode |= ETH_MACTSCR_TSEVNTENA;
		if (config->p2p != 0U) {
			mode |= BIT(ETH_MACTSCR_SNAPTYPSEL_Pos);
		}
		if (config->transmitter != 0U) {
			mode |= ETH_MACTSCR_TSMSTRENA;
		}
	}
	MODIFY_REG(eth->MACTSCR, TS_MODE_MASK, mode);
	eth->MACSPI0R = sys_get_be32(config->port_id + 6);
	eth->MACSPI1R = sys_get_be32(config->port_id + 2);
	eth->MACSPI2R = sys_get_be16(config->port_id);
	eth->MACLMIR = (uint8_t)config->log_sync_interval;
	if ((ops & ETHERNET_PTP_AUTO_DELAY_RESP) != 0U) {
		eth->MACLMIR |=
			(uint32_t)(config->log_delay_req_interval - config->log_sync_interval)
			<< ETH_MACLMIR_DRSYNCR_Pos;
	} else {
		control |= ETH_MACPOCR_DRRDIS;
	}
	if ((ops & STM32_PTP_AUTO) != 0U) {
		control |= ETH_MACPOCR_PTOEN;
	}
	if ((ops & ETHERNET_PTP_AUTO_SYNC) != 0U) {
		control |= ETH_MACPOCR_ASYNCEN;
	}
	uint32_t generation = data->ptp_config.generation + 1U;

	data->ptp_config = *config;
	data->ptp_config.capabilities = STM32_PTP_CAPS;
	data->ptp_config.generation = generation;
	__DMB();
	eth->MACPOCR = control;
	irq_unlock(key);
restore_rx:
	eth->DMACRCR |= dma_receiver;
	eth->MACCR |= receiver;
out:
	k_mutex_unlock(&data->ptp_lock);
	return ret;
}
