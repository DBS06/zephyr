/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define net_if_add_tx_timestamp test_add_tx_timestamp
#define net_recv_data           test_recv_data
#include "mock_hal.h"
#include "eth_stm32_hal_ptp_offload.c"
#include <zephyr/ztest.h>

static struct eth_stm32_dma_buf buffers;
static ETH_DMADescTypeDef descriptors[ETH_TX_DESC_CNT];
static ETH_TypeDef registers;
static struct eth_stm32_hal_dev_data data;
static const struct eth_stm32_hal_dev_cfg config = {.dma_buf = &buffers};
static const struct device dev = {.data = &data, .config = &config};
static struct net_pkt *pending_rx;
static struct net_ptp_packet received_meta;
static int timestamp_count;
static bool complete_on_publish;
static bool fault_during_drain;
static uint32_t completion_status;
static int dropped_rx_pending;
static bool partial_rx_pending;

void test_add_tx_timestamp(struct net_pkt *pkt)
{
	zassert_equal(pkt->timestamp.second, 7);
	zassert_equal(pkt->timestamp.nanosecond, 42);
	timestamp_count++;
}

int test_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	ARG_UNUSED(iface);
	received_meta = pkt->ptp;
	net_pkt_unref(pkt);
	return 0;
}

struct net_pkt *eth_stm32_rx_locked(const struct device *device)
{
	struct net_pkt *pkt = pending_rx;

	if (dropped_rx_pending > 0) {
		dropped_rx_pending--;
		return NULL;
	}
	pending_rx = NULL;
	if (fault_during_drain) {
		eth_stm32_ptp_offload_fault(&data);
	}
	if (pkt != NULL) {
		eth_stm32_ptp_offload_rx(device, pkt);
	}
	return pkt;
}

bool eth_stm32_rx_pending(const struct device *device)
{
	ARG_UNUSED(device);
	return dropped_rx_pending != 0 || partial_rx_pending;
}

static void complete_tx(void)
{
	for (size_t i = 0; i < ETH_TX_DESC_CNT; i++) {
		if (data.ptp_tx_pkt[i] != NULL) {
			descriptors[i].DESC0 = 42;
			descriptors[i].DESC1 = 7;
			descriptors[i].DESC3 = completion_status;
		} else {
			descriptors[i].DESC3 &= ~ETH_DMATXNDESCRF_OWN;
		}
	}
}

static void test_dma_barrier(void)
{
	if (complete_on_publish) {
		complete_tx();
	}
}

static struct net_pkt *packet(bool one_step)
{
	uint8_t frame[68] = {0};
	struct net_pkt *pkt =
		net_pkt_alloc_with_buffer(NULL, sizeof(frame), NET_AF_UNSPEC, 0, K_NO_WAIT);

	zassert_not_null(pkt);
	sys_put_be16(0x88f7, frame + 12);
	frame[15] = 2;
	sys_put_be16(44, frame + 16);
	zassert_ok(net_pkt_write(pkt, frame, sizeof(frame)));
	net_pkt_cursor_init(pkt);
	if (one_step) {
		pkt->ptp.flags = NET_PTP_PACKET_ONE_STEP;
		pkt->ptp.generation = data.ptp_config.generation;
	}
	return pkt;
}

static void driver_before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&data, 0, sizeof(data));
	memset(&registers, 0, sizeof(registers));
	memset(descriptors, 0, sizeof(descriptors));
	data.heth.Instance = &registers;
	data.heth.Init.TxDesc = descriptors;
	data.heth.gState = HAL_ETH_STATE_STARTED;
	eth_stm32_ptp_offload_init(&dev);
	data.ptp_stopping = false;
	data.ptp_config.operations = ETHERNET_PTP_ONE_STEP_SYNC;
	k_sem_init(&data.tx_int_sem, 0, 1);
	/* Keep the system worker out of deterministic descriptor tests. */
	k_mutex_lock(&data.ptp_lock, K_FOREVER);
	pending_rx = NULL;
	memset(&received_meta, 0, sizeof(received_meta));
	complete_on_publish = !IS_ENABLED(CONFIG_ETH_STM32_HAL_TX_ASYNC);
	fault_during_drain = false;
	completion_status = 0;
	timestamp_count = 0;
	dropped_rx_pending = 0;
	partial_rx_pending = false;
}

static void driver_after(void *fixture)
{
	struct k_work_sync sync;

	ARG_UNUSED(fixture);
	zassert_ok(eth_stm32_ptp_offload_stop(&dev));
	k_work_cancel_delayable_sync(&data.ptp_tx_work, &sync);
	k_mutex_unlock(&data.ptp_lock);
}

ZTEST(stm32_ptp_driver, test_wrap_completion_and_context_clear)
{
	for (size_t n = 0; n < ETH_TX_DESC_CNT * 2; n++) {
		bool one_step = n % 2 == 0;
		struct net_pkt *pkt = packet(one_step);
		uint16_t first = data.ptp_tx_head;
		uint16_t last = next_desc(first);

		net_pkt_set_tx_timestamping(pkt, true);
		completion_status = one_step ? 0 : ETH_DMATXNDESCWBF_TTSS;
		zassert_ok(eth_stm32_ptp_offload_tx(&dev, pkt));
		zassert_equal(descriptors[first].DESC3 & STM32_PTP_OSTC,
			      one_step ? STM32_PTP_OSTC : 0);
		zassert_equal(descriptors[last].DESC2 & ETH_DMATXNDESCRF_TTSE,
			      one_step ? 0 : ETH_DMATXNDESCRF_TTSE);
		complete_tx();
		reap_tx(&data);
		zassert_equal(data.ptp_tx_used, 0);
		zassert_is_null(data.ptp_tx_pkt[last]);
		net_pkt_unref(pkt);
	}
	zassert_equal(timestamp_count, ETH_TX_DESC_CNT);
}

ZTEST(stm32_ptp_driver, test_exhaustion_timeout_retains_owned_buffers)
{
	struct net_pkt *pkt = packet(false);
	int result;

	complete_on_publish = false;
	net_pkt_set_tx_timestamping(pkt, true);
	result = eth_stm32_ptp_offload_tx(&dev, pkt);
	if (IS_ENABLED(CONFIG_ETH_STM32_HAL_TX_ASYNC)) {
		zassert_ok(result);
		net_pkt_cursor_init(pkt);
		zassert_ok(eth_stm32_ptp_offload_tx(&dev, pkt));
		net_pkt_cursor_init(pkt);
		zassert_equal(eth_stm32_ptp_offload_tx(&dev, pkt), -ETIMEDOUT);
	} else {
		zassert_equal(result, -ETIMEDOUT);
	}
	zassert_not_equal(data.ptp_tx_used, 0);
	zassert_not_null(data.ptp_tx_pkt[1]);
	zassert_equal(registers.MACPOCR, 0);
	completion_status = ETH_DMATXNDESCWBF_TTSS;
	complete_tx();
	reap_tx(&data);
	zassert_equal(timestamp_count, 0, "abandoned exchanges must not get late callbacks");
	zassert_equal(data.ptp_tx_used, 0);
	zassert_is_null(data.ptp_tx_pkt[1]);
	/* A fault disables acceleration, not subsequent ordinary transmission. */
	complete_on_publish = true;
	net_pkt_cursor_init(pkt);
	zassert_ok(eth_stm32_ptp_offload_tx(&dev, pkt));
	reap_tx(&data);
	net_pkt_unref(pkt);
}

ZTEST(stm32_ptp_driver, test_dma_error_disables_offload_without_timestamp_callback)
{
	struct net_pkt *pkt = packet(false);
	struct ethernet_config effective;
	int result;

	completion_status = ETH_DMATXNDESCWBF_ES;
	net_pkt_set_tx_timestamping(pkt, true);
	result = eth_stm32_ptp_offload_tx(&dev, pkt);
	zassert_equal(result, IS_ENABLED(CONFIG_ETH_STM32_HAL_TX_ASYNC) ? 0 : -EIO);
	complete_tx();
	reap_tx(&data);
	zassert_equal(timestamp_count, 0);
	zassert_true(atomic_get(&data.ptp_fault));
	zassert_ok(eth_stm32_ptp_offload_get(&dev, NULL, ETHERNET_CONFIG_TYPE_PTP, &effective));
	zassert_equal(effective.ptp.capabilities, 0);
	pkt->ptp.flags = NET_PTP_PACKET_ONE_STEP;
	net_pkt_cursor_init(pkt);
	zassert_equal(eth_stm32_ptp_offload_tx(&dev, pkt), -EIO);
	net_pkt_unref(pkt);
}

ZTEST(stm32_ptp_driver, test_configuration_identity_and_clock_preservation)
{
	struct ethernet_ptp_config requested = {
		.operations = ETHERNET_PTP_AUTO_SYNC | ETHERNET_PTP_AUTO_DELAY_RESP,
		.port_id = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
		.domain = 17,
		.transmitter = 1,
		.log_sync_interval = 1,
		.log_delay_req_interval = 3,
	};
	/* Clock enable, fine adjustment, digital rollover and RX-all timestamping. */
	uint32_t clock_bits = BIT(0) | BIT(1) | BIT(8) | BIT(9);
	struct ethernet_config effective;

	/* Startup/link-down defers negotiation instead of failing port startup. */
	data.ptp_stopping = true;
	zassert_ok(eth_stm32_ptp_offload_get(&dev, NULL, ETHERNET_CONFIG_TYPE_PTP, &effective));
	zassert_equal(effective.ptp.capabilities, 0);
	data.ptp_stopping = false;
	zassert_ok(eth_stm32_ptp_offload_get(&dev, NULL, ETHERNET_CONFIG_TYPE_PTP, &effective));
	zassert_equal(effective.ptp.capabilities, STM32_PTP_CAPS);

	registers.MACTSCR = clock_bits;
	registers.MACCR = ETH_MACCR_RE;
	registers.DMACRCR = ETH_DMACRCR_SR;
	zassert_ok(eth_stm32_ptp_offload_configure(&dev, &requested));
	zassert_equal(registers.MACSPI0R, 0x06070809);
	zassert_equal(registers.MACSPI1R, 0x02030405);
	zassert_equal(registers.MACSPI2R, 0x0001);
	zassert_equal(registers.MACLMIR, 0x0201);
	zassert_equal(registers.MACPOCR, (17U << 8) | BIT(1) | BIT(0));
	zassert_equal(registers.MACTSCR & clock_bits, clock_bits);
	zassert_equal(registers.MACCR, ETH_MACCR_RE);
	zassert_equal(registers.DMACRCR, ETH_DMACRCR_SR);
	zassert_equal(data.ptp_config.generation, 2);

	requested.operations = ETHERNET_PTP_AUTO_PDELAY_RESP;
	requested.transmitter = 0;
	requested.p2p = 1;
	zassert_ok(eth_stm32_ptp_offload_configure(&dev, &requested));
	zassert_equal(registers.MACPOCR, (17U << 8) | BIT(6) | BIT(0));
	zassert_equal(registers.MACTSCR & ETH_MACTSCR_SNAPTYPSEL, BIT(16));
	zassert_equal(registers.MACTSCR & ETH_MACTSCR_TSMSTRENA, 0);
}

ZTEST(stm32_ptp_driver, test_queued_rx_retains_old_ownership_and_tx_rejects_old_epoch)
{
	struct ethernet_ptp_config disabled = {0};
	struct net_pkt *stale = packet(true);
	uint8_t type = 1;
	uint8_t source = 1;

	data.ptp_config.operations |= ETHERNET_PTP_AUTO_DELAY_RESP;
	pending_rx = packet(false);
	net_pkt_set_overwrite(pending_rx, true);
	zassert_ok(net_pkt_skip(pending_rx, 14));
	zassert_ok(net_pkt_write(pending_rx, &type, 1));
	zassert_ok(net_pkt_skip(pending_rx, 19));
	zassert_ok(net_pkt_write(pending_rx, &source, 1));
	zassert_ok(eth_stm32_ptp_offload_configure(&dev, &disabled));
	zassert_equal(received_meta.generation, 1);
	zassert_equal(received_meta.flags, NET_PTP_PACKET_RX_VALID | NET_PTP_PACKET_RX_RESPONDED);
	zassert_equal(data.ptp_config.generation, 2);
	zassert_equal(eth_stm32_ptp_offload_tx(&dev, stale), -ESTALE);
	zassert_equal(data.ptp_tx_used, 0);
	net_pkt_unref(stale);
}

ZTEST(stm32_ptp_driver, test_priority_tagged_rx_marks_hardware_response)
{
	uint8_t frame[72] = {1, 0x80, 0xc2, 0, 0, 0x0e};
	struct net_pkt *pkt =
		net_pkt_alloc_with_buffer(NULL, sizeof(frame), NET_AF_UNSPEC, 0, K_NO_WAIT);

	zassert_not_null(pkt);
	data.ptp_config.operations = ETHERNET_PTP_AUTO_DELAY_RESP;
	data.ptp_config.domain = 17;
	sys_put_be16(0x8100, frame + 12);
	sys_put_be16(0xa000, frame + 14); /* PCP 5, VID 0: delivered to the physical port. */
	sys_put_be16(0x88f7, frame + 16);
	frame[18] = 1; /* Delay_Req to the peer-delay multicast address. */
	frame[19] = 2;
	sys_put_be16(44, frame + 20);
	frame[22] = 17;
	frame[38] = 1;
	zassert_ok(net_pkt_write(pkt, frame, sizeof(frame)));
	eth_stm32_ptp_offload_rx(&dev, pkt);
	zassert_equal(pkt->ptp.flags, NET_PTP_PACKET_RX_VALID | NET_PTP_PACKET_RX_RESPONDED);
	zassert_equal(pkt->ptp.generation, data.ptp_config.generation);
	zassert_ok(net_pkt_read(pkt, frame, sizeof(frame)), "RX must restore the packet cursor");
	zassert_equal(frame[18], 1);

	data.ptp_config.operations = 0;
	eth_stm32_ptp_offload_rx(&dev, pkt);
	zassert_equal(pkt->ptp.flags, NET_PTP_PACKET_RX_VALID);
	net_pkt_unref(pkt);
}

ZTEST(stm32_ptp_driver, test_fault_during_reconfiguration_cannot_reenable_generator)
{
	struct ethernet_ptp_config requested = {
		.operations = ETHERNET_PTP_AUTO_SYNC,
		.transmitter = 1,
	};

	fault_during_drain = true;
	zassert_equal(eth_stm32_ptp_offload_configure(&dev, &requested), -EIO);
	zassert_equal(registers.MACPOCR, 0);
	zassert_equal(data.ptp_config.generation, 1);
}

ZTEST(stm32_ptp_driver, test_reconfiguration_timeout_does_not_change_generation)
{
	struct ethernet_ptp_config disabled = {0};

	registers.MACDR = ETH_MACDR_RPESTS;
	registers.MACPOCR = ETH_MACPOCR_PTOEN;
	zassert_equal(eth_stm32_ptp_offload_configure(&dev, &disabled), -ETIMEDOUT);
	zassert_equal(registers.MACPOCR, 0);
	zassert_equal(data.ptp_config.generation, 1);
}

ZTEST(stm32_ptp_driver, test_hal_error_cannot_relabel_undrainable_rx_generation)
{
	struct ethernet_ptp_config disabled = {0};

	data.ptp_config.operations = ETHERNET_PTP_AUTO_DELAY_RESP;
	data.heth.gState = 0;
	registers.MACPOCR = ETH_MACPOCR_PTOEN;
	zassert_equal(eth_stm32_ptp_offload_configure(&dev, &disabled), -EIO);
	zassert_equal(registers.MACPOCR, 0);
	zassert_equal(data.ptp_config.generation, 1);
	zassert_equal(data.ptp_config.operations, ETHERNET_PTP_AUTO_DELAY_RESP);
}

ZTEST(stm32_ptp_driver, test_rx_allocation_failures_do_not_end_drain_early)
{
	struct ethernet_ptp_config disabled = {0};

	dropped_rx_pending = 3;
	zassert_ok(eth_stm32_ptp_offload_configure(&dev, &disabled));
	zassert_equal(dropped_rx_pending, 0);
	partial_rx_pending = true;
	zassert_equal(eth_stm32_ptp_offload_configure(&dev, &disabled), -ETIMEDOUT);
	zassert_equal(data.ptp_config.generation, 2);
}

ZTEST_SUITE(stm32_ptp_driver, NULL, NULL, driver_before, driver_after, NULL);
