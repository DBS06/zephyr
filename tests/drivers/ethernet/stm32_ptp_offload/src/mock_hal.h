/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef STM32_PTP_MOCK_HAL_H_
#define STM32_PTP_MOCK_HAL_H_

/* Replace only the MMIO/HAL boundary, compiling the production driver below it. */
#define ZEPHYR_DRIVERS_ETHERNET_ETH_STM32_HAL_PRIV_H_

#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>

#define ETH_TX_DESC_CNT            5
#define ETH_STM32_TX_BUF_SIZE      1536
#define HAL_ETH_STATE_STARTED      1
#define ETH_DMATXCDESC_OSTC        BIT(27)
#define ETH_DMATXCDESC_CDE         BIT(23)
#define ETH_DMATXNDESCRF_OWN       BIT(31)
#define ETH_DMATXNDESCRF_IOC       BIT(31)
#define ETH_DMATXNDESCRF_TTSE      BIT(30)
#define ETH_DMATXNDESCRF_FD        BIT(29)
#define ETH_DMATXNDESCRF_LD        BIT(28)
#define ETH_DMATXNDESCWBF_ES       BIT(15)
#define ETH_DMATXNDESCWBF_TTSS     BIT(17)
#define ETH_DMACSR_TBU             BIT(2)
#define ETH_MACCR_RE               BIT(0)
#define ETH_DMACRCR_SR             BIT(0)
#define ETH_DMADSR_RPS             GENMASK(11, 8)
#define ETH_DMADSR_TPS             GENMASK(15, 12)
#define ETH_MACDR_RPESTS           BIT(0)
#define ETH_MACDR_TPESTS           BIT(16)
#define ETH_MTLRQDR_PRXQ           GENMASK(29, 16)
#define ETH_MTLRQDR_RXQSTS         GENMASK(5, 4)
#define ETH_MTLTQDR_TXQSTS         BIT(4)
#define ETH_MACTSCR_TSENMACADDR    BIT(18)
#define ETH_MACTSCR_SNAPTYPSEL_Pos 16
#define ETH_MACTSCR_SNAPTYPSEL     GENMASK(17, 16)
#define ETH_MACTSCR_TSMSTRENA      BIT(15)
#define ETH_MACTSCR_TSEVNTENA      BIT(14)
#define ETH_MACTSCR_TSIPENA        BIT(11)
#define ETH_MACTSCR_TSVER2ENA      BIT(10)
#define ETH_MACPOCR_DN_Pos         8
#define ETH_MACPOCR_DRRDIS         BIT(6)
#define ETH_MACPOCR_ASYNCEN        BIT(1)
#define ETH_MACPOCR_PTOEN          BIT(0)
#define ETH_MACLMIR_DRSYNCR_Pos    8

#define MODIFY_REG(reg, mask, value) ((reg) = ((reg) & ~(mask)) | (value))
#define __DMB()                      compiler_barrier()
#define __DSB()                      test_dma_barrier()

typedef struct {
	volatile uint32_t DESC0, DESC1, DESC2, DESC3;
} ETH_DMADescTypeDef;

typedef struct {
	uint32_t MACPOCR, MACCR, DMACRCR, DMADSR, MACDR, MTLRQDR, MTLTQDR;
	uint32_t MACTSCR, MACSPI0R, MACSPI1R, MACSPI2R, MACLMIR;
	uint32_t DMACSR, DMACTDLAR, DMACTDTPR;
} ETH_TypeDef;

struct eth_stm32_dma_buf {
	uint8_t tx_buf[ETH_TX_DESC_CNT][ETH_STM32_TX_BUF_SIZE];
};

struct eth_stm32_hal_dev_cfg {
	struct eth_stm32_dma_buf *dma_buf;
};

struct eth_stm32_hal_dev_data {
	struct {
		ETH_TypeDef *Instance;
		struct {
			ETH_DMADescTypeDef *TxDesc;
		} Init;
		int gState;
	} heth;
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_mutex ptp_lock;
	struct k_sem tx_int_sem;
	struct k_work_delayable ptp_tx_work;
	struct ethernet_ptp_config ptp_config;
	struct net_pkt *ptp_tx_pkt[ETH_TX_DESC_CNT];
	uint16_t ptp_tx_head, ptp_tx_tail, ptp_tx_used;
	atomic_t ptp_fault;
	bool ptp_stopping;
	int64_t ptp_tx_started[ETH_TX_DESC_CNT];
	bool ptp_tx_abandoned[ETH_TX_DESC_CNT];
};

static void test_dma_barrier(void);
struct net_pkt *eth_stm32_rx_locked(const struct device *dev);
bool eth_stm32_rx_pending(const struct device *dev);

#endif /* STM32_PTP_MOCK_HAL_H_ */
