/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NXP_ENET_QOS_TEST_REGISTERS_H_
#define NXP_ENET_QOS_TEST_REGISTERS_H_

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>

/* Only the PHC registers used by the real driver are modeled. */
typedef struct {
	volatile uint32_t control[1];
	volatile uint32_t MAC_SYSTEM_TIME_SECONDS;
	volatile uint32_t MAC_SYSTEM_TIME_NANOSECONDS;
	volatile uint32_t MAC_SYSTEM_TIME_SECONDS_UPDATE;
	volatile uint32_t MAC_SYSTEM_TIME_NANOSECONDS_UPDATE;
	volatile uint32_t MAC_SUB_SECOND_INCREMENT;
	volatile uint32_t MAC_TIMESTAMP_ADDEND;
	volatile uint32_t MAC_PPS_CONTROL;
} enet_qos_t;

/* A control-register read completes the preceding hardware update command. */
size_t enet_qos_test_control_index(void);
#define MAC_TIMESTAMP_CONTROL control[enet_qos_test_control_index()]

/* Values match MCXN's PERI_ENET.h; this is not a model of PPS electrical timing. */
#define ENET_MAC_TIMESTAMP_CONTROL_TSENA_MASK BIT(0)
#define ENET_MAC_TIMESTAMP_CONTROL_TSCFUPDT_MASK BIT(1)
#define ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK BIT(2)
#define ENET_MAC_TIMESTAMP_CONTROL_TSUPDT_MASK BIT(3)
#define ENET_MAC_TIMESTAMP_CONTROL_TSADDREG_MASK BIT(5)
#define ENET_MAC_TIMESTAMP_CONTROL_TSENALL_MASK BIT(8)
#define ENET_MAC_TIMESTAMP_CONTROL_TSCTRLSSR(x) ((x) << 9)
#define ENET_MAC_TIMESTAMP_CONTROL_TSVER2ENA_MASK BIT(10)
#define ENET_MAC_TIMESTAMP_CONTROL_TSIPENA_MASK BIT(11)
#define ENET_MAC_TIMESTAMP_CONTROL_TSIPV6ENA_MASK BIT(12)
#define ENET_MAC_TIMESTAMP_CONTROL_TSIPV4ENA_MASK BIT(13)
#define ENET_MAC_TIMESTAMP_CONTROL_TSEVNTENA_MASK BIT(14)
#define ENET_MAC_TIMESTAMP_CONTROL_SNAPTYPSEL_MASK (BIT(16) | BIT(17))
#define ENET_MAC_SYSTEM_TIME_NANOSECONDS_TSSS_MASK GENMASK(30, 0)
#define ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK GENMASK(30, 0)
#define ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB_MASK BIT(31)
#define ENET_MAC_SUB_SECOND_INCREMENT_SNSINC(x) (((x) & 0xffU) << 16)
#define ENET_MAC_PPS_CONTROL_PPSCTRL_PPSCMD_MASK GENMASK(3, 0)

struct nxp_enet_qos_config {
	enet_qos_t *base;
};
#define ENET_QOS_MODULE_CFG(dev) ((struct nxp_enet_qos_config *)(dev)->config)

#endif /* NXP_ENET_QOS_TEST_REGISTERS_H_ */
