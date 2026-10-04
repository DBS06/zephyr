/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_STM32_PTP_OFFLOAD_H_
#define ZEPHYR_DRIVERS_ETHERNET_STM32_PTP_OFFLOAD_H_

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/net/ethernet_ptp.h>
#include <zephyr/sys/byteorder.h>

#define STM32_PTP_CAPS                                                                             \
	(ETHERNET_PTP_ONE_STEP_SYNC | ETHERNET_PTP_ONE_STEP_PDELAY_RESP | ETHERNET_PTP_AUTO_SYNC | \
	 ETHERNET_PTP_AUTO_DELAY_RESP | ETHERNET_PTP_AUTO_PDELAY_RESP)
#define STM32_PTP_AUTO                                                                             \
	(ETHERNET_PTP_AUTO_SYNC | ETHERNET_PTP_AUTO_DELAY_RESP | ETHERNET_PTP_AUTO_PDELAY_RESP)

/* RM0481 Table 684. The text on p. 2741 incorrectly calls OSTC bit 20. */
#define STM32_PTP_CONTEXT BIT(30)
#define STM32_PTP_OSTC    BIT(27)
#define STM32_PTP_TCMSSV  BIT(26)

static inline int stm32_ptp_validate(const struct ethernet_ptp_config *cfg)
{
	uint32_t ops = cfg->operations;
	int ratio = cfg->log_delay_req_interval - cfg->log_sync_interval;

	if ((ops & ~STM32_PTP_CAPS) != 0U || cfg->p2p > 1U || cfg->transmitter > 1U) {
		return -ENOTSUP;
	}
	if (((ops & (ETHERNET_PTP_ONE_STEP_SYNC | ETHERNET_PTP_AUTO_SYNC |
		     ETHERNET_PTP_AUTO_DELAY_RESP)) != 0U &&
	     cfg->transmitter == 0U) ||
	    ((ops & (ETHERNET_PTP_ONE_STEP_PDELAY_RESP | ETHERNET_PTP_AUTO_PDELAY_RESP)) != 0U &&
	     cfg->p2p == 0U) ||
	    ((ops & ETHERNET_PTP_AUTO_DELAY_RESP) != 0U && cfg->p2p != 0U)) {
		return -ENOTSUP;
	}
	if (((ops & ETHERNET_PTP_AUTO_SYNC) != 0U &&
	     (cfg->sync_flags != 0U || !IN_RANGE(cfg->log_sync_interval, 0, 15))) ||
	    ((ops & ETHERNET_PTP_AUTO_DELAY_RESP) != 0U &&
	     (!IN_RANGE(cfg->log_sync_interval, -15, 15) || !IN_RANGE(ratio, 0, 5)))) {
		return -ENOTSUP;
	}
	/* In P2P modes PTOEN also enables the peer responder. It cannot be
	 * independently disabled while automatic Sync remains enabled.
	 */
	if (cfg->p2p != 0U && (ops & ETHERNET_PTP_AUTO_SYNC) != 0U &&
	    (ops & ETHERNET_PTP_AUTO_PDELAY_RESP) == 0U) {
		return -ENOTSUP;
	}
	return 0;
}

/* Return a type only for untagged, default-profile PTPv2 Ethernet frames. */
static inline int stm32_ptp_frame_type(const uint8_t *frame, size_t len, uint8_t domain)
{
	if (len < 48U || sys_get_be16(frame + 12) != 0x88f7U || (frame[14] & 0xf0U) != 0U ||
	    (frame[15] & 0xfU) != 2U || frame[18] != domain || sys_get_be16(frame + 16) < 34U ||
	    sys_get_be16(frame + 16) > len - 14U) {
		return -1;
	}
	return frame[14] & 0xfU;
}

static inline bool stm32_ptp_response_owned(const struct ethernet_ptp_config *cfg,
					    const uint8_t *frame, size_t len, const uint8_t *mac)
{
	static const uint8_t event_group[6] = {1, 0x1b, 0x19, 0, 0, 0};
	static const uint8_t peer_group[6] = {1, 0x80, 0xc2, 0, 0, 0x0e};
	size_t offset = 14U;
	const uint8_t *ptp;
	uint16_t protocol;
	uint8_t type;

	/* Match the RX filter, not the narrower one-step TX eligibility rules.
	 * RM0481 Table 654 permits either multicast DA for every message type
	 * and offsets tagged headers by four bytes. The driver leaves S-VLAN
	 * and double-tag processing disabled, so only a single C-tag is parsed.
	 */
	if (len < offset) {
		return false;
	}
	protocol = sys_get_be16(frame + 12);
	if (protocol == 0x8100U) {
		offset += 4U;
		if (len < offset) {
			return false;
		}
		protocol = sys_get_be16(frame + offset - 2U);
	}
	if (protocol != 0x88f7U || len < offset + 34U ||
	    (memcmp(frame, event_group, 6) != 0 && memcmp(frame, peer_group, 6) != 0 &&
	     memcmp(frame, mac, 6) != 0)) {
		return false;
	}
	ptp = frame + offset;
	if ((ptp[0] & 0xf0U) != 0U || (ptp[1] & 0xfU) != 2U || ptp[4] != cfg->domain ||
	    (ptp[6] & BIT(1)) != 0U || memcmp(ptp + 20, cfg->port_id, 10) == 0) {
		return false;
	}
	type = ptp[0] & 0xfU;
	return (type == 1 && (cfg->operations & ETHERNET_PTP_AUTO_DELAY_RESP) != 0U) ||
	       (type == 2 && (cfg->operations & ETHERNET_PTP_AUTO_PDELAY_RESP) != 0U);
}

static inline int stm32_ptp_context(const struct ethernet_ptp_config *cfg,
				    const struct net_ptp_packet *meta, const uint8_t *frame,
				    size_t len, uint32_t desc[4])
{
	int type;

	memset(desc, 0, 4 * sizeof(*desc));
	desc[3] = STM32_PTP_CONTEXT;
	/* RX ownership can survive cloning/forwarding; it is not a TX request. */
	if ((meta->flags & ~(NET_PTP_PACKET_RX_VALID | NET_PTP_PACKET_RX_RESPONDED)) == 0U) {
		return 0;
	}
	if (meta->generation != cfg->generation) {
		return -ESTALE;
	}
	type = stm32_ptp_frame_type(frame, len, cfg->domain);
	if (type < 0 || (meta->flags & NET_PTP_PACKET_ONE_STEP) == 0U ||
	    (meta->flags & ~(NET_PTP_PACKET_ONE_STEP | NET_PTP_PACKET_INGRESS_VALID)) != 0U ||
	    meta->reserved != 0U || (frame[20] & BIT(1)) != 0U ||
	    memcmp(frame + 34, cfg->port_id, 10) != 0) {
		return -EINVAL;
	}
	if (type == 0 && len >= 58U && sys_get_be16(frame + 16) >= 44U &&
	    (cfg->operations & ETHERNET_PTP_ONE_STEP_SYNC) != 0U &&
	    meta->flags == NET_PTP_PACKET_ONE_STEP) {
		desc[3] |= STM32_PTP_OSTC;
		return 0;
	}
	if (type != 3 || len < 68U || sys_get_be16(frame + 16) < 54U ||
	    (cfg->operations & ETHERNET_PTP_ONE_STEP_PDELAY_RESP) == 0U ||
	    (meta->flags & NET_PTP_PACKET_INGRESS_VALID) == 0U ||
	    meta->ingress_seconds > UINT32_MAX || meta->ingress_nanoseconds >= 1000000000U) {
		return -ENOTSUP;
	}
	desc[0] = meta->ingress_nanoseconds;
	desc[1] = meta->ingress_seconds;
	desc[3] |= STM32_PTP_OSTC | STM32_PTP_TCMSSV;
	return 0;
}

#endif /* ZEPHYR_DRIVERS_ETHERNET_STM32_PTP_OFFLOAD_H_ */
