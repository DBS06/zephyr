/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_ETHERNET_PTP_H_
#define ZEPHYR_INCLUDE_NET_ETHERNET_PTP_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

/**
 * @defgroup ethernet_ptp Ethernet PTP Packet Acceleration
 * @ingroup ethernet
 * @{
 */

/** @brief Optional Ethernet PTP packet operations. */
enum ethernet_ptp_operation {
	ETHERNET_PTP_ONE_STEP_SYNC = BIT(0),        /**< Insert Sync origin timestamp. */
	ETHERNET_PTP_ONE_STEP_PDELAY_RESP = BIT(1), /**< Update peer turnaround correction. */
	ETHERNET_PTP_AUTO_SYNC = BIT(2),            /**< Generate periodic Sync. */
	ETHERNET_PTP_AUTO_DELAY_RESP = BIT(3),      /**< Respond to Delay_Req. */
	ETHERNET_PTP_AUTO_PDELAY_RESP = BIT(4),     /**< Respond to Pdelay_Req. */
};

/** @brief PTP packet configuration, independent of clock discipline. */
struct ethernet_ptp_config {
	uint32_t capabilities;    /**< Supported operations, returned by get_config. */
	uint32_t operations;      /**< Enabled operations. Unsupported requests return -ENOTSUP. */
	uint32_t generation;      /**< Driver generation, returned by get_config. */
	uint8_t port_id[10];      /**< Clock identity followed by big-endian port number. */
	uint8_t domain;           /**< IEEE 1588 domain number. */
	uint8_t p2p;              /**< Peer delay when 1, end-to-end delay when 0. */
	uint8_t transmitter;      /**< Time transmitter when 1, receiver when 0. */
	uint8_t sync_flags;       /**< Second flag octet required in generated Sync. */
	int8_t log_sync_interval; /**< Log2 Sync interval in seconds. */
	int8_t log_delay_req_interval; /**< Log2 minimum Delay_Req interval. */
};

/** @brief Packet ancillary flags; TX and RX flags must not be mixed. */
enum net_ptp_packet_flag {
	NET_PTP_PACKET_ONE_STEP = BIT(0),      /**< TX: request one-step processing. */
	NET_PTP_PACKET_INGRESS_VALID = BIT(1), /**< TX: ingress timestamp is supplied. */
	NET_PTP_PACKET_RX_VALID = BIT(2),      /**< RX: ownership metadata is present. */
	NET_PTP_PACKET_RX_RESPONDED = BIT(3),  /**< RX: hardware owns the response. */
};

/**
 * @brief Ancillary data for ZSOCK_SCM_PTP_OFFLOAD on packet sockets.
 *
 * RX_RESPONDED assigns response ownership; it does not acknowledge successful
 * transmission. Generation identifies the configuration used for this packet.
 * Timestamp fields are zero unless INGRESS_VALID is set. All fields use host
 * byte order. Hardware may impose narrower timestamp limits.
 */
struct net_ptp_packet {
	uint64_t ingress_seconds;     /**< Ingress seconds on the interface PHC. */
	uint32_t ingress_nanoseconds; /**< Ingress nanoseconds, less than 1000000000. */
	uint32_t generation;          /**< Configuration generation. */
	uint32_t flags;               /**< Bitmask of net_ptp_packet_flag. */
	uint32_t reserved;            /**< Must be zero. */
};

/** @} */

#endif /* ZEPHYR_INCLUDE_NET_ETHERNET_PTP_H_ */
