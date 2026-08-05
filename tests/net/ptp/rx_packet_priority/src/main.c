/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "packet_priority.h"

static struct net_pkt *build_test_pkt(const uint8_t *frame, size_t frame_len,
				      enum net_priority priority)
{
	struct net_pkt *pkt;
	int ret;

	pkt = net_pkt_rx_alloc_with_buffer(NULL, frame_len, NET_AF_UNSPEC, 0, K_NO_WAIT);
	zassert_not_null(pkt, "Failed to allocate test packet");

	ret = net_pkt_write(pkt, frame, frame_len);
	zassert_ok(ret, "Failed to write test packet");
	net_pkt_set_priority(pkt, priority);

	return pkt;
}

static void expect_priority(const uint8_t *frame, size_t frame_len, enum net_priority initial,
			    enum net_priority expected)
{
	struct net_pkt *pkt = build_test_pkt(frame, frame_len, initial);

	zassert_true(net_pkt_filter_recv_ok(pkt), "Priority rule changed packet verdict");
	zassert_equal(net_pkt_priority(pkt), expected, "Unexpected packet priority");
	net_pkt_unref(pkt);
}

static void *rx_packet_priority_setup(void)
{
	ptp_rx_packet_priority_init();

	return NULL;
}

ZTEST(ptp_rx_packet_priority, test_frame_classification)
{
	uint8_t untagged_ptp[sizeof(struct net_eth_hdr)] = {0};
	uint8_t untagged_ip[sizeof(struct net_eth_hdr)] = {0};
	uint8_t vlan_ptp[sizeof(struct net_eth_vlan_hdr)] = {0};
	uint8_t vlan_ip[sizeof(struct net_eth_vlan_hdr)] = {0};
	uint8_t double_vlan_ptp[sizeof(struct net_eth_vlan_hdr) + 4] = {0};

	sys_put_be16(NET_ETH_PTYPE_PTP, &untagged_ptp[12]);
	sys_put_be16(NET_ETH_PTYPE_IP, &untagged_ip[12]);

	sys_put_be16(NET_ETH_PTYPE_VLAN, &vlan_ptp[12]);
	sys_put_be16(1, &vlan_ptp[14]);
	sys_put_be16(NET_ETH_PTYPE_PTP, &vlan_ptp[16]);

	sys_put_be16(NET_ETH_PTYPE_VLAN, &vlan_ip[12]);
	sys_put_be16(1, &vlan_ip[14]);
	sys_put_be16(NET_ETH_PTYPE_IP, &vlan_ip[16]);

	sys_put_be16(NET_ETH_PTYPE_VLAN, &double_vlan_ptp[12]);
	sys_put_be16(1, &double_vlan_ptp[14]);
	sys_put_be16(NET_ETH_PTYPE_VLAN, &double_vlan_ptp[16]);
	sys_put_be16(2, &double_vlan_ptp[18]);
	sys_put_be16(NET_ETH_PTYPE_PTP, &double_vlan_ptp[20]);

	expect_priority(untagged_ptp, sizeof(untagged_ptp), NET_PRIORITY_BE, NET_PRIORITY_CA);
	expect_priority(vlan_ptp, sizeof(vlan_ptp), NET_PRIORITY_BE, NET_PRIORITY_CA);
	expect_priority(untagged_ip, sizeof(untagged_ip), NET_PRIORITY_VI, NET_PRIORITY_VI);
	expect_priority(vlan_ip, sizeof(vlan_ip), NET_PRIORITY_VI, NET_PRIORITY_VI);
	expect_priority(double_vlan_ptp, sizeof(double_vlan_ptp), NET_PRIORITY_VI, NET_PRIORITY_VI);
}

ZTEST_SUITE(ptp_rx_packet_priority, NULL, rx_packet_priority_setup, NULL, NULL, NULL);
