/*
 * Copyright (c) 2026 Philipp Steiner <philipp.steiner1987@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt_filter.h>

#include "packet_priority.h"

static NPF_SIZE_MIN(ptp_eth_header, sizeof(struct net_eth_hdr));
static NPF_ETH_TYPE_MATCH(ptp_eth_type, NET_ETH_PTYPE_PTP);
static NPF_PRIORITY(ptp_untagged_priority, NET_PRIORITY_CA, ptp_eth_header, ptp_eth_type);

static NPF_SIZE_MIN(ptp_vlan_header, sizeof(struct net_eth_vlan_hdr));
static NPF_ETH_TYPE_MATCH(ptp_vlan_outer_type, NET_ETH_PTYPE_VLAN);
static NPF_ETH_VLAN_TYPE_MATCH(ptp_vlan_inner_type, NET_ETH_PTYPE_PTP);
static NPF_PRIORITY(ptp_vlan_priority, NET_PRIORITY_CA, ptp_vlan_header, ptp_vlan_outer_type,
		    ptp_vlan_inner_type);

void ptp_rx_packet_priority_init(void)
{
	npf_append_recv_priority_rule(&ptp_untagged_priority);
	npf_append_recv_priority_rule(&ptp_vlan_priority);
}
