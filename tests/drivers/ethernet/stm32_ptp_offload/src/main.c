/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/logging/log.h>
#include "eth_stm32_hal_ptp_offload.h"

LOG_MODULE_REGISTER(eth_stm32_hal, CONFIG_ETHERNET_LOG_LEVEL);

static const uint8_t mac[6] = {2, 3, 4, 5, 6, 7};

static void make_frame(uint8_t frame[68], uint8_t type)
{
	memset(frame, 0, 68);
	memcpy(frame, mac, 6);
	sys_put_be16(0x88f7, frame + 12);
	frame[14] = type;
	frame[15] = 2;
	sys_put_be16(type == 0 || type == 1 ? 44 : 54, frame + 16);
	frame[34] = 0xa5;
}

ZTEST(stm32_ptp_offload, test_sync_context_and_clear)
{
	struct ethernet_ptp_config cfg = {
		.operations = ETHERNET_PTP_ONE_STEP_SYNC,
		.generation = 5,
		.transmitter = 1,
		.port_id = {0xa5},
	};
	struct net_ptp_packet meta = {.flags = NET_PTP_PACKET_ONE_STEP, .generation = 5};
	uint8_t frame[68];
	uint32_t desc[4];

	make_frame(frame, 0);
	zassert_ok(stm32_ptp_context(&cfg, &meta, frame, 60, desc));
	zassert_equal(desc[3], BIT(30) | BIT(27));
	zassert_equal(desc[0] | desc[1] | desc[2], 0);
	meta.flags = 0;
	zassert_ok(stm32_ptp_context(&cfg, &meta, frame, 60, desc));
	zassert_equal(desc[3], BIT(30), "one-step state must not leak to the next packet");
	meta.flags = NET_PTP_PACKET_RX_VALID | NET_PTP_PACKET_RX_RESPONDED;
	meta.generation--;
	zassert_ok(stm32_ptp_context(&cfg, &meta, frame, 60, desc));
	zassert_equal(desc[3], BIT(30), "forwarded RX metadata must not request insertion");
}

ZTEST(stm32_ptp_offload, test_peer_context_and_invalid_ingress)
{
	struct ethernet_ptp_config cfg = {
		.operations = ETHERNET_PTP_ONE_STEP_PDELAY_RESP,
		.generation = 9,
		.p2p = 1,
		.port_id = {0xa5},
	};
	struct net_ptp_packet meta = {
		.flags = NET_PTP_PACKET_ONE_STEP | NET_PTP_PACKET_INGRESS_VALID,
		.generation = 9,
		.ingress_seconds = UINT32_MAX,
		.ingress_nanoseconds = 999999999,
	};
	uint8_t frame[68];
	uint32_t desc[4];

	make_frame(frame, 3);
	zassert_ok(stm32_ptp_context(&cfg, &meta, frame, sizeof(frame), desc));
	zassert_equal(desc[3], BIT(30) | BIT(27) | BIT(26));
	zassert_equal(desc[0], 999999999);
	zassert_equal(desc[1], UINT32_MAX);
	meta.ingress_seconds++;
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, sizeof(frame), desc), -ENOTSUP);
	meta.ingress_seconds = 1;
	meta.ingress_nanoseconds++;
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, sizeof(frame), desc), -ENOTSUP);
	meta.generation--;
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, sizeof(frame), desc), -ESTALE);
}

ZTEST(stm32_ptp_offload, test_response_ownership_is_exchange_specific)
{
	struct ethernet_ptp_config cfg = {.operations = ETHERNET_PTP_AUTO_DELAY_RESP};
	uint8_t frame[68];

	make_frame(frame, 1);
	zassert_true(stm32_ptp_response_owned(&cfg, frame, 60, mac));
	frame[20] = BIT(1);
	zassert_false(stm32_ptp_response_owned(&cfg, frame, 60, mac));
	frame[20] = 0;
	frame[18] = 1;
	zassert_false(stm32_ptp_response_owned(&cfg, frame, 60, mac));
	frame[18] = 0;
	frame[14] |= 0x10;
	zassert_false(stm32_ptp_response_owned(&cfg, frame, 60, mac));
	make_frame(frame, 2);
	zassert_false(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
	cfg.operations = ETHERNET_PTP_AUTO_PDELAY_RESP;
	zassert_true(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
	frame[0] = 0xff;
	zassert_false(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
}

ZTEST(stm32_ptp_offload, test_response_ownership_matches_mac_receive_filter)
{
	static const uint8_t destinations[][6] = {
		{2, 3, 4, 5, 6, 7},
		{1, 0x1b, 0x19, 0, 0, 0},
		{1, 0x80, 0xc2, 0, 0, 0x0e},
	};
	static const uint16_t tags[] = {0, 0xa000, 42};
	struct ethernet_ptp_config cfg = {0};
	uint8_t frame[72];

	for (uint8_t type = 1; type <= 2; type++) {
		cfg.operations = type == 1 ? ETHERNET_PTP_AUTO_DELAY_RESP
					   : ETHERNET_PTP_AUTO_PDELAY_RESP;
		for (size_t dst = 0; dst < ARRAY_SIZE(destinations); dst++) {
			make_frame(frame, type);
			memcpy(frame, destinations[dst], 6);
			zassert_true(stm32_ptp_response_owned(&cfg, frame, 68, mac));

			/* The MAC accepts either multicast address for either request,
			 * including priority-tagged frames delivered to the physical port.
			 */
			memmove(frame + 16, frame + 12, 56);
			sys_put_be16(0x8100, frame + 12);
			for (size_t tag = 0; tag < ARRAY_SIZE(tags); tag++) {
				sys_put_be16(tags[tag], frame + 14);
				zassert_true(stm32_ptp_response_owned(&cfg, frame, sizeof(frame),
							     mac));
			}
		}
	}

	for (size_t len = 0; len < 52; len++) {
		zassert_false(stm32_ptp_response_owned(&cfg, frame, len, mac));
	}
	frame[24] = BIT(1);
	zassert_false(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
	frame[24] = 0;
	frame[22] = 1;
	zassert_false(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
	frame[22] = 0;
	frame[18] |= 0x10;
	zassert_false(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
	frame[18] = 2;
	frame[19] = 1;
	zassert_false(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
	frame[19] = 2;
	memcpy(frame + 38, cfg.port_id, sizeof(cfg.port_id));
	zassert_false(stm32_ptp_response_owned(&cfg, frame, sizeof(frame), mac));
}

ZTEST(stm32_ptp_offload, test_unsupported_frame_never_gets_one_step_context)
{
	struct ethernet_ptp_config cfg = {
		.operations = ETHERNET_PTP_ONE_STEP_SYNC,
		.generation = 1,
		.port_id = {0xa5},
	};
	struct net_ptp_packet meta = {.flags = NET_PTP_PACKET_ONE_STEP, .generation = 1};
	uint8_t frame[68];
	uint32_t desc[4];

	make_frame(frame, 0);
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, 47, desc), -EINVAL);
	frame[20] = BIT(1);
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, 60, desc), -EINVAL);
	make_frame(frame, 0);
	frame[34]++;
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, 60, desc), -EINVAL);
	make_frame(frame, 0);
	sys_put_be16(0x8100, frame + 12);
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, 60, desc), -EINVAL);
	make_frame(frame, 0);
	sys_put_be16(0x0800, frame + 12);
	zassert_equal(stm32_ptp_context(&cfg, &meta, frame, 60, desc), -EINVAL);
}

ZTEST(stm32_ptp_offload, test_hardware_mode_limits)
{
	struct ethernet_ptp_config cfg = {.operations = ETHERNET_PTP_AUTO_SYNC, .transmitter = 1};

	zassert_ok(stm32_ptp_validate(&cfg));
	cfg.log_sync_interval = -1;
	zassert_equal(stm32_ptp_validate(&cfg), -ENOTSUP);
	cfg.log_sync_interval = 0;
	cfg.sync_flags = 8;
	zassert_equal(stm32_ptp_validate(&cfg), -ENOTSUP);
	cfg.sync_flags = 0;
	cfg.p2p = 1;
	zassert_equal(stm32_ptp_validate(&cfg), -ENOTSUP);
	cfg.operations |= ETHERNET_PTP_AUTO_PDELAY_RESP;
	zassert_ok(stm32_ptp_validate(&cfg));
	cfg.operations = ETHERNET_PTP_AUTO_DELAY_RESP;
	zassert_equal(stm32_ptp_validate(&cfg), -ENOTSUP);
	cfg.p2p = 0;
	cfg.log_delay_req_interval = 6;
	zassert_equal(stm32_ptp_validate(&cfg), -ENOTSUP);
	cfg.log_delay_req_interval = 5;
	zassert_ok(stm32_ptp_validate(&cfg));
}

ZTEST_SUITE(stm32_ptp_offload, NULL, NULL, NULL, NULL, NULL);
