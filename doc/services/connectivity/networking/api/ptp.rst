.. _ptp_interface:

Precision Time Protocol (PTP)
#############################

.. contents::
    :local:
    :depth: 2

Overview
********

PTP is a network protocol implemented in the application layer, used to synchronize
clocks in a computer network. It's accurate up to less than a microsecond.
The stack supports the protocol and procedures as defined in the `IEEE 1588-2019 standard`_
(IEEE Standard for a Precision Clock Synchronization Protocol
for Networked Measurement and Control Systems). It has multiple profiles,
and can be implemented on top of L2 (Ethernet) or L3 (UDP/IPv4 or UDP/IPv6).
Its accuracy is achieved by using hardware timestamping of the protocol packets.

Zephyr's implementation of PTP stack consist following items:

* PTP stack thread that handles incoming messages and events
* Integration with ptp_clock driver
* PTP stack initialization executed during system init

The implementation automatically creates PTP Ports (each PTP Port corresponds to unique interface).

Supported features
******************

Implementation of the stack doesn't support all features specified in the standard.
In the table below all supported features are listed.

.. csv-table:: Supported features
   :header: Feature, Supported
   :widths: 50,10

    Ordinary Clock, yes
    Boundary Clock, yes
    Transparent Clock,
    Management Node,
    End to end delay mechanism, yes
    Peer to peer delay mechanism, yes (one-step and two-step peers)
    Multicast operation mode, yes
    Hybrid operation mode, yes
    Unicast operation mode,
    Non-volatile storage,
    UDP IPv4 transport protocol, yes
    UDP IPv6 transport protocol, yes
    IEEE 802.3 (Ethernet) transport protocol, yes
    Hardware timestamping, yes
    Software timestamping,
    TIME_RECEIVER_ONLY PTP Instance, yes
    TIME_TRANSMITTER_ONLY PTP Instance,

Network transmission modes
**************************

The network transmission mode is selected with the ``PTP_NETWORK_MODE``
Kconfig choice:

* Multicast mode (:kconfig:option:`CONFIG_PTP_NETWORK_MODE_MULTICAST`, the
  default) is the standard PTP mode whereby all PTP messages are sent to the
  default multicast addresses. The implication is that every node receives the
  ``Delay_Req`` and ``Delay_Resp`` message pairs of all other nodes, thereby
  increasing the amount of traffic on the network. This is usually not a
  problem on smaller networks employing only a few timeReceivers.

* Hybrid mode (:kconfig:option:`CONFIG_PTP_NETWORK_MODE_HYBRID`) still sends
  ``Announce``, ``Sync`` and ``Follow_Up`` messages to the default multicast
  addresses, but a timeReceiver sends its ``Delay_Req`` messages as unicast
  directly to the protocol address (IP address, or MAC address for the
  IEEE 802.3 transport) of the current timeTransmitter, which in turn responds
  with a unicast ``Delay_Resp`` to the requesting timeReceiver. This reduces
  the level of PTP traffic on the network, which can be a factor when scaling
  to larger networks employing many timeReceivers. Hybrid mode only requires
  the network to support multicast transmission from the timeTransmitter to
  the timeReceivers. The mode is compatible with the ``hybrid_e2e`` option of
  linuxptp and the ``hybrid`` network mode of sfptpd; unicast negotiation is
  not supported.

  If the timeTransmitter fails to answer
  :kconfig:option:`CONFIG_PTP_HYBRID_FALLBACK_ATTEMPTS` consecutive unicast
  ``Delay_Req`` messages, the PTP Port logs an error, reverts to multicast
  delay measurement and stays in multicast until a new timeTransmitter is
  selected. The fallback can be disabled with
  :kconfig:option:`CONFIG_PTP_NETWORK_MODE_HYBRID_NO_FALLBACK`, in which case
  the port always keeps sending unicast ``Delay_Req`` messages.

  Hybrid mode requires the End-to-End delay mechanism
  (:kconfig:option:`CONFIG_PTP_DELAY_MECHANISM_E2E`) and is supported on all
  transport protocols (UDP IPv4, UDP IPv6 and IEEE 802.3).

Supported Management messages
*****************************

Based on Table 59 from section 15.5.2.3 of the IEEE 1588-2019 following management TLVs
are supported:

.. csv-table:: Supported management message's IDs
   :header: Management_ID, Management_ID name, Allowed actions
   :widths: 10,40,25

    0x0000, NULL_PTP_MANAGEMENT, GET SET COMMAND
    0x0001, CLOCK_DESCRIPTION, GET
    0x0002, USER_DESCRIPTION, GET
    0x0003, SAVE_IN_NON_VOLATILE_STORAGE, -
    0x0004, RESET_NON_VOLATILE_STORAGE, -
    0x0005, INITIALIZE, -
    0x0006, FAULT_LOG, -
    0x0007, FAULT_LOG_RESET, -
    0x2000, DEFAULT_DATA_SET, GET
    0x2001, CURRENT_DATA_SET, GET
    0x2002, PARENT_DATA_SET, GET
    0x2003, TIME_PROPERTIES_DATA_SET, GET
    0x2004, PORT_DATA_SET, GET
    0x2005, PRIORITY1, GET SET
    0x2006, PRIORITY2, GET SET
    0x2007, DOMAIN, GET SET
    0x2008, TIME_RECEIVER_ONLY, GET SET
    0x2009, LOG_ANNOUNCE_INTERVAL, GET SET
    0x200A, ANNOUNCE_RECEIPT_TIMEOUT, GET SET
    0x200B, LOG_SYNC_INTERVAL, GET SET
    0x200C, VERSION_NUMBER, GET SET
    0x200D, ENABLE_PORT, COMMAND
    0x200E, DISABLE_PORT, COMMAND
    0x200F, TIME, GET SET
    0x2010, CLOCK_ACCURACY, GET SET
    0x2011, UTC_PROPERTIES, GET SET
    0x2012, TRACEBILITY_PROPERTIES, GET SET
    0x2013, TIMESCALE_PROPERTIES, GET SET
    0x2014, UNICAST_NEGOTIATION_ENABLE, -
    0x2015, PATH_TRACE_LIST, -
    0x2016, PATH_TRACE_ENABLE, -
    0x2017, GRANDMASTER_CLUSTER_TABLE, -
    0x2018, UNICAST_TIME_TRANSMITTER_TABLE, -
    0x2019, UNICAST_TIME_TRANSMITTER_MAX_TABLE_SIZE, -
    0x201A, ACCEPTABLE_TIME_TRANSMITTER_TABLE, -
    0x201B, ACCEPTABLE_TIME_TRANSMITTER_TABLE_ENABLED, -
    0x201C, ACCEPTABLE_TIME_TRANSMITTER_MAX_TABLE_SIZE, -
    0x201D, ALTERNATE_TIME_TRANSMITTER, -
    0x201E, ALTERNATE_TIME_OFFSET_ENABLE, -
    0x201F, ALTERNATE_TIME_OFFSET_NAME, -
    0x2020, ALTERNATE_TIME_OFFSET_MAX_KEY, -
    0x2021, ALTERNATE_TIME_OFFSET_PROPERTIES, -
    0x3000, EXTERNAL_PORT_CONFIGURATION_ENABLED,
    0x3001, TIME_TRANSMITTER_ONLY, -
    0x3002, HOLDOVER_UPGRADE_ENABLE, -
    0x3003, EXT_PORT_CONFIG_PORT_DATA_SET, -
    0x4000, TRANSPARENT_CLOCK_DEFAULT_DATA_SET, -
    0x4001, TRANSPARENT_CLOCK_PORT_DATA_SET, -
    0x4002, PRIMARY_DOMAIN, -
    0x6000, DELAY_MECHANISM, GET
    0x6001, LOG_MIN_PDELAY_REQ_INTERVAL, GET SET

Timestamping notes
******************

When RX hardware timestamps are unavailable or invalid, synchronization falls
back to reading PHC time during receive processing. This can introduce
additional jitter from software handling latency (packet path and scheduling)
between frame arrival and PHC read.

This behavior is expected for L2 AF_PACKET paths without true driver-provided
RX hardware timestamps.

By default, Sync is sent in two-step mode and Follow_Up is
generated from TX timestamp callbacks. If a TX timestamp is missing or late,
the stack logs a warning and skips Follow_Up for that Sync sequence, then
continues normal Sync transmission on subsequent intervals (best-effort
behavior).

Peer-to-peer delay measurement can be selected with
:kconfig:option:`CONFIG_PTP_DELAY_MECHANISM_P2P`. Both two-step and one-step
``Pdelay_Resp`` are accepted. A one-step response carries the remote turnaround
time in its correction field and does not need ``Pdelay_Resp_Follow_Up``.
The calculation retains the fractional nanoseconds in the correction field.
One-step samples invalidate the neighbor-rate estimate because they do not
provide the separate remote timestamps needed by the estimator.

Optional packet acceleration
****************************

Hardware timestamp capture, one-step packet modification, and automatic packet
generation are separate capabilities. The ``PTP_PACKET_MODE`` choice selects a
preference; the stack discovers the Ethernet driver's capabilities at runtime:

* :kconfig:option:`CONFIG_PTP_PACKET_TWO_STEP` is the default and preserves
  software-generated messages and hardware timestamp callbacks.
* :kconfig:option:`CONFIG_PTP_PACKET_ONE_STEP` requests timestamp insertion in
  software-generated ``Sync`` and turnaround correction in ``Pdelay_Resp``.
  These messages clear ``twoStepFlag`` and have no follow-up message.
* :kconfig:option:`CONFIG_PTP_PACKET_AUTO` also permits hardware-generated
  ``Sync``, ``Delay_Resp``, and ``Pdelay_Resp`` when their required properties
  can be represented by the hardware template.

Packet acceleration is limited to ordinary clocks using IEEE 802.3 transport.
BMCA, Announce, servo control, and generation and matching of delay requests
remain in software. Unsupported operations use software scheduling and one-step
insertion when available, otherwise two-step timestamping. An uncertain
transmission is abandoned, not retransmitted using another mode.

Ethernet driver and packet-socket contract
=========================================

``ETHERNET_CONFIG_TYPE_PTP`` uses :c:struct:`ethernet_ptp_config` to discover
capabilities and configure operations, source port identity, domain, role, delay
mechanism, and intervals. The effective operations and configuration generation
are returned by ``get_config``. This interface does not change the PHC or its
frequency adjustment. A successful configuration change must serialize packet
submission and reception and preserve the old ownership of queued RX packets.

:kconfig:option:`CONFIG_NET_ETHERNET_PTP_OFFLOAD` adds optional
:c:struct:`net_ptp_packet` metadata to packets and their clones. Packet sockets
enable reception with an integer ``ZSOCK_SO_PTP_OFFLOAD`` socket option at
``ZSOCK_SOL_SOCKET``. ``sendmsg`` and ``recvmsg`` carry this structure in a
``ZSOCK_SCM_PTP_OFFLOAD`` control message at the same level. Existing timestamp
ancillary messages remain independent and can accompany it.

TX metadata requests one-step processing with the current generation and, for a
peer response, the request's ingress timestamp. Invalid metadata or stale
generations are rejected before submission. RX metadata assigns response
ownership for that packet's generation; ``NET_PTP_PACKET_RX_RESPONDED`` is
**not** confirmation of successful transmission. The stack suppresses software
responses to hardware-owned requests, including requests queued before a mode
change. Missing or truncated ownership information after enabling automatic
responses discards the exchange to avoid duplicate responses.

STM32H563 limitations
====================

:kconfig:option:`CONFIG_ETH_STM32_HAL_PTP_OFFLOAD` enables the STM32H563 HAL-v2
implementation. Other STM32 variants keep their existing transmit path. The
H563 path reserves a context descriptor and data descriptor together for each
packet, including ordinary traffic, and supports synchronous and asynchronous
TX. One ring slot remains unused; at least four TX descriptors are required.

The driver reports operation bits 0 through 4 for one-step Sync, one-step peer
response, automatic Sync, automatic delay response, and automatic peer response,
respectively. Effective operation masks are logged when the configuration
changes. The following restrictions apply:

* Offload is negotiated only for untagged, default-profile PTPv2 over Ethernet.
  VLAN interfaces and unsupported drivers retain software operation. When an
  automatic responder is active, RX ownership also covers C-tagged requests
  and either PTP multicast address accepted by the MAC filter. This prevents
  duplicate software responses, including for priority-tagged requests received
  on the physical port. One-step TX insertion still requires untagged frames.
  Hybrid mode uses insertion only; automatic responses are not negotiated.
* Automatic Sync requires a transmitter role, zero second-octet Sync flags,
  and a log interval from 0 through 15. Negative log intervals use software
  scheduling because RM0481 documents an approximate subsecond hardware cadence.
* Automatic E2E Delay_Resp requires a transmitter role, a Sync log interval
  from -15 through 15, and a Delay_Req minus Sync log interval from 0 through 5.
* Automatic Pdelay_Resp requires P2P operation. P2P automatic Sync also enables
  the peer responder because the hardware cannot separate these operations.
* Software peer-response insertion accepts ingress seconds through
  ``UINT32_MAX``. Wider timestamps use a two-step response.
* Automatic Delay_Req and Pdelay_Req are disabled: the hardware sequence
  counters cannot be read to match requests reliably in software.
* Automatic traffic stops on shutdown, link loss, or a DMA fault. After a fault,
  acceleration stays disabled until device reinitialization. DMA-owned buffers
  are retained until completion or a confirmed DMA stop; a persistently stuck
  DMA requires platform recovery. The PHC is not reset.

RM0481 Table 684 places the context descriptor's OSTC bit at bit 27; the prose
description naming bit 20 is inconsistent with the table. The implementation
uses bit 27 and clears one-step context on subsequent ordinary packets.

`STM32H563 errata ES0565`_ sections 2.22.6 and 2.22.7 also apply. Timestamp-status
interrupts are not enabled by packet offload; this avoids adding a status-clearing
path affected by the shadow-register erratum. Automatic packets can be corrupted
by a bus error and there is no hardware workaround. The driver disables offload
on fatal DMA bus errors, but cannot retract packets already transmitted.

Hardware acceptance requires captures against a hardware-timestamp-capable peer
in E2E and P2P, both roles, and all three modes. Check origin timestamps,
correction fields, cadence, sequence matching, absence of duplicate responses
and follow-ups, synchronization, role changes, link recovery, and fault handling.
Native tests and board builds do not establish wire-level interoperability.

.. _STM32H563 errata ES0565:
   https://www.st.com/resource/en/errata_sheet/es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf

Supported hardware
******************

Although the stack itself is hardware independent, Ethernet frame timestamping
support must be enabled in ethernet drivers.

Boards supported:

- :zephyr:board:`nucleo_h563zi`
- :zephyr:board:`nucleo_h743zi`
- :zephyr:board:`nucleo_h745zi_q`
- :zephyr:board:`nucleo_f767zi`
- :zephyr:board:`frdm_mcxn947`
- :zephyr:board:`native_sim` (only usable for simple testing, limited capabilities
  due to lack of hardware clock)

Enabling the stack
******************

The following configuration option must me enabled in :file:`prj.conf` file.

- :kconfig:option:`CONFIG_PTP`

Testing
*******

The stack has been informally tested using the
`Linux ptp4l <https://linuxptp.sourceforge.net/>`_ daemons. It has also been tested
with the :zephyr:board:`nucleo_h563zi` and :zephyr:board:`frdm_mcxn947` board
against a GPS clock, both with a direct Ethernet connection and with a PTP-capable
switch in between. All tests were performed using the
:zephyr:code-sample:`PTP sample application <ptp>`, with UDP IPv4, UDP IPv6, and
IEEE 802.3 transport.

The following table summarizes the informal test matrix:

+--------------------------------+-------------+----------+----------+
|                                | IEEE 802.3  | UDP IPv4 | UDP IPv6 |
+================================+=============+==========+==========+
| ptp4l daemons                  | yes         | yes      | yes      |
+--------------------------------+-------------+----------+----------+
| GPS Clock (direct link)        | yes         | yes      | yes      |
+--------------------------------+-------------+----------+----------+
| PTP-capable switch             | yes         | yes      | yes      |
+--------------------------------+-------------+----------+----------+
| GPS Clock + PTP-capable switch | yes         | yes      | yes      |
+--------------------------------+-------------+----------+----------+

Peer-to-peer delay measurement is implemented and validated for ordinary-clock
use. Boundary-clock operation is expected to share the same port-level Pdelay
machinery but has not yet been validated on multi-port hardware.

The :zephyr:code-sample:`PTP sample application <ptp>` from the Zephyr
source distribution can be used for testing.

.. _IEEE 1588-2019 standard:
   https://standards.ieee.org/ieee/1588/6825/

API Reference
*************

.. doxygengroup:: ptp
