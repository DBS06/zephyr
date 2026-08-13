.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
.. SPDX-License-Identifier: Apache-2.0

.. _precision_timing:

Precision Timing
################

The precision timing subsystem provides protocol-neutral helpers for
applications and subsystems that need to relate multiple clock domains. It is
enabled by :kconfig:option:`CONFIG_PRECISION_TIMING`.

.. note::

   The API is at :ref:`experimental <api_lifecycle_experimental>` maturity and
   may still change. Protocols that build on it, such as gPTP, do not become
   experimental by depending on this subsystem.

The API is split into :zephyr_file:`include/zephyr/precision_timing`, with one
header per topic. :zephyr_file:`include/zephyr/precision_timing/precision_timing.h`
is a convenience header that includes all of them.

The subsystem is intentionally below protocol layers such as PTP and gPTP. It
does not define grandmaster selection, clock quality, packet formats, or network
state machines. Protocols remain responsible for source selection and for
deciding when source changes invalidate synchronization state.

Clock Domains
*************

Every :c:struct:`precision_time_point` carries a
:c:struct:`precision_time_domain`. Raw counter values are not assumed to be TAI,
UTC, monotonic time, or PHC time unless the caller labels them with a domain.
Operations that combine or compare time points reject mismatched domains.

Domain mappings are represented by :c:struct:`precision_time_mapping`. The
mapping helper uses :c:struct:`timeutil_sync_state` internally and converts
between source and local domains with explicit ``-EAGAIN`` errors while no valid
mapping is available. A hard clock step, synchronization source change, or
discipline fault should invalidate the mapping before new observations are
accepted.

Clock Abstraction
*****************

:c:struct:`precision_clock` describes a clock with optional operations for read,
set, phase adjustment, rate adjustment, and capability queries. Rate adjustment
uses signed parts-per-billion at the precision timing boundary.

A clock provider must return the domain declared by its
:c:struct:`precision_clock`. A successful read that reports another domain is
rejected, so a provider cannot silently substitute one time scale for another.

The PTP clock adapter exposes existing PTP hardware clocks through this API. It
infers conservative capabilities from legacy callbacks, and uses driver-reported
limits when the PTP clock driver implements the optional capability callback.
This keeps existing :c:func:`ptp_clock_get`, :c:func:`ptp_clock_set`,
:c:func:`ptp_clock_adjust`, and :c:func:`ptp_clock_rate_adjust` users working
while allowing precision timing users to query resolution, phase-adjustment
limits, and rate range.

A zero ``max_phase_adjust_ns`` with phase adjustment enabled means that the
provider does not advertise a bound; it does not promise an unlimited range.
Callers must handle ``-ERANGE`` or ``-ENOTSUP`` from an attempted adjustment.

Adjustable Software Clock
*************************

:kconfig:option:`CONFIG_PRECISION_SOFTWARE_CLOCK` enables a caller-owned,
monotonic-backed software clock. When the platform provides a 64-bit system
cycle counter with a stable frequency, the clock uses it for sub-tick
resolution. Platforms without that facility, including platforms supporting
runtime system-timer frequency changes, use kernel uptime ticks as a portable
fallback. Initialize a
:c:struct:`precision_software_clock` with
:c:func:`precision_software_clock_init`, then obtain its clock interface with
:c:func:`precision_software_clock_get`.

The software clock supports read, set, phase adjustment, rate adjustment, and
capability discovery. Rate changes first re-anchor the current value, keeping
the clock continuous across the change. Explicit set and phase adjustments may
move the software clock backward. Signed parts-per-billion skew is calculated
with checked integer arithmetic.

The reported resolution is one system cycle for the cycle-counter backend and
one kernel tick for the fallback. The backing counter is monotonic but is not
synchronized to the source clock; the synchronization service still disciplines
the software clock's phase and rate.

Stage 3 deliberately chooses a caller-owned, application-visible time scale.
The constraint is that :c:macro:`SYS_CLOCK_REALTIME` is global and does not
provide the portable, exclusively owned rate-control contract required by the
synchronization service. The software clock therefore leaves kernel monotonic
time and :c:macro:`SYS_CLOCK_REALTIME` unchanged while it is being disciplined.

Three properties make a separate clock necessary rather than merely convenient:

* **Resolution.** :c:macro:`SYS_CLOCK_REALTIME` is derived from
  :c:func:`k_uptime_ticks`, so it can never read finer than one kernel tick.
  At the default :kconfig:option:`CONFIG_SYS_CLOCK_TICKS_PER_SEC` of 100 used
  by ``native_sim`` and QEMU targets that is 10 ms. The cycle-counter backend
  of the software clock reports 1 us resolution on ``native_sim/native/64``.
  The gap is four orders of magnitude and is structural, not a tuning
  parameter.
* **Cardinality.** :c:macro:`SYS_CLOCK_REALTIME` is a singleton with a single
  offset. It cannot represent several PTP domains at once, and it cannot be
  the sink of a PHC-to-PHC synchronization.
* **Time scale.** :c:macro:`SYS_CLOCK_REALTIME` is UTC by convention, while
  the interpretation of a PTP clock depends on its time-properties dataset.
  Mapping one to the other requires interpreting ``ptpTimescale``,
  ``currentUtcOffset``, and leap-second properties, which generic timing code
  deliberately does not do.

Applications may explicitly call :c:func:`precision_clock_step_realtime` to
perform a one-shot system real-time update from a clock whose domain type is
``PRECISION_TIME_DOMAIN_UTC``. The helper is enabled by
:kconfig:option:`CONFIG_PRECISION_REALTIME_BRIDGE` and deliberately rejects raw
PHC, PTP, TAI, and other domains. It does not infer UTC, schedule repeated
updates, or establish ownership of the global clock. Protocol-specific
UTC-offset and leap-second policy therefore remain above the generic precision
timing layer. It interprets the UTC clock value as signed nanoseconds from the
POSIX epoch, including representable dates before that epoch. After the step,
system real-time continues at kernel tick resolution and may drift with the
system timer independently of the precision clock.

Discipline Engine
*****************

:c:struct:`precision_pi_discipline` is an instance-based PI discipline. Its
default proportional and integral gains are configured by
:kconfig:option:`CONFIG_PRECISION_TIMING_PI_KP` and
:kconfig:option:`CONFIG_PRECISION_TIMING_PI_KI`, both expressed in thousandths
as defined by :c:macro:`PRECISION_PI_GAIN_DEN`. It
accepts domain-qualified observations and returns a control decision instead of
touching hardware directly:

* ``PRECISION_DISCIPLINE_STEP`` for large phase corrections.
* ``PRECISION_DISCIPLINE_ADJUST_RATE`` for normal frequency discipline.
* ``PRECISION_DISCIPLINE_IGNORE`` for rejected or stale samples.
* ``PRECISION_DISCIPLINE_RESET`` when outlier policy requires reacquisition.

:c:member:`precision_pi_config.target_offset_ns` declares the signed
relationship between the two clock domains:

.. code-block:: none

   desired sink time = source time + target_offset_ns

PTP and gPTP initialize this value to zero, preserving their existing
zero-offset behavior. Precision timing does not infer whether either domain is
TAI, UTC, or another time scale, and does not interpret leap seconds or a PTP
``currentUtcOffset``. The caller supplies the domains and their relationship.

The engine tracks ``UNSYNCED``, ``ACQUIRING``, ``LOCKED``, ``HOLDOVER``, and
``FAULT`` style state without embedding PTP-specific clock quality fields.
Source timeout handling enters holdover first, then resets the discipline when
the configured holdover interval expires. A clock-operation failure enters the
sticky ``FAULT`` state and blocks further control until the protocol explicitly
resets the discipline.

Clock Synchronization Service
*****************************

:kconfig:option:`CONFIG_PRECISION_CLOCK_SYNC_SERVICE` enables fixed,
caller-owned synchronization instances. A
:c:struct:`precision_clock_sync_config` identifies one source clock, one sink
clock, the PI policy, update interval, and number of readings per update.
:c:func:`precision_clock_sync_config_default` supplies the default policy, and
:c:func:`precision_clock_sync_init` copies and validates the configuration.
The caller-owned synchronization state must be zero-initialized before its
first initialization. Initializing an instance that is already initialized
returns ``-EBUSY``; release it with :c:func:`precision_clock_sync_deinit`
first to reconfigure it. Once an instance has been initialized once, the
lifecycle calls serialize against each other, so a release that races a
concurrent start, stop, or reset makes that call fail with ``-EINVAL``
instead of acting on a released instance. The very first initialization of a
given instance is the caller's responsibility to serialize, as for any
zero-initialized object.

The source and sink must be distinct clock instances with distinct domain
identities. The source must be readable. The sink must support read and rate
adjustment, plus either bounded phase adjustment or absolute set when stepping
is enabled. The update interval must be positive and at most one hour.
Initialization intersects configured rate limits with the sink's
capabilities and rejects an empty range. A synchronization instance exclusively
owns its sink while it is active; its source may remain controlled by PTP or
gPTP.

Each update brackets a source read with two sink reads and compares the source
time with the sink midpoint. The service repeats the bracketed read five times
by default and uses the valid attempt with the smallest sink bracket. Reported
sampling uncertainty is half that bracket plus the source and sink clock
resolutions.

The default policy uses:

* one-second update intervals and five bracketed readings per update;
* a one-second step threshold;
* a 10-millisecond lock threshold for three consecutive samples;
* a 100-millisecond outlier threshold for two consecutive samples;
* a maximum sampling uncertainty equal to the advertised source and sink
  resolutions plus a 10-millisecond sampling margin;
* Kconfig-selected proportional and integral gains (0.7 and 0.3 by default); and
* a three-second source timeout followed by three seconds of holdover.

A hard step first restores nominal rate, then uses bounded phase adjustment
when possible. An unsupported or out-of-range phase adjustment falls back to
absolute set when available. The integral and lock history are cleared after
the step. Source read failures retain the last applied rate until the source
timeout, enter ``HOLDOVER``, and restore nominal rate and become ``UNSYNCED``
after holdover expires. Recovery starts acquisition with fresh PI state.
Transient source or sink sampling failures are counted and retried; sink
control failures enter sticky ``FAULT``, stop periodic updates, and require
:c:func:`precision_clock_sync_reset`.

:c:func:`precision_clock_sync_start` schedules the caller-owned instance on the
system workqueue. :c:func:`precision_clock_sync_stop` is synchronous and
idempotent, while reset works for stopped and running instances.
:c:func:`precision_clock_sync_deinit` stops the instance, waits for any
in-flight update, and marks it uninitialized so that it can be initialized
again with a different configuration. Deinitialization leaves the sink at its
last applied rate; call :c:func:`precision_clock_sync_reset` first when the
sink must return to nominal rate. Use
:c:func:`precision_clock_sync_get_status` to retrieve a coherent snapshot of
lifecycle, domains, discipline state, last action, offset, applied rate, source
age, uncertainty, observation counters, read and control failures, and the last
error.

The service does not select PTP or gPTP sources, consume PPS, or modify kernel
monotonic or real-time clocks. A readable PHC does not prove that its upstream
grandmaster is still valid. An application must stop or reset the service when
protocol ownership or source identity changes.

Precision Timing Shell
**********************

:kconfig:option:`CONFIG_PRECISION_TIMING_SHELL` enables a fixed-capacity,
runtime registry and the ``precision_clock`` shell command. The registry size
is selected by
:kconfig:option:`CONFIG_PRECISION_TIMING_SHELL_MAX_INSTANCES`, which defaults to
four. Register a clock and optional synchronization instance with
:c:func:`precision_timing_shell_register`, and unregister it synchronously with
:c:func:`precision_timing_shell_unregister`.

Names are case-sensitive, copied into the registry, limited to 31 characters,
and may contain ASCII letters, digits, ``.``, ``-``, and ``_``. The caller owns
the registered objects and must keep them valid until unregister returns.
Unregister hides the name from new commands, then waits for in-flight shell
operations before releasing that lifetime obligation. In-flight operations are
tracked globally, so unregister waits for commands on any registered instance,
not only on the name being removed. Registration and unregistration must run in
thread context.

The shell provides read-only clock inspection:

.. code-block:: console

   precision_clock list
   precision_clock get <name>
   precision_clock caps <name>

``get`` prints signed nanoseconds and the clock domain type and identifier.
``caps`` prints support for read, set, phase adjustment, and rate adjustment,
plus the resolution, maximum phase adjustment, and rate range. The shell does
not expose clock set, phase, or rate commands because a synchronization service
instance exclusively owns its sink.

When :kconfig:option:`CONFIG_PRECISION_CLOCK_SYNC_SERVICE` is enabled, these
additional commands expose the existing lifecycle and status APIs:

.. code-block:: console

   precision_clock status <name>
   precision_clock start <name>
   precision_clock stop <name>
   precision_clock reset <name>

``status`` prints every :c:struct:`precision_clock_sync_status` diagnostic.
``start``, ``stop``, and ``reset`` preserve the service semantics described
above, including synchronous, idempotent stop and reset-driven fault recovery.
Provider and lifecycle errors are printed and returned unchanged. Runtime name
completion is intentionally omitted so unregister cannot race a completion
name reference.

Protocol Integration
********************

PTP and gPTP use the shared PI discipline and the PTP-clock precision adapter
for PHC control. Their protocol datasets, packet timestamp storage, management
messages, state machines, and application APIs remain unchanged.

When a protocol performs a hard clock step, loses its synchronization source, or
changes source, it clears protocol delay samples and invalidates the associated
domain mapping. Normal accepted synchronization observations update both the PI
discipline and the source-to-local mapping.

Relation To The PTP Roadmap
***************************

This subsystem implements the common time representation, clock operations,
discipline, domain mapping, state, and diagnostics described by stage 2, and
the clock-to-clock synchronization service described by stage 3 of the
PTP/gPTP roadmap discussed in
`issue 107837 <https://github.com/zephyrproject-rtos/zephyr/issues/107837>`_.
The stage 2 timestamped-event abstraction remains deferred to the stage 4
precision event work.

PTP and gPTP maintain domain mappings as part of their protocol integration.
The stage 3 service instead operates directly on two fixed clock instances and
their explicitly declared domains and offset. It can discipline a PHC from
another PHC or an application-visible software clock from a PHC without
changing protocol source selection.

Relation To The Precision Clock Subsystem RFC
*********************************************

`RFC 76335 <https://github.com/zephyrproject-rtos/zephyr/issues/76335>`_
is a work-in-progress proposal for a broader abstract precision clock
architecture. The precision timing subsystem shares several of its concepts,
but does not claim to implement or conform to the complete architecture.

The implemented subset provides domain-qualified time points, a clock
abstraction with explicit capabilities, a shared PI discipline, and a
clock-to-clock synchronization service for the concrete PTP and gPTP roadmap
use cases.

The RFC also discusses several areas that are intentionally deferred here, so
that later designs can adopt them without having to undo choices made here:

* Overflow-protected uptime-counter stacks, including hybrid low-power
  wake/sleep counters and clock-stack assembly through Kconfig, devicetree, or
  runtime configuration.
* Generic timing event services for hardware timestamping, PPS-style capture,
  and scheduled hardware RX/TX. Stage 4 of the roadmap addresses PPS and
  precision event I/O separately.
* Timescale and client-adapter work for POSIX and C library clocks, UTC and
  TAI epochs, calendar or time-zone integration, and leap-second or drift
  smearing algorithms. This stays above the generic layer; see
  :c:func:`precision_clock_step_realtime`.
* Broader use cases beyond Ethernet PTP and gPTP, including BLE and
  IEEE 802.15.4 timing.

This implementation retains Zephyr's existing PTP clock driver API in
:zephyr_file:`include/zephyr/drivers/ptp_clock.h`. The adapter in
:zephyr_file:`include/zephyr/precision_timing/precision_clock_ptp.h` sits on
top of that API. Source selection and protocol policy likewise remain PTP and
gPTP responsibilities.

API Reference
*************

.. doxygengroup:: precision_timing

.. doxygengroup:: precision_time

.. doxygengroup:: precision_clock

.. doxygengroup:: precision_clock_ptp

.. doxygengroup:: precision_software_clock

.. doxygengroup:: precision_clock_sync

.. doxygengroup:: precision_realtime

.. doxygengroup:: precision_mapping

.. doxygengroup:: precision_pi

.. doxygengroup:: precision_deadline

.. doxygengroup:: precision_timing_shell
