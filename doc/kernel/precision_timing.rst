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

The PTP clock adapter exposes existing PTP hardware clocks through this API. It
infers conservative capabilities from legacy callbacks, and uses driver-reported
limits when the PTP clock driver implements the optional capability callback.
This keeps existing :c:func:`ptp_clock_get`, :c:func:`ptp_clock_set`,
:c:func:`ptp_clock_adjust`, and :c:func:`ptp_clock_rate_adjust` users working
while allowing precision timing users to query resolution, phase-adjustment
limits, and rate range.

Scheduled Clock Output
**********************

:kconfig:option:`CONFIG_PRECISION_CLOCK_OUTPUT` enables scheduled clock output.
An adapter advertises ``PRECISION_CLOCK_CAP_SCHEDULED_OUTPUT`` only when the
provider exposes at least one usable channel. It reports the capabilities and
limits of each output channel through :c:func:`precision_clock_output_get_caps`.
The channel capability flags describe what a channel can do, independently of
any particular timer or PTP peripheral:
one-shot events (``PRECISION_CLOCK_OUTPUT_CAP_EVENT``), periodic waveforms
(``PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM``), an exact programmable pulse width
(``PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH``), a rising or falling edge
(``PRECISION_CLOCK_OUTPUT_CAP_EDGE_RISING`` and ``..._EDGE_FALLING``), and
whether the channel can observe physical output activity
(``PRECISION_CLOCK_OUTPUT_CAP_HARDWARE_ACTIVE``). A provider may implement only
events, only waveforms, or both.

The API distinguishes two kinds of scheduled output:

* A one-shot **event** drives a single edge.
  :c:func:`precision_clock_output_schedule_event` takes a
  :c:struct:`precision_clock_output_event_config` with an absolute target time
  and an edge action. The channel must advertise
  ``PRECISION_CLOCK_OUTPUT_CAP_EVENT`` and the capability for the requested edge.
* A periodic **waveform** repeats a high pulse.
  :c:func:`precision_clock_output_start_waveform` takes a
  :c:struct:`precision_clock_output_waveform_config` with an absolute first
  rising edge, a period, and a high-pulse width policy. The channel must
  advertise ``PRECISION_CLOCK_OUTPUT_CAP_WAVEFORM``.

A waveform's high-pulse width follows one of two policies. With
``PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT`` the provider selects its native
default width and the configured ``pulse_width_ns`` is ignored. With
``PRECISION_CLOCK_OUTPUT_WIDTH_EXACT`` the caller supplies an exact width; the
channel must also advertise ``PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH`` and
the width must be positive and shorter than the period.

Scheduled times are :c:struct:`precision_time_point` values whose domain must
match the output clock. Periods and pulse widths are expressed in nanoseconds,
must be positive, and must be exactly representable at the channel resolution.
Providers may impose additional documented range and minimum-lead-time limits.
Scheduling an event or starting a waveform on an already configured channel is
rejected with ``-EBUSY``; stop the channel before reconfiguring it.

Use :c:func:`precision_clock_output_get_status` to retrieve the channel state
and :c:func:`precision_clock_output_stop` to disable it. Stop is idempotent and
is delegated directly to the provider, so a capability-query failure cannot
prevent an active output from being stopped.
:c:func:`precision_clock_output_next_start_time` calculates the first
period-aligned boundary satisfying a required lead time with checked arithmetic.
Capability, configuration, status, and stop operations run in thread context
and may block in the provider.

:c:struct:`precision_clock_output_status` reports two independent facts. The
``configured`` flag is always meaningful and indicates whether the provider
holds a configuration for the channel; when it is set, the ``kind`` and the
effective-configuration union describe that configuration. Physical output
activity is optional: ``hardware_active`` is meaningful only when
``hardware_active_valid`` is set, which happens only for a provider that can
observe the pin. A configured channel is therefore not by itself proof of a
physical waveform.

The generated waveform follows the underlying clock, so rate adjustments may
continue while an output is active. A hard set or phase step can cross a
scheduled edge or make the effective waveform discontinuous. Stop and re-arm
the output around a hard step. Scheduled output is protocol-neutral: it does
not imply PTP lock, UTC correctness, grandmaster health, or PPS accuracy.
PPS input and external timestamp capture are outside this API.

The scheduled-output provider contract is defined in
:zephyr_file:`include/zephyr/drivers/precision_clock_output.h` as
:c:struct:`precision_clock_output_provider`. A driver populates that structure
with raw callbacks that operate on values in the clock's own time domain, with
no protocol prefix. An adapter such as the PTP clock adapter discovers the
extension, attaches the clock domain, and bridges the raw callbacks onto the
domain-qualified public API. This feature is therefore generic and independent
of PTP; PTP merely adapts an existing PHC to it.

The STM32 Ethernet PTP-clock provider exposes FlexPPS channel zero when
:kconfig:option:`CONFIG_PTP_CLOCK_STM32_HAL_OUTPUT` is enabled and the MAC
reports that it is present. Its output resolution is derived from
:kconfig:option:`CONFIG_ETH_STM32_HAL_PTP_CLOCK_SRC_HZ`; with the STM32H563
default, the resolution is 20 nanoseconds. On unaffected STM32 devices, at
that resolution, the minimum period is 40 nanoseconds, the minimum pulse width
is 20 nanoseconds, and the maximum period or width is 85.899345900 seconds.
The first edge must be aligned to the resolution, at least one millisecond in
the future, and have a seconds value representable by the MAC's 32-bit target
register.

STM32H562/H563/H573 erratum ES0565 Rev 8 section 2.22.9 documents that FlexPPS
pulse-train mode can produce an incorrect interval when fine timestamp
correction is combined with a large frequency drift between the driving clock
and the grandmaster. ST's documented workaround is to use coarse timestamp
correction. The provider instead preserves the PTP clock's existing fine
correction mode and avoids the affected pulse-train mode on these devices. It
issues each pulse as an absolute single-pulse command and rearms the next pulse
after the preceding falling edge on a dedicated workqueue, avoiding contention
with unrelated system-workqueue handlers. Consequently, the H562/H563/H573
provider advertises and accepts only a one-second period and pulse widths from
one resolution tick through an upper bound derived from the configured minimum
and maximum PHC rates. The bound reserves the falling-edge guard, minimum lead
time, workqueue tick rounding, and command-processing margin needed to rearm
the following pulse, and is capped at 500 milliseconds. With the default
90--110 percent rate envelope and the H563 default resolution, the range is
20-nanosecond-aligned widths from 20 nanoseconds through 500 milliseconds. A
far-future first edge remains valid: the workqueue approaches the target using
PHC time and starts its independent monotonic falling-edge guard only after
observing that target. A PHC step therefore cannot end the guard early, while
the original lead time cannot delay the following pulse. This is not ST's
documented workaround and requires hardware validation; successfully arming
the channel alone does not prove correct PPS timing.

Autonomous PPS Output
*********************

:kconfig:option:`CONFIG_PRECISION_PPS_OUTPUT` enables a runtime-configured
pulse-per-second service built on top of scheduled clock output. It depends on
:kconfig:option:`CONFIG_PRECISION_CLOCK_OUTPUT` and is a separate, opt-in layer
above it: applications that only need direct control of a provider's scheduled
output continue to call :c:func:`precision_clock_output_start_waveform` and
related functions without enabling this service.

The service is instance-based. Each instance is a caller-owned
:c:struct:`precision_pps_output` object, embedded by value in application
storage such as a static variable and kept valid for the instance's lifetime.
Multiple instances arm and run independently, each targeting its own clock and
channel. Every instance always drives a fixed one-second period; the rest of
the behavior comes from a :c:struct:`precision_pps_output_config` supplied at
initialization:

* ``channel`` selects the zero-based output channel.
* ``width_policy`` chooses the provider-default or the exact high-pulse width,
  and ``pulse_width_ns`` supplies the active-high width used only by the exact
  policy.
* ``start_guard_ns`` adds a non-negative guard beyond the provider's minimum
  lead time when computing the next whole-second start boundary.
* ``step_limit_ns`` is the hard-step threshold: an absolute continuity error
  between the clock and the instance's internal monotonic reference above this
  value forces a stop and rearm.
* ``poll_interval_ms`` selects how often the instance re-reads the clock and
  output status.

The lifetime is init, start, observe, and stop.
:c:func:`precision_pps_output_init` copies the configuration into fresh instance
storage, validates it against the fixed one-second period, and prepares the poll
work and lock without touching the clock.
:c:func:`precision_pps_output_start` validates the configuration against the
channel's reported output capabilities and schedules the first poll, returning
``-ENOTSUP`` or ``-ERANGE`` for a configuration the provider cannot satisfy, or
the provider's own error from a failed capability query. Arming is not gated on
PTP lock or any other synchronization state.
:c:func:`precision_pps_output_state_get` copies a coherent state snapshot, and a
stopped instance may be started again without reinitialization.

All instances share a single subsystem-owned dedicated workqueue, separate from
the system workqueue, whose stack size and priority are set by
:kconfig:option:`CONFIG_PRECISION_PPS_OUTPUT_WORKQUEUE_STACK_SIZE` and
:kconfig:option:`CONFIG_PRECISION_PPS_OUTPUT_WORKQUEUE_PRIORITY`. All polls and
callbacks run serially on that thread. Instance state, configuration, and
lifetime are independent, but a blocking provider operation or callback for one
instance can delay polling of every other instance.

A started instance requires exclusive management of its configured clock and
output channel until :c:func:`precision_pps_output_stop` succeeds. No other PPS
instance or direct scheduled-output API caller may operate on that channel in
the meantime. The service can adopt a matching waveform found by its first poll,
but only when the caller has transferred exclusive management of that waveform
to the instance. This ownership contract is caller-enforced because different
clock adapters can alias the same physical hardware channel.

Each poll performs one fresh clock read and one output-status query, and reacts
to what it observes:

* An already-active output whose configuration matches the instance
  configuration is adopted without a stop or start call.
* A read or status error preserves a possibly-still-correct output; the
  instance retries a fresh read and status query on the next poll instead of
  stopping output on a single transient failure.
* A hard step (an absolute continuity error beyond the configured step limit),
  an inactive output, or an active output whose configuration no longer matches
  triggers at most one stop attempt and, if that succeeds, at most one start
  attempt within the same poll. A start target is always the next whole-second
  boundary at or beyond the provider's minimum lead time plus the configured
  start guard. A failed stop or start is retried, at most once each, on the
  following poll.

:c:struct:`precision_pps_output_state` carries two saturating counters that do
not wrap. ``generation`` counts every successful arm and rearm, while
``rearm_count`` counts only the successful rearms that followed a prior armed
state. The callback receives an immutable snapshot of this state together with
an event bitmask built from ``PRECISION_PPS_OUTPUT_EVENT_ARMED``,
``_RECOVERED``, ``_HARD_STEP``, ``_OUTPUT_INACTIVE``, ``_READ_ERROR``,
``_READ_RECOVERED``, ``_STATUS_ERROR``, ``_STATUS_RECOVERED``, ``_STOP_ERROR``,
and ``_START_ERROR`` values. Its first argument is the instance that produced
the event, and it is invoked from the workqueue thread with no instance lock
held, so it may call back into the service on a different instance. It is not
invoked for a poll that changes nothing.
:c:func:`precision_pps_output_state_get` is thread-safe and returns the same
snapshot type on demand, independent of the callback.

Stop an instance with :c:func:`precision_pps_output_stop`. On success it
synchronously cancels the instance's poll work, stops the output, and guarantees
that no further callback invocation or clock access follows its return, so the
caller may then release the clock. If stopping the output fails, the instance is
left started and polling resumes so that the caller cannot mistakenly release a
clock the service is still driving; the provider's error is returned instead.
Calling stop from inside the instance's own callback would deadlock against that
synchronous cancellation, so a self-stop is detected and rejected with
``-EDEADLK``; stopping a different instance from a callback is allowed.

This service intentionally has no Ethernet or PTP clock discovery of its own and
no PTP lock gate; callers that need automatic clock discovery, such as
:zephyr_file:`samples/net/pps_out`, perform it themselves before calling
:c:func:`precision_pps_output_init`.

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

The engine tracks ``UNSYNCED``, ``ACQUIRING``, ``LOCKED``, ``HOLDOVER``, and
``FAULT`` style state without embedding PTP-specific clock quality fields.
Source timeout handling enters holdover first, then resets the discipline when
the configured holdover interval expires. A clock-operation failure enters the
sticky ``FAULT`` state and blocks further control until the protocol explicitly
resets the discipline.

Precision Timing Shell
**********************

:kconfig:option:`CONFIG_PRECISION_TIMING_SHELL` enables a fixed-capacity,
runtime registry and the ``precision_clock`` shell command. The registry size
is selected by
:kconfig:option:`CONFIG_PRECISION_TIMING_SHELL_MAX_INSTANCES`, which defaults to
four. Register a clock with :c:func:`precision_timing_shell_register`, and
unregister it synchronously with
:c:func:`precision_timing_shell_unregister`.

Names are case-sensitive, copied into the registry, limited to 31 characters,
and may contain ASCII letters, digits, ``.``, ``-``, and ``_``. The caller owns
the registered clock and must keep it valid until unregister returns.
Unregister hides the name from new commands, then waits for in-flight shell
operations before releasing that lifetime obligation. Registration and
unregistration must run in thread context.

The shell provides read-only clock inspection:

.. code-block:: console

   precision_clock list
   precision_clock get <name>
   precision_clock caps <name>

``get`` prints signed nanoseconds and the clock domain type and identifier.
``caps`` prints support for read, set, phase adjustment, and rate adjustment,
scheduled output, plus the resolution, maximum phase adjustment, and rate
range. The shell does not expose clock set, phase, or rate commands; the clock
owner remains responsible for discipline and adjustment policy.

When :kconfig:option:`CONFIG_PRECISION_CLOCK_OUTPUT` is enabled, nested output
commands expose the scheduled-output API:

.. code-block:: console

   precision_clock output caps <name> <channel>
   precision_clock output get <name> <channel>
   precision_clock output event <name> <channel> <target_time_ns> <rising|falling>
   precision_clock output waveform <name> <channel> <first_rising_ns> <period_ns> [pulse_width_ns]
   precision_clock output stop <name> <channel>

``event`` schedules a single edge and ``waveform`` starts a periodic output;
their time arguments are absolute times in the registered clock's domain. The
optional ``waveform`` pulse width selects the exact width policy when supplied
and the provider-default width policy when omitted. The ``caps`` and ``get``
commands show the channel limits and the accepted active configuration.
Provider errors, including late targets, active-channel reconfiguration, and
unsupported channels, are printed and returned unchanged.

The PPS convenience commands configure a one-second period without adding PPS
policy to the provider:

.. code-block:: console

   precision_clock pps start <name> <channel> [pulse_width_ns]
   precision_clock pps stop <name> <channel>

``pps start`` reads the output capabilities and registered clock, then chooses
the first whole-second boundary at or after the provider's minimum lead time
plus a two-second scheduling guard. It retries once at a later boundary if the
first target becomes late. Its default active-high pulse width is 200
milliseconds. It prints the chosen start time, period, and width; specifying
the optional width overrides only that default. The helper does not check
protocol synchronization state. Stop and re-arm it when a hard clock step
occurs.

Provider errors are printed and returned unchanged. Runtime name completion is
intentionally omitted so unregister cannot race a completion name reference.

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

This subsystem implements stage 2 and the scheduled-output portion of stage 4,
referred to here as stage 4a, of the PTP/gPTP roadmap discussed in
`issue 107837 <https://github.com/zephyrproject-rtos/zephyr/issues/107837>`_,
which identifies protocol-neutral time-domain conversion, clock discipline, and
precision I/O as reusable components.

The domain mapping is therefore maintained by PTP and gPTP even though those
protocols only need the discipline today. Keeping the mapping alongside the
discipline means that a source change, hard step, or fault invalidates both in
one place. Applications can use that correlation without maintaining a second
clock disciplined from the PHC.

Stage 4a covers scheduled clock output only, including the autonomous PPS output
service built on it. PPS input and external timestamp capture are intentionally
deferred to a later stage.

API Reference
*************

.. doxygengroup:: precision_timing

.. doxygengroup:: precision_time

.. doxygengroup:: precision_clock

.. doxygengroup:: precision_clock_output_provider

.. doxygengroup:: precision_mapping

.. doxygengroup:: precision_pi

.. doxygengroup:: precision_deadline

.. doxygengroup:: precision_clock_ptp

.. doxygengroup:: precision_pps_output

.. doxygengroup:: precision_timing_shell
