:orphan:

.. _migration_5.0:

Migration guide to Zephyr v5.0.0 (Working Draft)
################################################

This document describes the changes required when migrating your application to Zephyr v5.0.0.
Other changes can be found in the :ref:`release notes <zephyr_5.0>`.

Drivers and Sensors
*******************

Fuel Gauge
==========

* The unitless property enum aliases and :c:union:`fuel_gauge_prop_val` members deprecated
  in Zephyr 4.5 have been removed (:github:`104236`). Update applications and out-of-tree
  drivers to use the unit-suffixed names. For example, replace ``FUEL_GAUGE_CURRENT`` and
  ``val.current`` with ``FUEL_GAUGE_CURRENT_UA`` and ``val.current_ua``.

  The replacement suffixes are ``_UA``/``_ua`` (microamperes), ``_UV``/``_uv`` (microvolts),
  ``_UAH``/``_uah`` (microampere-hours), ``_MV``/``_mv`` (millivolts), ``_MINS``/``_mins``
  (minutes), ``_PCT``/``_pct`` (percent), and ``_DK``/``_dk`` (deci-kelvin).
  The charging current and voltage members are ``chg_current_ua`` and ``chg_voltage_uv``;
  the design voltage member is ``design_volt_mv``. The replacement members for
  ``avg_current``, ``current``, and ``voltage`` use ``int32_t`` rather than ``int``.

  The supported property values, member types, and units have not changed. Properties with
  units selected at runtime by ``FUEL_GAUGE_SBS_MODE`` remain available without a unit suffix:
  ``FUEL_GAUGE_DESIGN_CAPACITY``, ``FUEL_GAUGE_SBS_ATRATE``, and
  ``FUEL_GAUGE_SBS_REMAINING_CAPACITY_ALARM``, together with their corresponding members.
