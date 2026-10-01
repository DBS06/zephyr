:orphan:

.. _zephyr_5.0:

Zephyr 5.0.0 (Working Draft)
############################

API Changes
***********

Removed APIs and options
========================

* Fuel Gauge

  * Removed the unitless property enum aliases and :c:union:`fuel_gauge_prop_val`
    members deprecated in Zephyr 4.5. Use the unit-suffixed names instead.
    See the :ref:`migration guide <migration_5.0>` for details.
