/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/drivers/precision_clock_output.h>
#include <zephyr/drivers/ethernet/eth_nxp_enet_qos.h>
#include <zephyr/ztest.h>

#if !defined(TEST_NXP_OUTPUT_DISABLED)
/* Include the real platform-independent APIs before redirecting hardware calls. */
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mux.h>
#include <zephyr/drivers/pinctrl.h>

#define CONFIG_PTP_CLOCK_NXP_ENET_QOS_OUTPUT 1

static int test_gate_low(const struct gpio_dt_spec *spec, gpio_flags_t flags);
static int test_route(const struct pinctrl_dev_config *config, uint8_t state);
static int test_mux_apply(const struct device *dev, const struct mux_state *state);

#define gpio_pin_configure_dt test_gate_low
#define pinctrl_apply_state test_route
#define mux_state_apply test_mux_apply
#endif

static int test_clock_rate(const struct device *dev, clock_control_subsys_t subsys,
			   uint32_t *rate);
#define clock_control_get_rate test_clock_rate

/* Same inclusion technique as the PCIe PTM and SMBus driver tests. The provider,
 * its locks, its delayable work and PHC operations below are production code.
 */
#include "ptp_clock_nxp_enet_qos.c"

static enet_qos_t registers;
static struct ptp_clock_nxp_enet_qos_data clock_data;
static const struct nxp_enet_qos_config module_config = {
	.base = &registers,
};
static struct device_state module_state = {
	.initialized = true,
};
static const struct device module_device = {
	.config = &module_config,
	.state = &module_state,
};
static const struct ptp_clock_nxp_enet_qos_config clock_config = {
	.enet_qos_dev = &module_device,
#if !defined(TEST_NXP_OUTPUT_DISABLED)
	.output_gpio = {.port = &module_device},
	.mux_dev = &module_device,
#endif
};
static const struct device clock_device = {
	.name = "nxp-enet-qos-test",
	.config = &clock_config,
	.data = &clock_data,
	.api = &ptp_clock_nxp_enet_qos_api,
};

static bool pad_routed;
static bool phase_updated_while_routed;
static unsigned int phase_updates;

static precision_time_t hardware_time(void)
{
	return (precision_time_t)registers.MAC_SYSTEM_TIME_SECONDS * NSEC_PER_SEC +
	       registers.MAC_SYSTEM_TIME_NANOSECONDS;
}

static void hardware_time_set(precision_time_t time)
{
	registers.MAC_SYSTEM_TIME_SECONDS = time / NSEC_PER_SEC;
	registers.MAC_SYSTEM_TIME_NANOSECONDS = time % NSEC_PER_SEC;
}

size_t enet_qos_test_control_index(void)
{
	uint32_t control = registers.control[0];
	uint32_t update = registers.MAC_SYSTEM_TIME_NANOSECONDS_UPDATE;
	precision_time_t time;

	/* Emulate command completion on the next access, without replacing set()
	 * or adjust(). GPIO state is observed at the phase-register commit, so a
	 * test detects changing phase before gating the pending physical route.
	 */
	if ((control & (ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK |
			ENET_MAC_TIMESTAMP_CONTROL_TSUPDT_MASK)) != 0U) {
		phase_updates++;
		phase_updated_while_routed = pad_routed;
		if ((control & ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK) != 0U) {
			time = (precision_time_t)registers.MAC_SYSTEM_TIME_SECONDS_UPDATE *
			       NSEC_PER_SEC +
			       (update & ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK);
		} else {
			time = hardware_time();
			if ((update & ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB_MASK) != 0U) {
				time -= update & ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK;
			} else {
				time += update & ENET_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK;
			}
		}
		hardware_time_set(time);
	}
	registers.control[0] &= ~(ENET_MAC_TIMESTAMP_CONTROL_TSINIT_MASK |
				ENET_MAC_TIMESTAMP_CONTROL_TSUPDT_MASK |
				ENET_MAC_TIMESTAMP_CONTROL_TSADDREG_MASK);
	return 0;
}

static int test_clock_rate(const struct device *dev, clock_control_subsys_t subsys,
			   uint32_t *rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(subsys);
	*rate = 150000000U;
	return 0;
}

static K_SEM_DEFINE(mutex_held, 0, 1);
static K_SEM_DEFINE(mutex_release, 0, 1);
static K_THREAD_STACK_DEFINE(helper_stack, 2048);
static struct k_thread helper_thread;
static bool helper_thread_active;

static void hold_phase_mutex(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	k_mutex_lock(&clock_data.ptp_mutex, K_FOREVER);
	k_sem_give(&mutex_held);
	k_sem_take(&mutex_release, K_FOREVER);
	k_mutex_unlock(&clock_data.ptp_mutex);
}

ZTEST(nxp_enet_qos_provider, test_get_with_interrupts_locked_during_phase_transaction)
{
	struct net_ptp_time time;
	unsigned int key;
	int ret;

	hardware_time_set(4123 * (precision_time_t)NSEC_PER_MSEC);
	helper_thread_active = true;
	k_thread_create(&helper_thread, helper_stack, K_THREAD_STACK_SIZEOF(helper_stack),
			hold_phase_mutex, NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&mutex_held, K_SECONDS(1)));
	key = irq_lock();
	ret = ptp_clock_get(&clock_device, &time);
	irq_unlock(key);
	k_sem_give(&mutex_release);
	zassert_ok(k_thread_join(&helper_thread, K_SECONDS(1)));
	helper_thread_active = false;
	zassert_ok(ret);
	zassert_equal(time.second, 4);
	zassert_equal(time.nanosecond, 123 * NSEC_PER_MSEC);
}

#if !defined(TEST_NXP_OUTPUT_DISABLED)
static int gpio_error;
static int route_error;
static int cleanup_error_after_route;
static precision_time_t route_finishes_at;
static bool block_route;
static K_SEM_DEFINE(route_entered, 0, 1);
static K_SEM_DEFINE(route_release, 0, 1);
static K_SEM_DEFINE(stop_entered, 0, 1);
static K_SEM_DEFINE(stop_done, 0, 1);
static int stop_result;

static int test_gate_low(const struct gpio_dt_spec *spec, gpio_flags_t flags)
{
	ARG_UNUSED(spec);
	zassert_equal(flags, GPIO_OUTPUT_LOW);
	if (gpio_error != 0) {
		return gpio_error;
	}
	pad_routed = false;
	return 0;
}

static int test_route(const struct pinctrl_dev_config *config, uint8_t state)
{
	ARG_UNUSED(config);
	zassert_equal(state, PINCTRL_STATE_DEFAULT);
	if (block_route) {
		k_sem_give(&route_entered);
		k_sem_take(&route_release, K_FOREVER);
	}
	if (route_error != 0) {
		return route_error;
	}
	pad_routed = true;
	if (route_finishes_at != 0) {
		hardware_time_set(route_finishes_at);
	}
	if (cleanup_error_after_route != 0) {
		gpio_error = cleanup_error_after_route;
	}
	return 0;
}

static int test_mux_apply(const struct device *dev, const struct mux_state *state)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(state);
	return 0;
}

/* The native suite does not measure pulses or model electrical pin switching.
 * It verifies that the real driver keeps the register/GPIO transaction ordered
 * and retains ownership when physical cleanup is reported to have failed.
 */
static struct precision_clock_output_raw_waveform_config waveform(void)
{
	return (struct precision_clock_output_raw_waveform_config){
		.first_rising_time = 5 * (precision_time_t)NSEC_PER_SEC,
		.period_ns = NSEC_PER_SEC,
		.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_PROVIDER_DEFAULT,
	};
}

static const struct precision_clock_output_provider *provider(void)
{
	const struct device *dev = &clock_device;

	return DEVICE_API_GET(ptp_clock, dev)->output;
}

static int start_output(void)
{
	struct precision_clock_output_raw_waveform_config config = waveform();

	return provider()->start_waveform(&clock_device, 0, &config);
}

static int stop_output(void)
{
	return provider()->stop(&clock_device, 0);
}

static void assert_status(bool configured, int error)
{
	struct precision_clock_output_raw_status status;

	zassert_equal(provider()->get_status(&clock_device, 0, &status), error);
	zassert_equal(status.configured, configured);
	/* Pin mux state cannot establish whether the physical waveform is active. */
	zassert_false(status.hardware_active_valid);
	if (configured) {
		zassert_equal(status.kind, PRECISION_CLOCK_OUTPUT_KIND_WAVEFORM);
		zassert_equal(status.config.waveform.first_rising_time,
			      waveform().first_rising_time);
	}
}

static void run_pending_work(void)
{
	struct k_work_sync sync;

	/* Execute on the real system workqueue, never call the handler directly. */
	zassert_true(k_work_flush_delayable(&clock_data.output_work, &sync));
}

static void set_time(precision_time_t time, int expected_result)
{
	struct net_ptp_time ptp_time = {
		.second = time / NSEC_PER_SEC,
		.nanosecond = time % NSEC_PER_SEC,
	};

	zassert_equal(ptp_clock_set(&clock_device, &ptp_time), expected_result);
}

ZTEST(nxp_enet_qos_provider, test_future_first_target_routes_only_in_window)
{
	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(start_output());
	assert_status(true, 0);
	zassert_false(pad_routed);
	zassert_equal(start_output(), -EBUSY);
	hardware_time_set(4250 * (precision_time_t)NSEC_PER_MSEC);
	run_pending_work();
	zassert_false(pad_routed);
	assert_status(true, 0);
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	run_pending_work();
	zassert_true(pad_routed);
	zassert_true(hardware_time() < waveform().first_rising_time);
	assert_status(true, 0);
}

ZTEST(nxp_enet_qos_provider, test_start_inside_routing_window)
{
	hardware_time_set(waveform().first_rising_time - 20 * NSEC_PER_MSEC);
	zassert_ok(start_output());
	zassert_true(pad_routed);
	zassert_equal(k_work_delayable_busy_get(&clock_data.output_work), 0);
	assert_status(true, 0);
}

ZTEST(nxp_enet_qos_provider, test_unsupported_request_does_not_replace_owner)
{
	struct precision_clock_output_raw_waveform_config config = waveform();
	struct precision_clock_output_caps caps;

	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(provider()->get_caps(&clock_device, 0, &caps));
	zassert_false(caps.flags & PRECISION_CLOCK_OUTPUT_CAP_PROGRAMMABLE_WIDTH);
	zassert_ok(start_output());
	config.width_policy = PRECISION_CLOCK_OUTPUT_WIDTH_EXACT;
	config.pulse_width_ns = 200 * NSEC_PER_MSEC;
	zassert_equal(provider()->start_waveform(&clock_device, 0, &config), -ENOTSUP);
	config = waveform();
	config.first_rising_time++;
	zassert_equal(provider()->start_waveform(&clock_device, 0, &config), -ERANGE);
	config = waveform();
	config.period_ns *= 2;
	zassert_equal(provider()->start_waveform(&clock_device, 0, &config), -ERANGE);
	config = waveform();
	config.first_rising_time = ((precision_time_t)UINT32_MAX + 1) * NSEC_PER_SEC;
	zassert_equal(provider()->start_waveform(&clock_device, 0, &config), -ERANGE);
	assert_status(true, 0);
	zassert_false(pad_routed);
}

static void backward_phase_update(bool absolute)
{
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
	if (absolute) {
		set_time(3990 * (precision_time_t)NSEC_PER_MSEC, 0);
	} else {
		zassert_ok(ptp_clock_adjust(&clock_device, -510 * NSEC_PER_MSEC));
	}
	zassert_false(phase_updated_while_routed, "pending route was not gated before update");
	zassert_equal(hardware_time(), 3990 * (precision_time_t)NSEC_PER_MSEC);
	zassert_false(pad_routed);
	assert_status(true, 0);
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	run_pending_work();
	zassert_true(pad_routed);
	assert_status(true, 0);
}

ZTEST(nxp_enet_qos_provider, test_set_backward_before_first_edge)
{
	backward_phase_update(true);
}

ZTEST(nxp_enet_qos_provider, test_adjust_backward_before_first_edge)
{
	backward_phase_update(false);
}

static void forward_phase_update(bool absolute)
{
	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(start_output());
	if (absolute) {
		set_time(4750 * (precision_time_t)NSEC_PER_MSEC, 0);
	} else {
		zassert_ok(ptp_clock_adjust(&clock_device, 750 * NSEC_PER_MSEC));
	}
	zassert_false(phase_updated_while_routed);
	zassert_equal(hardware_time(), 4750 * (precision_time_t)NSEC_PER_MSEC);
	zassert_true(pad_routed);
	assert_status(true, 0);
}

ZTEST(nxp_enet_qos_provider, test_set_forward_routes_against_updated_phc)
{
	forward_phase_update(true);
}

ZTEST(nxp_enet_qos_provider, test_adjust_forward_routes_against_updated_phc)
{
	forward_phase_update(false);
}

static void missed_target_on_phase_update(bool absolute)
{
	hardware_time_set(4750 * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
	if (absolute) {
		set_time(5100 * (precision_time_t)NSEC_PER_MSEC, -ETIME);
	} else {
		zassert_equal(ptp_clock_adjust(&clock_device, 350 * NSEC_PER_MSEC), -ETIME);
	}
	zassert_false(phase_updated_while_routed);
	/* Rearming failed, but the PHC phase update itself has already committed. */
	zassert_equal(hardware_time(), 5100 * (precision_time_t)NSEC_PER_MSEC);
	zassert_false(pad_routed);
	assert_status(false, 0);
}

ZTEST(nxp_enet_qos_provider, test_set_crossing_first_target_reports_missed_activation)
{
	missed_target_on_phase_update(true);
}

ZTEST(nxp_enet_qos_provider, test_adjust_crossing_first_target_reports_missed_activation)
{
	missed_target_on_phase_update(false);
}

ZTEST(nxp_enet_qos_provider, test_running_output_is_not_regated_by_backward_step)
{
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
	hardware_time_set(5100 * (precision_time_t)NSEC_PER_MSEC);
	set_time(3990 * (precision_time_t)NSEC_PER_MSEC, 0);
	zassert_true(phase_updated_while_routed);
	zassert_true(pad_routed);
	zassert_ok(ptp_clock_adjust(&clock_device, 10 * NSEC_PER_MSEC));
	zassert_true(pad_routed);
	assert_status(true, 0);
}

static void stop_in_phase(unsigned int phase)
{
	hardware_time_set((phase == 0 ? 4000 : 4500) * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
	if (phase == 2) {
		hardware_time_set(5100 * (precision_time_t)NSEC_PER_MSEC);
		zassert_ok(ptp_clock_adjust(&clock_device, 0));
	}
	zassert_ok(stop_output());
	zassert_false(pad_routed);
	assert_status(false, 0);
	zassert_equal(k_work_delayable_busy_get(&clock_data.output_work), 0);
	/* A cancelled timer must not re-enable the route after stop returns. */
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	k_sleep(K_MSEC(300));
	zassert_false(pad_routed);
	zassert_ok(start_output());
	assert_status(true, 0);
}

ZTEST(nxp_enet_qos_provider, test_stop_held_low_cancels_future_routing)
{
	stop_in_phase(0);
}

ZTEST(nxp_enet_qos_provider, test_stop_routed_pending)
{
	stop_in_phase(1);
}

ZTEST(nxp_enet_qos_provider, test_stop_running)
{
	stop_in_phase(2);
}

ZTEST(nxp_enet_qos_provider, test_failed_stop_retains_owner_until_cleanup_succeeds)
{
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
	gpio_error = -EIO;
	zassert_equal(stop_output(), -EIO);
	zassert_true(pad_routed);
	assert_status(true, -EIO);
	zassert_equal(start_output(), -EBUSY);
	zassert_equal(stop_output(), -EIO);
	assert_status(true, -EIO);
	gpio_error = 0;
	zassert_ok(stop_output());
	zassert_false(pad_routed);
	assert_status(false, 0);
	zassert_ok(start_output());
}

ZTEST(nxp_enet_qos_provider, test_failed_phase_gate_rejects_update_and_retains_owner)
{
	unsigned int updates;

	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
	updates = phase_updates;
	gpio_error = -EIO;
	set_time(3990 * (precision_time_t)NSEC_PER_MSEC, -EIO);
	zassert_equal(ptp_clock_adjust(&clock_device, -510 * NSEC_PER_MSEC), -EIO);
	zassert_equal(phase_updates, updates);
	zassert_equal(hardware_time(), 4500 * (precision_time_t)NSEC_PER_MSEC);
	assert_status(true, -EIO);
	zassert_equal(start_output(), -EBUSY);
	gpio_error = 0;
	zassert_ok(stop_output());
	zassert_false(pad_routed);
	assert_status(false, 0);
}

ZTEST(nxp_enet_qos_provider, test_route_completion_missing_deadline_cleans_up)
{
	hardware_time_set(4980 * (precision_time_t)NSEC_PER_MSEC);
	route_finishes_at = waveform().first_rising_time;
	zassert_equal(start_output(), -ETIME);
	zassert_false(pad_routed);
	assert_status(false, 0);
	route_finishes_at = 0;
	hardware_time_set(4980 * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
}

ZTEST(nxp_enet_qos_provider, test_failed_start_retains_owner_when_cleanup_fails)
{
	hardware_time_set(4980 * (precision_time_t)NSEC_PER_MSEC);
	route_finishes_at = waveform().first_rising_time;
	cleanup_error_after_route = -EIO;
	zassert_equal(start_output(), -ETIME);
	zassert_true(pad_routed);
	assert_status(true, -ETIME);
	zassert_equal(start_output(), -EBUSY);
	zassert_equal(stop_output(), -EIO);
	assert_status(true, -EIO);
	gpio_error = 0;
	cleanup_error_after_route = 0;
	route_finishes_at = 0;
	zassert_ok(stop_output());
	assert_status(false, 0);
	hardware_time_set(4980 * (precision_time_t)NSEC_PER_MSEC);
	zassert_ok(start_output());
}

ZTEST(nxp_enet_qos_provider, test_work_missing_route_window_stays_low)
{
	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(start_output());
	hardware_time_set(waveform().first_rising_time - 1);
	run_pending_work();
	zassert_false(pad_routed);
	assert_status(false, 0);
}

ZTEST(nxp_enet_qos_provider, test_work_cleanup_failure_retains_owner_and_fault)
{
	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(start_output());
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	route_finishes_at = waveform().first_rising_time;
	gpio_error = -EIO;
	run_pending_work();
	zassert_true(pad_routed);
	assert_status(true, -ETIME);
	zassert_equal(start_output(), -EBUSY);
	zassert_equal(stop_output(), -EIO);
	assert_status(true, -EIO);
	gpio_error = 0;
	zassert_ok(stop_output());
	zassert_false(pad_routed);
	assert_status(false, 0);
}

ZTEST(nxp_enet_qos_provider, test_pinctrl_failure_retains_owner_if_gate_cleanup_fails)
{
	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(start_output());
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	route_error = -ENODEV;
	gpio_error = -EIO;
	run_pending_work();
	assert_status(true, -ENODEV);
	zassert_equal(start_output(), -EBUSY);
	gpio_error = 0;
	zassert_ok(stop_output());
	assert_status(false, 0);
}

static void stop_from_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	k_sem_give(&stop_entered);
	stop_result = stop_output();
	k_sem_give(&stop_done);
}

ZTEST(nxp_enet_qos_provider, test_stop_waits_for_running_routing_work)
{
	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(start_output());
	hardware_time_set(4500 * (precision_time_t)NSEC_PER_MSEC);
	block_route = true;
	zassert_true(k_work_reschedule(&clock_data.output_work, K_NO_WAIT) >= 0);
	zassert_ok(k_sem_take(&route_entered, K_SECONDS(1)));
	zassert_true(k_work_delayable_busy_get(&clock_data.output_work) & K_WORK_RUNNING);
	helper_thread_active = true;
	k_thread_create(&helper_thread, helper_stack, K_THREAD_STACK_SIZEOF(helper_stack),
			stop_from_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&stop_entered, K_SECONDS(1)));
	zassert_equal(k_sem_take(&stop_done, K_NO_WAIT), -EBUSY);
	k_sem_give(&route_release);
	zassert_ok(k_sem_take(&stop_done, K_SECONDS(1)));
	zassert_ok(k_thread_join(&helper_thread, K_SECONDS(1)));
	helper_thread_active = false;
	zassert_ok(stop_result);
	zassert_false(pad_routed);
	assert_status(false, 0);
	zassert_equal(k_work_delayable_busy_get(&clock_data.output_work), 0);
	block_route = false;
	/* Reusing the channel must not inherit a late route from cancelled work. */
	hardware_time_set(4 * (precision_time_t)NSEC_PER_SEC);
	zassert_ok(start_output());
	zassert_false(pad_routed);
}
#endif /* !TEST_NXP_OUTPUT_DISABLED */

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&registers, 0, sizeof(registers));
	memset(&clock_data, 0, sizeof(clock_data));
	pad_routed = false;
	phase_updated_while_routed = false;
	phase_updates = 0;
	k_sem_reset(&mutex_held);
	k_sem_reset(&mutex_release);
#if !defined(TEST_NXP_OUTPUT_DISABLED)
	gpio_error = 0;
	route_error = 0;
	cleanup_error_after_route = 0;
	route_finishes_at = 0;
	block_route = false;
	k_sem_reset(&route_entered);
	k_sem_reset(&route_release);
	k_sem_reset(&stop_entered);
	k_sem_reset(&stop_done);
#endif
	zassert_ok(ptp_clock_nxp_enet_qos_init(&clock_device));
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);
	k_sem_give(&mutex_release);
#if !defined(TEST_NXP_OUTPUT_DISABLED)
	gpio_error = 0;
	block_route = false;
	k_sem_give(&route_release);
#endif
	if (helper_thread_active) {
		zassert_ok(k_thread_join(&helper_thread, K_SECONDS(1)));
		helper_thread_active = false;
	}
#if !defined(TEST_NXP_OUTPUT_DISABLED)
	zassert_ok(stop_output());
#endif
}

ZTEST_SUITE(nxp_enet_qos_provider, NULL, NULL, before, after, NULL);
