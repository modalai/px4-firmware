/****************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Direct CDSP v68 ZVINS2 odometry ingress for PX4 in the ADSP v66 QSH.
 * The result path is one SMEM read, one root-owned IPCC wake, and one uORB
 * publication. APSS is not a relay and no user callback worker is involved.
 ****************************************************************************/
#include <drivers/drv_hrt.h>
#include <px4_log.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_odometry.h>

#include <pthread.h>
#include <stdint.h>
#include <string.h>

#include "qurt_signal.h"
#include "zvins_qsh_link_qdi.h"
#include "zvins_qsh_link_wire.h"

extern "C" __EXPORT int zvins_qsh_link_main(int argc, char *argv[]);

#ifndef ZQL_DIAGNOSTIC_HOP_TIMING
#define ZQL_DIAGNOSTIC_HOP_TIMING 0
#endif

namespace zvins_qsh_link
{

static constexpr uint16_t kFilterRunning = 2u;
/* The cadence deadline closes when the next message arrives. A 30 Hz clock
 * quantized to integer microseconds legitimately alternates between 33333 and
 * 33334 us, while 20 Hz (50000 us) is the lowest accepted sustained rate. */
static constexpr uint64_t kTargetMessageIntervalUs = 33334u;
static constexpr uint64_t kMaximumMessageIntervalUs = 50000u;
static constexpr uint64_t kRootRetryUs = 1000000u;
static constexpr uint64_t kSmemRetryUs = 100000u;
static constexpr unsigned kControlStopSignal = 1u << 0;

static_assert(vehicle_odometry_s::POSE_FRAME_FRD == ZQL_POSE_FRAME_FRD,
	      "wire/PX4 pose-frame contract changed");
static_assert(vehicle_odometry_s::VELOCITY_FRAME_FRD == ZQL_VELOCITY_FRAME_FRD,
	      "wire/PX4 velocity-frame contract changed");
static_assert(sizeof(((vehicle_odometry_s *)nullptr)->position) ==
	      sizeof(((zql_vehicle_odometry_t *)nullptr)->position),
	      "wire/PX4 position field changed");
static_assert(sizeof(((vehicle_odometry_s *)nullptr)->q) ==
	      sizeof(((zql_vehicle_odometry_t *)nullptr)->q),
	      "wire/PX4 quaternion field changed");
static_assert(sizeof(((vehicle_odometry_s *)nullptr)->velocity) ==
	      sizeof(((zql_vehicle_odometry_t *)nullptr)->velocity),
	      "wire/PX4 velocity field changed");
static_assert(sizeof(((vehicle_odometry_s *)nullptr)->angular_velocity) ==
	      sizeof(((zql_vehicle_odometry_t *)nullptr)->angular_velocity),
	      "wire/PX4 angular-velocity field changed");
static_assert(sizeof(((vehicle_odometry_s *)nullptr)->position_variance) ==
	      sizeof(((zql_vehicle_odometry_t *)nullptr)->position_variance),
	      "wire/PX4 position-variance field changed");
static_assert(sizeof(((vehicle_odometry_s *)nullptr)->orientation_variance) ==
	      sizeof(((zql_vehicle_odometry_t *)nullptr)->orientation_variance),
	      "wire/PX4 orientation-variance field changed");
static_assert(sizeof(((vehicle_odometry_s *)nullptr)->velocity_variance) ==
	      sizeof(((zql_vehicle_odometry_t *)nullptr)->velocity_variance),
	      "wire/PX4 velocity-variance field changed");

static px4::atomic_bool g_should_exit{false};
static px4::atomic_bool g_running{false};
static bool g_control_signal_initialized;
static qurt_signal_t g_control_signal;
static px4_task_t g_task = -1;

/* The stop path runs in the QSH command thread while the binding belongs to
 * the receiver thread.  Matched firmware wakes the root-owned cancellable
 * signal through WAKE_RX; an older root uses the remote RX signal.  The worker
 * distinguishes shutdown with g_should_exit. Hold this lock while invoking or
 * closing either handle so a recycled QDI handle is never signalled. */
static pthread_mutex_t g_binding_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_bound_wake_signal = -1;
static zql_transport_endpoint_t *g_bound_control;
static bool g_bound_wait_rx_ack;

struct Statistics {
	px4::atomic<uint32_t> root_binds{0};
	px4::atomic<uint32_t> doorbells{0};
	px4::atomic<uint32_t> published{0};
	px4::atomic<uint32_t> invalid_samples{0};
	px4::atomic<uint32_t> future_rejects{0};
	px4::atomic<uint32_t> target_cadence_misses{0};
	px4::atomic<uint32_t> minimum_rate_violations{0};
	px4::atomic<uint32_t> smem_failures{0};
	px4::atomic<uint32_t> transport_failures{0};
	px4::atomic<uint32_t> bind_failures{0};
	px4::atomic<uint32_t> wait_failures{0};
	px4::atomic<uint32_t> signal_failures{0};
	px4::atomic<uint32_t> spurious_wakes{0};
	px4::atomic<uint32_t> fused_waits{0};
	px4::atomic<uint32_t> legacy_waits{0};
	px4::atomic<uint32_t> recovery_waits{0};
	px4::atomic<uint32_t> last_sequence{0};
	px4::atomic<uint32_t> missed_samples{0};
	px4::atomic<uint32_t> ring_invalid_reads{0};
	px4::atomic<uint32_t> producer_restarts{0};
	px4::atomic<uint32_t> producer_generation{0};
	px4::atomic<uint32_t> producer_trigger_failures{0};
	px4::atomic<uint32_t> producer_rejected_samples{0};
	px4::atomic<uint32_t> root_doorbells_without_receiver{0};
	px4::atomic<uint32_t> root_signal_errors{0};
	px4::atomic<uint32_t> last_sample_age_us{0};
	px4::atomic<uint32_t> maximum_sample_age_us{0};
	px4::atomic<uint32_t> last_message_interval_us{0};
	px4::atomic<uint32_t> maximum_message_interval_us{0};
#if ZQL_DIAGNOSTIC_HOP_TIMING
	px4::atomic<uint32_t> diagnostic_samples{0};
	px4::atomic<uint32_t> diagnostic_invalid{0};
	px4::atomic<uint32_t> hop_last_ns{0};
	px4::atomic<uint32_t> hop_min_ns{0};
	px4::atomic<uint32_t> hop_max_ns{0};
	px4::atomic<uint32_t> wake_last_ns{0};
	px4::atomic<uint32_t> wake_max_ns{0};
	px4::atomic<uint32_t> uorb_tail_last_ns{0};
	px4::atomic<uint32_t> uorb_tail_max_ns{0};
#endif
};

static Statistics g_stats{};

struct CadenceState {
	uint64_t previous_publish_us{0};
	uint32_t previous_generation{0};
	bool valid{false};
};

static void update_max(px4::atomic<uint32_t> &destination, uint32_t value)
{
	uint32_t current = destination.load();

	while (value > current && !destination.compare_exchange(&current, value)) {}
}

#if ZQL_DIAGNOSTIC_HOP_TIMING
static constexpr uint64_t kQTimerMask = (UINT64_C(1) << 56) - 1u;
static constexpr uint64_t kMaximumDiagnosticTicks = UINT64_C(19200000);

/* CDSP v68 and ADSP v66 expose C31:30 as the same 19.2 MHz global counter. */
static uint64_t diagnostic_qtimer()
{
	uint64_t ticks;
	__asm__ __volatile__("%0 = c31:30" : "=r"(ticks));
	return ticks & kQTimerMask;
}

static uint64_t diagnostic_slot_qtimer(const zql_odometry_slot_t &slot)
{
	return (static_cast<uint64_t>(slot.reserved[ZQL_DIAGNOSTIC_QTIMER_LO_WORD]) |
		(static_cast<uint64_t>(slot.reserved[ZQL_DIAGNOSTIC_QTIMER_HI_WORD]) << 32)) &
	       kQTimerMask;
}

static bool diagnostic_delta_ns(uint64_t start, uint64_t end, uint32_t &ns)
{
	const uint64_t ticks = (end - start) & kQTimerMask;

	if (start == 0u || end == 0u || ticks > kMaximumDiagnosticTicks) {
		return false;
	}

	const uint64_t converted = (ticks * UINT64_C(625) + 6u) / 12u;
	ns = converted > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(converted);
	return true;
}

static void diagnostic_observe(const zql_odometry_slot_t &slot,
			       uint64_t wake_ticks, uint64_t publish_ticks)
{
	if (slot.reserved[ZQL_DIAGNOSTIC_MAGIC_WORD] != ZQL_DIAGNOSTIC_MAGIC) {
		g_stats.diagnostic_invalid.fetch_add(1u);
		return;
	}

	const uint64_t producer_ticks = diagnostic_slot_qtimer(slot);
	uint32_t hop_ns = 0u;
	uint32_t wake_ns = 0u;
	uint32_t tail_ns = 0u;

	if (!diagnostic_delta_ns(producer_ticks, publish_ticks, hop_ns) ||
	    !diagnostic_delta_ns(producer_ticks, wake_ticks, wake_ns) ||
	    !diagnostic_delta_ns(wake_ticks, publish_ticks, tail_ns)) {
		g_stats.diagnostic_invalid.fetch_add(1u);
		return;
	}

	g_stats.diagnostic_samples.fetch_add(1u);
	g_stats.hop_last_ns.store(hop_ns);
	g_stats.wake_last_ns.store(wake_ns);
	g_stats.uorb_tail_last_ns.store(tail_ns);
	uint32_t minimum = g_stats.hop_min_ns.load();

	while ((minimum == 0u || hop_ns < minimum) &&
	       !g_stats.hop_min_ns.compare_exchange(&minimum, hop_ns)) {}

	update_max(g_stats.hop_max_ns, hop_ns);
	update_max(g_stats.wake_max_ns, wake_ns);
	update_max(g_stats.uorb_tail_max_ns, tail_ns);
}
#endif

struct Transport {
	zql_transport_endpoint_t control;
	zql_rx_binding_t binding;
	zql_smem_endpoint_t smem;
	volatile zql_region_t *region;
	bool bound;
	bool wait_rx_ack;

	Transport() : control{}, binding{}, smem{-1}, region(nullptr), bound(false),
		wait_rx_ack(false)
	{
		control.handle = -1;
	}
};

static void unbind_root(Transport &transport)
{
	pthread_mutex_lock(&g_binding_lock);
	g_bound_wake_signal = -1;
	g_bound_control = nullptr;
	g_bound_wait_rx_ack = false;
	zql_transport_close(&transport.control);
	pthread_mutex_unlock(&g_binding_lock);
	memset(&transport.binding, 0, sizeof(transport.binding));
	transport.bound = false;
	transport.wait_rx_ack = false;
}

static int bind_root(Transport &transport)
{
	int result = zql_transport_open(&transport.control,
					ZQL_TRANSPORT_ROLE_ADSP_CONSUMER, 66u);

	if (result != 0) {
		g_stats.transport_failures.fetch_add(1u);
		return result;
	}

	result = zql_transport_bind_rx(&transport.control, &transport.binding);

	if (result != 0 || transport.binding.struct_bytes != sizeof(transport.binding) ||
	    transport.binding.signal_group < 0 || transport.binding.rx_signal < 0 ||
	    transport.binding.stop_signal < 0 ||
	    transport.binding.rx_signal == transport.binding.stop_signal) {
		g_stats.bind_failures.fetch_add(1u);
		zql_transport_close(&transport.control);
		memset(&transport.binding, 0, sizeof(transport.binding));
		return result != 0 ? result : -1;
	}

	result = zql_transport_refresh(&transport.control);

	if (result != 0 ||
	    !zql_transport_info_valid(&transport.control.info,
				      ZQL_TRANSPORT_ROLE_ADSP_CONSUMER, 66u) ||
	    (transport.control.info.status & ZQL_STATUS_RX_BOUND) == 0u) {
		g_stats.bind_failures.fetch_add(1u);
		zql_transport_close(&transport.control);
		memset(&transport.binding, 0, sizeof(transport.binding));
		return result != 0 ? result : -1;
	}

	pthread_mutex_lock(&g_binding_lock);
	g_bound_wake_signal = transport.binding.rx_signal;
	transport.wait_rx_ack =
		(transport.control.info.status & ZQL_STATUS_WAIT_RX_ACK) != 0u;
	g_bound_control = &transport.control;
	g_bound_wait_rx_ack = transport.wait_rx_ack;
	transport.bound = true;

	if (g_should_exit.load()) {
		if (g_bound_wait_rx_ack) {
			(void)zql_transport_wake_rx(g_bound_control);

		} else {
			(void)zql_signal_set(g_bound_wake_signal);
		}
	}

	pthread_mutex_unlock(&g_binding_lock);
	g_stats.root_binds.fetch_add(1u);
	return 0;
}

static int map_smem(Transport &transport)
{
	void *buffer = nullptr;
	int result = zql_smem_open(&transport.smem);

	if (result == 0) {
		result = zql_smem_map_fixed(&transport.smem, &buffer);
	}

	if (result != 0 || buffer == nullptr) {
		zql_smem_close(&transport.smem);
		g_stats.smem_failures.fetch_add(1u);
		return result != 0 ? result : -1;
	}

	transport.region = static_cast<volatile zql_region_t *>(buffer);
	return 0;
}

static void transport_close(Transport &transport)
{
	if (transport.bound || transport.control.handle >= 0) {
		unbind_root(transport);
	}

	transport.region = nullptr;
	zql_smem_close(&transport.smem);
}

static bool finite_float(float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static bool finite_or_nan(float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & UINT32_C(0x7fffffff)) != UINT32_C(0x7f800000);
}

static bool payload_valid(const zql_odometry_slot_t &slot)
{
	const zql_vehicle_odometry_t &odometry = slot.odometry;
	float quaternion_norm = 0.0f;

	if (slot.filter_state != kFilterRunning || slot.filter_generation == 0u ||
	    (slot.camera_count != 1u && slot.camera_count != 2u) ||
	    (slot.flags & ~ZQL_SAMPLE_DUAL_CAMERA) != 0u ||
	    ((slot.flags & ZQL_SAMPLE_DUAL_CAMERA) != 0u) != (slot.camera_count == 2u) ||
	    odometry.timestamp != 0u || odometry.timestamp_sample == 0u ||
	    odometry.pose_frame != ZQL_POSE_FRAME_FRD ||
	    odometry.velocity_frame != ZQL_VELOCITY_FRAME_FRD ||
	    odometry.quality < 0 || odometry.quality > 100) {
		return false;
	}

	for (unsigned lane = 0u; lane < 4u; ++lane) {
		if (!finite_float(odometry.q[lane])) {
			return false;
		}

		quaternion_norm += odometry.q[lane] * odometry.q[lane];
	}

	if (quaternion_norm < 0.999f || quaternion_norm > 1.001f) {
		return false;
	}

	for (unsigned lane = 0u; lane < 3u; ++lane) {
		if (!finite_float(odometry.position[lane]) ||
		    !finite_float(odometry.velocity[lane]) ||
		    !finite_or_nan(odometry.angular_velocity[lane]) ||
		    !finite_float(odometry.position_variance[lane]) ||
		    !finite_float(odometry.orientation_variance[lane]) ||
		    !finite_float(odometry.velocity_variance[lane]) ||
		    odometry.position_variance[lane] < 0.0f ||
		    odometry.orientation_variance[lane] < 0.0f ||
		    odometry.velocity_variance[lane] < 0.0f) {
			return false;
		}
	}

	return true;
}

static void observe_message_cadence(CadenceState &cadence, const zql_odometry_slot_t &slot,
				    uint64_t publish_us)
{
	/* A producer generation transition starts a new cadence series. There is no
	 * predecessor in the new series against which this message can be scored. */
	if (cadence.valid && cadence.previous_generation == slot.producer_generation &&
	    publish_us >= cadence.previous_publish_us) {
		const uint64_t interval_us = publish_us - cadence.previous_publish_us;
		const uint32_t bounded_interval = interval_us > UINT32_MAX
						  ? UINT32_MAX : static_cast<uint32_t>(interval_us);
		g_stats.last_message_interval_us.store(bounded_interval);
		update_max(g_stats.maximum_message_interval_us, bounded_interval);

		if (interval_us > kTargetMessageIntervalUs) {
			g_stats.target_cadence_misses.fetch_add(1u);
		}

		if (interval_us > kMaximumMessageIntervalUs) {
			g_stats.minimum_rate_violations.fetch_add(1u);
		}
	}

	cadence.previous_publish_us = publish_us;
	cadence.previous_generation = slot.producer_generation;
	cadence.valid = true;
}

static bool publish_sample(uORB::Publication<vehicle_odometry_s> &publication,
			   CadenceState &cadence, const zql_odometry_slot_t &slot,
			   uint64_t wake_ticks)
{
	if (!payload_valid(slot)) {
		g_stats.invalid_samples.fetch_add(1u);
		return false;
	}

	const uint64_t now_us = hrt_absolute_time();
	const uint64_t sample_us = slot.odometry.timestamp_sample;

	if (sample_us > now_us) {
		g_stats.future_rejects.fetch_add(1u);
		return false;
	}

	const uint64_t sample_age_us = now_us - sample_us;
	const uint32_t bounded_age = sample_age_us > UINT32_MAX
				     ? UINT32_MAX : static_cast<uint32_t>(sample_age_us);
	g_stats.last_sample_age_us.store(bounded_age);
	update_max(g_stats.maximum_sample_age_us, bounded_age);

	vehicle_odometry_s odometry{};
	odometry.timestamp = now_us;
	odometry.timestamp_sample = sample_us;
	odometry.pose_frame = slot.odometry.pose_frame;
	odometry.velocity_frame = slot.odometry.velocity_frame;
	memcpy(odometry.position, slot.odometry.position, sizeof(odometry.position));
	memcpy(odometry.q, slot.odometry.q, sizeof(odometry.q));
	memcpy(odometry.velocity, slot.odometry.velocity, sizeof(odometry.velocity));
	memcpy(odometry.angular_velocity, slot.odometry.angular_velocity,
	       sizeof(odometry.angular_velocity));
	memcpy(odometry.position_variance, slot.odometry.position_variance,
	       sizeof(odometry.position_variance));
	memcpy(odometry.orientation_variance, slot.odometry.orientation_variance,
	       sizeof(odometry.orientation_variance));
	memcpy(odometry.velocity_variance, slot.odometry.velocity_variance,
	       sizeof(odometry.velocity_variance));
	odometry.reset_counter = slot.odometry.reset_counter;
	odometry.quality = slot.odometry.quality;
	publication.publish(odometry);
	observe_message_cadence(cadence, slot, now_us);
#if ZQL_DIAGNOSTIC_HOP_TIMING
	diagnostic_observe(slot, wake_ticks, diagnostic_qtimer());
#else
	(void)wake_ticks;
#endif
	g_stats.published.fetch_add(1u);
	g_stats.last_sequence.store(slot.sequence);

	return true;
}

static void update_transport_statistics(Transport &transport)
{
	if (transport.bound && zql_transport_refresh(&transport.control) == 0) {
		g_stats.root_doorbells_without_receiver.store(
			transport.control.info.doorbells_without_receiver);
		g_stats.root_signal_errors.store(transport.control.info.signal_errors);
	}
}

static int drain_latest(Transport &transport, zql_consumer_cursor_t &cursor,
			uORB::Publication<vehicle_odometry_s> &publication,
			CadenceState &cadence)
{
	if (transport.region == nullptr) {
		return -1;
	}

	uint64_t wake_ticks = 0u;
#if ZQL_DIAGNOSTIC_HOP_TIMING
	wake_ticks = diagnostic_qtimer();
#endif
	zql_odometry_slot_t slot{};
	const int result = zql_ring_read_latest(transport.region, &cursor, &slot);
	g_stats.missed_samples.store(cursor.missed_samples);
	g_stats.ring_invalid_reads.store(cursor.invalid_reads);
	g_stats.producer_restarts.store(cursor.producer_restarts);
	g_stats.producer_generation.store(cursor.producer_generation);
	g_stats.producer_trigger_failures.store(transport.region->header.trigger_failures);
	g_stats.producer_rejected_samples.store(transport.region->header.rejected_samples);

	if (result > 0) {
		(void)publish_sample(publication, cadence, slot, wake_ticks);
	}

	return result;
}

static int worker(int, char *[])
{
	Transport transport;
	zql_consumer_cursor_t cursor{};
	uORB::Publication<vehicle_odometry_s> publication{ORB_ID(vehicle_visual_odometry)};
	CadenceState cadence{};
	uint64_t next_root_retry_us = 0u;
	uint64_t next_smem_retry_us = 0u;
	uint32_t retry_log_divider = 0u;
	g_running.store(true);

	while (!g_should_exit.load()) {
		const uint64_t now_us = hrt_absolute_time();

		if (transport.region == nullptr && now_us >= next_smem_retry_us) {
			const int result = map_smem(transport);
			next_smem_retry_us = now_us + kSmemRetryUs;

			if (result != 0 && (retry_log_divider++ % 10u) == 0u) {
				PX4_WARN("fixed SMEM unavailable (%d); retrying", result);
			}
		}

		if (transport.region != nullptr && !transport.bound &&
		    now_us >= next_root_retry_us) {
			const int result = bind_root(transport);
			next_root_retry_us = now_us + kRootRetryUs;

			if (result == 0) {
				/* Consume a sample committed before BIND_RX before blocking. */
				drain_latest(transport, cursor, publication, cadence);
				continue;
			}

			if ((retry_log_divider++ % 10u) == 0u) {
				PX4_WARN("ADSP doorbell transport unavailable (%d); publication disabled", result);
			}
		}

		if (g_should_exit.load()) {
			break;
		}

		if (transport.bound) {
			/* The matched root image blocks and acknowledges the IPCC-driven RX
			 * signal inside one QDI invocation.  An older ABI-1 root advertises no
			 * capability bit and retains a blocking WAIT+CLEAR compatibility path;
			 * neither path polls SMEM or signal state. */
			int wait_result;

			if (transport.wait_rx_ack) {
				g_stats.fused_waits.fetch_add(1u);
				wait_result = zql_transport_wait_rx_ack(&transport.control);

			} else {
				g_stats.legacy_waits.fetch_add(1u);
				wait_result = zql_signal_wait(transport.binding.rx_signal);

				if (wait_result == 0 &&
				    zql_signal_clear(transport.binding.rx_signal) < 0) {
					g_stats.signal_failures.fetch_add(1u);
					unbind_root(transport);
					next_root_retry_us = hrt_absolute_time() + kRootRetryUs;
					continue;
				}
			}

			/* A remote QDI signal wait returns QURT_EOK (zero) on a wake and
			 * positive QURT_ECANCEL on PD teardown.  The fused root command also
			 * normalizes its valid signal word to zero. */
			if (wait_result != 0) {
				g_stats.wait_failures.fetch_add(1u);
				unbind_root(transport);
				next_root_retry_us = hrt_absolute_time() + kRootRetryUs;
				continue;
			}

			if (g_should_exit.load()) {
				break;
			}

			g_stats.doorbells.fetch_add(1u);

			if (drain_latest(transport, cursor, publication, cadence) == 0) {
				g_stats.spurious_wakes.fetch_add(1u);
			}

		} else {
			/* Fail closed when the root doorbell path is absent.  This timed wait
			 * only schedules the next control-plane bind/map attempt; it never
			 * reads SMEM and is not a data-plane polling fallback. */
			const uint64_t control_now_us = hrt_absolute_time();
			const uint64_t retry_at_us = transport.region == nullptr
						     ? next_smem_retry_us : next_root_retry_us;
			const unsigned long long wait_us = retry_at_us > control_now_us
							 ? retry_at_us - control_now_us : 1u;
			unsigned signals = 0u;
			(void)qurt_signal_wait_timed(&g_control_signal, kControlStopSignal,
					     QURT_SIGNAL_ATTR_WAIT_ANY, &signals,
					     wait_us);
			qurt_signal_clear(&g_control_signal, kControlStopSignal);

			if (g_should_exit.load() || (signals & kControlStopSignal) != 0u) {
				break;
			}

			g_stats.recovery_waits.fetch_add(1u);
		}
	}

	update_transport_statistics(transport);
	transport_close(transport);
	g_running.store(false);
	return 0;
}

static int start(char *const argv[])
{
	if (g_running.load() || g_task >= 0) {
		PX4_WARN("already running");
		return 0;
	}

	g_stats = Statistics{};

	if (!g_control_signal_initialized) {
		qurt_signal_init(&g_control_signal);
		g_control_signal_initialized = true;
	}

	qurt_signal_clear(&g_control_signal, kControlStopSignal);
	g_should_exit.store(false);
	g_task = px4_task_spawn_cmd("zql_rx", SCHED_DEFAULT,
				    SCHED_PRIORITY_ESTIMATOR, 4096, worker, argv);

	if (g_task < 0) {
		PX4_ERR("receiver start failed");
		return -1;
	}

	return 0;
}

static int stop()
{
	if (g_task < 0) {
		return 0;
	}

	g_should_exit.store(true);
	pthread_mutex_lock(&g_binding_lock);

	if (g_bound_wait_rx_ack && g_bound_control != nullptr) {
		(void)zql_transport_wake_rx(g_bound_control);

	} else if (g_bound_wake_signal >= 0) {
		(void)zql_signal_set(g_bound_wake_signal);
	}

	pthread_mutex_unlock(&g_binding_lock);

	if (g_control_signal_initialized) {
		qurt_signal_set(&g_control_signal, kControlStopSignal);
	}

	const int result = px4_task_join(g_task);
	g_task = -1;
	return result;
}

static void status()
{
	pthread_mutex_lock(&g_binding_lock);

	if (g_bound_control != nullptr &&
	    zql_transport_refresh(g_bound_control) == 0) {
		g_stats.root_doorbells_without_receiver.store(
			g_bound_control->info.doorbells_without_receiver);
		g_stats.root_signal_errors.store(g_bound_control->info.signal_errors);
	}

	pthread_mutex_unlock(&g_binding_lock);
	PX4_INFO("%s: root_binds=%u doorbells=%u published=%u",
		 g_running.load() ? "running" : "stopped",
		 (unsigned)g_stats.root_binds.load(),
		 (unsigned)g_stats.doorbells.load(),
		 (unsigned)g_stats.published.load());
	PX4_INFO("invalid=%u future=%u last_seq=%u",
		 (unsigned)g_stats.invalid_samples.load(),
		 (unsigned)g_stats.future_rejects.load(),
		 (unsigned)g_stats.last_sequence.load());
	PX4_INFO("sample_age_us=%u max=%u generation=%u restarts=%u",
		 (unsigned)g_stats.last_sample_age_us.load(),
		 (unsigned)g_stats.maximum_sample_age_us.load(),
		 (unsigned)g_stats.producer_generation.load(),
		 (unsigned)g_stats.producer_restarts.load());
	PX4_INFO("message_interval_us=%u max=%u late_30hz=%u below_20hz=%u",
		 (unsigned)g_stats.last_message_interval_us.load(),
		 (unsigned)g_stats.maximum_message_interval_us.load(),
		 (unsigned)g_stats.target_cadence_misses.load(),
		 (unsigned)g_stats.minimum_rate_violations.load());
	PX4_INFO("missed=%u ring_invalid=%u trigger_fail=%u producer_reject=%u",
		 (unsigned)g_stats.missed_samples.load(),
		 (unsigned)g_stats.ring_invalid_reads.load(),
		 (unsigned)g_stats.producer_trigger_failures.load(),
		 (unsigned)g_stats.producer_rejected_samples.load());
	PX4_INFO("smem_fail=%u transport_fail=%u bind_fail=%u wait_fail=%u signal_fail=%u",
		 (unsigned)g_stats.smem_failures.load(),
		 (unsigned)g_stats.transport_failures.load(),
		 (unsigned)g_stats.bind_failures.load(),
		 (unsigned)g_stats.wait_failures.load(),
		 (unsigned)g_stats.signal_failures.load());
	PX4_INFO("spurious=%u fused_waits=%u legacy_waits=%u recovery_waits=%u",
		 (unsigned)g_stats.spurious_wakes.load(),
		 (unsigned)g_stats.fused_waits.load(),
		 (unsigned)g_stats.legacy_waits.load(),
		 (unsigned)g_stats.recovery_waits.load());
	PX4_INFO("data_polls=0 root_no_rx=%u root_sigerr=%u",
		 (unsigned)g_stats.root_doorbells_without_receiver.load(),
		 (unsigned)g_stats.root_signal_errors.load());
#if ZQL_DIAGNOSTIC_HOP_TIMING
	PX4_INFO("DIAGNOSTIC samples=%u invalid=%u hop_ns=%u min=%u max=%u",
		 (unsigned)g_stats.diagnostic_samples.load(),
		 (unsigned)g_stats.diagnostic_invalid.load(),
		 (unsigned)g_stats.hop_last_ns.load(),
		 (unsigned)g_stats.hop_min_ns.load(),
		 (unsigned)g_stats.hop_max_ns.load());
	PX4_INFO("DIAGNOSTIC wake_ns=%u max=%u uorb_tail_ns=%u max=%u",
		 (unsigned)g_stats.wake_last_ns.load(),
		 (unsigned)g_stats.wake_max_ns.load(),
		 (unsigned)g_stats.uorb_tail_last_ns.load(),
		 (unsigned)g_stats.uorb_tail_max_ns.load());
#endif
}

} // namespace zvins_qsh_link

int zvins_qsh_link_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: zvins_qsh_link {start|stop|status}");
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		return zvins_qsh_link::start(argv);
	}

	if (!strcmp(argv[1], "stop")) {
		return zvins_qsh_link::stop();
	}

	if (!strcmp(argv[1], "status")) {
		zvins_qsh_link::status();
		return 0;
	}

	PX4_INFO("usage: zvins_qsh_link {start|stop|status}");
	return -1;
}
