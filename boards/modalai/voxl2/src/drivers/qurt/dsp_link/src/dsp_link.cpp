/* SPDX-License-Identifier: BSD-3-Clause */
/* Receive, validate, and process CDSP messages through the ADSP dsp_link API. */
#include "dsp_link.h"
#include "odometry_packet.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_odometry.h>

#include <inttypes.h>
#include <pthread.h>
#include <string.h>

extern "C" __EXPORT int dsp_link_main(int argc, char *argv[]);

namespace dsp_link
{

static_assert(vehicle_odometry_s::POSE_FRAME_FRD == FrameFRD, "wire/PX4 pose frame changed");
static_assert(vehicle_odometry_s::VELOCITY_FRAME_FRD == FrameFRD, "wire/PX4 velocity frame changed");

#define CHECK_FIELD_SIZE(field) \
	static_assert(sizeof(((vehicle_odometry_s *)nullptr)->field) == sizeof(((OdometryPayload *)nullptr)->field), \
		      "wire/PX4 " #field " size changed")
CHECK_FIELD_SIZE(position);
CHECK_FIELD_SIZE(q);
CHECK_FIELD_SIZE(velocity);
CHECK_FIELD_SIZE(angular_velocity);
CHECK_FIELD_SIZE(position_variance);
CHECK_FIELD_SIZE(orientation_variance);
CHECK_FIELD_SIZE(velocity_variance);
#undef CHECK_FIELD_SIZE

static constexpr uint64_t StaleIntervalUs = 2000000; // Status only; not a publication gate.

enum class Reject : unsigned {
	None,
	Length,
	Envelope,
	Sequence,
	Time,
	Filter,
	Frame,
	Values,
	Publication,
	Count
};

static const char *reject_name(Reject reason)
{
	switch (reason) {
	case Reject::None: return "none";

	case Reject::Length: return "packet length";

	case Reject::Envelope: return "packet envelope";

	case Reject::Sequence: return "duplicate/backward sequence";

	case Reject::Time: return "zero/future/backward sample time";

	case Reject::Filter: return "filter/camera state";

	case Reject::Frame: return "unsupported frame";

	case Reject::Values: return "invalid odometry values";

	case Reject::Publication: return "uORB publication failed";

	case Reject::Count: break;
	}

	return "unknown";
}

class LockGuard
{
public:
	explicit LockGuard(pthread_mutex_t &mutex) : _mutex(mutex) { pthread_mutex_lock(&_mutex); }
	~LockGuard() { pthread_mutex_unlock(&_mutex); }
	LockGuard(const LockGuard &) = delete;
	LockGuard &operator=(const LockGuard &) = delete;
private:
	pthread_mutex_t &_mutex;
};

struct Statistics {
	uint32_t received{};
	uint32_t published{};
	uint32_t rejected[static_cast<unsigned>(Reject::Count)] {};
	uint32_t gaps{};
	uint32_t restarts{};
	uint32_t test_packets{};
	uint32_t generation{};
	uint32_t sequence{};
	uint32_t age_samples{};
	uint64_t first_receive_us{};
	uint64_t last_receive_us{};
	uint64_t last_publish_us{};
	uint64_t last_sample_us{};
	uint64_t rx_interval_us{};
	uint64_t tx_interval_us{};
	uint64_t age_last_us{};
	uint64_t age_min_us{};
	uint64_t age_max_us{};
	uint64_t age_total_us{};
	Reject last_reject{Reject::None};
	bool have_packet{};
	OdometryPacket last_packet{};
};

// Commands serialize lifecycle changes. The callback only takes stats_lock,
// so close can join it while command_lock is held. Keep callback state alive
// if a close fails; a later stop retries the same descriptor.
static pthread_mutex_t command_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t stats_lock = PTHREAD_MUTEX_INITIALIZER;
static Statistics stats;
static int link_fd = -1;
static int last_api_error;
static bool callback_registered;
static orb_advert_t publication;

static void reject(Reject reason)
{
	++stats.rejected[static_cast<unsigned>(reason)];
	stats.last_reject = reason;
}

// Bit checks retain validation even if the surrounding build enables fast math.
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

static bool values_valid(const OdometryPayload &odometry)
{
	if (odometry.quality < 0 || odometry.quality > 100) {
		return false;
	}

	float norm = 0.f;

	for (unsigned i = 0; i < 4; ++i) {
		if (!finite_float(odometry.q[i])) {
			return false;
		}

		norm += odometry.q[i] * odometry.q[i];
	}

	if (norm < 0.999f || norm > 1.001f) {
		return false;
	}

	for (unsigned i = 0; i < 3; ++i) {
		if (!finite_float(odometry.position[i]) || !finite_float(odometry.velocity[i])
		    || !finite_or_nan(odometry.angular_velocity[i])
		    || !finite_float(odometry.position_variance[i]) || odometry.position_variance[i] < 0.f
		    || !finite_float(odometry.orientation_variance[i]) || odometry.orientation_variance[i] < 0.f
		    || !finite_float(odometry.velocity_variance[i]) || odometry.velocity_variance[i] < 0.f) {
			return false;
		}
	}

	return true;
}

// Called by receive with stats_lock held. The current backend delivers one
// complete odometry packet per callback.
static void process_odometry(const void *data, size_t length, uint64_t now)
{
	if (data == nullptr || length != sizeof(OdometryPacket)) {
		reject(Reject::Length);
		return;
	}

	// The API owns data; copy it before returning from this callback. Do not
	// assume its address satisfies the packet's 128-byte alignment requirement.
	OdometryPacket packet;
	memcpy(&packet, data, sizeof(packet));

	if (packet.magic != PacketMagic || packet.abi_version != PacketVersion
	    || packet.struct_bytes != sizeof(packet) || packet.producer_generation == 0
	    || packet.sequence == 0 || packet.commit_sequence != packet.sequence) {
		reject(Reject::Envelope);
		return;
	}

	stats.last_packet = packet;
	stats.have_packet = true;

	if (packet.source_session == TestSourceSession) {
		++stats.test_packets;
	}

	if (stats.generation == packet.producer_generation) {
		if (packet.sequence <= stats.sequence) {
			reject(Reject::Sequence);
			return;
		}

		stats.gaps += packet.sequence - stats.sequence - 1;

	} else {
		if (stats.generation != 0) {
			++stats.restarts;
		}

		stats.last_sample_us = 0;
		stats.tx_interval_us = 0;
	}

	stats.generation = packet.producer_generation;
	stats.sequence = packet.sequence;
	const OdometryPayload &source = packet.odometry;

	// CDSP and this QuRT HRT implementation use the same QTimer microsecond
	// domain. Preserve sample time; stamp publication time locally below.
	if (source.timestamp != 0 || source.timestamp_sample == 0 || source.timestamp_sample > now
	    || (stats.last_sample_us != 0 && source.timestamp_sample <= stats.last_sample_us)) {
		reject(Reject::Time);
		return;
	}

	if (stats.last_sample_us != 0) {
		stats.tx_interval_us = source.timestamp_sample - stats.last_sample_us;
	}

	stats.last_sample_us = source.timestamp_sample;
	stats.age_last_us = now - source.timestamp_sample;

	if (stats.age_samples == 0 || stats.age_last_us < stats.age_min_us) {
		stats.age_min_us = stats.age_last_us;
	}

	if (stats.age_last_us > stats.age_max_us) {
		stats.age_max_us = stats.age_last_us;
	}

	stats.age_total_us += stats.age_last_us;
	++stats.age_samples;

	if (packet.filter_state != FilterRunning || packet.filter_generation == 0
	    || (packet.camera_count != 1 && packet.camera_count != 2)
	    || (packet.flags & ~DualCamera) != 0
	    || ((packet.flags & DualCamera) != 0) != (packet.camera_count == 2)) {
		reject(Reject::Filter);
		return;
	}

	if (source.pose_frame != FrameFRD || source.velocity_frame != FrameFRD) {
		reject(Reject::Frame);
		return;
	}

	if (!values_valid(source)) {
		reject(Reject::Values);
		return;
	}

	vehicle_odometry_s odometry{};
	odometry.timestamp = hrt_absolute_time();
	odometry.timestamp_sample = source.timestamp_sample;
	odometry.pose_frame = source.pose_frame;
	odometry.velocity_frame = source.velocity_frame;
	memcpy(odometry.position, source.position, sizeof(odometry.position));
	memcpy(odometry.q, source.q, sizeof(odometry.q));
	memcpy(odometry.velocity, source.velocity, sizeof(odometry.velocity));
	memcpy(odometry.angular_velocity, source.angular_velocity, sizeof(odometry.angular_velocity));
	memcpy(odometry.position_variance, source.position_variance, sizeof(odometry.position_variance));
	memcpy(odometry.orientation_variance, source.orientation_variance, sizeof(odometry.orientation_variance));
	memcpy(odometry.velocity_variance, source.velocity_variance, sizeof(odometry.velocity_variance));
	odometry.reset_counter = source.reset_counter;
	odometry.quality = source.quality;
	bool published;

	if (publication == nullptr) {
		publication = orb_advertise(ORB_ID(vehicle_visual_odometry), &odometry);
		published = publication != nullptr;

	} else {
		published = orb_publish(ORB_ID(vehicle_visual_odometry), publication, &odometry) == PX4_OK;
	}

	if (!published) {
		reject(Reject::Publication);
		return;
	}

	++stats.published;
	stats.last_publish_us = odometry.timestamp;
	stats.last_reject = Reject::None;
}

static void receive(const void *data, size_t length, void *context)
{
	(void)context;
	const uint64_t now = hrt_absolute_time();
	LockGuard guard(stats_lock);
	++stats.received;

	if (stats.received == 1) {
		stats.first_receive_us = now;

	} else {
		stats.rx_interval_us = now - stats.last_receive_us;
	}

	stats.last_receive_us = now;

	// The current wire contract has only odometry. Add message-type dispatch
	// here when the producer and transport support a typed message envelope.
	process_odometry(data, length, now);
}

// Called with command_lock held, after all callbacks have finished.
static void unadvertise()
{
	if (publication != nullptr) {
		orb_unadvertise(publication);
		publication = nullptr;
	}
}

static int start()
{
	LockGuard guard(command_lock);

	if (link_fd >= 0) {
		if (callback_registered) {
			PX4_INFO("already running");
			return PX4_OK;
		}

		PX4_ERR("descriptor retained after an error; run stop before start");
		return PX4_ERROR;
	}

	{
		LockGuard stats_guard(stats_lock);
		stats = {};
	}

	link_fd = dsp_link_open();

	if (link_fd < 0) {
		last_api_error = link_fd;
		PX4_ERR("dsp_link_open failed: %d", link_fd);
		return PX4_ERROR;
	}

	last_api_error = dsp_link_register_rx_callback(link_fd, receive, nullptr);

	if (last_api_error != 0) {
		PX4_ERR("register callback failed: %d", last_api_error);
		const int close_result = dsp_link_close(link_fd);

		if (close_result == 0) {
			link_fd = -1;
			unadvertise();

		} else {
			last_api_error = close_result;
			PX4_ERR("close failed: %d; fd=%d retained", close_result, link_fd);
		}

		return PX4_ERROR;
	}

	callback_registered = true;
	PX4_INFO("started; output topic: vehicle_visual_odometry");
	return PX4_OK;
}

static int stop()
{
	LockGuard guard(command_lock);

	if (link_fd < 0) {
		return PX4_OK;
	}

	last_api_error = dsp_link_close(link_fd);

	if (last_api_error != 0) {
		PX4_ERR("close failed: %d; fd=%d retained, retry stop", last_api_error, link_fd);
		return PX4_ERROR;
	}

	link_fd = -1;
	callback_registered = false;
	unadvertise();
	PX4_INFO("stopped");
	return PX4_OK;
}

static int status()
{
	Statistics snapshot;
	int fd;
	int api_error;
	bool active;

	{
		LockGuard command_guard(command_lock);
		fd = link_fd;
		api_error = last_api_error;
		active = callback_registered;
		LockGuard stats_guard(stats_lock);
		snapshot = stats;
	}

	const uint64_t now = hrt_absolute_time();
	const char *state = "stopped";

	if (fd >= 0) {
		if (!active) {
			state = "cleanup required (run stop)";

		} else if (snapshot.received == 0) {
			state = "waiting for CDSP";

		} else if (now - snapshot.last_receive_us > StaleIntervalUs) {
			state = "stale (no recent callbacks)";

		} else if (snapshot.published == 0 || now - snapshot.last_publish_us > StaleIntervalUs) {
			state = "receiving, no recent publication";

		} else {
			state = "publishing";
		}
	}

	PX4_INFO("%s; fd=%d last_api_error=%d", state, fd, api_error);
	PX4_INFO("received=%" PRIu32 " published=%" PRIu32 " gaps=%" PRIu32 " restarts=%" PRIu32,
		 snapshot.received, snapshot.published, snapshot.gaps, snapshot.restarts);
	PX4_INFO("rejected: length=%" PRIu32 " envelope=%" PRIu32 " sequence=%" PRIu32 " time=%" PRIu32,
		 snapshot.rejected[static_cast<unsigned>(Reject::Length)], snapshot.rejected[static_cast<unsigned>(Reject::Envelope)],
		 snapshot.rejected[static_cast<unsigned>(Reject::Sequence)], snapshot.rejected[static_cast<unsigned>(Reject::Time)]);
	PX4_INFO("rejected: filter=%" PRIu32 " frame=%" PRIu32 " values=%" PRIu32 " publication=%" PRIu32,
		 snapshot.rejected[static_cast<unsigned>(Reject::Filter)], snapshot.rejected[static_cast<unsigned>(Reject::Frame)],
		 snapshot.rejected[static_cast<unsigned>(Reject::Values)], snapshot.rejected[static_cast<unsigned>(Reject::Publication)]);
	PX4_INFO("last rejection=%s; TEST packets=%" PRIu32, reject_name(snapshot.last_reject), snapshot.test_packets);

	if (snapshot.received != 0) {
		const uint64_t duration = snapshot.last_receive_us - snapshot.first_receive_us;
		const double rate = duration > 0 ? (snapshot.received - 1) * 1000000.0 / duration : 0.0;
		PX4_INFO("rx_mean_hz=%.2f last_rx_ago_us=%" PRIu64 " rx_dt_us=%" PRIu64 " tx_dt_us=%" PRIu64,
			 rate, now - snapshot.last_receive_us, snapshot.rx_interval_us, snapshot.tx_interval_us);
	}

	if (snapshot.published != 0) {
		PX4_INFO("last_publication_ago_us=%" PRIu64, now - snapshot.last_publish_us);
	}

	if (snapshot.age_samples != 0) {
		PX4_INFO("sample_to_callback_us: last=%" PRIu64 " min=%" PRIu64 " avg=%" PRIu64 " max=%" PRIu64,
			 snapshot.age_last_us, snapshot.age_min_us, snapshot.age_total_us / snapshot.age_samples, snapshot.age_max_us);
	}

	if (snapshot.have_packet) {
		const OdometryPacket &packet = snapshot.last_packet;
		PX4_INFO("last packet: gen=%" PRIu32 " seq=%" PRIu32 " filter_gen=%" PRIu32 " session=0x%08" PRIx32,
			 packet.producer_generation, packet.sequence, packet.filter_generation, packet.source_session);
		PX4_INFO("filter=%u cameras=%u flags=0x%x frame_id=%" PRIu32,
			 (unsigned)packet.filter_state, (unsigned)packet.camera_count, (unsigned)packet.flags, packet.frame_id);
		PX4_INFO("pose_frame=%u velocity_frame=%u quality=%d reset_counter=%u",
			 (unsigned)packet.odometry.pose_frame, (unsigned)packet.odometry.velocity_frame,
			 (int)packet.odometry.quality, (unsigned)packet.odometry.reset_counter);
		PX4_INFO("position=(%.3f, %.3f, %.3f)", (double)packet.odometry.position[0],
			 (double)packet.odometry.position[1], (double)packet.odometry.position[2]);
	}

	return PX4_OK;
}

} // namespace dsp_link

int dsp_link_main(int argc, char *argv[])
{
	if (argc == 2) {
		if (!strcmp(argv[1], "start")) {
			return dsp_link::start();
		}

		if (!strcmp(argv[1], "stop")) {
			return dsp_link::stop();
		}

		if (!strcmp(argv[1], "status")) {
			return dsp_link::status();
		}
	}

	PX4_INFO("usage: dsp_link {start|stop|status}");
	return PX4_ERROR;
}
