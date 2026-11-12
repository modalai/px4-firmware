/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dsp_link
{

// Temporary odometry-specific CQL ABI 3. The generic dsp_link API delivers the
// complete packet; this application owns its interpretation. Preserve the wire
// layout until the producer and consumer move to a generic message format.
static constexpr uint32_t PacketMagic = UINT32_C(0x334f565a);
static constexpr uint16_t PacketVersion = 3;
static constexpr uint16_t FilterRunning = 2;
static constexpr uint8_t DualCamera = 1;
static constexpr uint8_t FrameFRD = 2;
static constexpr uint32_t TestSourceSession = UINT32_C(0x54534554);

struct alignas(8) OdometryPayload {
	uint64_t timestamp;
	uint64_t timestamp_sample; // Shared QTimer domain, microseconds.
	uint8_t pose_frame;
	uint8_t reserved_pose[3];
	float position[3];
	float q[4]; // Hamilton (w, x, y, z), local FRD.
	uint8_t velocity_frame;
	uint8_t reserved_velocity[3];
	float velocity[3];
	float angular_velocity[3];
	float position_variance[3];
	float orientation_variance[3];
	float velocity_variance[3];
	uint8_t reset_counter;
	int8_t quality;
	uint8_t reserved_tail[6];
};

struct alignas(128) OdometryPacket {
	uint32_t magic;
	uint16_t abi_version;
	uint16_t struct_bytes;
	uint32_t producer_generation;
	uint32_t sequence;
	uint32_t filter_generation;
	uint32_t source_session;
	uint16_t filter_state;
	uint8_t camera_count;
	uint8_t flags;
	uint32_t frame_id;
	OdometryPayload odometry;
	uint32_t reserved[25];
	uint32_t commit_sequence;
};

static_assert(sizeof(float) == 4, "wire float size");
static_assert(sizeof(OdometryPayload) == 120, "wire odometry size");
static_assert(offsetof(OdometryPayload, timestamp_sample) == 8, "wire sample time offset");
static_assert(offsetof(OdometryPayload, position) == 20, "wire position offset");
static_assert(offsetof(OdometryPayload, q) == 32, "wire quaternion offset");
static_assert(offsetof(OdometryPayload, quality) == 113, "wire quality offset");
static_assert(sizeof(OdometryPacket) == 256, "wire packet size");
static_assert(offsetof(OdometryPacket, odometry) == 32, "wire payload offset");
static_assert(offsetof(OdometryPacket, commit_sequence) == 252, "wire commit offset");

} // namespace dsp_link
