/* SPDX-License-Identifier: BSD-3-Clause */
/** Fixed CDSP v68 -> ADSP v66 VehicleOdometry transport ABI. */
#ifndef ZVINS_QSH_LINK_WIRE_H
#define ZVINS_QSH_LINK_WIRE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZQL_REGION_MAGIC UINT32_C(0x334c515a) /* "ZQL3" little-endian */
#define ZQL_SLOT_MAGIC UINT32_C(0x334f565a)   /* "ZVO3" little-endian */
#define ZQL_ABI_VERSION UINT16_C(3)
#define ZQL_CONTRACT_FINGERPRINT UINT32_C(0xac6c7156)

#define ZQL_HEADER_BYTES UINT16_C(128)
#define ZQL_SLOT_BYTES UINT16_C(256)
#define ZQL_SLOT_COUNT UINT16_C(32)
#define ZQL_REGION_BYTES \
  ((uint32_t)ZQL_HEADER_BYTES + (uint32_t)ZQL_SLOT_COUNT * \
                                    (uint32_t)ZQL_SLOT_BYTES)

#define ZQL_SMEM_ID_VENDOR2 UINT16_C(136)
#define ZQL_SMEM_INVALID_HOST UINT16_C(0xffff)
#define ZQL_TRANSPORT_DEVICE_NAME "/dev/zql_transport"
#define ZQL_TRANSPORT_MAGIC UINT32_C(0x544c515a) /* "ZQLT" */
#define ZQL_TRANSPORT_ABI_VERSION UINT16_C(1)

#define ZQL_IPCC_PROTOCOL UINT32_C(0)
#define ZQL_IPCC_ADSP_CLIENT UINT32_C(3)
#define ZQL_IPCC_CDSP_CLIENT UINT32_C(6)
#define ZQL_IPCC_SIGNAL UINT32_C(3)

/* Optional precommit timing occupies the slot envelope, never the uORB-shaped
 * payload.  Production builds leave every reserved word zero. */
#define ZQL_DIAGNOSTIC_MAGIC UINT32_C(0x47414944) /* "DIAG" */
#define ZQL_DIAGNOSTIC_QTIMER_LO_WORD 0u
#define ZQL_DIAGNOSTIC_QTIMER_HI_WORD 1u
#define ZQL_DIAGNOSTIC_MAGIC_WORD 2u

enum zql_producer_state {
  ZQL_PRODUCER_OFFLINE = 0,
  ZQL_PRODUCER_INITIALIZING = 1,
  ZQL_PRODUCER_READY = 2
};

enum zql_sample_flags {
  ZQL_SAMPLE_DUAL_CAMERA = 1u << 0
};

enum zql_odometry_frame {
  ZQL_POSE_FRAME_FRD = 2,
  ZQL_VELOCITY_FRAME_FRD = 2
};

/**
 * Float-only mirror of the fields in PX4 VehicleOdometry.msg.
 *
 * CDSP leaves timestamp zero and supplies timestamp_sample in the global
 * QTimer microsecond domain.  ADSP fills timestamp from hrt_absolute_time()
 * immediately before uORB publication.  Explicit padding makes this layout
 * compiler- and language-independent; no PX4 generated header is imported by
 * the CDSP image.
 */
typedef struct __attribute__((aligned(8))) zql_vehicle_odometry {
  uint64_t timestamp;
  uint64_t timestamp_sample;
  uint8_t pose_frame;
  uint8_t reserved_pose[3];
  float position[3];
  float q[4];                 /* Hamilton w, x, y, z */
  uint8_t velocity_frame;
  uint8_t reserved_velocity[3];
  float velocity[3];
  float angular_velocity[3]; /* body FRD; NaN when unavailable */
  float position_variance[3];
  float orientation_variance[3];
  float velocity_variance[3];
  uint8_t reset_counter;
  int8_t quality;
  uint8_t reserved_tail[6];
} zql_vehicle_odometry_t;

typedef struct __attribute__((aligned(128))) zql_region_header {
  uint32_t magic;
  uint16_t abi_version;
  uint16_t header_bytes;
  uint32_t region_bytes;
  uint32_t contract_fingerprint;
  uint16_t slot_count;
  uint16_t slot_bytes;
  volatile uint32_t producer_generation;
  volatile uint32_t producer_state;
  volatile uint32_t write_sequence;
  volatile uint32_t publish_count;
  volatile uint32_t trigger_failures;
  volatile uint32_t rejected_samples;
  volatile uint32_t restart_count;
  uint32_t reserved[20];
} zql_region_header_t;

/** One self-contained sample. commit_sequence is always written last. */
typedef struct __attribute__((aligned(128))) zql_odometry_slot {
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
  zql_vehicle_odometry_t odometry;
  uint32_t reserved[25];
  volatile uint32_t commit_sequence;
} zql_odometry_slot_t;

typedef struct __attribute__((aligned(128))) zql_region {
  zql_region_header_t header;
  zql_odometry_slot_t slot[ZQL_SLOT_COUNT];
} zql_region_t;

typedef struct zql_consumer_cursor {
  uint32_t producer_generation;
  uint32_t last_sequence;
  uint32_t missed_samples;
  uint32_t invalid_reads;
  uint32_t producer_restarts;
} zql_consumer_cursor_t;

void zql_region_initialize(volatile zql_region_t *region,
                           uint32_t producer_generation,
                           uint32_t restart_count);
void zql_region_mark_offline(volatile zql_region_t *region);
int zql_region_header_valid(const volatile zql_region_t *region);
int zql_ring_publish(volatile zql_region_t *region,
                     const zql_odometry_slot_t *sample);
int zql_ring_read_latest(const volatile zql_region_t *region,
                         zql_consumer_cursor_t *cursor,
                         zql_odometry_slot_t *sample);

#define ZQL_WIRE_ASSERT(name, condition) \
  typedef char zql_wire_assert_##name[(condition) ? 1 : -1]
ZQL_WIRE_ASSERT(payload_size, sizeof(zql_vehicle_odometry_t) == 120u);
ZQL_WIRE_ASSERT(payload_timestamp_offset,
                offsetof(zql_vehicle_odometry_t, timestamp) == 0u);
ZQL_WIRE_ASSERT(payload_sample_offset,
                offsetof(zql_vehicle_odometry_t, timestamp_sample) == 8u);
ZQL_WIRE_ASSERT(payload_pose_frame_offset,
                offsetof(zql_vehicle_odometry_t, pose_frame) == 16u);
ZQL_WIRE_ASSERT(payload_position_offset,
                offsetof(zql_vehicle_odometry_t, position) == 20u);
ZQL_WIRE_ASSERT(payload_q_offset,
                offsetof(zql_vehicle_odometry_t, q) == 32u);
ZQL_WIRE_ASSERT(payload_velocity_frame_offset,
                offsetof(zql_vehicle_odometry_t, velocity_frame) == 48u);
ZQL_WIRE_ASSERT(payload_velocity_offset,
                offsetof(zql_vehicle_odometry_t, velocity) == 52u);
ZQL_WIRE_ASSERT(payload_angular_velocity_offset,
                offsetof(zql_vehicle_odometry_t, angular_velocity) == 64u);
ZQL_WIRE_ASSERT(payload_position_variance_offset,
                offsetof(zql_vehicle_odometry_t, position_variance) == 76u);
ZQL_WIRE_ASSERT(payload_orientation_variance_offset,
                offsetof(zql_vehicle_odometry_t, orientation_variance) == 88u);
ZQL_WIRE_ASSERT(payload_velocity_variance_offset,
                offsetof(zql_vehicle_odometry_t, velocity_variance) == 100u);
ZQL_WIRE_ASSERT(payload_reset_offset,
                offsetof(zql_vehicle_odometry_t, reset_counter) == 112u);
ZQL_WIRE_ASSERT(payload_quality_offset,
                offsetof(zql_vehicle_odometry_t, quality) == 113u);
ZQL_WIRE_ASSERT(header_size, sizeof(zql_region_header_t) == ZQL_HEADER_BYTES);
ZQL_WIRE_ASSERT(slot_size, sizeof(zql_odometry_slot_t) == ZQL_SLOT_BYTES);
ZQL_WIRE_ASSERT(region_size, sizeof(zql_region_t) == ZQL_REGION_BYTES);
ZQL_WIRE_ASSERT(slot_count_power_of_two,
                (ZQL_SLOT_COUNT & (ZQL_SLOT_COUNT - 1u)) == 0u);
ZQL_WIRE_ASSERT(region_slot_offset,
                offsetof(zql_region_t, slot) == ZQL_HEADER_BYTES);
ZQL_WIRE_ASSERT(slot_payload_offset,
                offsetof(zql_odometry_slot_t, odometry) == 32u);
ZQL_WIRE_ASSERT(slot_commit_offset,
                offsetof(zql_odometry_slot_t, commit_sequence) == 252u);
#undef ZQL_WIRE_ASSERT

#ifdef __cplusplus
}
#endif
#endif /* ZVINS_QSH_LINK_WIRE_H */
