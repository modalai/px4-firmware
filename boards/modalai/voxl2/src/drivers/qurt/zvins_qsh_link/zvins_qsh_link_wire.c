/* SPDX-License-Identifier: BSD-3-Clause */
#include "zvins_qsh_link_wire.h"

#include <string.h>

static void zql_shared_fence(void) {
#if defined(__hexagon__)
  __asm__ __volatile__("syncht" ::: "memory");
#else
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

int zql_region_header_valid(const volatile zql_region_t *region) {
  const volatile zql_region_header_t *header;
  if (region == NULL) return 0;
  header = &region->header;
  zql_shared_fence();
  return header->magic == ZQL_REGION_MAGIC &&
         header->abi_version == ZQL_ABI_VERSION &&
         header->header_bytes == ZQL_HEADER_BYTES &&
         header->region_bytes == ZQL_REGION_BYTES &&
         header->contract_fingerprint == ZQL_CONTRACT_FINGERPRINT &&
         header->slot_count == ZQL_SLOT_COUNT &&
         header->slot_bytes == ZQL_SLOT_BYTES;
}

void zql_region_initialize(volatile zql_region_t *region,
                           uint32_t producer_generation,
                           uint32_t restart_count) {
  uint32_t index;
  volatile zql_region_header_t *header;
  if (region == NULL || producer_generation == 0u) return;
  header = &region->header;

  header->producer_state = ZQL_PRODUCER_INITIALIZING;
  zql_shared_fence();
  for (index = 0u; index < ZQL_SLOT_COUNT; ++index)
    region->slot[index].commit_sequence = 0u;
  zql_shared_fence();

  header->magic = ZQL_REGION_MAGIC;
  header->abi_version = ZQL_ABI_VERSION;
  header->header_bytes = ZQL_HEADER_BYTES;
  header->region_bytes = ZQL_REGION_BYTES;
  header->contract_fingerprint = ZQL_CONTRACT_FINGERPRINT;
  header->slot_count = ZQL_SLOT_COUNT;
  header->slot_bytes = ZQL_SLOT_BYTES;
  header->producer_generation = producer_generation;
  header->write_sequence = 0u;
  header->publish_count = 0u;
  header->trigger_failures = 0u;
  header->rejected_samples = 0u;
  header->restart_count = restart_count;
  memset((void *)header->reserved, 0, sizeof(header->reserved));
  zql_shared_fence();
  header->producer_state = ZQL_PRODUCER_READY;
  zql_shared_fence();
}

void zql_region_mark_offline(volatile zql_region_t *region) {
  if (region == NULL || !zql_region_header_valid(region)) return;
  zql_shared_fence();
  region->header.producer_state = ZQL_PRODUCER_OFFLINE;
  zql_shared_fence();
}

int zql_ring_publish(volatile zql_region_t *region,
                     const zql_odometry_slot_t *sample) {
  volatile zql_odometry_slot_t *destination;
  zql_odometry_slot_t committed;
  uint32_t sequence;
  uint32_t index;
  if (region == NULL || sample == NULL || !zql_region_header_valid(region) ||
      region->header.producer_state != ZQL_PRODUCER_READY)
    return -1;

  sequence = region->header.write_sequence + 1u;
  if (sequence == 0u) return -2;
  index = (sequence - 1u) & ((uint32_t)ZQL_SLOT_COUNT - 1u);
  destination = &region->slot[index];
  committed = *sample;
  committed.magic = ZQL_SLOT_MAGIC;
  committed.abi_version = ZQL_ABI_VERSION;
  committed.struct_bytes = ZQL_SLOT_BYTES;
  committed.producer_generation = region->header.producer_generation;
  committed.sequence = sequence;
  committed.commit_sequence = 0u;

  destination->commit_sequence = 0u;
  zql_shared_fence();
  memcpy((void *)destination, &committed,
         offsetof(zql_odometry_slot_t, commit_sequence));
  zql_shared_fence();
  destination->commit_sequence = sequence;
  zql_shared_fence();
  region->header.write_sequence = sequence;
  region->header.publish_count = region->header.publish_count + 1u;
  zql_shared_fence();
  return 0;
}

int zql_ring_read_latest(const volatile zql_region_t *region,
                         zql_consumer_cursor_t *cursor,
                         zql_odometry_slot_t *sample) {
  const volatile zql_odometry_slot_t *source;
  uint32_t producer_state;
  uint32_t generation;
  uint32_t sequence;
  uint32_t commit_before;
  uint32_t commit_after;
  uint32_t index;
  if (region == NULL || cursor == NULL || sample == NULL) {
    if (cursor != NULL) ++cursor->invalid_reads;
    return -1;
  }
  producer_state = region->header.producer_state;
  zql_shared_fence();
  if (producer_state == ZQL_PRODUCER_OFFLINE ||
      producer_state == ZQL_PRODUCER_INITIALIZING)
    return 0;
  if (producer_state != ZQL_PRODUCER_READY ||
      !zql_region_header_valid(region)) {
    ++cursor->invalid_reads;
    return -1;
  }

  generation = region->header.producer_generation;
  zql_shared_fence();
  sequence = region->header.write_sequence;
  if (generation == 0u) {
    ++cursor->invalid_reads;
    return -1;
  }
  if (cursor->producer_generation != generation) {
    if (cursor->producer_generation != 0u) ++cursor->producer_restarts;
    cursor->producer_generation = generation;
    cursor->last_sequence = 0u;
  }
  if (sequence == 0u || sequence == cursor->last_sequence) return 0;
  if (cursor->last_sequence != 0u && sequence < cursor->last_sequence) {
    ++cursor->invalid_reads;
    return -1;
  }

  index = (sequence - 1u) & ((uint32_t)ZQL_SLOT_COUNT - 1u);
  source = &region->slot[index];
  commit_before = source->commit_sequence;
  if (commit_before != sequence) return 0;
  zql_shared_fence();
  memcpy(sample, (const void *)source, sizeof(*sample));
  zql_shared_fence();
  commit_after = source->commit_sequence;
  if (commit_after != sequence || sample->commit_sequence != sequence ||
      sample->magic != ZQL_SLOT_MAGIC ||
      sample->abi_version != ZQL_ABI_VERSION ||
      sample->struct_bytes != ZQL_SLOT_BYTES ||
      sample->producer_generation != generation ||
      sample->sequence != sequence ||
      region->header.producer_generation != generation ||
      region->header.producer_state != ZQL_PRODUCER_READY) {
    ++cursor->invalid_reads;
    return -1;
  }

  if (cursor->last_sequence != 0u && sequence > cursor->last_sequence + 1u)
    cursor->missed_samples += sequence - cursor->last_sequence - 1u;
  cursor->last_sequence = sequence;
  return 1;
}
