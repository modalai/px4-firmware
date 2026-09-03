/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef ZVINS_QSH_LINK_QDI_H
#define ZVINS_QSH_LINK_QDI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZQL_QDI_CMD_GET_INFO (256 + 0)
#define ZQL_QDI_CMD_BIND_RX (256 + 1)
#define ZQL_QDI_CMD_RING (256 + 2)
#define ZQL_QDI_CMD_WAIT_RX_ACK (256 + 3)
#define ZQL_QDI_CMD_WAKE_RX (256 + 4)

enum zql_transport_role {
  ZQL_TRANSPORT_ROLE_NONE = 0,
  ZQL_TRANSPORT_ROLE_ADSP_CONSUMER = 1,
  ZQL_TRANSPORT_ROLE_CDSP_PRODUCER = 2
};

enum zql_transport_status {
  ZQL_STATUS_SMEM_READY = 1u << 0,
  ZQL_STATUS_IPCC_ATTACHED = 1u << 1,
  ZQL_STATUS_RX_REGISTERED = 1u << 2,
  ZQL_STATUS_QDI_REGISTERED = 1u << 3,
  ZQL_STATUS_RX_BOUND = 1u << 4,
  ZQL_STATUS_WAIT_RX_ACK = 1u << 5,
  ZQL_STATUS_READY = 1u << 31
};

typedef struct zql_transport_info {
  uint32_t magic;
  uint16_t transport_abi_version;
  uint16_t wire_abi_version;
  uint32_t struct_bytes;
  uint32_t contract_fingerprint;
  uint32_t status;
  uint32_t role;
  uint32_t hexagon_arch;
  uint32_t smem_item;
  uint32_t region_bytes;
  uint32_t local_ipcc_client;
  uint32_t remote_ipcc_client;
  uint32_t ipcc_protocol;
  uint32_t ipcc_signal;
  uint32_t doorbells;
  uint32_t doorbells_without_receiver;
  uint32_t signal_errors;
  uint32_t rings;
  uint32_t ring_errors;
  int32_t smem_result;
  int32_t ipcc_result;
} zql_transport_info_t;

typedef struct zql_rx_binding {
  uint32_t struct_bytes;
  int32_t signal_group;
  int32_t rx_signal;
  int32_t stop_signal;
} zql_rx_binding_t;

typedef struct zql_transport_endpoint {
  int handle;
  zql_transport_info_t info;
} zql_transport_endpoint_t;

typedef struct zql_smem_endpoint {
  int handle;
} zql_smem_endpoint_t;

int zql_transport_info_valid(const zql_transport_info_t *info,
                             uint32_t expected_role,
                             uint32_t expected_hexagon_arch);
int zql_transport_open(zql_transport_endpoint_t *endpoint,
                       uint32_t expected_role,
                       uint32_t expected_hexagon_arch);
void zql_transport_close(zql_transport_endpoint_t *endpoint);
int zql_transport_refresh(zql_transport_endpoint_t *endpoint);
int zql_transport_bind_rx(zql_transport_endpoint_t *endpoint,
                          zql_rx_binding_t *binding);
int zql_transport_ring(zql_transport_endpoint_t *endpoint);
int zql_transport_wait_rx_ack(zql_transport_endpoint_t *endpoint);
int zql_transport_wake_rx(zql_transport_endpoint_t *endpoint);

int zql_smem_open(zql_smem_endpoint_t *endpoint);
void zql_smem_close(zql_smem_endpoint_t *endpoint);
int zql_smem_map_fixed(zql_smem_endpoint_t *endpoint, void **buffer);

int zql_signal_wait(int signal);
int zql_signal_set(int signal);
int zql_signal_clear(int signal);

#ifdef __cplusplus
}
#endif
#endif /* ZVINS_QSH_LINK_QDI_H */
