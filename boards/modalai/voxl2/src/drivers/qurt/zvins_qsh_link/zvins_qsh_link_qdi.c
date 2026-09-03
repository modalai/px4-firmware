/* SPDX-License-Identifier: BSD-3-Clause */
/** User-PD client for the source-pinned fixed ZQL root transport. */
#include "zvins_qsh_link_qdi.h"

#include <stddef.h>
#include <string.h>

#include "qurt_qdi.h"
#include "qurt_qdi_constants.h"
#include "zvins_qsh_link_wire.h"

#define ZQL_SMEM_QDI_ALLOC (QDI_PRIVATE + 0)

typedef struct zql_smem_alloc_params {
  uint16_t remote_host;
  uint16_t smem_type;
  uint32_t size;
  void *buffer;
  uint32_t flags;
} zql_smem_alloc_params_t;

typedef char zql_smem_params_size_assert[
    sizeof(zql_smem_alloc_params_t) == 16u ? 1 : -1];
typedef char zql_transport_info_size_assert[
    sizeof(zql_transport_info_t) == 80u ? 1 : -1];
typedef char zql_rx_binding_size_assert[
    sizeof(zql_rx_binding_t) == 16u ? 1 : -1];

int zql_transport_info_valid(const zql_transport_info_t *info,
                             uint32_t expected_role,
                             uint32_t expected_hexagon_arch) {
  uint32_t expected_local;
  uint32_t expected_remote;
  if (info == NULL) return 0;
  if (expected_role == ZQL_TRANSPORT_ROLE_CDSP_PRODUCER) {
    expected_local = ZQL_IPCC_CDSP_CLIENT;
    expected_remote = ZQL_IPCC_ADSP_CLIENT;
  } else if (expected_role == ZQL_TRANSPORT_ROLE_ADSP_CONSUMER) {
    expected_local = ZQL_IPCC_ADSP_CLIENT;
    expected_remote = ZQL_IPCC_CDSP_CLIENT;
  } else {
    return 0;
  }
  return info->magic == ZQL_TRANSPORT_MAGIC &&
         info->transport_abi_version == ZQL_TRANSPORT_ABI_VERSION &&
         info->wire_abi_version == ZQL_ABI_VERSION &&
         info->struct_bytes == sizeof(*info) &&
         info->contract_fingerprint == ZQL_CONTRACT_FINGERPRINT &&
         (info->status & ZQL_STATUS_READY) != 0u &&
         (info->status & ZQL_STATUS_QDI_REGISTERED) != 0u &&
         info->role == expected_role &&
         info->hexagon_arch == expected_hexagon_arch &&
         info->smem_item == ZQL_SMEM_ID_VENDOR2 &&
         info->region_bytes == ZQL_REGION_BYTES &&
         info->local_ipcc_client == expected_local &&
         info->remote_ipcc_client == expected_remote &&
         info->ipcc_protocol == ZQL_IPCC_PROTOCOL &&
         info->ipcc_signal == ZQL_IPCC_SIGNAL;
}

int zql_transport_refresh(zql_transport_endpoint_t *endpoint) {
  zql_transport_info_t info;
  int result;
  if (endpoint == NULL || endpoint->handle < 0) return -1;
  memset(&info, 0, sizeof(info));
  result = qurt_qdi_handle_invoke(endpoint->handle, ZQL_QDI_CMD_GET_INFO,
                                  &info, sizeof(info));
  if (result != 0) return result;
  endpoint->info = info;
  return 0;
}

int zql_transport_open(zql_transport_endpoint_t *endpoint,
                       uint32_t expected_role,
                       uint32_t expected_hexagon_arch) {
  int result;
  if (endpoint == NULL) return -1;
  memset(endpoint, 0, sizeof(*endpoint));
  endpoint->handle = qurt_qdi_open(ZQL_TRANSPORT_DEVICE_NAME);
  if (endpoint->handle < 0) return endpoint->handle;
  result = zql_transport_refresh(endpoint);
  if (result == 0 &&
      !zql_transport_info_valid(&endpoint->info, expected_role,
                                expected_hexagon_arch))
    result = -1;
  if (result != 0) {
    (void)qurt_qdi_close(endpoint->handle);
    memset(endpoint, 0, sizeof(*endpoint));
    endpoint->handle = -1;
  }
  return result;
}

void zql_transport_close(zql_transport_endpoint_t *endpoint) {
  if (endpoint == NULL) return;
  if (endpoint->handle >= 0) (void)qurt_qdi_close(endpoint->handle);
  memset(endpoint, 0, sizeof(*endpoint));
  endpoint->handle = -1;
}

int zql_transport_bind_rx(zql_transport_endpoint_t *endpoint,
                          zql_rx_binding_t *binding) {
  if (endpoint == NULL || endpoint->handle < 0 || binding == NULL)
    return -1;
  memset(binding, 0, sizeof(*binding));
  return qurt_qdi_handle_invoke(endpoint->handle, ZQL_QDI_CMD_BIND_RX,
                                binding, sizeof(*binding));
}

int zql_transport_ring(zql_transport_endpoint_t *endpoint) {
  if (endpoint == NULL || endpoint->handle < 0) return -1;
  return qurt_qdi_handle_invoke(endpoint->handle, ZQL_QDI_CMD_RING);
}

int zql_transport_wait_rx_ack(zql_transport_endpoint_t *endpoint) {
  if (endpoint == NULL || endpoint->handle < 0) return -1;
  return qurt_qdi_handle_invoke(endpoint->handle,
                                ZQL_QDI_CMD_WAIT_RX_ACK);
}

int zql_transport_wake_rx(zql_transport_endpoint_t *endpoint) {
  if (endpoint == NULL || endpoint->handle < 0) return -1;
  return qurt_qdi_handle_invoke(endpoint->handle, ZQL_QDI_CMD_WAKE_RX);
}

int zql_smem_open(zql_smem_endpoint_t *endpoint) {
  if (endpoint == NULL) return -1;
  endpoint->handle = qurt_qdi_open("/dev/smem");
  return endpoint->handle < 0 ? endpoint->handle : 0;
}

void zql_smem_close(zql_smem_endpoint_t *endpoint) {
  if (endpoint == NULL || endpoint->handle < 0) return;
  (void)qurt_qdi_close(endpoint->handle);
  endpoint->handle = -1;
}

int zql_smem_map_fixed(zql_smem_endpoint_t *endpoint, void **buffer) {
  zql_smem_alloc_params_t params;
  int api_result = -1;
  int invoke_result;
  if (endpoint == NULL || endpoint->handle < 0 || buffer == NULL) return -1;
  *buffer = NULL;
  memset(&params, 0, sizeof(params));
  params.remote_host = ZQL_SMEM_INVALID_HOST;
  params.smem_type = ZQL_SMEM_ID_VENDOR2;
  params.size = ZQL_REGION_BYTES;
  invoke_result = qurt_qdi_handle_invoke(
      endpoint->handle, ZQL_SMEM_QDI_ALLOC, &params, &api_result);
  if (invoke_result != 0) return invoke_result;
  if (api_result != 0) return api_result;
  if (params.buffer == NULL || params.size != ZQL_REGION_BYTES) return -1;
  *buffer = params.buffer;
  return 0;
}

int zql_signal_wait(int signal) {
  return qurt_qdi_handle_invoke(signal, QDI_SIGNAL_WAIT);
}

int zql_signal_set(int signal) {
  return qurt_qdi_handle_invoke(signal, QDI_SIGNAL_SET);
}

int zql_signal_clear(int signal) {
  return qurt_qdi_handle_invoke(signal, QDI_SIGNAL_CLEAR);
}
