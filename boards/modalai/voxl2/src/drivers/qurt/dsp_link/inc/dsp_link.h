/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef DSP_LINK_H
#define DSP_LINK_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Receive data on a library-owned thread in the ADSP sensor user PD.
 *
 * The buffer is read-only and valid only until the callback returns. Copy any
 * data needed later, and keep the callback short. Do not close the link or
 * replace/unregister its callback from inside the callback.
 *
 * The current backend delivers a complete 256-byte CQL wire-ABI-3 packet,
 * including its envelope, unchanged from the CDSP sender. The application
 * decodes the packet. Delivery retains the existing latest-sample behavior:
 * intermediate samples may be skipped; this is not a reliable message queue.
 */
typedef void (*dsp_link_rx_cb_t)(const void *data, size_t length, void *context);

/**
 * Open the CDSP-to-ADSP link. Returns a nonnegative descriptor or a negative
 * errno value. The current transport supports one receiver at a time.
 *
 * Descriptors belong to this API; use them only with dsp_link_* functions.
 * No callback runs until dsp_link_register_rx_callback() installs one.
 * The local transport must be available, but the CDSP sender need not be
 * running. Returns -EBUSY if this API or another client already owns reception.
 * A successful open retries the transport connection if a later wait fails.
 */
int dsp_link_open(void);

/**
 * Install or replace the receive callback. A NULL callback disables delivery.
 * On success, any invocation of the previous callback has finished, so its
 * context can be released. Returns zero or a negative errno value. On failure,
 * retain the context until a successful unregister or close. Calling this from
 * the receive callback returns -EDEADLK.
 */
int dsp_link_register_rx_callback(int fd, dsp_link_rx_cb_t callback,
                                  void *context);

/**
 * Send data to the peer. ADSP-to-CDSP transmission is currently unsupported:
 * this returns -ENOTSUP for an open descriptor, or -EBADF for an invalid one.
 */
int dsp_link_send(int fd, const void *data, size_t length);

/**
 * Stop reception, join the receive thread, and release the descriptor.
 * After a successful close, no callback can still be running. Close the link
 * before unloading client code or freeing its callback context.
 * Returns zero or a negative errno value. Calling this from the receive
 * callback returns -EDEADLK. On failure, keep the context alive and retry close.
 */
int dsp_link_close(int fd);

#ifdef __cplusplus
}
#endif

#endif /* DSP_LINK_H */
