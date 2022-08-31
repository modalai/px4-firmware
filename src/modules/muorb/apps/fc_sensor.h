#ifndef FC_SENSOR_H
#define FC_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef void (*fc_receive_cb)(const char *topic,
			      const uint8_t *data,
			      uint32_t length_in_bytes);
typedef void (*fc_advertise_cb)(const char *topic);
typedef void (*fc_add_subscription_cb)(const char *topic);
typedef void (*fc_remove_subscription_cb)(const char *topic);

typedef struct {
	fc_receive_cb               rx_callback;
	fc_advertise_cb             ad_callback;
	fc_add_subscription_cb      sub_callback;
	fc_remove_subscription_cb   remove_sub_callback;
} fc_callbacks;

int fc_sensor_initialize(bool enable_debug_messages, fc_callbacks *callbacks);
int fc_sensor_advertise(const char *topic);
int fc_sensor_subscribe(const char *topic);
int fc_sensor_unsubscribe(const char *topic);
int fc_sensor_send_data(const char *topic,
			const uint8_t *data,
			uint32_t length_in_bytes);
#ifdef __cplusplus
}
#endif

#endif // FC_SENSOR_H
