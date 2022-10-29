/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uORBAppsProtobufChannel.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <cstdio>
#include <pthread.h>
#include <string.h>

#include "fc_sensor.h"

// TODO: Get rid of all the shmem stuff!
unsigned char *adsp_changed_index = nullptr;

// Initialize the static members
uORB::AppsProtobufChannel *uORB::AppsProtobufChannel::_InstancePtr = nullptr;
uORBCommunicator::IChannelRxHandler *uORB::AppsProtobufChannel::_RxHandler = nullptr;
std::map<std::string, int> uORB::AppsProtobufChannel::_SlpiSubscriberCache;
pthread_mutex_t uORB::AppsProtobufChannel::_tx_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t uORB::AppsProtobufChannel::_rx_mutex = PTHREAD_MUTEX_INITIALIZER;
bool uORB::AppsProtobufChannel::_Debug;

static bool print_first_one = true;
static bool print_first_runt = true;

void uORB::AppsProtobufChannel::ReceiveCallback(const char *topic,
                                                const uint8_t *data,
                                                uint32_t length_in_bytes) {
    if (_Debug) PX4_INFO("Got received data callback for topic %s", topic);

    if (strcmp(topic, "slpi_debug") == 0) {
        PX4_INFO("SLPI: %s", (const char *) data);
    } else if (strcmp(topic, "slpi_error") == 0) {
        PX4_ERR("SLPI: %s", (const char *) data);
    } else if (_RxHandler) {
        if (strcmp(topic, "aggregation") == 0) {
            // PX4_INFO("Parsing aggregate buffer of length %u", length_in_bytes);
            uint32_t current_index = 0;
            char name_buffer[80];
            while (current_index < length_in_bytes) {
                uint32_t sync_flag = *((uint32_t*) &data[current_index]);
                if (sync_flag != 0x5A01FF00) {
                    PX4_ERR("\tExpected sync flag but got 0x%X", sync_flag);
                    break;
                }
                current_index += 4;
                uint32_t name_length = *((uint32_t*) &data[current_index]);
                // if (name_length > 79) {
                //     PX4_ERR("\tName length too long %u", name_length);
                //     break;
                // }
                current_index += 4;
                uint32_t data_length = *((uint32_t*) &data[current_index]);
                // if (data_length > (length_in_bytes - current_index)) {
                //     PX4_ERR("\tData length too long %u", data_length);
                //     break;
                // }
                current_index += 4;
                memcpy(name_buffer, &data[current_index], name_length);
                name_buffer[name_length] = 0;
                current_index += name_length;
                // PX4_INFO("\tTopic: %s, name length %u, data length: %u", name_buffer, name_length, data_length);
                _RxHandler->process_received_message(name_buffer,
                                                     data_length,
                                                     const_cast<uint8_t*>(&data[current_index]));
                current_index += data_length;
            }
            // for (uint8_t i = 0; i < 10; i++) {
            //     uint8_t* accel_data = (uint8_t*) &data[i*48];
            //     _RxHandler->process_received_message(topic,
            //                                          48,
            //                                          const_cast<uint8_t*>(accel_data));
            // }
            if (print_first_one) {
                print_first_one = false;
                PX4_INFO("FIRST: %u %u %u", *((uint32_t*) &data[0]), *((uint32_t*) &data[4]), *((uint32_t*) &data[8]));
            }
            if (print_first_runt && (length_in_bytes < 12)) {
                print_first_runt = false;
                PX4_INFO("RUNT: %u %u %u", *((uint32_t*) &data[0]), *((uint32_t*) &data[4]), *((uint32_t*) &data[8]));
            }
        } else {
            _RxHandler->process_received_message(topic,
                                                 length_in_bytes,
                                                 const_cast<uint8_t*>(data));
        }
    } else {
        PX4_ERR("uORB pointer is null in %s", __FUNCTION__);
    }
}

void uORB::AppsProtobufChannel::AdvertiseCallback(const char *topic) {
    if (_Debug) PX4_INFO("Got advertisement callback for topic %s", topic);

    if (_RxHandler) {
        _RxHandler->process_remote_topic(topic);
    } else {
        PX4_ERR("uORB pointer is null in %s", __FUNCTION__);
    }
}

void uORB::AppsProtobufChannel::SubscribeCallback(const char *topic) {
    if (_Debug) PX4_INFO("Got subscription callback for topic %s", topic);

    pthread_mutex_lock(&_rx_mutex);
    _SlpiSubscriberCache[topic]++;
    pthread_mutex_unlock(&_rx_mutex);

    if (_RxHandler) {
        _RxHandler->process_add_subscription(topic);
    } else {
        // This can happen on startup if the remote entity is up and
        // running before this side has completed initialization. It is
        // okay because we have noted the event in the subscriber cache.
        PX4_WARN("uORB pointer is null in %s", __FUNCTION__);
    }
}

void uORB::AppsProtobufChannel::UnsubscribeCallback(const char *topic) {
    if (_Debug) PX4_INFO("Got remove subscription callback for topic %s", topic);

    pthread_mutex_lock(&_rx_mutex);
    if (_SlpiSubscriberCache[topic]) _SlpiSubscriberCache[topic]--;
    pthread_mutex_unlock(&_rx_mutex);

    if (_RxHandler) {
        _RxHandler->process_remove_subscription(topic);
    } else {
        PX4_ERR("uORB pointer is null in %s", __FUNCTION__);
    }
}

bool uORB::AppsProtobufChannel::Initialize(bool enable_debug) {
    if (_Initialized == false) {
        fc_callbacks cb = {&ReceiveCallback, &AdvertiseCallback,
                           &SubscribeCallback, &UnsubscribeCallback};
        if (fc_sensor_initialize(enable_debug, &cb) != 0) {
            PX4_ERR("Error calling the muorb protobuf initalize method");
        } else {
            if (enable_debug) {
                _Debug = true;
                PX4_INFO("muorb protobuf initalize method succeeded");
            }
            _Initialized = true;
        }
    }
    return _Initialized;
}

int16_t uORB::AppsProtobufChannel::topic_advertised(const char *messageName)
{
    if (_Initialized) {
        if (_Debug) PX4_INFO("Advertising topic %s to remote side", messageName);
        pthread_mutex_lock(&_tx_mutex);
        int16_t rc = fc_sensor_advertise(messageName);
        pthread_mutex_unlock(&_tx_mutex);
        return rc;
    }
    return -1;
}

int16_t uORB::AppsProtobufChannel::add_subscription(const char *messageName, int msgRateInHz)
{
    (void)(msgRateInHz);
    if (_Initialized) {
        pthread_mutex_lock(&_tx_mutex);
        int16_t rc = fc_sensor_subscribe(messageName);
        pthread_mutex_unlock(&_tx_mutex);
        return rc;
    }
    return -1;
}

int16_t uORB::AppsProtobufChannel::remove_subscription(const char *messageName)
{
    if (_Initialized) {
        pthread_mutex_lock(&_tx_mutex);
        int16_t rc = fc_sensor_unsubscribe(messageName);
        pthread_mutex_unlock(&_tx_mutex);
        return rc;
    }
    return -1;
}

int16_t uORB::AppsProtobufChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}

int16_t uORB::AppsProtobufChannel::send_message(const char *messageName, int length, uint8_t *data)
{
    bool enable_debug = false;
    if ((_MessageCounter++ % 100) == 0) enable_debug = true;

    if (_Initialized) {
        pthread_mutex_lock(&_rx_mutex);
        int has_subscribers = _SlpiSubscriberCache[messageName];
        pthread_mutex_unlock(&_rx_mutex);

        if (has_subscribers) {
            // PX4_DEBUG("Sending data for topic %s", messageName);
            if (_Debug && enable_debug) PX4_INFO("Sending data for topic %s", messageName);
            // if ((strcmp(messageName, "mavlink_tx_msg") == 0) && (data[17] == 22)) {
            //     PX4_INFO("apps mavlink_tx_msg param_value: %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %lu",
            //              data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29], hrt_absolute_time());
            // }
            pthread_mutex_lock(&_tx_mutex);
            int16_t rc = fc_sensor_send_data(messageName, data, length);
            pthread_mutex_unlock(&_tx_mutex);
            return rc;
        } else {
            // There are no remote subscribers so no need to actually send
            // the data. If a subscription comes in later, the data will
            // be re-sent to them at that time.
            // PX4_WARN("No subscribers (yet) in %s for topic %s", __FUNCTION__, messageName);
            if (_Debug && enable_debug) PX4_INFO("No subscribers (yet) in %s for topic %s", __FUNCTION__, messageName);
            return 0;
        }
    }
    return -1;
}
