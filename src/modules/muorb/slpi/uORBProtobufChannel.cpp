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
#include "uORBManager.hpp"
#include "uORBProtobufChannel.hpp"
#include <px4_platform_common/log.h>
#include <algorithm>
#include <string.h>
#include <drivers/drv_hrt.h>

fc_func_ptrs muorb_func_ptrs;

// static initialization.
uORB::ProtobufChannel uORB::ProtobufChannel::_Instance;
uORBCommunicator::IChannelRxHandler *uORB::ProtobufChannel::_RxHandler;
std::map<std::string, int> uORB::ProtobufChannel::_AppsSubscriberCache;
pthread_mutex_t uORB::ProtobufChannel::_rx_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t uORB::ProtobufChannel::_tx_mutex = PTHREAD_MUTEX_INITIALIZER;

// TODO: Create a way to set this a run time
bool uORB::ProtobufChannel::_debug = false;

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::topic_advertised(const char *messageName)
{
	if (_debug) PX4_INFO("Advertising %s on remote side", messageName);
	if (muorb_func_ptrs.advertise_func_ptr) {
        pthread_mutex_lock(&_tx_mutex);
        int16_t rc = muorb_func_ptrs.advertise_func_ptr(messageName);
        pthread_mutex_unlock(&_tx_mutex);
        return rc;
    }

    PX4_ERR("advertise_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::add_subscription(const char *messageName, int32_t msgRateInHz)
{
	if (_debug) PX4_INFO("Subscribing to %s on remote side", messageName);
	if (muorb_func_ptrs.subscribe_func_ptr) {
        pthread_mutex_lock(&_tx_mutex);
        int16_t rc = muorb_func_ptrs.subscribe_func_ptr(messageName);
        pthread_mutex_unlock(&_tx_mutex);
        return rc;
    }

    PX4_ERR("subscribe_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::remove_subscription(const char *messageName)
{
	if (_debug) PX4_INFO("Unsubscribing from %s on remote side", messageName);
	if (muorb_func_ptrs.unsubscribe_func_ptr) {
        pthread_mutex_lock(&_tx_mutex);
        int16_t rc = muorb_func_ptrs.unsubscribe_func_ptr(messageName);
        pthread_mutex_unlock(&_tx_mutex);
        return rc;
    }

    PX4_ERR("unsubscribe_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}

//==============================================================================
//==============================================================================

int16_t uORB::ProtobufChannel::send_message(const char *messageName, int32_t length, uint8_t *data)
{
    // This function can be called from the PX4 log function so we have to make
    // sure that we do not call PX4_INFO, PX4_ERR, etc. That would cause an
    // infinite loop!
    bool is_not_slpi_log = true;
    if ((strcmp(messageName, "slpi_debug") == 0) || (strcmp(messageName, "slpi_error") == 0)) is_not_slpi_log = false;

    if (muorb_func_ptrs.topic_data_func_ptr) {
        if ((_debug) && (is_not_slpi_log)) PX4_INFO("Got message for topic %s", messageName);
        std::string temp(messageName);
        int has_subscribers = 0;
        pthread_mutex_lock(&_rx_mutex);
        has_subscribers = _AppsSubscriberCache[temp];
        pthread_mutex_unlock(&_rx_mutex);

        if ((has_subscribers) || (is_not_slpi_log == false)) {
            if ((_debug) && (is_not_slpi_log)) PX4_INFO("Sending message for topic %s", messageName);
            pthread_mutex_lock(&_tx_mutex);
            int16_t rc = muorb_func_ptrs.topic_data_func_ptr(messageName, data, length);
            pthread_mutex_unlock(&_tx_mutex);
            return rc;
        }

        // If there are no remote subscribers then we do not need to send the
        // message over. That is still a success.
        if ((_debug) && (is_not_slpi_log)) PX4_INFO("Skipping message for topic %s", messageName);
        return 0;
    }

    if (is_not_slpi_log) PX4_ERR("topic_data_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

__BEGIN_DECLS
extern int dspal_main(int argc, char *argv[]);
__END_DECLS

static bool px4muorb_orb_initialized = false;

int px4muorb_orb_initialize(fc_func_ptrs *func_ptrs, int32_t clock_offset_us)
{
    // Make sure SLPI clock is, more or less, aligned with apps clock. This
    // alignment drifts over time so this function will get called to update
    // the offset.
    // PX4_INFO("Got time offset %d", clock_offset_us);
    hrt_set_absolute_time_offset(clock_offset_us);

    // If this is the first time this function has been called, initialize
    // everything. Otherwise it is just being called to update the time offset.
    if ( ! px4muorb_orb_initialized) {
    	// The uORB Manager needs to be initialized first up, otherwise the instance is nullptr.
    	uORB::Manager::initialize();
    	// Register the protobuf muorb with uORBManager.
    	uORB::Manager::get_instance()->set_uorb_communicator(
    		uORB::ProtobufChannel::GetInstance());

    	// Now continue with the usual dspal startup.
    	const char *argv[3] = { "dspal", "start" };
    	int argc = 2;

        // Make sure that argv has a NULL pointer in the end.
        argv[argc] = NULL;

    	if (dspal_main(argc, (char **) argv)) {
            PX4_ERR("dspal_main failed in %s", __FUNCTION__);
            return -1;
        }

        if (func_ptrs == NULL) {
            PX4_ERR("NULL func_ptrs in %s", __FUNCTION__);
            return -1;
        }

        // Save off the function pointers needed to get access to
        // the SLPI protobuf functions.
        muorb_func_ptrs = *func_ptrs;
        if ((muorb_func_ptrs.advertise_func_ptr == NULL) ||
            (muorb_func_ptrs.subscribe_func_ptr == NULL) ||
            (muorb_func_ptrs.unsubscribe_func_ptr == NULL) ||
            (muorb_func_ptrs.topic_data_func_ptr == NULL) ||
            (muorb_func_ptrs.config_spi_bus == NULL) ||
            (muorb_func_ptrs.spi_transfer == NULL) ||
            (muorb_func_ptrs.config_i2c_bus == NULL) ||
            (muorb_func_ptrs.set_i2c_address == NULL) ||
            (muorb_func_ptrs.i2c_transfer == NULL) ||
            (muorb_func_ptrs.open_uart_func == NULL) ||
            (muorb_func_ptrs.write_uart_func == NULL) ||
            (muorb_func_ptrs.read_uart_func == NULL) ||
            (muorb_func_ptrs.register_interrupt_callback == NULL)) {
            PX4_ERR("NULL function pointers in %s", __FUNCTION__);
            return -1;
        }

        // Configure the I2C driver function pointers
        device::I2C::configure_callbacks(muorb_func_ptrs.config_i2c_bus, muorb_func_ptrs.set_i2c_address, muorb_func_ptrs.i2c_transfer);

        // Configure the SPI driver function pointers
        device::SPI::configure_callbacks(muorb_func_ptrs.config_spi_bus, muorb_func_ptrs.spi_transfer);

        // Configure the UART driver function pointers
        configure_uart_callbacks(muorb_func_ptrs.open_uart_func, muorb_func_ptrs.write_uart_func, muorb_func_ptrs.read_uart_func);

        // Initialize the interrupt callback registration
        register_interrupt_callback_initalizer(muorb_func_ptrs.register_interrupt_callback);

        px4muorb_orb_initialized = true;
    }

    // Proof of concept to send debug messages to Apps side.
    // char hello_world_message[] = "Hello, World!";
	// uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();
	// if (ch != nullptr) {
	// 	ch->send_message("slpi_debug", strlen(hello_world_message) + 1, (uint8_t *) hello_world_message);
	// }

	return 0;
}

int px4muorb_topic_advertised(const char *topic_name)
{
	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
        if (channel->DebugEnabled()) PX4_INFO("px4muorb_topic_advertised [%s] on remote side...", topic_name);
        uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
        if (rxHandler) {
            return rxHandler->process_remote_topic(topic_name, 0);
        } else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
        }
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}

int px4muorb_add_subscriber(const char *topic_name)
{
	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
        if (channel->DebugEnabled()) PX4_INFO("px4muorb_add_subscriber [%s] on remote side...", topic_name);
    	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
    	if (rxHandler) {
            channel->AddRemoteSubscriber(topic_name);
    		return rxHandler->process_add_subscription(topic_name);
    	} else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
    	}
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}

int px4muorb_remove_subscriber(const char *topic_name)
{
	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
        if (channel->DebugEnabled()) PX4_INFO("px4muorb_remove_subscriber [%s] on remote side...", topic_name);
    	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
    	if (rxHandler) {
            channel->RemoveRemoteSubscriber(topic_name);
    		return rxHandler->process_remove_subscription(topic_name);
    	} else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
    	}
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}

int px4muorb_send_topic_data(const char *topic_name, const uint8_t *data,
			                 int data_len_in_bytes)
{
	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
        if (channel->DebugEnabled()) PX4_INFO("px4muorb_send_topic_data [%s] on remote side...", topic_name);
    	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
    	if (rxHandler) {
    		return rxHandler->process_received_message(topic_name,
                                                       data_len_in_bytes,
    				                                   (uint8_t *) data);
    	} else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
    	}
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}
