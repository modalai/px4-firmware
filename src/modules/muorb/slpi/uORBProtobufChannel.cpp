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
#include <qurt.h>

fc_func_ptrs muorb_func_ptrs;

// static initialization.
uORB::ProtobufChannel uORB::ProtobufChannel::_Instance;
uORBCommunicator::IChannelRxHandler *uORB::ProtobufChannel::_RxHandler;
std::map<std::string, int> uORB::ProtobufChannel::_AppsSubscriberCache;
pthread_mutex_t uORB::ProtobufChannel::_rx_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t uORB::ProtobufChannel::_tx_mutex = PTHREAD_MUTEX_INITIALIZER;

const std::string uORB::ProtobufChannel::AggregationBuffer::AGGREGATION_BUFFER_TOPIC_NAME = "aggregation";
uORB::ProtobufChannel::AggregationBuffer uORB::ProtobufChannel::_AggregationBuffer;

// TODO: Create a way to set this a run time
bool uORB::ProtobufChannel::_debug = false;

qurt_thread_t test_tid;
qurt_thread_attr_t test_attr;
char test_stack[8096];

static void test_func(void *ptr)
{
	PX4_INFO("muorb aggregator thread running");

	uORB::ProtobufChannel *muorb = uORB::ProtobufChannel::GetInstance();

	while (true) {

		pthread_mutex_lock(&muorb->_tx_mutex);

		// Check for timeout. Send buffer if timeout happened.
		uint32_t current_length = muorb->_AggregationBuffer.GetBufferLength();

		if (current_length && muorb->_AggregationBuffer.Timeout()) {
			uint8_t* current_pointer = muorb->_AggregationBuffer.GetBufferPointer();

			// PX4_INFO("*** Thread timeout: Sending aggregate buffer of length %u at time %llu", current_length, hrt_absolute_time());

			(void) muorb_func_ptrs.topic_data_func_ptr(uORB::ProtobufChannel::AggregationBuffer::AGGREGATION_BUFFER_TOPIC_NAME.c_str(),
													   current_pointer, current_length);
			muorb->_AggregationBuffer.MoveToNextBuffer();
		// } else {
		// 	PX4_INFO("@@@ Skipping send of %u bytes at %llu, no timeout yet", current_length, hrt_absolute_time());
		}

		pthread_mutex_unlock(&muorb->_tx_mutex);

		qurt_timer_sleep(5000);
	}

    qurt_thread_exit(QURT_EOK);
}

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

void uORB::ProtobufChannel::AddRemoteSubscriber(const std::string &messageName)
{
    pthread_mutex_lock(&_rx_mutex);
    _AppsSubscriberCache[messageName]++;
    pthread_mutex_unlock(&_rx_mutex);

	PX4_INFO("Added remote subscriber for topic %s", messageName.c_str());
}

// Message format in aggregation buffer
// 4 byte sync flag
// 4 byte message name length
// 4 byte data length
// Variable length message name
// Variable length message data



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
			// if ((_debug) && (is_not_slpi_log)) PX4_INFO("-- Sending message for topic %s length %d", messageName, length);
			// PX4_INFO("--- Sending message for topic %s length %d, time %llu", messageName, length, hrt_absolute_time());

			int16_t rc = 0;
			// bool sent_due_to_full = false;
			// bool sent_due_to_timeout = false;

			pthread_mutex_lock(&_tx_mutex);

			// if (_AggregationBuffer.NewRecordOverflows(messageName, length)) {
			// 	rc = muorb_func_ptrs.topic_data_func_ptr(AggregationBuffer::AGGREGATION_BUFFER_TOPIC_NAME.c_str(),
			// 											 _AggregationBuffer.GetBufferPointer(),
			// 											 _AggregationBuffer.GetBufferLength());
			// 	sent_due_to_full = true;
			// 	_AggregationBuffer.MoveToNextBuffer();
			// }

			_AggregationBuffer.AddRecordToBuffer(messageName, length, data);

			// Check for timeout. Send buffer if timeout happened. Note, this assumes
			// that data is coming in fairly frequently.
			if (_AggregationBuffer.Timeout()) {
				uint8_t* current_pointer = _AggregationBuffer.GetBufferPointer();
				uint32_t current_length = _AggregationBuffer.GetBufferLength();

				// PX4_INFO("*** Send timeout: Sending aggregate buffer of length %u at time %llu", current_length, hrt_absolute_time());

				rc = muorb_func_ptrs.topic_data_func_ptr(AggregationBuffer::AGGREGATION_BUFFER_TOPIC_NAME.c_str(),
														 current_pointer, current_length);
				// sent_due_to_timeout = true;
				_AggregationBuffer.MoveToNextBuffer();
			}

			pthread_mutex_unlock(&_tx_mutex);

			// if (sent_due_to_full) PX4_INFO("--- Sending full buffer at %llu", hrt_absolute_time());
			// if (sent_due_to_timeout) PX4_INFO("--- Sending buffer due to timeout at %llu", hrt_absolute_time());

			return rc;
		}

        // If there are no remote subscribers then we do not need to send the
        // message over. That is still a success.
        // if ((_debug) && (is_not_slpi_log)) PX4_INFO("Skipping message for topic %s", messageName);
        // PX4_INFO("Skipping message for topic %s", messageName);
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

	    qurt_thread_attr_init(&test_attr);
	    qurt_thread_attr_set_stack_addr(&test_attr, test_stack);
	    qurt_thread_attr_set_stack_size(&test_attr, 8096);
	    qurt_thread_attr_set_priority(&test_attr, 40);
	    (void) qurt_thread_create(&test_tid, &test_attr, test_func, NULL);

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
            return rxHandler->process_remote_topic(topic_name);
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
        // if ((strcmp(topic_name, "mavlink_tx_msg") == 0) && (data[17] == 22)) {
        //     PX4_INFO("slpi mavlink_tx_msg param_value: %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %lu",
        //              data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29], hrt_absolute_time());
        // }
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
