/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#include "mUORBAggregator.hpp"
#include <px4_platform_common/log.h>

const bool mUORB::Aggregator::debugFlag = true;

void mUORB::Aggregator::processTopic(const char *topic, const uint8_t *data, uint32_t length_in_bytes) {
    if (isAggregate(topic)) {
        if (debugFlag) PX4_INFO("Parsing aggregate buffer of length %u", length_in_bytes);
        uint32_t current_index = 0;
        const uint32_t name_buffer_length = 80;
        char name_buffer[name_buffer_length];

        while ((current_index + headerSize) < length_in_bytes) {
            uint32_t sync_flag = *((uint32_t*) &data[current_index]);
            if (sync_flag != syncFlag) {
                PX4_ERR("Expected sync flag but got 0x%X", sync_flag);
                break;
            }
            current_index += syncFlagSize;

            uint32_t name_length = *((uint32_t*) &data[current_index]);
            // Make sure name plus a terminating null can fit into our buffer
            if (name_length > (name_buffer_length - 1)) {
                PX4_ERR("Name length too long %u", name_length);
                break;
            }
            current_index += topicNameLengthSize;

            uint32_t data_length = *((uint32_t*) &data[current_index]);
            current_index += dataLengthSize;

            int32_t payload_size = name_length + data_length;
            int32_t remaining_bytes = length_in_bytes - current_index;
            if (payload_size > remaining_bytes) {
                PX4_ERR("Payload too big %u. Remaining bytes %d", payload_size, remaining_bytes);
                break;
            }

            memcpy(name_buffer, &data[current_index], name_length);
            name_buffer[name_length] = 0;

            current_index += name_length;

            if (debugFlag) PX4_INFO("Parsed topic: %s, name length %u, data length: %u", name_buffer, name_length, data_length);
            _RxHandler->process_received_message(name_buffer,
                                                 data_length,
                                                 const_cast<uint8_t*>(&data[current_index]));
            current_index += data_length;
        }
    } else {
        // It isn't an aggregated buffer so just process normally
        _RxHandler->process_received_message(topic,
                                             length_in_bytes,
                                             const_cast<uint8_t*>(data));
    }
}
