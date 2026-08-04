/****************************************************************************
 *
 *   Copyright (c) 2025-2026 ModalAI, inc. All rights reserved.
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

#include "mpa.hpp"

#include <drivers/drv_hrt.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/crsf_raw.h>
#include <uORB/uORB.h>

#include <string.h>

class CrsfBridge : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	CrsfBridge();
	~CrsfBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void request_stop() override;

private:
	void Run() override;
	void cleanup();

	static void control_callback(int ch, char *data, int bytes, void *context);

	uORB::SubscriptionCallbackWorkItem _crsf_raw_rx_sub{this, ORB_ID(crsf_raw_rx)};
	crsf_raw_s _crsf_raw_rx{};

	orb_advert_t _crsf_raw_tx_pub{nullptr};
	int _crsf_pipe_ch{-1};
};

ModuleBase::Descriptor CrsfBridge::desc{task_spawn, custom_command, print_usage};

CrsfBridge::CrsfBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

CrsfBridge::~CrsfBridge()
{
	cleanup();
}

void CrsfBridge::cleanup()
{
	_crsf_raw_rx_sub.unregisterCallback();

	if (_crsf_pipe_ch >= 0) {
		MPA::PipeServerClose(_crsf_pipe_ch);
		_crsf_pipe_ch = -1;
	}

	if (_crsf_raw_tx_pub != nullptr) {
		orb_unadvertise(_crsf_raw_tx_pub);
		_crsf_raw_tx_pub = nullptr;
	}
}

bool CrsfBridge::init()
{
	if (MPA::Initialize() < 0) {
		PX4_ERR("MPA init failed");
		return false;
	}

	char crsf_pipe_name[] = "crsf_raw";
	_crsf_pipe_ch = MPA::PipeCreate(crsf_pipe_name, SERVER_FLAG_EN_CONTROL_PIPE);

	if (_crsf_pipe_ch < 0) {
		PX4_ERR("Pipe create failed for %s", crsf_pipe_name);
		return false;
	}

	if (MPA::PipeServerSetControlCb(_crsf_pipe_ch, &CrsfBridge::control_callback, this) < 0) {
		PX4_ERR("Control callback setup failed for %s", crsf_pipe_name);
		cleanup();
		return false;
	}

	if (!_crsf_raw_rx_sub.registerCallback()) {
		PX4_ERR("crsf_raw_rx callback registration failed");
		cleanup();
		return false;
	}

	PX4_INFO("Created pipe '%s' on channel %d", crsf_pipe_name, _crsf_pipe_ch);
	return true;
}

void CrsfBridge::request_stop()
{
	ModuleBase::request_stop();
	ScheduleNow();
}

void CrsfBridge::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	int processed = 0;

	while (processed < crsf_raw_s::ORB_QUEUE_LENGTH && _crsf_raw_rx_sub.update(&_crsf_raw_rx)) {
		processed++;

		if (_crsf_raw_rx.len == 0 || _crsf_raw_rx.len > sizeof(_crsf_raw_rx.data)) {
			PX4_WARN("Invalid CRSF raw RX length: %u", static_cast<unsigned>(_crsf_raw_rx.len));
			continue;
		}

		crsf_raw_data_t crsf{};
		crsf.magic_number = CRSF_RAW_MAGIC_NUMBER;
		crsf.len = _crsf_raw_rx.len;
		crsf.timestamp_ns = static_cast<int64_t>(_crsf_raw_rx.timestamp) * 1000;
		memcpy(crsf.data, _crsf_raw_rx.data, _crsf_raw_rx.len);

		if (MPA::PipeWrite(_crsf_pipe_ch, &crsf, sizeof(crsf)) < 0) {
			PX4_ERR("Pipe %d write failed", _crsf_pipe_ch);
		}
	}

	if (_crsf_raw_rx_sub.updated()) {
		PX4_WARN("CRSF bridge hit max batch limit (%u messages)",
			 static_cast<unsigned>(crsf_raw_s::ORB_QUEUE_LENGTH));
		ScheduleNow();
	}
}

void CrsfBridge::control_callback(int ch, char *data, int bytes, void *context)
{
	CrsfBridge *bridge = static_cast<CrsfBridge *>(context);

	if (bridge == nullptr || data == nullptr || bytes <= 0 || ch != bridge->_crsf_pipe_ch) {
		PX4_WARN("Invalid CRSF control callback");
		return;
	}

	const size_t packet_size = sizeof(crsf_raw_data_t);

	if (static_cast<size_t>(bytes) % packet_size != 0) {
		PX4_WARN("Invalid CRSF control size: %d bytes", bytes);
		return;
	}

	const size_t packet_count = static_cast<size_t>(bytes) / packet_size;

	for (size_t i = 0; i < packet_count; i++) {
		crsf_raw_data_t crsf{};
		memcpy(&crsf, data + i * packet_size, packet_size);

		if (crsf.magic_number != CRSF_RAW_MAGIC_NUMBER) {
			PX4_WARN("Invalid CRSF control magic in packet %zu", i);
			continue;
		}

		if (crsf.len == 0 || crsf.len > sizeof(crsf.data)) {
			PX4_WARN("Invalid CRSF control length in packet %zu: %u", i, crsf.len);
			continue;
		}

		crsf_raw_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.len = static_cast<uint8_t>(crsf.len);
		memcpy(msg.data, crsf.data, crsf.len);

		if (bridge->_crsf_raw_tx_pub == nullptr) {
			bridge->_crsf_raw_tx_pub = orb_advertise(ORB_ID(crsf_raw_tx), &msg);

			if (bridge->_crsf_raw_tx_pub == nullptr) {
				PX4_ERR("crsf_raw_tx advertise failed");
			}

		} else if (orb_publish(ORB_ID(crsf_raw_tx), bridge->_crsf_raw_tx_pub, &msg) != PX4_OK) {
			PX4_ERR("crsf_raw_tx publish failed");
		}
	}
}

int CrsfBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CrsfBridge::task_spawn(int argc, char *argv[])
{
	CrsfBridge *instance = new CrsfBridge();

	if (instance != nullptr) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int CrsfBridge::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Forwards raw CRSF frames between uORB and the bidirectional crsf_raw MPA pipe.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("crsf_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int crsf_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(CrsfBridge::desc, argc, argv);
}
