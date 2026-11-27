/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "CrsfRcSim.hpp"

using namespace time_literals;

ModuleBase::Descriptor CrsfRcSim::desc{task_spawn, custom_command, print_usage};

// Name of the debug_vect that carries the simulated link state
static const char SIM_LINK_NAME[] = "RC_SIMLK";

int CrsfRcSim::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device_name) {
		PX4_ERR("Valid device required");
		return PX4_ERROR;
	}

	if (board_rc_conflicting(device_name)) {
		PX4_INFO("unable to start, conflict with PX4IO on %s", device_name);
		return PX4_ERROR;
	}

	CrsfRcSim *instance = new CrsfRcSim(device_name);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	PX4_INFO("simulated RC link enabled: link state from %s", SIM_LINK_NAME);

	instance->ScheduleNow();

	return PX4_OK;
}

void CrsfRcSim::BeginCycle(const hrt_abstime &now)
{
	// Once per cycle, before the UART is read: take new link messages
	UpdateSimLink(now);
}

bool CrsfRcSim::DropFrame()
{
	if (_sim_link_state != SimLinkState::lost) {
		return false;
	}

	// as if the receiver had lost the transmitter: nothing reaches the driver
	_sim_link_frames_dropped++;
	return true;
}

void CrsfRcSim::AdjustInputRc(input_rc_s &input_rc, const hrt_abstime &now)
{
	if (_sim_link_state == SimLinkState::ok && (now - _last_packet_seen <= 500_ms)) {
		// Simulated link ok and the real link statistics valid: show the
		// simulated link quality. A real loss still wins (statistics invalid).
		if (PX4_ISFINITE(_sim_link_lq)) {
			input_rc.link_quality = (int8_t)_sim_link_lq;
		}

		if (PX4_ISFINITE(_sim_link_rssi_dbm)) {
			input_rc.rssi_dbm = _sim_link_rssi_dbm;

			if (now - _last_stats_tx_seen > 500_ms) {
				input_rc.rssi = (1.f - input_rc.rssi_dbm / -130.f) * input_rc.RSSI_MAX;
			}
		}
	}
}

const char *CrsfRcSim::SimLinkStateName(SimLinkState s)
{
	switch (s) {
	case SimLinkState::off:
		return "off";

	case SimLinkState::stale:
		return "stale (real link)";

	case SimLinkState::ok:
		return "ok";

	case SimLinkState::lost:
		return "lost";
	}

	return "?";
}

void CrsfRcSim::UpdateSimLink(const hrt_abstime &now)
{
	debug_vect_s dv;

	while (_debug_vect_sub.update(&dv)) {
		if (strncmp(dv.name, SIM_LINK_NAME, sizeof(dv.name)) != 0) {
			continue; // someone else's debug vector
		}

		// x: 1 lost, 0 ok (anything else rejects the message);
		// y: LQ 0..100 %, z: RSSI -150..0 dBm (out of range: not shown)
		const bool is_ok = PX4_ISFINITE(dv.x) && fabsf(dv.x) < 0.01f;
		const bool is_lost = PX4_ISFINITE(dv.x) && fabsf(dv.x - 1.f) < 0.01f;

		if (!is_ok && !is_lost) {
			_sim_link_bad++;
			continue;
		}

		_sim_link_msgs++;
		_sim_link_last = now; // the arrival time here, not the sender's clock
		_sim_link_lost = is_lost;
		_sim_link_lq = (PX4_ISFINITE(dv.y) && dv.y >= 0.f && dv.y <= 100.f) ? dv.y : NAN;
		_sim_link_rssi_dbm = (PX4_ISFINITE(dv.z) && dv.z >= -150.f && dv.z <= 0.f) ? dv.z : NAN;
	}

	SimLinkState s = SimLinkState::stale;

	if (_sim_link_last != 0 && now - _sim_link_last < SIM_LINK_TIMEOUT) {
		s = _sim_link_lost ? SimLinkState::lost : SimLinkState::ok;
	}

	if (s != _sim_link_state) {
		if (s == SimLinkState::lost) {
			_sim_link_losses++;
		}

		PX4_INFO("simulated RC link: %s -> %s", SimLinkStateName(_sim_link_state), SimLinkStateName(s));
		_sim_link_state = s;
	}
}

int CrsfRcSim::print_status()
{
	CrsfRc::print_status();

	PX4_INFO("Simulated link: %s, LQ %.0f, RSSI %.0f dBm; %" PRIu32 " messages, %" PRIu32 " rejected, %"
		 PRIu32 " losses, %" PRIu32 " frames dropped", SimLinkStateName(_sim_link_state), (double)_sim_link_lq,
		 (double)_sim_link_rssi_dbm, _sim_link_msgs, _sim_link_bad, _sim_link_losses, _sim_link_frames_dropped);

	return 0;
}

int CrsfRcSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CrsfRcSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The crsf_rc driver with a simulated RC link, for the voxl-mvx-simulator sim mode only.
Start it instead of crsf_rc (never both). The link loss and link quality come from the
debug_vect named "RC_SIMLK"; while it says lost, every receiver frame is dropped so that
the real RC-loss failsafe runs. Without such messages it behaves as crsf_rc.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("crsf_rc_sim", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("radio_control");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int crsf_rc_sim_main(int argc, char *argv[])
{
	return ModuleBase::main(CrsfRcSim::desc, argc, argv);
}
