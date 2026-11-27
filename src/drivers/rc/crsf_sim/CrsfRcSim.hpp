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

#pragma once

#include "CrsfRc.hpp" // stock driver, ../crsf_rc

#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_vect.h>

/**
 * crsf_rc with a simulated RC link, for voxl-mvx-simulator's sim mode only.
 *
 * The simulator sends its RF model's link state as MAVLink DEBUG_VECT named
 * "RC_SIMLK" (x: 1 lost, 0 ok; y: LQ %; z: RSSI dBm), which the stock mavlink
 * receiver publishes to debug_vect. While the simulated link is lost, every
 * frame from the receiver is dropped, so the driver sees what a real link loss
 * gives it: no RC frames, rc_lost after 500 ms and invalid link statistics.
 * While it is ok, the simulated LQ and RSSI replace the real ones in input_rc.
 * A stale or absent message (SIM_LINK_TIMEOUT) means the real link only.
 *
 * Everything else is the stock CrsfRc; this class only uses its hooks.
 */
class CrsfRcSim : public CrsfRc
{
public:
	static Descriptor desc;

	explicit CrsfRcSim(const char *device) : CrsfRc(device, desc) {}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

protected:
	void BeginCycle(const hrt_abstime &now) override;
	bool DropFrame() override;
	void AdjustInputRc(input_rc_s &input_rc, const hrt_abstime &now) override;

private:
	static constexpr hrt_abstime SIM_LINK_TIMEOUT{500000}; // us
	enum class SimLinkState : uint8_t { off, stale, ok, lost };

	void UpdateSimLink(const hrt_abstime &now);
	static const char *SimLinkStateName(SimLinkState s);

	uORB::Subscription _debug_vect_sub{ORB_ID(debug_vect)};
	SimLinkState _sim_link_state{SimLinkState::off};
	hrt_abstime _sim_link_last{0};
	bool _sim_link_lost{false};
	float _sim_link_lq{NAN};
	float _sim_link_rssi_dbm{NAN};
	uint32_t _sim_link_msgs{0};
	uint32_t _sim_link_bad{0};
	uint32_t _sim_link_losses{0};
	uint32_t _sim_link_frames_dropped{0};
};
