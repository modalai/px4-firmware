/****************************************************************************
 * Copyright (c) 2026 ModalAI, Inc. All rights reserved.
 * Solution-separation monitor for ekf2zai: chi-square block tests + CUSUM on
 * the companion separation, re-clone window policy, EV quarantine/failover.
 ****************************************************************************/

#pragma once

#include "ekf.h"

#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_solution_separation.h>

class SolutionSeparation
{
public:
	void update(Ekf &ekf, uint64_t now_us);

private:
	static constexpr int NUM_BLOCKS = 5; // pos_h, pos_z, vel_h, vel_z, yaw
	static constexpr float CUSUM_DRIFT = 1.5f;
	static constexpr float CUSUM_ALARM = 8.f;
	static constexpr uint8_t ALERT_PERSIST = 5;

	void refreshParams(uint64_t now_us);
	static float probit(float p); // inverse standard normal CDF

	param_t _h_ctrl{PARAM_INVALID};
	param_t _h_pfa{PARAM_INVALID};
	param_t _h_win{PARAM_INVALID};
	param_t _h_sig{PARAM_INVALID};
	param_t _h_quar{PARAM_INVALID};

	int32_t _ctrl{0};
	float _pfa{1e-4f};
	float _win{30.f};
	float _sig{1.5f};
	float _quar{10.f};

	uint64_t _last_param_refresh_us{0};

	uint8_t _state{0}; // estimator_solution_separation_s::MONITOR_STATE_*
	uint8_t _alert_count{0};
	uint64_t _purge_time_us{0};
	float _cusum[NUM_BLOCKS] {};

	uORB::Publication<estimator_solution_separation_s> _pub{ORB_ID(estimator_solution_separation)};
};
