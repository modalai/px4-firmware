/****************************************************************************
 * Copyright (c) 2026 ModalAI, Inc. All rights reserved.
 ****************************************************************************/

#include "solution_separation.hpp"

#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

using namespace estimator;

float SolutionSeparation::probit(float p)
{
	// Acklam-style rational approximation, adequate for tail thresholds
	p = math::constrain(p, 1e-9f, 1.f - 1e-9f);

	if (p > 0.5f) {
		return -probit(1.f - p);
	}

	const float t = sqrtf(-2.f * logf(p));
	return -(t - (2.30753f + 0.27061f * t) / (1.f + t * (0.99229f + 0.04481f * t)));
}

void SolutionSeparation::refreshParams(const uint64_t now_us)
{
	if ((now_us - _last_param_refresh_us) < 1_s && (_last_param_refresh_us != 0)) {
		return;
	}

	_last_param_refresh_us = now_us;

	if (_h_ctrl == PARAM_INVALID) {
		_h_ctrl = param_find("EKF2_SS_CTRL");
		_h_pfa = param_find("EKF2_SS_PFA");
		_h_win = param_find("EKF2_SS_WIN");
		_h_sig = param_find("EKF2_SS_SIG");
		_h_quar = param_find("EKF2_SS_QUAR");
	}

	param_get(_h_ctrl, &_ctrl);
	param_get(_h_pfa, &_pfa);
	param_get(_h_win, &_win);
	param_get(_h_sig, &_sig);
	param_get(_h_quar, &_quar);
}

void SolutionSeparation::update(Ekf &ekf, const uint64_t now_us)
{
	refreshParams(now_us);

	if ((_ctrl & 1) == 0) {
		_state = estimator_solution_separation_s::MONITOR_STATE_DISABLED;
		return;
	}

	if (!ekf.companion().enabled()) {
		ekf.enableCompanion();
	}

	if (!ekf.companion().initialized()) {
		return;
	}

	const auto &d = ekf.companion().separation();
	const auto &D = ekf.companion().D();

	constexpr unsigned IP = State::pos.idx;
	constexpr unsigned IV = State::vel.idx;
	constexpr unsigned IY = State::quat_nominal.idx + 2;
	constexpr float VAR_FLOOR = 1e-6f;

	// block statistics q = d' inv(D_blk) d (2x2 closed form, scalars direct)
	float q[NUM_BLOCKS] {};

	{
		const float a = D(IP, IP) + VAR_FLOOR, b = D(IP, IP + 1), c = D(IP + 1, IP + 1) + VAR_FLOOR;
		const float det = math::max(a * c - b * b, 1e-12f);
		q[0] = (c * d(IP) * d(IP) - 2.f * b * d(IP) * d(IP + 1) + a * d(IP + 1) * d(IP + 1)) / det;
	}
	q[1] = d(IP + 2) * d(IP + 2) / (D(IP + 2, IP + 2) + VAR_FLOOR);
	{
		const float a = D(IV, IV) + VAR_FLOOR, b = D(IV, IV + 1), c = D(IV + 1, IV + 1) + VAR_FLOOR;
		const float det = math::max(a * c - b * b, 1e-12f);
		q[2] = (c * d(IV) * d(IV) - 2.f * b * d(IV) * d(IV + 1) + a * d(IV + 1) * d(IV + 1)) / det;
	}
	q[3] = d(IV + 2) * d(IV + 2) / (D(IV + 2, IV + 2) + VAR_FLOOR);
	q[4] = d(IY) * d(IY) / (D(IY, IY) + VAR_FLOOR);

	// thresholds: chi2 inverse — 1 dof: probit(1 - a/2)^2, 2 dof: -2 ln a
	const float alpha = math::constrain(_pfa / NUM_BLOCKS, 1e-9f, 0.1f);
	const float z = probit(1.f - 0.5f * alpha);
	const float t1 = z * z;
	const float t2 = -2.f * logf(alpha);
	const float thresh[NUM_BLOCKS] = {t2, t1, t2, t1, t1};

	bool alarm = false;
	float test_ratio[NUM_BLOCKS];

	for (int i = 0; i < NUM_BLOCKS; i++) {
		test_ratio[i] = q[i] / thresh[i];

		_cusum[i] = math::constrain(_cusum[i] + sqrtf(math::max(q[i], 0.f)) - CUSUM_DRIFT, 0.f, 3.f * CUSUM_ALARM);

		if ((test_ratio[i] > 1.f) || (_cusum[i] > CUSUM_ALARM)) {
			alarm = true;
		}
	}

	const float age_s = 1e-6f * static_cast<float>(ekf.time_delayed_us() - ekf.companion().cloneTime());
	const float sigma_ph = sqrtf(0.5f * (D(IP, IP) + D(IP + 1, IP + 1)));

	switch (_state) {
	default:
	case estimator_solution_separation_s::MONITOR_STATE_DISABLED:
		_state = estimator_solution_separation_s::MONITOR_STATE_NOMINAL;

	// fall through
	case estimator_solution_separation_s::MONITOR_STATE_NOMINAL:
		if (alarm) {
			_state = estimator_solution_separation_s::MONITOR_STATE_ALERT;
			_alert_count = 1;

		} else if ((sigma_ph > _sig) || ((_win > 0.1f) && (age_s > _win))) {
			// finite-window policy: bound A's dead-reckoning growth
			ekf.companion().requestClone();

			for (auto &g : _cusum) { g = 0.f; }
		}

		break;

	case estimator_solution_separation_s::MONITOR_STATE_ALERT:
		if (alarm) {
			if (++_alert_count >= ALERT_PERSIST) {
				if (_ctrl & 4) {
					// failover: quarantine EV, adopt the clean companion
					ekf.resetToCompanion();
					_purge_time_us = now_us;
					_state = estimator_solution_separation_s::MONITOR_STATE_PURGED;

					for (auto &g : _cusum) { g = 0.f; }
				}

				// detect-only: hold in ALERT (visible in the topic)
			}

		} else {
			_state = estimator_solution_separation_s::MONITOR_STATE_NOMINAL;
			_alert_count = 0;
		}

		break;

	case estimator_solution_separation_s::MONITOR_STATE_PURGED:
		if ((now_us - _purge_time_us) > static_cast<uint64_t>(_quar * 1e6f)) {
			ekf.setEvFusionInhibited(false);
			_state = estimator_solution_separation_s::MONITOR_STATE_NOMINAL;
			_alert_count = 0;
		}

		break;
	}

	if (_ctrl & 2) {
		estimator_solution_separation_s msg{};
		msg.timestamp_sample = ekf.time_delayed_us();

		for (int i = 0; i < 3; i++) {
			msg.d_pos[i] = d(IP + i);
			msg.d_vel[i] = d(IV + i);
			msg.sigma_pos[i] = sqrtf(math::max(D(IP + i, IP + i), 0.f));
			msg.sigma_vel[i] = sqrtf(math::max(D(IV + i, IV + i), 0.f));
		}

		msg.d_yaw = d(IY);
		msg.sigma_yaw = sqrtf(math::max(D(IY, IY), 0.f));

		for (int i = 0; i < NUM_BLOCKS; i++) {
			msg.test_ratio[i] = test_ratio[i];
			msg.cusum[i] = _cusum[i];
		}

		msg.window_age_s = age_s;
		msg.clone_count = ekf.companion().cloneCount();
		msg.monitor_state = _state;
		msg.ev_inhibited = ekf.isEvFusionInhibited();
		msg.timestamp = hrt_absolute_time();
		_pub.publish(msg);
	}
}
