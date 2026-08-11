/****************************************************************************
 * Copyright (c) 2026 ModalAI, Inc. All rights reserved.
 * Dual-output companion filter for EKF2 — full exact recursion.
 *
 * Joint second moments of the pair (System B primary, System A EV-free) in
 * difference coordinates: D = Cov(d), C = E[e_B d'] with d = x_B [-] x_A.
 * No optimality assumptions: the recursions carry clipped gains, gating
 * asymmetry and hard resets exactly (to first order in the error state).
 *
 *   e_B+ = (I - K_b H') e_B - K_b v
 *   d+   = (I - K_a H') d  - dK (H'e_B + v),          dK = K_a - K_b
 *   D+   = (I-K_a H')D(I-K_a H')' - (I-K_a H')C'H dK' - dK H'C (I-K_a H')'
 *          + dK S_b dK'
 *   C+   = (I-K_b H')C(I-K_a H')' - (P H - K_b S_b) dK'
 *
 * The last term of C+ is the non-optimal-gain correction: it vanishes iff
 * K_b is the Kalman gain, which EKF2 routinely violates (inhibited-state
 * gain clearing). Hard resets are the same primitive with unit block gains
 * and the PRE-reset P column — exact, no special cases.
 *
 * Propagation: both systems share the IMU noise, so D and C propagate
 * WITHOUT Q: X <- F X F'. F is the sparse error-state transition matching
 * EKF/python/ekf_derivation/derivation.py exactly (dt^2 terms dropped
 * there, hence no right-Jacobian correction):
 *   F[th,bg] = -R dt, F[v,th] = -skew(R dv_corr), F[v,ba] = -R dt,
 *   F[p,v] = I dt, identity elsewhere.
 ****************************************************************************/

#include "dual_companion.h"

#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

using namespace estimator;
using matrix::AxisAnglef;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Dcmf;

namespace
{
// X <- F X F' for the sparse F above. In-place, two passes (rows then cols).
void applyF(matrix::SquareMatrix<float, State::size> &X, const Dcmf &R, const Vector3f &dv_ef, const float dt)
{
	constexpr unsigned IT = State::quat_nominal.idx;	// 0
	constexpr unsigned IV = State::vel.idx;			// 3
	constexpr unsigned IP = State::pos.idx;			// 6
	constexpr unsigned IBG = State::gyro_bias.idx;		// 9
	constexpr unsigned IBA = State::accel_bias.idx;		// 12
	constexpr unsigned N = State::size;

	matrix::SquareMatrix3f Vx;
	Vx(0, 0) = 0.f;        Vx(0, 1) = -dv_ef(2); Vx(0, 2) = dv_ef(1);
	Vx(1, 0) = dv_ef(2);   Vx(1, 1) = 0.f;       Vx(1, 2) = -dv_ef(0);
	Vx(2, 0) = -dv_ef(1);  Vx(2, 1) = dv_ef(0);  Vx(2, 2) = 0.f;

	// rows pass: new_row(th) = row(th) - dt*R*row(bg)
	//            new_row(v)  = row(v) - Vx*row(th) - dt*R*row(ba)
	//            new_row(p)  = row(p) + dt*row(v)
	float old_t[3][N], old_v[3][N];

	for (unsigned j = 0; j < N; j++) {
		for (unsigned k = 0; k < 3; k++) {
			old_t[k][j] = X(IT + k, j);
			old_v[k][j] = X(IV + k, j);
		}
	}

	for (unsigned j = 0; j < N; j++) {
		for (unsigned k = 0; k < 3; k++) {
			float acc_t = old_t[k][j];
			float acc_v = old_v[k][j];

			for (unsigned m = 0; m < 3; m++) {
				acc_t -= dt * R(k, m) * X(IBG + m, j);
				acc_v -= Vx(k, m) * old_t[m][j] + dt * R(k, m) * X(IBA + m, j);
			}

			X(IT + k, j) = acc_t;
			X(IV + k, j) = acc_v;
		}

		for (unsigned k = 0; k < 3; k++) {
			X(IP + k, j) += dt * old_v[k][j];
		}
	}

	// cols pass (right-multiply by F'): mirror operations on columns
	for (unsigned i = 0; i < N; i++) {
		for (unsigned k = 0; k < 3; k++) {
			old_t[k][i] = X(i, IT + k);
			old_v[k][i] = X(i, IV + k);
		}
	}

	for (unsigned i = 0; i < N; i++) {
		for (unsigned k = 0; k < 3; k++) {
			float acc_t = old_t[k][i];
			float acc_v = old_v[k][i];

			for (unsigned m = 0; m < 3; m++) {
				acc_t -= dt * X(i, IBG + m) * R(k, m);
				acc_v -= old_t[m][i] * Vx(k, m) + dt * X(i, IBA + m) * R(k, m);
			}

			X(i, IT + k) = acc_t;
			X(i, IV + k) = acc_v;
		}

		for (unsigned k = 0; k < 3; k++) {
			X(i, IP + k) += dt * old_v[k][i];
		}
	}
}
} // namespace

void DualCompanion::predictCovariance(const StateSample &state_b_pre, const imuSample &imu_delayed)
{
	const float dt = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

	if (dt < 1e-6f) {
		return;
	}

	// Jacobian evaluated at B's pre-prediction state, same point as the
	// generated covariance prediction (see derivation.py)
	const Dcmf R(state_b_pre.quat_nominal);
	const Vector3f accel = imu_delayed.delta_vel / imu_delayed.delta_vel_dt - state_b_pre.accel_bias;
	const Vector3f dv_ef = R * accel * dt;

	if (_D_active) {
		applyF(_D, R, dv_ef, dt);
	}

	if (_C_active) {
		applyF(_C, R, dv_ef, dt);
	}

	if (!_D_active) {
		return;
	}

	// D is symmetric PSD in exact arithmetic — enforce in float32
	for (unsigned i = 0; i < N; i++) {
		if (_D(i, i) < 0.f) {
			_D(i, i) = 0.f;
		}

		for (unsigned j = 0; j < i; j++) {
			const float m = 0.5f * (_D(i, j) + _D(j, i));
			_D(i, j) = m;
			_D(j, i) = m;
		}
	}
}

void DualCompanion::predictState(const imuSample &imu_delayed, const float gravity,
				 const Vector3f &earth_rate_ned, const float vel_limit,
				 const bool zero_z_vel_on_clipping)
{
	// mirror of Ekf::predictState() on the companion mean
	const Vector3f delta_ang_bias_scaled = _state_a.gyro_bias * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;
	corrected_delta_ang -= _R_to_earth_a.transpose() * earth_rate_ned * imu_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});
	_state_a.quat_nominal = (_state_a.quat_nominal * dq).normalized();
	_R_to_earth_a = Dcmf(_state_a.quat_nominal);

	const Vector3f delta_vel_bias_scaled = _state_a.accel_bias * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth_a * corrected_delta_vel;

	const Vector3f vel_last = _state_a.vel;
	_state_a.vel += corrected_delta_vel_ef;

	const Vector3f gravity_acceleration(0.f, 0.f, gravity);
	const Vector3f coriolis_acceleration = -2.f * earth_rate_ned.cross(vel_last);
	const Vector3f transport_rate = -_gpos_a.computeAngularRateNavFrame(vel_last).cross(vel_last);
	_state_a.vel += (gravity_acceleration + coriolis_acceleration + transport_rate) * imu_delayed.delta_vel_dt;

	if (zero_z_vel_on_clipping && _state_a.vel(2) > 0.f) {
		_state_a.vel(2) = 0.f;
	}

	_gpos_a += (vel_last + _state_a.vel) * imu_delayed.delta_vel_dt * 0.5f;
	_state_a.pos.zero();
	_state_a.pos(2) = -_gpos_a.altitude();

	_state_a.vel = matrix::constrain(_state_a.vel, -vel_limit, vel_limit);
}

void DualCompanion::refreshSeparation(const StateSample &state_b, const LatLonAlt &gpos_b)
{
	// attitude: global (left) error convention, d_att = Log(q_B * q_A^-1)
	const Quatf q_err((state_b.quat_nominal * _state_a.quat_nominal.inversed()).normalized());
	const AxisAnglef aa(q_err);
	_d.slice<3, 1>(State::quat_nominal.idx, 0) = Vector3f(aa);

	_d.slice<3, 1>(State::vel.idx, 0) = state_b.vel - _state_a.vel;

	const double dlat = gpos_b.latitude_rad() - _gpos_a.latitude_rad();
	const double dlon = gpos_b.longitude_rad() - _gpos_a.longitude_rad();
	const double clat = cos(gpos_b.latitude_rad());
	_d(State::pos.idx + 0) = static_cast<float>(dlat * CONSTANTS_RADIUS_OF_EARTH);
	_d(State::pos.idx + 1) = static_cast<float>(dlon * clat * CONSTANTS_RADIUS_OF_EARTH);
	_d(State::pos.idx + 2) = -(gpos_b.altitude() - _gpos_a.altitude());

	_d.slice<3, 1>(State::gyro_bias.idx, 0) = state_b.gyro_bias - _state_a.gyro_bias;
	_d.slice<3, 1>(State::accel_bias.idx, 0) = state_b.accel_bias - _state_a.accel_bias;
#if defined(CONFIG_EKF2_MAGNETOMETER)
	_d.slice<3, 1>(State::mag_I.idx, 0) = state_b.mag_I - _state_a.mag_I;
	_d.slice<3, 1>(State::mag_B.idx, 0) = state_b.mag_B - _state_a.mag_B;
#endif
#if defined(CONFIG_EKF2_WIND)
	_d.slice<2, 1>(State::wind_vel.idx, 0) = state_b.wind_vel - _state_a.wind_vel;
#endif
#if defined(CONFIG_EKF2_TERRAIN)
	_d(State::terrain.idx) = state_b.terrain - _state_a.terrain;
#endif
}

void DualCompanion::computeCaches(const VectorState &H)
{
	// u = D H, c1 = C H, c2 = C' H, HDH = H'DH, sc = H'CH
	// dormant matrices are identically zero — skip their passes (exact)
	if (_D_active && _C_active) {
		for (unsigned i = 0; i < N; i++) {
			float au = 0.f, a1 = 0.f, a2 = 0.f;

			for (unsigned j = 0; j < N; j++) {
				au += _D(i, j) * H(j);
				a1 += _C(i, j) * H(j);
				a2 += _C(j, i) * H(j);
			}

			_u(i) = au;
			_c1(i) = a1;
			_c2(i) = a2;
		}

	} else if (_D_active) {
		for (unsigned i = 0; i < N; i++) {
			float au = 0.f;

			for (unsigned j = 0; j < N; j++) {
				au += _D(i, j) * H(j);
			}

			_u(i) = au;
		}

		_c1.zero();
		_c2.zero();

	} else {
		_u.zero();
		_c1.zero();
		_c2.zero();
	}

	_HDH = _D_active ? _u.dot(H) : 0.f;
	_sc = _C_active ? _c1.dot(H) : 0.f;
}

void DualCompanion::prepareCorrection(const VectorState &H, const VectorState &PH, const float S_b,
				      const float innov_b, VectorState &K_a, float &innov_a, float &S_a)
{
	computeCaches(H);

	// S_a = H' P_aa H + R = S_b + H'DH + 2 H'CH
	S_a = math::max(S_b + _HDH + 2.f * _sc, 1e-12f);
	innov_a = innov_b - H.dot(_d);

	if (_mode == Mode::ExclusiveB) {
		K_a.zero();
		return;
	}

	// A-side innovation gate (5 sigma): A can reject without blocking B;
	// the asymmetry is carried exactly by the (C, D) recursion
	if (innov_a * innov_a > 25.f * S_a) {
		K_a.zero();
		return;
	}

	// K_a = P_aa H / S_a = (P_bb H + C H + C'H + D H) / S_a
	for (unsigned i = 0; i < N; i++) {
		K_a(i) = (PH(i) + _c1(i) + _c2(i) + _u(i)) / S_a;
	}
}

void DualCompanion::applyCorrection(const VectorState &H, const VectorState &PH, const VectorState &K_b,
				    const VectorState &K_a, const float S_b, const float innov_b,
				    const float innov_a, const MeanUpdateContext &ctx)
{
	VectorState dK;
	bool a_updates = false;
	bool dk_nonzero = false;
	bool kb_suboptimal = false;

	for (unsigned i = 0; i < N; i++) {
		dK(i) = K_a(i) - K_b(i);

		if (fabsf(K_a(i)) > 0.f) {
			a_updates = true;
		}

		if (fabsf(dK(i)) > 0.f) {
			dk_nonzero = true;
		}

		// clearInhibitedStateKalmanGains only ever ZEROES entries, and all
		// EKF2 callers build K_b = PH/S_b: a zeroed row against nonzero PH
		// is the exact signature of a suboptimal gain (also catches the
		// deliberate unit-gain resets). No epsilons involved.
		if (!(fabsf(K_b(i)) > 0.f) && (fabsf(PH(i)) > 0.f)) {
			kb_suboptimal = true;
		}
	}

	if (!dk_nonzero && !_D_active && !_C_active) {
		// identical gains on an identical pair: moments unchanged (exact)
		for (unsigned i = 0; i < N; i++) {
			_d(i) += -K_b(i) * innov_b + K_a(i) * innov_a;
		}

		if (a_updates) {
			applyCorrectionToA(K_a, innov_a, ctx);
		}

		return;
	}

	const bool c_was_active = _C_active;

	if (dk_nonzero) {
		_D_active = true;
	}

	if (_D_active) {
		if (c_was_active) {
			// w = (I - K_a H') C' H = c2 - K_a * sc
			VectorState w;

			for (unsigned i = 0; i < N; i++) {
				w(i) = _c2(i) - K_a(i) * _sc;
			}

			// D+ = D - K_a u' - u K_a' + K_a HDH K_a' + dK S_b dK' - w dK' - dK w'
			for (unsigned i = 0; i < N; i++) {
				const float a_i = K_a(i) * _HDH;
				const float s_i = dK(i) * S_b;

				for (unsigned j = 0; j <= i; j++) {
					const float v = _D(i, j)
							- K_a(i) * _u(j) - _u(i) * K_a(j)
							+ a_i * K_a(j)
							+ s_i * dK(j)
							- w(i) * dK(j) - dK(i) * w(j);
					_D(i, j) = v;
					_D(j, i) = v;
				}
			}

		} else {
			// C dormant: the w-terms vanish identically
			for (unsigned i = 0; i < N; i++) {
				const float a_i = K_a(i) * _HDH;
				const float s_i = dK(i) * S_b;

				for (unsigned j = 0; j <= i; j++) {
					const float v = _D(i, j)
							- K_a(i) * _u(j) - _u(i) * K_a(j)
							+ a_i * K_a(j)
							+ s_i * dK(j);
					_D(i, j) = v;
					_D(j, i) = v;
				}
			}
		}

		for (unsigned i = 0; i < N; i++) {
			if (_D(i, i) < 0.f) {
				_D(i, i) = 0.f;
			}
		}
	}

	// C+ = C - K_b c2' - c1 K_a' + K_b sc K_a' - (PH - K_b S_b) dK'
	// While dormant, C can only be seeded by the last term, which requires a
	// suboptimal K_b AND dk != 0; otherwise it stays zero to within the
	// rounding of g = PH - K_b*S_b (below float update noise).
	if (c_was_active || (kb_suboptimal && dk_nonzero)) {
		_C_active = true;

		if (c_was_active) {
			for (unsigned i = 0; i < N; i++) {
				const float g_i = PH(i) - K_b(i) * S_b;
				const float b_i = K_b(i) * _sc;

				for (unsigned j = 0; j < N; j++) {
					_C(i, j) += -K_b(i) * _c2(j) - _c1(i) * K_a(j) + b_i * K_a(j) - g_i * dK(j);
				}
			}

		} else {
			// first seed: C = -g dK' (rank one)
			for (unsigned i = 0; i < N; i++) {
				const float g_i = PH(i) - K_b(i) * S_b;

				if (fabsf(g_i) > 0.f) {
					for (unsigned j = 0; j < N; j++) {
						_C(i, j) = -g_i * dK(j);
					}
				}
			}
		}
	}

	// separation mean: d+ = d - K_b nu_b + K_a nu_a  (x <- x [-] K nu convention)
	for (unsigned i = 0; i < N; i++) {
		_d(i) += -K_b(i) * innov_b + K_a(i) * innov_a;
	}

	if (a_updates) {
		applyCorrectionToA(K_a, innov_a, ctx);
	}
}

void DualCompanion::applyCorrectionToA(const VectorState &K, const float innovation, const MeanUpdateContext &ctx)
{
	// mirror of Ekf::fuse() on the companion mean
	const Quatf delta_quat(AxisAnglef(K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx, 0) *
					  (-1.f * innovation)));
	_state_a.quat_nominal = delta_quat * _state_a.quat_nominal;
	_state_a.quat_nominal.normalize();
	_R_to_earth_a = Dcmf(_state_a.quat_nominal);

	_state_a.vel = matrix::constrain(_state_a.vel - K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation,
					 -1.e3f, 1.e3f);

	const Vector3f pos_correction = K.slice<State::pos.dof, 1>(State::pos.idx, 0) * (-innovation);
	_gpos_a += pos_correction;
	_state_a.pos.zero();
	_state_a.pos(2) = -_gpos_a.altitude();

	_state_a.gyro_bias = matrix::constrain(_state_a.gyro_bias -
					       K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx, 0) * innovation,
					       -ctx.gyro_bias_limit, ctx.gyro_bias_limit);
	_state_a.accel_bias = matrix::constrain(_state_a.accel_bias -
						K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx, 0) * innovation,
						-ctx.accel_bias_limit, ctx.accel_bias_limit);

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (ctx.mag_states_active) {
		_state_a.mag_I = matrix::constrain(_state_a.mag_I - K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation,
						   -1.f, 1.f);
		_state_a.mag_B = matrix::constrain(_state_a.mag_B - K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation,
						   -ctx.mag_bias_limit, ctx.mag_bias_limit);
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	if (ctx.wind_states_active) {
		_state_a.wind_vel = matrix::constrain(_state_a.wind_vel -
						      K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) * innovation,
						      -1.e2f, 1.e2f);
	}

#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	_state_a.terrain = math::constrain(_state_a.terrain - K(State::terrain.idx) * innovation, -1e4f, 1e4f);
#endif
}

void DualCompanion::onResetB(const SquareMatrixState &P_pre, const unsigned idx, const unsigned dof,
			     const float delta[], const float var[], const MeanUpdateContext &ctx)
{
	if (!_enabled || !_initialized) {
		return;
	}

	// A hard reset of axis i to a measurement m with variance R_m is the
	// correction primitive with unit gain: K_b = e_i, nu_b = -delta,
	// S = P_ii(pre) + R_m. Shared-source resets also reset A (K_a = e_i);
	// EV-class resets leave A untouched (K_a = 0). Exact — see header.
	for (unsigned k = 0; k < dof; k++) {
		const unsigned i = idx + k;
		const float var_pre = P_pre(i, i);
		const float r_m = PX4_ISFINITE(var[k]) ? math::max(var[k], 0.f) : var_pre;
		const float s = math::max(var_pre + r_m, 1e-12f);
		const float nu_b = -delta[k];

		VectorState H;
		H(i) = 1.f;
		VectorState PH;

		for (unsigned j = 0; j < N; j++) {
			PH(j) = P_pre(j, i);
		}

		computeCaches(H);

		VectorState K_b;
		K_b(i) = 1.f;
		VectorState K_a;
		float nu_a = nu_b - _d(i);

		if (_mode != Mode::ExclusiveB) {
			K_a(i) = 1.f; // A resets to the same observation
		}

		applyCorrection(H, PH, K_b, K_a, s, nu_b, nu_a, ctx);
	}
}

void DualCompanion::onResetYawB(const SquareMatrixState &P_pre, const float yaw_delta, const float yaw_var,
				const MeanUpdateContext &ctx)
{
	if (!_enabled || !_initialized) {
		return;
	}

	constexpr unsigned IY = State::quat_nominal.idx + 2;
	const float delta[1] = {yaw_delta};
	const float var[1] = {yaw_var};
	onResetB(P_pre, IY, 1, delta, var, ctx);
}

void DualCompanion::syncStatesB(const StateSample &state_b, const unsigned idx, const unsigned dof)
{
	if (!_enabled || !_initialized) {
		return;
	}

	if (idx == State::mag_I.idx) {
#if defined(CONFIG_EKF2_MAGNETOMETER)
		_state_a.mag_I = state_b.mag_I;
#endif

	} else if (idx == State::mag_B.idx) {
#if defined(CONFIG_EKF2_MAGNETOMETER)
		_state_a.mag_B = state_b.mag_B;
#endif

	} else if (idx == State::wind_vel.idx) {
#if defined(CONFIG_EKF2_WIND)
		_state_a.wind_vel = state_b.wind_vel;
#endif

	} else if (idx == State::terrain.idx) {
#if defined(CONFIG_EKF2_TERRAIN)
		_state_a.terrain = state_b.terrain;
#endif
	}

	for (unsigned k = 0; k < dof; k++) {
		const unsigned i = idx + k;
		_d(i) = 0.f;

		for (unsigned j = 0; j < N; j++) {
			_D(i, j) = 0.f;
			_D(j, i) = 0.f;
			_C(i, j) = 0.f;
			_C(j, i) = 0.f;
		}
	}
}
