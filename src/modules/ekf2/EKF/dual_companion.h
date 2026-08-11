/****************************************************************************
 * Copyright (c) 2026 ModalAI, Inc. All rights reserved.
 * Dual-output companion filter ("solution separation") for EKF2.
 *
 * Carries the EV-free shadow estimate (System A) alongside the primary
 * EV-fused filter (System B) in difference coordinates: companion mean x_A,
 * separation d = x_B [-] x_A, its exact covariance D = Cov(d), and the full
 * cross term C = E[e_B d'] — no optimality assumptions (clipped gains,
 * gating asymmetry and hard resets are carried exactly to first order).
 * Shared IMU noise cancels in the separation: D and C propagate without Q.
 * Sign conventions follow EKF2 (innovation = predicted - observed,
 * fusion applies x <- x [-] K*innov). Full formulas in dual_companion.cpp.
 ****************************************************************************/

#ifndef EKF_DUAL_COMPANION_H
#define EKF_DUAL_COMPANION_H

#include "common.h"

#include <ekf_derivation/generated/state.h>
#include <lib/lat_lon_alt/lat_lon_alt.hpp>

namespace estimator
{

class DualCompanion
{
public:
	static constexpr uint8_t N = State::size;
	using VectorState = matrix::Vector<float, State::size>;
	using SquareMatrixState = matrix::SquareMatrix<float, State::size>;

	enum class Mode : uint8_t {
		Shared = 0,	// both systems fuse this correction
		ExclusiveB = 1	// only System B fuses (EV-class); System A untouched
	};

	// scoped tag for EV-class (B-exclusive) fusion sections
	class ExclusiveScope
	{
	public:
		ExclusiveScope(DualCompanion &c, bool active) : _c(c), _active(active)
		{ if (_active) { _prev = _c._mode; _c._mode = Mode::ExclusiveB; } }
		~ExclusiveScope() { if (_active) { _c._mode = _prev; } }
	private:
		DualCompanion &_c;
		bool _active;
		Mode _prev{Mode::Shared};
	};

	void enable() { _enabled = true; }
	void disable() { _enabled = false; _initialized = false; }
	bool enabled() const { return _enabled; }
	bool initialized() const { return _initialized; }
	Mode mode() const { return _mode; }

	void cloneFrom(const StateSample &state_b, const LatLonAlt &gpos_b)
	{
		_state_a = state_b;
		_gpos_a = gpos_b;
		_R_to_earth_a = matrix::Dcmf(_state_a.quat_nominal);
		_D.zero();
		_C.zero();
		_d.zero();
		_D_active = false;
		_C_active = false;
		_initialized = true;
		_clone_count++;
	}

	void setCloneTime(uint64_t t_us) { _clone_time_us = t_us; }
	uint64_t cloneTime() const { return _clone_time_us; }
	uint32_t cloneCount() const { return _clone_count; }
	void requestClone() { _clone_request = true; }
	bool cloneRequested() const { return _clone_request; }
	void clearCloneRequest() { _clone_request = false; }

	// D <- F D F', C <- F C F' (no Q — shared IMU noise cancels); F is the
	// sparse error-state transition evaluated at B's pre-prediction state
	void predictCovariance(const StateSample &state_b_pre, const imuSample &imu_delayed);

	// strapdown propagation of x_A, mirroring Ekf::predictState()
	void predictState(const imuSample &imu_delayed, float gravity, const Vector3f &earth_rate_ned,
			  float vel_limit, bool zero_z_vel_on_clipping);

	// recompute cached d from the two means once per EKF cycle
	void refreshSeparation(const StateSample &state_b, const LatLonAlt &gpos_b);

	struct MeanUpdateContext {
		float gyro_bias_limit{0.4f};
		float accel_bias_limit{0.4f};
		float mag_bias_limit{0.5f};
		bool mag_states_active{false};
		bool wind_states_active{false};
	};

	// --- scalar correction primitive ------------------------------------
	// step 1: A-side gain/innovation from B's pre-update quantities
	// (PH = P_bb H, S_b = H'P_bb H + R). K_a returned unclipped.
	void prepareCorrection(const VectorState &H, const VectorState &PH, float S_b, float innov_b,
			       VectorState &K_a, float &innov_a, float &S_a);

	// step 2: exact (C, D, d, x_A) update. K_b is the gain actually applied
	// to B (post clipping); K_a the (clipped) A gain, zero if A rejects.
	void applyCorrection(const VectorState &H, const VectorState &PH, const VectorState &K_b,
			     const VectorState &K_a, float S_b, float innov_b, float innov_a,
			     const MeanUpdateContext &ctx);

	// --- hard resets = unit-gain corrections (exact) --------------------
	// call BEFORE B's covariance rows are overwritten; P_pre is pre-reset.
	// delta = new - old per axis (error space); var = new variance (NAN ->
	// keep pre-reset variance as the observation variance proxy).
	void onResetB(const SquareMatrixState &P_pre, unsigned idx, unsigned dof,
		      const float delta[], const float var[], const MeanUpdateContext &ctx);
	void onResetYawB(const SquareMatrixState &P_pre, float yaw_delta, float yaw_var,
			 const MeanUpdateContext &ctx);

	// shared-source direct initialization of a state block (mag/wind/terrain):
	// A adopts B's block; separation and its moments vanish on the block
	void syncStatesB(const StateSample &state_b, unsigned idx, unsigned dof);

	// horizontal frame redefinition / origin set: A adopts B's lat/lon
	// (coordinate surgery, not a measurement — exact at any magnitude)
	void syncLatLonB(const LatLonAlt &gpos_b)
	{
		if (!_enabled || !_initialized) { return; }

		_gpos_a.setLatLonDeg(gpos_b.latitude_deg(), gpos_b.longitude_deg());

		for (unsigned k = 0; k < 2; k++) {
			const unsigned i = State::pos.idx + k;
			_d(i) = 0.f;

			for (unsigned j = 0; j < N; j++) {
				_D(i, j) = 0.f; _D(j, i) = 0.f;
				_C(i, j) = 0.f; _C(j, i) = 0.f;
			}
		}
	}

	const StateSample &stateA() const { return _state_a; }
	const LatLonAlt &gposA() const { return _gpos_a; }
	const SquareMatrixState &D() const { return _D; }
	const SquareMatrixState &C() const { return _C; }
	// dormancy: D (resp. C) is identically zero until first activated by an
	// exclusive/asymmetric event (resp. a suboptimal-gain event) — the fast
	// paths keyed on these flags are exact, not approximations
	bool dActive() const { return _D_active; }
	bool cActive() const { return _C_active; }
	const VectorState &separation() const { return _d; }

	// P_aa = P_bb + C + C' + D (sum form — no cancellation)
	float varianceA(const SquareMatrixState &P_bb, unsigned i) const
	{
		return P_bb(i, i) + 2.f * _C(i, i) + _D(i, i);
	}

private:
	void computeCaches(const VectorState &H);
	void applyCorrectionToA(const VectorState &K, float innovation, const MeanUpdateContext &ctx);

	StateSample _state_a{};
	LatLonAlt _gpos_a{0.0, 0.0, 0.f};
	SquareMatrixState _D{};
	SquareMatrixState _C{};
	VectorState _d{};

	matrix::Dcmf _R_to_earth_a{};

	// per-scalar caches set by computeCaches, consumed by applyCorrection
	VectorState _u{};	// D H
	VectorState _c1{};	// C H
	VectorState _c2{};	// C' H
	float _HDH{0.f};	// H'DH
	float _sc{0.f};		// H'CH

	bool _enabled{false};
	bool _initialized{false};
	bool _D_active{false};
	bool _C_active{false};
	bool _clone_request{false};
	Mode _mode{Mode::Shared};
	uint32_t _clone_count{0};
	uint64_t _clone_time_us{0};
};

} // namespace estimator

#endif // !EKF_DUAL_COMPANION_H
