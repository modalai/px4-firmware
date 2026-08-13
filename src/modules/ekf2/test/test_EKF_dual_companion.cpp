/****************************************************************************
 * Copyright (c) 2026 ModalAI, Inc. All rights reserved.
 *
 * Tests for the dual-output solution-separation companion (System A) carried
 * alongside the primary EKF (System B).
 *  - algebraic exactness of the unit-gain reset primitive (C, D, d)
 *  - hand-coded sparse F consistency against sym::PredictCovariance
 *  - B-invariance: enabling the companion never changes System B
 *  - clone invariance: no EV-class updates -> d = 0, D = 0
 *  - EV fault detection: separation chi-square fires under EV bias
 *  - EV reset continuity: B jumps, A continuous, D absorbs the jump
 ****************************************************************************/

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "EKF/dual_companion.h"
#include <ekf_derivation/generated/predict_covariance.h>
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include <lib/geo/geo.h>

using namespace estimator;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

static constexpr unsigned IP = State::pos.idx;
static constexpr unsigned IV = State::vel.idx;

TEST(DualCompanionAlgebra, exclusiveResetExactness)
{
	DualCompanion c;
	c.enable();

	StateSample s{};
	const LatLonAlt gpos(47.0, 8.0, 100.f);
	c.cloneFrom(s, gpos);

	DualCompanion::SquareMatrixState P;

	for (unsigned i = 0; i < State::size; i++) {
		P(i, i) = 4.f;
	}

	P(IP, IP + 1) = P(IP + 1, IP) = 0.5f;

	DualCompanion::MeanUpdateContext ctx{};

	{
		DualCompanion::ExclusiveScope scope(c, true);
		const float delta[1] = {2.f};
		const float var[1] = {0.25f};
		c.onResetB(P, IP, 1, delta, var, ctx);
	}

	// d absorbs the jump; A untouched
	EXPECT_NEAR(c.separation()(IP), 2.f, 1e-6f);
	EXPECT_FLOAT_EQ(c.gposA().altitude(), 100.f);

	// D_ii+ = D + 2C + P_ii(pre) + R_m = 0 + 0 + 4 + 0.25
	EXPECT_NEAR(c.D()(IP, IP), 4.25f, 1e-5f);

	// C_ii+ = -R_m ; C col i += P col (pre); C row i zeroed
	EXPECT_NEAR(c.C()(IP, IP), -0.25f, 1e-5f);
	EXPECT_NEAR(c.C()(IP + 1, IP), 0.5f, 1e-5f);
	EXPECT_NEAR(c.C()(IP, IP + 1), 0.f, 1e-6f);
}

TEST(DualCompanionAlgebra, sharedResetZeroesBlock)
{
	DualCompanion c;
	c.enable();

	StateSample s{};
	const LatLonAlt gpos(47.0, 8.0, 100.f);
	c.cloneFrom(s, gpos);

	DualCompanion::SquareMatrixState P;

	for (unsigned i = 0; i < State::size; i++) {
		P(i, i) = 4.f;
	}

	DualCompanion::MeanUpdateContext ctx{};

	// precondition: EV-class reset creates nonzero D/C on the axis
	{
		DualCompanion::ExclusiveScope scope(c, true);
		const float delta[1] = {2.f};
		const float var[1] = {0.25f};
		c.onResetB(P, IP, 1, delta, var, ctx);
	}

	ASSERT_GT(c.D()(IP, IP), 1.f);

	// shared-source reset on the same axis: both systems land on the same
	// observation -> separation and its uncertainty vanish on that axis
	const float delta2[1] = {1.5f};
	const float var2[1] = {0.09f};
	c.onResetB(P, IP, 1, delta2, var2, ctx);

	EXPECT_NEAR(c.separation()(IP), 0.f, 1e-5f);
	EXPECT_NEAR(c.D()(IP, IP), 0.f, 1e-5f);
	EXPECT_NEAR(c.C()(IP, IP), 0.f, 1e-4f);

	// A moved (it adopted the same observation): north shift = nu_a = 1.5 m + prior separation 2 m
	const double dlat_m = (c.gposA().latitude_rad() - gpos.latitude_rad()) * CONSTANTS_RADIUS_OF_EARTH;
	EXPECT_NEAR(dlat_m, 3.5, 0.05); // WGS84 vs spherical radius
}

TEST(DualCompanionAlgebra, predictMatchesSymforceJacobian)
{
	DualCompanion c;
	c.enable();

	StateSample s{};
	s.quat_nominal = Quatf(Eulerf(0.17f, -0.35f, 0.7f));
	s.vel = Vector3f(3.f, -2.f, 0.5f);
	s.gyro_bias = Vector3f(0.01f, -0.02f, 0.005f);
	s.accel_bias = Vector3f(0.05f, -0.1f, 0.02f);

	const LatLonAlt gpos(47.0, 8.0, 100.f);
	c.cloneFrom(s, gpos);

	DualCompanion::SquareMatrixState P;

	for (unsigned i = 0; i < State::size; i++) {
		P(i, i) = 2.f + 0.1f * static_cast<float>(i);
	}

	DualCompanion::MeanUpdateContext ctx{};

	// build a non-trivial symmetric D (and C) through EV-class resets
	{
		DualCompanion::ExclusiveScope scope(c, true);
		const float d1[3] = {1.f, -0.5f, 0.25f};
		const float v1[3] = {0.3f, 0.2f, 0.1f};
		c.onResetB(P, IP, 3, d1, v1, ctx);
		const float d2[2] = {0.4f, -0.2f};
		const float v2[2] = {0.05f, 0.05f};
		c.onResetB(P, IV, 2, d2, v2, ctx);
		c.onResetYawB(P, 0.1f, 0.02f, ctx);
	}

	const DualCompanion::SquareMatrixState D_before = c.D();

	imuSample imu{};
	imu.delta_ang = Vector3f(0.02f, -0.01f, 0.03f);
	imu.delta_vel = Vector3f(0.05f, 0.02f, -0.098f);
	imu.delta_ang_dt = 0.01f;
	imu.delta_vel_dt = 0.01f;

	c.predictCovariance(s, imu);

	// oracle: the generated covariance prediction with ZERO noise variances
	// is exactly F D F' (valid for symmetric input; upper triangle mirrored)
	const float dt = 0.5f * (imu.delta_vel_dt + imu.delta_ang_dt);
	const matrix::SquareMatrix<float, State::size> D_oracle =
		sym::PredictCovariance(s.vector(), D_before,
				       imu.delta_vel / imu.delta_vel_dt, Vector3f(0.f, 0.f, 0.f),
				       imu.delta_ang / imu.delta_ang_dt, 0.f, dt);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			// the generated function fills the upper triangle only
			const float ref = (i <= j) ? D_oracle(i, j) : D_oracle(j, i);
			EXPECT_NEAR(c.D()(i, j), ref, 1e-4f * (1.f + fabsf(ref)))
					<< "D mismatch at (" << i << "," << j << ")";
		}
	}
}

class EkfDualCompanionTest : public ::testing::Test
{
public:
	EkfDualCompanionTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	void SetUp() override
	{
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}
};

TEST_F(EkfDualCompanionTest, systemBInvariance)
{
	// two locally-built identical stacks: companion on vs off
	auto ekf_a = std::make_shared<Ekf>();
	SensorSimulator sim_a(ekf_a);
	ekf_a->init(0);
	ekf_a->set_in_air_status(false);
	ekf_a->set_vehicle_at_rest(true);
	ekf_a->enableCompanion();

	auto ekf_ref = std::make_shared<Ekf>();
	SensorSimulator sim_ref(ekf_ref);
	ekf_ref->init(0);
	ekf_ref->set_in_air_status(false);
	ekf_ref->set_vehicle_at_rest(true);

	for (int leg = 0; leg < 3; leg++) {
		sim_a.runSeconds(5);
		sim_ref.runSeconds(5);

		const auto v1 = ekf_a->state().vector();
		const auto v2 = ekf_ref->state().vector();

		for (unsigned i = 0; i < State::size + 1; i++) {
			EXPECT_FLOAT_EQ(v1(i), v2(i)) << "B state diverged at index " << i << " leg " << leg;
		}
	}
}

TEST_F(EkfDualCompanionTest, cloneInvarianceWithoutExclusiveUpdates)
{
	_ekf->enableCompanion();
	_ekf_wrapper.enableGpsFusion();

	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(25);

	ASSERT_TRUE(_ekf->companion().initialized());

	const auto &d = _ekf->companion().separation();
	const auto &D = _ekf->companion().D();

	// No EV-class updates -> the means agree (d ~ 0). D is not identically
	// zero: transient A/B gating asymmetry around alignment resets is
	// carried EXACTLY by the recursion (when A rejects a scalar B fuses,
	// dK = -K_b inflates D honestly). Assert d ~ 0 and D bounded at the
	// asymmetry-episode scale: sigma_att < 0.6 deg, sigma_vel < 7 cm/s,
	// sigma_pos < 16 cm.
	for (unsigned i = 0; i < State::size; i++) {
		EXPECT_NEAR(d(i), 0.f, 5e-3f) << "separation at index " << i;
	}

	for (unsigned k = 0; k < 3; k++) {
		EXPECT_LT(D(k, k), 1e-4f) << "D att " << k;
		EXPECT_LT(D(State::vel.idx + k, State::vel.idx + k), 5e-3f) << "D vel " << k;
		EXPECT_LT(D(State::pos.idx + k, State::pos.idx + k), 2.5e-2f) << "D pos " << k;
	}
}

TEST_F(EkfDualCompanionTest, evBiasIsDetectable)
{
	_ekf->enableCompanion();
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(10);

	_ekf_wrapper.enableExternalVisionPositionFusion();
	_sensor_simulator._vio.setPositionFrameToLocalNED();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(10);

	ASSERT_TRUE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());

	const auto &D = _ekf->companion().D();
	const auto &d = _ekf->companion().separation();

	// honest EV: separation consistent with its own covariance
	EXPECT_GT(D(IP, IP), 0.f); // EV exclusive updates inflate D
	const float q_nominal = d(IP) * d(IP) / (D(IP, IP) + 1e-9f);
	EXPECT_LT(q_nominal, 20.f);

	// inject a 2 m EV north bias: B gets dragged, A (EV-free) does not
	_sensor_simulator._vio.setPosition(Vector3f(2.f, 0.f, 0.f));
	_sensor_simulator.runSeconds(8);

	const float q_fault = d(IP) * d(IP) / (D(IP, IP) + 1e-9f);
	EXPECT_GT(fabsf(d(IP)), 0.2f);
	EXPECT_GT(q_fault, 9.21f); // chi-square 99%, 1 dof is 6.63; 2 dof 9.21 — use the stricter
}

TEST_F(EkfDualCompanionTest, evResetLeavesCompanionContinuous)
{
	_ekf->enableCompanion();
	_ekf_wrapper.disableGpsFusion();
	_ekf_wrapper.enableExternalVisionPositionFusion();
	_ekf_wrapper.enableExternalVisionHeightFusion();
	_sensor_simulator._vio.setPositionFrameToLocalNED();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(12);

	ASSERT_TRUE(_ekf->companion().initialized());

	const double lat_a_pre = _ekf->companion().gposA().latitude_rad();
	const float d_pre = _ekf->companion().separation()(IP);
	const float D_pre = _ekf->companion().D()(IP, IP);

	// VIO reset: 5 m position jump with a bumped reset counter
	extVisionSample sample{};
	sample.pos = Vector3f(5.f, 0.f, 0.f);
	sample.quat = Quatf();
	sample.position_var = Vector3f(0.01f, 0.01f, 0.01f);
	sample.velocity_var = Vector3f(0.01f, 0.01f, 0.01f);
	sample.orientation_var = Vector3f(0.01f, 0.01f, 0.01f);
	sample.pos_frame = PositionFrame::LOCAL_FRAME_NED;
	sample.vel_frame = VelocityFrame::BODY_FRAME_FRD;
	sample.reset_counter = 1;
	sample.quality = 100;
	_sensor_simulator._vio.setData(sample);
	_sensor_simulator.runSeconds(2);

	// the jump landed in the separation, not in A
	const float d_post = _ekf->companion().separation()(IP);
	EXPECT_GT(fabsf(d_post - d_pre), 3.f) << "EV reset jump not absorbed by separation";

	const double dlat_m = (_ekf->companion().gposA().latitude_rad() - lat_a_pre) * CONSTANTS_RADIUS_OF_EARTH;
	EXPECT_LT(fabs(dlat_m), 0.5) << "companion A moved on an EV reset";

	EXPECT_GT(_ekf->companion().D()(IP, IP), D_pre) << "separation covariance did not absorb the reset";
}

TEST_F(EkfDualCompanionTest, failoverAdoptsCompanionVariance)
{
	_ekf->enableCompanion();
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(20);

	ASSERT_TRUE(_ekf->companion().initialized());

	constexpr unsigned IY = State::quat_nominal.idx + 2;

	auto &c = _ekf->companion();
	const auto &P_b = _ekf->covariances(); // live reference into the filter

	// pin a guaranteed-nonzero C through the exact EV-class reset primitive:
	// an exclusive unit-gain reset lands C_ii = -R_m on its axis regardless
	// of the prior ledger (dormant seed and active recursion agree), and
	// delta = 0 makes it pure covariance surgery — no mean moves. R_m scales
	// with the live P so the construction holds in any environment (no
	// reliance on simulation-emergent residuals)
	float r_pos[3];
	float r_vel[3];
	float r_yaw;

	{
		DualCompanion::ExclusiveScope scope(c, true);
		DualCompanion::MeanUpdateContext ctx{};
		const float zero3[3] = {0.f, 0.f, 0.f};

		for (unsigned k = 0; k < 3; k++) {
			r_pos[k] = 0.5f * P_b(IP + k, IP + k);
			r_vel[k] = 0.5f * P_b(IV + k, IV + k);
		}

		r_yaw = 0.5f * P_b(IY, IY);

		c.onResetB(P_b, IP, 3, zero3, r_pos, ctx);
		c.onResetB(P_b, IV, 3, zero3, r_vel, ctx);
		c.onResetYawB(P_b, 0.f, r_yaw, ctx);
	}

	ASSERT_TRUE(c.cActive());

	for (unsigned k = 0; k < 3; k++) {
		ASSERT_NEAR(c.C()(IP + k, IP + k), -r_pos[k], 1e-6f + 1e-3f * r_pos[k]) << "C pos " << k;
		ASSERT_NEAR(c.C()(IV + k, IV + k), -r_vel[k], 1e-6f + 1e-3f * r_vel[k]) << "C vel " << k;
	}

	ASSERT_NEAR(c.C()(IY, IY), -r_yaw, 1e-6f + 1e-3f * r_yaw) << "C yaw";

	// expected adopted variances from one consistent pre-failover snapshot
	// (same floor as the reset plumbing, sq(0.01))
	const float yaw_var_a = math::max(c.varianceA(P_b, IY), sq(1e-2f));
	float vel_var_a[3];
	float pos_var_a[3];

	for (unsigned k = 0; k < 3; k++) {
		vel_var_a[k] = math::max(c.varianceA(P_b, IV + k), sq(1e-2f));
		pos_var_a[k] = math::max(c.varianceA(P_b, IP + k), sq(1e-2f));
	}

	ASSERT_TRUE(_ekf->resetToCompanion());

	// P_b now reads the post-failover covariance: every adopted axis carries
	// A's variance P_aa = P_bb + 2C + D. With C_ii = -R_m < 0 the old P + D
	// shortcut would overshoot each axis by 2 R_m = P_ii — far outside the
	// tolerance — and mid-sequence C/D mutation would shift it further
	EXPECT_NEAR(P_b(IY, IY), yaw_var_a, 1e-6f + 1e-4f * yaw_var_a) << "yaw";

	for (unsigned k = 0; k < 3; k++) {
		EXPECT_NEAR(P_b(IV + k, IV + k), vel_var_a[k], 1e-6f + 1e-4f * vel_var_a[k]) << "vel " << k;
		EXPECT_NEAR(P_b(IP + k, IP + k), pos_var_a[k], 1e-6f + 1e-4f * pos_var_a[k]) << "pos " << k;
	}
}
