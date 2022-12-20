/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/**
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include "RoverPositionControl.hpp"
#include <cmath>
#include <lib/ecl/geo/geo.h>
#include <systemlib/mavlink_log.h>

#define ACTUATOR_PUBLISH_PERIOD_MS 4

using namespace matrix;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);

RoverPositionControl::RoverPositionControl() :
		ModuleParams(nullptr),
		/* performance counters */
		_loop_perf(perf_alloc(PC_ELAPSED, "rover position control")) // TODO : do we even need these perf counters
{
	turn_inplace_thresh = M_PI * 0.3;
	turn_inplace_thresh_half = turn_inplace_thresh * 0.5;
	current_roll_body = 0.0;
	current_pitch_body = 0.0;
	current_yaw_speed = 0.0;

}

RoverPositionControl::~RoverPositionControl() {
	perf_free(_loop_perf);
}

void RoverPositionControl::parameters_update(bool force) {
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_gnd_control.set_l1_damping(_param_l1_damping.get());
		_gnd_control.set_l1_period(_param_l1_period.get());
		_gnd_control.set_l1_roll_limit(math::radians(0.0f));

		pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_speed_ctrl, _param_speed_p.get(),
				_param_speed_i.get(), _param_speed_d.get(),
				_param_speed_imax.get(), _param_gndspeed_max.get());

		pid_init(&_yawrate_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_yawrate_ctrl, _param_gnd_yaw_p.get(),
				_param_gnd_yaw_i.get(), _param_gnd_yaw_d.get(),
				_param_gnd_yaw_imax.get(), 0.5f);

		pid_init(&_brake_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_brake_ctrl, _param_brake_p.get(),
				_param_brake_i.get(), _param_brake_d.get(),
				_param_brake_imax.get(), 0.5f);

		//MODALAI TODO, fix for ROVER as it expects the throttle range to be -1 to 1 instead of 0-1 for UAVCAN
		param_get(param_find("SYS_AUTOSTART"), &_frame_type);

	}
}

void RoverPositionControl::vehicle_control_mode_poll() {
	bool updated;
	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub,
				&_control_mode);
	}
}

void RoverPositionControl::manual_control_setpoint_poll() {
	bool manual_updated;
	orb_check(_manual_control_setpoint_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_setpoint_sub,
				&_manual_control_setpoint);
	}
}

void RoverPositionControl::position_setpoint_triplet_poll() {
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub,
				&_pos_sp_triplet);
	}
}

void RoverPositionControl::attitude_setpoint_poll() {
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}
}

void RoverPositionControl::vehicle_attitude_poll() {
	bool att_updated;
	static float last_yaw = 0.0;

	orb_check(_vehicle_attitude_sub, &att_updated);

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub,
				&_vehicle_att);

		// safety
		matrix::Quatf q(_vehicle_att.q);
//		q.copyTo(_vehicle_att.q_d);
		matrix::Eulerf current_euler{q};
		current_roll_body = (double) current_euler.phi();
		current_pitch_body = (double) current_euler.theta();

		float yaw_delta = current_euler.psi() - last_yaw;

		float dt = 0.01; // Using non zero value to a avoid division by zero
		if (_yaw_last_called > 0) {
			dt = hrt_elapsed_time(&_yaw_last_called) * 1e-6f;
		}
		current_yaw_speed = yaw_delta / dt;

		_yaw_last_called = hrt_absolute_time();
		last_yaw = current_euler.psi();

		//PX4_ERR("Yaw speed: %f %f", (double) last_yaw, (double)current_yaw_speed);
	}
}


double RoverPositionControl::wrap_180(double angle) {
	double a = fmod(angle + M_PI, 2 * M_PI);
	return a >= 0 ? (a - M_PI) : (a + M_PI);
}


//static double map_range(double x, double in_min, double in_max, double out_min, double out_max) {
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}


bool RoverPositionControl::protect_rollover(float tracking_error)
{


	//PX4_ERR("Dist: %f pitch: %f", (double) tracking_error, (double)current_pitch_body);


	if (fabs(current_pitch_body) > 0.4 || fabs(current_roll_body) > 0.4)
	{
		_anti_rollover_request = true;
	}


	if (_anti_rollover_request)
	{
		if (fabs(current_pitch_body) < 0.14 && fabs(current_roll_body) < 0.14)
		{
			_anti_rollover_request = false;
		}
		else
		{
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = -1.0f * _param_throttle_max.get();
			_act_controls.control[actuator_controls_s::INDEX_ROLL] = 0.;
			_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0.;
			_act_controls.control[actuator_controls_s::INDEX_FLAPS] = 0.;
		}
	}

	return _anti_rollover_request;
}

bool RoverPositionControl::control_vio(
		const position_setpoint_triplet_s &pos_sp_triplet) {


	matrix::Vector3f current_position((float) _local_pos.x,
			(float) _local_pos.y, (float) _local_pos.z);
	matrix::Vector3f current_velocity((float) _local_pos.vx,
			(float) _local_pos.vy, (float) _local_pos.vz);


	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	bool setpoint = true;

	if ((_control_mode.flag_control_auto_enabled
			|| _control_mode.flag_control_offboard_enabled)
			&& pos_sp_triplet.current.valid) {

		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* get circle mode */
		//bool was_circle_mode = _gnd_control.circle_mode();
		/* current waypoint request (the one currently heading for) */
		// NOTE pos_sp_triplet y is lat, x is lon in NED space which is reverse from PX4 notation of NED space being XYZ
		matrix::Vector2f curr_wp_request((float) pos_sp_triplet.current.x,
				(float) pos_sp_triplet.current.y);

		if (curr_wp_request != _curr_wp) {
//			PX4_INFO("VIO NED --> WP request: [%f,%f],\tprev(x,y)[%f,%f],\tcur(x,y)[%f,%f]",
//					(double)pos_sp_triplet.current.x,
//					(double)pos_sp_triplet.current.y,
//					(double)_prev_wp(0),
//					(double)_prev_wp(1),
//					(double)_curr_wp(0),
//					(double)_curr_wp(1));
			_prev_wp = _curr_wp;
			_curr_wp = curr_wp_request;
		}

		matrix::Vector2f current_position_2d(current_position);
		matrix::Vector2f ground_speed_2d(current_velocity);

		float mission_throttle = _param_throttle_cruise.get();

		/* Just control the throttle */
		if (_param_speed_control_mode.get() == 1 && !_skid_steer_turn_request) {
			/* control the speed in closed loop */

			float mission_target_speed = _param_gndspeed_trim.get();

			// if mavros sends a velocity, use it. it must be > 0.1 m/s
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed)
					&& _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			}

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
			const Vector3f vel = R_to_body
					* Vector3f(current_velocity(0), current_velocity(1),
							current_velocity(2));

			const float x_vel = vel(0);
			const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

			bool brake = _param_throttle_max.get() == 0.0f;

			if (brake)
			{
				 mission_target_speed = 0;
//				 mission_throttle = -x_vel * _param_brake_p.get();

   			    mission_throttle = pid_calculate(&_brake_ctrl, mission_target_speed, x_vel,
									x_acc, dt);


				// Constrain throttle between min and max
				mission_throttle = math::constrain(mission_throttle,
						-1.0f, 1.0f);

				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] += mission_throttle;
				_act_controls.control[actuator_controls_s::INDEX_YAW] = 0;

				return setpoint;

			}
			else
			{

				pid_reset_integral(&_brake_ctrl);

				// Compute airspeed control out and just scale it as a constant
				mission_throttle = _param_throttle_speed_scaler.get()
						* pid_calculate(&_speed_ctrl, mission_target_speed, x_vel,
								x_acc, dt);

					// Constrain throttle between min and max
					mission_throttle = math::constrain(mission_throttle,
							_param_throttle_min.get(), _param_throttle_max.get());
			}

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle)
					&& _pos_sp_triplet.current.cruising_throttle > 0.01f) {
				mission_throttle = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		float dist_target = get_distance_to_next_waypoint_vio(
				(double) current_position(0),
				(double) current_position(1),
				(double) _curr_wp(0),
				(double) _curr_wp(1),
				0, 0);

		double t_bearing = (double)get_bearing_to_next_waypoint_vio(
				(double) current_position(0),
				(double) current_position(1),
				(double) _curr_wp(0),
				(double) _curr_wp(1));

		float vel_mag = ground_speed_2d.norm();

		float smooth_accel_multipler = 1.0;

//		PX4_ERR("\t\tRoll: %f Pitch: %f ", (double)current_roll_body/M_PI*180., (double)current_pitch_body/M_PI*180.);

		switch (_pos_ctrl_state) {

		case GOTO_WAYPOINT: {

			if (protect_rollover(tracking_dist))
				break;

			PX4_INFO("NED --> WP request: [%f,%f]\tcur(x,y)[%f,%f]  distance: %f",
					(double) current_position(0),
					(double) current_position(1),
					(double) _curr_wp(0),
					(double)_curr_wp(1),
					(double)dist_target);


			if (dist_target < _param_gnd_nav_rad.get()) {
				PX4_INFO("** dist_target too small or out of range %f %f",
						(double) dist_target, (double)fabs(t_bearing) );
				_pos_ctrl_state = STOPPING; // We are closer than loiter radius to waypoint, stop.

			} else {
				double siny_cosp = 2.0
						* (double) (_vehicle_att.q[0] * _vehicle_att.q[3]
								+ _vehicle_att.q[1] * _vehicle_att.q[2]);
				double cosy_cosp =
						1.0
								- 2.0
										* (double) (_vehicle_att.q[2] * _vehicle_att.q[2]
												+ _vehicle_att.q[3]
														* _vehicle_att.q[3]);
				double b_yaw = atan2(siny_cosp, cosy_cosp);

				double turn_request = wrap_180(t_bearing - b_yaw);
				double turn_request_delta = fabs(turn_request);
				int yaw_direction = sign(turn_request);

//				PX4_ERR("t_bearing: %f %f %f %f (%f %f) %f %f",
//						(double) current_position(0),
//						(double) current_position(1),
//						(double) _curr_wp(0),
//						(double) _curr_wp(1),
//						t_bearing*180./M_PI,
//						b_yaw*180./M_PI,
//						(double)turn_request_delta,
//						(double)dist_target);



				if (!_skid_steer_turn_request) {
	   			    if ((turn_request_delta >= 1.45) || (turn_request_delta > turn_inplace_thresh && (dist_target > 3.0f*_param_gnd_nav_rad.get()))) //
					{
						nav_yaw_direction = yaw_direction;

						_skid_steer_turn_complete = false;
						_skid_steer_turn_request = true;
						pid_reset_integral(&_speed_ctrl);
						pid_reset_integral(&_yawrate_ctrl);
						_last_derivative = 0;
						_integrator = 0;
						_last_yaw_cmd = 0;
						pos_mode_last_yaw = 0;

					}
				}
				//0.174533
				else if (_skid_steer_turn_request && ((turn_request_delta <= (double)_param_gnd_turn_min.get()) || (nav_yaw_direction != yaw_direction))) // 10deg
				{
					_skid_steer_turn_complete = true;
					_skid_steer_turn_request = false;
				}

				if (_skid_steer_turn_request) {

					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
							0.0f;

					float input_yaw_cmd = _param_gnd_turn_spd.get() * sign(turn_request);

					if (fabs(input_yaw_cmd) < 0.05f)
						input_yaw_cmd = 0.0f;

					int yaw_cmd_direction = sign(input_yaw_cmd);

					float input_yaw_rate = current_yaw_speed; // _vio_pos.yawspeed;
					if (fabs(input_yaw_rate) < 0.0174533f)
						input_yaw_rate = 0.0f;

					float current_yaw_mag = fabs(input_yaw_cmd);

					if (current_yaw_mag> 0.05f)
					{
						if (fabs(input_yaw_rate) < _param_gndspeed_trim_yaw.get())
						{
							pos_mode_last_yaw += (yaw_cmd_direction*_param_gnd_yawrate_p.get());
						}
						else
						{
							pos_mode_last_yaw -= (0.3f*yaw_cmd_direction*_param_gnd_yawrate_p.get());
						}

					}
					else
						pos_mode_last_yaw = 0.0f;

					_act_controls.control[actuator_controls_s::INDEX_YAW] = pos_mode_last_yaw;

//
//					double rate_proportion = (double)_param_gnd_turn_spd.get();
//					if (turn_request_delta <= turn_inplace_thresh_half)
//						rate_proportion *= (turn_request_delta / turn_inplace_thresh);
//
//					if (rate_proportion < (double)_param_gnd_yawv_min.get())
//						rate_proportion = (double) _param_gnd_yawv_min.get();
//
//					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
//							0.0f;
//					_act_controls.control[actuator_controls_s::INDEX_YAW] =
//							(float) rate_proportion * sign(turn_request);

//					float input_yaw_rate = _vio_pos.yawspeed;
//					if (fabs(input_yaw_rate) < 0.0174533f)
//						input_yaw_rate = 0.0f;
//
//					if (fabs(input_yaw_rate) < _param_gndspeed_trim_yaw.get() )
//							_last_yaw_cmd += yaw_direction*_param_gnd_yawrate_p.get();
//
//					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
//							0.0f;
//					_act_controls.control[actuator_controls_s::INDEX_YAW] =
//							_last_yaw_cmd;

					_act_controls.control[actuator_controls_s::INDEX_ROLL] = b_yaw*180.0/M_PI;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = turn_request*180.0/M_PI;
					_act_controls.control[actuator_controls_s::INDEX_FLAPS] = t_bearing*180.0/M_PI;
				}
				else if (_skid_steer_turn_complete)
				{
					if (vel_mag < _param_imu_delay.get())
					{
						_skid_steer_turn_complete = false;
						_skid_steer_turn_request = false;
					}

					_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
					_act_controls.control[actuator_controls_s::INDEX_ROLL] = 0.;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0.;
					_act_controls.control[actuator_controls_s::INDEX_FLAPS] = 0.;

				}
				else
				{
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
							mission_throttle * smooth_accel_multipler;

					// PID for yaw
					double yaw_authority = (double)_param_gnd_yaw_p.get() * turn_request;
					if (dt > 0)
					{
						double derivative = (turn_request - _last_turn_request) / (double)dt;
						double RC = 1/(2*M_PI*_fCut);
						derivative = _last_derivative +
						                     (((double)dt / (RC + (double)dt)) *
						                      (derivative - _last_derivative));
						_last_turn_request   = turn_request;
				        _last_derivative    = derivative;
				        yaw_authority +=  (double)_param_gnd_yaw_d.get() * derivative;


				        _integrator += ((float)turn_request * _param_gnd_yaw_i.get()) * dt;
				        if (_integrator < -_param_gnd_yaw_imax.get()) {
				            _integrator = -_param_gnd_yaw_imax.get();
				        } else if (_integrator > _param_gnd_yaw_imax.get()) {
				            _integrator = _param_gnd_yaw_imax.get();
				        }
				        yaw_authority += (double)_integrator;

					}

					float control_effort = math::constrain((float)yaw_authority, -0.8f, 0.8f);
					_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;


					_act_controls.control[actuator_controls_s::INDEX_ROLL] = vel_mag;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = turn_request;
					_act_controls.control[actuator_controls_s::INDEX_FLAPS] = _last_derivative;
					_act_controls.control[actuator_controls_s::INDEX_SPOILERS] = b_yaw;
					_act_controls.control[actuator_controls_s::INDEX_AIRBRAKES] = t_bearing;

/////////////////////////////////////////////////////////////
#ifdef USE_L1
					_gnd_control.navigate_waypoints_local(_prev_wp, _curr_wp,
							current_position_2d, ground_speed_2d, (float)b_yaw);
					float desired_r = vel_mag / fabs(_gnd_control.nav_lateral_acceleration_demand());
					float desired_theta = (0.5f * M_PI_F) - (float) atan2(desired_r, _param_wheel_base.get());
					float control_effort = (desired_theta / _param_max_turn_angle.get()) * sign(_gnd_control.nav_lateral_acceleration_demand());
//
//					PX4_INFO(" Distance to target %f, Turn request [%f %f] %f(%f) accel: %f desired r: %f (%f / %f) control_effort: %f skid [%s]",
//							(double)dist_target,
//							(double)t_bearing*180/M_PI,
//							(double)b_yaw*180/M_PI,
//							(double)turn_request*180/M_PI,
//							(double)turn_request_delta*180/M_PI,
//							(double)_gnd_control.nav_lateral_acceleration_demand(),
//							(double)desired_r,
//							(double)(atan2(desired_r, _param_wheel_base.get())*180.0f/M_PI_F),
//							(double)desired_theta*180/M_PI,
//							(double)control_effort,
//							_skid_steer_turn_request ? "true" : "false");
//
//					PX4_INFO("Lateral accel: %f nav_bearing: %f vel: %f",
//							(double)_gnd_control.nav_lateral_acceleration_demand(),
//							(double)_gnd_control.nav_bearing(),
//							(double) vel_mag);

					// DEBUG HACK
//					if (_frame_type == 50005)

					if (vel_mag < 0.0254f || desired_theta > 0.785398f)
						control_effort = 0;
					else
						control_effort *= .6f;  //holonomic only

					control_effort = math::constrain(control_effort, -.9f,
								.9f);

					_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

					_act_controls.control[actuator_controls_s::INDEX_ROLL] = desired_r;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = (double)desired_theta*180.0/M_PI*sign(_gnd_control.nav_lateral_acceleration_demand());
					_act_controls.control[actuator_controls_s::INDEX_FLAPS] = _gnd_control.nav_lateral_acceleration_demand();
					_act_controls.control[actuator_controls_s::INDEX_SPOILERS] = b_yaw;
					_act_controls.control[actuator_controls_s::INDEX_AIRBRAKES] = _gnd_control.nav_bearing();
					_act_controls.control[actuator_controls_s::INDEX_LANDING_GEAR] = vel_mag;
#endif

				}
			}
		}
		break;

		case STOPPING: {
			nav_yaw_direction = 0;

			_act_controls.control[actuator_controls_s::INDEX_ROLL] = 0;
			_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0;
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.;
			_act_controls.control[actuator_controls_s::INDEX_FLAPS] = 0;
			_act_controls.control[actuator_controls_s::INDEX_SPOILERS] = 0;
			_act_controls.control[actuator_controls_s::INDEX_AIRBRAKES] = 0;
			_act_controls.control[actuator_controls_s::INDEX_LANDING_GEAR] = 0;


			float dist_between_waypoints = get_distance_to_next_waypoint_vio(
					(double) _prev_wp(0), (double) _prev_wp(1),
					(double) _curr_wp(0), (double) _curr_wp(1), 0, 0);


			if (dist_between_waypoints > 0) {
				_pos_ctrl_state = GOTO_WAYPOINT; // A new waypoint has arrived go to it
				_act_controls.control[actuator_controls_s::INDEX_ROLL] = 88;
				_act_controls.control[actuator_controls_s::INDEX_ROLL] = 88;
			}
		}
		break;

		default:
			PX4_ERR("Unknown Rover State");
			_pos_ctrl_state = STOPPING;
			break;
		}

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		setpoint = false;
	}

	return setpoint;
}

bool RoverPositionControl::control_position(
		const matrix::Vector2f &current_position,
		const matrix::Vector3f &ground_speed,
		const position_setpoint_triplet_s &pos_sp_triplet) {

	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	bool setpoint = true;

	if ((_control_mode.flag_control_auto_enabled
			|| _control_mode.flag_control_offboard_enabled)
			&& pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* get circle mode */
		//bool was_circle_mode = _gnd_control.circle_mode();
		/* current waypoint (the one currently heading for) */
		matrix::Vector2f curr_wp((float) pos_sp_triplet.current.lat,
				(float) pos_sp_triplet.current.lon);

		/* previous waypoint */
		matrix::Vector2f prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float) pos_sp_triplet.previous.lat;
			prev_wp(1) = (float) pos_sp_triplet.previous.lon;
		}

		matrix::Vector2f ground_speed_2d(ground_speed);

		float mission_throttle = _param_throttle_cruise.get();

		/* Just control the throttle */
		if (_param_speed_control_mode.get() == 1 && !!_skid_steer_turn_request) {
			/* control the speed in closed loop */

			float mission_target_speed = _param_gndspeed_trim.get();

			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed)
					&& _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			}

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
			const Vector3f vel = R_to_body
					* Vector3f(ground_speed(0), ground_speed(1),
							ground_speed(2));

			const float x_vel = vel(0);
			const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

			// Compute airspeed control out and just scale it as a constant
			mission_throttle = _param_throttle_speed_scaler.get()
					* pid_calculate(&_speed_ctrl, mission_target_speed, x_vel,
							x_acc, dt);

			// Constrain throttle between min and max
			mission_throttle = math::constrain(mission_throttle,
					_param_throttle_min.get(), _param_throttle_max.get());

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle)
					&& _pos_sp_triplet.current.cruising_throttle > 0.01f) {

				mission_throttle = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		float dist_target = get_distance_to_next_waypoint(_global_pos.lat,
				_global_pos.lon, (double) curr_wp(0), (double) curr_wp(1));

		double t_bearing = (double)get_bearing_to_next_waypoint(
				(double) current_position(0),
				(double) current_position(1),
				(double) curr_wp(0),
				(double) curr_wp(1));

		float vel_mag = ground_speed_2d.norm_squared();

		float smooth_accel_multipler = 1.0;

		switch (_pos_ctrl_state) {
		case GOTO_WAYPOINT: {

			if (protect_rollover(tracking_dist))
				break;

			if (dist_target < _param_gnd_nav_rad.get()) {
//				PX4_INFO("** dist_target too small or out of range %f %f",
//						(double) dist_target, (double)fabs(t_bearing) );
				_pos_ctrl_state = STOPPING; // We are closer than loiter radius to waypoint, stop.

			} else {
				double siny_cosp = 2.0
						* (double) (_vehicle_att.q[0] * _vehicle_att.q[3]
								+ _vehicle_att.q[1] * _vehicle_att.q[2]);
				double cosy_cosp =
						1.0
								- 2.0
										* (double) (_vehicle_att.q[2] * _vehicle_att.q[2]
												+ _vehicle_att.q[3]
														* _vehicle_att.q[3]);
				double b_yaw = atan2(siny_cosp, cosy_cosp);

				double turn_request = wrap_180(t_bearing - b_yaw);
				double turn_request_delta = fabs(turn_request);

			if (!_skid_steer_turn_request) {
					if (turn_request_delta > turn_inplace_thresh && (dist_target > 3.0f*_param_gnd_nav_rad.get())) //
					{
						_skid_steer_turn_complete = false;
						_skid_steer_turn_request = true;
						pid_reset_integral(&_speed_ctrl);
						pid_reset_integral(&_yawrate_ctrl);

						_last_derivative = 0;
						_integrator = 0;

					}
				}
				else if (_skid_steer_turn_request && ((turn_request_delta <= (double)_param_gnd_turn_min.get()))) // 10deg
				{
					_skid_steer_turn_complete = true;
					_skid_steer_turn_request = false;
					pid_reset_integral(&_speed_ctrl);
				}

		     	if (_skid_steer_turn_request) {

					float input_yaw_cmd = _param_gnd_turn_spd.get() * sign(turn_request);

					if (fabs(input_yaw_cmd) < 0.05f)
						input_yaw_cmd = 0.0f;

					int yaw_cmd_direction = sign(input_yaw_cmd);

					float input_yaw_rate = current_yaw_speed; // _vio_pos.yawspeed;
					if (fabs(input_yaw_rate) < 0.0174533f)
						input_yaw_rate = 0.0f;

					float current_yaw_mag = fabs(input_yaw_cmd);

					if (current_yaw_mag> 0.05f)
					{
						if (fabs(input_yaw_rate) < _param_gndspeed_trim_yaw.get())
						{
							pos_mode_last_yaw += (yaw_cmd_direction*_param_gnd_yawrate_p.get());
						}
						else
						{
							pos_mode_last_yaw -= (0.3f*yaw_cmd_direction*_param_gnd_yawrate_p.get());
						}

					}
					else
						pos_mode_last_yaw = 0.0f;

				}
				else if (_skid_steer_turn_complete)
				{
					if (vel_mag < _param_imu_delay.get())
					{
						_skid_steer_turn_complete = false;
						_skid_steer_turn_request = false;
					}
					_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
				}
				else
				{

					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
							mission_throttle * smooth_accel_multipler;

					// PID for yaw
					double yaw_authority = (double)_param_gnd_yaw_p.get() * turn_request;
					if (dt > 0)
					{
						double derivative = (turn_request - _last_turn_request) / (double)dt;
						double RC = 1/(2*M_PI*_fCut);
						derivative = _last_derivative +
						                     (((double)dt / (RC + (double)dt)) *
						                      (derivative - _last_derivative));
						_last_turn_request   = turn_request;
				        _last_derivative    = derivative;
				        yaw_authority +=  (double)_param_gnd_yaw_d.get() * derivative;

				        _integrator += ((float)turn_request * _param_gnd_yaw_i.get()) * dt;
				        if (_integrator < -_param_gnd_yaw_imax.get()) {
				            _integrator = -_param_gnd_yaw_imax.get();
				        } else if (_integrator > _param_gnd_yaw_imax.get()) {
				            _integrator = _param_gnd_yaw_imax.get();
				        }
				        yaw_authority += (double)_integrator;
					}

					float control_effort = math::constrain((float)yaw_authority, -0.98f, 0.98f);
					_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

					_act_controls.control[actuator_controls_s::INDEX_ROLL] = vel_mag;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = turn_request;
					_act_controls.control[actuator_controls_s::INDEX_FLAPS] = _last_derivative;
					_act_controls.control[actuator_controls_s::INDEX_SPOILERS] = b_yaw;
					_act_controls.control[actuator_controls_s::INDEX_AIRBRAKES] = t_bearing;


#ifdef USE_L1
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
							mission_throttle * smooth_accel_multipler;
					_gnd_control.navigate_waypoints(prev_wp, curr_wp,
							current_position, ground_speed_2d);
					float desired_r = vel_mag / fabs(_gnd_control.nav_lateral_acceleration_demand());
					float desired_theta = (0.5f * M_PI_F) - (float) atan2(desired_r, _param_wheel_base.get());
					float control_effort = (desired_theta / _param_max_turn_angle.get()) * sign(_gnd_control.nav_lateral_acceleration_demand());
//
//					PX4_INFO("[%f,%f,%f,%f]  Distance to target %f, Turn request [%f %f] %f(%f) accel: %f desired r: %f (%f / %f) control_effort: %f skid [%s]",
//							(double) current_position(0),
//							(double) current_position(1),
//							(double) curr_wp(0),
//							(double) curr_wp(1),
//							(double)dist_target,
//							(double)t_bearing*180/M_PI,
//							(double)b_yaw*180/M_PI,
//							(double)turn_request*180/M_PI,
//							(double)turn_request_delta*180/M_PI,
//							(double)_gnd_control.nav_lateral_acceleration_demand(),
//							(double)desired_r,
//							(double)(atan2(desired_r, _param_wheel_base.get())*180.0f/M_PI_F),
//							(double)desired_theta*180/M_PI,
//							(double)control_effort,
//							_skid_steer_turn_request ? "true" : "false");

					// DEBUG HACK
					_act_controls.control[actuator_controls_s::INDEX_ROLL] = vel_mag;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = t_bearing;
					_act_controls.control[actuator_controls_s::INDEX_FLAPS] = _gnd_control.nav_lateral_acceleration_demand();
					_act_controls.control[actuator_controls_s::INDEX_SPOILERS] = b_yaw;
					_act_controls.control[actuator_controls_s::INDEX_AIRBRAKES] = _gnd_control.nav_bearing();
					_act_controls.control[actuator_controls_s::INDEX_LANDING_GEAR] = desired_r;

//					if (_frame_type == 50005)

					if (vel_mag < 0.1f || desired_theta > 0.785398f)
						control_effort = 0;
					else
						control_effort *= .5f;

					control_effort = math::constrain(control_effort, -.9f,
								.9f);

					_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
#endif

				}
			}
		}
		break;

		case STOPPING: {

			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
			// Note _prev_wp is different to the local prev_wp which is related to a mission waypoint.
			float dist_between_waypoints = get_distance_to_next_waypoint(
					(double) _prev_wp(0), (double) _prev_wp(1),
					(double) curr_wp(0), (double) curr_wp(1));

			if (dist_between_waypoints > 0) {
				_pos_ctrl_state = GOTO_WAYPOINT; // A new waypoint has arrived go to it
			}
			_act_controls.control[actuator_controls_s::INDEX_ROLL] = vel_mag;
			_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0;
			_act_controls.control[actuator_controls_s::INDEX_FLAPS] = _gnd_control.nav_lateral_acceleration_demand();
			_act_controls.control[actuator_controls_s::INDEX_SPOILERS] = 9999;
			_act_controls.control[actuator_controls_s::INDEX_AIRBRAKES] = 9999;
			_act_controls.control[actuator_controls_s::INDEX_LANDING_GEAR] = 0;


		}
			break;

		default:
			PX4_ERR("Unknown Rover State");
			_pos_ctrl_state = STOPPING;
			break;
		}

		_prev_wp = curr_wp;

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		setpoint = false;
		_prev_wp(0) = 0.0f;
		_prev_wp(1) = 0.0f;
	}

	return setpoint;
}

void RoverPositionControl::control_velocity(
		const matrix::Vector3f &current_velocity,
		const position_setpoint_triplet_s &pos_sp_triplet) {
	PX4_INFO("RoverPositionControl::control_velocity");

	float dt = 0.01; // Using non zero value to a avoid division by zero

	const float mission_throttle = _param_throttle_cruise.get();
	const matrix::Vector3f desired_velocity { pos_sp_triplet.current.vx,
			pos_sp_triplet.current.vy, pos_sp_triplet.current.vz };
	// initial throttle setting
	const float desired_speed = desired_velocity.norm();

	if (desired_speed > 0.01f) {

		const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
		const Vector3f vel = R_to_body
				* Vector3f(current_velocity(0), current_velocity(1),
						current_velocity(2));

		const float x_vel = vel(0);
		const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

		const float control_throttle = pid_calculate(&_speed_ctrl,
				desired_speed, x_vel, x_acc, dt);

		//Constrain maximum throttle to mission throttle
		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
				math::constrain(control_throttle, 0.0f, mission_throttle);

		Vector3f desired_body_velocity;

		if (pos_sp_triplet.current.velocity_frame
				== position_setpoint_s::VELOCITY_FRAME_BODY_NED) {
			desired_body_velocity = desired_velocity;

		} else {
			// If the frame of the velocity setpoint is unknown, assume it is in local frame
			desired_body_velocity = R_to_body * desired_velocity;

		}

		const float desired_theta = atan2f(desired_body_velocity(1),
				desired_body_velocity(0));
		float control_effort = desired_theta / _param_max_turn_angle.get();
		control_effort = math::constrain(control_effort, -1.0f, 1.0f);

		//PX4_INFO("   VEL CONTROL: Yaw control effort: %f", (double) control_effort);
		_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	} else {

		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
		_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;

	}
}

void RoverPositionControl::control_attitude(const vehicle_attitude_s &att,
		const vehicle_attitude_setpoint_s &att_sp) {
#ifdef NOT_USED
        PX4_INFO("RoverPositionControl::control_attitude");

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = Quatf(att.q).inversed() * Quatf(att_sp.q_d);
	const Eulerf euler_sp = qe;

	float control_effort = euler_sp(2) / _param_max_turn_angle.get();
	control_effort = math::constrain(control_effort, -1.0f, 1.0f);

	//PX4_INFO("   ATTI CONTROL: Yaw control effort: %f", (double) control_effort);
	_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	const float control_throttle = att_sp.thrust_body[0];

	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =  math::constrain(control_throttle, 0.0f, 1.0f);
#endif
}

void RoverPositionControl::run() {

	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vio_pos_sub = orb_subscribe(ORB_ID(vehicle_odometry));
	_manual_control_setpoint_sub = orb_subscribe(
			ORB_ID(manual_control_setpoint));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);
	orb_set_interval(_local_pos_sub, 20);
	orb_set_interval(_vio_pos_sub, 20);

	parameters_update(true);

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[6];

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _manual_control_setpoint_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _sensor_combined_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _vehicle_attitude_sub; // Poll attitude
	fds[3].events = POLLIN;
	fds[4].fd = _local_pos_sub;  // Added local position as source of position
	fds[4].events = POLLIN;
	fds[5].fd = _vio_pos_sub;  // Added local position as source of position
	fds[5].events = POLLIN;

	PX4_INFO("RoverPositionControl::run()");

	_param_gnd_ver.set(1.18);
	_param_gnd_ver.commit_no_notification();


	double pos_mode_target_bearing = 0.0;
	double demo_mode_target_bearing = 3.141;
	float  pos_mode_last_vel = 0.0;
	hrt_abstime control_pos_manual_last_called{0}; 	/**<last call of control_position  */
	int actual_turn_direction = 1;
	int refine_yaw = 0;
	int debounce_yaw = 0;

	while (!should_exit()) {

		//PX4_ERR("Rover control");

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}
		// HACK
		// HACK
		// HACK
		if (_control_mode.flag_control_offboard_enabled) {
			_control_mode.flag_control_position_enabled = 1;
			_control_mode.flag_control_velocity_enabled = 0;
			_control_mode.flag_control_attitude_enabled = 0;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		attitude_setpoint_poll();
		//manual_control_setpoint_poll();

		_vehicle_acceleration_sub.update();

		/* update parameters from storage */
		parameters_update();

		bool manual_mode = _control_mode.flag_control_manual_enabled;
		bool pos_mode = manual_mode && _control_mode.flag_control_position_enabled;
		bool stab_mode = manual_mode && _control_mode.flag_control_altitude_enabled && !_control_mode.flag_control_position_enabled;

		//PX4_INFO("mode: %d %d", pos_mode, stab_mode);

		/* only run controller if position changed */
		if (fds[0].revents & POLLIN || fds[4].revents & POLLIN
				|| fds[5].revents & POLLIN) {
			perf_begin(_loop_perf);

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub,
					&_global_pos);
			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub,
					&_local_pos);
			orb_copy(ORB_ID(vehicle_odometry), _vio_pos_sub, &_vio_pos);


//			tracking_dist = _local_pos.x - _last_local_position_2d(0);;
//			_last_local_position_2d(0) = _local_pos.x;
//			_last_local_position_2d(1) = _local_pos.y;

			//PX4_ERR("%f %f %f", (double)_local_pos.x, (double)_local_pos.y, (double) _local_pos.z);

			//
			//
			// Get position from mavros client offboard
			// NEW CODE HAS THIS
			position_setpoint_triplet_poll();

			// update the reset counters in any case
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			matrix::Vector3f current_velocity(_local_pos.vx, _local_pos.vy,
					_local_pos.vz);

			if (!manual_mode && _control_mode.flag_control_position_enabled) {
				// if the requested set points come in as NED/FRD coordinates
				if ((float) _pos_sp_triplet.current.lat <= 1e-5f
						&& (float) _pos_sp_triplet.current.lon <= 1e-5f
						&& (float) _global_pos.lat <= 1e-5f
						&& (float) _global_pos.lon <= 1e-5f) {

					control_vio(_pos_sp_triplet);

				}
				else // they must be coming in as geo NED
				{
					matrix::Vector3f ground_speed(_local_pos.vx, _local_pos.vy,
							_local_pos.vz);
					matrix::Vector2f current_position((float) _global_pos.lat,
							(float) _global_pos.lon);
					control_position(current_position, ground_speed,
							_pos_sp_triplet);
				}
			} else if (!manual_mode
					&& _control_mode.flag_control_velocity_enabled) {
				control_velocity(current_velocity, _pos_sp_triplet);
			} else {
				_control_position_last_called = 0;
				_skid_steer_turn_request = false;
				_prev_wp(0) = 0.0f;
				_prev_wp(1) = 0.0f;
				_last_turn_request   = 0;
				_last_derivative    = 0;
				_integrator = 0;
			}

			//TODO: check if radius makes sense here
			float turn_distance = _param_l1_distance.get(); //_gnd_control.switch_distance(100.0f);

			// publish status
			position_controller_status_s pos_ctrl_status = { };

			pos_ctrl_status.nav_roll = 0.0f;
			pos_ctrl_status.nav_pitch = 0.0f;
			pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

			pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
			pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

			pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(
					_global_pos.lat, _global_pos.lon,
					_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

			pos_ctrl_status.acceptance_radius = turn_distance;
			pos_ctrl_status.yaw_acceptance = NAN;

			pos_ctrl_status.timestamp = hrt_absolute_time();

			_pos_ctrl_status_pub.publish(pos_ctrl_status);

			perf_end(_loop_perf);
		}

		if (fds[3].revents & POLLIN) {

			vehicle_attitude_poll();

			if (!manual_mode && _control_mode.flag_control_attitude_enabled
					&& !_control_mode.flag_control_position_enabled
					&& !_control_mode.flag_control_velocity_enabled) {

				control_attitude(_vehicle_att, _att_sp);

			}

		}

		if (fds[1].revents & POLLIN) {

			// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
			// returning immediately and this loop will eat up resources.
			orb_copy(ORB_ID(manual_control_setpoint),
					_manual_control_setpoint_sub, &_manual_control_setpoint);

			if (manual_mode) {
				 if (!pos_mode && !stab_mode)
				 {
					 control_pos_manual_last_called = 0;
						_last_turn_request   = 0;
						_last_derivative    = 0;
						_integrator = 0;
					 /* manual/direct control */
					//PX4_INFO("Manual mode!");
					_act_controls.control[actuator_controls_s::INDEX_ROLL] =
							_manual_control_setpoint.y;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] =
							-_manual_control_setpoint.x;
					_act_controls.control[actuator_controls_s::INDEX_YAW] =
							_manual_control_setpoint.r; //TODO: Readd yaw scale param
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
							_manual_control_setpoint.z;

					// reset target bearing
					demo_mode_target_bearing = 3.141;
					pos_mode_target_bearing = 0;
					pos_mode_last_yaw = 0.0f;
					actual_turn_direction = 0;

					_param_set_angle.set(0);
					_param_set_throttle.set(0);
					param_t th = param_handle(px4::params::GND_SET_THR);
					param_set(th, &_param_set_throttle);
					param_t an = param_handle(px4::params::GND_SET_ANG);
					param_set(an, &_param_set_angle);

				 }
				 else
				{
					//OVERRIDE
					double target_angle = (double) _param_set_angle.get();
					double target_throttle = (double) _param_set_throttle.get();
////					PX4_INFO("RoverPositionControl::target_throttle %f", target_throttle);
//					if (pos_mode)
//					{
//						if (fabs(target_angle) > 0.05 && fabs(target_angle) < 1.0)
//						{
//							_manual_control_setpoint.y = target_angle;
//						}
//
//						if (target_throttle > 0.05)
//						{
//							_manual_control_setpoint.z = target_throttle;
//						}
//						else
//							_manual_control_setpoint.z = 0;
//					}

					if (pos_mode)
					{
						 if  (target_throttle > 0.05)
						{
							 _manual_control_setpoint.z = target_throttle;
							pos_mode_last_yaw = 0.0;
							refine_yaw = 0;
							debounce_yaw = 0;
						}
						 else
								_manual_control_setpoint.z = 0;
					}

//					PX4_INFO("RoverPositionControl::Z %f", (double)_manual_control_setpoint.z);


					float dt = 0.01; // Using non zero value to a avoid division by zero

					// hold current heading based on localization data
					double siny_cosp = 2.0
							* (double) (_vehicle_att.q[0] * _vehicle_att.q[3]
									+ _vehicle_att.q[1] * _vehicle_att.q[2]);
					double cosy_cosp =
							1.0
									- 2.0
											* (double) (_vehicle_att.q[2] * _vehicle_att.q[2]
													+ _vehicle_att.q[3]
															* _vehicle_att.q[3]);
					double b_yaw = atan2(siny_cosp, cosy_cosp);

					// account for deadband
					if (!pos_mode && fabs(_manual_control_setpoint.z) <= 0.05f)
					{
						pos_mode_target_bearing = b_yaw;
					}

					double turn_request = wrap_180(pos_mode_target_bearing - b_yaw);

//					PX4_ERR("RoverPositionControl::YAW: %f  [%f]",
//							pos_mode_target_bearing, turn_request);

					//				double turn_request_delta = fabs(turn_request);
	//				int yaw_direction = sign(turn_request);

//					PX4_ERR("RoverPositionControl::YAW: %f %f %f %f",
//							(double) turn_request/M_PI*180.0,
//							(double) pos_mode_target_bearing/M_PI*180.0,
//							(double) b_yaw/M_PI*180.0,
//							(double) _manual_control_setpoint.z);


					if (control_pos_manual_last_called > 0) {
						dt = hrt_elapsed_time(&control_pos_manual_last_called) * 1e-6f;
					}
					control_pos_manual_last_called = hrt_absolute_time();

					// PID for yaw
					double yaw_authority = (double)_param_gnd_yaw_p.get() * turn_request;
					double derivative = (turn_request - _last_turn_request) / (double)dt;
					double RC = 1/(2*M_PI*_fCut);
					derivative = _last_derivative +
										 (((double)dt / (RC + (double)dt)) *
										  (derivative - _last_derivative));
					_last_turn_request   = turn_request;
					_last_derivative    = derivative;
					yaw_authority +=  (double)_param_gnd_yaw_d.get() * derivative;


					_integrator += ((float)turn_request * _param_gnd_yaw_i.get()) * dt;
					if (_integrator < -_param_gnd_yaw_imax.get()) {
						_integrator = -_param_gnd_yaw_imax.get();
					} else if (_integrator > _param_gnd_yaw_imax.get()) {
						_integrator = _param_gnd_yaw_imax.get();
					}
					yaw_authority += (double)_integrator;

					float control_effort = math::constrain((float)yaw_authority, -0.6f, 0.6f);

					const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
					const Vector3f vel = R_to_body
							* Vector3f(_local_pos.vx, _local_pos.vy,
									_local_pos.vz);

					const float x_vel = vel(0);
					const float x_acc = _vehicle_acceleration_sub.get().xyz[0];


					float mission_target_speed = _param_gndspeed_trim.get();

					if (fabs(_manual_control_setpoint.z) <= 0.05f)
					{
//						PX4_INFO("RoverPositionControl::brake on");
						mission_target_speed = 0;
					}

					float  cmd_throttle = _param_throttle_speed_scaler.get()
							* pid_calculate(&_speed_ctrl, mission_target_speed, x_vel,
									x_acc, dt);

					//PX4_INFO("RoverPositionControl::POS() %f %f", (double) x_vel, (double)cmd_throttle);
					//PX4_INFO("RoverPositionControl::YAW: %f", (double) _vio_pos.yawspeed);

					// Constrain throttle between min and max
//					cmd_throttle = math::constrain(cmd_throttle,
//							_param_throttle_min.get(), _param_throttle_max.get());

					cmd_throttle = math::constrain(cmd_throttle,
							-_param_throttle_max.get()*0.5f, _param_throttle_max.get());


					if (_manual_control_setpoint.z > 0.05f)
					{
						actual_turn_direction = 0;

						if (_manual_control_setpoint.z < cmd_throttle)
							pos_mode_last_vel = _manual_control_setpoint.z ;
						else
							pos_mode_last_vel = cmd_throttle ;
					}
					else
					{
//						pos_mode_last_vel = 0.0f ;
						pos_mode_last_vel = cmd_throttle ;
					}

					float input_yaw_cmd = _manual_control_setpoint.y;
					if (fabs(input_yaw_cmd) < 0.05f)
						input_yaw_cmd = 0.0f;

					int yaw_cmd_direction = sign(input_yaw_cmd);

					float input_yaw_rate = current_yaw_speed; //_vio_pos.yawspeed;
//					if (fabs(input_yaw_rate) < 0.001)
//						input_yaw_rate = 0.0f;

					float current_yaw_mag = fabs(input_yaw_cmd);

					if (stab_mode)
					{
						if (current_yaw_mag> 0.05f)
						{
//								if (fabs(input_yaw_rate) < _param_gndspeed_trim_yaw.get() )
//									pos_mode_last_yaw += (yaw_cmd_direction*_param_gnd_yawrate_p.get());

								if (fabs(input_yaw_rate) < _param_gndspeed_trim_yaw.get())
								{
									pos_mode_last_yaw += (yaw_cmd_direction*_param_gnd_yawrate_p.get());
								}
								else
								{
									pos_mode_last_yaw -= (0.3f*yaw_cmd_direction*_param_gnd_yawrate_p.get());
								}

						}
						else
						{

							if (actual_turn_direction > 0)
							{
								_param_set_angle.set(0);
								_param_set_angle.commit_no_notification();
							}
							pos_mode_last_yaw = 0.0f;
							actual_turn_direction = 0;
						}

//						if (protect_rollover(tracking_dist))
//							continue;
					}
					else
					{
						if (fabs(target_angle) >= 1.0 ) // || _param_set_angle_status.get() == 1)
						{
							_act_controls.control[actuator_controls_s::INDEX_PITCH] = 1.0;

							turn_request = wrap_180(demo_mode_target_bearing - b_yaw);

							int turn_request_direction  = sign(turn_request);
							if (actual_turn_direction == 0)
								actual_turn_direction = turn_request_direction;

							bool user_turn_request = (fabs (turn_request) > (double)_param_gnd_turn_min.get()) && (turn_request_direction == actual_turn_direction);
							if (user_turn_request)
							{
								float kp_yawv = _param_gnd_yawrate_p.get();
								float target_yaw_speed = _param_gndspeed_trim_yaw.get();
								if (fabs(turn_request) < 0.5) // 25deg
								{
									target_yaw_speed = 0.05;   // 5 deg/s
								}

								if (fabs(input_yaw_rate) < target_yaw_speed)
								{
									pos_mode_last_yaw += (turn_request_direction*kp_yawv);
//									float v_diff = (turn_request_direction*_param_gndspeed_trim_yaw.get()) - input_yaw_rate;
//									pos_mode_last_yaw = (_param_gnd_yawrate_p.get()*v_diff);
								}
								else
								{
									pos_mode_last_yaw -= (turn_request_direction*kp_yawv);
								}

							}
							else
							{
								if 	(fabs (turn_request) < (double)_param_gnd_turn_min.get())
								{
//									if(refine_yaw > 20)
//										PX4_INFO("RoverPositionControl::stopped correction due to refined yaw limit");
//									else
//										PX4_INFO("RoverPositionControl::stopped correction got target: %f", turn_request);

									pos_mode_target_bearing = demo_mode_target_bearing;

									if (demo_mode_target_bearing <= 0.0)
										demo_mode_target_bearing = 3.141;
									else
										demo_mode_target_bearing = 0.0;

									_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0.0;

									_param_set_angle.set(0);
									_param_set_angle.commit_no_notification();
								}
								actual_turn_direction = 0;
								pos_mode_last_yaw = 0.0f;
							}

						}
						else
						{
							if (current_yaw_mag> 0.05f)
							{
									if (fabs(input_yaw_rate) < _param_gndspeed_trim_yaw.get() )
										pos_mode_last_yaw += yaw_cmd_direction*_param_gnd_yawrate_p.get();
							}
						}
					}


//					float sp_error = (yaw_cmd_direction*_param_gndspeed_trim_yaw.get() ) - input_yaw_rate;
//
//					float  cmd_yaw = sp_error * _param_gnd_yawrate_p.get();
//
//					cmd_yaw = math::constrain(cmd_yaw, -0.8f, 0.8f);
//
//					PX4_INFO("RoverPositionControl::YAW: %f %f %f",
//							(double) input_yaw_rate,
//							(double) input_yaw_cmd,
//							(double) cmd_yaw,
//							(double) (yaw_cmd_direction*_param_gndspeed_trim_yaw.get()));
//
//
//					float current_yaw_mag = fabs(input_yaw_cmd);
//					if (current_yaw_mag > 0.05f)
//					{
////						if (current_yaw_mag < fabs(cmd_yaw))
////							pos_mode_last_yaw = input_yaw_cmd ;
////						else
//							pos_mode_last_yaw = input_yaw_cmd- cmd_yaw;
//					}
//					else
//						pos_mode_last_yaw = 0.0f;


					_act_controls.control[actuator_controls_s::INDEX_ROLL] = 0;

					if (current_yaw_mag > 0.05f || (target_angle != 0.0))
					{
//						PX4_INFO("RoverPositionControl::YAW: %f %f %f",
//								(double) input_yaw_rate,
//								(double) _param_gndspeed_trim_yaw.get(),
//								(double) pos_mode_last_yaw);

						_act_controls.control[actuator_controls_s::INDEX_YAW] = pos_mode_last_yaw;

						if (target_throttle > 0.01)
						{
							_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = pos_mode_last_vel;

						}
						else
						{
							_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0;
						}
					}
					else if (fabs(_manual_control_setpoint.z) > 0.05f)
					{
						_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
						_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
								pos_mode_last_vel;
						pid_reset_integral(&_brake_ctrl);
					}
					else
					{
//						matrix::Vector3f current_velocity(_local_pos.vx, _local_pos.vy,
//								_local_pos.vz);
//
//						 mission_target_speed = 0;
//						// Velocity in body frame
//						const Dcmf man_R_to_body(Quatf(_vehicle_att.q).inversed());
//						const Vector3f man_vel = man_R_to_body
//								* Vector3f(current_velocity(0), current_velocity(1),
//										current_velocity(2));
//
//						const float x_man_vel = man_vel(0);
//						const float x_man_acc = _vehicle_acceleration_sub.get().xyz[0];
//
//						mission_target_speed = 0;
//
//		   			   float  mission_throttle = pid_calculate(&_brake_ctrl, mission_target_speed, x_man_vel,
//		   					x_man_acc, dt);
//
//
//						// Constrain throttle between min and max
//						mission_throttle = math::constrain(mission_throttle,
//								-1.0f, 1.0f);
//
//						_act_controls.control[actuator_controls_s::INDEX_THROTTLE] += mission_throttle;
//						_act_controls.control[actuator_controls_s::INDEX_YAW] = 0;

						_act_controls.control[actuator_controls_s::INDEX_YAW] = 0;

						if (fabs(current_pitch_body) >=0.0872665)
						{
							_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = pos_mode_last_vel;
						}
						else
						{
							_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0;
						}
					}

//					PX4_INFO("RoverPositionControl::MotorControl: %f %f",  (double)pos_mode_last_vel,
//							 (double)_act_controls.control[actuator_controls_s::INDEX_THROTTLE]);
				}
			}
		}

		if (fds[2].revents & POLLIN) {

			orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub,
					&_sensor_combined);


			//PX4_ERR("Ang: %f %f %f", (double) _sensor_combined.gyro_rad[0],  (double) _sensor_combined.gyro_rad[1],  (double) _sensor_combined.gyro_rad[2]);
			//orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
			_act_controls.timestamp = hrt_absolute_time();

			/* Only publish if any of the proper modes are enabled */
			if (_control_mode.flag_control_velocity_enabled
					|| _control_mode.flag_control_attitude_enabled
					|| _control_mode.flag_control_position_enabled
					|| manual_mode) {
				/* publish the actuator controls */
				_actuator_controls_pub.publish(_act_controls);

			}
		}

	}

	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_manual_control_setpoint_sub);
	orb_unsubscribe(_pos_sp_triplet_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_sensor_combined_sub);

	warnx("exiting.\n");
}

int RoverPositionControl::task_spawn(int argc, char *argv[]) {
	/* start the task */
	_task_id = px4_task_spawn_cmd("rover_pos_ctrl", SCHED_DEFAULT,
	SCHED_PRIORITY_POSITION_CONTROL, 1700,
			(px4_main_t) &RoverPositionControl::run_trampoline, nullptr);

	if (_task_id < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

RoverPositionControl* RoverPositionControl::instantiate(int argc,
		char *argv[]) {

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}

	RoverPositionControl *instance = new RoverPositionControl();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate RoverPositionControl object");
	}

	return instance;
}

int RoverPositionControl::custom_command(int argc, char *argv[]) {
	return print_usage("unknown command");
}

int RoverPositionControl::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
$ rover_pos_control start
$ rover_pos_control status
$ rover_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_pos_control_main(int argc, char *argv[]) {
	return RoverPositionControl::main(argc, argv);
}
