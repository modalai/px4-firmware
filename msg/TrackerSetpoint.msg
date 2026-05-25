# External tracker intent for the FOLLOW_ME flight task.
#
# Published by an off-board computer (e.g. a visual target tracker) to steer a
# multicopter horizontally and in yaw while PX4 keeps its normal altitude lock.
# This is *intent*, not raw pixel measurements: the producer is expected to have
# already converted image-space geometry into normalized control demands so the
# flight task does not need to understand the camera model.

uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the tracker observation this intent is derived from (microseconds)

# Horizontal steering intent, interpreted exactly like roll/pitch stick deflection.
# Range [-1, 1]. Mapped to a horizontal acceleration setpoint through MPC_MAN_TILT_MAX,
# identical to manual Altitude mode (forward pitch_body accelerates toward the nose).
float32 roll_body       # right (+) / left (-) intent, normalized [-1, 1]
float32 pitch_body      # forward (+) / backward (-) intent, normalized [-1, 1]

# Yaw steering intent as an angular rate around the body/NED z-axis, in radians/second.
# Clamped to MPC_MAN_Y_MAX. When ~0 the heading is locked, like Altitude mode.
float32 yaw_rate        # desired yaw rate (rad/s)

# Vertical steering intent as a velocity in the NED frame: positive = down (descend),
# negative = up (climb), in meters/second. Clamped to MPC_Z_VEL_MAX_UP/DN. When ~0 the
# vehicle falls back to the stock altitude lock (baro position hold), so it does not drift.
# Leave at 0 (or set valid=false) to keep altitude held.
float32 vz              # desired vertical velocity, NED (m/s, +down)

# Raw-ish tracking error and its rate, carried for logging / future feed-forward only.
# The flight task does not consume these in this version.
float32 error_x         # horizontal tracking error, normalized image frame
float32 error_y         # vertical tracking error, normalized image frame
float32 error_x_rate    # d(error_x)/dt
float32 error_y_rate    # d(error_y)/dt

float32 confidence      # tracker confidence [0, 1]; below 1 may be used to scale intent in future
bool valid              # true when roll_body/pitch_body/yaw_rate are usable this cycle
