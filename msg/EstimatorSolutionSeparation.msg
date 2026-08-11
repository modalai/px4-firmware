uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # timestamp of the delayed fusion horizon

# separation d = x_B [-] x_A between the EV-fused primary (B) and the EV-free
# companion (A), with 1-sigma bounds from the exact separation covariance D

float32[3] d_pos                # NED position separation (m)
float32[3] d_vel                # NED velocity separation (m/s)
float32 d_yaw                   # yaw separation (rad)

float32[3] sigma_pos            # sqrt(diag D) position (m)
float32[3] sigma_vel            # sqrt(diag D) velocity (m/s)
float32 sigma_yaw               # sqrt(D yaw) (rad)

float32[5] test_ratio           # chi-square test ratio per block: pos_h, pos_z, vel_h, vel_z, yaw
float32[5] cusum                # CUSUM level per block

float32 window_age_s            # time since last companion clone (s)
uint32 clone_count              # companion clone counter

uint8 MONITOR_STATE_DISABLED = 0
uint8 MONITOR_STATE_NOMINAL = 1
uint8 MONITOR_STATE_ALERT = 2
uint8 MONITOR_STATE_PURGED = 3
uint8 monitor_state

bool ev_inhibited               # EV fusion currently quarantined
