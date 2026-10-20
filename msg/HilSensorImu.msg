uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # simulator time of the sample, HIL_SENSOR.time_usec (microseconds)

float32[3] accelerometer_m_s2   # acceleration in body frame (m/s^2)
float32[3] gyroscope_rad_s      # angular velocity in body frame (rad/s)
float32 temperature             # temperature (Celsius)

uint8 ORB_QUEUE_LENGTH = 16     # dsp_hitl drains every sample; several can arrive between polls
