uint64 timestamp                # time since system start (microseconds)

# Magnetometer data
float32[3] magnetometer_ga      # magnetic field in body frame (Gauss)

# Barometer data
float32 pressure_pa             # pressure (Pa)

float32 temperature             # temperature (Celsius)

# Bitmask indicating which fields have new data
uint8 fields_updated            # bit 0: mag updated, bit 1: baro updated
uint8 FIELD_MAG = 1
uint8 FIELD_BARO = 2