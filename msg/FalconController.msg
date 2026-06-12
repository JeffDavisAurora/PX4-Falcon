int32 controller_type

# Roll/pitch/yaw
float32[3] proportional_gain
float32[3] integral_gain
float32[3] derivative_gain
float32[3] feedfoward_gain

uint64 timestamp

# Rate Errors
float32 roll_rate_error
float32 pitch_rate_error
float32 yaw_rate_error

# Integrals
float32 roll_rate_integral
float32 pitch_rate_integral
float32 yaw_rate_integral

# Outputs
float32 roll_torque
float32 pitch_torque
float32 yaw_torque


# Setpoints
float32 roll_rate_sp
float32 pitch_rate_sp
float32 yaw_rate_sp


# Actual Rates
float32 roll_rate
float32 pitch_rate
float32 yaw_rate

# Observer Integral
float32 roll_ei
float32 pitch_ei
float32 yaw_ei

# Observer Omega
float32 roll_omega
float32 pitch_omega
float32 yaw_omega
