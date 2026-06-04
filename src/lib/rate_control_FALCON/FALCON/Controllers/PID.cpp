#include "PID.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

float PID::update(float rate_error,
		  float rate_int,
		  float angular_accel,
		  float rate_sp)
{
	float torque = _gain_p * rate_error + rate_int - _gain_d * angular_accel + _gain_ff * rate_sp;
	return torque;
}
