#include "RSLQR.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

float RSLQR::update(float rate,
		    float rate_error,
		    float rate_int,
		    float rate_int_prev,
		    float angular_accel,
		    float rate_sp,
		    float dt)
{
	float torque = _gain_p * rate_error + rate_int - _gain_d * angular_accel;
	return torque;
}
