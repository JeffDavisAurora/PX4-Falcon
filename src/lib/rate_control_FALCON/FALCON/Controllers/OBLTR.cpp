#include "OBLTR.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

float OBLTR::update(float rate_error,
                  float rate_int,
                  float angular_accel,
                  float rate_sp)
{
	float omega_hat = rate_sp - rate_error;
        float torque = -_gain_i * rate_int
		       -_gain_p * omega_hat; //gain_p is equivallent to gain_x

	return torque;
}
