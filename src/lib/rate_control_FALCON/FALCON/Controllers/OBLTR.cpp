#include "OBLTR.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

float OBLTR::update(float rate,
		  float rate_error,
                  float rate_int,
                  float angular_accel,
                  float rate_sp)
{
        float torque = - _gain_p * rate//gain_p is equivallent to gain_x
		       - _gain_i * rate_int; 	
	return torque;
}
