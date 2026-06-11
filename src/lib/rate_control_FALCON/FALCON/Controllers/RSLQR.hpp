#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include "RateControllerBase.hpp"

class RSLQR : public RateControllerBase
{
public:
	RSLQR(float saturation_positive,
	      float saturation_negative)
	: RateControllerBase(saturation_positive, saturation_negative)
	{}

	float update(float rate,
		     float rate_error,
	             float rate_int,
		     float rate_int_prev,
		     float angular_accel,
		     float rate_sp,
		     float dt,
		     bool landed) override;
};
