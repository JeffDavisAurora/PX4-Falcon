#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include "RateControllerBase.hpp"
#include "FALCON/Filters/OBLTRFilter.hpp"

class OBLTR : public RateControllerBase
{
public:
        OBLTR(float saturation_positive,
              float saturation_negative,
	      OBLTRFilter &filter)
        : RateControllerBase(saturation_positive, saturation_negative)
	, _filter{filter}
        {}


        float update(float rate,
		     float rate_error,
                     float rate_int,
		     float rate_int_prev,
                     float angular_accel,
		     float rate_sp,
		     float dt,
		     bool landed) override;

	void reset();
	
	float getXHatEI() const {return _new_x_hat_eI;}
	float getXHatOmega() const {return _new_x_hat_omega;}
private:
	
	OBLTRFilter &_filter;
	float _torque{0.f};
	float _e_yI{0.f};
	float _new_x_hat_eI{0.f};
	float _new_x_hat_omega{0.f};
};
