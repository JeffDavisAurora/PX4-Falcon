#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include "RateControllerBase.hpp"

class OBLTR : public RateControllerBase
{
public:
        OBLTR(float saturation_positive,
            float saturation_negative)
        : RateControllerBase(saturation_positive, saturation_negative)
        {}


        float update(float rate_error,
                     float rate_int,
                     float angular_accel,
                     float rate_sp) override;
};
