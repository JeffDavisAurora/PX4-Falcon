#include "RSLQR.hpp"
#include <px4_platform_common/defines.h>

RSLQR::RSLQR(float p_gain, float i_gain, float sat_pos, float sat_neg, float lim_int){
    _proportional_gain = p_gain;
    _integral_gain = i_gain;
    _saturation_positive = sat_pos;
    _saturation_negative = sat_neg;
    _integral_limit = lim_int;
    printf("Creating RSLQR Controller");
}

RSLQR::~RSLQR()
{
}

float RSLQR::update(float state, float state_setpoint, const float dt, const bool landed)
{
    
    float rate_error = state_setpoint - state;

    // TODO: flip signs to match implementation described by orange book
    float torque = _proportional_gain * rate_error + _rate_int;
    // float torque = proportional_gain * rate_error + integral_value + feed_forward_gain * state_setpoint;

    // update integral only if we are not landed
    if (!landed) {
        update_integral(rate_error, dt);
    }

    return torque; // replace with actual torque calculation
}

void RSLQR::update_integral(float &rate_error, float dt)
{
    // prevent further positive control saturation
    if (_saturation_positive < 0.001f) {
        rate_error = math::min(rate_error, 0.f);
    }

    // prevent further negative control saturation
    if (_saturation_negative < 0.001f) {
        rate_error = math::max(rate_error, 0.f);
    }

    // I term factor: reduce the I gain with increasing rate error.
    // This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
    // change (noticeable in a bounce-back effect after a flip).
    // The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
    // with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
    // and up to 200 deg error leads to <25% reduction of I.
    float i_factor = rate_error / math::radians(400.f);
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

    // Perform the integration using a first order method
    float rate_i = _rate_int + i_factor * _integral_gain * rate_error * dt;

    // do not propagate the result if out of range or invalid
    if (PX4_ISFINITE(rate_i)) {
        _rate_int = math::constrain(rate_i, -_integral_limit, _integral_limit);
    }
}