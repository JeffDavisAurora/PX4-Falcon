#pragma once
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include "RateControllerBase.hpp"
#include "FALCON/Filters/OBLTRFilter.hpp"

class AOBLTR : public RateControllerBase
{
public:
    AOBLTR(float saturation_positive,
           float saturation_negative,
           OBLTRFilter &filter,
           // tunable parameters with defaults matching your current values
           float eNullDeadzone      = 0.5f,
           float B_torque_ol        = 0.04f,
           float theta_gain_torque  = 0.00005f,
           float eps_theta_torque   = 0.0003f,
           float theta_hat_max      = 1.0f,
           float theta_hat_min      = -1.0f)
    : RateControllerBase(saturation_positive, saturation_negative)
    , _filter{filter}
    , _eNullDeadzone{eNullDeadzone}
    , _B_torque_ol{B_torque_ol}
    , _theta_gain_torque{theta_gain_torque}
    , _eps_theta_torque{eps_theta_torque}
    , _theta_hat_max_torque{theta_hat_max}
    , _theta_hat_min_torque{theta_hat_min}
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
    void calculateAdaptation(const float rate);
    void updateWeights(const float rate, const float rate_int, const float dt);
    matrix::Matrix<float, 11, 1> evaluateRBF(const float rate);

    float muDeadzone(const matrix::Matrix<float, 2, 1> &e, const float dt);

    matrix::Matrix<float, 12, 1> rectangularProjection(
        const matrix::Matrix<float, 12, 1> &thetaHat,
        const matrix::Matrix<float, 12, 1> &thetaHatDot,
        float epsTheta,
        float thetaHatMax,
        float thetaHatMin,
        float dt);

    float getXHatEI() const { return _new_x_hat_eI; }
    float getXHatOmega() const { return _new_x_hat_omega; }

private:
    OBLTRFilter &_filter;

    float _torque{0.f};
    float _baseline_torque{0.f};
    float _e_yI{0.f};
    float _new_x_hat_eI{0.f};
    float _new_x_hat_omega{0.f};
    float _adaptation_torque{0.f};

    // tunable parameters
    float _eNullDeadzone;        // deadzone radius
    float _B_torque_ol;          // input gain
    float _theta_gain_torque;    // adaptation gain
    float _eps_theta_torque;     // projection epsilon
    float _theta_hat_max_torque; // upper bound
    float _theta_hat_min_torque; // lower bound

    matrix::Matrix<float, 12, 1> _torque_theta_hat;
    matrix::Matrix<float, 12, 1> _torque_bar;
};
