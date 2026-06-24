/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rate_control_falcon.cpp
 */

#include "rate_control_falcon.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

using namespace matrix;

RateControlFalcon::RateControlFalcon()
{

	m_filt_roll.setaExt("0,1;"
			    "0,0;");
	m_filt_roll.setbExt("0;"
			    "41.93278258816682;");
	m_filt_roll.setbCmd("-1;"
		            "0;");
	m_filt_roll.setl("16.701,0.79697;"
			 "39.848,65.56;");

	m_filt_pitch.setaExt("0,1;"
			    "0,0;");
	m_filt_pitch.setbExt("0;"
			    "41.75257258475981;");
	m_filt_pitch.setbCmd("-1;"
		            "0;");
	m_filt_pitch.setl("16.701,0.79697;"
			 "39.848,65.56;");

	m_filt_yaw.setaExt("0,1;"
			    "0,0;");
	m_filt_yaw.setbExt("0;"
    			    "22.7267872004552622;");
	m_filt_yaw.setbCmd("-1;"
		            "0;");
	m_filt_yaw.setl("13.451,0.8926;"
			 "4.463,111.79;");
}

void RateControlFalcon::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;

	_roll_controller->set_gains(_gain_p(0), _gain_i(0), _gain_d(0));
	_pitch_controller->set_gains(_gain_p(1), _gain_i(1), _gain_d(1));
	_yaw_controller->set_gains(_gain_p(2), _gain_i(2), _gain_d(2));

	// TODO: do this but good
	_roll_controller->set_ff(_gain_p(0));
	_pitch_controller->set_ff(_gain_p(1));
	_yaw_controller->set_ff(_gain_p(2));
}

void RateControlFalcon::setFeedForwardGain(const Vector3f &FF)
{
	_gain_ff = _gain_p;

	// _roll_controller->set_ff(0);
	// _pitch_controller->set_ff(0);
	// _yaw_controller->set_ff(_gain_p);
}

void RateControlFalcon::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControlFalcon::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}


void RateControlFalcon::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

Vector3f RateControlFalcon::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	Vector3f rate_error = rate_sp - rate;	
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	float roll_torque 	= _roll_controller->update(rate(0), rate_error(0),
			_rate_int(0), _rate_int_prev(0), angular_accel(0), rate_sp(0),
			dt, landed);
	float pitch_torque 	= _pitch_controller->update(rate(1), rate_error(1),
		       	_rate_int(1), _rate_int_prev(1), angular_accel(1), rate_sp(1),
			dt, landed);
	float yaw_torque 	= _yaw_controller->update(rate(2), rate_error(2),
		       	_rate_int(2), _rate_int_prev(2), angular_accel(2), rate_sp(2),
			dt, landed);
	Vector3f torque = {roll_torque, pitch_torque, yaw_torque};
	Vector3f obs_eI = {0.f, 0.f, 0.f};
	Vector3f obs_omega = {0.f, 0.f, 0.f};

	if(_active_ctrl_type == 2) {

		auto* r = static_cast<OBLTR*>(_roll_controller);
		auto* p = static_cast<OBLTR*>(_pitch_controller);
		auto* y = static_cast<OBLTR*>(_yaw_controller);
		obs_eI = {r->getXHatEI(),
			  p->getXHatEI(),
		          y->getXHatEI()};

		obs_omega = {r->getXHatOmega(),
			     p->getXHatOmega(),
		             y->getXHatOmega()};
	}
	publishStatus(rate_error, rate_sp, rate, torque, obs_eI, obs_omega);
	
	return torque;
}

void RateControlFalcon::publishStatus(const Vector3f &rate_error,
				const Vector3f &rate_sp,
                		const Vector3f &rate, 
				const Vector3f &torque,
				const Vector3f &obs_eI,
				const Vector3f &obs_omega)
{
	//falcon_rate_control_messages
        falcon_controller_s status{};
        status.timestamp = hrt_absolute_time();

        //Controller type
        status.controller_type = _active_ctrl_type;

        // PID Gains
        status.proportional_gain[0] = _gain_p(0);
        status.proportional_gain[1] = _gain_p(1);
        status.proportional_gain[2] = _gain_p(2);

        status.integral_gain[0] = _gain_i(0);
        status.integral_gain[1] = _gain_i(1);
        status.integral_gain[2] = _gain_i(2);

        status.derivative_gain[0] = _gain_d(0);
        status.derivative_gain[1] = _gain_d(1);
        status.derivative_gain[2] = _gain_d(2);

	status.feedfoward_gain[0] = _gain_ff(0);
	status.feedfoward_gain[1] = _gain_ff(1);
	status.feedfoward_gain[2] = _gain_ff(2);

        // Rate Errors
        status.roll_rate_error = rate_error(0);
        status.pitch_rate_error = rate_error(1);
        status.yaw_rate_error = rate_error(2);

        // Integrals
        status.roll_rate_integral = _rate_int(0);
        status.pitch_rate_integral = _rate_int(1);
        status.yaw_rate_integral = _rate_int(2);

        // Outputs
        status.roll_torque = torque(0);
        status.pitch_torque = torque(1);
        status.yaw_torque = torque(2);

        // Setpoints
        status.roll_rate_sp = rate_sp(0);
        status.pitch_rate_sp = rate_sp(1);
        status.yaw_rate_sp = rate_sp(2);

        // Actual Rates
        status.roll_rate = rate(0);
        status.pitch_rate = rate(1);
        status.yaw_rate = rate(2);

	// Observer Integral
	status.roll_ei = obs_eI(0);
	status.pitch_ei = obs_eI(1);
	status.yaw_ei = obs_eI(2);
	
	// Observer Omega
	status.roll_omega = obs_omega(0);
	status.pitch_omega = obs_omega(1);
	status.yaw_omega = obs_omega(2);

        _falcon_status_pub.publish(status);
}

void RateControlFalcon::updateIntegral(Vector3f &rate_error, const float dt)
{

	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		float rate_i;
		float rate_i_real;
		//if(_active_ctrl_type == 2) {
			rate_i = _rate_int(i) - rate_error(i) * dt;
		//}
			// Perform the integration using a first order method
		rate_i_real = _rate_int_prev(i) + i_factor * _gain_i(i) * rate_error(i) * dt;
		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
			_rate_int_prev(i) = math::constrain(rate_i_real, -_lim_int(i), _lim_int(i));
		}
	}
}


void RateControlFalcon::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}

void RateControlFalcon::switchController(int32_t type)
{
	if(type == _active_ctrl_type) return;

	delete _roll_controller;
	delete _pitch_controller;
	delete _yaw_controller;

	const char *controller_name = "RSLQR";

	switch(type){
	case 0:
		_roll_controller = new RSLQR(1.0f, -1.0f);
		_pitch_controller = new RSLQR(1.0f, -1.0f);
		_yaw_controller = new RSLQR(1.0f, -1.0f);
		controller_name = "RSLQR";
		break;
	case 1:
		_roll_controller = new PID(1.0f, -1.0f);
		_pitch_controller = new PID(1.0f, -1.0f);
		_yaw_controller = new PID(1.0f, -1.0f);
		controller_name = "PID";
		break;
	case 2:
	/*	_roll_controller = new OBLTR(1.0f, -1.0f,
					m_filt_roll);
		_pitch_controller = new OBLTR(1.0f, -1.0f,
					m_filt_pitch);
		_yaw_controller = new OBLTR(1.0f, -1.0f,
					m_filt_yaw);*/
		controller_name = "OBLTR";
		break;
	default:
		_roll_controller = new RSLQR(1.0f, -1.0f);
		_pitch_controller = new RSLQR(1.0f, -1.0f);
		_yaw_controller = new RSLQR(1.0f, -1.0f);
		controller_name = "RSLQR (default)";
		break;
	}
	_active_ctrl_type = type;
	PX4_INFO("Switched to controller: %d (%s)", (int)_active_ctrl_type, controller_name);
}

RateControlFalcon::~RateControlFalcon()
{
	delete _roll_controller;
	delete _pitch_controller;
	delete _yaw_controller;
}
