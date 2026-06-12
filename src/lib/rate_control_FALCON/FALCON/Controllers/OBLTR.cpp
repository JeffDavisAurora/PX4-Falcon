#include "OBLTR.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

float OBLTR::update(float rate,
		  float rate_error,
                  float rate_int,
		  float rate_int_prev,
                  float angular_accel,
                  float rate_sp,
		  float dt,
		  bool landed)
{	
	_e_yI = rate_int;
	std::vector<float> x_hat = _filter.getXHat();
	std::vector<float> y_hat = x_hat;

	float torque_sat = std::clamp(_torque, _saturation_negative,
		       		_saturation_positive);
	std::vector<float> y_ext_meas;
	y_ext_meas.push_back(_e_yI);
	y_ext_meas.push_back(rate);

	float deltaFcim_surge = 0.0f;
	std::vector<float> new_x_hat = _filter.updateXHat(torque_sat, y_ext_meas, 
			                       y_hat, rate_sp, deltaFcim_surge, dt);	
  	_new_x_hat_eI = new_x_hat[0];
	_new_x_hat_omega = new_x_hat[1];

  	/*float torque =  _gain_i * new_x_hat[0] //gain_p is equivallent to gain_x
		        - _gain_p * new_x_hat[1];*/
	float torque = -_gain_p*rate + rate_int_prev + _gain_ff*rate_sp;
	_torque = torque;	
	return _torque;
}

void OBLTR::reset()
{
	_e_yI = 0.f;
	_torque = 0.f;
}
