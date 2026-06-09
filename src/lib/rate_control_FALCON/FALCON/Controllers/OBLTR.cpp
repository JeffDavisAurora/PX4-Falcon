#include "OBLTR.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

float OBLTR::update(float rate,
		  float rate_error,
                  float rate_int,
		  float rate_int_prev,
                  float angular_accel,
                  float rate_sp,
		  float dt)
{
	std::vector<float> new_x_hat;
	new_x_hat.push_back(rate_int_prev);
	new_x_hat.push_back(rate);

	float torque_sat = std::clamp(rate, _saturation_negative,
		       		_saturation_positive);
	std::vector<float> y_ext_meas;
	y_ext_meas.push_back(rate_int);
	y_ext_meas.push_back(rate);

	std::vector<float> x_hat = _filter.getXHat();
	std::vector<float> y_hat = x_hat;

	float deltaFcim_surge = 0.0f;

	new_x_hat = _filter.updateXHat(torque_sat, y_ext_meas, y_hat, rate_sp,
		 		       deltaFcim_surge, dt);	

        float torque =  _gain_p * new_x_hat[1] //gain_p is equivallent to gain_x
		        +  rate_int; 	
	return torque;
}
