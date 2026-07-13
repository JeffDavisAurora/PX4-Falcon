#include "AAOBLTR.hpp"
#include <px4_platform_common/log.h>

matrix::Matrix<float, 11, 1> AAOBLTR::evaluateRBF(const float rate) {
	
	matrix::Matrix<float, 11, 1> centers = _filter.getRBF();

	float dMax = abs(centers(1,0) - centers(0,0));
	float sigma = sqrtf((dMax * dMax) / (-8.0f * logf(sqrtf(0.5f))));

	matrix::Matrix<float, 11, 1> phi;
	phi.setZero();

	for(int i = 0; i < 11-1; ++i) {
		float diff = rate - centers(i,0);
		phi(i,0) = expf(-(diff * diff) / (2.0f * sigma * sigma));
	}

	return phi;
}

void AAOBLTR::calculateAdaptation(const float rate) {
	
	matrix::Matrix<float, 11, 1> torque_rbf = evaluateRBF(rate);

	for(int i = 0; i < 11; ++i) {
		_phi_bar(i,0) = torque_rbf(i,0);
	}
	_phi_bar(11, 0) = _baseline_torque; // adding a new element to phi_bar

	float adaptation_torque = -(_torque_theta_hat.T() * _phi_bar)(0,0); // getting the scalar value
	_adaptation_torque = adaptation_torque;
}

void AAOBLTR::updateWeights(const float rate, const float rate_int, const float dt) {
	matrix::Matrix<float, 2, 1> current_rate_state;
	current_rate_state(0,0) = rate_int;
	current_rate_state(1,0) = rate;

	matrix::Matrix<float, 2, 1> x_hat = _filter.getXHat();
	matrix::Matrix<float, 2, 1> e_state = current_rate_state - x_hat; // (2x1) calculate the error vector
	
	float mu = muDeadzone(e_state, dt);
	matrix::Matrix<float, 2, 1> eDeadzone_torque = e_state * mu;
	matrix::Matrix<float, 1, 2> eDeadzone_T = eDeadzone_torque.transpose();  // (1x2) Take the transponse of the the deadzone torque vector
	matrix::Matrix<float, 12, 2> tmp_12x2 = _phi_bar * eDeadzone_T;   // (12x2) Multiply phi bar with deadzone transpose
	matrix::Matrix<float, 2, 2> _P_lyap_surge = _filter.getLyap(); // Get the lyap function
	matrix::SquareMatrix<float, 2> _P_lyap_surge_square = _P_lyap_surge; // Convert the lyap to a square matrix
	matrix::Matrix<float, 2, 2> _P_lyap_surge_INV = _P_lyap_surge_square.I(); // Calculate the inverse of the lyap matrix
	matrix::Matrix<float, 12, 2> tmp_12x2b = tmp_12x2 * _P_lyap_surge_INV;   // (12x2) multiply the inverse of lyap with the product of phi bar and deadzone
	matrix::Matrix<float, 2, 1> _B_torque_ol_ext;
	_B_torque_ol_ext.setZero();
	_B_torque_ol_ext(1,0) = _B_torque_ol;  // (2x1) Squared up B-matrix
	matrix::Matrix<float, 12, 1> tmp_12x1 = tmp_12x2b * _B_torque_ol_ext;     // (12x1)
	
	matrix::Matrix<float, 12, 1> torque_theta_hat_dot = -_theta_gain_torque * tmp_12x1; // Calculate theta hat dot 

	matrix::Matrix<float, 12, 1> torque_theta_hat_dot_proj = rectangularProjection(_torque_theta_hat,
									torque_theta_hat_dot,
									_eps_theta_torque,
									_theta_hat_max_torque,
									_theta_hat_min_torque,
									dt);
		
	_torque_theta_hat = _torque_theta_hat + torque_theta_hat_dot_proj * dt;
}

matrix::Matrix<float, 12, 1> AAOBLTR::rectangularProjection(
    const matrix::Matrix<float, 12, 1> &thetaHat,
    const matrix::Matrix<float, 12, 1> &thetaHatDot,
    float epsTheta,
    float thetaHatMax,
    float thetaHatMin,
    float dt)
{

	matrix::Matrix<float, 12, 1> thetaHatDot_proj = thetaHatDot;

	for (int i = 0; i < 12; ++i) {
		float theta_i    = thetaHat(i, 0);
		float thetaDot_i = thetaHatDot_proj(i, 0);

		if (theta_i > thetaHatMax - epsTheta && thetaDot_i > 0.0f) {
		    thetaHatDot_proj(i, 0) =
			(thetaHatMax - theta_i) * thetaDot_i * (1.0f / epsTheta);
		}

		if (theta_i < thetaHatMin + epsTheta && thetaDot_i < 0.0f) {
		    thetaHatDot_proj(i, 0) =
			(theta_i - thetaHatMin) * thetaDot_i * (1.0f / epsTheta);
		}
	}

	// Update thetaHat
	matrix::Matrix<float, 12, 1> thetaHatNew  = thetaHat + dt * thetaHatDot_proj;

	// Second pass: ensure next step does not exceed bounds
	for (int i = 0; i < 12; ++i) {
		float thetaNew_i = thetaHatNew(i, 0);
		float theta_i    = thetaHat(i, 0);

		if (thetaNew_i > thetaHatMax) {
		    thetaHatDot_proj(i, 0) = (thetaHatMax - theta_i) / dt;
		}

		if (thetaNew_i < thetaHatMin) {
		    thetaHatDot_proj(i, 0) = (thetaHatMin - theta_i) / dt;
		}
	}

	return thetaHatDot_proj;
}

float AAOBLTR::muDeadzone(const matrix::Matrix<float, 2, 1> &e, const float dt) {
	
	float sum_sq = 0.0f;
	// manually calculate the norm of the error vector
	for (int i = 0; i < 2; i++) {
		float v = e(i, 0);
		sum_sq += v * v;
	}
	float e_norm = sqrtf(sum_sq);

	float numerator   = e_norm - dt * _eNullDeadzone;
	float denominator = (1.0f - dt) * _eNullDeadzone;

	float raw = numerator / denominator;

	float eDeadzone = math::max(0.0f, math::min(1.0f, raw));
	return eDeadzone;
}

float AAOBLTR::update(float rate,
                  float rate_error,
                  float rate_int,
                  float rate_int_prev,
                  float angular_accel,
                  float rate_sp,
                  float dt,
                  bool landed) {
	_e_yI = rate_int;
	matrix::Matrix<float, 2, 1> x_hat = _filter.getXHat();
	matrix::Matrix<float, 2, 1> y_hat = x_hat;

	float torque_sat = math::constrain(_torque, _saturation_negative,
							_saturation_positive);
	matrix::Matrix<float, 2, 1> y_ext_meas;
	y_ext_meas(0,0) = _e_yI;
	y_ext_meas(1,0) = rate;

	float deltaFcim_surge = 0.0f;
	matrix::Matrix<float, 2, 1> new_x_hat = _filter.updateXHat(torque_sat, y_ext_meas,
											y_hat, rate_sp, deltaFcim_surge, dt);
	_new_x_hat_eI = new_x_hat(0,0);
	_new_x_hat_omega = new_x_hat(1,0);

	float torque =  - _gain_i * new_x_hat(0,0)
					- _gain_p * new_x_hat(1,0) 
					+ _gain_ff * rate_sp;
	_baseline_torque = torque;
	
	calculateAdaptation(rate);
	updateWeights(rate, rate_int, dt);
	_torque = _baseline_torque + _adaptation_torque;
        
	return _torque;
}

void AAOBLTR::reset()
{
	_e_yI = 0.f;
	_torque = 0.f;
}
