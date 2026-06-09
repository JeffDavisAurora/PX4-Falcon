#pragma once

class RateControllerBase {
public:
	virtual ~RateControllerBase() = default;

	virtual float update(float rate, float rate_error, float rate_int, float angular_accel, float rate_sp) = 0;

	virtual void set_gains(float p, float i, float d) {
		_gain_p = p;
		_gain_i = i;
		_gain_d = d;
	}

	virtual void set_ff(float ff) { _gain_ff = ff; }

	virtual void reset() {}

protected:
	RateControllerBase(float saturation_positive,
			   float saturation_negative)
	: _gain_p{0.0f}
	, _gain_i{0.0f}
	, _gain_d{0.0f}
	, _gain_ff{0.0f}
	, _saturation_positive{saturation_positive}
	, _saturation_negative{saturation_negative}
	{}

	float _gain_p;
	float _gain_i;
	float _gain_d;
	float _gain_ff;
	float _saturation_positive;
	float _saturation_negative;
};
