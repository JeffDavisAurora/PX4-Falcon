#pragma once

#include <vector>
#include <string>
#include <matrix/matrix/math.hpp>
#include "MatrixUtils.h"

class OBLTRFilter
{
public:
	OBLTRFilter();
	~OBLTRFilter() {};
	bool setaExt(const std::string &spec) { return parseMatrixString<2, 2>(m_aExt, spec); }
	bool setbExt(const std::string &spec) { return parseMatrixString<2, 1>(m_bExt, spec); }
	bool setbCmd(const std::string &spec) { return parseMatrixString<2, 1>(m_bCmd, spec); }
	bool setl(const std::string &spec)    { return parseMatrixString<2, 2>(m_l, spec)   ; }
	
	std::vector<float> updateXHat(float u_ctrl, const std::vector<float> y_ext_meas,
		const std::vector<float> y_hat, float y_cmd, float deltaFcim,
		float dt);

	std::vector<float> getXHat();

	void reset();

	std::string getSpec();
private:

	bool m_soft_start;

	matrix::Matrix<float, 2, 1>  m_xhat;
	
	matrix::Matrix<float, 2, 2> m_aExt;
	matrix::Matrix<float, 2, 1> m_bExt;
	matrix::Matrix<float, 2, 1> m_bCmd;
	matrix::Matrix<float, 2, 2> m_l;

	matrix::Matrix<float, 2, 1> m_bFcim;
};
