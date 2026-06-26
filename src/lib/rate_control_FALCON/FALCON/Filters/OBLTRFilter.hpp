#pragma once

#include <matrix/matrix/math.hpp>
#include "MatrixUtils.h"

class OBLTRFilter
{
public:
	OBLTRFilter();
	~OBLTRFilter() {};
	bool setaExt(const char *spec) { return parseMatrixString<2, 2>(m_aExt, spec); }
	bool setbExt(const char *spec) { return parseMatrixString<2, 1>(m_bExt, spec); }
	bool setbCmd(const char *spec) { return parseMatrixString<2, 1>(m_bCmd, spec); }
	bool setl(const char *spec)    { return parseMatrixString<2, 2>(m_l, spec)   ; }
	bool setrbf(const char *spec)  { return parseMatrixString<11, 1>(m_rbf, spec); }
	bool setlyap(const char *spec)  { return parseMatrixString<2, 2>(m_lyap, spec); }
	matrix::Matrix<float, 2, 1> updateXHat(float u_ctrl, 
			const matrix::Matrix<float, 2, 1> y_ext_meas,
			const matrix::Matrix<float, 2, 1> y_hat, float y_cmd, float deltaFcim,
			float dt);

	matrix::Matrix<float, 2, 1> getXHat();
	matrix::Matrix<float, 11, 1> getRBF();
	matrix::Matrix<float, 2, 2> getLyap();
	
	void reset();

	//std::string getSpec();
private:

	bool m_soft_start;

	matrix::Matrix<float, 2, 1>  m_xhat;
	
	matrix::Matrix<float, 2, 2> m_aExt;
	matrix::Matrix<float, 2, 1> m_bExt;
	matrix::Matrix<float, 2, 1> m_bCmd;
	matrix::Matrix<float, 2, 2> m_l;
	matrix::Matrix<float, 11, 1> m_rbf;
	matrix::Matrix<float, 2, 2> m_lyap;

	matrix::Matrix<float, 2, 1> m_bFcim;
};
