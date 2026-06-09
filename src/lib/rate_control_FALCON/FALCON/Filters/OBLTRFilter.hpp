#pragma once

#include <vector>
#include <string>
#include <armadillo>
#include "ArmaMatrixUtils.h"

class OBLTRFilter
{
public:
	OBLTRFilter();
	OBLTRFilter(int state_size);
	~OBLTRFilter() {};
	bool setaExt(std::string spec) {return( parseMatrixString(m_aExt, spec) );};
	bool setbExt(std::string spec) {return( parseMatrixString(m_bExt, spec) );};
	bool setbCmd(std::string spec) {return( parseMatrixString(m_bCmd, spec) );};
	bool setl(std::string spec)    {return( parseMatrixString(m_l,    spec) );};
	
	std::vector<float> updateXHat(float u_ctrl, std::vector<float> y_ext_meas,
		std::vector<float> y_hat, float y_cmd, float deltaFcim,
		float dt);

	std::vector<float> getXHat();

	void reset();

	std::string getSpec();

private:

	bool m_soft_start;

	arma::fvec m_xhat;
	
	arma::fmat m_aExt;
	arma::fmat m_bExt;
	arma::fmat m_bCmd;
	arma::fmat m_l;
	arma::fmat m_bFcim;
};
