#include <px4_platform_common/log.h>
#include "OBLTRFilter.hpp"

OBLTRFilter::OBLTRFilter()
{
	m_soft_start = true; 
}

//---------------------------------------------------
// updateXHat:          main update function
//                inputs:     u_control_sat = saturated control input
//                        y_ext_meas = external measurment of y
//                             y_hat = latest estimate of output y
//                             y_cmd = reference signal for y
//                                dt = delta time

matrix::Matrix<float, 2, 1> OBLTRFilter::updateXHat(float u_control_sat, 
              const matrix::Matrix<float, 2, 1> y_ext_meas,
              const matrix::Matrix<float, 2, 1> y_hat, float y_cmd, float deltaFcim,
              float dt)
{
  // convert the input cpp vectors to arma vectors since we want the arma lib
  // to be completely encapsilated.
  matrix::Matrix<float, 2, 1> ret_vec;	
  matrix::Matrix<float, 2, 1> matrix_y_ext_meas = y_ext_meas;	
  matrix::Matrix<float, 2, 1> matrix_y_hat = y_hat;	
  
  // Checks
  if (dt <=0.0f) {
	  ret_vec.setZero();
	  return(ret_vec);
  }

  // If we have recently cleared the filter we need to do a soft start
  // Set the current state, xhat, and the observed state yhat
  if (m_soft_start){
    m_xhat       = y_ext_meas;
    matrix_y_hat = y_ext_meas; 
    m_soft_start = false; 
  }

  matrix::Matrix<float, 2, 1> dx_hat = m_aExt * m_xhat + m_bExt * u_control_sat;

  dx_hat = dx_hat + m_bCmd * y_cmd; 

  dx_hat = dx_hat + m_l * (matrix_y_ext_meas - matrix_y_hat);

  m_bFcim.setZero();

  dx_hat = dx_hat + deltaFcim * m_bFcim;
  m_xhat = m_xhat + dx_hat * dt;
  return(getXHat()); 
}

//----------------------------------------------------
//  getXHat()      returns x hat

matrix::Matrix<float, 2, 1> OBLTRFilter::getXHat()
{
  return m_xhat; 
}

//----------------------------------------------------
//  getRBF()      returns rbf

matrix::Matrix<float, 11, 1> OBLTRFilter::getRBF()
{
  return m_rbf;
}

//----------------------------------------------------
//  getRBF()      returns rbf

matrix::Matrix<float, 2, 2> OBLTRFilter::getLyap()
{
  return m_lyap;
}

//----------------------------------------------------
// reset()

void OBLTRFilter::reset()
{
  m_xhat.setZero();
  m_soft_start = true; 
  return; 

}
//----------------------------------------------------
// getSpec()
/*
std::string OBLTRFilter::getSpec() {
	
	std::string spec = "";

        spec += "x_hat = "  + getMatSpec(m_xhat);
  	spec += ", aExt = " + getMatSpec(m_aExt);
    	spec += ", bExt = " + getMatSpec(m_bExt);
      	spec += ", bCmd = " + getMatSpec(m_bCmd);
        spec += ", l = "    + getMatSpec(m_l);
	return(spec); 
}
*/
