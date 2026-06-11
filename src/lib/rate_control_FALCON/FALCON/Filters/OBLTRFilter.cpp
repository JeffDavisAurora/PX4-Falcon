#include "OBLTRFilter.hpp"
#include "px4_platform_common/log.h"

OBLTRFilter::OBLTRFilter()
{
	m_soft_start = true; 
}

OBLTRFilter::OBLTRFilter(int state_size)
{
  m_xhat.zeros(state_size);

  m_aExt.zeros(state_size, state_size);
  m_bExt.zeros(state_size, 1);
  m_bCmd.zeros(state_size, 1);
  m_l.zeros(state_size, state_size);
  PX4_INFO("Entered consturctor");
  m_bFcim.zeros(state_size, 1);
  m_soft_start = true; 
}



//---------------------------------------------------
// updateXHat:          main update function
//                inputs:     u_control_sat = saturated control input
//                        y_ext_meas = external measurment of y
//                             y_hat = latest estimate of output y
//                             y_cmd = reference signal for y
//                                dt = delta time

std::vector<float> OBLTRFilter::updateXHat(float u_control_sat, 
              std::vector<float> y_ext_meas,
              std::vector<float> y_hat, float y_cmd, float deltaFcim,
              float dt)
{
  // convert the input cpp vectors to arma vectors since we want the arma lib
  // to be completely encapsilated.
  arma::fvec arma_y_ext_meas = convVec(y_ext_meas);
  arma::fvec arma_y_hat      = convVec(y_hat);
  std::vector<float> ret_vec; 
  
  // Checks
  if (dt <=0.0f) {
	  return(ret_vec);
  }

  // If we have recently cleared the filter we need to do a soft start
  // Set the current state, xhat, and the observed state yhat
  if (m_soft_start){
    m_xhat     = arma_y_ext_meas;
    arma_y_hat = arma_y_ext_meas; 
    m_soft_start = false; 
  }

  arma::fvec dx_hat = m_aExt * m_xhat + m_bExt * u_control_sat;

  dx_hat = dx_hat + m_bCmd * y_cmd; 

  dx_hat = dx_hat + m_l * (arma_y_ext_meas - arma_y_hat);

  m_bFcim.zeros(m_xhat.n_rows, 1);

  dx_hat = dx_hat + deltaFcim * m_bFcim;
  m_xhat = m_xhat + dx_hat * dt;
  return(getXHat()); 
}

//----------------------------------------------------
//  getXHat()      returns x hat

std::vector<float> OBLTRFilter::getXHat()
{
  std::vector<float> ret_vec; 

  arma::fvec::iterator it;
  for (it = m_xhat.begin(); it != m_xhat.end(); it++){
    ret_vec.push_back(*it);
  }
  return(ret_vec); 
}


//----------------------------------------------------
// reset()

void OBLTRFilter::reset()
{
  m_xhat.zeros(m_xhat.n_cols);
  m_soft_start = true; 
  return; 
}


//----------------------------------------------------
// getSpec()

std::string OBLTRFilter::getSpec()
{
  std::string spec = ""; 

  spec += "x_hat = "  + getMatSpec(m_xhat);
  spec += ", aExt = " + getMatSpec(m_aExt);
  spec += ", bExt = " + getMatSpec(m_bExt);
  spec += ", bCmd = " + getMatSpec(m_bCmd);
  spec += ", l = "    + getMatSpec(m_l);
  return(spec); 
}
