#include "ArmaMatrixUtils.h"


bool parseMatrixString( arma::fmat &config_matrix,
		std::string matrix_string)
{
  config_matrix = arma::fmat(matrix_string);
  return (true);
};


arma::fvec convVec(std::vector<float> in)
{
  arma::fvec new_vec(in.size());
  for (std::size_t i=0; i<in.size(); i++){
    new_vec(i) = in[i];
  }
  return (new_vec); 
};


std::string getMatSpec(arma::fmat mat)
{
  std::stringstream ss;
  uint cols = mat.n_cols;
  uint rows = mat.n_rows;

  if((cols == 0) || (rows == 0))
    return("empty"); 

  for (uint j=0; j<rows; j++){
    for (uint i=0; i<cols; i++){
      ss << std::to_string( mat(j,i) );
      if (i < (cols-1))
  ss << ",";
    }
    if (j < (rows-1))
      ss << ";"; 
  }
  return(ss.str()); 
};
