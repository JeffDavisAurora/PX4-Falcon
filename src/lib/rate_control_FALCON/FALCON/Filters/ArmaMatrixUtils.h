#ifndef ARMA_MATRIX_UTILS_HEADER
#define ARMA_MATRIX_UTILS_HEADER

#include <armadillo>
#include <string>
#include <vector>
#include <sstream>     // for getSpec

bool parseMatrixString( arma::fmat &config_matrix,
	std::string matrix_string);

arma::fvec convVec(std::vector<float> in);

std::string getMatSpec(arma::fmat); 


#endif
