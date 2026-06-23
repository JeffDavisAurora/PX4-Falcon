#ifndef ARMA_MATRIX_UTILS_HEADER
#define ARMA_MATRIX_UTILS_HEADER

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>

template<size_t ROWS, size_t COLS>
bool parseMatrixString(matrix::Matrix<float, ROWS, COLS> &config_matrix,
	const std::string matrix_string);

matrix::Matrix<float, 2, 1>convVec(const std::vector<float> in);

template<size_t ROWS, size_t COLS>
std::string getMatSpec(const matrix::Matrix<float, ROWS, COLS> &mat);

template<size_t ROWS, size_t COLS>
bool parseMatrixString(matrix::Matrix<float, ROWS, COLS> &config_matrix,
	const std::string matrix_string)
{
	std::istringstream ss(matrix_string);
	std::string row_str;

	for(size_t r = 0; r < ROWS; r++) {
		if(!std::getline(ss, row_str, ';')) {
			PX4_ERR("Matrix parse failed: expected %zu rows, got %zu", ROWS, r);
			return false;
		}

		std::istringstream row_ss(row_str);
		std::string cell_str;

		for(size_t c = 0; c < COLS; c++){
			if(!std::getline(row_ss, cell_str, ',')) {
				PX4_ERR("Matrix parse failed: row %zu expects %zu columns," 
						"missing col %zu", r, COLS, c);
				return false;
			}

			auto start = cell_str.find_first_not_of("\t");
			auto end = cell_str.find_last_not_of("\t");
			if(start == std::string::npos) {
				PX4_ERR("Matrix parse failed: empty value at row %zu col %zu",
						r, c);
				return false;
			}

			cell_str = cell_str.substr(start, end - start + 1);

			char *endptr = nullptr;
			float v = strtof(cell_str.c_str(), &endptr);
			if(endptr == cell_str.c_str() || *endptr != '\0') {
				PX4_ERR("Matrix parse failed: invalid float '%s' at row %zu"
						"col %zu", cell_str.c_str(), r, c);
				return false;
			}

			if(!PX4_ISFINITE(v)) {
				PX4_ERR("Matrix contains NAN/Inf at row %zu col %zu", r, c);
				return false;
			}

			config_matrix(r, c) = v;
		}
	}
	return true;
}

template<size_t ROWS, size_t COLS>
std::string getMatSpec(const matrix::Matrix<float, ROWS, COLS> &mat) {
	
	std::stringstream ss;

	if((COLS == 0) || (ROWS == 0)) {
		return "empty";
	}

	for(size_t r = 0; r < ROWS; r++) {
		for(size_t c = 0; c < COLS; c++) {
			ss << std::to_string(mat(r,c));

			if(c < (COLS - 1)) {
				ss << ",";
			}

		}

		if(r < (ROWS - 1)) { 
			ss << ";";
		}
	}
	return ss.str();
}

#endif
