#ifndef ARMA_MATRIX_UTILS_HEADER
#define ARMA_MATRIX_UTILS_HEADER

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <cstring>

template<size_t ROWS, size_t COLS>
bool parseMatrixString(matrix::Matrix<float, ROWS, COLS> &config_matrix,
	const char * matrix_string);

template<size_t ROWS, size_t COLS>
void getMatSpec(const matrix::Matrix<float, ROWS, COLS> &mat, char *out, size_t out_len);

template<size_t ROWS, size_t COLS>
bool parseMatrixString(matrix::Matrix<float, ROWS, COLS> &config_matrix,
                       const char *matrix_string)
{
    if (matrix_string == nullptr) {
        PX4_ERR("Matrix parse failed: null input");
        return false;
    }

    constexpr size_t MAX_STR = 256;  // adjust if needed
    char buf[MAX_STR];
    strncpy(buf, matrix_string, MAX_STR - 1);
    buf[MAX_STR - 1] = '\0';

    char *saveptr_row = nullptr;
    char *row_str = strtok_r(buf, ";", &saveptr_row);

    for (size_t r = 0; r < ROWS; r++) {
        if (!row_str) {
            PX4_ERR("Matrix parse failed: expected %zu rows, got %zu", ROWS, r);
            return false;
        }

        char *saveptr_cell = nullptr;
        char *cell_str = strtok_r(row_str, ",", &saveptr_cell);

        for (size_t c = 0; c < COLS; c++) {
            if (!cell_str) {
                PX4_ERR("Matrix parse failed: row %zu expects %zu columns, missing col %zu",
                        r, COLS, c);
                return false;
            }

            while (*cell_str == ' ' || *cell_str == '\t') {
                ++cell_str;
            }

            char *end_trim = cell_str + strlen(cell_str);
            while (end_trim > cell_str &&
                   (end_trim[-1] == ' ' || end_trim[-1] == '\t')) {
                --end_trim;
            }
            *end_trim = '\0';

            if (*cell_str == '\0') {
                PX4_ERR("Matrix parse failed: empty value at row %zu col %zu", r, c);
                return false;
            }

            char *endptr = nullptr;
            float v = strtof(cell_str, &endptr);

            if (endptr == cell_str || *endptr != '\0') {
                PX4_ERR("Matrix parse failed: invalid float '%s' at row %zu col %zu",
                        cell_str, r, c);
                return false;
            }
            if (!PX4_ISFINITE(v)) {
                PX4_ERR("Matrix contains NAN/Inf at row %zu col %zu", r, c);
                return false;
            }
	    PX4_INFO("Added value %f at row %zu col %zu", (double)v, r, c);
            config_matrix(r, c) = v;

            cell_str = strtok_r(nullptr, ",", &saveptr_cell);
        }

        row_str = strtok_r(nullptr, ";", &saveptr_row);
    }

    return true;
}

/*template<size_t ROWS, size_t COLS>
void getMatSpec(const matrix::Matrix<float, ROWS, COLS> &mat,
                char *out, size_t out_len)
{
    if (out_len == 0) {
        return;
    }

    if (ROWS == 0 || COLS == 0) {
        strncpy(out, "empty", out_len - 1);
        out[out_len - 1] = '\0';
        return;
    }

    char *ptr = out;
    size_t remaining = out_len;

    auto append_char = [&](char ch) {
        if (remaining <= 1) {
            return false;
        }
        *ptr++ = ch;
        --remaining;
        return true;
    };

    auto append_float = [&](float v) {
        int n = px4_snprintf(ptr, remaining, "%g", (double)v);
        if (n <= 0 || (size_t)n >= remaining) {
            remaining = 1;
            ptr[0] = '\0';
            return false;
        }
        ptr += n;
        remaining -= n;
        return true;
    };

    for (size_t r = 0; r < ROWS; r++) {
        for (size_t c = 0; c < COLS; c++) {
            if (!append_float(mat(r, c))) {
                out[out_len - 1] = '\0';
                return;
            }
            if (c + 1 < COLS) {
                if (!append_char(',')) {
                    out[out_len - 1] = '\0';
                    return;
                }
            }
        }
        if (r + 1 < ROWS) {
            if (!append_char(';')) {
                out[out_len - 1] = '\0';
                return;
            }
        }
    }

    *ptr = '\0';
}*/
#endif
