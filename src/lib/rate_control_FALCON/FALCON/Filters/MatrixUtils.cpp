#include <px4_platform_common/log.h>
#include "MatrixUtils.h"

matrix::Matrix<float, 2, 1> convVec(const std::vector<float> in)
{
  matrix::Matrix<float, 2, 1> new_vec;

  new_vec(0, 0) = (in.size() > 0) ? in[0] : 0.0f;
  new_vec(1, 0) = (in.size() > 1) ? in[1] : 0.0f;

  return new_vec;
}
