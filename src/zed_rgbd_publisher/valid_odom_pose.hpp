#pragma once

#include <cmath>

namespace zed_dual_stream
{

inline bool valid_odom_pose(
  double px, double py, double pz,
  double qx, double qy, double qz, double qw)
{
  if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
      !std::isfinite(qx) || !std::isfinite(qy) ||
      !std::isfinite(qz) || !std::isfinite(qw))
  {
    return false;
  }
  const double norm_squared = qx * qx + qy * qy + qz * qz + qw * qw;
  return std::abs(norm_squared - 1.0) <= 1.0e-3;
}

}  // namespace zed_dual_stream
