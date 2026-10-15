#include "valid_odom_pose.hpp"

#include <cassert>
#include <limits>

int main()
{
  using zed_dual_stream::valid_odom_pose;
  assert(valid_odom_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
  assert(valid_odom_pose(1.0, -2.0, 3.0, 0.0, 0.0, 0.70710678, 0.70710678));
  assert(!valid_odom_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  assert(!valid_odom_pose(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0));
  assert(!valid_odom_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9));
  return 0;
}
