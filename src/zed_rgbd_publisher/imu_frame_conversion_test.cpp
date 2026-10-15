#include "imu_frame_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

constexpr double kTolerance = 1e-10;

void near(double actual, double expected)
{
  if (std::abs(actual - expected) >= kTolerance) {
    throw std::runtime_error("IMU conversion numeric mismatch");
  }
}

void same_rotation(
  const zed_imu_conversion::Quaternion &actual,
  const zed_imu_conversion::Quaternion &expected)
{
  const double dot = std::abs(
    actual.x * expected.x + actual.y * expected.y +
    actual.z * expected.z + actual.w * expected.w);
  near(dot, 1.0);
}

}  // namespace

int main()
{
  using namespace zed_imu_conversion;
  const Matrix3 identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
  const Matrix3 diagonal{1, 0, 0, 0, 4, 0, 0, 0, 9};

  const auto identity_result = convert(
    {0, 0, 0, 1}, identity, {180, -90, 45}, {1, 2, 3},
    diagonal, diagonal, diagonal);
  same_rotation(identity_result.world_from_camera, {-std::sqrt(0.5), 0, 0, std::sqrt(0.5)});
  near(identity_result.angular_velocity_camera_rad_s[0], 3.14159265358979323846);
  near(identity_result.angular_velocity_camera_rad_s[1], -1.57079632679489661923);
  near(identity_result.angular_velocity_camera_rad_s[2], 0.78539816339744830962);
  near(identity_result.linear_acceleration_camera_m_s2[0], 1);
  near(identity_result.angular_velocity_covariance_camera_rad2_s2[4],
       4 * kDegreesToRadians * kDegreesToRadians);

  // R_camera_imu is +90 degrees about Z: IMU +X becomes camera +Y.
  const Matrix3 camera_from_imu{0, -1, 0, 1, 0, 0, 0, 0, 1};
  const auto rotated = convert(
    {0, 0, 0, 1}, camera_from_imu, {1, 0, 0}, {1, 0, 0},
    diagonal, diagonal, diagonal);
  near(rotated.angular_velocity_camera_rad_s[0], 0);
  near(rotated.angular_velocity_camera_rad_s[1], kDegreesToRadians);
  near(rotated.linear_acceleration_camera_m_s2[0], 0);
  near(rotated.linear_acceleration_camera_m_s2[1], 1);
  near(rotated.orientation_covariance_camera[0], 4);
  near(rotated.orientation_covariance_camera[4], 1);
  near(rotated.orientation_covariance_camera[8], 9);
  same_rotation(
    rotated.world_from_camera,
    {-0.5, -0.5, -0.5, 0.5});

  // Physical fixtures: optical camera forward, down, up, and rolled.
  // Recorded SDK IMAGE-world specific force is -Y. After conversion it is +Z.
  const double s = std::sqrt(0.5);
  const std::array<Quaternion, 4> orientations{{{0,0,0,1}, {-s,0,0,s}, {s,0,0,s}, {0,0,s,s}}};
  const std::array<Vector3, 4> accelerations{{{0,-9.81,0}, {0,0,-9.81}, {0,0,9.81}, {-9.81,0,0}}};
  for (std::size_t i=0; i<orientations.size(); ++i) {
    const auto c=convert(orientations[i],identity,{0,0,0},accelerations[i],diagonal,diagonal,diagonal);
    const auto &q=c.world_from_camera;
    const Matrix3 R{1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.z*q.w),2*(q.x*q.z+q.y*q.w),
                    2*(q.x*q.y+q.z*q.w),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.x*q.w),
                    2*(q.x*q.z-q.y*q.w),2*(q.y*q.z+q.x*q.w),1-2*(q.x*q.x+q.y*q.y)};
    const auto world_acc=multiply(R,c.linear_acceleration_camera_m_s2);
    near(world_acc[0],0); near(world_acc[1],0); near(world_acc[2],9.81);
    for (std::size_t k=0;k<3;++k) near(c.linear_acceleration_camera_m_s2[k],accelerations[i][k]);
  }

  // The production gravity orientation derives roll/pitch from acceleration,
  // independent of the SDK visual/world orientation.
  const std::array<Vector3, 5> gravity_samples{{
    {0,-9.81,0}, {0,0,-9.81}, {0,0,9.81}, {-9.81,0,0}, {1.2,-9.6,0.8}}};
  const Matrix3 base_from_camera{0,0,1,-1,0,0,0,-1,0};
  const Matrix3 memory_yaw{0,1,0,-1,0,0,0,0,1};
  for (const auto &sample : gravity_samples) {
    Quaternion message;
    if (!acceleration_gravity_world_from_camera(sample, message)) {
      throw std::runtime_error("valid acceleration gravity sample rejected");
    }
    const auto &q=message;
    const Matrix3 message_rotation{
      1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.z*q.w),2*(q.x*q.z+q.y*q.w),
      2*(q.x*q.y+q.z*q.w),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.x*q.w),
      2*(q.x*q.z-q.y*q.w),2*(q.y*q.z+q.x*q.w),1-2*(q.x*q.x+q.y*q.y)};
    const auto world_from_base=multiply(
      multiply(memory_yaw,message_rotation),transpose(base_from_camera));
    auto base_acceleration=multiply(base_from_camera,sample);
    const double n=std::sqrt(base_acceleration[0]*base_acceleration[0]+
                             base_acceleration[1]*base_acceleration[1]+
                             base_acceleration[2]*base_acceleration[2]);
    for(double &v:base_acceleration) v/=n;
    const auto mapped=multiply(world_from_base,base_acceleration);
    near(mapped[0],0); near(mapped[1],0); near(mapped[2],1);
  }
  Quaternion invalid_message;
  if (acceleration_gravity_world_from_camera({0,0,0},invalid_message) ||
      acceleration_gravity_world_from_camera({0,0,13},invalid_message)) {
    throw std::runtime_error("unsafe acceleration magnitude was accepted");
  }

  bool rejected = false;
  try {
    (void)normalized({0, 0, 0, 0});
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  if (!rejected) {
    throw std::runtime_error("zero quaternion was not rejected");
  }

  std::cout << "imu_frame_conversion_test: PASS\n";
  return 0;
}
