#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace zed_imu_conversion
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;  // Row-major.

struct Quaternion
{
  double x{};
  double y{};
  double z{};
  double w{1.0};
};

inline bool finite(const Quaternion &q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}

inline Quaternion normalized(const Quaternion &q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y +
                                q.z * q.z + q.w * q.w);
  if (!finite(q) || !(norm > 1e-12)) {
    throw std::runtime_error("invalid IMU quaternion");
  }
  return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

inline Quaternion conjugate(const Quaternion &q)
{
  return {-q.x, -q.y, -q.z, q.w};
}

inline Quaternion multiply(const Quaternion &a, const Quaternion &b)
{
  return normalized({
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z});
}

inline Matrix3 transpose(const Matrix3 &m)
{
  return {
    m[0], m[3], m[6],
    m[1], m[4], m[7],
    m[2], m[5], m[8]};
}

inline Matrix3 multiply(const Matrix3 &a, const Matrix3 &b)
{
  Matrix3 out{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int inner = 0; inner < 3; ++inner) {
        out[3 * row + column] +=
          a[3 * row + inner] * b[3 * inner + column];
      }
    }
  }
  return out;
}

inline Vector3 multiply(const Matrix3 &m, const Vector3 &v)
{
  return {
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}

inline Quaternion quaternion_from_rotation(const Matrix3 &m)
{
  Quaternion q;
  const double trace = m[0] + m[4] + m[8];
  if (trace > 0.0) {
    const double s = 2.0 * std::sqrt(trace + 1.0);
    q.w = 0.25 * s;
    q.x = (m[7] - m[5]) / s;
    q.y = (m[2] - m[6]) / s;
    q.z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const double s = 2.0 * std::sqrt(1.0 + m[0] - m[4] - m[8]);
    q.w = (m[7] - m[5]) / s;
    q.x = 0.25 * s;
    q.y = (m[1] + m[3]) / s;
    q.z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const double s = 2.0 * std::sqrt(1.0 + m[4] - m[0] - m[8]);
    q.w = (m[2] - m[6]) / s;
    q.x = (m[1] + m[3]) / s;
    q.y = 0.25 * s;
    q.z = (m[5] + m[7]) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + m[8] - m[0] - m[4]);
    q.w = (m[3] - m[1]) / s;
    q.x = (m[2] + m[6]) / s;
    q.y = (m[5] + m[7]) / s;
    q.z = 0.25 * s;
  }
  return normalized(q);
}

inline Matrix3 rotate_covariance(const Matrix3 &rotation, const Matrix3 &covariance)
{
  return multiply(multiply(rotation, covariance), transpose(rotation));
}

inline Matrix3 scale_covariance(const Matrix3 &covariance, double scale)
{
  Matrix3 out{};
  const double squared_scale = scale * scale;
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = covariance[i] * squared_scale;
  }
  return out;
}

// Specific force is a gravity-quality check, not a complete orientation.
// Reconstructing roll/pitch with yaw=0 on each sample makes small acceleration
// noise near base pitch +/-90 degrees become an arbitrary large rotation.
// RTAB-Map also differences these orientations for its motion guess, so that
// reconstruction must never replace the SDK's gyro/accelerometer fusion.
inline bool gravity_magnitude_valid(const Vector3 &acceleration_camera_m_s2)
{
  const double norm = std::sqrt(
    acceleration_camera_m_s2[0] * acceleration_camera_m_s2[0] +
    acceleration_camera_m_s2[1] * acceleration_camera_m_s2[1] +
    acceleration_camera_m_s2[2] * acceleration_camera_m_s2[2]);
  return std::isfinite(norm) && norm >= 8.0 && norm <= 12.0;
}

struct ConvertedImu
{
  Quaternion world_from_camera;
  Vector3 angular_velocity_camera_rad_s;
  Vector3 linear_acceleration_camera_m_s2;
  Matrix3 orientation_covariance_camera;
  Matrix3 angular_velocity_covariance_camera_rad2_s2;
  Matrix3 linear_acceleration_covariance_camera;
};

// The SDK documents camera_imu_rotation as the IMU-to-left-camera rotation.
// Vectors are therefore rotated by R_camera_imu. For pose orientation,
// R_zup_camera = R_zup_sdk_world * R_sdk_world_imu * R_imu_camera.
// R_zup_sdk_world maps (x, y, z) to (x, z, -y), a -90 degree X rotation.
inline ConvertedImu convert(
  const Quaternion &world_from_imu,
  const Matrix3 &camera_from_imu_rotation,
  const Vector3 &angular_velocity_imu_deg_s,
  const Vector3 &linear_acceleration_imu_m_s2,
  const Matrix3 &orientation_covariance_imu,
  const Matrix3 &angular_velocity_covariance_imu_deg2_s2,
  const Matrix3 &linear_acceleration_covariance_imu)
{
  const auto imu_from_camera_rotation = transpose(camera_from_imu_rotation);
  const auto imu_from_camera = quaternion_from_rotation(imu_from_camera_rotation);
  auto angular_velocity = multiply(camera_from_imu_rotation, angular_velocity_imu_deg_s);
  for (double &value : angular_velocity) {
    value *= kDegreesToRadians;
  }

  return {
    // SDK IMAGE world has Y down. ROS IMU orientation must use a Z-up world.
    // This is a world-basis change only: camera-frame vectors/covariances stay.
    multiply(Quaternion{-std::sqrt(0.5), 0, 0, std::sqrt(0.5)},
             multiply(normalized(world_from_imu), imu_from_camera)),
    angular_velocity,
    multiply(camera_from_imu_rotation, linear_acceleration_imu_m_s2),
    rotate_covariance(camera_from_imu_rotation, orientation_covariance_imu),
    scale_covariance(
      rotate_covariance(
        camera_from_imu_rotation,
        angular_velocity_covariance_imu_deg2_s2),
      kDegreesToRadians),
    rotate_covariance(
      camera_from_imu_rotation,
      linear_acceleration_covariance_imu)};
}

}  // namespace zed_imu_conversion
