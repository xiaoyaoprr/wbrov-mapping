#ifndef ZED_RAW_CALIBRATION_AUDIT_HPP_
#define ZED_RAW_CALIBRATION_AUDIT_HPP_

// Pure OpenCV-file/number checks. No ZED Camera, ROS, capture, or device access.
#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace zed_raw_calibration_audit
{
constexpr double kIntrinsicAbsoluteTolerance = 1e-3;
constexpr double kIntrinsicRelativeTolerance = 1e-6;
constexpr double kDistortionAbsoluteTolerance = 1e-6;
constexpr double kDistortionRelativeTolerance = 1e-6;

struct CameraNumbers
{
  // fx, fy, cx, cy; these must describe unrectified images, not output CameraInfo.
  std::array<double, 4> intrinsics{};
  // SDK CameraParameters::disto begins with k1, k2, p1, p2, k3.
  std::array<double, 5> distortion{};
};

struct CalibrationNumbers
{
  int width{};
  int height{};
  CameraNumbers left;
  CameraNumbers right;
};

inline cv::Mat read_matrix(const cv::FileStorage &storage, const char *name)
{
  const auto node = storage[name];
  if (node.empty()) {
    throw std::runtime_error(std::string("custom calibration is missing ") + name);
  }
  cv::Mat matrix;
  node >> matrix;
  if (matrix.empty() || matrix.dims != 2 || matrix.channels() != 1 ||
    (matrix.depth() != CV_32F && matrix.depth() != CV_64F))
  {
    throw std::runtime_error(std::string("invalid floating-point matrix ") + name);
  }
  matrix.convertTo(matrix, CV_64F);
  for (int row = 0; row < matrix.rows; ++row) {
    for (int column = 0; column < matrix.cols; ++column) {
      if (!std::isfinite(matrix.at<double>(row, column))) {
        throw std::runtime_error(std::string("non-finite value in ") + name);
      }
    }
  }
  return matrix;
}

inline CameraNumbers read_camera_numbers(
  const cv::FileStorage &storage, const char *intrinsic_name, const char *distortion_name)
{
  const auto matrix = read_matrix(storage, intrinsic_name);
  if (matrix.rows != 3 || matrix.cols != 3) {
    throw std::runtime_error(std::string(intrinsic_name) + " must be 3x3");
  }
  // CameraParameters has no skew term. Reject unsupported matrices instead of
  // silently checking only four entries of an incompatible camera model.
  if (matrix.at<double>(0, 0) <= 0.0 || matrix.at<double>(1, 1) <= 0.0 ||
    matrix.at<double>(0, 1) != 0.0 || matrix.at<double>(1, 0) != 0.0 ||
    matrix.at<double>(2, 0) != 0.0 || matrix.at<double>(2, 1) != 0.0 ||
    matrix.at<double>(2, 2) != 1.0)
  {
    throw std::runtime_error(std::string(intrinsic_name) + " is not a supported pinhole K");
  }
  const auto distortion = read_matrix(storage, distortion_name);
  if (!((distortion.rows == 1 && distortion.cols == 5) ||
    (distortion.rows == 5 && distortion.cols == 1)))
  {
    throw std::runtime_error(std::string(distortion_name) + " must contain exactly five coefficients");
  }
  CameraNumbers result;
  result.intrinsics = {
    matrix.at<double>(0, 0), matrix.at<double>(1, 1),
    matrix.at<double>(0, 2), matrix.at<double>(1, 2)};
  for (int index = 0; index < 5; ++index) {
    result.distortion[index] = distortion.rows == 1 ?
      distortion.at<double>(0, index) : distortion.at<double>(index, 0);
  }
  return result;
}

inline CalibrationNumbers read_expected(const cv::FileStorage &storage)
{
  if (!storage.isOpened()) {
    throw std::runtime_error("custom calibration file is not open");
  }
  const auto size = storage["Size"];
  if (!size.isSeq() || size.size() != 2 || !size[0].isInt() || !size[1].isInt()) {
    throw std::runtime_error("custom calibration Size must be [integer width, integer height]");
  }
  CalibrationNumbers result;
  result.width = static_cast<int>(size[0]);
  result.height = static_cast<int>(size[1]);
  if (result.width <= 0 || result.height <= 0) {
    throw std::runtime_error("custom calibration Size must be positive");
  }
  result.left = read_camera_numbers(storage, "K_LEFT", "D_LEFT");
  result.right = read_camera_numbers(storage, "K_RIGHT", "D_RIGHT");
  return result;
}

inline CalibrationNumbers load_expected(const std::string &path)
{
  try {
    const cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
      throw std::runtime_error("cannot open custom calibration file: " + path);
    }
    return read_expected(storage);
  } catch (const cv::Exception &error) {
    throw std::runtime_error("invalid custom calibration file " + path + ": " + error.what());
  }
}

inline void require_close(
  double actual, double expected, double absolute_tolerance,
  double relative_tolerance, const std::string &field)
{
  const double tolerance = absolute_tolerance + relative_tolerance * std::abs(expected);
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
    std::abs(actual - expected) > tolerance)
  {
    std::ostringstream description;
    description << std::setprecision(17) << field << " mismatch: SDK raw=" << actual <<
      ", input=" << expected << ", tolerance=" << tolerance;
    throw std::runtime_error(description.str());
  }
}

inline void verify_camera(
  const CameraNumbers &actual, const CameraNumbers &expected, const char *side)
{
  const std::array<const char *, 4> intrinsic_names = {"fx", "fy", "cx", "cy"};
  const std::array<const char *, 5> distortion_names = {"k1", "k2", "p1", "p2", "k3"};
  for (std::size_t index = 0; index < intrinsic_names.size(); ++index) {
    require_close(actual.intrinsics[index], expected.intrinsics[index],
      kIntrinsicAbsoluteTolerance, kIntrinsicRelativeTolerance,
      std::string("raw.") + side + "." + intrinsic_names[index]);
  }
  for (std::size_t index = 0; index < distortion_names.size(); ++index) {
    require_close(actual.distortion[index], expected.distortion[index],
      kDistortionAbsoluteTolerance, kDistortionRelativeTolerance,
      std::string("raw.") + side + "." + distortion_names[index]);
  }
}

inline void verify(const CalibrationNumbers &actual, const CalibrationNumbers &expected)
{
  // The candidate is native HD1200. Do not silently resize the YAML or compare
  // its raw K with the rectified/scaled CameraInfo matrix.
  if (actual.width != expected.width || actual.height != expected.height) {
    throw std::runtime_error("SDK native Size mismatch: SDK=" +
      std::to_string(actual.width) + "x" + std::to_string(actual.height) +
      ", input=" + std::to_string(expected.width) + "x" + std::to_string(expected.height));
  }
  verify_camera(actual.left, expected.left, "left");
  verify_camera(actual.right, expected.right, "right");
}
}  // namespace zed_raw_calibration_audit

#endif  // ZED_RAW_CALIBRATION_AUDIT_HPP_
