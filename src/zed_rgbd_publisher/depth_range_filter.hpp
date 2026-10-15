#ifndef ZED_DEPTH_RANGE_FILTER_HPP
#define ZED_DEPTH_RANGE_FILTER_HPP

// Pure numeric helpers: no ROS, ZED, device access, or process creation.
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>

namespace zed_depth_ranges
{
inline double env_number(const char *name, double fallback)
{
  const char *text = std::getenv(name);
  if (text == nullptr) {
    return fallback;
  }
  static const std::regex decimal(
    R"([+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?)");
  if (!std::regex_match(text, decimal)) {
    throw std::runtime_error(std::string(name) + " must be a finite decimal number");
  }
  char *end = nullptr;
  errno = 0;
  const double value = std::strtod(text, &end);
  if (errno == ERANGE || end == text || *end != '\0' || !std::isfinite(value)) {
    throw std::runtime_error(std::string(name) + " must be a finite decimal number");
  }
  return value;
}

struct Interval
{
  double minimum;
  double maximum;

  Interval(double lower, double upper) : minimum(lower), maximum(upper)
  {
    if (!std::isfinite(lower) || !std::isfinite(upper) ||
      !(0.0 < lower && lower < upper) ||
      upper > std::numeric_limits<float>::max())
    {
      throw std::runtime_error("Depth/range interval must satisfy finite 0 < min < max");
    }
    const float lower_float = static_cast<float>(lower);
    const float upper_float = static_cast<float>(upper);
    if (!(lower_float > 0.0f && lower_float < upper_float)) {
      throw std::runtime_error("Depth/range interval collapses in float32");
    }
  }

  bool contains(double value) const
  {
    return std::isfinite(value) && value >= minimum && value <= maximum;
  }
};

struct Sample
{
  float tracking_z;
  float display_z;
};

inline Sample split(
  float z, float ray_scale, const Interval &tracking, const Interval &display)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  // Registered depth carries optical-axis Z. A display cutoff must never
  // remove usable depth from visual odometry or the saved RGB-D database.
  if (!tracking.contains(z)) {
    return {nan, nan};
  }
  const double radial = static_cast<double>(z) * ray_scale;
  const bool display_valid = std::isfinite(ray_scale) && ray_scale >= 1.0f &&
    display.contains(radial);
  return {z, display_valid ? z : nan};
}
}  // namespace zed_depth_ranges

#endif  // ZED_DEPTH_RANGE_FILTER_HPP
