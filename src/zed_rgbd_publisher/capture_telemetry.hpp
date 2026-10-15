#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

namespace zed_capture_telemetry
{
struct Quaternion
{
  double x{};
  double y{};
  double z{};
  double w{};
};

struct Summary
{
  std::size_t count{};
  double mean{};
  double p50{};
  double p95{};
  double maximum{};
};

inline double percentile(const std::vector<double> &sorted, double fraction)
{
  if (sorted.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (sorted.size() == 1) {
    return sorted.front();
  }
  const double position = static_cast<double>(sorted.size() - 1) * fraction;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double weight = position - static_cast<double>(lower);
  return sorted[lower] * (1.0 - weight) + sorted[upper] * weight;
}

class ScalarWindow
{
public:
  bool add(double value)
  {
    if (!std::isfinite(value)) {
      ++invalid_count_;
      return false;
    }
    values_.push_back(value);
    return true;
  }

  Summary summary() const
  {
    if (values_.empty()) {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      return {0, nan, nan, nan, nan};
    }
    auto sorted = values_;
    std::sort(sorted.begin(), sorted.end());
    return {
      sorted.size(),
      std::accumulate(sorted.begin(), sorted.end(), 0.0) /
      static_cast<double>(sorted.size()),
      percentile(sorted, 0.50),
      percentile(sorted, 0.95),
      sorted.back()};
  }

  std::size_t invalid_count() const {return invalid_count_;}

  void reset()
  {
    values_.clear();
    invalid_count_ = 0;
  }

private:
  std::vector<double> values_;
  std::size_t invalid_count_{};
};

inline std::optional<Quaternion> normalized(Quaternion value)
{
  const double norm_squared = value.x * value.x + value.y * value.y +
    value.z * value.z + value.w * value.w;
  if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) {
    return std::nullopt;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  value.x *= inverse_norm;
  value.y *= inverse_norm;
  value.z *= inverse_norm;
  value.w *= inverse_norm;
  return value;
}

inline double relative_angle_degrees(const Quaternion &left, const Quaternion &right)
{
  const double raw_dot = left.x * right.x + left.y * right.y +
    left.z * right.z + left.w * right.w;
  const double sign_invariant_dot = std::clamp(std::abs(raw_dot), 0.0, 1.0);
  return 2.0 * std::acos(sign_invariant_dot) * 180.0 / std::acos(-1.0);
}

class ImageRotationWindow
{
public:
  bool add(Quaternion raw)
  {
    const auto current = normalized(raw);
    if (!current) {
      ++invalid_quaternion_count_;
      return false;
    }
    if (previous_) {
      deltas_degrees_.add(relative_angle_degrees(*previous_, *current));
    }
    previous_ = current;
    return true;
  }

  Summary summary() const {return deltas_degrees_.summary();}
  std::size_t invalid_quaternion_count() const {return invalid_quaternion_count_;}

  void reset_report_window()
  {
    // Keep the previous quaternion so the first image after a report boundary
    // still contributes a true adjacent-image delta.
    deltas_degrees_.reset();
    invalid_quaternion_count_ = 0;
  }

private:
  std::optional<Quaternion> previous_;
  ScalarWindow deltas_degrees_;
  std::size_t invalid_quaternion_count_{};
};
}  // namespace zed_capture_telemetry
