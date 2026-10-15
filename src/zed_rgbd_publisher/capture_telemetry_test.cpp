#include "capture_telemetry.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

int main()
{
  using zed_capture_telemetry::ImageRotationWindow;
  using zed_capture_telemetry::Quaternion;
  using zed_capture_telemetry::ScalarWindow;

  const auto require = [](bool condition, const char * message) {
    if (!condition) {
      std::cerr << "FAILED: " << message << '\n';
      std::exit(1);
    }
  };

  const double half = std::sqrt(0.5);
  ImageRotationWindow rotations;
  require(rotations.add({0.0, 0.0, 0.0, 1.0}), "accept first quaternion");
  require(rotations.add({0.0, 0.0, half, half}), "accept second quaternion");
  // The quaternion sign represents the same orientation and must not create a
  // false 360 degree jump.
  require(rotations.add({0.0, 0.0, -1.0, 0.0}), "accept sign-flipped quaternion");
  auto summary = rotations.summary();
  require(summary.count == 2, "two rotation deltas");
  require(std::abs(summary.mean - 90.0) < 1.0e-9, "mean rotation is 90 degrees");
  require(std::abs(summary.maximum - 90.0) < 1.0e-9, "maximum rotation is 90 degrees");

  rotations.reset_report_window();
  require(rotations.add({0.0, 0.0, -half, half}), "preserve previous quaternion across report windows");
  summary = rotations.summary();
  require(summary.count == 1, "one post-reset delta");
  require(std::abs(summary.maximum - 90.0) < 1.0e-9, "post-reset rotation is 90 degrees");

  require(!rotations.add({0.0, 0.0, 0.0, 0.0}), "reject null quaternion");
  require(rotations.invalid_quaternion_count() == 1, "count invalid quaternion");

  ScalarWindow values;
  require(values.add(1.0), "accept scalar 1");
  require(values.add(2.0), "accept scalar 2");
  require(values.add(3.0), "accept scalar 3");
  require(values.add(4.0), "accept scalar 4");
  require(!values.add(std::numeric_limits<double>::quiet_NaN()), "reject NaN scalar");
  const auto scalar_summary = values.summary();
  require(scalar_summary.count == 4, "four scalar samples");
  require(std::abs(scalar_summary.mean - 2.5) < 1.0e-12, "scalar mean");
  require(std::abs(scalar_summary.p50 - 2.5) < 1.0e-12, "scalar p50");
  require(std::abs(scalar_summary.p95 - 3.85) < 1.0e-12, "scalar p95");
  require(scalar_summary.maximum == 4.0, "scalar maximum");
  require(values.invalid_count() == 1, "count invalid scalar");

  std::cout << "capture telemetry tests passed\n";
  return 0;
}
