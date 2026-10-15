#include "depth_range_filter.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
int checks = 0;
void check(bool condition, const std::string &message)
{
  ++checks;
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template<class Function>
void rejects(Function function, const std::string &message)
{
  bool rejected = false;
  try {
    function();
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  check(rejected, message);
}

void set_test_env(const char *value)
{
#ifdef _WIN32
  _putenv_s("ZED_RANGE_TEST_ONLY", value == nullptr ? "" : value);
#else
  if (value == nullptr) {
    unsetenv("ZED_RANGE_TEST_ONLY");
  } else {
    setenv("ZED_RANGE_TEST_ONLY", value, 1);
  }
#endif
}
}  // namespace

int main()
{
  try {
    using zed_depth_ranges::Interval;
    using zed_depth_ranges::split;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const Interval tracking(0.30, 1.00);
    const Interval display(0.30, 0.65);
    const auto center = split(0.635f, 1.0f, tracking, display);
    check(center.tracking_z == 0.635f && center.display_z == 0.635f, "center retained");
    const auto edge = split(0.635f, 1.2f, tracking, display);
    check(edge.tracking_z == 0.635f && std::isnan(edge.display_z), "display cannot crop tracking");
    check(split(1.0f, 1.0f, tracking, display).tracking_z == 1.0f, "upper tracking boundary");
    check(std::isnan(split(1.001f, 1.0f, tracking, display).tracking_z),
      "above tracking maximum");
    const Interval exact(0.25, 0.50);
    check(exact.contains(0.25) && exact.contains(0.50), "interval boundaries inclusive");
    check(!exact.contains(std::nextafter(0.25, 0.0)), "below boundary");
    check(!exact.contains(std::nextafter(0.50, 1.0)), "above boundary");

    for (float invalid_z : {nan, inf, -inf, -0.3f, 0.0f, 0.29f, 1.01f}) {
      const auto sample = split(invalid_z, 1.0f, tracking, display);
      check(std::isnan(sample.tracking_z) && std::isnan(sample.display_z),
        "invalid depth rejected");
    }
    for (float invalid_ray : {nan, inf, -inf, -1.0f, 0.0f, 0.99f}) {
      const auto sample = split(0.5f, invalid_ray, tracking, display);
      check(sample.tracking_z == 0.5f && std::isnan(sample.display_z),
        "invalid display ray never changes tracking");
    }
    const auto unavailable = split(0.25f, 2.0f, tracking, Interval(0.2, 0.9));
    check(std::isnan(unavailable.tracking_z) && std::isnan(unavailable.display_z),
      "display is tracking subset");

    for (auto bounds : std::array<std::array<double, 2>, 10>{{
      {0.0, 1.0}, {-1.0, 1.0}, {1.0, 1.0}, {2.0, 1.0}, {nan, 1.0}, {0.3, inf},
      {0.3, nan}, {0.3, 1e39}, {1e-50, 1.0}, {1.0, 1.0 + 1e-10}}})
    {
      rejects([&]() {Interval range(bounds[0], bounds[1]);}, "invalid interval rejected");
    }

    set_test_env(nullptr);
    check(zed_depth_ranges::env_number("ZED_RANGE_TEST_ONLY", 0.75) == 0.75,
      "missing env defaults");
    for (const char *value : {"0.75", ".75", "+.75", "7.5e-1", "75E-2"}) {
      set_test_env(value);
      check(zed_depth_ranges::env_number("ZED_RANGE_TEST_ONLY", 0.0) == 0.75,
        "valid decimal env");
    }
    for (const char *value : {"nan", "NaN", "inf", "Infinity", "1e309", "1e-999",
      "1.0junk", " 1", "1 ", "0x1p0"})
    {
      set_test_env(value);
      rejects([]() {zed_depth_ranges::env_number("ZED_RANGE_TEST_ONLY", 1.0);},
        "invalid env rejected");
    }
#ifndef _WIN32
    set_test_env("");
    rejects([]() {zed_depth_ranges::env_number("ZED_RANGE_TEST_ONLY", 1.0);},
      "empty env rejected");
#endif
    set_test_env(nullptr);
    std::cout << "PASS checks=" << checks << " no_hardware=true\n";
  } catch (const std::exception &error) {
    set_test_env(nullptr);
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
  return 0;
}
