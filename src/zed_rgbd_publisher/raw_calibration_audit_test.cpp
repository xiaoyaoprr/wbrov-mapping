#include "raw_calibration_audit.hpp"

#include <functional>
#include <iostream>
#include <limits>

namespace audit = zed_raw_calibration_audit;

namespace
{
int checks = 0;

void require(bool condition, const std::string &name)
{
  if (!condition) {
    throw std::runtime_error("FAIL: " + name);
  }
  ++checks;
}

void rejects(const std::function<void()> &action, const std::string &name)
{
  bool rejected = false;
  try {
    action();
  } catch (const std::exception &) {
    rejected = true;
  }
  require(rejected, name);
}

std::string fixture(const std::string &fault = "")
{
  cv::FileStorage output("", cv::FileStorage::WRITE |
    cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
  if (fault == "size_string") {
    output << "Size" << "1920x1200";
  } else if (fault == "size_real") {
    output << "Size" << "[" << 1920.0 << 1200 << "]";
  } else if (fault == "size_short") {
    output << "Size" << "[" << 1920 << "]";
  } else if (fault != "size_missing") {
    output << "Size" << "[" << (fault == "size_zero" ? 0 : 1920) << 1200 << "]";
  }
  cv::Mat left = (cv::Mat_<double>(3, 3) <<
    986.2275062742521, 0, 997.5283374438162,
    0, 986.6390603935716, 614.1119377202925,
    0, 0, 1);
  const cv::Mat right = (cv::Mat_<double>(3, 3) <<
    989.4567751222353, 0, 941.4614543244986,
    0, 989.57948337683, 632.4112437670922,
    0, 0, 1);
  cv::Mat distortion = (cv::Mat_<double>(1, 5) <<
    0.3724558451676341, 0.09123592818856389, 0.001754598242476779,
    -4.066398407105886e-05, -0.0825864575048849);
  if (fault == "K_shape") {
    left = cv::Mat::zeros(2, 3, CV_64F);
  } else if (fault == "K_skew") {
    left.at<double>(0, 1) = 1.0;
  } else if (fault == "K_normalization") {
    left.at<double>(2, 2) = 2.0;
  } else if (fault == "K_negative") {
    left.at<double>(0, 0) = -1.0;
  } else if (fault == "K_nan") {
    left.at<double>(0, 0) = std::numeric_limits<double>::quiet_NaN();
  } else if (fault == "K_integer") {
    left.convertTo(left, CV_32S);
  }
  if (fault == "D_shape") {
    distortion = cv::Mat::zeros(1, 8, CV_64F);
  } else if (fault == "D_column") {
    distortion = distortion.t();
  } else if (fault == "D_infinite") {
    distortion.at<double>(0, 0) = std::numeric_limits<double>::infinity();
  }
  output << "K_LEFT" << left;
  if (fault != "K_missing") {
    output << "K_RIGHT" << right;
  }
  output << "D_LEFT" << distortion;
  if (fault != "D_missing") {
    output << "D_RIGHT" << distortion;
  }
  return output.releaseAndGetString();
}

audit::CalibrationNumbers parse_fixture(const std::string &fault = "")
{
  const cv::FileStorage storage(
    fixture(fault), cv::FileStorage::READ | cv::FileStorage::MEMORY);
  return audit::read_expected(storage);
}
}  // namespace

int main(int argc, char **argv)
{
  try {
    const auto expected = parse_fixture();
    audit::verify(expected, expected);
    require(true, "exact raw values accepted");

    auto actual = expected;
    for (auto *camera : {&actual.left, &actual.right}) {
      for (auto &value : camera->intrinsics) {
        value = static_cast<float>(value);
      }
      for (auto &value : camera->distortion) {
        value = static_cast<float>(value);
      }
    }
    audit::verify(actual, expected);
    require(true, "SDK float rounding accepted");
    audit::verify(parse_fixture("D_column"), expected);
    require(true, "OpenCV five-coefficient column vectors accepted");

    for (const std::string side : {"left", "right"}) {
      for (std::size_t index = 0; index < 4; ++index) {
        actual = expected;
        auto &camera = side == "left" ? actual.left : actual.right;
        camera.intrinsics[index] += 0.1;
        rejects([&] {audit::verify(actual, expected);}, side + " K mismatch");
      }
      for (std::size_t index = 0; index < 5; ++index) {
        actual = expected;
        auto &camera = side == "left" ? actual.left : actual.right;
        camera.distortion[index] += 0.01;
        rejects([&] {audit::verify(actual, expected);}, side + " D mismatch");
      }
    }

    actual = expected;
    actual.width /= 3;
    rejects([&] {audit::verify(actual, expected);}, "output width cannot replace native Size");
    actual = expected;
    actual.height /= 3;
    rejects([&] {audit::verify(actual, expected);}, "output height cannot replace native Size");
    actual = expected;
    actual.left.intrinsics[0] /= 3.0;
    rejects([&] {audit::verify(actual, expected);}, "scaled output K rejected as raw K");
    actual = expected;
    actual.left.intrinsics[0] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] {audit::verify(actual, expected);}, "SDK raw NaN rejected");
    actual = expected;
    actual.right.distortion[4] = std::numeric_limits<double>::infinity();
    rejects([&] {audit::verify(actual, expected);}, "SDK raw infinity rejected");

    const double intrinsic = expected.left.intrinsics[0];
    const double tolerance = audit::kIntrinsicAbsoluteTolerance +
      audit::kIntrinsicRelativeTolerance * std::abs(intrinsic);
    audit::require_close(intrinsic + tolerance * 0.99, intrinsic,
      audit::kIntrinsicAbsoluteTolerance, audit::kIntrinsicRelativeTolerance, "tolerance_inside");
    require(true, "intrinsic tolerance inside accepted");
    rejects([&] {audit::require_close(intrinsic + tolerance * 1.01, intrinsic,
      audit::kIntrinsicAbsoluteTolerance, audit::kIntrinsicRelativeTolerance, "tolerance_outside");},
      "intrinsic tolerance outside rejected");

    for (const std::string fault : {"size_string", "size_real", "size_short", "size_missing",
      "size_zero", "K_shape", "K_skew", "K_normalization", "K_negative", "K_nan",
      "K_integer", "K_missing", "D_shape", "D_infinite", "D_missing"})
    {
      rejects([&] {parse_fixture(fault);}, "invalid YAML " + fault);
    }
    rejects([] {audit::load_expected("/no-such-raw-calibration-audit-input.yml");},
      "missing input rejected");
    const cv::FileStorage unopened;
    rejects([&] {audit::read_expected(unopened);}, "unopened storage rejected");
    if (argc == 2) {
      const auto handoff = audit::load_expected(argv[1]);
      require(handoff.width == 1920 && handoff.height == 1200, "handoff native Size");
      audit::require_close(handoff.left.intrinsics[0], 986.2275062742521,
        1e-10, 0.0, "handoff final86 left fx");
      require(true, "handoff final86 YAML read without camera");
    } else if (argc > 2) {
      throw std::runtime_error("usage: raw_calibration_audit_test [handoff_sdk_yaml]");
    }
    std::cout << "PASS: " << checks <<
      " raw calibration file/numeric checks; no ZED SDK, ROS, camera, or network used." << std::endl;
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
  return 0;
}
