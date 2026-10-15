#include <sl/Camera.hpp>

#include "depth_range_filter.hpp"
#include "capture_telemetry.hpp"
#include "dual_stream_schedule.hpp"
#include "imu_frame_conversion.hpp"
#include "map_frame_lifecycle.hpp"
#include "raw_calibration_audit.hpp"
#include "valid_odom_pose.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace
{
constexpr char kOdomRgbTopic[] = "/zed/rgb/image_rect_color";
constexpr char kOdomDepthTopic[] = "/zed/depth/depth_registered";
constexpr char kOdomDisplayDepthTopic[] = "/zed/depth/depth_display";
constexpr char kOdomInfoTopic[] = "/zed/rgb/camera_info";
constexpr char kOdomRgbdTopic[] = "/zed/rgbd_image";
constexpr char kMapRgbTopic[] = "/zed/mapping/rgb/image_rect_color";
constexpr char kMapDepthTopic[] = "/zed/mapping/depth/depth_registered";
constexpr char kMapInfoTopic[] = "/zed/mapping/rgb/camera_info";
constexpr char kImuTopic[] = "/zed/imu/data";
constexpr char kOdomTopic[] = "/odom";
constexpr char kOpticalFrame[] = "zed_left_camera_optical_frame";
constexpr std::int64_t kMapOdomAckTimeoutNs = 750000000;

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

int env_int(const char *name, int fallback)
{
  const char *value = std::getenv(name);
  if (!value || !*value) {
    return fallback;
  }
  try {
    std::size_t parsed = 0;
    const int result = std::stoi(value, &parsed);
    if (parsed != std::strlen(value)) {
      throw std::invalid_argument("trailing characters");
    }
    return result;
  } catch (const std::exception &) {
    throw std::runtime_error(std::string(name) + " must be an integer");
  }
}

double env_double(const char *name, double fallback)
{
  const char *value = std::getenv(name);
  if (!value || !*value) {
    return fallback;
  }
  try {
    std::size_t parsed = 0;
    const double result = std::stod(value, &parsed);
    if (parsed != std::strlen(value) || !std::isfinite(result)) {
      throw std::invalid_argument("invalid number");
    }
    return result;
  } catch (const std::exception &) {
    throw std::runtime_error(std::string(name) + " must be a finite number");
  }
}

std::string env_string(const char *name, const char *fallback)
{
  const char *value = std::getenv(name);
  return value && *value ? value : fallback;
}

sl::DEPTH_MODE parse_depth_mode(std::string name)
{
  std::transform(name.begin(), name.end(), name.begin(),
    [](unsigned char value) {return static_cast<char>(std::toupper(value));});
  if (name == "NEURAL_LIGHT") {
    return sl::DEPTH_MODE::NEURAL_LIGHT;
  }
  if (name == "NEURAL") {
    return sl::DEPTH_MODE::NEURAL;
  }
  if (name == "PERFORMANCE") {
    return sl::DEPTH_MODE::PERFORMANCE;
  }
  throw std::runtime_error(
          "RTAB_DEPTH_MODE only supports NEURAL_LIGHT, NEURAL or PERFORMANCE");
}

void log_camera_parameters(
  const rclcpp::Logger &logger, const char *space, const char *side,
  const sl::CameraParameters &parameters)
{
  RCLCPP_INFO(logger,
    "[CALIB_AUDIT] %s.%s image_size=%zux%zu K_px=[fx=%.9g,fy=%.9g,cx=%.9g,cy=%.9g]",
    space, side, static_cast<std::size_t>(parameters.image_size.width),
    static_cast<std::size_t>(parameters.image_size.height),
    parameters.fx, parameters.fy, parameters.cx, parameters.cy);
  const auto &d = parameters.disto;
  RCLCPP_INFO(logger,
    "[CALIB_AUDIT] %s.%s D_first5=[%.17g,%.17g,%.17g,%.17g,%.17g]",
    space, side, d[0], d[1], d[2], d[3], d[4]);
}

void log_stereo_parameters(
  const rclcpp::Logger &logger, const char *space,
  const sl::CalibrationParameters &parameters)
{
  log_camera_parameters(logger, space, "left", parameters.left_cam);
  log_camera_parameters(logger, space, "right", parameters.right_cam);
  const auto translation = parameters.stereo_transform.getTranslation();
  const auto rotation_vector = parameters.stereo_transform.getRotationVector();
  RCLCPP_INFO(logger,
    "[CALIB_AUDIT] %s.stereo T_m=[%.9g,%.9g,%.9g] Rodrigues_rad=[%.9g,%.9g,%.9g] baseline_m=%.9g",
    space, translation.x, translation.y, translation.z,
    rotation_vector.x, rotation_vector.y, rotation_vector.z,
    parameters.getCameraBaseline());
}

zed_raw_calibration_audit::CameraNumbers raw_camera_numbers(
  const sl::CameraParameters &parameters)
{
  zed_raw_calibration_audit::CameraNumbers result;
  result.intrinsics = {parameters.fx, parameters.fy, parameters.cx, parameters.cy};
  std::copy_n(parameters.disto, result.distortion.size(), result.distortion.begin());
  return result;
}

void audit_open_calibration(
  const rclcpp::Logger &logger, const sl::CameraInformation &information,
  const std::string &calibration_file)
{
  const auto &configuration = information.camera_configuration;
  RCLCPP_INFO(logger,
    "[CALIB_AUDIT] version=2 SDK=%s serial=%u model=%s native_size=%zux%zu native_fps=%.3f "
    "self_calibration=disabled source=%s",
    sl::Camera::getSDKVersion().c_str(), information.serial_number,
    sl::toString(information.camera_model).c_str(),
    static_cast<std::size_t>(configuration.resolution.width),
    static_cast<std::size_t>(configuration.resolution.height),
    configuration.fps,
    calibration_file.empty() ? "factory" : calibration_file.c_str());
  log_stereo_parameters(logger, "RAW_UNRECTIFIED", configuration.calibration_parameters_raw);
  log_stereo_parameters(logger, "RECTIFIED_NATIVE", configuration.calibration_parameters);
  if (calibration_file.empty()) {
    RCLCPP_INFO(logger, "[CALIB_AUDIT] custom_raw_gate=SKIPPED reason=factory_profile");
    return;
  }
  const auto expected = zed_raw_calibration_audit::load_expected(calibration_file);
  zed_raw_calibration_audit::CalibrationNumbers actual;
  actual.width = static_cast<int>(configuration.resolution.width);
  actual.height = static_cast<int>(configuration.resolution.height);
  actual.left = raw_camera_numbers(configuration.calibration_parameters_raw.left_cam);
  actual.right = raw_camera_numbers(configuration.calibration_parameters_raw.right_cam);
  zed_raw_calibration_audit::verify(actual, expected);
  RCLCPP_INFO(logger,
    "[CALIB_AUDIT] custom_raw_gate=PASS verified=native_Size,left/right_raw_K,left/right_D_first5");
}

zed_imu_conversion::Vector3 vector_from_sdk(const sl::float3 &value)
{
  return {value.x, value.y, value.z};
}

zed_imu_conversion::Matrix3 matrix_from_sdk(const sl::Matrix3f &value)
{
  zed_imu_conversion::Matrix3 result{};
  std::copy_n(value.r, result.size(), result.begin());
  return result;
}

struct Geometry
{
  int width{};
  int height{};
  double fx{};
  double fy{};
  double cx{};
  double cy{};
  sl::Resolution resolution;
  std::vector<float> ray_scale;
  sensor_msgs::msg::CameraInfo info;
};

Geometry make_geometry(
  int width, int height, const sl::CameraParameters &calibration)
{
  if (width <= 0 || height <= 0 ||
      static_cast<int>(calibration.image_size.width) != width ||
      static_cast<int>(calibration.image_size.height) != height)
  {
    throw std::runtime_error(
            "SDK scaled calibration image_size does not match requested stream geometry");
  }
  Geometry result;
  result.width = width;
  result.height = height;
  result.resolution = sl::Resolution(width, height);
  result.fx = calibration.fx;
  result.fy = calibration.fy;
  result.cx = calibration.cx;
  result.cy = calibration.cy;
  result.ray_scale.resize(static_cast<std::size_t>(width) * height);
  for (int y = 0; y < height; ++y) {
    const double v = (y - result.cy) / result.fy;
    for (int x = 0; x < width; ++x) {
      const double u = (x - result.cx) / result.fx;
      result.ray_scale[static_cast<std::size_t>(y) * width + x] =
        static_cast<float>(std::sqrt(1.0 + u * u + v * v));
    }
  }
  result.info.header.frame_id = kOpticalFrame;
  result.info.width = width;
  result.info.height = height;
  result.info.distortion_model = "plumb_bob";
  result.info.d.assign(5, 0.0);
  result.info.k = {
    result.fx, 0.0, result.cx,
    0.0, result.fy, result.cy,
    0.0, 0.0, 1.0};
  result.info.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  result.info.p = {
    result.fx, 0.0, result.cx, 0.0,
    0.0, result.fy, result.cy, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return result;
}

std::unique_ptr<sensor_msgs::msg::Image> color_message(
  const sl::Mat &source, const Geometry &geometry, const rclcpp::Time &stamp)
{
  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header.stamp = stamp;
  message->header.frame_id = kOpticalFrame;
  message->height = geometry.height;
  message->width = geometry.width;
  message->encoding = "bgr8";
  message->is_bigendian = false;
  message->step = geometry.width * 3;
  message->data.resize(static_cast<std::size_t>(message->step) * geometry.height);
  const auto *bgra = source.getPtr<sl::uchar1>(sl::MEM::CPU);
  const std::size_t source_step = source.getStepBytes(sl::MEM::CPU);
  for (int y = 0; y < geometry.height; ++y) {
    const auto *row = bgra + static_cast<std::size_t>(y) * source_step;
    auto *target = message->data.data() + static_cast<std::size_t>(y) * message->step;
    for (int x = 0; x < geometry.width; ++x) {
      target[3 * x] = row[4 * x];
      target[3 * x + 1] = row[4 * x + 1];
      target[3 * x + 2] = row[4 * x + 2];
    }
  }
  return message;
}

struct DepthMessages
{
  std::unique_ptr<sensor_msgs::msg::Image> tracking;
  std::unique_ptr<sensor_msgs::msg::Image> display;
  double tracking_valid_ratio{};
  double display_valid_ratio{};
};

DepthMessages depth_messages(
  const sl::Mat &source, const Geometry &geometry, const rclcpp::Time &stamp,
  const zed_depth_ranges::Interval &tracking_range,
  const zed_depth_ranges::Interval &display_range, bool include_display)
{
  DepthMessages result;
  result.tracking = std::make_unique<sensor_msgs::msg::Image>();
  if (include_display) {
    result.display = std::make_unique<sensor_msgs::msg::Image>();
  }
  const auto initialize = [&](sensor_msgs::msg::Image *message) {
    message->header.stamp = stamp;
    message->header.frame_id = kOpticalFrame;
    message->height = geometry.height;
    message->width = geometry.width;
    message->encoding = "32FC1";
    message->is_bigendian = false;
    message->step = geometry.width * sizeof(float);
    message->data.resize(static_cast<std::size_t>(message->step) * geometry.height);
  };
  initialize(result.tracking.get());
  if (result.display) {
    initialize(result.display.get());
  }
  const auto *depth = source.getPtr<float>(sl::MEM::CPU);
  const std::size_t source_step = source.getStepBytes(sl::MEM::CPU) / sizeof(float);
  std::size_t tracking_valid = 0;
  std::size_t display_valid = 0;
  const std::size_t count = static_cast<std::size_t>(geometry.width) * geometry.height;
  std::vector<float> tracking(count);
  std::vector<float> display(include_display ? count : 0);
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * geometry.width + x;
      const auto sample = zed_depth_ranges::split(
        depth[static_cast<std::size_t>(y) * source_step + x],
        geometry.ray_scale[index], tracking_range, display_range);
      tracking[index] = sample.tracking_z;
      if (include_display) {
        display[index] = sample.display_z;
      }
      tracking_valid += std::isfinite(sample.tracking_z) ? 1u : 0u;
      display_valid += include_display && std::isfinite(sample.display_z) ? 1u : 0u;
    }
  }
  std::memcpy(result.tracking->data.data(), tracking.data(), count * sizeof(float));
  if (result.display) {
    std::memcpy(result.display->data.data(), display.data(), count * sizeof(float));
  }
  result.tracking_valid_ratio = static_cast<double>(tracking_valid) / count;
  result.display_valid_ratio = include_display ?
    static_cast<double>(display_valid) / count : 0.0;
  return result;
}
}  // namespace

class ZedDualStreamPublisher : public rclcpp::Node
{
public:
  ZedDualStreamPublisher()
  : Node("zed_rgbd_dual_stream_publisher"),
    tracking_range_(
      zed_depth_ranges::env_number("RTAB_TRACKING_MIN_M", 0.30),
      zed_depth_ranges::env_number("RTAB_TRACKING_MAX_M", 1.00)),
    display_range_(
      zed_depth_ranges::env_number("DENSE_MIN_M", 0.30),
      zed_depth_ranges::env_number("DENSE_MAX_M", 0.65)),
    grab_resolution_name_(env_string("ZED_GRAB_RESOLUTION", "HD1200")),
    grab_fps_(env_int("ZED_GRAB_FPS", 60)),
    odom_width_(env_int("ZED_ODOM_WIDTH", 384)),
    odom_height_(env_int("ZED_ODOM_HEIGHT", 240)),
    map_width_(env_int("ZED_MAP_WIDTH", 960)),
    map_height_(env_int("ZED_MAP_HEIGHT", 600)),
    odom_publish_hz_(env_double("ZED_ODOM_FPS", 20.0)),
    map_publish_hz_(env_double("ZED_MAP_FPS", 1.0)),
    compute_cap_hz_(env_double("ZED_COMPUTE_CAP_FPS", 30.0)),
    schedule_(odom_publish_hz_, map_publish_hz_),
    depth_mode_name_(env_string("RTAB_DEPTH_MODE", "NEURAL")),
    calibration_file_(env_string("ZED_OPENCV_CALIBRATION_FILE", ""))
  {
    if (grab_resolution_name_ != "HD1200") {
      throw std::runtime_error("ZED_GRAB_RESOLUTION must be exactly HD1200");
    }
    if (grab_fps_ != 15 && grab_fps_ != 30 && grab_fps_ != 60) {
      throw std::runtime_error("HD1200 ZED_GRAB_FPS must be 15, 30 or 60");
    }
    if (odom_publish_hz_ >= grab_fps_) {
      throw std::runtime_error(
              "ZED_ODOM_FPS must be strictly lower than ZED_GRAB_FPS for SDK compute capping");
    }
    if (compute_cap_hz_ < odom_publish_hz_ || compute_cap_hz_ > grab_fps_) {
      throw std::runtime_error(
              "ZED_COMPUTE_CAP_FPS must be between ZED_ODOM_FPS and ZED_GRAB_FPS");
    }
    if (tracking_range_.minimum > 3.0) {
      throw std::runtime_error("RTAB_TRACKING_MIN_M must not exceed SDK limit 3 m");
    }

    const auto low_qos = rclcpp::SensorDataQoS().keep_last(4);
    // The sparse map frames are multi-megabyte messages.  Reliable local DDS
    // delivery is preferable here; the high stream is not the latency-critical
    // odometry path and is only published after an exact /odom acknowledgement.
    const auto high_qos = rclcpp::QoS(rclcpp::KeepLast(4)).reliable().durability_volatile();
    odom_rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(kOdomRgbTopic, low_qos);
    odom_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(kOdomDepthTopic, low_qos);
    odom_display_depth_pub_ =
      create_publisher<sensor_msgs::msg::Image>(kOdomDisplayDepthTopic, low_qos);
    odom_rgbd_pub_ =
      create_publisher<rtabmap_msgs::msg::RGBDImage>(kOdomRgbdTopic, low_qos);
    odom_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(kOdomInfoTopic, low_qos);
    map_rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(kMapRgbTopic, high_qos);
    map_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(kMapDepthTopic, high_qos);
    map_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(kMapInfoTopic, high_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(kImuTopic, low_qos);
    odom_ack_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      kOdomTopic, rclcpp::SensorDataQoS().keep_last(64),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const auto &stamp = message->header.stamp;
        if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
          return;
        }
        const std::uint64_t stamp_ns =
          static_cast<std::uint64_t>(stamp.sec) * 1000000000ull + stamp.nanosec;
        const auto callback_steady_ns = steady_now_ns();
        const auto &position = message->pose.pose.position;
        const auto &orientation = message->pose.pose.orientation;
        if (!zed_dual_stream::valid_odom_pose(
              position.x, position.y, position.z,
              orientation.x, orientation.y, orientation.z, orientation.w))
        {
          ++invalid_odom_ack_rejected_count_;
          const auto disposition = map_frame_lifecycle_.resolve_invalid(
            stamp_ns, callback_steady_ns);
          if (disposition ==
            zed_dual_stream::OdomReceiptDisposition::kInvalidPendingDropped)
          {
            pending_map_.reset();
            ++map_invalid_odom_drop_count_;
            const auto *record = map_frame_lifecycle_.lookup(stamp_ns);
            if (!record) throw std::logic_error("invalid drop lacks lifecycle record");
            RCLCPP_WARN(
              get_logger(),
              "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
              "event=invalid_odom_for_pending "
              "map_stamp_ns=%llu planned_sequence=%llu terminal_steady_ns=%lld",
              run_id_.c_str(),
              static_cast<unsigned long long>(next_map_event_sequence()),
              static_cast<unsigned long long>(stamp_ns),
              static_cast<unsigned long long>(record->planned_sequence),
              static_cast<long long>(record->terminal_steady_ns));
          } else if (
            disposition ==
            zed_dual_stream::OdomReceiptDisposition::kInvalidBeforePublishConflict)
          {
            pending_map_.reset();
            ++map_invalid_odom_drop_count_;
            ++map_stamp_conflict_count_;
            const auto *record = map_frame_lifecycle_.lookup(stamp_ns);
            if (!record) throw std::logic_error("invalid conflict lacks lifecycle record");
            RCLCPP_ERROR(
              get_logger(),
              "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
              "event=invalid_before_publish_conflict "
              "map_stamp_ns=%llu planned_sequence=%llu terminal_steady_ns=%lld",
              run_id_.c_str(),
              static_cast<unsigned long long>(next_map_event_sequence()),
              static_cast<unsigned long long>(stamp_ns),
              static_cast<unsigned long long>(record->planned_sequence),
              static_cast<long long>(record->terminal_steady_ns));
          } else if (
            disposition ==
            zed_dual_stream::OdomReceiptDisposition::kInvalidAfterPublishConflict)
          {
            ++map_stamp_conflict_count_;
            RCLCPP_ERROR(
              get_logger(),
              "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
              "event=invalid_after_publish_conflict "
              "map_stamp_ns=%llu",
              run_id_.c_str(),
              static_cast<unsigned long long>(next_map_event_sequence()),
              static_cast<unsigned long long>(stamp_ns));
          } else if (
            disposition ==
            zed_dual_stream::OdomReceiptDisposition::kLateInvalidCausedTimeout)
          {
            pending_map_.reset();
            ++map_unacknowledged_drop_count_;
            const auto *timed_out_record = map_frame_lifecycle_.lookup(stamp_ns);
            if (!timed_out_record ||
              timed_out_record->state != zed_dual_stream::MapFrameState::kTimedOut)
            {
              throw std::logic_error("late invalid timeout lacks finalized lifecycle record");
            }
            RCLCPP_WARN(
              get_logger(),
              "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
              "event=timeout_no_matching_odom_received_by_bridge_before_deadline "
              "map_stamp_ns=%llu planned_sequence=%llu "
              "reason=invalid_receipt_after_deadline timeout_ms=%.3f terminal_steady_ns=%lld",
              run_id_.c_str(),
              static_cast<unsigned long long>(next_map_event_sequence()),
              static_cast<unsigned long long>(stamp_ns),
              static_cast<unsigned long long>(timed_out_record->planned_sequence),
              static_cast<double>(kMapOdomAckTimeoutNs) / 1.0e6,
              static_cast<long long>(timed_out_record->terminal_steady_ns));
            RCLCPP_WARN(
              get_logger(),
              "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
              "event=late_invalid_after_timeout map_stamp_ns=%llu",
              run_id_.c_str(),
              static_cast<unsigned long long>(next_map_event_sequence()),
              static_cast<unsigned long long>(stamp_ns));
          } else if (
            disposition ==
            zed_dual_stream::OdomReceiptDisposition::kLateInvalidAfterTimeout)
          {
            RCLCPP_WARN(
              get_logger(),
              "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
              "event=late_invalid_after_timeout "
              "map_stamp_ns=%llu",
              run_id_.c_str(),
              static_cast<unsigned long long>(next_map_event_sequence()),
              static_cast<unsigned long long>(stamp_ns));
          } else if (
            disposition == zed_dual_stream::OdomReceiptDisposition::kInvalidAfterShutdown)
          {
            RCLCPP_WARN(
              get_logger(),
              "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
              "event=invalid_after_shutdown "
              "map_stamp_ns=%llu",
              run_id_.c_str(),
              static_cast<unsigned long long>(next_map_event_sequence()),
              static_cast<unsigned long long>(stamp_ns));
          }
          return;
        }
        const auto callback_now_ns = get_clock()->now().nanoseconds();
        if (stamp_ns == last_odom_ack_stamp_ns_) {
          ++duplicate_valid_odom_ack_count_;
        } else if (stamp_ns < last_odom_ack_stamp_ns_) {
          ++regressed_valid_odom_ack_count_;
        } else {
          last_odom_ack_stamp_ns_ = stamp_ns;
          ++unique_valid_odom_ack_count_;
          if (callback_now_ns >= 0 &&
            static_cast<std::uint64_t>(callback_now_ns) >= stamp_ns)
          {
            image_to_odom_ms_window_.add(
              static_cast<double>(static_cast<std::uint64_t>(callback_now_ns) - stamp_ns) /
              1.0e6);
          } else {
            image_to_odom_ms_window_.add(std::numeric_limits<double>::quiet_NaN());
          }
        }
        const auto disposition = map_frame_lifecycle_.resolve_valid(
          stamp_ns, callback_steady_ns);
        if (disposition == zed_dual_stream::OdomReceiptDisposition::kOnTimeValidReady) {
          RCLCPP_INFO(
            get_logger(),
            "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
            "event=valid_odom_received_on_time "
            "map_stamp_ns=%llu receipt_steady_ns=%lld",
            run_id_.c_str(),
            static_cast<unsigned long long>(next_map_event_sequence()),
            static_cast<unsigned long long>(stamp_ns),
            static_cast<long long>(callback_steady_ns));
        } else if (
          disposition == zed_dual_stream::OdomReceiptDisposition::kLateValidCausedTimeout)
        {
          pending_map_.reset();
          ++map_unacknowledged_drop_count_;
          ++map_late_valid_after_timeout_count_;
          const auto *timed_out_record = map_frame_lifecycle_.lookup(stamp_ns);
          if (!timed_out_record ||
            timed_out_record->state != zed_dual_stream::MapFrameState::kTimedOut)
          {
            throw std::logic_error("late valid timeout lacks finalized lifecycle record");
          }
          RCLCPP_WARN(
            get_logger(),
            "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
            "event=timeout_no_matching_odom_received_by_bridge_before_deadline "
            "map_stamp_ns=%llu planned_sequence=%llu "
            "reason=valid_receipt_after_deadline timeout_ms=%.3f terminal_steady_ns=%lld",
            run_id_.c_str(),
            static_cast<unsigned long long>(next_map_event_sequence()),
            static_cast<unsigned long long>(stamp_ns),
            static_cast<unsigned long long>(timed_out_record->planned_sequence),
            static_cast<double>(kMapOdomAckTimeoutNs) / 1.0e6,
            static_cast<long long>(timed_out_record->terminal_steady_ns));
          RCLCPP_WARN(
            get_logger(),
            "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
            "event=late_valid_after_timeout "
            "map_stamp_ns=%llu",
            run_id_.c_str(),
            static_cast<unsigned long long>(next_map_event_sequence()),
            static_cast<unsigned long long>(stamp_ns));
        } else if (
          disposition == zed_dual_stream::OdomReceiptDisposition::kLateValidAfterTimeout)
        {
          ++map_late_valid_after_timeout_count_;
          RCLCPP_WARN(
            get_logger(),
            "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
            "event=late_valid_after_timeout "
            "map_stamp_ns=%llu",
            run_id_.c_str(),
            static_cast<unsigned long long>(next_map_event_sequence()),
            static_cast<unsigned long long>(stamp_ns));
        } else if (
          disposition == zed_dual_stream::OdomReceiptDisposition::kValidAfterInvalidConflict)
        {
          ++map_stamp_conflict_count_;
          RCLCPP_ERROR(
            get_logger(),
            "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
            "event=valid_after_invalid_conflict "
            "map_stamp_ns=%llu",
            run_id_.c_str(),
            static_cast<unsigned long long>(next_map_event_sequence()),
            static_cast<unsigned long long>(stamp_ns));
        } else if (
          disposition == zed_dual_stream::OdomReceiptDisposition::kValidAfterShutdown)
        {
          RCLCPP_WARN(
            get_logger(),
            "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
            "event=late_valid_after_shutdown "
            "map_stamp_ns=%llu",
            run_id_.c_str(),
            static_cast<unsigned long long>(next_map_event_sequence()),
            static_cast<unsigned long long>(stamp_ns));
        }
      });

    sl::InitParameters init;
    init.camera_resolution = sl::RESOLUTION::HD1200;
    init.camera_fps = grab_fps_;
    // ZED SDK's native split: keep the camera capture mode at 60 fps for
    // shorter exposure/latency while capping image/depth computation at the
    // sustainable low-stream target.
    init.grab_compute_capping_fps = static_cast<float>(compute_cap_hz_);
    init.depth_mode = parse_depth_mode(depth_mode_name_);
    init.coordinate_units = sl::UNIT::METER;
    init.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init.depth_minimum_distance = static_cast<float>(tracking_range_.minimum);
    init.depth_maximum_distance = static_cast<float>(tracking_range_.maximum);
    init.camera_disable_self_calib = true;
    if (!calibration_file_.empty()) {
      init.optional_opencv_calibration_file = calibration_file_.c_str();
    }
    const auto status = camera_.open(init);
    if (status != sl::ERROR_CODE::SUCCESS) {
      throw std::runtime_error(
              std::string("ZED X open failed: ") + sl::toString(status).c_str());
    }

    const auto information = camera_.getCameraInformation();
    try {
      audit_open_calibration(get_logger(), information, calibration_file_);
    } catch (...) {
      camera_.close();
      throw;
    }
    const auto odom_information = camera_.getCameraInformation(
      sl::Resolution(odom_width_, odom_height_));
    const auto map_information = camera_.getCameraInformation(
      sl::Resolution(map_width_, map_height_));
    odom_geometry_ = make_geometry(
      odom_width_, odom_height_,
      odom_information.camera_configuration.calibration_parameters.left_cam);
    map_geometry_ = make_geometry(
      map_width_, map_height_,
      map_information.camera_configuration.calibration_parameters.left_cam);

    const auto camera_imu_transform =
      information.sensors_configuration.camera_imu_transform;
    camera_from_imu_rotation_ =
      matrix_from_sdk(camera_imu_transform.getRotationMatrix());
    runtime_.enable_depth = false;
    report_started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
      "[DUAL_STREAM] native=HD1200@%d sdk_compute_cap=%.2f depth=%s "
      "odom=%dx%d@%.2f map=%dx%d@%.2f "
      "same_grab=true map_stamp_subset_of_odom=true map_publish_requires_exact_valid_odom_ack=true "
      "low_published_before_high_retrieve=true "
      "self_calibration=disabled",
      grab_fps_, compute_cap_hz_, depth_mode_name_.c_str(),
      odom_width_, odom_height_, odom_publish_hz_,
      map_width_, map_height_, map_publish_hz_);
    RCLCPP_INFO(get_logger(),
      "[DUAL_STREAM] odom_K=[%.6f %.6f %.6f %.6f] map_K=[%.6f %.6f %.6f %.6f]",
      odom_geometry_.fx, odom_geometry_.fy, odom_geometry_.cx, odom_geometry_.cy,
      map_geometry_.fx, map_geometry_.fy, map_geometry_.cx, map_geometry_.cy);
    RCLCPP_INFO(
      get_logger(),
      "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
      "event=run_started timeout_ns=%lld single_threaded_executor=true "
      "one_authoritative_odom_verdict_per_stamp_required=true producer_pid=%llu "
      "log_stream=dedicated_bridge_stdout_stderr "
      "map_triplet_semantics=local_publish_calls_only",
      run_id_.c_str(), static_cast<unsigned long long>(next_map_event_sequence()),
      static_cast<long long>(kMapOdomAckTimeoutNs),
      static_cast<unsigned long long>(::getpid()));
  }

  ~ZedDualStreamPublisher() override
  {
    if (const auto record = map_frame_lifecycle_.shutdown_censor(steady_now_ns())) {
      pending_map_.reset();
      if (record->state == zed_dual_stream::MapFrameState::kShutdownCensored) {
        ++map_shutdown_censored_count_;
        RCLCPP_WARN(
          get_logger(),
          "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
          "event=shutdown_censored "
          "map_stamp_ns=%llu planned_sequence=%llu terminal_steady_ns=%lld",
          run_id_.c_str(),
          static_cast<unsigned long long>(next_map_event_sequence()),
          static_cast<unsigned long long>(record->stamp_ns),
          static_cast<unsigned long long>(record->planned_sequence),
          static_cast<long long>(record->terminal_steady_ns));
      } else {
        ++map_publish_failed_count_;
        RCLCPP_ERROR(
          get_logger(),
          "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
          "event=publish_failed "
          "map_stamp_ns=%llu planned_sequence=%llu reason=shutdown_while_publishing "
          "terminal_steady_ns=%lld",
          run_id_.c_str(),
          static_cast<unsigned long long>(next_map_event_sequence()),
          static_cast<unsigned long long>(record->stamp_ns),
          static_cast<unsigned long long>(record->planned_sequence),
          static_cast<long long>(record->terminal_steady_ns));
      }
    }
    camera_.close();
    RCLCPP_INFO(
      get_logger(),
      "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
      "event=run_finished",
      run_id_.c_str(),
      static_cast<unsigned long long>(next_map_event_sequence()));
  }

  bool capture_and_publish()
  {
    publish_pending_map_if_ready();
    const auto decision_time = std::chrono::steady_clock::now();
    const auto decision_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      decision_time.time_since_epoch()).count();
    const auto decision = schedule_.preview(decision_ns);
    const std::uint64_t planned_sequence = decision.publish_map ?
      ++map_planned_sequence_ : 0;
    bool plan_outcome_logged = false;
    const auto log_plan_outcome =
      [this, planned_sequence, &plan_outcome_logged](
        const char *outcome, std::uint64_t stamp_ns) {
        if (planned_sequence == 0) return;
        if (plan_outcome_logged) {
          throw std::logic_error("map planned tick received multiple attempt outcomes");
        }
        plan_outcome_logged = true;
        RCLCPP_INFO(
          get_logger(),
          "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
          "event=map_plan_outcome "
          "planned_sequence=%llu outcome=%s map_stamp_ns=%llu",
          run_id_.c_str(),
          static_cast<unsigned long long>(next_map_event_sequence()),
          static_cast<unsigned long long>(planned_sequence), outcome,
          static_cast<unsigned long long>(stamp_ns));
      };
    if (planned_sequence != 0) {
      RCLCPP_INFO(
        get_logger(),
        "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
        "event=map_due_planned "
        "planned_sequence=%llu decision_steady_ns=%lld",
        run_id_.c_str(),
        static_cast<unsigned long long>(next_map_event_sequence()),
        static_cast<unsigned long long>(planned_sequence),
        static_cast<long long>(decision_ns));
    }
    if (const auto expired = map_frame_lifecycle_.expire(decision_ns)) {
      pending_map_.reset();
      ++map_unacknowledged_drop_count_;
      RCLCPP_WARN(
        get_logger(),
        "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
        "event=timeout_no_matching_odom_received_by_bridge_before_deadline "
        "map_stamp_ns=%llu planned_sequence=%llu reason=no_receipt_before_poll "
        "timeout_ms=%.3f terminal_steady_ns=%lld",
        run_id_.c_str(),
        static_cast<unsigned long long>(next_map_event_sequence()),
        static_cast<unsigned long long>(expired->stamp_ns),
        static_cast<unsigned long long>(expired->planned_sequence),
        static_cast<double>(kMapOdomAckTimeoutNs) / 1.0e6,
        static_cast<long long>(expired->terminal_steady_ns));
    }
    // Keep the sensor at native 60 fps, but do not run NEURAL depth on grabs
    // which will not be consumed by either stream.
    runtime_.enable_depth = decision.publish_odom;
    if (camera_.grab(runtime_) != sl::ERROR_CODE::SUCCESS) {
      ++grab_error_count_;
      log_plan_outcome("grab_failure", 0);
      return false;
    }
    if (camera_.getSensorsData(sensors_data_, sl::TIME_REFERENCE::IMAGE) !=
        sl::ERROR_CODE::SUCCESS || !sensors_data_.imu.is_available)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[IMU_AUDIT] rejecting grab without image-synchronized IMU");
      log_plan_outcome("imu_failure", 0);
      return false;
    }
    const std::uint64_t image_stamp_ns =
      camera_.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
    if (image_stamp_ns == 0) {
      log_plan_outcome("zero_image_stamp", 0);
      throw std::runtime_error("ZED image timestamp is zero");
    }
    if (image_stamp_ns < previous_image_stamp_ns_) {
      log_plan_outcome("regressed_image_stamp", image_stamp_ns);
      throw std::runtime_error("ZED image timestamp regressed");
    }
    if (image_stamp_ns == previous_image_stamp_ns_) {
      // grab_compute_capping_fps may return SUCCESS while the SDK intentionally
      // reuses the last computed image.  This is not a new RGB-D sample and
      // must neither be published nor treated as a camera failure.
      ++duplicate_image_stamp_count_;
      log_plan_outcome("duplicate_image_stamp", image_stamp_ns);
      return false;
    }
    const auto image_orientation = sensors_data_.imu.pose.getOrientation();
    image_rotation_window_.add(
      {image_orientation.x, image_orientation.y, image_orientation.z, image_orientation.w});
    const rclcpp::Time stamp(static_cast<std::int64_t>(image_stamp_ns));

    // The low stream is the control path for visual odometry.  Retrieve,
    // construct and publish it before doing any full-resolution work.  This
    // deliberately gives up the old all-or-nothing low/high publish commit:
    // a slow high-resolution copy must never hold back a valid odometry frame.
    if (decision.publish_odom && !retrieve_stream(
        odom_geometry_, odom_left_, odom_depth_, image_stamp_ns))
    {
      log_plan_outcome("odom_retrieve_failure", image_stamp_ns);
      throw std::runtime_error("due odometry RGB-D retrieve failed");
    }
    std::unique_ptr<sensor_msgs::msg::Image> odom_color;
    DepthMessages odom_depth_messages;
    sensor_msgs::msg::CameraInfo odom_info;

    if (decision.publish_odom) {
      odom_color = color_message(odom_left_, odom_geometry_, stamp);
      odom_depth_messages = depth_messages(
        odom_depth_, odom_geometry_, stamp, tracking_range_, display_range_, true);
      odom_info = odom_geometry_.info;
      odom_info.header.stamp = stamp;
    }

    publish_imu(stamp);
    if (decision.publish_odom) {
      last_odom_tracking_valid_ = odom_depth_messages.tracking_valid_ratio;
      last_odom_display_valid_ = odom_depth_messages.display_valid_ratio;
      auto odom_rgbd = std::make_unique<rtabmap_msgs::msg::RGBDImage>();
      odom_rgbd->header = odom_color->header;
      odom_rgbd->rgb_camera_info = odom_info;
      odom_rgbd->depth_camera_info = odom_info;
      odom_rgbd->rgb = *odom_color;
      odom_rgbd->depth = *odom_depth_messages.tracking;
      // Publish the atomic odometry input first. Raw topics remain for RViz,
      // AprilTag and the single-frame cloud, but odometry no longer needs
      // three independent DDS samples to survive exact synchronization.
      odom_rgbd_pub_->publish(std::move(odom_rgbd));
      odom_rgb_pub_->publish(std::move(odom_color));
      odom_depth_pub_->publish(std::move(odom_depth_messages.tracking));
      odom_display_depth_pub_->publish(std::move(odom_depth_messages.display));
      odom_info_pub_->publish(odom_info);
      const auto low_publish_now_ns = get_clock()->now().nanoseconds();
      if (low_publish_now_ns >= 0 &&
        static_cast<std::uint64_t>(low_publish_now_ns) >= image_stamp_ns)
      {
        image_to_low_publish_ms_window_.add(
          static_cast<double>(static_cast<std::uint64_t>(low_publish_now_ns) - image_stamp_ns) /
          1.0e6);
      } else {
        image_to_low_publish_ms_window_.add(std::numeric_limits<double>::quiet_NaN());
      }
      low_path_ms_window_.add(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - decision_time).count());
      ++odom_count_;
    }

    if (decision.publish_map) {
      if (!decision.publish_odom) {
        throw std::logic_error("map frame must also be an odometry frame");
      }
      if (!map_frame_lifecycle_.active()) {
        if (!retrieve_stream(map_geometry_, map_left_, map_depth_, image_stamp_ns)) {
          log_plan_outcome("map_retrieve_failure", image_stamp_ns);
          throw std::runtime_error(
                  "due mapping RGB-D retrieve failed after low frame publication");
        }
        auto map_depth_messages = depth_messages(
          map_depth_, map_geometry_, stamp, tracking_range_, display_range_, false);
        PendingMapFrame pending;
        pending.color = color_message(map_left_, map_geometry_, stamp);
        pending.depth = std::move(map_depth_messages.tracking);
        pending.info = map_geometry_.info;
        pending.info.header.stamp = stamp;
        pending.tracking_valid_ratio = map_depth_messages.tracking_valid_ratio;
        pending_map_ = std::move(pending);
        const auto queue_steady_ns = steady_now_ns();
        const auto evicted = map_frame_lifecycle_.queue(
          image_stamp_ns, planned_sequence, queue_steady_ns);
        if (evicted) {
          ++map_stamp_tombstone_eviction_count_;
          RCLCPP_WARN(
            get_logger(),
            "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
            "event=tombstone_evicted "
            "map_stamp_ns=%llu state=%s conflict_poisoned=%s",
            run_id_.c_str(),
            static_cast<unsigned long long>(next_map_event_sequence()),
            static_cast<unsigned long long>(evicted->stamp_ns),
            zed_dual_stream::map_frame_state_name(evicted->state),
            evicted->conflict_poisoned ? "true" : "false");
        }
        ++map_queued_count_;
        RCLCPP_INFO(
          get_logger(),
          "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
          "event=queued_waiting_exact_odom "
          "map_stamp_ns=%llu planned_sequence=%llu queue_steady_ns=%lld",
          run_id_.c_str(),
          static_cast<unsigned long long>(next_map_event_sequence()),
          static_cast<unsigned long long>(image_stamp_ns),
          static_cast<unsigned long long>(planned_sequence),
          static_cast<long long>(queue_steady_ns));
        log_plan_outcome("queued", image_stamp_ns);
      } else {
        ++map_due_while_pending_count_;
        RCLCPP_WARN(
          get_logger(),
          "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
          "event=map_due_while_previous_pending "
          "planned_sequence=%llu skipped_stamp_ns=%llu pending_stamp_ns=%llu",
          run_id_.c_str(),
          static_cast<unsigned long long>(next_map_event_sequence()),
          static_cast<unsigned long long>(planned_sequence),
          static_cast<unsigned long long>(image_stamp_ns),
          static_cast<unsigned long long>(
            map_frame_lifecycle_.active_record()->stamp_ns));
        log_plan_outcome("due_previous_pending", image_stamp_ns);
      }
    }

    if (planned_sequence != 0 && !plan_outcome_logged) {
      throw std::logic_error("map planned tick completed without an attempt outcome");
    }

    schedule_.commit(decision, decision_ns);
    previous_image_stamp_ns_ = image_stamp_ns;
    if (report_first_image_stamp_ns_ == 0) {
      report_first_image_stamp_ns_ = image_stamp_ns;
    }
    report_last_image_stamp_ns_ = image_stamp_ns;
    ++grab_count_;

    const auto steady_now = std::chrono::steady_clock::now();
    const double report_elapsed =
      std::chrono::duration<double>(steady_now - report_started_).count();
    if (report_elapsed >= 5.0) {
      double grab_sdk_hz = 0.0;
      if (grab_count_ > 1 && report_last_image_stamp_ns_ > report_first_image_stamp_ns_) {
        grab_sdk_hz = static_cast<double>(grab_count_ - 1) * 1.0e9 /
          static_cast<double>(report_last_image_stamp_ns_ - report_first_image_stamp_ns_);
      }
      const double odom_hz = odom_count_ / report_elapsed;
      const double valid_odom_ack_hz = unique_valid_odom_ack_count_ / report_elapsed;
      const double map_hz = map_count_ / report_elapsed;
      const char *status = valid_odom_ack_hz >= 15.0 ? "met" : "missed";
      const auto image_rotation = image_rotation_window_.summary();
      const auto low_path = low_path_ms_window_.summary();
      const auto image_to_low_publish = image_to_low_publish_ms_window_.summary();
      const auto image_to_odom = image_to_odom_ms_window_.summary();
      RCLCPP_INFO(get_logger(),
        "[CAMERA_RATE] schema_version=1 accepted_grab_sdk_stamp_hz=%.2f "
        "odom_published_hz=%.2f valid_unique_odom_ack_hz=%.2f "
        "map_published_hz=%.2f odom_consumer_minimum_15hz=%s grab_errors=%zu "
        "duplicate_image_stamps_skipped=%zu "
        "map_queued=%zu map_exact_valid_acks=%zu map_unacknowledged_dropped=%zu "
        "map_invalid_odom_dropped=%zu map_due_while_pending=%zu "
        "map_stamp_conflicts=%zu map_late_valid_after_timeout=%zu "
        "map_tombstone_evictions=%zu map_shutdown_censored=%zu map_publish_failed=%zu "
        "invalid_odom_ack_rejected=%zu "
        "duplicate_valid_odom_acks=%zu regressed_valid_odom_acks=%zu "
        "odom_tracking/display_valid_pct=%.1f/%.1f map_tracking_valid_pct=%.1f",
        grab_sdk_hz, odom_hz, valid_odom_ack_hz, map_hz, status, grab_error_count_,
        duplicate_image_stamp_count_, map_queued_count_, map_exact_valid_ack_count_,
        map_unacknowledged_drop_count_, map_invalid_odom_drop_count_,
        map_due_while_pending_count_, map_stamp_conflict_count_,
        map_late_valid_after_timeout_count_, map_stamp_tombstone_eviction_count_,
        map_shutdown_censored_count_, map_publish_failed_count_,
        invalid_odom_ack_rejected_count_, duplicate_valid_odom_ack_count_,
        regressed_valid_odom_ack_count_,
        100.0 * last_odom_tracking_valid_, 100.0 * last_odom_display_valid_,
        100.0 * last_map_tracking_valid_);
      RCLCPP_INFO(get_logger(),
        "[IMU_IMAGE_DELTA] schema_version=1 source=image_synchronized_sdk_orientation "
        "quaternion_sign_invariant=true samples=%zu invalid_quaternions=%zu "
        "mean_deg=%.6f p50_deg=%.6f p95_deg=%.6f max_deg=%.6f",
        image_rotation.count, image_rotation_window_.invalid_quaternion_count(),
        image_rotation.mean, image_rotation.p50, image_rotation.p95, image_rotation.maximum);
      RCLCPP_INFO(get_logger(),
        "[LOW_PATH_LATENCY] schema_version=1 scope=publisher_capture_enter_to_low_publish "
        "includes_grab_retrieve_message_build=true excludes_dds_and_odometry=true "
        "samples=%zu invalid_samples=%zu mean_ms=%.3f p50_ms=%.3f p95_ms=%.3f max_ms=%.3f",
        low_path.count, low_path_ms_window_.invalid_count(),
        low_path.mean, low_path.p50, low_path.p95, low_path.maximum);
      RCLCPP_INFO(get_logger(),
        "[STAMP_LATENCY] schema_version=1 clock=node_clock "
        "image_to_low_publish_samples=%zu invalid=%zu mean_ms=%.3f p50_ms=%.3f "
        "p95_ms=%.3f max_ms=%.3f image_to_valid_odom_samples=%zu invalid=%zu "
        "mean_ms=%.3f p50_ms=%.3f p95_ms=%.3f max_ms=%.3f",
        image_to_low_publish.count, image_to_low_publish_ms_window_.invalid_count(),
        image_to_low_publish.mean, image_to_low_publish.p50,
        image_to_low_publish.p95, image_to_low_publish.maximum,
        image_to_odom.count, image_to_odom_ms_window_.invalid_count(),
        image_to_odom.mean, image_to_odom.p50, image_to_odom.p95, image_to_odom.maximum);
      log_camera_settings();
      grab_count_ = 0;
      grab_error_count_ = 0;
      duplicate_image_stamp_count_ = 0;
      odom_count_ = 0;
      map_count_ = 0;
      map_queued_count_ = 0;
      map_exact_valid_ack_count_ = 0;
      map_unacknowledged_drop_count_ = 0;
      map_invalid_odom_drop_count_ = 0;
      map_due_while_pending_count_ = 0;
      map_stamp_conflict_count_ = 0;
      map_late_valid_after_timeout_count_ = 0;
      map_stamp_tombstone_eviction_count_ = 0;
      map_shutdown_censored_count_ = 0;
      map_publish_failed_count_ = 0;
      invalid_odom_ack_rejected_count_ = 0;
      unique_valid_odom_ack_count_ = 0;
      duplicate_valid_odom_ack_count_ = 0;
      regressed_valid_odom_ack_count_ = 0;
      report_first_image_stamp_ns_ = 0;
      report_last_image_stamp_ns_ = 0;
      image_rotation_window_.reset_report_window();
      low_path_ms_window_.reset();
      image_to_low_publish_ms_window_.reset();
      image_to_odom_ms_window_.reset();
      report_started_ = steady_now;
    }
    return true;
  }

private:
  struct PendingMapFrame
  {
    std::unique_ptr<sensor_msgs::msg::Image> color;
    std::unique_ptr<sensor_msgs::msg::Image> depth;
    sensor_msgs::msg::CameraInfo info;
    double tracking_valid_ratio{};
  };

  std::uint64_t next_map_event_sequence()
  {
    return ++map_event_sequence_;
  }

  void log_camera_settings()
  {
    const auto started = std::chrono::steady_clock::now();
    int aec_agc = -1;
    int exposure_percent = -1;
    int gain_percent = -1;
    int exposure_time_us = -1;
    int analog_gain_mdb = -1;
    int digital_gain_factor = -1;
    int scene_illuminance_0p1_lux = -1;
    int auto_exposure_min_us = -1;
    int auto_exposure_max_us = -1;

    const bool aec_agc_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::AEC_AGC, aec_agc) == sl::ERROR_CODE::SUCCESS;
    const bool exposure_percent_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::EXPOSURE, exposure_percent) == sl::ERROR_CODE::SUCCESS;
    const bool gain_percent_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::GAIN, gain_percent) == sl::ERROR_CODE::SUCCESS;
    const bool exposure_time_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::EXPOSURE_TIME, exposure_time_us) == sl::ERROR_CODE::SUCCESS;
    const bool analog_gain_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::ANALOG_GAIN, analog_gain_mdb) == sl::ERROR_CODE::SUCCESS;
    const bool digital_gain_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::DIGITAL_GAIN, digital_gain_factor) == sl::ERROR_CODE::SUCCESS;
    const bool scene_illuminance_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::SCENE_ILLUMINANCE, scene_illuminance_0p1_lux) ==
      sl::ERROR_CODE::SUCCESS;
    const bool auto_exposure_range_valid = camera_.getCameraSettings(
      sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE,
      auto_exposure_min_us, auto_exposure_max_us) == sl::ERROR_CODE::SUCCESS;
    const std::size_t read_errors =
      static_cast<std::size_t>(!aec_agc_valid) +
      static_cast<std::size_t>(!exposure_percent_valid) +
      static_cast<std::size_t>(!gain_percent_valid) +
      static_cast<std::size_t>(!exposure_time_valid) +
      static_cast<std::size_t>(!analog_gain_valid) +
      static_cast<std::size_t>(!digital_gain_valid) +
      static_cast<std::size_t>(!scene_illuminance_valid) +
      static_cast<std::size_t>(!auto_exposure_range_valid);
    const double frame_period_us = 1.0e6 / static_cast<double>(grab_fps_);
    const double exposure_frame_percent = exposure_time_valid ?
      100.0 * static_cast<double>(exposure_time_us) / frame_period_us :
      std::numeric_limits<double>::quiet_NaN();
    const double read_elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();

    RCLCPP_INFO(get_logger(),
      "[CAMERA_EXPOSURE] schema_version=1 settings_read_only=true "
      "aec_agc_valid=%s aec_agc=%d exposure_percent_valid=%s exposure_percent=%d "
      "gain_percent_valid=%s gain_percent=%d "
      "exposure_time_valid=%s exposure_time_us=%d exposure_frame_percent=%.3f "
      "analog_gain_valid=%s analog_gain_mdb=%d "
      "digital_gain_valid=%s digital_gain_factor=%d "
      "scene_illuminance_valid=%s scene_illuminance_0p1_lux=%d "
      "auto_exposure_range_valid=%s auto_exposure_min_us=%d auto_exposure_max_us=%d "
      "read_errors=%zu read_elapsed_ms=%.3f",
      aec_agc_valid ? "true" : "false", aec_agc,
      exposure_percent_valid ? "true" : "false", exposure_percent,
      gain_percent_valid ? "true" : "false", gain_percent,
      exposure_time_valid ? "true" : "false", exposure_time_us, exposure_frame_percent,
      analog_gain_valid ? "true" : "false", analog_gain_mdb,
      digital_gain_valid ? "true" : "false", digital_gain_factor,
      scene_illuminance_valid ? "true" : "false", scene_illuminance_0p1_lux,
      auto_exposure_range_valid ? "true" : "false",
      auto_exposure_min_us, auto_exposure_max_us,
      read_errors, read_elapsed_ms);
  }

  void publish_pending_map_if_ready()
  {
    if (!pending_map_ || !map_frame_lifecycle_.ready()) {
      return;
    }
    const auto *active = map_frame_lifecycle_.active_record();
    if (!active || active->conflict_poisoned) {
      throw std::logic_error("map publication requires a clean active lifecycle record");
    }
    const std::uint64_t stamp_ns = active->stamp_ns;
    const std::int64_t queued_steady_ns = active->queued_steady_ns;
    const std::int64_t receipt_steady_ns = active->receipt_steady_ns;
    map_frame_lifecycle_.begin_publish(stamp_ns);
    try {
      last_map_tracking_valid_ = pending_map_->tracking_valid_ratio;
      map_rgb_pub_->publish(std::move(pending_map_->color));
      map_depth_pub_->publish(std::move(pending_map_->depth));
      map_info_pub_->publish(pending_map_->info);
    } catch (...) {
      const auto failed = map_frame_lifecycle_.fail_publish(stamp_ns, steady_now_ns());
      pending_map_.reset();
      ++map_publish_failed_count_;
      RCLCPP_ERROR(
        get_logger(),
        "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
        "event=publish_failed "
        "map_stamp_ns=%llu planned_sequence=%llu reason=map_triplet_publish_exception "
        "terminal_steady_ns=%lld",
        run_id_.c_str(),
        static_cast<unsigned long long>(next_map_event_sequence()),
        static_cast<unsigned long long>(stamp_ns),
        static_cast<unsigned long long>(failed.planned_sequence),
        static_cast<long long>(failed.terminal_steady_ns));
      throw;
    }
    // Commit outside the publish exception handler. Later bookkeeping/logging
    // failures must not be relabelled as a publish failure or create a second
    // terminal verdict for the same stamp.
    const auto committed = map_frame_lifecycle_.commit_publish(
      stamp_ns, steady_now_ns());
    pending_map_.reset();
    ++map_count_;
    ++map_exact_valid_ack_count_;
    const double ack_wait_ms = static_cast<double>(
      receipt_steady_ns - queued_steady_ns) / 1.0e6;
    RCLCPP_INFO(
      get_logger(),
      "[MAP_STAMP_EVENT] schema_version=2 run_id=%s event_sequence=%llu "
      "event=exact_valid_odom_ack "
      "map_stamp_ns=%llu planned_sequence=%llu ack_wait_ms=%.3f "
      "map_triplet_publish_calls_completed=true dds_atomic=false "
      "downstream_rtabmap_receipt=false terminal_steady_ns=%lld",
      run_id_.c_str(),
      static_cast<unsigned long long>(next_map_event_sequence()),
      static_cast<unsigned long long>(stamp_ns),
      static_cast<unsigned long long>(committed.planned_sequence), ack_wait_ms,
      static_cast<long long>(committed.terminal_steady_ns));
  }

  bool retrieve_stream(
    const Geometry &geometry, sl::Mat &left, sl::Mat &depth,
    std::uint64_t expected_stamp_ns)
  {
    const bool retrieved = camera_.retrieveImage(
      left, sl::VIEW::LEFT, sl::MEM::CPU, geometry.resolution) ==
      sl::ERROR_CODE::SUCCESS &&
      camera_.retrieveMeasure(
      depth, sl::MEASURE::DEPTH, sl::MEM::CPU, geometry.resolution) ==
      sl::ERROR_CODE::SUCCESS;
    if (!retrieved) {
      return false;
    }
    if (static_cast<int>(left.getWidth()) != geometry.width ||
        static_cast<int>(left.getHeight()) != geometry.height ||
        left.getDataType() != sl::MAT_TYPE::U8_C4 ||
        left.getStepBytes(sl::MEM::CPU) < static_cast<std::size_t>(geometry.width) * 4u)
    {
      throw std::runtime_error("retrieved RGB Mat geometry/type/step mismatch");
    }
    if (static_cast<int>(depth.getWidth()) != geometry.width ||
        static_cast<int>(depth.getHeight()) != geometry.height ||
        depth.getDataType() != sl::MAT_TYPE::F32_C1 ||
        depth.getStepBytes(sl::MEM::CPU) <
        static_cast<std::size_t>(geometry.width) * sizeof(float))
    {
      throw std::runtime_error("retrieved depth Mat geometry/type/step mismatch");
    }
    if (left.timestamp.getNanoseconds() != expected_stamp_ns ||
        depth.timestamp.getNanoseconds() != expected_stamp_ns)
    {
      throw std::runtime_error("retrieved RGB/depth timestamp differs from grab timestamp");
    }
    return true;
  }

  void publish_imu(const rclcpp::Time &stamp)
  {
    const auto sdk_orientation = sensors_data_.imu.pose.getOrientation();
    const auto converted = zed_imu_conversion::convert(
      {sdk_orientation.x, sdk_orientation.y, sdk_orientation.z, sdk_orientation.w},
      camera_from_imu_rotation_,
      vector_from_sdk(sensors_data_.imu.angular_velocity),
      vector_from_sdk(sensors_data_.imu.linear_acceleration),
      matrix_from_sdk(sensors_data_.imu.pose_covariance),
      matrix_from_sdk(sensors_data_.imu.angular_velocity_covariance),
      matrix_from_sdk(sensors_data_.imu.linear_acceleration_covariance));
    zed_imu_conversion::Quaternion gravity_orientation;
    if (!zed_imu_conversion::acceleration_gravity_world_from_camera(
          converted.linear_acceleration_camera_m_s2, gravity_orientation))
    {
      ++imu_gravity_rejected_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[IMU_GRAVITY] rejecting sample outside finite 8-12 m/s^2 gate; "
        "rejected=%zu", imu_gravity_rejected_count_);
      return;
    }
    auto imu = std::make_unique<sensor_msgs::msg::Imu>();
    imu->header.stamp = stamp;
    imu->header.frame_id = kOpticalFrame;
    imu->orientation.x = gravity_orientation.x;
    imu->orientation.y = gravity_orientation.y;
    imu->orientation.z = gravity_orientation.z;
    imu->orientation.w = gravity_orientation.w;
    imu->angular_velocity.x = converted.angular_velocity_camera_rad_s[0];
    imu->angular_velocity.y = converted.angular_velocity_camera_rad_s[1];
    imu->angular_velocity.z = converted.angular_velocity_camera_rad_s[2];
    imu->linear_acceleration.x = converted.linear_acceleration_camera_m_s2[0];
    imu->linear_acceleration.y = converted.linear_acceleration_camera_m_s2[1];
    imu->linear_acceleration.z = converted.linear_acceleration_camera_m_s2[2];
    imu->orientation_covariance = converted.orientation_covariance_camera;
    imu->angular_velocity_covariance =
      converted.angular_velocity_covariance_camera_rad2_s2;
    imu->linear_acceleration_covariance =
      converted.linear_acceleration_covariance_camera;
    imu_pub_->publish(std::move(imu));
  }

  zed_depth_ranges::Interval tracking_range_;
  zed_depth_ranges::Interval display_range_;
  std::string grab_resolution_name_;
  int grab_fps_{};
  int odom_width_{};
  int odom_height_{};
  int map_width_{};
  int map_height_{};
  double odom_publish_hz_{};
  double map_publish_hz_{};
  double compute_cap_hz_{};
  zed_dual_stream::Schedule schedule_;
  std::string depth_mode_name_;
  std::string calibration_file_;
  std::string run_id_{
    std::to_string(static_cast<unsigned long long>(::getpid())) + "-" +
    std::to_string(static_cast<unsigned long long>(steady_now_ns()))};

  sl::Camera camera_;
  sl::RuntimeParameters runtime_;
  sl::SensorsData sensors_data_;
  Geometry odom_geometry_;
  Geometry map_geometry_;
  sl::Mat odom_left_;
  sl::Mat odom_depth_;
  sl::Mat map_left_;
  sl::Mat map_depth_;
  zed_imu_conversion::Matrix3 camera_from_imu_rotation_{};
  std::size_t imu_gravity_rejected_count_{};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr odom_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr odom_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr odom_display_depth_pub_;
  rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr odom_rgbd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr odom_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr map_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_ack_sub_;

  zed_dual_stream::MapFrameLifecycle map_frame_lifecycle_{kMapOdomAckTimeoutNs};
  std::optional<PendingMapFrame> pending_map_;

  std::size_t grab_count_{};
  std::size_t grab_error_count_{};
  std::size_t duplicate_image_stamp_count_{};
  std::size_t odom_count_{};
  std::size_t map_count_{};
  std::size_t map_queued_count_{};
  std::size_t map_exact_valid_ack_count_{};
  std::size_t map_unacknowledged_drop_count_{};
  std::size_t map_invalid_odom_drop_count_{};
  std::size_t map_due_while_pending_count_{};
  std::size_t map_stamp_conflict_count_{};
  std::size_t map_late_valid_after_timeout_count_{};
  std::size_t map_stamp_tombstone_eviction_count_{};
  std::size_t map_shutdown_censored_count_{};
  std::size_t map_publish_failed_count_{};
  std::size_t invalid_odom_ack_rejected_count_{};
  std::size_t unique_valid_odom_ack_count_{};
  std::size_t duplicate_valid_odom_ack_count_{};
  std::size_t regressed_valid_odom_ack_count_{};
  std::uint64_t last_odom_ack_stamp_ns_{};
  std::uint64_t map_planned_sequence_{};
  std::uint64_t map_event_sequence_{};
  zed_capture_telemetry::ImageRotationWindow image_rotation_window_;
  zed_capture_telemetry::ScalarWindow low_path_ms_window_;
  zed_capture_telemetry::ScalarWindow image_to_low_publish_ms_window_;
  zed_capture_telemetry::ScalarWindow image_to_odom_ms_window_;
  double last_odom_tracking_valid_{};
  double last_odom_display_valid_{};
  double last_map_tracking_valid_{};
  std::chrono::steady_clock::time_point report_started_;
  std::uint64_t report_first_image_stamp_ns_{};
  std::uint64_t report_last_image_stamp_ns_{};
  std::uint64_t previous_image_stamp_ns_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ZedDualStreamPublisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok()) {
      executor.spin_some();
      node->capture_and_publish();
      executor.spin_some();
    }
  } catch (const std::exception &error) {
    std::cerr << "ZED dual-stream publisher failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
