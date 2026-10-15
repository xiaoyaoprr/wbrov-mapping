#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using SteadyClock = std::chrono::steady_clock;

enum class Mode
{
  Positive,
  SecondPublisher,
  GidSwap,
  Disappear,
};

struct Arguments
{
  Mode mode{Mode::Positive};
  double duration_s{8.0};
  double fault_after_s{2.0};
  double publish_hz{50.0};
  std::uint32_t high_divisor{4U};
  std::optional<std::filesystem::path> fault_trigger_file;
};

double parseFinite(const std::string & option, const std::string & value)
{
  std::size_t consumed = 0U;
  double parsed = 0.0;
  try {
    parsed = std::stod(value, &consumed);
  } catch (const std::exception &) {
    throw std::invalid_argument(option + " requires a finite number");
  }
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(option + " requires a finite number");
  }
  return parsed;
}

std::string requireValue(int argc, char ** argv, int & index, const std::string & option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(option + " requires a value");
  }
  ++index;
  return argv[index];
}

Arguments parseArguments(int argc, char ** argv)
{
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--mode") {
      const std::string value = requireValue(argc, argv, index, option);
      if (value == "positive") {
        result.mode = Mode::Positive;
      } else if (value == "second_publisher") {
        result.mode = Mode::SecondPublisher;
      } else if (value == "gid_swap") {
        result.mode = Mode::GidSwap;
      } else if (value == "disappear") {
        result.mode = Mode::Disappear;
      } else {
        throw std::invalid_argument("unknown fixture mode: " + value);
      }
    } else if (option == "--duration-s") {
      result.duration_s = parseFinite(option, requireValue(argc, argv, index, option));
    } else if (option == "--fault-after-s") {
      result.fault_after_s = parseFinite(option, requireValue(argc, argv, index, option));
    } else if (option == "--publish-hz") {
      result.publish_hz = parseFinite(option, requireValue(argc, argv, index, option));
    } else if (option == "--high-divisor") {
      const std::string value = requireValue(argc, argv, index, option);
      std::size_t consumed = 0U;
      const unsigned long parsed = std::stoul(value, &consumed, 10);
      if (consumed != value.size() || parsed == 0UL ||
        parsed > static_cast<unsigned long>(std::numeric_limits<std::uint32_t>::max()))
      {
        throw std::invalid_argument("--high-divisor requires a positive uint32");
      }
      result.high_divisor = static_cast<std::uint32_t>(parsed);
    } else if (option == "--fault-trigger-file") {
      result.fault_trigger_file =
        std::filesystem::path(requireValue(argc, argv, index, option));
    } else if (option == "--help" || option == "-h") {
      std::cout << "usage: header_health_fixture --mode "
        "positive|second_publisher|gid_swap|disappear "
        "[--duration-s S] [--fault-after-s S] [--publish-hz HZ] "
        "[--high-divisor N] [--fault-trigger-file ABSOLUTE_PATH]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + option);
    }
  }
  if (result.duration_s <= 0.0 || result.fault_after_s <= 0.0 ||
    result.publish_hz <= 0.0 || result.fault_after_s >= result.duration_s)
  {
    throw std::invalid_argument("fixture durations/rate are invalid");
  }
  if (result.fault_trigger_file.has_value() && !result.fault_trigger_file->is_absolute()) {
    throw std::invalid_argument("--fault-trigger-file must be absolute");
  }
  return result;
}

rclcpp::NodeOptions nodeOptions()
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.enable_rosout(false);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  return options;
}

builtin_interfaces::msg::Time makeStamp(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

const char * modeName(Mode mode)
{
  switch (mode) {
    case Mode::Positive: return "positive";
    case Mode::SecondPublisher: return "second_publisher";
    case Mode::GidSwap: return "gid_swap";
    case Mode::Disappear: return "disappear";
  }
  return "unknown";
}

class Fixture
{
public:
  explicit Fixture(const Arguments & args)
  : args_(args),
    odom_node_(std::make_shared<rclcpp::Node>("rtabmap_rgbd_odometry", nodeOptions())),
    high_node_(std::make_shared<rclcpp::Node>("zed_rgbd_dual_stream_publisher", nodeOptions())),
    info_node_(std::make_shared<rclcpp::Node>("rtabmap", nodeOptions())),
    intruder_node_(std::make_shared<rclcpp::Node>("intruder_odom_source", nodeOptions()))
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100));
    qos.best_effort().durability_volatile();
    qos_ = qos;
    odom_publisher_ = odom_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", qos_);
    odom_info_publisher_ =
      odom_node_->create_publisher<rtabmap_msgs::msg::OdomInfo>("/odom_info_lite", qos_);
    high_publisher_ = high_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/zed/mapping/rgb/camera_info", qos_);
    info_publisher_ = info_node_->create_publisher<rtabmap_msgs::msg::Info>("/info", qos_);
    executor_.add_node(odom_node_);
    executor_.add_node(high_node_);
    executor_.add_node(info_node_);
    executor_.add_node(intruder_node_);
  }

  void run()
  {
    const auto start = SteadyClock::now();
    const auto period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / args_.publish_hz));
    auto next_publish = start;
    while (rclcpp::ok()) {
      executor_.spin_some();
      const auto now = SteadyClock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      const bool trigger_present = args_.fault_trigger_file.has_value() &&
        std::filesystem::exists(*args_.fault_trigger_file);
      const bool timed_fault = !args_.fault_trigger_file.has_value() &&
        elapsed >= args_.fault_after_s;
      if (args_.mode != Mode::Positive && !fault_injected_ && (trigger_present || timed_fault)) {
        injectFault();
      }
      if (elapsed >= args_.duration_s) {
        break;
      }
      if (now >= next_publish) {
        publishOnce();
        next_publish += period;
        if (next_publish < now - period) {
          next_publish = now + period;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "{\"fixture_final\":true,\"mode\":\"" << modeName(args_.mode)
              << "\",\"fault_injected\":" << (fault_injected_ ? "true" : "false")
              << ",\"published_ticks\":" << tick_ << "}\n";
  }

private:
  void publishOnce()
  {
    const std::int64_t stamp_ns = base_stamp_ns_ +
      static_cast<std::int64_t>(tick_) * 20000000LL;
    const auto stamp = makeStamp(stamp_ns);

    if (odom_publisher_) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.orientation.w = 1.0;
      odom_publisher_->publish(odom);
    }
    if (intruder_publisher_) {
      nav_msgs::msg::Odometry intruder;
      intruder.header.stamp = stamp;
      intruder.header.frame_id = "intruder";
      intruder.child_frame_id = "base_link";
      intruder.pose.pose.orientation.w = 1.0;
      intruder_publisher_->publish(intruder);
    }

    rtabmap_msgs::msg::OdomInfo odom_info;
    odom_info.header.stamp = stamp;
    odom_info.lost = false;
    odom_info_publisher_->publish(odom_info);

    if (tick_ % args_.high_divisor == 0U) {
      sensor_msgs::msg::CameraInfo camera_info;
      camera_info.header.stamp = stamp;
      camera_info.header.frame_id = "zed_left_camera_optical_frame";
      camera_info.width = 1920U;
      camera_info.height = 1200U;
      camera_info.k[0] = 1000.0;
      camera_info.k[2] = 960.0;
      camera_info.k[4] = 1000.0;
      camera_info.k[5] = 600.0;
      camera_info.k[8] = 1.0;
      high_publisher_->publish(camera_info);

      rtabmap_msgs::msg::Info info;
      info.header.stamp = stamp;
      info_publisher_->publish(info);
    }
    ++tick_;
  }

  void injectFault()
  {
    fault_injected_ = true;
    if (args_.mode == Mode::SecondPublisher) {
      intruder_publisher_ =
        intruder_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", qos_);
    } else if (args_.mode == Mode::GidSwap) {
      odom_publisher_.reset();
      odom_publisher_ =
        odom_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", qos_);
    } else if (args_.mode == Mode::Disappear) {
      odom_publisher_.reset();
    }
    std::cerr << "fixture_fault_injected=" << modeName(args_.mode) << '\n';
  }

  Arguments args_;
  rclcpp::QoS qos_{rclcpp::KeepLast(100)};
  std::shared_ptr<rclcpp::Node> odom_node_;
  std::shared_ptr<rclcpp::Node> high_node_;
  std::shared_ptr<rclcpp::Node> info_node_;
  std::shared_ptr<rclcpp::Node> intruder_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odom_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr high_publisher_;
  rclcpp::Publisher<rtabmap_msgs::msg::Info>::SharedPtr info_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr intruder_publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  bool fault_injected_{false};
  std::uint64_t tick_{0U};
  const std::int64_t base_stamp_ns_{1700000000000000000LL};
};

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments args = parseArguments(argc, argv);
    int ros_argc = 1;
    char program_name[] = "header_health_fixture";
    char * ros_argv[] = {program_name, nullptr};
    rclcpp::init(ros_argc, ros_argv);
    Fixture fixture(args);
    fixture.run();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "fixture fatal: " << error.what() << '\n';
    return 1;
  }
}
