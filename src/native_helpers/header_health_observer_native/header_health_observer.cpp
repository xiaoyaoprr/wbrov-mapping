#include "rolling_header_metrics.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <rmw/types.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace whh = wbrov_header_health;
namespace fs = std::filesystem;

namespace
{

constexpr int kExitOk = 0;
constexpr int kExitRuntimeError = 1;
constexpr int kExitGateFailed = 2;
constexpr int kExitWindowNotReady = 3;
constexpr int kExitHup = 129;
constexpr int kExitInterrupt = 130;
constexpr int kExitTerminated = 143;
constexpr std::size_t kRequiredGidBytes = 24U;
static_assert(RMW_GID_STORAGE_SIZE == kRequiredGidBytes, "Humble GID storage must be exactly 24 bytes");

volatile sig_atomic_t g_signal_number = 0;

extern "C" void managedSignalHandler(int signal_number)
{
  if (g_signal_number == 0) {
    g_signal_number = signal_number;
  }
}

std::int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t wallNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::int64_t secondsToNs(double seconds)
{
  const long double scaled = static_cast<long double>(seconds) * 1000000000.0L;
  if (!std::isfinite(seconds) || scaled < 0.0L ||
    scaled > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    throw std::invalid_argument("duration cannot be represented in nanoseconds");
  }
  return static_cast<std::int64_t>(std::llround(scaled));
}

struct Arguments
{
  bool check_only{false};
  bool fail_fast{false};
  double duration_s{60.0};
  double emit_period_s{1.0};
  whh::MetricsConfig metrics;
  std::string odom_topic{"/odom"};
  std::string odom_info_topic{"/odom_info_lite"};
  std::string high_topic{"/zed/mapping/rgb/camera_info"};
  std::string info_topic{"/info"};
  std::string expected_odom_node{"rtabmap_rgbd_odometry"};
  std::string expected_odom_info_node{"rtabmap_rgbd_odometry"};
  std::string expected_high_node{"zed_rgbd_dual_stream_publisher"};
  std::string expected_info_node{"rtabmap"};
  std::string expected_namespace{"/"};
  std::optional<fs::path> jsonl;
  std::optional<fs::path> state_json;
};

double parseFiniteDouble(const std::string & name, const std::string & value)
{
  std::size_t consumed = 0U;
  double result = 0.0;
  try {
    result = std::stod(value, &consumed);
  } catch (const std::exception &) {
    throw std::invalid_argument(name + " requires a finite number");
  }
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument(name + " requires a finite number");
  }
  return result;
}

std::uint32_t parsePositiveUint32(const std::string & name, const std::string & value)
{
  std::size_t consumed = 0U;
  unsigned long parsed = 0UL;
  try {
    parsed = std::stoul(value, &consumed, 10);
  } catch (const std::exception &) {
    throw std::invalid_argument(name + " requires a positive integer");
  }
  if (consumed != value.size() || parsed == 0UL ||
    parsed > static_cast<unsigned long>(std::numeric_limits<std::uint32_t>::max()))
  {
    throw std::invalid_argument(name + " requires a positive uint32");
  }
  return static_cast<std::uint32_t>(parsed);
}

std::string requireValue(int argc, char ** argv, int & index, const std::string & option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(option + " requires a value");
  }
  ++index;
  return argv[index];
}

bool validTopic(const std::string & value)
{
  return value.size() > 1U && value.front() == '/' && value.back() != '/' &&
    value.find_first_of(" \t\r\n") == std::string::npos &&
    value.find("//") == std::string::npos && value.find("..") == std::string::npos;
}

bool validNodeName(const std::string & value)
{
  return !value.empty() && value.find('/') == std::string::npos &&
    value.find_first_of(" \t\r\n") == std::string::npos;
}

void validateArguments(Arguments & args)
{
  args.metrics.validate();
  if (args.duration_s < 0.0 || !std::isfinite(args.duration_s)) {
    throw std::invalid_argument("--duration-s must be finite and nonnegative");
  }
  if (args.duration_s > 0.0 && args.duration_s < args.metrics.window_s) {
    throw std::invalid_argument("positive --duration-s must be at least --window-s");
  }
  if (!std::isfinite(args.emit_period_s) || args.emit_period_s <= 0.0 ||
    args.emit_period_s > args.metrics.window_s)
  {
    throw std::invalid_argument("--emit-period-s must be in (0,window]");
  }
  for (const auto * topic : {&args.odom_topic, &args.odom_info_topic, &args.high_topic, &args.info_topic}) {
    if (!validTopic(*topic)) {
      throw std::invalid_argument("all four topics must be normalized absolute ROS topic names");
    }
  }
  for (const auto * name : {
      &args.expected_odom_node, &args.expected_odom_info_node,
      &args.expected_high_node, &args.expected_info_node})
  {
    if (!validNodeName(*name)) {
      throw std::invalid_argument("expected publisher names must be unqualified node names");
    }
  }
  if (args.expected_namespace.empty() || args.expected_namespace.front() != '/' ||
    args.expected_namespace.find_first_of(" \t\r\n") != std::string::npos ||
    args.expected_namespace.find("//") != std::string::npos ||
    args.expected_namespace.find("..") != std::string::npos)
  {
    throw std::invalid_argument("expected namespace must be a normalized absolute namespace");
  }
  while (args.expected_namespace.size() > 1U && args.expected_namespace.back() == '/') {
    args.expected_namespace.pop_back();
  }
  if (!args.check_only && !args.state_json.has_value()) {
    throw std::invalid_argument("runtime mode requires --state-json for terminal evidence");
  }
  for (const auto * path : {&args.jsonl, &args.state_json}) {
    if (path->has_value() && !(*path)->is_absolute()) {
      throw std::invalid_argument("output paths must be absolute");
    }
  }
}

Arguments parseArguments(int argc, char ** argv)
{
  Arguments args;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--check-only") {
      args.check_only = true;
    } else if (option == "--fail-fast") {
      args.fail_fast = true;
    } else if (option == "--require-integration-proxy") {
      args.metrics.require_integration_proxy = true;
    } else if (option == "--no-require-integration-proxy") {
      args.metrics.require_integration_proxy = false;
    } else if (option == "--duration-s") {
      args.duration_s = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--window-s") {
      args.metrics.window_s = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--emit-period-s") {
      args.emit_period_s = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--match-grace-s") {
      args.metrics.match_grace_s = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--match-settle-s") {
      args.metrics.match_settle_s = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--odom-min-hz") {
      args.metrics.odom_min_hz = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--high-min-hz") {
      args.metrics.high_min_hz = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--coverage-min") {
      args.metrics.coverage_min = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--integration-min-hz") {
      args.metrics.integration_min_hz = parseFiniteDouble(option, requireValue(argc, argv, index, option));
    } else if (option == "--expected-high-width") {
      args.metrics.expected_high_width = parsePositiveUint32(
        option, requireValue(argc, argv, index, option));
    } else if (option == "--expected-high-height") {
      args.metrics.expected_high_height = parsePositiveUint32(
        option, requireValue(argc, argv, index, option));
    } else if (option == "--odom-topic") {
      args.odom_topic = requireValue(argc, argv, index, option);
    } else if (option == "--odom-info-topic") {
      args.odom_info_topic = requireValue(argc, argv, index, option);
    } else if (option == "--high-camera-info-topic") {
      args.high_topic = requireValue(argc, argv, index, option);
    } else if (option == "--rtabmap-info-topic") {
      args.info_topic = requireValue(argc, argv, index, option);
    } else if (option == "--expected-odom-node") {
      args.expected_odom_node = requireValue(argc, argv, index, option);
    } else if (option == "--expected-odom-info-node") {
      args.expected_odom_info_node = requireValue(argc, argv, index, option);
    } else if (option == "--expected-high-node") {
      args.expected_high_node = requireValue(argc, argv, index, option);
    } else if (option == "--expected-info-node") {
      args.expected_info_node = requireValue(argc, argv, index, option);
    } else if (option == "--expected-node-namespace") {
      args.expected_namespace = requireValue(argc, argv, index, option);
    } else if (option == "--jsonl") {
      args.jsonl = fs::path(requireValue(argc, argv, index, option));
    } else if (option == "--state-json") {
      args.state_json = fs::path(requireValue(argc, argv, index, option));
    } else if (option == "--help" || option == "-h") {
      std::cout <<
        "usage: header_health_observer [--check-only] --state-json ABSOLUTE_PATH "
        "[--jsonl ABSOLUTE_NEW_PATH] [rolling thresholds/topics/expected publisher options]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + option);
    }
  }
  validateArguments(args);
  return args;
}

void writeAll(int descriptor, const std::string & data)
{
  std::size_t offset = 0U;
  while (offset < data.size()) {
    const ssize_t written = ::write(descriptor, data.data() + offset, data.size() - offset);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error("write failed: " + std::string(std::strerror(errno)));
    }
    if (written == 0) {
      throw std::runtime_error("zero-length write");
    }
    offset += static_cast<std::size_t>(written);
  }
}

class EventWriter
{
public:
  EventWriter(const std::optional<fs::path> & jsonl, const fs::path & state)
  {
    openStateDirectory(state);
    if (jsonl.has_value()) {
      openJsonl(*jsonl);
    }
  }

  ~EventWriter()
  {
    if (jsonl_fd_ >= 0) {
      ::close(jsonl_fd_);
    }
    if (state_dir_fd_ >= 0) {
      ::close(state_dir_fd_);
    }
  }

  EventWriter(const EventWriter &) = delete;
  EventWriter & operator=(const EventWriter &) = delete;

  void emit(const std::string & event)
  {
    const std::string encoded = event + "\n";
    std::cout << encoded;
    std::cout.flush();
    if (!std::cout) {
      throw std::runtime_error("stdout write failed");
    }
    if (jsonl_fd_ >= 0) {
      writeAll(jsonl_fd_, encoded);
      if (::fsync(jsonl_fd_) != 0) {
        throw std::runtime_error("jsonl fsync failed: " + std::string(std::strerror(errno)));
      }
    }
    writeState(encoded);
  }

private:
  static void requireSafeExistingTarget(int directory_fd, const std::string & name)
  {
    struct stat info {};
    if (::fstatat(directory_fd, name.c_str(), &info, AT_SYMLINK_NOFOLLOW) == 0) {
      if (!S_ISREG(info.st_mode) || info.st_uid != ::geteuid() || info.st_nlink != 1 ||
        (info.st_mode & 07777) != 0600)
      {
        throw std::runtime_error("unsafe existing state target");
      }
    } else if (errno != ENOENT) {
      throw std::runtime_error("state target stat failed: " + std::string(std::strerror(errno)));
    }
  }

  static std::pair<int, std::string> openParent(const fs::path & output)
  {
    const fs::path parent = output.parent_path();
    const std::string name = output.filename().string();
    if (parent.empty() || name.empty() || name == "." || name == ".." || name.find('/') != std::string::npos) {
      throw std::invalid_argument("invalid output path");
    }
    const int fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
      throw std::runtime_error("cannot open output parent: " + std::string(std::strerror(errno)));
    }
    struct stat info {};
    if (::fstat(fd, &info) != 0 || !S_ISDIR(info.st_mode) || info.st_uid != ::geteuid()) {
      ::close(fd);
      throw std::runtime_error("output parent is not a current-user directory");
    }
    return {fd, name};
  }

  void openStateDirectory(const fs::path & state)
  {
    const auto opened = openParent(state);
    state_dir_fd_ = opened.first;
    state_name_ = opened.second;
    requireSafeExistingTarget(state_dir_fd_, state_name_);
  }

  void openJsonl(const fs::path & jsonl)
  {
    const auto opened = openParent(jsonl);
    const int directory_fd = opened.first;
    const std::string name = opened.second;
    const int descriptor = ::openat(
      directory_fd, name.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    const int saved_errno = errno;
    ::close(directory_fd);
    if (descriptor < 0) {
      throw std::runtime_error("cannot create exclusive jsonl: " + std::string(std::strerror(saved_errno)));
    }
    struct stat info {};
    if (::fstat(descriptor, &info) != 0 || !S_ISREG(info.st_mode) || info.st_uid != ::geteuid() ||
      info.st_nlink != 1 || (info.st_mode & 07777) != 0600)
    {
      ::close(descriptor);
      throw std::runtime_error("new jsonl identity/mode check failed");
    }
    jsonl_fd_ = descriptor;
  }

  void writeState(const std::string & encoded)
  {
    requireSafeExistingTarget(state_dir_fd_, state_name_);
    const std::string temporary = "." + state_name_ + ".tmp." +
      std::to_string(::getpid()) + "." + std::to_string(++temporary_sequence_);
    int descriptor = ::openat(
      state_dir_fd_, temporary.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (descriptor < 0) {
      throw std::runtime_error("cannot create state temporary: " + std::string(std::strerror(errno)));
    }
    bool renamed = false;
    try {
      if (::fchmod(descriptor, 0600) != 0) {
        throw std::runtime_error("state temporary chmod failed");
      }
      writeAll(descriptor, encoded);
      if (::fsync(descriptor) != 0) {
        throw std::runtime_error("state temporary fsync failed: " + std::string(std::strerror(errno)));
      }
      struct stat info {};
      if (::fstat(descriptor, &info) != 0 || !S_ISREG(info.st_mode) || info.st_uid != ::geteuid() ||
        info.st_nlink != 1 || (info.st_mode & 07777) != 0600)
      {
        throw std::runtime_error("state temporary identity/mode check failed");
      }
      if (::close(descriptor) != 0) {
        descriptor = -1;
        throw std::runtime_error("state temporary close failed");
      }
      descriptor = -1;
      if (::renameat(state_dir_fd_, temporary.c_str(), state_dir_fd_, state_name_.c_str()) != 0) {
        throw std::runtime_error("state atomic rename failed: " + std::string(std::strerror(errno)));
      }
      renamed = true;
      if (::fsync(state_dir_fd_) != 0) {
        throw std::runtime_error("state directory fsync failed: " + std::string(std::strerror(errno)));
      }
      requireSafeExistingTarget(state_dir_fd_, state_name_);
    } catch (...) {
      if (descriptor >= 0) {
        ::close(descriptor);
      }
      if (!renamed) {
        ::unlinkat(state_dir_fd_, temporary.c_str(), 0);
      }
      throw;
    }
  }

  int state_dir_fd_{-1};
  std::string state_name_;
  int jsonl_fd_{-1};
  std::uint64_t temporary_sequence_{0U};
};

enum class Role : std::size_t {Odom = 0U, OdomInfo = 1U, High = 2U, Info = 3U};

const char * roleName(Role role)
{
  switch (role) {
    case Role::Odom: return "odom";
    case Role::OdomInfo: return "odom_info_lite";
    case Role::High: return "high_camera_info";
    case Role::Info: return "rtabmap_info";
  }
  return "unknown";
}

struct SourceBinding
{
  Role role{};
  std::string topic;
  std::string node_name;
  std::string node_namespace;
  bool authenticated{false};
  std::array<std::uint8_t, kRequiredGidBytes> gid{};
};

bool gidNonzero(const std::array<std::uint8_t, kRequiredGidBytes> & gid)
{
  return std::any_of(gid.begin(), gid.end(), [](std::uint8_t value) {return value != 0U;});
}

std::string gidHex(const std::array<std::uint8_t, kRequiredGidBytes> & gid)
{
  static constexpr char kHex[] = "0123456789abcdef";
  std::string result;
  result.reserve(kRequiredGidBytes * 2U);
  for (const std::uint8_t value : gid) {
    result.push_back(kHex[(value >> 4U) & 0x0fU]);
    result.push_back(kHex[value & 0x0fU]);
  }
  return result;
}

std::array<std::uint8_t, kRequiredGidBytes> messageGid(const rclcpp::MessageInfo & message_info)
{
  const rmw_gid_t & raw = message_info.get_rmw_message_info().publisher_gid;
  std::array<std::uint8_t, kRequiredGidBytes> result{};
  std::copy(raw.data, raw.data + kRequiredGidBytes, result.begin());
  return result;
}

std::array<std::uint8_t, kRequiredGidBytes> endpointGid(
  const rclcpp::TopicEndpointInfo & endpoint)
{
  const auto & raw = endpoint.endpoint_gid();
  if (raw.size() != kRequiredGidBytes) {
    throw std::runtime_error("graph endpoint GID is not 24 bytes");
  }
  std::array<std::uint8_t, kRequiredGidBytes> result{};
  std::copy(raw.begin(), raw.end(), result.begin());
  return result;
}

std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.nanosec >= 1000000000U) {
    return -1;
  }
  const std::int64_t seconds = static_cast<std::int64_t>(stamp.sec);
  if (seconds < 0 || seconds > (std::numeric_limits<std::int64_t>::max() /
    whh::kNsPerSecond))
  {
    return -1;
  }
  const std::int64_t result = seconds * whh::kNsPerSecond +
    static_cast<std::int64_t>(stamp.nanosec);
  return result > 0 ? result : -1;
}

class HeaderObserverNode : public rclcpp::Node
{
public:
  HeaderObserverNode(const Arguments & args, whh::RollingHeaderMetrics & metrics)
  : Node(
      "header_health_observer_" + std::to_string(::getpid()),
      observerNodeOptions()),
    metrics_(metrics)
  {
    sources_[0] = {Role::Odom, args.odom_topic, args.expected_odom_node, args.expected_namespace};
    sources_[1] = {
      Role::OdomInfo, args.odom_info_topic, args.expected_odom_info_node, args.expected_namespace};
    sources_[2] = {Role::High, args.high_topic, args.expected_high_node, args.expected_namespace};
    sources_[3] = {Role::Info, args.info_topic, args.expected_info_node, args.expected_namespace};
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    best_effort_qos.best_effort().durability_volatile();
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      args.odom_topic, best_effort_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        onOdom(*message, info);
      });
    odom_info_subscription_ = create_subscription<rtabmap_msgs::msg::OdomInfo>(
      args.odom_info_topic, best_effort_qos,
      [this](rtabmap_msgs::msg::OdomInfo::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        onOdomInfo(*message, info);
      });
    high_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      args.high_topic, best_effort_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        onHigh(*message, info);
      });
    info_subscription_ = create_subscription<rtabmap_msgs::msg::Info>(
      args.info_topic, best_effort_qos,
      [this](rtabmap_msgs::msg::Info::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        onInfo(*message, info);
      });
  }

  bool hasFailure() const noexcept {return !failure_.empty();}
  const std::string & failure() const noexcept {return failure_;}

  bool allAuthenticated() const noexcept
  {
    return !hasFailure() && std::all_of(
      sources_.begin(), sources_.end(), [](const SourceBinding & source) {
        return source.authenticated;
      });
  }

  void verifyFrozenSources()
  {
    if (hasFailure()) {
      return;
    }
    for (auto & source : sources_) {
      if (!source.authenticated) {
        continue;
      }
      const auto endpoints = get_publishers_info_by_topic(source.topic);
      if (endpoints.size() != 1U) {
        failSource(
          "publisher_set_changed:" + std::string(roleName(source.role)) + ":" +
          std::to_string(endpoints.size()));
        return;
      }
      const auto & endpoint = endpoints.front();
      if (endpoint.node_name() != source.node_name ||
        endpoint.node_namespace() != source.node_namespace)
      {
        failSource("publisher_identity_changed:" + std::string(roleName(source.role)));
        return;
      }
      const auto current_gid = endpointGid(endpoint);
      if (!gidNonzero(current_gid) || current_gid != source.gid) {
        failSource("publisher_gid_changed:" + std::string(roleName(source.role)));
        return;
      }
    }
  }

  std::string sourceStateJson() const
  {
    std::vector<std::pair<std::string, std::string>> source_fields;
    for (const auto & source : sources_) {
      source_fields.emplace_back(
        roleName(source.role),
        whh::jsonObject({
          {"topic", whh::jsonEscape(source.topic)},
          {"node_name", whh::jsonEscape(source.node_name)},
          {"node_namespace", whh::jsonEscape(source.node_namespace)},
          {"publisher_gid", source.authenticated ? whh::jsonEscape(gidHex(source.gid)) : "null"},
          {"publisher_gid_bytes", source.authenticated ? "24" : "null"},
          {"authenticated", whh::jsonBool(source.authenticated)},
        }));
    }
    return whh::jsonObject({
      {"all_authenticated", whh::jsonBool(allAuthenticated())},
      {"failure", hasFailure() ? whh::jsonEscape(failure_) : "null"},
      {"gid_contract_bytes", "24"},
      {"sources", whh::jsonObject(source_fields)},
    });
  }

private:
  static rclcpp::NodeOptions observerNodeOptions()
  {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.enable_rosout(false);
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    return options;
  }

  SourceBinding & source(Role role) {return sources_[static_cast<std::size_t>(role)];}

  void failSource(const std::string & reason)
  {
    if (failure_.empty()) {
      failure_ = reason;
    }
  }

  bool authenticate(Role role, const rclcpp::MessageInfo & message_info)
  {
    if (hasFailure()) {
      return false;
    }
    SourceBinding & binding = source(role);
    std::vector<rclcpp::TopicEndpointInfo> endpoints;
    try {
      endpoints = get_publishers_info_by_topic(binding.topic);
    } catch (const std::exception & error) {
      failSource(
        "publisher_graph_error:" + std::string(roleName(role)) + ":" + error.what());
      return false;
    }
    if (endpoints.size() != 1U) {
      failSource(
        "publisher_set_not_singleton:" + std::string(roleName(role)) + ":" +
        std::to_string(endpoints.size()));
      return false;
    }
    const auto & endpoint = endpoints.front();
    const auto endpoint_gid = endpointGid(endpoint);
    const auto incoming_gid = messageGid(message_info);
    const bool pending_name = endpoint.node_name() == "_NODE_NAME_UNKNOWN_";
    const bool pending_namespace = endpoint.node_namespace() == "_NODE_NAMESPACE_UNKNOWN_";
    if (!binding.authenticated && gidNonzero(endpoint_gid) && gidNonzero(incoming_gid) &&
      endpoint_gid == incoming_gid && (pending_name || pending_namespace) &&
      (pending_name || endpoint.node_name() == binding.node_name) &&
      (pending_namespace || endpoint.node_namespace() == binding.node_namespace))
    {
      return false;  // Unresolved discovery is not an authenticated observation.
    }
    if (endpoint.node_name() != binding.node_name ||
      endpoint.node_namespace() != binding.node_namespace)
    {
      failSource("publisher_identity_mismatch:" + std::string(roleName(role)));
      return false;
    }
    if (!gidNonzero(endpoint_gid) || !gidNonzero(incoming_gid)) {
      failSource("publisher_gid_zero_or_wrong_size:" + std::string(roleName(role)));
      return false;
    }
    if (endpoint_gid != incoming_gid) {
      failSource("publisher_gid_mismatch:" + std::string(roleName(role)));
      return false;
    }
    if (binding.authenticated && binding.gid != endpoint_gid) {
      failSource("publisher_gid_changed:" + std::string(roleName(role)));
      return false;
    }
    binding.authenticated = true;
    binding.gid = endpoint_gid;
    return true;
  }

  void onOdom(const nav_msgs::msg::Odometry & message, const rclcpp::MessageInfo & info)
  {
    if (!authenticate(Role::Odom, info)) {
      return;
    }
    const auto & pose = message.pose.pose;
    metrics_.addOdom(
      steadyNowNs(), stampNs(message.header.stamp),
      {pose.position.x, pose.position.y, pose.position.z},
      {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w});
  }

  void onOdomInfo(const rtabmap_msgs::msg::OdomInfo & message, const rclcpp::MessageInfo & info)
  {
    if (authenticate(Role::OdomInfo, info)) {
      metrics_.addOdomInfo(steadyNowNs(), stampNs(message.header.stamp), message.lost);
    }
  }

  void onHigh(const sensor_msgs::msg::CameraInfo & message, const rclcpp::MessageInfo & info)
  {
    if (!authenticate(Role::High, info)) {
      return;
    }
    std::array<double, 9> matrix{};
    std::copy(message.k.begin(), message.k.end(), matrix.begin());
    metrics_.addHighCameraInfo(
      steadyNowNs(), stampNs(message.header.stamp), message.width, message.height, matrix);
  }

  void onInfo(const rtabmap_msgs::msg::Info & message, const rclcpp::MessageInfo & info)
  {
    if (authenticate(Role::Info, info)) {
      metrics_.addRtabmapInfo(steadyNowNs(), stampNs(message.header.stamp));
    }
  }

  whh::RollingHeaderMetrics & metrics_;
  std::array<SourceBinding, 4> sources_;
  std::string failure_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<rtabmap_msgs::msg::OdomInfo>::SharedPtr odom_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr high_subscription_;
  rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr info_subscription_;
};

std::string topicsJson(const Arguments & args)
{
  return whh::jsonObject({
    {"odom", whh::jsonEscape(args.odom_topic)},
    {"odom_info_lite", whh::jsonEscape(args.odom_info_topic)},
    {"high_camera_info", whh::jsonEscape(args.high_topic)},
    {"rtabmap_info", whh::jsonEscape(args.info_topic)},
  });
}

std::string expectedPublishersJson(const Arguments & args)
{
  const auto source = [&](const std::string & topic, const std::string & node) {
      return whh::jsonObject({
        {"topic", whh::jsonEscape(topic)},
        {"node_name", whh::jsonEscape(node)},
        {"node_namespace", whh::jsonEscape(args.expected_namespace)},
      });
    };
  return whh::jsonObject({
    {"odom", source(args.odom_topic, args.expected_odom_node)},
    {"odom_info_lite", source(args.odom_info_topic, args.expected_odom_info_node)},
    {"high_camera_info", source(args.high_topic, args.expected_high_node)},
    {"rtabmap_info", source(args.info_topic, args.expected_info_node)},
  });
}

std::string eventEnvelope(
  const std::string & kind,
  std::uint64_t sequence,
  const whh::SnapshotResult & snapshot,
  const Arguments & args,
  std::int64_t now_ns,
  const std::string & publisher_sources,
  const std::optional<std::string> & exit_reason = std::nullopt,
  const std::optional<int> & exit_code = std::nullopt)
{
  std::vector<std::pair<std::string, std::string>> fields{
    {"schema_version", "2"},
    {"kind", whh::jsonEscape(kind)},
    {"sequence", whh::jsonUInt(sequence)},
    {"monotonic_ns", whh::jsonInt(now_ns)},
    {"wall_time_ns", whh::jsonInt(wallNowNs())},
    {"topics", topicsJson(args)},
    {"snapshot", snapshot.json},
    {"observer_effects", whh::jsonObject({
        {"publishes", "false"},
        {"services", "false"},
        {"timestamp_modification", "false"},
        {"stored_full_messages_or_images", "false"},
      })},
    {"publisher_sources", publisher_sources},
  };
  if (exit_reason.has_value()) {
    fields.emplace_back("exit_reason", whh::jsonEscape(*exit_reason));
  }
  if (exit_code.has_value()) {
    fields.emplace_back("exit_code", std::to_string(*exit_code));
  }
  return whh::jsonObject(fields);
}

std::string checkOnlyJson(const Arguments & args)
{
  return whh::jsonObject({
    {"schema_version", "2"},
    {"kind", whh::jsonEscape("check_only")},
    {"status", whh::jsonEscape("passed")},
    {"ros_context_started", "false"},
    {"device_started", "false"},
    {"topics", topicsJson(args)},
    {"expected_publishers", expectedPublishersJson(args)},
    {"publisher_authentication", whh::jsonObject({
        {"message_info_gid_bytes", "24"},
        {"endpoint_gid_bytes", "24"},
        {"nonzero_required", "true"},
        {"unique_endpoint_required", "true"},
        {"exact_gid_match_required", "true"},
        {"endpoint_change_is_terminal", "true"},
      })},
    {"thresholds", whh::jsonObject({
        {"window_s", whh::jsonDouble(args.metrics.window_s)},
        {"valid_unique_odom_hz", whh::jsonDouble(args.metrics.odom_min_hz)},
        {"valid_unique_high_camera_info_hz", whh::jsonDouble(args.metrics.high_min_hz)},
        {"high_to_valid_odom_exact_coverage", whh::jsonDouble(args.metrics.coverage_min)},
        {"high_integrated_info_proxy_hz", whh::jsonDouble(args.metrics.integration_min_hz)},
        {"integration_proxy_required", whh::jsonBool(args.metrics.require_integration_proxy)},
      })},
    {"runtime_event_kinds", whh::jsonArray({
        whh::jsonEscape("warming"), whh::jsonEscape("ready"),
        whh::jsonEscape("rolling"), whh::jsonEscape("final")})},
    {"exit_codes", whh::jsonObject({
        {"0", whh::jsonEscape("ready final window passed or check-only passed")},
        {"1", whh::jsonEscape("runtime/output/publisher authentication failure")},
        {"2", whh::jsonEscape("ready gate failure")},
        {"3", whh::jsonEscape("duration ended before rolling window ready")},
        {"129", whh::jsonEscape("SIGHUP")},
        {"130", whh::jsonEscape("SIGINT")},
        {"143", whh::jsonEscape("SIGTERM")},
      })},
  });
}

struct SignalGuard
{
  std::array<struct sigaction, 3> previous{};
  std::array<int, 3> numbers{SIGHUP, SIGINT, SIGTERM};
  std::size_t installed_count{0U};

  void install()
  {
    struct sigaction action {};
    action.sa_handler = managedSignalHandler;
    ::sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    for (std::size_t index = 0; index < numbers.size(); ++index) {
      if (::sigaction(numbers[index], &action, &previous[index]) != 0) {
        throw std::runtime_error("sigaction install failed: " + std::string(std::strerror(errno)));
      }
      ++installed_count;
    }
  }

  ~SignalGuard()
  {
    for (std::size_t index = 0; index < installed_count; ++index) {
      ::sigaction(numbers[index], &previous[index], nullptr);
    }
  }
};

std::pair<std::string, int> signalExit(sig_atomic_t signal_number)
{
  if (signal_number == SIGHUP) {
    return {"SIGHUP", kExitHup};
  }
  if (signal_number == SIGINT) {
    return {"SIGINT", kExitInterrupt};
  }
  if (signal_number == SIGTERM) {
    return {"SIGTERM", kExitTerminated};
  }
  return {"unknown_signal", kExitRuntimeError};
}

int runObserver(const Arguments & args)
{
  g_signal_number = 0;
  SignalGuard signals;
  signals.install();
  EventWriter writer(args.jsonl, *args.state_json);
  const std::int64_t start_ns = steadyNowNs();
  whh::RollingHeaderMetrics metrics(start_ns, args.metrics);
  int ros_argc = 1;
  char program_name[] = "header_health_observer";
  char * ros_argv[] = {program_name, nullptr};
  rclcpp::InitOptions init_options;
  rclcpp::init(
    ros_argc, ros_argv, init_options, rclcpp::SignalHandlerOptions::None);

  int exit_code = kExitRuntimeError;
  std::string exit_reason = "runtime_error";
  std::uint64_t sequence = 0U;
  bool ready_emitted = false;
  std::shared_ptr<HeaderObserverNode> node;
  try {
    node = std::make_shared<HeaderObserverNode>(args, metrics);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::int64_t next_emit_ns = start_ns;
    std::int64_t next_graph_check_ns = start_ns;
    std::optional<std::int64_t> duration_ns;
    if (args.duration_s != 0.0) {
      duration_ns.emplace(secondsToNs(args.duration_s));
    }
    const std::int64_t emit_period_ns = secondsToNs(args.emit_period_s);
    while (rclcpp::ok()) {
      executor.spin_some();
      const std::int64_t now_ns = steadyNowNs();
      if (g_signal_number != 0) {
        // The launcher intentionally stops this observer while all four data
        // sources are still alive.  Re-authenticate the frozen graph before
        // accepting a normal signal terminal; otherwise a source teardown or
        // swap could be hidden behind an apparently clean exit 130.
        node->verifyFrozenSources();
        if (node->hasFailure()) {
          exit_reason = "publisher_authentication_failed_before_signal_terminal:" +
            node->failure();
          exit_code = kExitRuntimeError;
        } else {
          const auto terminal = signalExit(g_signal_number);
          exit_reason = terminal.first;
          exit_code = terminal.second;
        }
        break;
      }
      if (now_ns >= next_graph_check_ns) {
        node->verifyFrozenSources();
        next_graph_check_ns = now_ns + 100000000LL;
      }
      if (node->hasFailure()) {
        exit_reason = "publisher_authentication_failed:" + node->failure();
        exit_code = kExitRuntimeError;
        break;
      }
      if (now_ns >= next_emit_ns) {
        const auto snapshot = metrics.snapshot(now_ns);
        std::string kind;
        if (snapshot.ready && !ready_emitted) {
          kind = "ready";
          ready_emitted = true;
        } else {
          kind = snapshot.ready ? "rolling" : "warming";
        }
        writer.emit(eventEnvelope(
            kind, sequence++, snapshot, args, now_ns, node->sourceStateJson()));
        next_emit_ns = now_ns + emit_period_ns;
        if (args.fail_fast && snapshot.ready && snapshot.candidate_pass != std::optional<bool>(true)) {
          exit_reason = "gate_failed_fail_fast";
          exit_code = kExitGateFailed;
          break;
        }
      }
      if (duration_ns.has_value() && now_ns - start_ns >= *duration_ns) {
        const auto snapshot = metrics.snapshot(now_ns);
        if (!snapshot.ready) {
          exit_reason = "duration_complete_window_not_ready";
          exit_code = kExitWindowNotReady;
        } else if (snapshot.candidate_pass == std::optional<bool>(true)) {
          exit_reason = "duration_complete_passed";
          exit_code = kExitOk;
        } else {
          exit_reason = "duration_complete_gate_failed";
          exit_code = kExitGateFailed;
        }
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    executor.remove_node(node);
  } catch (const std::exception & error) {
    exit_reason = "runtime_error:" + std::string(error.what());
    exit_code = kExitRuntimeError;
  }

  const std::int64_t final_now_ns = steadyNowNs();
  const auto final_snapshot = metrics.snapshot(final_now_ns);
  const std::string source_state = node ? node->sourceStateJson() : whh::jsonObject({});
  if (node && !node->allAuthenticated() && exit_code == kExitOk) {
    exit_reason = "publisher_authentication_incomplete";
    exit_code = kExitRuntimeError;
  }
  writer.emit(eventEnvelope(
      "final", sequence, final_snapshot, args, final_now_ns, source_state,
      exit_reason, exit_code));
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments args = parseArguments(argc, argv);
    if (args.check_only) {
      std::cout << checkOnlyJson(args) << '\n';
      return kExitOk;
    }
    return runObserver(args);
  } catch (const std::exception & error) {
    std::cerr << whh::jsonObject({
      {"schema_version", "2"},
      {"kind", whh::jsonEscape("fatal")},
      {"error", whh::jsonEscape(error.what())},
    }) << '\n';
    return kExitRuntimeError;
  }
}
