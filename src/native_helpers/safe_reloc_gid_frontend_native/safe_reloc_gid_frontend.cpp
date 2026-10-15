#include "safe_reloc_ipc.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <std_msgs/msg/string.hpp>

#include <rmw/types.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ipc = wbrov_safe_reloc_ipc;
namespace fs = std::filesystem;

namespace
{

constexpr std::size_t kGidBytes = 24U;
static_assert(RMW_GID_STORAGE_SIZE == kGidBytes, "Humble publisher GID must be 24 bytes");

volatile sig_atomic_t g_signal_number = 0;

extern "C" void managedSignalHandler(int number)
{
  if (g_signal_number == 0) {
    g_signal_number = number;
  }
}

std::int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Arguments
{
  bool check_only{false};
  fs::path socket_path;
  fs::path state_path;
  std::string session_token;
  fs::path expected_engine_identity_file;
  std::string metadata_topic{"/apriltag/pose_candidate_metadata"};
  std::string info_topic{"/info"};
  std::string odom_topic{"/odom_info_lite"};
  std::string core_state_topic{"/wbrov/safe_relocalization/state"};
  std::string core_status_topic{"/wbrov/safe_relocalization/status"};
  std::string proposal_topic{"/wbrov/safe_relocalization/proposal"};
  std::string expected_metadata_node{"apriltag_landmark"};
  std::string expected_core_node{"rtabmap"};
  std::string expected_odom_node{"rtabmap_rgbd_odometry"};
  std::string expected_namespace{"/"};
};

std::string requireValue(int argc, char ** argv, int & index, const std::string & option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(option + " requires a value");
  }
  ++index;
  return argv[index];
}

std::uint64_t parsePositive(const std::string & option, const std::string & value)
{
  std::size_t used = 0U;
  unsigned long long parsed = 0ULL;
  try {
    parsed = std::stoull(value, &used, 10);
  } catch (const std::exception &) {
    throw std::invalid_argument(option + " requires a positive integer");
  }
  if (used != value.size() || parsed == 0ULL) {
    throw std::invalid_argument(option + " requires a positive integer");
  }
  return static_cast<std::uint64_t>(parsed);
}

bool validTopic(const std::string & value)
{
  return value.size() > 1U && value.front() == '/' && value.back() != '/' &&
    value.find_first_of(" \t\r\n") == std::string::npos &&
    value.find("//") == std::string::npos && value.find("..") == std::string::npos;
}

bool validNode(const std::string & value)
{
  return !value.empty() && value.find('/') == std::string::npos &&
    value.find_first_of(" \t\r\n") == std::string::npos;
}

Arguments parseArguments(int argc, char ** argv)
{
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--check-only") {
      result.check_only = true;
    } else if (option == "--socket") {
      result.socket_path = requireValue(argc, argv, index, option);
    } else if (option == "--state") {
      result.state_path = requireValue(argc, argv, index, option);
    } else if (option == "--session-token") {
      result.session_token = requireValue(argc, argv, index, option);
    } else if (option == "--expected-engine-identity-file") {
      result.expected_engine_identity_file = requireValue(argc, argv, index, option);
    } else if (option == "--metadata-topic") {
      result.metadata_topic = requireValue(argc, argv, index, option);
    } else if (option == "--info-topic") {
      result.info_topic = requireValue(argc, argv, index, option);
    } else if (option == "--odom-topic") {
      result.odom_topic = requireValue(argc, argv, index, option);
    } else if (option == "--core-state-topic") {
      result.core_state_topic = requireValue(argc, argv, index, option);
    } else if (option == "--core-status-topic") {
      result.core_status_topic = requireValue(argc, argv, index, option);
    } else if (option == "--proposal-topic") {
      result.proposal_topic = requireValue(argc, argv, index, option);
    } else if (option == "--expected-metadata-node") {
      result.expected_metadata_node = requireValue(argc, argv, index, option);
    } else if (option == "--expected-core-node") {
      result.expected_core_node = requireValue(argc, argv, index, option);
    } else if (option == "--expected-odom-node") {
      result.expected_odom_node = requireValue(argc, argv, index, option);
    } else if (option == "--expected-namespace") {
      result.expected_namespace = requireValue(argc, argv, index, option);
    } else if (option == "--help" || option == "-h") {
      std::cout << "safe_reloc_gid_frontend --check-only | --socket ABS --state ABS "
        "--session-token HEX64 --expected-engine-identity-file ABS "
        "[topic/node overrides]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + option);
    }
  }
  const bool token_valid = result.session_token.size() == 64U &&
    std::all_of(result.session_token.begin(), result.session_token.end(), [](char value) {
      return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
    });
  if (!token_valid) {
    throw std::invalid_argument("session token must be 64 lowercase hexadecimal characters");
  }
  for (const auto * topic : {
      &result.metadata_topic, &result.info_topic, &result.odom_topic,
      &result.core_state_topic, &result.core_status_topic, &result.proposal_topic})
  {
    if (!validTopic(*topic)) {
      throw std::invalid_argument("all topics must be normalized absolute names");
    }
  }
  for (const auto * node : {
      &result.expected_metadata_node, &result.expected_core_node,
      &result.expected_odom_node})
  {
    if (!validNode(*node)) {
      throw std::invalid_argument("expected publisher node name is invalid");
    }
  }
  if (result.expected_namespace != "/") {
    throw std::invalid_argument("only the reviewed root namespace is supported");
  }
  if (result.check_only) {
    if (!result.socket_path.empty() || !result.state_path.empty() ||
      !result.expected_engine_identity_file.empty())
    {
      throw std::invalid_argument("check-only cannot connect or write runtime state");
    }
  } else {
    if (!result.socket_path.is_absolute() || !result.state_path.is_absolute() ||
      !result.expected_engine_identity_file.is_absolute())
    {
      throw std::invalid_argument("runtime paths and exact engine identity are required");
    }
    for (const fs::path & directory : {
        result.socket_path.parent_path(), result.state_path.parent_path(),
        result.expected_engine_identity_file.parent_path()})
    {
      struct stat info {};
      if (directory.empty() || fs::canonical(directory) != directory ||
        ::lstat(directory.c_str(), &info) != 0 || !S_ISDIR(info.st_mode) ||
        info.st_uid != ::geteuid() || (info.st_mode & 07777) != 0700)
      {
        throw std::invalid_argument(
                "runtime socket/state/identity parents must be canonical current-user 0700 directories");
      }
    }
  }
  return result;
}

struct SignalGuard
{
  std::array<int, 3> numbers{{SIGHUP, SIGINT, SIGTERM}};
  std::array<struct sigaction, 3> previous{};
  std::size_t installed{0U};

  void install()
  {
    struct sigaction action {};
    action.sa_handler = managedSignalHandler;
    ::sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    for (std::size_t index = 0U; index < numbers.size(); ++index) {
      if (::sigaction(numbers[index], &action, &previous[index]) != 0) {
        throw std::runtime_error(ipc::systemError("sigaction install failed"));
      }
      ++installed;
    }
  }

  ~SignalGuard()
  {
    for (std::size_t index = 0U; index < installed; ++index) {
      ::sigaction(numbers[index], &previous[index], nullptr);
    }
  }
};

enum class Role : std::size_t
{
  Metadata = 0U,
  Info = 1U,
  Odom = 2U,
  CoreState = 3U,
  CoreStatus = 4U,
};

const char * roleName(Role role)
{
  switch (role) {
    case Role::Metadata: return "metadata";
    case Role::Info: return "rtabmap_info";
    case Role::Odom: return "odom_info_lite";
    case Role::CoreState: return "core_state";
    case Role::CoreStatus: return "core_status";
  }
  return "unknown";
}

ipc::MessageType roleType(Role role)
{
  switch (role) {
    case Role::Metadata: return ipc::MessageType::Metadata;
    case Role::Info: return ipc::MessageType::Info;
    case Role::Odom: return ipc::MessageType::Odom;
    case Role::CoreState: return ipc::MessageType::CoreState;
    case Role::CoreStatus: return ipc::MessageType::CoreStatus;
  }
  throw std::runtime_error("unknown source role");
}

struct SourceBinding
{
  Role role{};
  std::string topic;
  std::string node_name;
  std::string node_namespace;
  bool endpoint_bound{false};
  bool message_seen{false};
  bool message_required{true};
  std::array<std::uint8_t, kGidBytes> gid{};
};

std::array<std::uint8_t, kGidBytes> incomingGid(const rclcpp::MessageInfo & info)
{
  std::array<std::uint8_t, kGidBytes> result{};
  const auto & raw = info.get_rmw_message_info().publisher_gid;
  std::copy(raw.data, raw.data + kGidBytes, result.begin());
  return result;
}

std::array<std::uint8_t, kGidBytes> endpointGid(const rclcpp::TopicEndpointInfo & endpoint)
{
  const auto & raw = endpoint.endpoint_gid();
  if (raw.size() != kGidBytes) {
    throw std::runtime_error("graph endpoint GID is not exactly 24 bytes");
  }
  std::array<std::uint8_t, kGidBytes> result{};
  std::copy(raw.begin(), raw.end(), result.begin());
  return result;
}

bool nonzero(const std::array<std::uint8_t, kGidBytes> & value)
{
  return std::any_of(value.begin(), value.end(), [](std::uint8_t item) {return item != 0U;});
}

std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return -1;
  }
  const std::int64_t seconds = static_cast<std::int64_t>(stamp.sec);
  if (seconds > std::numeric_limits<std::int64_t>::max() / 1000000000LL) {
    return -1;
  }
  const std::int64_t result = seconds * 1000000000LL + stamp.nanosec;
  return result > 0 ? result : -1;
}

std::uint64_t extractPositiveJsonInteger(const std::string & input, const std::string & name)
{
  const std::string needle = "\"" + name + "\"";
  const std::size_t first = input.find(needle);
  if (first == std::string::npos || input.find(needle, first + needle.size()) != std::string::npos) {
    throw std::runtime_error("JSON field " + name + " is missing or duplicated");
  }
  std::size_t cursor = first + needle.size();
  while (cursor < input.size() && std::isspace(static_cast<unsigned char>(input[cursor]))) {
    ++cursor;
  }
  if (cursor >= input.size() || input[cursor++] != ':') {
    throw std::runtime_error("JSON field " + name + " lacks colon");
  }
  while (cursor < input.size() && std::isspace(static_cast<unsigned char>(input[cursor]))) {
    ++cursor;
  }
  const std::size_t begin = cursor;
  while (cursor < input.size() && input[cursor] >= '0' && input[cursor] <= '9') {
    ++cursor;
  }
  if (cursor == begin) {
    throw std::runtime_error("JSON field " + name + " is not an unsigned integer");
  }
  return parsePositive(name, input.substr(begin, cursor - begin));
}

std::int64_t mapId(const rtabmap_msgs::msg::Info & message)
{
  if (message.stats_keys.size() != message.stats_values.size()) {
    return -1;
  }
  for (std::size_t index = 0U; index < message.stats_keys.size(); ++index) {
    if (message.stats_keys[index] == "WBROV/Reference_map_id/") {
      const double value = static_cast<double>(message.stats_values[index]);
      if (!std::isfinite(value) || value < 0.0 ||
        value > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
      {
        return -1;
      }
      return static_cast<std::int64_t>(std::llround(value));
    }
  }
  return -1;
}

struct Outgoing
{
  ipc::MessageType type{};
  std::uint64_t stamp_ns{0U};
  std::string payload;
  std::chrono::steady_clock::time_point first;
  std::chrono::steady_clock::time_point last;
  unsigned attempts{1U};
};

class FrontendNode : public rclcpp::Node
{
public:
  FrontendNode(const Arguments & args, int connection)
  : Node("safe_reloc_gid_frontend", nodeOptions()), args_(args), connection_(connection)
  {
    sources_[0] = {Role::Metadata, args.metadata_topic,
      args.expected_metadata_node, args.expected_namespace, false, false, false};
    sources_[1] = {Role::Info, args.info_topic,
      args.expected_core_node, args.expected_namespace};
    sources_[2] = {Role::Odom, args.odom_topic,
      args.expected_odom_node, args.expected_namespace};
    sources_[3] = {Role::CoreState, args.core_state_topic,
      args.expected_core_node, args.expected_namespace};
    sources_[4] = {Role::CoreStatus, args.core_status_topic,
      args.expected_core_node, args.expected_namespace, false, false, false};

    auto reliable = rclcpp::QoS(rclcpp::KeepLast(50));
    reliable.reliable().durability_volatile();
    proposal_publisher_ = create_publisher<std_msgs::msg::String>(args.proposal_topic, reliable);
  }

  bool allEndpointsBound() const noexcept
  {
    return !hasFailure() && std::all_of(sources_.begin(), sources_.end(),
      [](const SourceBinding & source) {return source.endpoint_bound;});
  }
  bool subscriptionsActive() const noexcept {return subscriptions_active_;}

  void activateSubscriptions()
  {
    if (subscriptions_active_) {return;}
    if (!allEndpointsBound()) {
      throw std::runtime_error("cannot activate subscriptions before all publisher endpoints bind");
    }
    auto reliable = rclcpp::QoS(rclcpp::KeepLast(50));
    reliable.reliable().durability_volatile();
    auto best_effort = rclcpp::QoS(rclcpp::KeepLast(50));
    best_effort.best_effort().durability_volatile();
    auto epoch = rclcpp::QoS(rclcpp::KeepLast(1));
    epoch.reliable().transient_local();
    metadata_subscription_ = create_subscription<std_msgs::msg::String>(
      args_.metadata_topic, reliable,
      [this](std_msgs::msg::String::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        if (!authenticate(Role::Metadata, info)) {return;}
        if (quiescing_) {return;}
        try {
          const auto stamp = extractPositiveJsonInteger(message->data, "image_stamp_ns");
          forward(Role::Metadata, stamp, message->data);
        } catch (const std::exception & error) {fail("metadata_serialization:" + std::string(error.what()));}
      });
    info_subscription_ = create_subscription<rtabmap_msgs::msg::Info>(
      args_.info_topic, reliable,
      [this](rtabmap_msgs::msg::Info::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        if (!authenticate(Role::Info, info)) {return;}
        if (quiescing_) {return;}
        const auto stamp = stampNs(message->header.stamp);
        const auto map = mapId(*message);
        if (stamp <= 0 || message->ref_id <= 0 || map < 0) {
          fail("invalid_authenticated_info"); return;
        }
        std::ostringstream payload;
        payload << "{\"map_id\":" << map << ",\"ref_id\":" << message->ref_id
                << ",\"stamp_ns\":" << stamp << '}';
        forward(Role::Info, static_cast<std::uint64_t>(stamp), payload.str());
      });
    odom_subscription_ = create_subscription<rtabmap_msgs::msg::OdomInfo>(
      args_.odom_topic, best_effort,
      [this](rtabmap_msgs::msg::OdomInfo::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        if (!authenticate(Role::Odom, info)) {return;}
        if (quiescing_) {return;}
        const auto stamp = stampNs(message->header.stamp);
        if (stamp <= 0) {fail("invalid_authenticated_odom_stamp"); return;}
        std::ostringstream payload;
        payload << "{\"lost\":" << (message->lost ? "true" : "false")
                << ",\"stamp_ns\":" << stamp << '}';
        forward(Role::Odom, static_cast<std::uint64_t>(stamp), payload.str());
      });
    core_state_subscription_ = create_subscription<std_msgs::msg::String>(
      args_.core_state_topic, epoch,
      [this](std_msgs::msg::String::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        if (authenticate(Role::CoreState, info) && !quiescing_) {
          forward(Role::CoreState, 0U, message->data);
        }
      });
    core_status_subscription_ = create_subscription<std_msgs::msg::String>(
      args_.core_status_topic, reliable,
      [this](std_msgs::msg::String::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        if (authenticate(Role::CoreStatus, info)) {forward(Role::CoreStatus, 0U, message->data);}
      });
    subscriptions_active_ = true;
  }

  bool hasFailure() const noexcept {return !failure_.empty();}
  const std::string & failure() const noexcept {return failure_;}
  bool allAuthenticated() const noexcept
  {
    return !hasFailure() && std::all_of(sources_.begin(), sources_.end(),
      [](const SourceBinding & source) {
        return source.endpoint_bound && (!source.message_required || source.message_seen);
      });
  }
  std::size_t outstandingCount() const noexcept {return outgoing_.size();}
  bool isAcknowledged(std::uint64_t sequence) const
  {
    return outgoing_.find(sequence) == outgoing_.end();
  }
  bool engineQuiesced() const noexcept {return engine_quiesced_;}

  void fail(const std::string & reason)
  {
    if (failure_.empty()) {
      failure_ = reason;
    }
  }

  void verifyFrozenSources(bool require_authenticated)
  {
    if (hasFailure()) {return;}
    for (auto & source : sources_) {
      std::vector<rclcpp::TopicEndpointInfo> endpoints;
      try {
        endpoints = get_publishers_info_by_topic(source.topic);
      } catch (const std::exception & error) {
        fail("publisher_graph_error:" + std::string(roleName(source.role)) + ":" + error.what());
        return;
      }
      if (endpoints.empty() && !source.endpoint_bound && !require_authenticated) {
        continue;
      }
      if (endpoints.size() != 1U) {
        fail("publisher_set_changed:" + std::string(roleName(source.role)) + ":" +
          std::to_string(endpoints.size()));
        return;
      }
      const auto & endpoint = endpoints.front();
      const bool pending_name = endpoint.node_name() == "_NODE_NAME_UNKNOWN_";
      const bool pending_namespace = endpoint.node_namespace() == "_NODE_NAMESPACE_UNKNOWN_";
      if (!source.endpoint_bound && !require_authenticated && nonzero(endpointGid(endpoint)) &&
        (pending_name || pending_namespace) &&
        (pending_name || endpoint.node_name() == source.node_name) &&
        (pending_namespace || endpoint.node_namespace() == source.node_namespace))
      {
        continue;  // Keep the startup handshake pending, without binding this source.
      }
      if (endpoint.node_name() != source.node_name ||
        endpoint.node_namespace() != source.node_namespace)
      {
        fail("publisher_identity_changed:" + std::string(roleName(source.role)));
        return;
      }
      const auto gid = endpointGid(endpoint);
      if (!nonzero(gid)) {
        fail("publisher_gid_zero:" + std::string(roleName(source.role))); return;
      }
      if (source.endpoint_bound && source.gid != gid) {
        fail("publisher_gid_changed:" + std::string(roleName(source.role))); return;
      }
      if (!source.endpoint_bound) {
        source.endpoint_bound = true;
        source.gid = gid;
      }
      if (require_authenticated &&
        (!source.endpoint_bound || (source.message_required && !source.message_seen)))
      {
        fail("publisher_not_authenticated:" + std::string(roleName(source.role)));
        return;
      }
    }
  }

  std::string publisherSourcesJson() const
  {
    std::ostringstream output;
    output << "{\"all_authenticated\":" << (allAuthenticated() ? "true" : "false")
           << ",\"failure\":" << (hasFailure() ? ipc::jsonEscape(failure_) : "null")
           << ",\"sources\":{";
    for (std::size_t index = 0U; index < sources_.size(); ++index) {
      if (index != 0U) {output << ',';}
      const auto & source = sources_[index];
      const bool authenticated = source.endpoint_bound &&
        (!source.message_required || source.message_seen);
      output << ipc::jsonEscape(roleName(source.role)) << ":{\"authenticated\":"
             << (authenticated ? "true" : "false")
             << ",\"endpoint_bound\":" << (source.endpoint_bound ? "true" : "false")
             << ",\"message_required\":" << (source.message_required ? "true" : "false")
             << ",\"message_seen\":" << (source.message_seen ? "true" : "false")
             << ",\"node_name\":" << ipc::jsonEscape(source.node_name)
             << ",\"node_namespace\":" << ipc::jsonEscape(source.node_namespace)
             << ",\"publisher_gid\":"
             << (source.endpoint_bound ? ipc::jsonEscape(ipc::hex(source.gid.data(), source.gid.size())) : "null")
             << ",\"topic\":" << ipc::jsonEscape(source.topic) << '}';
    }
    output << "}}";
    return output.str();
  }

  std::uint64_t sendDetach()
  {
    return sendTracked(ipc::MessageType::Detach, 0U, "");
  }

  std::uint64_t beginQuiesce()
  {
    if (quiescing_) {throw std::runtime_error("frontend QUIESCE already started");}
    quiescing_ = true;
    return sendTracked(ipc::MessageType::Quiesce, 0U, "");
  }

  void serviceIpc()
  {
    try {
      while (ipc::waitReadable(connection_, 0)) {
        auto message = receiver_.receiveOne(connection_, true);
        if (message.has_value()) {processIncoming(*message);}
      }
      retryOutgoing();
    } catch (const std::exception & error) {
      fail("ipc_failure:" + std::string(error.what()));
    }
  }

private:
  static rclcpp::NodeOptions nodeOptions()
  {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.enable_rosout(false);
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    return options;
  }

  SourceBinding & source(Role role) {return sources_[static_cast<std::size_t>(role)];}

  bool authenticate(Role role, const rclcpp::MessageInfo & info)
  {
    if (hasFailure()) {return false;}
    auto & binding = source(role);
    try {
      const auto endpoints = get_publishers_info_by_topic(binding.topic);
      if (endpoints.size() != 1U) {
        fail("publisher_set_not_singleton:" + std::string(roleName(role)) + ":" +
          std::to_string(endpoints.size())); return false;
      }
      const auto & endpoint = endpoints.front();
      const auto graph_gid = endpointGid(endpoint);
      const auto message_gid = incomingGid(info);
      const bool pending_name = endpoint.node_name() == "_NODE_NAME_UNKNOWN_";
      const bool pending_namespace = endpoint.node_namespace() == "_NODE_NAMESPACE_UNKNOWN_";
      if (!binding.endpoint_bound && nonzero(graph_gid) && nonzero(message_gid) &&
        graph_gid == message_gid && (pending_name || pending_namespace) &&
        (pending_name || endpoint.node_name() == binding.node_name) &&
        (pending_namespace || endpoint.node_namespace() == binding.node_namespace))
      {
        return false;  // Drop the sample until ROS graph metadata resolves.
      }
      if (endpoint.node_name() != binding.node_name ||
        endpoint.node_namespace() != binding.node_namespace)
      {
        fail("publisher_identity_mismatch:" + std::string(roleName(role))); return false;
      }
      if (!nonzero(graph_gid) || !nonzero(message_gid) || graph_gid != message_gid) {
        fail("publisher_gid_mismatch:" + std::string(roleName(role))); return false;
      }
      if (binding.endpoint_bound && binding.gid != graph_gid) {
        fail("publisher_gid_changed:" + std::string(roleName(role))); return false;
      }
      binding.endpoint_bound = true;
      binding.message_seen = true;
      binding.gid = graph_gid;
      return true;
    } catch (const std::exception & error) {
      fail("publisher_authentication_error:" + std::string(roleName(role)) + ":" + error.what());
      return false;
    }
  }

  void forward(Role role, std::uint64_t stamp_ns, const std::string & payload)
  {
    if (payload.size() > ipc::kMaxPayloadBytes) {
      fail("authenticated_payload_too_large:" + std::string(roleName(role))); return;
    }
    try {
      sendTracked(roleType(role), stamp_ns, payload);
    } catch (const std::exception & error) {
      fail("ipc_send_failed:" + std::string(roleName(role)) + ":" + error.what());
    }
  }

  std::uint64_t sendTracked(
    ipc::MessageType type, std::uint64_t stamp_ns, const std::string & payload)
  {
    const std::uint64_t sequence = ++output_sequence_;
    ipc::sendLogical(connection_, type, sequence, 0U, stamp_ns, payload);
    const auto now = std::chrono::steady_clock::now();
    outgoing_.emplace(sequence, Outgoing{type, stamp_ns, payload, now, now, 1U});
    return sequence;
  }

  void retryOutgoing()
  {
    const auto now = std::chrono::steady_clock::now();
    for (auto it = acknowledged_retries_.begin(); it != acknowledged_retries_.end();) {
      if (now >= it->second.second) {it = acknowledged_retries_.erase(it);}
      else {++it;}
    }
    for (auto & item : outgoing_) {
      const double age = std::chrono::duration<double>(now - item.second.first).count();
      const double since = std::chrono::duration<double>(now - item.second.last).count();
      if (age >= 5.0 || item.second.attempts >= 20U) {
        fail("ipc_ack_timeout:" + std::to_string(item.first)); return;
      }
      if (since >= 0.25) {
        ipc::sendLogical(connection_, item.second.type, item.first, 0U,
          item.second.stamp_ns, item.second.payload);
        item.second.last = now;
        ++item.second.attempts;
      }
    }
  }

  void processIncoming(const ipc::LogicalMessage & message)
  {
    if (message.type == ipc::MessageType::Ack) {
      const auto found = outgoing_.find(message.ack_sequence);
      if (found == outgoing_.end()) {
        const auto retry = acknowledged_retries_.find(message.ack_sequence);
        if (retry == acknowledged_retries_.end() || retry->second.first == 0U ||
          std::chrono::steady_clock::now() >= retry->second.second)
        {
          throw std::runtime_error("engine ACK names unknown/already-ACKed sequence");
        }
        // Each exact retransmission is ACKed independently by the engine.
        // Accept only as many delayed ACKs as messages we actually resent.
        if (--retry->second.first == 0U) {acknowledged_retries_.erase(retry);}
        return;
      }
      if (found->second.attempts > 1U) {
        acknowledged_retries_.emplace(message.ack_sequence, std::make_pair(
          found->second.attempts - 1U,
          std::chrono::steady_clock::now() + std::chrono::seconds(5)));
      }
      outgoing_.erase(found);
      return;
    }
    const auto identity = std::make_pair(message.type, message.digest);
    const auto old = incoming_seen_.find(message.sequence);
    if (old != incoming_seen_.end()) {
      if (old->second != identity) {
        throw std::runtime_error("engine replay changed type/digest");
      }
      ipc::sendAck(connection_, message.sequence);
      return;
    }
    if (message.sequence <= highest_incoming_sequence_) {
      throw std::runtime_error("new engine sequence is not monotonic");
    }
    if (message.type == ipc::MessageType::Quiesced) {
      if (!quiescing_ || message.payload != "{\"safe\":true}" || engine_quiesced_) {
        throw std::runtime_error("invalid or duplicate engine QUIESCED");
      }
      engine_quiesced_ = true;
    } else if (message.type == ipc::MessageType::Proposal) {
      std_msgs::msg::String proposal;
      proposal.data = message.payload;
      proposal_publisher_->publish(proposal);
    } else {
      throw std::runtime_error("engine sent forbidden post-handshake IPC type");
    }
    incoming_seen_[message.sequence] = identity;
    highest_incoming_sequence_ = message.sequence;
    ipc::sendAck(connection_, message.sequence);
  }

  const Arguments & args_;
  int connection_{-1};
  std::array<SourceBinding, 5> sources_;
  std::string failure_;
  std::uint64_t output_sequence_{1U};
  std::uint64_t highest_incoming_sequence_{1U};
  std::map<std::uint64_t, Outgoing> outgoing_;
  std::map<std::uint64_t, std::pair<unsigned int, std::chrono::steady_clock::time_point>> acknowledged_retries_;
  std::map<std::uint64_t, std::pair<ipc::MessageType, std::array<std::uint8_t, 32>>> incoming_seen_;
  ipc::Receiver receiver_;
  bool subscriptions_active_{false};
  bool quiescing_{false};
  bool engine_quiesced_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr proposal_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_subscription_;
  rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr info_subscription_;
  rclcpp::Subscription<rtabmap_msgs::msg::OdomInfo>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr core_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr core_status_subscription_;
};

struct EnginePeer
{
  pid_t pid{0};
  uid_t uid{0U};
  std::uint64_t start_ticks{0U};
  ipc::FileIdentity exe;
  ipc::FileIdentity script;
};

struct EngineExpectation
{
  pid_t pid{0};
  std::uint64_t start_ticks{0U};
  ipc::FileIdentity exe;
  ipc::FileIdentity script;
};

EngineExpectation readEngineExpectation(const fs::path & path)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  int descriptor = -1;
  while (descriptor < 0) {
    descriptor = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (descriptor >= 0) {break;}
    if (errno != ENOENT || std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(ipc::systemError("cannot open engine identity file"));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  struct stat info {};
  if (::fstat(descriptor, &info) != 0 || !S_ISREG(info.st_mode) ||
    info.st_uid != ::geteuid() || info.st_nlink != 1 || (info.st_mode & 07777) != 0600)
  {
    ::close(descriptor);
    throw std::runtime_error("engine identity file must be current-user 0600 single-link regular");
  }
  std::string contents;
  std::array<char, 4096> buffer{};
  for (;;) {
    const ssize_t count = ::read(descriptor, buffer.data(), buffer.size());
    if (count < 0 && errno == EINTR) {continue;}
    if (count < 0) {::close(descriptor); throw std::runtime_error(ipc::systemError("identity read failed"));}
    if (count == 0) {break;}
    contents.append(buffer.data(), static_cast<std::size_t>(count));
    if (contents.size() > 8192U) {::close(descriptor); throw std::runtime_error("identity file too large");}
  }
  ::close(descriptor);
  std::vector<std::string> lines;
  std::istringstream stream(contents);
  std::string line;
  while (std::getline(stream, line)) {lines.push_back(line);}
  const std::array<std::string, 9> names{{
    "schema_version", "pid", "process_start_ticks", "exe_dev", "exe_ino",
    "script_dev", "script_ino", "exe_path", "script_path"}};
  if (lines.size() != names.size()) {throw std::runtime_error("engine identity schema line count mismatch");}
  std::array<std::string, 9> values{};
  for (std::size_t index = 0U; index < names.size(); ++index) {
    const std::string prefix = names[index] + "=";
    if (lines[index].rfind(prefix, 0U) != 0U || lines[index].size() == prefix.size()) {
      throw std::runtime_error("engine identity schema/order mismatch");
    }
    values[index] = lines[index].substr(prefix.size());
  }
  if (values[0] != "1") {throw std::runtime_error("engine identity schema version mismatch");}
  const auto parsed_pid = parsePositive("identity pid", values[1]);
  if (parsed_pid > static_cast<std::uint64_t>(std::numeric_limits<pid_t>::max())) {
    throw std::runtime_error("engine identity PID out of range");
  }
  EngineExpectation expected;
  expected.pid = static_cast<pid_t>(parsed_pid);
  expected.start_ticks = parsePositive("identity start ticks", values[2]);
  expected.exe.device = parsePositive("identity exe dev", values[3]);
  expected.exe.inode = parsePositive("identity exe ino", values[4]);
  expected.script.device = parsePositive("identity script dev", values[5]);
  expected.script.inode = parsePositive("identity script ino", values[6]);
  expected.exe.canonical_path = values[7];
  expected.script.canonical_path = values[8];
  if (!fs::path(expected.exe.canonical_path).is_absolute() ||
    !fs::path(expected.script.canonical_path).is_absolute() ||
    expected.exe.device != ipc::fileIdentity(expected.exe.canonical_path).device ||
    expected.exe.inode != ipc::fileIdentity(expected.exe.canonical_path).inode ||
    expected.script.device != ipc::fileIdentity(expected.script.canonical_path, true).device ||
    expected.script.inode != ipc::fileIdentity(expected.script.canonical_path, true).inode)
  {
    throw std::runtime_error("engine identity paths do not match dev/inode");
  }
  return expected;
}

void verifySocketPath(const fs::path & path)
{
  struct stat info {};
  if (::lstat(path.c_str(), &info) != 0) {
    throw std::runtime_error(ipc::systemError("cannot stat engine socket"));
  }
  if (!S_ISSOCK(info.st_mode) || info.st_uid != ::geteuid() ||
    (info.st_mode & 07777) != 0600)
  {
    throw std::runtime_error("engine socket is not a current-user 0600 Unix socket");
  }
}

int connectSocket(const fs::path & path)
{
  if (path.string().size() >= sizeof(sockaddr_un::sun_path)) {
    throw std::invalid_argument("Unix socket path is too long");
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  for (;;) {
    try {
      verifySocketPath(path);
    } catch (const std::exception &) {
      if (std::chrono::steady_clock::now() >= deadline) {throw;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    const int descriptor = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (descriptor < 0) {
      throw std::runtime_error(ipc::systemError("cannot create seqpacket socket"));
    }
    struct sockaddr_un address {};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.string().size() + 1U);
    if (::connect(descriptor, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) == 0) {
      return descriptor;
    }
    const int saved = errno;
    ::close(descriptor);
    if ((saved != ECONNREFUSED && saved != ENOENT) ||
      std::chrono::steady_clock::now() >= deadline)
    {
      errno = saved;
      throw std::runtime_error(ipc::systemError("cannot connect engine socket"));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

EnginePeer verifyEngine(const EngineExpectation & expected, int descriptor)
{
  struct ucred credentials {};
  socklen_t size = sizeof(credentials);
  if (::getsockopt(descriptor, SOL_SOCKET, SO_PEERCRED, &credentials, &size) != 0 ||
    size != sizeof(credentials))
  {
    throw std::runtime_error(ipc::systemError("cannot read engine peer credentials"));
  }
  if (credentials.uid != ::geteuid() || credentials.pid != expected.pid) {
    throw std::runtime_error("engine peer PID/UID mismatch");
  }
  const auto start_ticks = ipc::processStartTicks(credentials.pid);
  const auto exe = ipc::processExeIdentity(credentials.pid);
  const auto script = ipc::fileIdentity(expected.script.canonical_path, true);
  if (start_ticks != expected.start_ticks ||
    exe.device != expected.exe.device || exe.inode != expected.exe.inode ||
    exe.canonical_path != expected.exe.canonical_path ||
    script.device != expected.script.device || script.inode != expected.script.inode ||
    script.canonical_path != expected.script.canonical_path)
  {
    throw std::runtime_error("engine process/executable/script identity mismatch");
  }
  return EnginePeer{credentials.pid, credentials.uid, start_ticks, exe, script};
}

std::string identityJson(const ipc::FileIdentity & value)
{
  std::ostringstream output;
  output << "{\"device\":" << value.device << ",\"inode\":" << value.inode
         << ",\"path\":" << ipc::jsonEscape(value.canonical_path) << '}';
  return output.str();
}

std::string frontendHello(const std::string & token)
{
  const auto exe = ipc::processExeIdentity(::getpid());
  std::ostringstream output;
  output << "{\"exe\":" << identityJson(exe)
         << ",\"pid\":" << ::getpid()
         << ",\"process_start_ticks\":" << ipc::processStartTicks(::getpid())
         << ",\"role\":\"safe_reloc_gid_frontend\",\"schema_version\":1"
         << ",\"session_token_sha256\":"
         << ipc::jsonEscape(ipc::hex(ipc::sha256(token))) << '}';
  return output.str();
}

std::string expectedEngineHello(const Arguments & args, const EnginePeer & peer)
{
  std::ostringstream output;
  output << "{\"exe\":" << identityJson(peer.exe)
         << ",\"pid\":" << peer.pid
         << ",\"process_start_ticks\":" << peer.start_ticks
         << ",\"role\":\"safe_reloc_ipc_engine\",\"schema_version\":1"
         << ",\"script\":" << identityJson(peer.script)
         << ",\"session_token_sha256\":"
         << ipc::jsonEscape(ipc::hex(ipc::sha256(args.session_token))) << '}';
  return output.str();
}

void performHandshake(const Arguments & args, const EnginePeer & peer, int descriptor)
{
  ipc::Receiver receiver;
  constexpr std::uint64_t kFrontendHelloSequence = 1U;
  ipc::sendLogical(descriptor, ipc::MessageType::Hello,
    kFrontendHelloSequence, 0U, 0U, frontendHello(args.session_token));
  bool frontend_hello_acked = false;
  bool engine_hello_received = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!frontend_hello_acked || !engine_hello_received) {
    if (std::chrono::steady_clock::now() >= deadline || !ipc::waitReadable(descriptor, 100)) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("engine HELLO handshake timed out");
      }
      continue;
    }
    const auto message = receiver.receiveOne(descriptor, false);
    if (!message.has_value()) {continue;}
    if (message->type == ipc::MessageType::Ack) {
      if (message->ack_sequence != kFrontendHelloSequence || frontend_hello_acked) {
        throw std::runtime_error("unexpected/duplicate HELLO ACK");
      }
      frontend_hello_acked = true;
    } else if (message->type == ipc::MessageType::Hello) {
      if (engine_hello_received || message->sequence != 1U ||
        message->payload != expectedEngineHello(args, peer))
      {
        throw std::runtime_error("engine HELLO identity mismatch");
      }
      ipc::sendAck(descriptor, message->sequence);
      engine_hello_received = true;
    } else {
      throw std::runtime_error("non-HELLO traffic during handshake");
    }
  }
}

std::string stateJson(
  const std::string & kind, int exit_code, const FrontendNode * node,
  const Arguments & args, const EnginePeer & peer)
{
  const auto self_exe = ipc::processExeIdentity(::getpid());
  std::ostringstream output;
  output << "{\"engine_peer\":{\"exe\":" << identityJson(peer.exe)
         << ",\"pid\":" << peer.pid
         << ",\"process_start_ticks\":" << peer.start_ticks
         << ",\"script\":" << identityJson(peer.script) << '}'
         << ",\"exit_code\":" << exit_code
         << ",\"failure\":" <<
    ((node != nullptr && node->hasFailure()) ? ipc::jsonEscape(node->failure()) : "null")
         << ",\"kind\":" << ipc::jsonEscape(kind)
         << ",\"monotonic_ns\":" << steadyNowNs()
         << ",\"pid\":" << ::getpid()
         << ",\"process_start_ticks\":" << ipc::processStartTicks(::getpid())
         << ",\"producer\":\"wbrov_safe_reloc_gid_frontend\""
         << ",\"publisher_sources\":" <<
    (node == nullptr ? "{\"all_authenticated\":false,\"failure\":null,\"sources\":{}}" :
    node->publisherSourcesJson())
         << ",\"ready\":" <<
    ((node != nullptr && node->allAuthenticated() && exit_code < 0) ? "true" : "false")
         << ",\"schema_version\":2,\"self_exe\":" << identityJson(self_exe)
         << ",\"session_token_sha256\":"
         << ipc::jsonEscape(ipc::hex(ipc::sha256(args.session_token)))
         << ",\"terminal\":" << (exit_code >= 0 ? "true" : "false") << '}';
  return output.str();
}

int runFrontend(const Arguments & args)
{
  g_signal_number = 0;
  SignalGuard signal_guard;
  signal_guard.install();
  const EngineExpectation expected = readEngineExpectation(args.expected_engine_identity_file);
  const int connection = connectSocket(args.socket_path);
  const EnginePeer peer = verifyEngine(expected, connection);
  try {
    performHandshake(args, peer, connection);
    int ros_argc = 1;
    char program_name[] = "safe_reloc_gid_frontend";
    char * ros_argv[] = {program_name, nullptr};
    rclcpp::init(ros_argc, ros_argv, rclcpp::InitOptions(),
      rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<FrontendNode>(args, connection);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    ipc::AtomicStateWriter state(args.state_path);
    state.write(stateJson("started", -1, node.get(), args, peer));
    auto next_graph_check = std::chrono::steady_clock::now();
    bool ready_written = false;
    int exit_code = 1;
    std::string final_kind = "failure";
    while (rclcpp::ok()) {
      executor.spin_some();
      node->serviceIpc();
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_graph_check) {
        node->verifyFrozenSources(false);
        if (node->allEndpointsBound() && !node->subscriptionsActive()) {
          try {
            node->activateSubscriptions();
          } catch (const std::exception & error) {
            node->fail("subscription_activation_failed:" + std::string(error.what()));
          }
        }
        next_graph_check = now + std::chrono::milliseconds(250);
      }
      if (node->allAuthenticated() && !ready_written) {
        state.write(stateJson("ready", -1, node.get(), args, peer));
        ready_written = true;
      }
      if (node->hasFailure()) {
        final_kind = "failure";
        exit_code = 1;
        break;
      }
      if (g_signal_number != 0) {
        node->verifyFrozenSources(true);
        if (node->hasFailure()) {
          final_kind = "source_failure_before_signal_terminal";
          exit_code = 1;
          break;
        }
        if (g_signal_number != SIGINT) {
          node->fail("only SIGINT is an accepted clean frontend shutdown");
          final_kind = "unsupported_signal";
          exit_code = 1;
          break;
        }
        const std::uint64_t quiesce_sequence = node->beginQuiesce();
        const auto quiesce_deadline = now + std::chrono::seconds(8);
        while ((!node->isAcknowledged(quiesce_sequence) ||
          !node->engineQuiesced() || node->outstandingCount() != 0U) &&
          std::chrono::steady_clock::now() < quiesce_deadline && !node->hasFailure())
        {
          executor.spin_some();
          node->serviceIpc();
          node->verifyFrozenSources(true);
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!node->isAcknowledged(quiesce_sequence) ||
          !node->engineQuiesced() || node->outstandingCount() != 0U ||
          node->hasFailure())
        {
          node->fail("engine did not prove a quiescent Core transaction boundary");
          final_kind = "quiesce_failure";
          exit_code = 1;
          break;
        }
        const std::uint64_t detach_sequence = node->sendDetach();
        const auto detach_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!node->isAcknowledged(detach_sequence) &&
          std::chrono::steady_clock::now() < detach_deadline && !node->hasFailure())
        {
          node->serviceIpc();
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!node->isAcknowledged(detach_sequence) || node->hasFailure()) {
          node->fail("engine did not acknowledge clean detach");
          final_kind = "detach_ack_failure";
          exit_code = 1;
        } else {
          // Recheck publisher graph after the detach ACK so a simultaneous
          // source teardown cannot be hidden by a nominal SIGINT terminal.
          node->verifyFrozenSources(true);
          if (node->hasFailure()) {
            final_kind = "source_failure_after_detach_ack";
            exit_code = 1;
          } else {
            final_kind = "final_sigint";
            exit_code = 130;
          }
        }
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    state.write(stateJson(final_kind, exit_code, node.get(), args, peer));
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    ::close(connection);
    return exit_code;
  } catch (...) {
    if (rclcpp::ok()) {rclcpp::shutdown();}
    ::close(connection);
    throw;
  }
}

std::string checkOnlyJson(const Arguments & args)
{
  std::ostringstream output;
  output << "{\"check_only\":true,\"device_started\":false"
         << ",\"gid_bytes\":24,\"graph_mutation\":false"
         << ",\"ipc\":\"AF_UNIX_SOCK_SEQPACKET_0600\""
         << ",\"producer\":\"wbrov_safe_reloc_gid_frontend\""
         << ",\"python_ros_network_fallback\":false"
         << ",\"ros_context_started\":false,\"schema_version\":2"
         << ",\"session_token_sha256\":"
         << ipc::jsonEscape(ipc::hex(ipc::sha256(args.session_token))) << '}';
  return output.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments args = parseArguments(argc, argv);
    if (args.check_only) {
      std::cout << checkOnlyJson(args) << '\n';
      return 0;
    }
    return runFrontend(args);
  } catch (const std::exception & error) {
    std::cerr << "SAFE_RELOC_GID_FRONTEND_REFUSED: " << error.what() << '\n';
    return 1;
  }
}
