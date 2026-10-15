#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <std_msgs/msg/string.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using SteadyClock = std::chrono::steady_clock;

enum class Mode {Positive, SecondPublisher, GidSwap, Disappear, SameStampConflict, EpochCutoff};
enum class FaultRole {Metadata, Status};

struct Arguments
{
  Mode mode{Mode::Positive};
  double duration_s{12.0};
  double fault_after_s{3.0};
  double publish_hz{10.0};
  std::string session_token;
  bool publish_status{true};
  bool publish_metadata{true};
  bool publish_info{true};
  bool publish_odom{true};
  bool publish_core_state{true};
  FaultRole fault_role{FaultRole::Metadata};
  std::optional<std::filesystem::path> fault_trigger_file;
};

std::string requireValue(int argc, char ** argv, int & index, const std::string & option)
{
  if (index + 1 >= argc) {throw std::invalid_argument(option + " requires a value");}
  ++index;
  return argv[index];
}

double parseFinite(const std::string & option, const std::string & value)
{
  std::size_t used = 0U;
  double parsed = 0.0;
  try {parsed = std::stod(value, &used);} catch (const std::exception &) {
    throw std::invalid_argument(option + " requires a finite number");
  }
  if (used != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(option + " requires a finite number");
  }
  return parsed;
}

Arguments parseArguments(int argc, char ** argv)
{
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--mode") {
      const std::string value = requireValue(argc, argv, index, option);
      if (value == "positive") {result.mode = Mode::Positive;}
      else if (value == "second_publisher") {result.mode = Mode::SecondPublisher;}
      else if (value == "gid_swap") {result.mode = Mode::GidSwap;}
      else if (value == "disappear") {result.mode = Mode::Disappear;}
      else if (value == "same_stamp_conflict") {result.mode = Mode::SameStampConflict;}
      else if (value == "epoch_cutoff") {result.mode = Mode::EpochCutoff;}
      else {throw std::invalid_argument("unknown fixture mode");}
    } else if (option == "--duration-s") {
      result.duration_s = parseFinite(option, requireValue(argc, argv, index, option));
    } else if (option == "--fault-after-s") {
      result.fault_after_s = parseFinite(option, requireValue(argc, argv, index, option));
    } else if (option == "--publish-hz") {
      result.publish_hz = parseFinite(option, requireValue(argc, argv, index, option));
    } else if (option == "--session-token") {
      result.session_token = requireValue(argc, argv, index, option);
    } else if (option == "--fault-trigger-file") {
      result.fault_trigger_file = requireValue(argc, argv, index, option);
    } else if (option == "--no-status") {
      result.publish_status = false;
    } else if (option == "--no-metadata") {
      result.publish_metadata = false;
    } else if (option == "--no-info") {
      result.publish_info = false;
    } else if (option == "--no-odom") {
      result.publish_odom = false;
    } else if (option == "--no-core-state") {
      result.publish_core_state = false;
    } else if (option == "--fault-role") {
      const std::string value = requireValue(argc, argv, index, option);
      if (value == "metadata") {result.fault_role = FaultRole::Metadata;}
      else if (value == "status") {result.fault_role = FaultRole::Status;}
      else {throw std::invalid_argument("fault role must be metadata or status");}
    } else if (option == "--help" || option == "-h") {
      std::cout << "safe_reloc_gid_fixture --mode positive|second_publisher|gid_swap|"
        "disappear|same_stamp_conflict|epoch_cutoff --session-token HEX64 "
        "[--duration-s S] [--fault-after-s S] [--publish-hz HZ] "
        "[--fault-trigger-file ABS] [--fault-role metadata|status] "
        "[--no-status] [--no-metadata] [--no-info] [--no-odom] "
        "[--no-core-state]\n";
      std::exit(0);
    } else {throw std::invalid_argument("unknown argument: " + option);}
  }
  if (result.duration_s <= 0.0 || result.fault_after_s <= 0.0 ||
    result.fault_after_s >= result.duration_s || result.publish_hz <= 0.0 ||
    result.session_token.size() != 64U)
  {
    throw std::invalid_argument("fixture timing/session arguments invalid");
  }
  if (result.fault_trigger_file.has_value() && !result.fault_trigger_file->is_absolute()) {
    throw std::invalid_argument("fault trigger must be absolute");
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

builtin_interfaces::msg::Time stampFromNs(std::int64_t value)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(value / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(value % 1000000000LL);
  return stamp;
}

std::string sha256Hex(const std::string & value)
{
  // A fixed 64-hex session token is generated by the test launcher.  To avoid
  // adding a crypto dependency to this publisher-only fixture, its SHA is
  // supplied through an environment variable computed by sha256sum.
  const char * raw = std::getenv("WBROV_TEST_SESSION_SHA256");
  if (raw == nullptr || std::string(raw).size() != 64U || value.size() != 64U) {
    throw std::runtime_error("WBROV_TEST_SESSION_SHA256 missing");
  }
  return raw;
}

const char * modeName(Mode mode)
{
  switch (mode) {
    case Mode::Positive: return "positive";
    case Mode::SecondPublisher: return "second_publisher";
    case Mode::GidSwap: return "gid_swap";
    case Mode::Disappear: return "disappear";
    case Mode::SameStampConflict: return "same_stamp_conflict";
    case Mode::EpochCutoff: return "epoch_cutoff";
  }
  return "unknown";
}

class Fixture
{
public:
  explicit Fixture(const Arguments & args)
  : args_(args),
    metadata_node_(std::make_shared<rclcpp::Node>("apriltag_landmark", nodeOptions())),
    core_node_(std::make_shared<rclcpp::Node>("rtabmap", nodeOptions())),
    odom_node_(std::make_shared<rclcpp::Node>("rtabmap_rgbd_odometry", nodeOptions())),
    intruder_node_(std::make_shared<rclcpp::Node>("intruder_metadata", nodeOptions()))
  {
    auto reliable = rclcpp::QoS(rclcpp::KeepLast(50));
    reliable.reliable().durability_volatile();
    auto best_effort = rclcpp::QoS(rclcpp::KeepLast(50));
    best_effort.best_effort().durability_volatile();
    auto epoch = rclcpp::QoS(rclcpp::KeepLast(1));
    epoch.reliable().transient_local();
    reliable_qos_ = reliable;
    metadata_publisher_ = metadata_node_->create_publisher<std_msgs::msg::String>(
      "/apriltag/pose_candidate_metadata", reliable);
    info_publisher_ = core_node_->create_publisher<rtabmap_msgs::msg::Info>("/info", reliable);
    state_publisher_ = core_node_->create_publisher<std_msgs::msg::String>(
      "/wbrov/safe_relocalization/state", epoch);
    status_publisher_ = core_node_->create_publisher<std_msgs::msg::String>(
      "/wbrov/safe_relocalization/status", reliable);
    odom_publisher_ = odom_node_->create_publisher<rtabmap_msgs::msg::OdomInfo>(
      "/odom_info_lite", best_effort);
    proposal_subscription_ = core_node_->create_subscription<std_msgs::msg::String>(
      "/wbrov/safe_relocalization/proposal", reliable,
      [this](std_msgs::msg::String::ConstSharedPtr) {++proposal_count_;});
    executor_.add_node(metadata_node_);
    executor_.add_node(core_node_);
    executor_.add_node(odom_node_);
    executor_.add_node(intruder_node_);
  }

  void run()
  {
    const auto start = SteadyClock::now();
    const auto period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / args_.publish_hz));
    auto next = start;
    while (rclcpp::ok()) {
      executor_.spin_some();
      const auto now = SteadyClock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      const bool trigger = args_.fault_trigger_file.has_value() &&
        std::filesystem::exists(*args_.fault_trigger_file);
      const bool timed = !args_.fault_trigger_file.has_value() && elapsed >= args_.fault_after_s;
      if (!fault_injected_ && args_.mode != Mode::Positive &&
        args_.mode != Mode::EpochCutoff && (trigger || timed))
      {
        injectFault();
      }
      if (elapsed >= args_.duration_s) {break;}
      if (now >= next) {
        publishOnce();
        next += period;
        if (next < now - period) {next = now + period;}
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "{\"fault_injected\":" << (fault_injected_ ? "true" : "false")
              << ",\"fixture_final\":true,\"mode\":\"" << modeName(args_.mode)
              << "\",\"proposal_count\":" << proposal_count_
              << ",\"ticks\":" << tick_ << "}\n";
  }

private:
  std::int64_t currentStamp() const
  {
    if (args_.mode == Mode::SameStampConflict) {return base_stamp_ns_;}
    return base_stamp_ns_ + static_cast<std::int64_t>(tick_) * 100000000LL;
  }

  std::string metadataJson(std::int64_t stamp) const
  {
    const double u = (args_.mode == Mode::SameStampConflict && fault_injected_) ? 0.75 : 0.25;
    std::ostringstream output;
    output << "{\"camera_frame\":\"zed_left_camera_optical_frame\""
      ",\"geometry_valid_before_temporal_confirmation\":true"
      ",\"graph_edge_added\":false,\"image_stamp_ns\":" << stamp
      << ",\"observations\":[{\"camera_quaternion_xyzw\":[0,0,0,1]"
      ",\"camera_translation_m\":[0,0,0.5],\"image_u_norm\":" << u
      << ",\"image_v_norm\":0.5,\"mean_side_px\":64.0,\"range_m\":0.5"
      ",\"reprojection_px\":0.4,\"tag_id\":7}]"
      ",\"producer\":\"wbrov_apriltag_landmark_node\",\"schema_version\":1}";
    return output.str();
  }

  std::string coreStateJson() const
  {
    const bool cutoff = args_.mode == Mode::EpochCutoff;
    const std::uint64_t epoch = cutoff ? 1U : 0U;
    const std::int64_t epoch_start = cutoff ? base_stamp_ns_ + 5000000000LL : 0LL;
    std::ostringstream output;
    output << "{\"database_binding_valid\":true"
      ",\"database_sha256\":\"" << std::string(64U, '0') << "\""
      ",\"epoch_start_stamp_ns\":" << epoch_start
      << ",\"graph_poses\":[],\"graph_revision\":" << (cutoff ? 2 : 1)
      << ",\"graph_snapshot_valid\":true"
      ",\"producer\":\"wbrov_rtabmap_safe_relocalization_core\""
      ",\"reset_epoch\":" << epoch << ",\"schema_version\":3"
      ",\"session_token_sha256\":\"" << sha256Hex(args_.session_token) << "\"}";
    return output.str();
  }

  std::string coreStatusJson() const
  {
    std::ostringstream output;
    output << "{\"core_graph_revision\":1,\"core_reset_epoch\":0"
      ",\"database_sha256\":\"" << std::string(64U, '0') << "\""
      ",\"evidence_pairs\":[],\"observer_graph_revision\":1"
      ",\"producer\":\"wbrov_rtabmap_safe_relocalization_core\""
      ",\"reason\":\"fixture_no_pending\",\"result\":\"rejected\""
      ",\"schema_version\":1,\"session_token_sha256\":\""
      << sha256Hex(args_.session_token)
      << "\",\"source_reset_epoch\":0,\"tag_id\":7"
      ",\"transaction_token\":\"" << std::string(64U, '0') << "\"}";
    return output.str();
  }

  void publishOnce()
  {
    const std::int64_t stamp = currentStamp();
    if (metadata_publisher_ && args_.publish_metadata) {
      std_msgs::msg::String metadata;
      metadata.data = metadataJson(stamp);
      metadata_publisher_->publish(metadata);
    }
    if (intruder_publisher_) {
      std_msgs::msg::String intruder;
      intruder.data = metadataJson(stamp);
      intruder_publisher_->publish(intruder);
    }
    rtabmap_msgs::msg::Info info;
    info.header.stamp = stampFromNs(stamp);
    info.ref_id = 1;
    // A fresh map has no loop target; this must not invalidate its own map ID.
    info.stats_keys.push_back("Loop/Map_id/");
    info.stats_values.push_back(-1.0F);
    info.stats_keys.push_back("WBROV/Reference_map_id/");
    info.stats_values.push_back(0.0F);
    if (args_.publish_info) {info_publisher_->publish(info);}
    rtabmap_msgs::msg::OdomInfo odom;
    odom.header.stamp = stampFromNs(stamp);
    odom.lost = false;
    if (args_.publish_odom) {odom_publisher_->publish(odom);}
    std_msgs::msg::String state;
    state.data = coreStateJson();
    if (args_.publish_core_state) {state_publisher_->publish(state);}
    if (args_.publish_status) {
      std_msgs::msg::String status;
      status.data = coreStatusJson();
      status_publisher_->publish(status);
    }
    ++tick_;
  }

  void injectFault()
  {
    fault_injected_ = true;
    if (args_.mode == Mode::SecondPublisher) {
      intruder_publisher_ = intruder_node_->create_publisher<std_msgs::msg::String>(
        args_.fault_role == FaultRole::Metadata ?
        "/apriltag/pose_candidate_metadata" :
        "/wbrov/safe_relocalization/status", reliable_qos_);
    } else if (args_.mode == Mode::GidSwap) {
      if (args_.fault_role == FaultRole::Metadata) {
        metadata_publisher_.reset();
        metadata_publisher_ = metadata_node_->create_publisher<std_msgs::msg::String>(
          "/apriltag/pose_candidate_metadata", reliable_qos_);
      } else {
        status_publisher_.reset();
        status_publisher_ = core_node_->create_publisher<std_msgs::msg::String>(
          "/wbrov/safe_relocalization/status", reliable_qos_);
      }
    } else if (args_.mode == Mode::Disappear) {
      if (args_.fault_role == FaultRole::Metadata) {metadata_publisher_.reset();}
      else {status_publisher_.reset();}
    }
    std::cerr << "fixture_fault_injected=" << modeName(args_.mode) << '\n';
  }

  Arguments args_;
  rclcpp::QoS reliable_qos_{rclcpp::KeepLast(50)};
  std::shared_ptr<rclcpp::Node> metadata_node_;
  std::shared_ptr<rclcpp::Node> core_node_;
  std::shared_ptr<rclcpp::Node> odom_node_;
  std::shared_ptr<rclcpp::Node> intruder_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_publisher_;
  rclcpp::Publisher<rtabmap_msgs::msg::Info>::SharedPtr info_publisher_;
  rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intruder_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr proposal_subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  bool fault_injected_{false};
  std::uint64_t tick_{0U};
  std::uint64_t proposal_count_{0U};
  const std::int64_t base_stamp_ns_{1700000000000000000LL};
};

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments args = parseArguments(argc, argv);
    int ros_argc = 1;
    char program_name[] = "safe_reloc_gid_fixture";
    char * ros_argv[] = {program_name, nullptr};
    rclcpp::init(ros_argc, ros_argv);
    Fixture fixture(args);
    fixture.run();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {rclcpp::shutdown();}
    std::cerr << "fixture fatal: " << error.what() << '\n';
    return 1;
  }
}
