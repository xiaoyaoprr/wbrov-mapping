#ifndef WBROV_ROLLING_HEADER_METRICS_HPP_
#define WBROV_ROLLING_HEADER_METRICS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace wbrov_header_health
{

constexpr std::int64_t kNsPerSecond = 1000000000LL;

inline std::string jsonEscape(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (character < 0x20U) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(character);
        }
    }
  }
  out << '"';
  return out.str();
}

inline std::string jsonBool(bool value)
{
  return value ? "true" : "false";
}

inline std::string jsonBool(const std::optional<bool> & value)
{
  return value.has_value() ? jsonBool(*value) : "null";
}

inline std::string jsonInt(std::int64_t value)
{
  return std::to_string(value);
}

inline std::string jsonUInt(std::uint64_t value)
{
  return std::to_string(value);
}

inline std::string jsonDouble(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  if (value == 0.0) {
    value = 0.0;
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return out.str();
}

inline std::string jsonDouble(const std::optional<double> & value)
{
  return value.has_value() ? jsonDouble(*value) : "null";
}

inline std::string jsonObject(const std::vector<std::pair<std::string, std::string>> & fields)
{
  std::ostringstream out;
  out << '{';
  bool first = true;
  for (const auto & field : fields) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << jsonEscape(field.first) << ':' << field.second;
  }
  out << '}';
  return out.str();
}

inline std::string jsonArray(const std::vector<std::string> & values)
{
  std::ostringstream out;
  out << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    out << values[index];
  }
  out << ']';
  return out.str();
}

struct Sample
{
  std::int64_t receipt_ns{};
  std::int64_t stamp_ns{};
  bool valid{};
  std::string reason;
};

struct MetricsConfig
{
  double window_s{10.0};
  double match_grace_s{1.0};
  double match_settle_s{0.5};
  double odom_min_hz{15.0};
  double high_min_hz{4.5};
  double coverage_min{0.90};
  double integration_min_hz{4.5};
  std::uint32_t expected_high_width{1920U};
  std::uint32_t expected_high_height{1200U};
  bool require_integration_proxy{true};

  void validate() const
  {
    const std::array<double, 7> numeric{
      window_s, match_grace_s, match_settle_s, odom_min_hz,
      high_min_hz, coverage_min, integration_min_hz};
    for (const double value : numeric) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("metrics configuration must be finite");
      }
    }
    if (window_s <= 0.0 || match_grace_s < 0.0 || match_settle_s < 0.0 ||
      match_settle_s >= window_s)
    {
      throw std::invalid_argument("invalid window/grace/settle configuration");
    }
    if (odom_min_hz <= 0.0 || high_min_hz <= 0.0 || integration_min_hz <= 0.0) {
      throw std::invalid_argument("rate thresholds must be positive");
    }
    if (coverage_min < 0.0 || coverage_min > 1.0) {
      throw std::invalid_argument("coverage threshold must be in [0,1]");
    }
    if (expected_high_width == 0U || expected_high_height == 0U) {
      throw std::invalid_argument("expected CameraInfo geometry must be positive");
    }
  }
};

struct SnapshotResult
{
  bool ready{};
  std::optional<bool> candidate_pass;
  std::string json;
};

class RollingHeaderMetrics
{
public:
  RollingHeaderMetrics(std::int64_t start_monotonic_ns, MetricsConfig config)
  : start_monotonic_ns_(start_monotonic_ns), config_(std::move(config))
  {
    config_.validate();
    if (start_monotonic_ns_ < 0) {
      throw std::invalid_argument("negative monotonic start");
    }
    window_ns_ = secondsToNs(config_.window_s);
    match_grace_ns_ = secondsToNs(config_.match_grace_s);
    match_settle_ns_ = secondsToNs(config_.match_settle_s);
  }

  std::pair<bool, std::string> addOdom(
    std::int64_t receipt_ns,
    std::int64_t stamp_ns,
    const std::array<double, 3> & position_xyz,
    const std::array<double, 4> & quaternion_xyzw)
  {
    std::pair<bool, std::string> validity{true, "valid"};
    if (stamp_ns <= 0) {
      validity = {false, "invalid_header_stamp"};
    } else if (!finite(position_xyz)) {
      validity = {false, "invalid_position"};
    } else if (!finite(quaternion_xyzw)) {
      validity = {false, "invalid_quaternion"};
    } else {
      double norm_squared = 0.0;
      for (const double value : quaternion_xyzw) {
        norm_squared += value * value;
      }
      const double norm = std::sqrt(norm_squared);
      if (norm <= 1e-12) {
        validity = {false, "zero_quaternion"};
      } else if (std::abs(norm - 1.0) > 0.05) {
        validity = {false, "nonunit_quaternion"};
      }
    }
    odom_.push_back({receipt_ns, stamp_ns, validity.first, validity.second});
    ++total_received_[kOdom];
    trackOrder(kOdom, stamp_ns, receipt_ns);
    return validity;
  }

  std::pair<bool, std::string> addOdomInfo(
    std::int64_t receipt_ns, std::int64_t stamp_ns, bool lost)
  {
    std::pair<bool, std::string> validity;
    if (stamp_ns <= 0) {
      validity = {false, "invalid_header_stamp"};
    } else if (lost) {
      validity = {false, "odometry_lost"};
    } else {
      validity = {true, "tracking_not_lost"};
    }
    odom_info_.push_back({receipt_ns, stamp_ns, validity.first, validity.second});
    ++total_received_[kOdomInfo];
    trackOrder(kOdomInfo, stamp_ns, receipt_ns);
    return validity;
  }

  std::pair<bool, std::string> addHighCameraInfo(
    std::int64_t receipt_ns,
    std::int64_t stamp_ns,
    std::uint32_t width,
    std::uint32_t height,
    const std::array<double, 9> & matrix)
  {
    std::pair<bool, std::string> validity{true, "valid"};
    if (stamp_ns <= 0) {
      validity = {false, "invalid_header_stamp"};
    } else if (width != config_.expected_high_width || height != config_.expected_high_height) {
      validity = {false, "unexpected_camera_info_geometry"};
    } else if (!finite(matrix) || matrix[0] <= 0.0 || matrix[4] <= 0.0 ||
      std::abs(matrix[8] - 1.0) > 1e-9)
    {
      validity = {false, "invalid_camera_matrix"};
    }
    high_.push_back({receipt_ns, stamp_ns, validity.first, validity.second});
    ++total_received_[kHigh];
    trackOrder(kHigh, stamp_ns, receipt_ns);
    return validity;
  }

  std::pair<bool, std::string> addRtabmapInfo(
    std::int64_t receipt_ns, std::int64_t stamp_ns)
  {
    const bool valid = stamp_ns > 0;
    const std::string reason = valid ? "valid" : "invalid_header_stamp";
    info_.push_back({receipt_ns, stamp_ns, valid, reason});
    ++total_received_[kInfo];
    trackOrder(kInfo, stamp_ns, receipt_ns);
    return {valid, reason};
  }

  SnapshotResult snapshot(std::int64_t now_ns)
  {
    if (now_ns < start_monotonic_ns_) {
      throw std::invalid_argument("monotonic time moved before observer start");
    }
    prune(now_ns);
    const std::int64_t cutoff = now_ns - window_ns_;
    const std::int64_t match_cutoff = cutoff - match_grace_ns_;
    const std::int64_t settle_cutoff = now_ns - match_settle_ns_;
    const double elapsed_s = static_cast<double>(now_ns - start_monotonic_ns_) /
      static_cast<double>(kNsPerSecond);
    const bool ready = elapsed_s >= config_.window_s;
    const double denominator_s = ready ? config_.window_s : std::max(elapsed_s, 1e-9);
    const double eligible_duration_s = std::max(
      denominator_s - config_.match_settle_s, 1e-9);

    const auto odom_window = samplesSince(odom_, cutoff);
    const auto odom_info_window = samplesSince(odom_info_, cutoff);
    const auto high_window = samplesSince(high_, cutoff);
    const auto info_window = samplesSince(info_, cutoff);
    const auto math_valid_odom_stamps = validStamps(odom_window);
    const auto nonlost_info_window_stamps = validStamps(odom_info_window);
    const auto lost_info_window_stamps = reasonStamps(odom_info_window, "odometry_lost");
    const auto matching_nonlost = validStamps(samplesSince(odom_info_, match_cutoff));
    const auto matching_lost = reasonStamps(samplesSince(odom_info_, match_cutoff), "odometry_lost");
    const auto valid_odom_stamps = setDifference(
      setIntersection(math_valid_odom_stamps, matching_nonlost), matching_lost);
    const auto valid_high_stamps = validStamps(high_window);

    std::set<std::int64_t> eligible_high_stamps;
    for (const Sample * sample : high_window) {
      if (sample->valid && sample->receipt_ns <= settle_cutoff) {
        eligible_high_stamps.insert(sample->stamp_ns);
      }
    }
    const auto retained_math_odom = validStamps(samplesSince(odom_, match_cutoff));
    const auto retained_valid_odom = setDifference(
      setIntersection(retained_math_odom, matching_nonlost), matching_lost);
    const auto retained_info = validStamps(samplesSince(info_, match_cutoff));
    const auto covered = setIntersection(eligible_high_stamps, retained_valid_odom);
    const auto integrated = setIntersection(eligible_high_stamps, retained_info);
    std::optional<double> coverage_ratio;
    if (!eligible_high_stamps.empty()) {
      coverage_ratio.emplace(
        static_cast<double>(covered.size()) / static_cast<double>(eligible_high_stamps.size()));
    }
    const double integration_rate = static_cast<double>(integrated.size()) / eligible_duration_s;
    std::optional<double> integration_coverage;
    if (!eligible_high_stamps.empty()) {
      integration_coverage.emplace(
        static_cast<double>(integrated.size()) /
        static_cast<double>(eligible_high_stamps.size()));
    }
    const bool info_observable = !validStamps(info_window).empty();
    const double odom_rate = static_cast<double>(valid_odom_stamps.size()) / denominator_s;
    const double high_rate = static_cast<double>(valid_high_stamps.size()) / denominator_s;

    std::optional<bool> odom_check;
    std::optional<bool> high_check;
    std::optional<bool> coverage_check;
    std::optional<bool> hard_pass;
    if (ready) {
      odom_check.emplace(odom_rate >= config_.odom_min_hz);
      high_check.emplace(high_rate >= config_.high_min_hz);
      coverage_check.emplace(
        coverage_ratio.has_value() && *coverage_ratio >= config_.coverage_min);
      hard_pass.emplace(*odom_check && *high_check && *coverage_check);
    }
    std::optional<bool> integration_pass;
    if (ready && info_observable) {
      integration_pass = integration_rate >= config_.integration_min_hz;
    }
    std::optional<bool> candidate_pass;
    if (ready) {
      candidate_pass = config_.require_integration_proxy ?
        (*hard_pass && integration_pass.has_value() && *integration_pass) : *hard_pass;
    }

    const auto odom_reason_counts = reasonCounts(odom_window);
    const auto odom_info_reason_counts = reasonCounts(odom_info_window);
    const auto high_reason_counts = reasonCounts(high_window);
    const auto math_missing = setDifference(
      setDifference(math_valid_odom_stamps, matching_nonlost), matching_lost);
    const auto math_lost = setIntersection(math_valid_odom_stamps, matching_lost);
    const auto math_conflicting = setIntersection(
      setIntersection(math_valid_odom_stamps, matching_nonlost), matching_lost);
    const auto missing_high = setDifference(eligible_high_stamps, retained_valid_odom);

    const std::string odom_json = jsonObject({
      {"received_in_window", jsonUInt(odom_window.size())},
      {"math_valid_unique_stamps", jsonUInt(math_valid_odom_stamps.size())},
      {"valid_unique_stamps", jsonUInt(valid_odom_stamps.size())},
      {"valid_unique_rate_hz", jsonDouble(odom_rate)},
      {"duplicate_math_valid_stamp_count", jsonUInt(
          countValid(odom_window) - math_valid_odom_stamps.size())},
      {"math_invalid_count", jsonUInt(odom_window.size() - countValid(odom_window))},
      {"math_reason_counts", countsJson(odom_reason_counts)},
      {"math_valid_missing_odom_info_count", jsonUInt(math_missing.size())},
      {"math_valid_lost_count", jsonUInt(math_lost.size())},
      {"math_valid_conflicting_status_count", jsonUInt(math_conflicting.size())},
      {"header_gap", gapJson(valid_odom_stamps)},
      {"header_regressions_in_window", jsonUInt(regressions_[kOdom].size())},
    });
    const std::string odom_info_json = jsonObject({
      {"received_in_window", jsonUInt(odom_info_window.size())},
      {"tracking_not_lost_unique_stamps", jsonUInt(nonlost_info_window_stamps.size())},
      {"lost_unique_stamps", jsonUInt(lost_info_window_stamps.size())},
      {"reason_counts", countsJson(odom_info_reason_counts)},
      {"header_regressions_in_window", jsonUInt(regressions_[kOdomInfo].size())},
    });
    const std::string high_json = jsonObject({
      {"received_in_window", jsonUInt(high_window.size())},
      {"valid_unique_stamps", jsonUInt(valid_high_stamps.size())},
      {"valid_unique_rate_hz", jsonDouble(high_rate)},
      {"duplicate_valid_stamp_count", jsonUInt(countValid(high_window) - valid_high_stamps.size())},
      {"invalid_count", jsonUInt(high_window.size() - countValid(high_window))},
      {"reason_counts", countsJson(high_reason_counts)},
      {"header_regressions_in_window", jsonUInt(regressions_[kHigh].size())},
    });
    const std::string metrics_json = jsonObject({
      {"odom", odom_json},
      {"odom_info_lite", odom_info_json},
      {"high_camera_info", high_json},
      {"high_to_valid_odom_exact", jsonObject({
          {"eligible_high_unique_stamps", jsonUInt(eligible_high_stamps.size())},
          {"covered_unique_stamps", jsonUInt(covered.size())},
          {"coverage_ratio", jsonDouble(coverage_ratio)},
          {"missing_count", jsonUInt(missing_high.size())},
          {"nearest_stamp_or_restamp_used", "false"},
        })},
      {"high_integrated_proxy", jsonObject({
          {"source", jsonEscape("exact stamp intersection with /info")},
          {"observable", jsonBool(info_observable)},
          {"eligible_duration_s", jsonDouble(eligible_duration_s)},
          {"matched_unique_stamps", jsonUInt(integrated.size())},
          {"rate_hz", info_observable ? jsonDouble(integration_rate) : "null"},
          {"coverage_ratio", info_observable ? jsonDouble(integration_coverage) : "null"},
          {"is_not_database_keyframe_or_dense_map_proof", "true"},
        })},
      {"rtabmap_info", jsonObject({
          {"received_in_window", jsonUInt(info_window.size())},
          {"valid_unique_stamps", jsonUInt(validStamps(info_window).size())},
          {"header_regressions_in_window", jsonUInt(regressions_[kInfo].size())},
        })},
    });
    const std::string gates_json = jsonObject({
      {"hard_checks", jsonObject({
          {"valid_unique_odom_rate_at_least_threshold", jsonBool(odom_check)},
          {"valid_unique_high_camera_info_rate_at_least_threshold", jsonBool(high_check)},
          {"eligible_high_exact_valid_odom_coverage_at_least_threshold", jsonBool(coverage_check)},
        })},
      {"hard_pass", jsonBool(hard_pass)},
      {"integration_proxy_required", jsonBool(config_.require_integration_proxy)},
      {"integration_proxy_pass", jsonBool(integration_pass)},
      {"candidate_pass", jsonBool(candidate_pass)},
    });
    const std::string totals_json = jsonObject({
      {"odom", jsonUInt(total_received_[kOdom])},
      {"odom_info_lite", jsonUInt(total_received_[kOdomInfo])},
      {"high_camera_info", jsonUInt(total_received_[kHigh])},
      {"rtabmap_info", jsonUInt(total_received_[kInfo])},
    });
    const std::string interpretation_json = jsonObject({
      {"odom_valid_means", jsonEscape(
          "same integer-nanosecond stamp has finite-position near-unit-quaternion /odom and "
          "/odom_info_lite lost=false, with any conflicting lost=true excluded; not external truth")},
      {"high_topic_means", jsonEscape(
          "valid 1920x1200 CameraInfo header proxy; RGB/depth payload topics are not subscribed")},
      {"coverage", jsonEscape(
          "integer-nanosecond equality only; no nearest match and no timestamp modification")},
      {"integration_proxy", jsonEscape(
          "/info same stamp indicates a processed callback only; not database persistence")},
    });
    return {
      ready,
      candidate_pass,
      jsonObject({
        {"schema_version", "2"},
        {"ready", jsonBool(ready)},
        {"elapsed_s", jsonDouble(elapsed_s)},
        {"denominator_s", jsonDouble(denominator_s)},
        {"metrics", metrics_json},
        {"gates", gates_json},
        {"totals_since_start", totals_json},
        {"interpretation", interpretation_json},
      })
    };
  }

  const MetricsConfig & config() const noexcept {return config_;}

private:
  static constexpr std::size_t kOdom = 0U;
  static constexpr std::size_t kOdomInfo = 1U;
  static constexpr std::size_t kHigh = 2U;
  static constexpr std::size_t kInfo = 3U;

  static std::int64_t secondsToNs(double seconds)
  {
    const long double scaled = static_cast<long double>(seconds) *
      static_cast<long double>(kNsPerSecond);
    if (scaled > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      throw std::invalid_argument("duration overflows nanoseconds");
    }
    return static_cast<std::int64_t>(std::llround(scaled));
  }

  template<std::size_t SizeT>
  static bool finite(const std::array<double, SizeT> & values)
  {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
      });
  }

  void trackOrder(std::size_t role, std::int64_t stamp_ns, std::int64_t receipt_ns)
  {
    if (stamp_ns <= 0) {
      return;
    }
    if (last_stamp_[role].has_value() && stamp_ns < *last_stamp_[role]) {
      regressions_[role].push_back(receipt_ns);
    }
    last_stamp_[role] = stamp_ns;
  }

  void prune(std::int64_t now_ns)
  {
    const std::int64_t retention_cutoff = now_ns - window_ns_ - match_grace_ns_;
    for (auto * queue : {&odom_, &odom_info_, &high_, &info_}) {
      while (!queue->empty() && queue->front().receipt_ns < retention_cutoff) {
        queue->pop_front();
      }
    }
    const std::int64_t window_cutoff = now_ns - window_ns_;
    for (auto & queue : regressions_) {
      while (!queue.empty() && queue.front() < window_cutoff) {
        queue.pop_front();
      }
    }
  }

  static std::vector<const Sample *> samplesSince(
    const std::deque<Sample> & samples, std::int64_t cutoff)
  {
    std::vector<const Sample *> result;
    result.reserve(samples.size());
    for (const auto & sample : samples) {
      if (sample.receipt_ns >= cutoff) {
        result.push_back(&sample);
      }
    }
    return result;
  }

  static std::set<std::int64_t> validStamps(const std::vector<const Sample *> & samples)
  {
    std::set<std::int64_t> result;
    for (const Sample * sample : samples) {
      if (sample->valid) {
        result.insert(sample->stamp_ns);
      }
    }
    return result;
  }

  static std::set<std::int64_t> reasonStamps(
    const std::vector<const Sample *> & samples, const std::string & reason)
  {
    std::set<std::int64_t> result;
    for (const Sample * sample : samples) {
      if (sample->reason == reason) {
        result.insert(sample->stamp_ns);
      }
    }
    return result;
  }

  static std::set<std::int64_t> setIntersection(
    const std::set<std::int64_t> & lhs, const std::set<std::int64_t> & rhs)
  {
    std::set<std::int64_t> result;
    std::set_intersection(
      lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
      std::inserter(result, result.end()));
    return result;
  }

  static std::set<std::int64_t> setDifference(
    const std::set<std::int64_t> & lhs, const std::set<std::int64_t> & rhs)
  {
    std::set<std::int64_t> result;
    std::set_difference(
      lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
      std::inserter(result, result.end()));
    return result;
  }

  static std::size_t countValid(const std::vector<const Sample *> & samples)
  {
    return static_cast<std::size_t>(std::count_if(
        samples.begin(), samples.end(), [](const Sample * sample) {return sample->valid;}));
  }

  static std::map<std::string, std::uint64_t> reasonCounts(
    const std::vector<const Sample *> & samples)
  {
    std::map<std::string, std::uint64_t> result;
    for (const Sample * sample : samples) {
      ++result[sample->reason];
    }
    return result;
  }

  static std::string countsJson(const std::map<std::string, std::uint64_t> & values)
  {
    std::vector<std::pair<std::string, std::string>> fields;
    fields.reserve(values.size());
    for (const auto & value : values) {
      fields.emplace_back(value.first, jsonUInt(value.second));
    }
    return jsonObject(fields);
  }

  static std::optional<double> percentile(std::vector<double> values, double quantile)
  {
    if (values.empty()) {
      return std::nullopt;
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1U) {
      return values.front();
    }
    const double position = quantile * static_cast<double>(values.size() - 1U);
    const auto low = static_cast<std::size_t>(std::floor(position));
    const auto high = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(low);
    return values[low] * (1.0 - fraction) + values[high] * fraction;
  }

  static std::string gapJson(const std::set<std::int64_t> & stamps)
  {
    std::vector<double> gaps;
    if (stamps.size() > 1U) {
      auto previous = stamps.begin();
      for (auto current = std::next(previous); current != stamps.end(); ++current) {
        gaps.push_back(static_cast<double>(*current - *previous) / 1000000.0);
        previous = current;
      }
    }
    std::optional<double> maximum;
    if (!gaps.empty()) {
      maximum.emplace(*std::max_element(gaps.begin(), gaps.end()));
    }
    return jsonObject({
      {"p50_ms", jsonDouble(percentile(gaps, 0.50))},
      {"p95_ms", jsonDouble(percentile(gaps, 0.95))},
      {"max_ms", jsonDouble(maximum)},
    });
  }

  std::int64_t start_monotonic_ns_{};
  MetricsConfig config_;
  std::int64_t window_ns_{};
  std::int64_t match_grace_ns_{};
  std::int64_t match_settle_ns_{};
  std::deque<Sample> odom_;
  std::deque<Sample> odom_info_;
  std::deque<Sample> high_;
  std::deque<Sample> info_;
  std::array<std::deque<std::int64_t>, 4> regressions_;
  std::array<std::optional<std::int64_t>, 4> last_stamp_;
  std::array<std::uint64_t, 4> total_received_{};
};

}  // namespace wbrov_header_health

#endif  // WBROV_ROLLING_HEADER_METRICS_HPP_
