#pragma once

#include <cstdint>
#include <stdexcept>

namespace zed_dual_stream
{

struct Decision
{
  bool publish_odom{};
  bool publish_map{};
};

class Schedule
{
public:
  Schedule(double odom_hz, double map_hz)
  : odom_period_ns_(period_ns(odom_hz)), map_period_ns_(period_ns(map_hz))
  {
    if (map_hz > odom_hz) {
      throw std::invalid_argument("map publish rate cannot exceed odom publish rate");
    }
  }

  Decision preview(std::int64_t steady_now_ns)
  {
    if (!initialized_) {
      initialized_ = true;
      next_odom_ns_ = steady_now_ns;
      next_map_ns_ = steady_now_ns;
    }
    const bool map_due = steady_now_ns >= next_map_ns_;
    const bool odom_due = map_due || steady_now_ns >= next_odom_ns_;
    return {odom_due, map_due};
  }

  void commit(const Decision &decision, std::int64_t steady_now_ns)
  {
    const auto expected = preview(steady_now_ns);
    if (decision.publish_odom != expected.publish_odom ||
        decision.publish_map != expected.publish_map)
    {
      throw std::logic_error("cannot commit a stale dual-stream schedule decision");
    }
    if (decision.publish_map) {
      advance(next_map_ns_, map_period_ns_, steady_now_ns);
    }
    if (decision.publish_odom) {
      advance(next_odom_ns_, odom_period_ns_, steady_now_ns);
    }
  }

private:
  static std::int64_t period_ns(double hz)
  {
    if (!(hz > 0.0 && hz <= 1000.0)) {
      throw std::invalid_argument("publish rate must be in (0, 1000] Hz");
    }
    return static_cast<std::int64_t>(1.0e9 / hz + 0.5);
  }

  static void advance(
    std::int64_t &deadline, std::int64_t period, std::int64_t now)
  {
    do {
      deadline += period;
    } while (deadline <= now);
  }

  std::int64_t odom_period_ns_{};
  std::int64_t map_period_ns_{};
  std::int64_t next_odom_ns_{};
  std::int64_t next_map_ns_{};
  bool initialized_{};
};

}  // namespace zed_dual_stream
