#include "dual_stream_schedule.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace
{
struct Counts {int odom{}; int map{}; int map_without_odom{};};

void require(bool condition, const char *message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

Counts simulate(double grab_hz, double odom_hz, double map_hz, double seconds)
{
  zed_dual_stream::Schedule schedule(odom_hz, map_hz);
  Counts result;
  const auto step = static_cast<std::int64_t>(1.0e9 / grab_hz + 0.5);
  const auto end = static_cast<std::int64_t>(seconds * 1.0e9);
  for (std::int64_t now = 0; now < end; now += step) {
    const auto decision = schedule.preview(now);
    schedule.commit(decision, now);
    result.odom += decision.publish_odom ? 1 : 0;
    result.map += decision.publish_map ? 1 : 0;
    result.map_without_odom +=
      decision.publish_map && !decision.publish_odom ? 1 : 0;
  }
  return result;
}
}  // namespace

int main()
{
  for (double grab_hz : {60.0, 30.0}) {
    const auto counts = simulate(grab_hz, 30.0, 5.0, 10.0);
    require(std::abs(counts.odom - 300) <= 1, "30Hz odom schedule count");
    require(std::abs(counts.map - 50) <= 1, "5Hz map schedule count");
    require(counts.map_without_odom == 0, "map is always an odom frame");
  }
  const auto limited = simulate(20.0, 30.0, 5.0, 10.0);
  require(limited.odom == 200, "grab rate caps odom publication");
  require(std::abs(limited.map - 50) <= 1, "map rate survives 20Hz grab");
  require(limited.map_without_odom == 0, "limited map is always an odom frame");
  const auto slow = simulate(10.0, 30.0, 5.0, 10.0);
  require(slow.odom == 100, "slow grab caps odom publication");
  require(std::abs(slow.map - 50) <= 1, "map rate survives 10Hz grab");
  require(slow.map_without_odom == 0, "slow map is always an odom frame");

  const auto trial = simulate(60.0, 20.0, 0.5, 20.0);
  require(std::abs(trial.odom - 400) <= 1, "trial 20Hz low stream count");
  require(std::abs(trial.map - 10) <= 1, "trial 0.5Hz high stream count");
  require(trial.map_without_odom == 0, "trial high stream is low subset");

  const auto rate_ab = simulate(60.0, 12.0, 1.0, 20.0);
  require(std::abs(rate_ab.odom - 240) <= 1, "rate A/B 12Hz low stream count");
  require(std::abs(rate_ab.map - 20) <= 1, "rate A/B 1Hz high stream count");
  require(rate_ab.map_without_odom == 0, "rate A/B high stream is low subset");

  zed_dual_stream::Schedule retry(20.0, 5.0);
  const auto first = retry.preview(0);
  const auto repeated_after_failure = retry.preview(10000000);
  require(first.publish_odom && first.publish_map, "first frame due");
  require(repeated_after_failure.publish_odom && repeated_after_failure.publish_map,
    "uncommitted failure remains due");
  retry.commit(repeated_after_failure, 10000000);
  const auto after_commit = retry.preview(20000000);
  require(!after_commit.publish_odom && !after_commit.publish_map,
    "commit advances both deadlines");

  bool rejected = false;
  try {
    zed_dual_stream::Schedule invalid(3.0, 15.0);
    (void)invalid;
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "map rate above odom rate rejected");
  std::cout << "DUAL_STREAM_SCHEDULE_TEST_OK\n";
}
