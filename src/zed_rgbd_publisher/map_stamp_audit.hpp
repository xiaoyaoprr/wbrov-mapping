#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>

namespace zed_dual_stream
{

enum class MapStampState
{
  kPending,
  kAcknowledged,
  kTimedOut,
  kInvalidDropped,
  kShutdownCensored
};

enum class MapStampObservation
{
  kUnknownStamp,
  kExactValidPending,
  kDuplicateValidAfterAck,
  kLateValidAfterTimeout,
  kLateValidAfterShutdown,
  kValidAfterInvalidConflict,
  kInvalidPending,
  kDuplicateInvalid,
  kLateInvalidAfterTimeout,
  kInvalidAfterValidConflict,
  kInvalidAfterShutdown
};

inline const char *map_stamp_state_name(MapStampState state)
{
  switch (state) {
    case MapStampState::kPending:
      return "pending";
    case MapStampState::kAcknowledged:
      return "acknowledged";
    case MapStampState::kTimedOut:
      return "timed_out";
    case MapStampState::kInvalidDropped:
      return "invalid_dropped";
    case MapStampState::kShutdownCensored:
      return "shutdown_censored";
  }
  throw std::logic_error("unreachable map stamp state name");
}

struct MapStampRecord
{
  std::uint64_t stamp_ns{};
  std::int64_t queued_steady_ns{};
  std::int64_t terminal_steady_ns{};
  MapStampState state{MapStampState::kPending};
  bool conflict_poisoned{};
};

class MapStampAuditLedger
{
public:
  explicit MapStampAuditLedger(std::size_t capacity = 2048)
  : capacity_(capacity)
  {
    if (capacity_ == 0) {
      throw std::invalid_argument("map stamp audit capacity must be positive");
    }
  }

  std::optional<MapStampRecord> queue(std::uint64_t stamp_ns, std::int64_t steady_ns)
  {
    if (stamp_ns == 0 || steady_ns < 0 || find(stamp_ns) != nullptr) {
      throw std::logic_error("invalid or duplicate map stamp audit queue");
    }
    std::optional<MapStampRecord> evicted;
    if (records_.size() == capacity_) {
      evicted = records_.front();
      records_.pop_front();
    }
    records_.push_back({stamp_ns, steady_ns, 0, MapStampState::kPending, false});
    return evicted;
  }

  MapStampObservation observe_valid(std::uint64_t stamp_ns, std::int64_t steady_ns)
  {
    MapStampRecord *record = find(stamp_ns);
    if (!record) {
      return MapStampObservation::kUnknownStamp;
    }
    switch (record->state) {
      case MapStampState::kPending:
        record->state = MapStampState::kAcknowledged;
        record->terminal_steady_ns = steady_ns;
        return MapStampObservation::kExactValidPending;
      case MapStampState::kAcknowledged:
        return MapStampObservation::kDuplicateValidAfterAck;
      case MapStampState::kTimedOut:
        return MapStampObservation::kLateValidAfterTimeout;
      case MapStampState::kInvalidDropped:
        record->conflict_poisoned = true;
        return MapStampObservation::kValidAfterInvalidConflict;
      case MapStampState::kShutdownCensored:
        return MapStampObservation::kLateValidAfterShutdown;
    }
    throw std::logic_error("unreachable map stamp valid state");
  }

  MapStampObservation observe_invalid(std::uint64_t stamp_ns, std::int64_t steady_ns)
  {
    MapStampRecord *record = find(stamp_ns);
    if (!record) {
      return MapStampObservation::kUnknownStamp;
    }
    switch (record->state) {
      case MapStampState::kPending:
        record->state = MapStampState::kInvalidDropped;
        record->terminal_steady_ns = steady_ns;
        return MapStampObservation::kInvalidPending;
      case MapStampState::kAcknowledged:
        record->conflict_poisoned = true;
        return MapStampObservation::kInvalidAfterValidConflict;
      case MapStampState::kTimedOut:
        return MapStampObservation::kLateInvalidAfterTimeout;
      case MapStampState::kInvalidDropped:
        return MapStampObservation::kDuplicateInvalid;
      case MapStampState::kShutdownCensored:
        return MapStampObservation::kInvalidAfterShutdown;
    }
    throw std::logic_error("unreachable map stamp invalid state");
  }

  bool timeout(std::uint64_t stamp_ns, std::int64_t steady_ns)
  {
    MapStampRecord *record = find(stamp_ns);
    if (!record || record->state != MapStampState::kPending) {
      return false;
    }
    record->state = MapStampState::kTimedOut;
    record->terminal_steady_ns = steady_ns;
    return true;
  }

  bool shutdown_censor(std::uint64_t stamp_ns, std::int64_t steady_ns)
  {
    MapStampRecord *record = find(stamp_ns);
    if (!record || record->state != MapStampState::kPending) {
      return false;
    }
    record->state = MapStampState::kShutdownCensored;
    record->terminal_steady_ns = steady_ns;
    return true;
  }

  const MapStampRecord *lookup(std::uint64_t stamp_ns) const {return find(stamp_ns);}
  std::size_t size() const {return records_.size();}

private:
  MapStampRecord *find(std::uint64_t stamp_ns)
  {
    for (auto &record : records_) {
      if (record.stamp_ns == stamp_ns) {
        return &record;
      }
    }
    return nullptr;
  }

  const MapStampRecord *find(std::uint64_t stamp_ns) const
  {
    for (const auto &record : records_) {
      if (record.stamp_ns == stamp_ns) {
        return &record;
      }
    }
    return nullptr;
  }

  std::size_t capacity_{};
  std::deque<MapStampRecord> records_;
};

}  // namespace zed_dual_stream
