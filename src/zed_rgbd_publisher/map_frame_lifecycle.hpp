#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>

namespace zed_dual_stream
{

enum class MapFrameState
{
  kPending,
  kReadyToPublish,
  kPublishing,
  kPublished,
  kTimedOut,
  kInvalidDropped,
  kShutdownCensored,
  kPublishFailed
};

enum class OdomReceiptDisposition
{
  kUnknownStamp,
  kOnTimeValidReady,
  kLateValidCausedTimeout,
  kLateValidAfterTimeout,
  kDuplicateValidBeforePublish,
  kDuplicateValidAfterPublish,
  kValidAfterInvalidConflict,
  kValidAfterShutdown,
  kValidAfterPublishFailure,
  kInvalidPendingDropped,
  kInvalidBeforePublishConflict,
  kInvalidAfterPublishConflict,
  kLateInvalidCausedTimeout,
  kLateInvalidAfterTimeout,
  kDuplicateInvalid,
  kInvalidAfterShutdown,
  kInvalidAfterPublishFailure
};

struct MapFrameRecord
{
  std::uint64_t stamp_ns{};
  std::uint64_t planned_sequence{};
  std::int64_t queued_steady_ns{};
  std::int64_t receipt_steady_ns{};
  std::int64_t terminal_steady_ns{};
  MapFrameState state{MapFrameState::kPending};
  bool conflict_poisoned{};
};

inline const char *map_frame_state_name(MapFrameState state)
{
  switch (state) {
    case MapFrameState::kPending: return "pending";
    case MapFrameState::kReadyToPublish: return "ready_to_publish";
    case MapFrameState::kPublishing: return "publishing";
    case MapFrameState::kPublished: return "published";
    case MapFrameState::kTimedOut: return "timed_out";
    case MapFrameState::kInvalidDropped: return "invalid_dropped";
    case MapFrameState::kShutdownCensored: return "shutdown_censored";
    case MapFrameState::kPublishFailed: return "publish_failed";
  }
  throw std::logic_error("unreachable map frame state");
}

// Single-executor state machine. It owns deadline, odom verdict and publish
// phase so a callback cannot acknowledge first and check the deadline later.
class MapFrameLifecycle
{
public:
  explicit MapFrameLifecycle(
    std::int64_t timeout_ns, std::size_t tombstone_capacity = 2048)
  : timeout_ns_(timeout_ns), tombstone_capacity_(tombstone_capacity)
  {
    if (timeout_ns_ <= 0 || tombstone_capacity_ == 0) {
      throw std::invalid_argument("invalid map frame lifecycle configuration");
    }
  }

  std::optional<MapFrameRecord> queue(
    std::uint64_t stamp_ns, std::uint64_t planned_sequence,
    std::int64_t queued_steady_ns)
  {
    if (active_ || stamp_ns == 0 || planned_sequence == 0 || queued_steady_ns < 0 ||
      find_tombstone(stamp_ns) != nullptr)
    {
      throw std::logic_error("invalid, overlapping or reused map stamp queue");
    }
    active_ = MapFrameRecord{
      stamp_ns, planned_sequence, queued_steady_ns, 0, 0,
      MapFrameState::kPending, false};
    return evict_if_full();
  }

  OdomReceiptDisposition resolve_valid(
    std::uint64_t stamp_ns, std::int64_t receipt_steady_ns)
  {
    if (receipt_steady_ns < 0) {
      throw std::logic_error("negative valid receipt steady time");
    }
    if (active_ && active_->stamp_ns == stamp_ns) {
      if (active_->state == MapFrameState::kPending) {
        if (receipt_steady_ns < active_->queued_steady_ns) {
          throw std::logic_error("valid receipt predates map queue");
        }
        active_->receipt_steady_ns = receipt_steady_ns;
        if (receipt_steady_ns - active_->queued_steady_ns <= timeout_ns_) {
          active_->state = MapFrameState::kReadyToPublish;
          return OdomReceiptDisposition::kOnTimeValidReady;
        }
        active_->state = MapFrameState::kTimedOut;
        active_->terminal_steady_ns = receipt_steady_ns;
        finalize_active();
        return OdomReceiptDisposition::kLateValidCausedTimeout;
      }
      if (active_->state == MapFrameState::kReadyToPublish ||
        active_->state == MapFrameState::kPublishing)
      {
        return OdomReceiptDisposition::kDuplicateValidBeforePublish;
      }
      throw std::logic_error("unreachable active valid receipt state");
    }
    MapFrameRecord *record = find_tombstone(stamp_ns);
    if (!record) {
      return OdomReceiptDisposition::kUnknownStamp;
    }
    switch (record->state) {
      case MapFrameState::kPublished:
        return OdomReceiptDisposition::kDuplicateValidAfterPublish;
      case MapFrameState::kTimedOut:
        return OdomReceiptDisposition::kLateValidAfterTimeout;
      case MapFrameState::kInvalidDropped:
        record->conflict_poisoned = true;
        return OdomReceiptDisposition::kValidAfterInvalidConflict;
      case MapFrameState::kShutdownCensored:
        return OdomReceiptDisposition::kValidAfterShutdown;
      case MapFrameState::kPublishFailed:
        return OdomReceiptDisposition::kValidAfterPublishFailure;
      default:
        throw std::logic_error("nonterminal map record stored as tombstone");
    }
  }

  OdomReceiptDisposition resolve_invalid(
    std::uint64_t stamp_ns, std::int64_t receipt_steady_ns)
  {
    if (receipt_steady_ns < 0) {
      throw std::logic_error("negative invalid receipt steady time");
    }
    if (active_ && active_->stamp_ns == stamp_ns) {
      if (receipt_steady_ns < active_->queued_steady_ns) {
        throw std::logic_error("invalid receipt predates map queue");
      }
      if (active_->state == MapFrameState::kPending) {
        if (receipt_steady_ns - active_->queued_steady_ns > timeout_ns_) {
          active_->state = MapFrameState::kTimedOut;
          active_->receipt_steady_ns = receipt_steady_ns;
          active_->terminal_steady_ns = receipt_steady_ns;
          finalize_active();
          return OdomReceiptDisposition::kLateInvalidCausedTimeout;
        }
        active_->state = MapFrameState::kInvalidDropped;
        active_->receipt_steady_ns = receipt_steady_ns;
        active_->terminal_steady_ns = receipt_steady_ns;
        finalize_active();
        return OdomReceiptDisposition::kInvalidPendingDropped;
      }
      if (active_->state == MapFrameState::kReadyToPublish) {
        active_->state = MapFrameState::kInvalidDropped;
        active_->receipt_steady_ns = receipt_steady_ns;
        active_->terminal_steady_ns = receipt_steady_ns;
        active_->conflict_poisoned = true;
        finalize_active();
        return OdomReceiptDisposition::kInvalidBeforePublishConflict;
      }
      if (active_->state == MapFrameState::kPublishing) {
        throw std::logic_error("invalid receipt cannot interleave single-threaded publish");
      }
      throw std::logic_error("unreachable active invalid receipt state");
    }
    MapFrameRecord *record = find_tombstone(stamp_ns);
    if (!record) {
      return OdomReceiptDisposition::kUnknownStamp;
    }
    switch (record->state) {
      case MapFrameState::kPublished:
        record->conflict_poisoned = true;
        return OdomReceiptDisposition::kInvalidAfterPublishConflict;
      case MapFrameState::kTimedOut:
        return OdomReceiptDisposition::kLateInvalidAfterTimeout;
      case MapFrameState::kInvalidDropped:
        return OdomReceiptDisposition::kDuplicateInvalid;
      case MapFrameState::kShutdownCensored:
        return OdomReceiptDisposition::kInvalidAfterShutdown;
      case MapFrameState::kPublishFailed:
        return OdomReceiptDisposition::kInvalidAfterPublishFailure;
      default:
        throw std::logic_error("nonterminal map record stored as tombstone");
    }
  }

  std::optional<MapFrameRecord> expire(std::int64_t steady_now_ns)
  {
    if (!active_ || active_->state != MapFrameState::kPending ||
      steady_now_ns < active_->queued_steady_ns ||
      steady_now_ns - active_->queued_steady_ns <= timeout_ns_)
    {
      return std::nullopt;
    }
    active_->state = MapFrameState::kTimedOut;
    active_->terminal_steady_ns = steady_now_ns;
    const MapFrameRecord result = *active_;
    finalize_active();
    return result;
  }

  void begin_publish(std::uint64_t stamp_ns)
  {
    require_active(stamp_ns, MapFrameState::kReadyToPublish);
    active_->state = MapFrameState::kPublishing;
  }

  MapFrameRecord commit_publish(std::uint64_t stamp_ns, std::int64_t steady_now_ns)
  {
    require_active(stamp_ns, MapFrameState::kPublishing);
    active_->state = MapFrameState::kPublished;
    active_->terminal_steady_ns = steady_now_ns;
    const MapFrameRecord result = *active_;
    finalize_active();
    return result;
  }

  MapFrameRecord fail_publish(std::uint64_t stamp_ns, std::int64_t steady_now_ns)
  {
    require_active(stamp_ns, MapFrameState::kPublishing);
    active_->state = MapFrameState::kPublishFailed;
    active_->terminal_steady_ns = steady_now_ns;
    const MapFrameRecord result = *active_;
    finalize_active();
    return result;
  }

  std::optional<MapFrameRecord> shutdown_censor(std::int64_t steady_now_ns)
  {
    if (!active_) {
      return std::nullopt;
    }
    active_->state = active_->state == MapFrameState::kPublishing ?
      MapFrameState::kPublishFailed : MapFrameState::kShutdownCensored;
    active_->terminal_steady_ns = steady_now_ns;
    const MapFrameRecord result = *active_;
    finalize_active();
    return result;
  }

  bool active() const {return active_.has_value();}
  bool ready() const
  {
    return active_ && active_->state == MapFrameState::kReadyToPublish;
  }
  const MapFrameRecord *active_record() const {return active_ ? &*active_ : nullptr;}
  const MapFrameRecord *lookup(std::uint64_t stamp_ns) const
  {
    if (active_ && active_->stamp_ns == stamp_ns) {
      return &*active_;
    }
    return find_tombstone(stamp_ns);
  }

private:
  void require_active(std::uint64_t stamp_ns, MapFrameState expected) const
  {
    if (!active_ || active_->stamp_ns != stamp_ns || active_->state != expected) {
      throw std::logic_error("map frame publish transition mismatch");
    }
  }

  std::optional<MapFrameRecord> evict_if_full()
  {
    if (tombstones_.size() < tombstone_capacity_) {
      return std::nullopt;
    }
    const MapFrameRecord evicted = tombstones_.front();
    tombstones_.pop_front();
    return evicted;
  }

  void finalize_active()
  {
    if (!active_) {
      throw std::logic_error("cannot finalize missing map frame");
    }
    if (tombstones_.size() == tombstone_capacity_) {
      tombstones_.pop_front();
    }
    tombstones_.push_back(*active_);
    active_.reset();
  }

  MapFrameRecord *find_tombstone(std::uint64_t stamp_ns)
  {
    for (auto &record : tombstones_) {
      if (record.stamp_ns == stamp_ns) return &record;
    }
    return nullptr;
  }

  const MapFrameRecord *find_tombstone(std::uint64_t stamp_ns) const
  {
    for (const auto &record : tombstones_) {
      if (record.stamp_ns == stamp_ns) return &record;
    }
    return nullptr;
  }

  std::int64_t timeout_ns_{};
  std::size_t tombstone_capacity_{};
  std::optional<MapFrameRecord> active_;
  std::deque<MapFrameRecord> tombstones_;
};

}  // namespace zed_dual_stream
