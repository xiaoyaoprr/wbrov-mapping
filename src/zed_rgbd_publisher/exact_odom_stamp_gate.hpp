#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>

namespace zed_dual_stream
{

class ExactOdomStampGate
{
public:
  explicit ExactOdomStampGate(std::int64_t timeout_ns, std::size_t history_size = 128)
  : timeout_ns_(timeout_ns), history_size_(history_size)
  {
    if (timeout_ns_ <= 0 || history_size_ == 0) {
      throw std::invalid_argument("exact odom stamp gate configuration is invalid");
    }
  }

  void acknowledge(std::uint64_t stamp_ns)
  {
    if (stamp_ns == 0) {
      return;
    }
    if (recent_.empty() || recent_.back() != stamp_ns) {
      recent_.push_back(stamp_ns);
      while (recent_.size() > history_size_) {
        recent_.pop_front();
      }
    }
    ready_ = pending_ && pending_stamp_ns_ == stamp_ns;
  }

  void queue(std::uint64_t stamp_ns, std::int64_t steady_now_ns)
  {
    if (pending_ || stamp_ns == 0 || steady_now_ns < 0) {
      throw std::logic_error("cannot queue invalid or overlapping map frame");
    }
    pending_ = true;
    pending_stamp_ns_ = stamp_ns;
    queued_steady_ns_ = steady_now_ns;
    ready_ = std::find(recent_.begin(), recent_.end(), stamp_ns) != recent_.end();
  }

  bool expire(std::int64_t steady_now_ns)
  {
    if (!pending_ || ready_ || steady_now_ns - queued_steady_ns_ <= timeout_ns_) {
      return false;
    }
    clear();
    return true;
  }

  bool pending() const {return pending_;}
  bool ready() const {return pending_ && ready_;}
  std::uint64_t pending_stamp_ns() const {return pending_stamp_ns_;}
  std::int64_t queued_steady_ns() const {return queued_steady_ns_;}

  bool cancel(std::uint64_t stamp_ns)
  {
    if (!pending_ || pending_stamp_ns_ != stamp_ns) {
      return false;
    }
    clear();
    return true;
  }

  void consume()
  {
    if (!ready()) {
      throw std::logic_error("cannot consume an unacknowledged map frame");
    }
    clear();
  }

private:
  void clear()
  {
    pending_ = false;
    ready_ = false;
    pending_stamp_ns_ = 0;
    queued_steady_ns_ = 0;
  }

  std::int64_t timeout_ns_{};
  std::size_t history_size_{};
  std::deque<std::uint64_t> recent_;
  bool pending_{};
  bool ready_{};
  std::uint64_t pending_stamp_ns_{};
  std::int64_t queued_steady_ns_{};
};

}  // namespace zed_dual_stream
