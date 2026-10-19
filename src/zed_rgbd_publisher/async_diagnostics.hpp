#pragma once
#include "bounded_async_writer.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>

// Diagnostics only: no measurement, acknowledgement, or map state uses this queue.
// Sink calls (including ROS logging locks and disk I/O) occur on the worker.
class AsyncDiagnostics {
public:
  struct Record {
    int severity{};
    long long enqueued_steady_ns{};
    std::array<char, 128> logger{};
    std::array<char, 2048> message{};
  };
  struct Counts {std::size_t accepted, emitted, rejected; bool sink_failed;};
  AsyncDiagnostics(std::size_t capacity, std::function<void(const Record &)> sink)
    : writer_(capacity, [this, sink](const Record &r) {sink(r); ++emitted_;}) {}
  // Overflow or formatting failure is explicit in counts and close's result.
  // It must not throw while a camera destructor is preserving its recording.
  void log(int severity, const char *logger, const char *format, ...)
    __attribute__((format(printf, 4, 5))) {
    Record r;
    r.severity=severity;
    r.enqueued_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const int n=std::snprintf(r.logger.data(), r.logger.size(), "%s", logger);
    va_list args; va_start(args, format);
    const int m=std::vsnprintf(r.message.data(), r.message.size(), format, args);
    va_end(args);
    if(n<0 || static_cast<std::size_t>(n)>=r.logger.size() || m<0 ||
       static_cast<std::size_t>(m)>=r.message.size()) {++rejected_; return;}
    try {writer_.append(std::move(r)); ++accepted_;}
    catch(...) {++rejected_;}
  }
  bool close() noexcept {
    try {writer_.close();} catch(...) {sink_failed_=true;}
    return !sink_failed_ && rejected_==0 && emitted_==accepted_;
  }
  Counts counts() const {return {accepted_.load(), emitted_.load(), rejected_.load(), sink_failed_};}
  ~AsyncDiagnostics() {close();}
private:
  std::atomic<std::size_t> accepted_{0}, emitted_{0}, rejected_{0};
  bool sink_failed_=false;
  // Last member: joined before the counters/sink capture are destroyed.
  BoundedAsyncWriter<Record> writer_;
};
