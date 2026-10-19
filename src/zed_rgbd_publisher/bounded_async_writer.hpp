#pragma once
#include <condition_variable>
#include <chrono>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

// Only the worker invokes the sink. A slow disk cannot hold the producer lock.
// Overflow/errors stop acceptance explicitly; no accepted record is discarded
// to make room and close() drains accepted records before returning.
template<class Record> class BoundedAsyncWriter {
public:
  struct Snapshot {
    std::size_t queued=0,highWater=0,accepted=0,completed=0,waits=0;
    double waitMs=0,maxWaitMs=0;
  };
  Snapshot snapshot() {std::lock_guard<std::mutex> lock(mutex_);auto s=stats_;s.queued=queue_.size();return s;}
  BoundedAsyncWriter(std::size_t capacity, std::function<void(const Record &)> sink)
    : capacity_(capacity), sink_(std::move(sink)) {
    if(!capacity_ || !sink_)throw std::invalid_argument("invalid journal writer");
    worker_=std::thread([this]{run();});
  }
  BoundedAsyncWriter(const BoundedAsyncWriter &)=delete;
  BoundedAsyncWriter &operator=(const BoundedAsyncWriter &)=delete;
  ~BoundedAsyncWriter() {try {close();}catch(...) {}}
  void append(Record record) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(error_)std::rethrow_exception(error_);
      if(closing_)throw std::runtime_error("recording journal already closing");
      if(queue_.size()>=capacity_)throw std::runtime_error("recording journal backlog limit reached");
      queue_.push_back(std::move(record));
      accepted();
    }
    ready_.notify_one();
  }
  // Backpressure retains the CURRENT captured record while the bounded queue
  // drains. No oldest/newest frame is dropped and capacity stays unchanged.
  // A zero timeout waits for capacity or an actual sink error/close. This is
  // used by lossless capture: elapsed time is not evidence of corrupt input.
  bool append_wait(Record record, std::chrono::milliseconds timeout) {
    bool waited=false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if(queue_.size()>=capacity_ && !closing_ && !error_) {
        waited=true;
        const auto begin=std::chrono::steady_clock::now();
        const auto available=[this]{return error_ || closing_ || queue_.size()<capacity_;};
        if(timeout.count()==0)space_.wait(lock,available);
        else if(!space_.wait_for(lock,timeout,available))
          throw std::runtime_error("recording writer backpressure timeout");
        const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        ++stats_.waits;stats_.waitMs+=ms;if(ms>stats_.maxWaitMs)stats_.maxWaitMs=ms;
      }
      if(error_)std::rethrow_exception(error_);
      if(closing_)throw std::runtime_error("recording writer already closing");
      queue_.push_back(std::move(record));
      accepted();
    }
    ready_.notify_one();return waited;
  }
  void close() {
    {std::lock_guard<std::mutex> lock(mutex_);closing_=true;}
    ready_.notify_one();
    space_.notify_all();
    if(worker_.joinable())worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    if(error_)std::rethrow_exception(error_);
  }
private:
  void accepted() {++stats_.accepted;if(queue_.size()>stats_.highWater)stats_.highWater=queue_.size();}
  void run() noexcept {
    try {
      for(;;) {
        Record record;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          ready_.wait(lock,[this]{return closing_ || !queue_.empty();});
          if(queue_.empty())return;
          record=std::move(queue_.front());queue_.pop_front();
        }
        space_.notify_one();
        sink_(record);
        {std::lock_guard<std::mutex> lock(mutex_);++stats_.completed;}
      }
    } catch(...) {
      {std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();closing_=true;}
      space_.notify_all();
    }
  }
  const std::size_t capacity_;
  std::function<void(const Record &)> sink_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::condition_variable space_;
  std::deque<Record> queue_;
  Snapshot stats_;
  std::exception_ptr error_;
  bool closing_=false;
  std::thread worker_;
};
