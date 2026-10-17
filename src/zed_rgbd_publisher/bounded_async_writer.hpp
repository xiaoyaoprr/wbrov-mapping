#pragma once
#include <condition_variable>
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
    }
    ready_.notify_one();
  }
  void close() {
    {std::lock_guard<std::mutex> lock(mutex_);closing_=true;}
    ready_.notify_one();
    if(worker_.joinable())worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    if(error_)std::rethrow_exception(error_);
  }
private:
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
        sink_(record);
      }
    } catch(...) {
      std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();closing_=true;
    }
  }
  const std::size_t capacity_;
  std::function<void(const Record &)> sink_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Record> queue_;
  std::exception_ptr error_;
  bool closing_=false;
  std::thread worker_;
};
