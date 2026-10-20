#pragma once
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

// Both the active record and waiting records count against the limits. Callers
// supply owned byte capacity when records contain buffers. Backpressure remains
// lossless and bounded if storage cannot catch up; this is not unlimited RAM.
template<class Record> class SegmentCommitQueue {
public:
    struct Snapshot { std::size_t pending=0,highWater=0; std::uint64_t bytes=0,highWaterBytes=0; };
    SegmentCommitQueue(std::size_t countLimit,std::uint64_t byteLimit,std::function<void(const Record &)> sink)
        : countLimit_(countLimit),byteLimit_(byteLimit),sink_(std::move(sink)) {
        if(!countLimit || !byteLimit || !sink_)throw std::invalid_argument("invalid commit queue");
        worker_=std::thread([this]{run();});
    }
    SegmentCommitQueue(const SegmentCommitQueue &)=delete;
    SegmentCommitQueue &operator=(const SegmentCommitQueue &)=delete;
    ~SegmentCommitQueue(){try{close();}catch(...){}}
    void check(){std::lock_guard<std::mutex> lock(mutex_);if(error_)std::rethrow_exception(error_);}
    Snapshot snapshot(){std::lock_guard<std::mutex> lock(mutex_);return stats_;}
    void append(Record record,std::uint64_t bytes) {
        if(bytes>byteLimit_)throw std::runtime_error("segment exceeds durability backlog byte limit");
        std::unique_lock<std::mutex> lock(mutex_);
        space_.wait(lock,[&]{return error_ || closing_ || (stats_.pending<countLimit_ && bytes<=byteLimit_-stats_.bytes);});
        if(error_)std::rethrow_exception(error_);
        if(closing_)throw std::runtime_error("commit queue already closing");
        queue_.push_back({std::move(record),bytes});++stats_.pending;stats_.bytes+=bytes;
        if(stats_.pending>stats_.highWater)stats_.highWater=stats_.pending;
        if(stats_.bytes>stats_.highWaterBytes)stats_.highWaterBytes=stats_.bytes;
        ready_.notify_one();
    }
    void close() {
        {std::lock_guard<std::mutex> lock(mutex_);closing_=true;}
        ready_.notify_all();space_.notify_all();
        if(worker_.joinable())worker_.join();
        check();
    }
private:
    void run() noexcept {
        try {
            for(;;) {
                std::pair<Record,std::uint64_t> item;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    ready_.wait(lock,[this]{return closing_ || !queue_.empty();});
                    if(queue_.empty())return;
                    item=std::move(queue_.front());queue_.pop_front();
                }
                sink_(item.first);
                item.first=Record{}; // release owned buffers before admitting more
                {std::lock_guard<std::mutex> lock(mutex_);--stats_.pending;stats_.bytes-=item.second;}
                space_.notify_all();
            }
        } catch(...) {
            {std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();closing_=true;}
            space_.notify_all();
        }
    }
    const std::size_t countLimit_;const std::uint64_t byteLimit_;
    std::function<void(const Record &)> sink_;std::mutex mutex_;
    std::condition_variable ready_,space_;std::deque<std::pair<Record,std::uint64_t>> queue_;
    Snapshot stats_;std::exception_ptr error_;bool closing_=false;std::thread worker_;
};
