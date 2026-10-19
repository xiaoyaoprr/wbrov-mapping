#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <deque>
#include <exception>
#include <stdexcept>

// Backpressure is allowed here: this is the disk replay worker, never the
// camera. FIFO jobs are retained up to a fixed bound. A failed job prevents
// acknowledgement of the input segment and propagates to the supervisor.
class OrderedProcessingLane {
public:
    explicit OrderedProcessingLane(std::function<void()> initialize={},std::size_t capacity=64):capacity_(capacity) {
        if(!capacity_)throw std::invalid_argument("empty processing queue");
        worker_=std::thread([this,initialize]{
            try {
                if(initialize)initialize();
                for(;;) {
                    std::function<void()> job;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        changed_.wait(lock,[&]{return closing_ || !queue_.empty();});
                        if(queue_.empty())return;
                        job=std::move(queue_.front());queue_.pop_front();busy_=true;changed_.notify_all();
                    }
                    job();
                    {std::lock_guard<std::mutex> lock(mutex_);busy_=false;changed_.notify_all();}
                }
            }catch(...) {
                std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();closing_=true;busy_=false;changed_.notify_all();
            }
        });
    }
    ~OrderedProcessingLane(){try{close();}catch(...){}}
    void submit(std::function<void()> job) {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock,[&]{return closing_ || error_ || queue_.size()<capacity_;});
        if(error_)std::rethrow_exception(error_);
        if(closing_)throw std::runtime_error("processing lane closed");
        queue_.push_back(std::move(job));changed_.notify_all();
    }
    void drain() {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock,[&]{return error_ || (queue_.empty() && !busy_);});
        if(error_)std::rethrow_exception(error_);
    }
    void close() {
        {std::lock_guard<std::mutex> lock(mutex_);closing_=true;changed_.notify_all();}
        if(worker_.joinable())worker_.join();
        if(error_)std::rethrow_exception(error_);
    }
private:
    std::size_t capacity_;std::mutex mutex_;std::condition_variable changed_;
    std::deque<std::function<void()>> queue_;std::thread worker_;bool busy_=false,closing_=false;std::exception_ptr error_;
};
