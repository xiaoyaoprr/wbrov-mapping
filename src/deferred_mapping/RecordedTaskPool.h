#pragma once
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

// Bounded CPU-only work on owned, immutable export inputs. Callers retain the
// futures and publish in original node order. No Core or cache access here.
template<class Result> class RecordedTaskPool {
public:
    explicit RecordedTaskPool(std::size_t workers):capacity_(workers) {
        if(workers<1 || workers>3)throw std::invalid_argument("invalid export thread count");
        threads_.reserve(workers);
        try {
            for(std::size_t i=0;i<workers;++i)threads_.emplace_back([this] {
                for(;;) {
                    std::packaged_task<Result()> task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        changed_.wait(lock,[&]{return closing_ || !queue_.empty();});
                        if(queue_.empty())return;
                        task=std::move(queue_.front());queue_.pop_front();changed_.notify_all();
                    }
                    task(); // packaged_task propagates exceptions to its future
                }
            });
        }catch(...){close();throw;}
    }
    ~RecordedTaskPool(){close();}
    template<class Function> std::future<Result> submit(Function function) {
        std::packaged_task<Result()> task(std::move(function));auto future=task.get_future();
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock,[&]{return closing_ || queue_.size()<capacity_;});
        if(closing_)throw std::runtime_error("export pool is closed");
        queue_.push_back(std::move(task));changed_.notify_all();return future;
    }
    void close() {
        {std::lock_guard<std::mutex> lock(mutex_);closing_=true;changed_.notify_all();}
        for(auto &thread:threads_)if(thread.joinable())thread.join();
    }
private:
    std::size_t capacity_;std::mutex mutex_;std::condition_variable changed_;
    std::deque<std::packaged_task<Result()>> queue_;std::vector<std::thread> threads_;
    bool closing_=false;
};
