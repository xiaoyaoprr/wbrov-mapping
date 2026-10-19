#pragma once
#include "RecordedTaskPool.h"
#include <algorithm>
#include <functional>
#include <memory>

// Preparation runs on the caller, where Core/cache access is serialized.
// The resulting CPU job must own its input. Only the oldest prepared ID may
// be consumed, so completion order cannot change voxel collision winners.
template<class Result> class RecordedOrderedPrefetch {
public:
    using Job=std::function<Result()>;
    using Prepare=std::function<Job(int)>;
    RecordedOrderedPrefetch(std::vector<int> ids,Prepare prepare,std::size_t workers=2)
        :ids_(std::move(ids)),prepare_(std::move(prepare)),workers_(workers) {
        if(workers<1 || workers>3 || !std::is_sorted(ids_.begin(),ids_.end()) ||
           std::adjacent_find(ids_.begin(),ids_.end())!=ids_.end())throw std::invalid_argument("invalid ordered export plan");
        // Tiny updates cost less without creating threads or copying ahead.
        if(ids_.size()<workers*2)return;
        pool_.reset(new RecordedTaskPool<Result>(workers));
        while(ready_.size()<workers_ && next_<ids_.size())schedule();
    }
    bool parallel() const{return bool(pool_);}
    bool prepared(int id) const {
        if(ready_.empty())return false;
        if(id>ready_.front().first)throw std::runtime_error("prepared export node skipped");
        return id==ready_.front().first;
    }
    Result take(int id) {
        if(!prepared(id))throw std::runtime_error("export result not prepared");
        auto result=ready_.front().second.get();ready_.pop_front();
        if(next_<ids_.size())schedule();
        return result;
    }
    void finish() const {
        if(!ready_.empty() || (pool_ && next_!=ids_.size()))throw std::runtime_error("unconsumed export result");
    }
private:
    void schedule() {
        const int id=ids_[next_++];auto job=prepare_(id);
        ready_.emplace_back(id,pool_->submit(std::move(job)));
    }
    std::vector<int> ids_;Prepare prepare_;std::size_t workers_,next_=0;
    // Destruction drains the pool even if the caller exits on an exception.
    std::unique_ptr<RecordedTaskPool<Result>> pool_;
    std::deque<std::pair<int,std::future<Result>>> ready_;
};
