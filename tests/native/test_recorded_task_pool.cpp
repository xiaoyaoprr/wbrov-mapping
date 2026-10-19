#include "RecordedTaskPool.h"
#include <cassert>
#include <atomic>
#include <chrono>
int main() {
    bool invalid=false;try{RecordedTaskPool<int> bad(0);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
    RecordedTaskPool<int> pool(2);std::promise<void> release;auto gate=release.get_future().share();
    std::mutex mutex;std::condition_variable changed;int started=0;std::atomic<int> active{0},peak{0};
    auto work=[&](int value) {
        const int n=++active;int old=peak.load();while(old<n && !peak.compare_exchange_weak(old,n)){}
        {std::lock_guard<std::mutex> lock(mutex);++started;changed.notify_all();}
        gate.wait();--active;return value;
    };
    auto first=pool.submit([&]{return work(1);});auto second=pool.submit([&]{return work(2);});
    {std::unique_lock<std::mutex> lock(mutex);assert(changed.wait_for(lock,std::chrono::seconds(5),[&]{return started==2;}));}
    auto third=pool.submit([&]{return work(3);});
    assert(third.wait_for(std::chrono::milliseconds(1))==std::future_status::timeout);
    release.set_value();assert(first.get()==1 && second.get()==2 && third.get()==3 && peak==2);
    auto failed=pool.submit([]()->int{throw std::runtime_error("transform failed");});
    bool propagated=false;try{failed.get();}catch(const std::runtime_error&){propagated=true;}assert(propagated);
    assert(pool.submit([]{return 4;}).get()==4);
    auto final=pool.submit([]{return 5;});pool.close();assert(final.get()==5);
    bool closed=false;try{pool.submit([]{return 6;});}catch(const std::runtime_error&){closed=true;}assert(closed);
}
