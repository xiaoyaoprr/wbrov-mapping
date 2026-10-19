#include "OrderedProcessingLane.h"
#include <cassert>
#include <vector>
#include <chrono>
#include <future>
int main() {
    std::vector<int> result;OrderedProcessingLane lane({},2);
    for(int i=0;i<20;++i)lane.submit([&,i]{std::this_thread::sleep_for(std::chrono::milliseconds(2));result.push_back(i);});
    lane.drain();assert(result.size()==20);for(int i=0;i<20;++i)assert(result[i]==i);lane.close();
    OrderedProcessingLane failed({},2);failed.submit([]{throw std::runtime_error("input failure");});
    bool rejected=false;try{failed.drain();}catch(const std::runtime_error &){rejected=true;}assert(rejected);
    rejected=false;try{failed.submit([]{});}catch(const std::runtime_error &){rejected=true;}assert(rejected);

    // Queue a complete sealed segment without waiting for feature extraction.
    // The odometry owner waits on each future and retains exact FIFO order.
    std::promise<void> start;auto released=start.get_future().share();
    OrderedProcessingLane features({},64),odometry({},64);std::vector<int> measured;
    for(int i=0;i<64;++i) {
        auto p=std::make_shared<std::promise<int>>();auto ready=p->get_future().share();
        features.submit([released,p,i]{released.wait();p->set_value(i);});
        odometry.submit([ready,&measured]{measured.push_back(ready.get());});
    }
    assert(measured.empty());start.set_value();features.drain();odometry.drain();
    assert(measured.size()==64);for(int i=0;i<64;++i)assert(measured[i]==i);
    features.close();odometry.close();

    OrderedProcessingLane badFeatures({},64),badOdometry({},64);
    auto p=std::make_shared<std::promise<int>>();auto ready=p->get_future().share();
    badFeatures.submit([p]{p->set_exception(std::make_exception_ptr(std::runtime_error("feature extraction failed")));});
    badOdometry.submit([ready]{ready.get();});
    rejected=false;try{badOdometry.drain();}catch(const std::runtime_error &){rejected=true;}assert(rejected);
}
