#include "RecordedLookahead.h"
#include <cassert>
#include <future>
template<class F> bool fails(F f){try{f();return false;}catch(const std::runtime_error &){return true;}}
int main() {
    RecordedLookahead<int> cache;
    assert(!cache.ready());assert(fails([&]{cache.take(1);}));
    cache.begin("segment0:all-pins");cache.add(1,10);cache.add(2,20);
    assert(fails([&]{cache.begin("segment1");}));
    assert(fails([&]{cache.require("segment0:changed-pins");}));
    assert(fails([&]{cache.finish();}));
    cache.require("segment0:all-pins");assert(cache.take(1)==10);
    assert(fails([&]{cache.take(1);}));assert(cache.take(2)==20);cache.finish();assert(!cache.ready());
    cache.begin("missing-IMU-only");cache.finish();
    cache.begin("bounded");for(int n=0;n<64;++n)cache.add(n,n);
    assert(fails([&]{cache.add(64,64);}));for(int n=0;n<64;++n)assert(cache.take(n)==n);cache.finish();

    // A measured pose may still be in flight when the supervisor admits its
    // exact segment. Move-only futures keep source ownership and exceptions.
    RecordedLookahead<std::future<int>> pending;
    std::promise<int> first,second;pending.begin("verified-next");
    pending.add(1,first.get_future());pending.add(2,second.get_future());
    assert(fails([&]{pending.require("changed-next");}));
    pending.require("verified-next");auto result=pending.take(1);
    assert(result.wait_for(std::chrono::seconds(0))==std::future_status::timeout);
    first.set_value(10);assert(result.get()==10);
    second.set_exception(std::make_exception_ptr(std::runtime_error("odometry failed")));
    assert(fails([&]{pending.take(2).get();}));pending.finish();
}
