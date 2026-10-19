#include "RecordedLookaheadWindow.h"
#include <cassert>
#include <future>
template<class F> bool fails(F f){try{f();return false;}catch(const std::runtime_error&){return true;}}
int main() {
    RecordedLookaheadWindow<std::future<int>> window;std::promise<int> a,b,c,d;
    assert(!window.ready());assert(fails([&]{window.begin(0,2,"gap");}));
    window.begin(0,1,"pins1");window.add(1,100,a.get_future());
    window.begin(0,2,"pins2");window.add(2,200,b.get_future());
    window.begin(0,3,"pins3");window.add(3,300,c.get_future());
    assert(fails([&]{window.begin(0,4,"bound");}));
    window.require(2,"pins2");assert(fails([&]{window.require(2,"changed");}));
    assert(fails([&]{window.requireFront(2,"pins2");}));
    assert(fails([&]{window.take(2,200);}));assert(fails([&]{window.finish(2);}));
    window.requireFront(1,"pins1");assert(fails([&]{window.finish(1);}));
    a.set_value(10);assert(window.take(1,100).get()==10);window.finish(1);
    window.begin(1,4,"pins4");window.add(4,400,d.get_future());
    assert(!window.contains(1) && window.contains(4));
    b.set_value(20);assert(window.take(2,200).get()==20);window.finish(2);
    c.set_value(30);assert(window.take(3,300).get()==30);window.finish(3);
    d.set_exception(std::make_exception_ptr(std::runtime_error("failed odometry")));
    assert(fails([&]{window.take(4,400).get();}));window.finish(4);assert(!window.ready());
    // A live producer may have no sealed lookahead for several commands.
    // It can resume at the next contiguous input without filling fake frames.
    window.begin(9,10,"missing-imu");window.requireFront(10,"missing-imu");window.finish(10);
    assert(!window.ready());
}
