#include <rtabmap_slam/GlobalPublicationGate.h>
#include <cassert>
#include <future>
#include <thread>
#include <iostream>

int main() {
    using Gate = rtabmap_slam::GlobalPublicationGate;
    Gate gate; int sends=0;
    auto send = [&] { ++sends; };
    assert(!gate.mapAllowed(0));
    assert(!gate.withFreshPose(100,100,30,send));
    assert(gate.observe(100,100,true));
    assert(!gate.mapAllowed(0)); // tracking alone is not frame registration
    assert(gate.coreUpdated(gate.ticket(),0,true));
    assert(gate.mapAllowed(0) && !gate.mapAllowed(1));
    assert(gate.withFreshPose(130,130,30,send) && sends==1);
    assert(!gate.withFreshPose(131,130,30,send)); // stale arrival
    assert(!gate.withFreshPose(130,131,30,send)); // stale source
    assert(!gate.withFreshPose(99,100,30,send)); // monotonic regression
    assert(!gate.withFreshPose(100,99,30,send)); // future source / ROS clock rewind
    assert(!gate.observe(100,130,true));
    assert(!gate.observe(99,130,true));
    assert(!gate.withFreshPose(131,130,30,send)); // duplicate cannot refresh
    auto staleTicket=gate.ticket();
    assert(gate.observe(140,140,false));
    assert(!gate.mapAllowed(0));
    assert(gate.observe(150,150,true));
    assert(!gate.coreUpdated(staleTicket,0,true)); // loss during Core computation
    assert(!gate.mapAllowed(0)); // good VO must not rearm old frame
    assert(!gate.coreUpdated(gate.ticket(),1,false));
    assert(!gate.mapAllowed(1));
    assert(gate.coreUpdated(gate.ticket(),1,true)); // verified Core registration only
    assert(gate.withFreshPose(151,151,30,send));
    gate.invalidate(); // suspected jump or pending boundary
    assert(!gate.mapAllowed(1));
    assert(!gate.withFreshPose(151,151,30,send));
    assert(gate.coreUpdated(gate.ticket(),1,true));
    std::promise<void> entered,release,invalidating;
    auto released=release.get_future();
    auto publisher=std::async(std::launch::async,[&] {
        return gate.withFreshPose(151,151,30,[&] { entered.set_value(); released.wait(); });
    });
    entered.get_future().wait();
    auto invalidator=std::async(std::launch::async,[&] { invalidating.set_value(); gate.invalidate(); });
    invalidating.get_future().wait();
    assert(invalidator.wait_for(std::chrono::milliseconds(10))==std::future_status::timeout);
    release.set_value(); assert(publisher.get()); invalidator.get();
    assert(!gate.withFreshPose(151,151,30,send)); // no sends after loss acknowledged
    gate.reset();
    assert(!gate.mapAllowed(1));
    assert(gate.observe(1,160,true)); // only explicit reset clears source high-water
    assert(!gate.coreUpdated(staleTicket,0,true));
    std::cout << "PASS global publication gate: registration, loss generation, epochs, dual clocks, duplicates, serialized send, reset\n";
}
