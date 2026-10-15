#include <rtabmap_slam/JumpQuarantineGate.h>
#include <cassert>
#include <iostream>
using Gate=rtabmap_slam::JumpQuarantineGate;
using D=Gate::Decision;
int main() {
    Gate g;
    assert(g.observe(1000000000LL,1,true)==D::Reject);
    assert(g.observe(1000000000LL,1,true)==D::Reject);
    assert(g.observe(900000000LL,1,true)==D::Reject);
    assert(g.observe(1100000000LL,1,true)==D::Reject);
    assert(g.observe(1200000000LL,1,true)==D::Reject); // 3 frames, insufficient time span
    assert(g.observe(1500000000LL,1,true)==D::Arm);
    assert(g.pendingStamp()==1500000000LL);
    assert(g.observe(1600000000LL,2,true)==D::Reject); // pending cannot change coordinate chain
    assert(g.observe(1600000000LL,1,false)==D::Reject);
    assert(g.observe(1600000000LL,1,true)==D::Pending);
    g.resetCandidate();assert(g.pendingStamp()!=0);
    g.reset();assert(g.pendingStamp()==0);
    assert(g.observe(2000000000LL,1,true)==D::Reject);
    assert(g.observe(3000000000LL,2,true)==D::Reject); // new chain restarts streak
    assert(g.observe(4000000000LL,2,false)==D::Reject);
    assert(g.observe(5000000000LL,2,true)==D::Reject);
    assert(g.observe(6000000000LL,2,true)==D::Reject);
    g.resetCandidate();
    assert(g.observe(7000000000LL,2,true)==D::Reject);
    assert(g.observe(13000000000LL,2,true)==D::Reject); // expired candidate cannot bridge six seconds
    assert(g.observe(14000000000LL,2,true)==D::Reject);
    assert(g.observe(15000000000LL,2,true)==D::Arm);
    assert(g.observe(16000000000LL,3,true)==D::Reject);
    assert(g.observe(17000000000LL,3,true)==D::Reject);
    assert(g.pendingStamp()==15000000000LL && g.chain()==2);
    assert(g.observe(17000000000LL,3,true)==D::Reject); // duplicate cannot complete replacement
    assert(g.observe(18000000000LL,3,true)==D::Arm);
    assert(g.pendingStamp()==18000000000LL && g.chain()==3);
    assert(g.observe(17000000000LL,2,true)==D::Reject); // old boundary cannot consume new generation
    assert(g.observe(19000000000LL,4,true)==D::Reject);
    assert(g.observe(20000000000LL,4,false)==D::Reject);
    assert(g.observe(21000000000LL,4,true)==D::Reject);
    assert(g.observe(22000000000LL,4,true)==D::Reject);
    assert(g.pendingStamp()==18000000000LL);
    assert(g.observe(23000000000LL,4,true)==D::Arm);
    std::cout<<"PASS jump gate: duplicate, reorder, time span, chain change, weak evidence, guarded replacement generation, expiration"<<std::endl;
}
