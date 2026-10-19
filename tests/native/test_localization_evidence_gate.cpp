#include <rtabmap_slam/LocalizationEvidenceGate.h>
#include <cassert>
#include <limits>
using Gate=rtabmap_slam::LocalizationEvidenceGate;
int main() {
    constexpr long long S=1000000000LL;
    Gate g;
    assert(!g.observe(1,4,S,S,true,0,0));
    assert(g.candidate(1,4,S));assert(!g.current(1,4,S,true));
    assert(!g.observe(1,4,2*S,2*S,true,.001,.1));
    assert(g.observe(1,4,3*S,3*S,true,.001,.1));
    assert(g.current(1,4,3*S,true));
    assert(!g.current(1,4,3*S,false)); // a recent anchor cannot authorize stale odometry
    assert(!g.current(1,5,3*S,true)); // fresh receipt cannot bridge a cut
    assert(!g.current(2,4,3*S,true));
    assert(!g.current(1,4,10*S,true));
    assert(!g.observe(1,4,4*S,4*S,true,.026,.1));
    assert(g.observations()==1); // changed correction must rebuild evidence
    assert(!g.observe(1,4,5*S,5*S,true,std::numeric_limits<double>::quiet_NaN(),0));
    assert(!g.candidate(1,4,5*S));
    assert(!g.observe(1,4,6*S,6*S,false,0,0));
    assert(!g.current(1,4,6*S,true));
}
