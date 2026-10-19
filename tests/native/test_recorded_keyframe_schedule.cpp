#include "RecordedKeyframeSchedule.h"
#include <cassert>
#include <limits>
int main() {
    const auto due=[](long long source,bool healthy=true,bool reference=true,bool newTag=false,double d=0,double r=0) {
        return recordedKeyframeDue(source,1000000000LL,healthy,reference,newTag,d,r);
    };
    assert(!due(1200000000LL));assert(due(2000000000LL));
    assert(due(1200000000LL,false)); // paused/degraded/unregistered bypass
    assert(due(1200000000LL,true,false));assert(due(1200000000LL,true,true,true));
    assert(due(1200000000LL,true,true,false,.02));assert(!due(1200000000LL,true,true,false,.019));
    assert(due(1200000000LL,true,true,false,0,2.*3.14159265358979323846/180.));
    assert(due(1200000000LL,true,true,false,std::numeric_limits<double>::quiet_NaN()));
    assert(due(900000000LL)); // invalid time never suppressed by the scheduler

}
