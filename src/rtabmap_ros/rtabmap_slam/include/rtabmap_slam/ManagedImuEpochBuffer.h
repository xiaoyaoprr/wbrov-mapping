#pragma once
#include <map>
#include <cstddef>
namespace rtabmap_slam { namespace managed_imu {
// Callers hold auth->IMU locks. Exact nanoseconds decide epoch membership;
// double keys exist only for the RTAB-Map interpolation API.
template<class T> void insert(std::map<double,T>& samples,
    std::map<double,long long>& stamps, double time, long long ns,
    const T& value, std::size_t limit=1000)
{
    // A later authenticated sample in the same floating-point bucket replaces
    // both payload and provenance, never the provenance of an older payload.
    samples[time]=value; stamps[time]=ns;
    while(samples.size()>limit) {
        stamps.erase(samples.begin()->first); samples.erase(samples.begin());
    }
}
template<class T> void cut(std::map<double,T>& samples,
    std::map<double,long long>& stamps, long long cutoff)
{
    for(auto it=samples.begin();it!=samples.end();) {
        auto identity=stamps.find(it->first);
        if(identity==stamps.end() || identity->second<cutoff) {
            if(identity!=stamps.end()) stamps.erase(identity);
            it=samples.erase(it);
        } else ++it;
    }
    for(auto it=stamps.begin();it!=stamps.end();) {
        if(!samples.count(it->first)) it=stamps.erase(it); else ++it;
    }
}
}} // namespace
