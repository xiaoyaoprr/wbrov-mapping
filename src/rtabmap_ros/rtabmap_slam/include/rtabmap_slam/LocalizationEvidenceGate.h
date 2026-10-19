#ifndef WBROV_LOCALIZATION_EVIDENCE_GATE_H
#define WBROV_LOCALIZATION_EVIDENCE_GATE_H
#include <cmath>
#include <cstdint>

namespace rtabmap_slam {
// Evidence about a position, never a grant to register an epoch or write a map.
// Caller serializes access and checks authenticated, fresh measured odometry.
class LocalizationEvidenceGate {
public:
    static constexpr long long MAX_ANCHOR_AGE_NS=6000000000LL;
    static constexpr unsigned REQUIRED_OBSERVATIONS=3;
    void reset() { *this=LocalizationEvidenceGate{}; }
    bool observe(unsigned long long epoch,std::uint64_t serial,long long stamp,long long completed,
                 bool valid,double translation,double rotation) {
        if(stamp<=0 || completed<=0 || !valid || !std::isfinite(translation) || !std::isfinite(rotation)) {
            reset();return false;
        }
        if(epoch!=epoch_ || serial!=serial_ || stamp<=stamp_ ||
           (stamp_ && stamp-stamp_>MAX_ANCHOR_AGE_NS) || translation>.025 || rotation>2.)reset();
        epoch_=epoch;serial_=serial;stamp_=stamp;completed_=completed;
        if(count_==0)first_=stamp;
        ++count_;return count_>=REQUIRED_OBSERVATIONS && stamp_-first_>=500000000LL;
    }
    bool current(unsigned long long epoch,std::uint64_t serial,long long now,bool freshTracking) const {
        return freshTracking && epoch==epoch_ && serial==serial_ && now>=completed_ &&
            now-completed_<=MAX_ANCHOR_AGE_NS && count_>=REQUIRED_OBSERVATIONS && stamp_-first_>=500000000LL;
    }
    bool candidate(unsigned long long epoch,std::uint64_t serial,long long now) const {
        return count_>0 && epoch==epoch_ && serial==serial_ && now>=completed_ && now-completed_<=MAX_ANCHOR_AGE_NS;
    }
    unsigned observations() const {return count_;}
    long long stamp() const {return stamp_;}
private:
    unsigned long long epoch_=0;
    std::uint64_t serial_=0;
    long long stamp_=0,first_=0,completed_=0;
    unsigned count_=0;
};
}
#endif
