#ifndef WBROV_GLOBAL_PUBLICATION_GATE_H
#define WBROV_GLOBAL_PUBLICATION_GATE_H

#include <cstdint>
#include <mutex>
#include <utility>

namespace rtabmap_slam {
// Frame-registration/freshness gate, NOT a navigation-quality certificate.
// Never reads Core from the TF thread. Loss invalidates in-flight Core tickets.
class GlobalPublicationGate {
public:
    using Epoch = unsigned long long;
    using Ticket = std::uint64_t;
    Ticket ticket() const { std::lock_guard<std::mutex> lock(mutex_); return generation_; }
    void invalidate() {
        std::lock_guard<std::mutex> lock(mutex_);
        ++generation_; registered_ = false;
    }
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        ++generation_; registered_ = good_ = false; sourceNs_ = arrivalNs_ = 0;
    }
    bool observe(long long sourceNs, long long arrivalNs, bool good) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(sourceNs <= sourceNs_ || sourceNs <= 0 || arrivalNs <= 0) return false;
        sourceNs_ = sourceNs; arrivalNs_ = arrivalNs; good_ = good;
        if(!good) { ++generation_; registered_ = false; }
        return true;
    }
    bool coreUpdated(Ticket before, Epoch epoch, bool registered) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(before != generation_) return false;
        epoch_ = epoch; registered_ = registered && good_;
        return registered_;
    }
    bool mapAllowed(Epoch epoch) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return registered_ && epoch_ == epoch;
    }
    template<class Publish>
    bool withFreshPose(long long steadyNs, long long rosNs, long long maxAgeNs, Publish && publish) const {
        // Serialize the final small send with invalidate(): no check/send loss race.
        // Callback must not call this gate or acquire authentication/state mutexes.
        std::lock_guard<std::mutex> lock(mutex_);
        if(!registered_ || !good_ || sourceNs_ <= 0 || arrivalNs_ <= 0 || maxAgeNs <= 0 ||
           steadyNs < arrivalNs_ || rosNs < sourceNs_ ||
           steadyNs-arrivalNs_ > maxAgeNs || rosNs-sourceNs_ > maxAgeNs) return false;
        std::forward<Publish>(publish)();
        return true;
    }
private:
    mutable std::mutex mutex_;
    Ticket generation_ = 0;
    Epoch epoch_ = 0;
    long long sourceNs_ = 0, arrivalNs_ = 0;
    bool registered_ = false, good_ = false;
};
} // namespace rtabmap_slam
#endif
