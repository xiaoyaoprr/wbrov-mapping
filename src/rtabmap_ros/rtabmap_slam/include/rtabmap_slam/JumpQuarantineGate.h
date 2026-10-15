#ifndef RTABMAP_SLAM_JUMP_QUARANTINE_GATE_H_
#define RTABMAP_SLAM_JUMP_QUARANTINE_GATE_H_

namespace rtabmap_slam {
// Wrapper callers own the backend queue AND admission/last-pose locks while
// observing. Processing owns that same queue lock until graph update ends.
// This authorizes ONLY a separate epoch, never an edge or an alignment.
// Replacing an abandoned boundary needs another complete stable streak.
class JumpQuarantineGate {
public:
    enum class Decision { Reject, Arm, Pending };
    Decision observe(long long stamp, unsigned long long chain, bool eligible) {
        if(!eligible || stamp<=0 || chain==0) { resetCandidate();return Decision::Reject; }
        if(pending_>0) {
            if(stamp<pending_) return Decision::Reject;
            if(chain==pendingChain_) { resetCandidate();return Decision::Pending; }
            if(stamp==pending_) return Decision::Reject;
        }
        if(stamp<=last_) return Decision::Reject; // Replays cannot accumulate evidence.
        if(chain!=chain_ || first_==0 || stamp-last_>5000000000LL) {
            resetCandidate();chain_=chain;first_=stamp;
        }
        last_=stamp;
        if(count_<3) ++count_;
        if(count_>=3 && stamp-first_>=500000000LL) {
            pending_=stamp;pendingChain_=chain;
            resetCandidate();return Decision::Arm;
        }
        return Decision::Reject;
    }
    void resetCandidate() { first_=last_=0;chain_=0;count_=0; }
    void reset() { resetCandidate();pending_=0;pendingChain_=0; }
    long long pendingStamp() const { return pending_; }
    unsigned long long chain() const { return pending_>0 ? pendingChain_ : chain_; }
private:
    long long first_=0,last_=0,pending_=0;
    unsigned long long chain_=0,pendingChain_=0;
    unsigned int count_=0;
};
}
#endif
