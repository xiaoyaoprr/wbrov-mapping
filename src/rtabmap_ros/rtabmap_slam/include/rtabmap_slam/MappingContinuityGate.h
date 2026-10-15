#ifndef WBROV_MAPPING_CONTINUITY_GATE_H
#define WBROV_MAPPING_CONTINUITY_GATE_H
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <set>

namespace rtabmap_slam {
// No sensor is inferred from a lost flag. Callers supply authenticated measured
// evidence. Core alone supplies frame registration. This class never calls Core.
class MappingContinuityGate {
public:
    using Epoch = unsigned long long;
    enum class State { NORMAL, DEGRADED, PAUSED, RECOVERING };
    struct Evidence {
        bool lost = true;
        bool visualGeometry = false;
        bool depthGeometry = false;
        bool finiteMotion = false;
        bool covarianceValid = false;
        bool recoveryGeometry = false;
        bool usable() const { return !lost && finiteMotion && covarianceValid && (visualGeometry || depthGeometry); }
    };
    struct Ticket { std::uint64_t generation; long long frameNs; };
    struct Snapshot {
        State state; Epoch epoch; std::uint64_t generation;
        long long sourceNs; unsigned goodSamples; bool registered;
        bool mainMapWriteAllowed; bool visualGeometry; bool depthGeometry;
        bool observingMainMap; unsigned mapObservationCount; bool observingCandidate;
        std::string reason;
    };
    static constexpr long long MAX_AGE_NS = 300000000LL;
    static constexpr long long RECOVERY_SPAN_NS = 200000000LL;
    static constexpr unsigned RECOVERY_SAMPLES = 3;
    static constexpr long long MIN_PAUSE_NS = 200000000LL;
    static constexpr unsigned MAP_OBSERVATIONS = 5;
    static constexpr long long MAP_OBSERVATION_SPAN_NS = 500000000LL;

    static const char * stateName(State s) {
        switch(s) { case State::NORMAL:return "NORMAL";case State::DEGRADED:return "DEGRADED";
                    case State::RECOVERING:return "RECOVERING";default:return "PAUSED"; }
    }
    static const char * chinese(State s) {
        switch(s) { case State::NORMAL:return "定位正常，持续建图";
                    case State::DEGRADED:return "视觉退化，深度几何辅助跟踪";
                    case State::RECOVERING:return "正在恢复定位，主地图暂停更新";
                    default:return "建图暂停，请缓慢退回此参考视角（人工操作提示，非导航指令）"; }
    }
    // Covariance must be finite, symmetric, positive definite and bounded. A
    // zero/default covariance is not measured confidence. LDLT catches coupled
    // degeneracy, not only diagonal values. This is NOT proof of physical accuracy.
    static bool covarianceHealthy(const std::array<double,36> & c) {
        double l[6][6] = {}; double d[6] = {};
        for(unsigned i=0;i<6;++i) {
            if(!std::isfinite(c[7*i]) || c[7*i] <= 0.0 || c[7*i] > (i<3?0.04:0.1218693434)) return false;
            for(unsigned j=0;j<6;++j)
                if(!std::isfinite(c[6*i+j]) || std::fabs(c[6*i+j]-c[6*j+i]) > 1e-7*(1.0+std::fabs(c[6*i+j]))) return false;
        }
        for(unsigned i=0;i<6;++i) {
            for(unsigned j=0;j<i;++j) {
                double v=c[6*i+j];for(unsigned k=0;k<j;++k)v-=l[i][k]*l[j][k]*d[k];
                l[i][j]=v/d[j];
            }
            double v=c[7*i];for(unsigned k=0;k<i;++k)v-=l[i][k]*l[i][k]*d[k];
            if(!std::isfinite(v) || v<=1e-14) return false;
            d[i]=v;l[i][i]=1.0;
        }
        return true;
    }
    // Inliers must cover the observed feature field and constrain at least two
    // independent spatial axes. A line/cluster cannot certify visual recovery.
    // Rows are u,v,x,y,z with 3D points from the measured F2M local map.
    static bool visualSupportHealthy(const std::vector<std::array<double,5>> & points,
                                     const std::array<double,4> & bounds) {
        if(points.size()<15U)return false;
        const double w=bounds[1]-bounds[0],h=bounds[3]-bounds[2];
        if(!std::isfinite(w)||!std::isfinite(h)||w<32.0||h<24.0)return false;
        double mean[5]={};std::set<unsigned> cells;
        for(const auto & p:points) {
            for(unsigned k=0;k<5;++k) {if(!std::isfinite(p[k]))return false;mean[k]+=p[k];}
            const double u=(p[0]-bounds[0])/w,v=(p[1]-bounds[2])/h;
            if(u<0.0||u>1.0||v<0.0||v>1.0)return false;
            cells.insert(std::min(3U,static_cast<unsigned>(u*4.0))+4U*std::min(2U,static_cast<unsigned>(v*3.0)));
        }
        if(cells.size()<6U)return false;
        for(double & v:mean)v/=static_cast<double>(points.size());
        double a=0,b=0,c=0,spatial[3][3]={};
        for(const auto & p:points) {
            const double u=(p[0]-mean[0])/w,v=(p[1]-mean[1])/h;
            a+=u*u;b+=u*v;c+=v*v;
            for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)
                spatial[i][j]+=(p[i+2]-mean[i+2])*(p[j+2]-mean[j+2]);
        }
        const double imageTrace=a+c;
        if(imageTrace<=0 || (a*c-b*b)/(imageTrace*imageTrace)<0.01)return false;
        const double tr=spatial[0][0]+spatial[1][1]+spatial[2][2];
        double minors=0;for(unsigned i=0;i<3;++i)for(unsigned j=i+1;j<3;++j)
            minors+=spatial[i][i]*spatial[j][j]-spatial[i][j]*spatial[i][j];
        return std::isfinite(tr)&&std::isfinite(minors)&&tr>1e-8&&minors/(tr*tr)>=0.01;
    }
    Ticket ticket(long long frameNs) const { std::lock_guard<std::mutex> l(mutex_);return {generation_,frameNs}; }
    // Tracks actual local-estimation discontinuities separately from failed
    // main-map alignment hypotheses. A rejected loop must not split a healthy
    // local submap; a real measurement loss must never get a neighbor bridge.
    std::uint64_t localContinuitySerial() const { std::lock_guard<std::mutex> l(mutex_);return localSerial_; }
    bool localGeometryReady(long long frameNs,long long steadyNs,long long rosNs) const {
        std::lock_guard<std::mutex> l(mutex_);
        return stableLocked() && freshLocked(steadyNs,rosNs) && frameNs>=goodStartNs_ &&
            frameNs>0 && frameNs<=sourceNs_ && sourceNs_-frameNs<=MAX_AGE_NS;
    }
    void invalidate(const char * reason="continuity invalidated") {
        std::lock_guard<std::mutex> l(mutex_); ++localSerial_;pauseLocked(reason);
    }
    void rejectAlignment(const char * reason) {
        std::lock_guard<std::mutex> l(mutex_);pauseLocked(reason);
    }
    void reset() {
        std::lock_guard<std::mutex> l(mutex_);++generation_;++localSerial_;state_=State::PAUSED;
        epoch_=0;epochKnown_=false;registered_=false;everRegistered_=false;observing_=false;candidateObservation_=candidateReady_=false;receiptOnlyPause_=false;
        sourceNs_=arrivalNs_=goodStartNs_=pauseUntilNs_=0;observationStartNs_=lastObservationNs_=0;mapObservations_=0;
        goodSamples_=visualGoodSamples_=0;visualGoodStartNs_=0;evidence_=Evidence{};reason_="waiting for authenticated geometry";
    }
    bool observe(long long sourceNs,long long arrivalNs,long long rosNs,const Evidence & evidence) {
        std::lock_guard<std::mutex> l(mutex_);
        if(sourceNs<=0 || arrivalNs<=0 || sourceNs<=sourceNs_) return false;
        // Every unique packet advances the high-water mark; stale packets never
        // acquire confidence or refresh a previous valid registration.
        if(sourceNs_>0 && (arrivalNs<arrivalNs_ || sourceNs-sourceNs_>MAX_AGE_NS)) {++localSerial_;pauseLocked("measurement gap");}
        sourceNs_=sourceNs;arrivalNs_=arrivalNs;evidence_=evidence;
        if(rosNs<sourceNs || rosNs-sourceNs>MAX_AGE_NS) {
            // Receipt latency revokes write/pose permission, not the measured
            // local coordinate chain. A subsequent source gap, lost solution,
            // bad geometry or future clock still cuts that chain explicitly.
            if(rosNs<sourceNs || !evidence.usable()) {
                ++localSerial_;pauseLocked(rosNs<sourceNs?"future source":"stale source with invalid geometry");
            } else pauseReceiptLocked("stale source");
            return true;
        }
        if(!evidence.usable()) {++localSerial_;pauseLocked(evidence.lost?"odometry lost":"geometry quality insufficient");return true;}
        if(evidence.recoveryGeometry) {
            if(goodSamples_==0) goodStartNs_=sourceNs;
            if(goodSamples_<1000000U)++goodSamples_;
        } else {goodSamples_=0;goodStartNs_=0;}
        if(evidence.visualGeometry && evidence.recoveryGeometry) {
            if(visualGoodSamples_==0)visualGoodStartNs_=sourceNs;
            if(visualGoodSamples_<1000000U)++visualGoodSamples_;
        } else {visualGoodSamples_=0;visualGoodStartNs_=0;}
        if(state_==State::NORMAL || state_==State::DEGRADED) {
            if(!evidence.visualGeometry)state_=State::DEGRADED;
            else if(state_!=State::DEGRADED || (visualGoodSamples_>=RECOVERY_SAMPLES &&
                    sourceNs_-visualGoodStartNs_>=RECOVERY_SPAN_NS))state_=State::NORMAL;
            reason_=state_==State::NORMAL?"registered visual geometry":"registered measured depth geometry";
        } else {
            state_=sourceNs<pauseUntilNs_?State::PAUSED:State::RECOVERING;
            reason_=observing_?(candidateObservation_?"candidate main-map observation; no graph commit":"registered main-map observation period; no graph writes"):"waiting for stable geometry and Core registration";
        }
        return true;
    }
    void tick(long long steadyNs,long long rosNs) {
        std::lock_guard<std::mutex> l(mutex_);
        // A timer cannot establish that the frontend lost its coordinate
        // frame. Invalidate in-flight work now; observe() decides continuity
        // from the next authenticated source stamp and measured geometry.
        if(!freshLocked(steadyNs,rosNs) && state_!=State::PAUSED) {
            if(steadyNs<arrivalNs_ || rosNs<sourceNs_)pauseLocked("clock moved behind measurement");
            else pauseReceiptLocked("source or arrival expired");
        }
    }
    // A delivery stall does not change an already registered odometry frame.
    // Reopen only after fresh measured motion is continuous for the full
    // observation span, and Core confirms the SAME registered epoch. Any lost
    // solution, source gap, reset, bad geometry or alignment rejection cancels
    // this path. No old-keyframe loop or new coordinate transform is inferred.
    bool resumeContinuousReceipt(Epoch epoch,bool coreRegistered,long long frameNs,long long steadyNs,long long rosNs) {
        std::lock_guard<std::mutex> l(mutex_);
        if(!receiptOnlyPause_ || !epochKnown_ || epoch!=epoch_ || !coreRegistered ||
           localSerial_!=receiptPauseSerial_ || !stableLocked() || !freshLocked(steadyNs,rosNs) ||
           goodSamples_<MAP_OBSERVATIONS || sourceNs_-goodStartNs_<MAP_OBSERVATION_SPAN_NS ||
           frameNs<goodStartNs_ || frameNs>sourceNs_ || sourceNs_-frameNs>MAX_AGE_NS)return false;
        receiptOnlyPause_=false;registered_=true;
        state_=visualStableLocked()?State::NORMAL:State::DEGRADED;
        reason_="fresh continuous measured motion; same Core-registered coordinate frame";
        return true;
    }
    // Registered graph: skip process until measured local tracking has recovered.
    // Unregistered graph: allow private observations; Core quarantine owns them.
    bool processAllowed(bool coreRegistered,long long frameNs,long long steadyNs,long long rosNs) const {
        std::lock_guard<std::mutex> l(mutex_);
        if(!freshLocked(steadyNs,rosNs) || !evidence_.usable() || frameNs<=0 || frameNs<goodStartNs_ || frameNs>sourceNs_ || sourceNs_-frameNs>MAX_AGE_NS)return false;
        if(observing_)return false;
        if(!coreRegistered)return stableLocked();
        if(observing_ || (everRegistered_&&!registered_))return false;
        return registered_ || stableLocked();
    }
    bool coreValidated(Ticket t,Epoch epoch,bool registered,bool exactFrameGeometry,long long steadyNs,long long rosNs) {
        std::lock_guard<std::mutex> l(mutex_);
        if(t.generation!=generation_)return false;
        if(epochKnown_ && epoch!=epoch_) {
            ++generation_;registered_=false;receiptOnlyPause_=false;state_=State::RECOVERING;
            epoch_=epoch;reason_="new private epoch needs verified registration";
            goodStartNs_=sourceNs_;goodSamples_=evidence_.usable()&&evidence_.recoveryGeometry?1U:0U;observing_=false;candidateObservation_=candidateReady_=false;
            // A Core-authenticated epoch boundary is an identity change, never
            // implicit recovery. The current transaction must be retried fresh.
            return false;
        }
        epoch_=epoch;epochKnown_=true;
        if(candidateObservation_)return false; // only candidateCommitted may open this prepared transaction
        if(!registered) {registered_=false;receiptOnlyPause_=false;observing_=false;state_=State::RECOVERING;reason_="private recovery observations only";return false;}
        if(!exactFrameGeometry || !stableLocked() || !freshLocked(steadyNs,rosNs) ||
           t.frameNs<goodStartNs_ || t.frameNs>sourceNs_ || sourceNs_-t.frameNs>MAX_AGE_NS) return false;
        if(receiptOnlyPause_)return false; // same-frame resume has its own longer observation budget
        if(everRegistered_ && !registered_) {startObservationLocked();return false;}
        if(observing_)return false;
        registered_=true;everRegistered_=true;
        state_=evidence_.visualGeometry && (state_==State::NORMAL || visualStableLocked())?State::NORMAL:State::DEGRADED;
        reason_=evidence_.visualGeometry?"registered visual geometry":"registered measured depth geometry";
        return true;
    }
    bool mainMapAllowed(Epoch epoch,long long steadyNs,long long rosNs) const {
        std::lock_guard<std::mutex> l(mutex_);return allowedLocked(epoch,steadyNs,rosNs);
    }
    bool recoveryCommitAllowed(Ticket t,Epoch epoch,long long steadyNs,long long rosNs) const {
        std::lock_guard<std::mutex> l(mutex_);
        return t.generation==generation_ && epochKnown_ && epoch==epoch_ &&
               stableLocked() && freshLocked(steadyNs,rosNs) && t.frameNs>=goodStartNs_ && t.frameNs<=sourceNs_;
    }
    bool beginMapObservation(Epoch epoch,bool coreRegistered,long long steadyNs,long long rosNs) {
        std::lock_guard<std::mutex> l(mutex_);
        if(observing_)return true;
        if(receiptOnlyPause_ || !everRegistered_ || registered_ || !coreRegistered || !epochKnown_ || epoch!=epoch_ ||
           !stableLocked() || !freshLocked(steadyNs,rosNs))return false;
        startObservationLocked();return true;
    }
    bool beginCandidateObservation(Ticket t,Epoch epoch,long long steadyNs,long long rosNs) {
        std::lock_guard<std::mutex> l(mutex_);
        if(t.generation!=generation_ || !epochKnown_ || epoch!=epoch_ || registered_ ||
           !stableLocked() || !freshLocked(steadyNs,rosNs))return false;
        startObservationLocked();registered_=false;candidateObservation_=true;candidateReady_=false;
        reason_="candidate main-map alignment observation; graph is uncommitted";return true;
    }
    bool observationInputAllowed(Ticket t,Epoch epoch,long long steadyNs,long long rosNs) const {
        std::lock_guard<std::mutex> l(mutex_);
        return t.generation==generation_ && epoch==epoch_ && observing_ && stableLocked() && freshLocked(steadyNs,rosNs) &&
            t.frameNs>=goodStartNs_ && t.frameNs>lastObservationNs_ && t.frameNs<=sourceNs_ && sourceNs_-t.frameNs<=MAX_AGE_NS &&
            (lastObservationNs_==0 || t.frameNs-lastObservationNs_>=MAP_OBSERVATION_SPAN_NS/(MAP_OBSERVATIONS-1));
    }
    bool candidateCommitReady(Ticket t,Epoch epoch,long long steadyNs,long long rosNs) const {
        std::lock_guard<std::mutex> l(mutex_);
        return t.generation==generation_ && epochKnown_ && epoch==epoch_ && candidateObservation_ &&
            candidateReady_ && observing_ && !registered_ && stableLocked() && freshLocked(steadyNs,rosNs);
    }
    bool candidateCommitted(Ticket t,Epoch epoch,bool coreRegistered,long long steadyNs,long long rosNs) {
        std::lock_guard<std::mutex> l(mutex_);
        if(t.generation!=generation_ || epoch!=epoch_ || !coreRegistered || !candidateObservation_ ||
           !candidateReady_ || !observing_ || !stableLocked() || !freshLocked(steadyNs,rosNs))return false;
        observing_=candidateObservation_=candidateReady_=false;registered_=everRegistered_=true;
        state_=visualStableLocked()?State::NORMAL:State::DEGRADED;
        reason_="observed candidate committed; next qualified frame may write";return true;
    }
    bool observeRegisteredMap(Ticket t,Epoch epoch,bool directlyVerified,
                              double seedTranslation,double seedRotationDeg,
                              double stepTranslation,double stepRotationDeg,
                              long long steadyNs,long long rosNs) {
        std::lock_guard<std::mutex> l(mutex_);
        if(t.generation!=generation_ || !observing_ || epoch!=epoch_ || t.frameNs<=lastObservationNs_)return false;
        if(!directlyVerified || !std::isfinite(seedTranslation)||!std::isfinite(seedRotationDeg)||
           !std::isfinite(stepTranslation)||!std::isfinite(stepRotationDeg)||
           seedTranslation>0.05 || seedRotationDeg>3.0 || stepTranslation>0.03 || stepRotationDeg>2.0) {
            pauseLocked("main-map observation failed or correction exceeded limits");return false;
        }
        // Exact frame freshness was checked before the read-only solve. During
        // a long solve only continuous fresh measurements may retain this ticket.
        if(!stableLocked() || !freshLocked(steadyNs,rosNs) || t.frameNs<goodStartNs_ ||
           t.frameNs>sourceNs_)return false;
        if(mapObservations_==0)observationStartNs_=t.frameNs;
        lastObservationNs_=t.frameNs;++mapObservations_;
        if(mapObservations_<MAP_OBSERVATIONS || t.frameNs-observationStartNs_<MAP_OBSERVATION_SPAN_NS)return false;
        if(candidateObservation_) {
            candidateReady_=true;reason_="candidate observations passed; waiting for final atomic commit";return true;
        }
        observing_=false;registered_=true;state_=visualStableLocked()?State::NORMAL:State::DEGRADED;
        reason_="independent registered-map observations passed; next frame may write";return true;
    }
    template<class Publish> bool withFreshMainPose(long long steadyNs,long long rosNs,Publish && publish) const {
        std::lock_guard<std::mutex> l(mutex_);
        if(!allowedLocked(epoch_,steadyNs,rosNs))return false;
        std::forward<Publish>(publish)();return true;
    }
    Snapshot snapshot(long long steadyNs,long long rosNs) const {
        std::lock_guard<std::mutex> l(mutex_);
        return {state_,epoch_,generation_,sourceNs_,goodSamples_,registered_,allowedLocked(epoch_,steadyNs,rosNs),evidence_.visualGeometry,evidence_.depthGeometry,observing_,mapObservations_,candidateObservation_,reason_};
    }
private:
    bool freshLocked(long long steadyNs,long long rosNs) const {
        return sourceNs_>0 && arrivalNs_>0 && steadyNs>=arrivalNs_ && rosNs>=sourceNs_ &&
               steadyNs-arrivalNs_<=MAX_AGE_NS && rosNs-sourceNs_<=MAX_AGE_NS;
    }
    bool stableLocked() const {return evidence_.usable() && evidence_.recoveryGeometry && sourceNs_>=pauseUntilNs_ && goodSamples_>=RECOVERY_SAMPLES && sourceNs_-goodStartNs_>=RECOVERY_SPAN_NS;}
    bool visualStableLocked() const {return evidence_.visualGeometry && visualGoodSamples_>=RECOVERY_SAMPLES && sourceNs_-visualGoodStartNs_>=RECOVERY_SPAN_NS;}
    bool allowedLocked(Epoch epoch,long long steadyNs,long long rosNs) const {
        return registered_ && epochKnown_ && epoch==epoch_ && (state_==State::NORMAL||state_==State::DEGRADED) && freshLocked(steadyNs,rosNs);
    }
    void pauseLocked(const char * reason) {
        ++generation_;registered_=false;receiptOnlyPause_=false;observing_=false;candidateObservation_=candidateReady_=false;state_=State::PAUSED;goodSamples_=visualGoodSamples_=0;goodStartNs_=visualGoodStartNs_=0;reason_=reason;
        pauseUntilNs_=sourceNs_+MIN_PAUSE_NS;mapObservations_=0;observationStartNs_=lastObservationNs_=0;
    }
    void pauseReceiptLocked(const char * reason) {
        const bool keep=(receiptOnlyPause_ && receiptPauseSerial_==localSerial_) ||
            (registered_ && everRegistered_ && epochKnown_ && !observing_ && !candidateObservation_);
        pauseLocked(reason);
        receiptOnlyPause_=keep;receiptPauseSerial_=localSerial_;
    }
    void startObservationLocked() {
        registered_=true;receiptOnlyPause_=false;observing_=true;candidateObservation_=candidateReady_=false;state_=State::RECOVERING;mapObservations_=0;observationStartNs_=lastObservationNs_=0;
        reason_="registered main-map observation period; no graph writes";
    }
    mutable std::mutex mutex_;
    State state_=State::PAUSED;Epoch epoch_=0;std::uint64_t generation_=0,localSerial_=0;
    long long sourceNs_=0,arrivalNs_=0,goodStartNs_=0;unsigned goodSamples_=0;
    bool registered_=false,epochKnown_=false,everRegistered_=false,observing_=false;
    bool candidateObservation_=false,candidateReady_=false;
    bool receiptOnlyPause_=false;std::uint64_t receiptPauseSerial_=0;
    long long pauseUntilNs_=0,observationStartNs_=0,lastObservationNs_=0;unsigned mapObservations_=0;
    long long visualGoodStartNs_=0;unsigned visualGoodSamples_=0;
    Evidence evidence_;std::string reason_="waiting for authenticated geometry";
};
}
#endif
