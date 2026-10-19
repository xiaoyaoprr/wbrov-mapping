#pragma once
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Odometry.h>
#include "NativeMappingEvidence.h"
#include "RecordedCloudCache.h"
#include "RecordedCloudAssembly.h"
#include "RecordedCloudSamples.h"
#include "RecordedCloudPly.h"
#include "RecordedOrderedPrefetch.h"
#include "RecordedKeyframeSchedule.h"
#include <filesystem>
#include <fstream>
#include <tuple>
#include <iomanip>
#include <stdexcept>
#include <chrono>

namespace rtabmap {
// Serialized, recording-only adapter. It has no ROS node, TF, navigation or
// actuator output. Its clock is RECORDING time, never wall time; all public
// status/output says historical. The caller must verify immutable SVO input.
// Loss, measured gaps, covariance/support, 3-bridge shadow validation and five
// held-out observations use the same Core and gate as the live adapter.
class DeferredMappingSession {
public:
    using Gate=rtabmap_slam::MappingContinuityGate;
    DeferredMappingSession(ParametersMap parameters,ParametersMap odomParameters,
            const std::filesystem::path &output,const std::string &sessionHash,
            const std::string &inputIdentity):output_(output) {
        if(!receiptHex(sessionHash,64) || !receiptHex(inputIdentity,64))throw std::runtime_error("recording identity required");
        if(!std::filesystem::create_directory(output_))throw std::runtime_error("new worker output required");
        context_={sessionHash,std::string(64,'0'),inputIdentity.substr(0,48),sessionHash.substr(0,48),"verified_recording_rgbd"};
        parameters["Rtabmap/WorkingDirectory"]=output_.string();
        parameters["Mem/BinDataKept"]="true";
        parameters["Mem/GenerateIds"]="true";
        parameters["Mem/UseOdomGravity"]="false";
        parameters["Rtabmap/DetectionRate"]="0"; // caller paces by original source time
        parameters["Vis/MinInliers"]="25";
        core_.init(parameters,(output_/"map.db").string());core_.setSafeRelocalizationGuardEnabled(true);
        odomParameters["Odom/FillInfoData"]="true";
        odom_.reset(Odometry::create(odomParameters));
        // Exact production limits. Recorded time does not relax geometry.
        limits_.maxHoldoutTranslation=.08f;limits_.maxHoldoutRotation=8;
        limits_.minBridgeTranslation=.02f;limits_.minBridgeRotation=2;
        limits_.maxAnchorZ=.02f;limits_.maxAnchorTilt=2;
        limits_.maxOldPoseTranslation=.005f;limits_.maxOldPoseRotation=.5f;
        limits_.maxNewRigidTranslation=.03f;limits_.maxNewRigidRotation=3;
        limits_.maxNewRigidZError=.02f;limits_.maxNewRigidTiltError=2;
        limits_.maxGravityResidualIncrease=.5f;limits_.maxInformationCondition=1e8;
        trace_.open(output_/"frames.csv");
        trace_<<"source_ns,lost,inliers,matches,usable,epoch,cuts,main_write,main_nodes,local_nodes,recoveries,input_present,reason\n";
    }
    ~DeferredMappingSession(){try {close();}catch(...) {}}
    // Missing input is not a measured odometry failure. Write no point/pose for
    // it. Let the SAME source-gap/freshness rules and next measured VO interval
    // decide continuity, just as the live camera's missing-IMU return does.
    void missing(long long source,const std::string &reason) {
        currentInputPresent_=false;advance(source);Gate::Evidence e;e.qualityReason=reason;
        gate_.tick(source,source);++missing_;reason_=reason;
        trace({},e);writeStatus("processing");
    }
    void process(SensorData low,SensorData map,long long source,
            const std::vector<AcceptedTagCornerObservation> &tags={}) {
        consume(map,source,tags,estimate(low));
    }
    struct MeasuredFrame {OdometryInfo info;Transform pose;int width=0,height=0;double computeMs=0;};
    // Only the frontend lane calls this. Core/gate/status are owned exclusively
    // by consume()/missing(); no historical pose is published as current.
    MeasuredFrame estimate(SensorData low) {
        const auto started=std::chrono::steady_clock::now();MeasuredFrame result;
        result.pose=odom_->process(low,&result.info);result.info.reg.covariance=result.info.reg.covariance.clone();
        result.width=low.imageRaw().cols;result.height=low.imageRaw().rows;
        result.computeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        return result;
    }
    void consume(SensorData map,long long source,const std::vector<AcceptedTagCornerObservation> &tags,const MeasuredFrame &frame) {
        currentInputPresent_=true;advance(source);
        currentTags_=tags;
        const auto &info=frame.info;const auto &pose=frame.pose;
        const auto odomEnd=std::chrono::steady_clock::now();
        odomMs_+=frame.computeMs;
        if(std::isfinite(info.timeEstimation) && info.timeEstimation>=0)odomEstimationMs_+=1000.*info.timeEstimation;
        if(std::isfinite(info.localBundleTime) && info.localBundleTime>=0)odomBundleMs_+=1000.*info.localBundleTime;
        auto e=wbrov::mappingEvidence(info,frame.width,frame.height);
        if(!pose.isNull() && !previousPose_.isNull()) {
            const double translation=pose.getDistance(previousPose_);
            const double rotation=pose.getAngle(previousPose_)*180./M_PI;
            if(pose.isIdentity() || !std::isfinite(translation) || !std::isfinite(rotation) || translation>.5 || rotation>30) {
                e.finiteMotion=false;e.qualityReason+="; unannounced pose jump";
                jumpUntil_=source+500000000LL;jumpSamples_=0;
            }
        }
        if(!pose.isNull())previousPose_=pose;
        if(e.usable() && jumpUntil_)++jumpSamples_;
        gate_.observe(source,source,source,e);
        const auto serial=gate_.localContinuitySerial();
        if(pending_ && (serial!=pendingSerial_ || !current(pendingTicket_))) {
            pending_=false;observations_.clear();gate_.rejectAlignment("recorded candidate lost its source chain");
        }
        if(e.usable() && !pose.isNull() && (!jumpUntil_ || (source>=jumpUntil_ && jumpSamples_>=3)) &&
           gate_.localGeometryReady(source,source,source)) {
            jumpUntil_=0;
            if(written_ && serial!=writtenSerial_ && writtenEpoch_==core_.getResetEpoch()) {
                pending_=false;observations_.clear();
                if(core_.triggerNewMapForContinuation()<0)throw std::runtime_error("recorded boundary refused");
            }
            const bool mapDue=!map.imageRaw().empty() && (!lastAttempt_ || source-lastAttempt_>=200000000LL);
            if(mapDue) {
                lastAttempt_=source;
                if(pending_)observeCandidate(map,pose);
                else {
                    gate_.resumeContinuousReceipt(core_.getResetEpoch(),core_.isCurrentMapFrameRegistered(),source,source,source);
                    if(gate_.beginMapObservation(core_.getResetEpoch(),core_.isCurrentMapFrameRegistered(),source,source))
                        observeCurrent(map,pose);
                    else if(gate_.processAllowed(core_.isCurrentMapFrameRegistered(),source,source,source)) {
                        bool newTag=false;
                        for(const auto &tag:currentTags_)if(!tags_.count(tag.tagId))newTag=true;
                        const auto state=gate_.snapshot(source,source);
                        const bool healthy=state.state==Gate::State::NORMAL && state.mainMapWriteAllowed &&
                            core_.isCurrentMapFrameRegistered() && writtenSerial_==serial && writtenEpoch_==core_.getResetEpoch();
                        if(recordedKeyframeDue(source,lastCoreAttempt_,healthy,!lastAcceptedPose_.isNull(),newTag,
                            lastAcceptedPose_.isNull()?0:pose.getDistance(lastAcceptedPose_),
                            lastAcceptedPose_.isNull()?0:pose.getAngle(lastAcceptedPose_))) {
                            lastCoreAttempt_=source;retain(map,pose,info,serial);
                        } else ++redundantCoreFrames_;
                    }
                }
            }
        }
        trace(info,e);
        mapMs_+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-odomEnd).count();
        if(frames_%30==0)writeStatus("processing");
    }
    void checkpoint() {
        trace_.flush();if(!trace_)throw std::runtime_error("frame ledger failed");
        {TimedPhase phase(cloudExportMs_);exportCloud();}
        {TimedPhase phase(tagExportMs_);exportTags();}
        writeStatus("waiting_for_recording");
    }
    void close() {
        if(closed_)return;
        checkpoint();closedMainNodes_=mainPoses().size();closedEpoch_=core_.getResetEpoch();
        core_.close(true);closed_=true;trace_.close();writeStatus("closed");
    }
    long long source() const{return source_;}
private:
    struct TimedPhase {
        explicit TimedPhase(double &total):total_(total),start_(std::chrono::steady_clock::now()){}
        ~TimedPhase(){total_+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start_).count();}
        double &total_;std::chrono::steady_clock::time_point start_;
    };
    void advance(long long source) {
        if(source<=0 || source<=source_)throw std::runtime_error("recording source time not strictly increasing");
        source_=source;++frames_;
    }
    ImageInputIdentity identity(const SensorData &data) const {
        ImageInputIdentity i;i.headerStampNs=source_;i.width=data.imageRaw().cols;i.height=data.imageRaw().rows;
        i.encoding=data.imageRaw().channels()==3?"bgr8":"mono8";i.pixelSha256=canonicalImageSha256(data.imageRaw());
        const auto k=data.cameraModels().front().K();for(int n=0;n<9;++n)i.K[n]=k.template at<double>(n/3,n%3);
        i.D={0,0,0,0,0};i.sensorFrame="recorded_camera_optical";i.rgbPublisherGidHex=context_.rgbPublisherGidHex;
        i.infoPublisherGidHex=context_.infoPublisherGidHex;i.sessionTokenSha256=context_.sessionTokenSha256;
        i.databaseSha256=context_.databaseSha256;i.provenanceScope=context_.provenanceScope;return i;
    }
    bool current(Gate::Ticket ticket) const {
        return gate_.recoveryCommitAllowed(ticket,core_.getResetEpoch(),source_,source_);
    }
    void retain(const SensorData &data,const Transform &pose,const OdometryInfo &info,std::uint64_t serial) {
        const auto ticket=gate_.ticket(source_);std::string why;
        std::map<std::string,float> stats;
        if(!currentTags_.empty() && (lastReferencePose_.isNull() ||
           (source_-lastReferenceNs_>=200000000LL && (pose.getDistance(lastReferencePose_)>=.03 ||
            pose.getAngle(lastReferencePose_)>=3.*M_PI/180.))))stats["Recovery/ReferenceCapture/"]=1.f;
        bool processed=false;
        {TimedPhase phase(coreProcessMs_);processed=core_.processWithAuthenticatedImage(data,pose,info.reg.covariance,{},stats,identity(data),why);}
        ++coreCalls_;
        if(processed)for(const auto &stat:core_.getStatistics().data())
            if(stat.first.rfind("Timing",0)==0 && std::isfinite(stat.second) && stat.second>=0)
                coreTimings_[stat.first]+=stat.second;
        if(!processed) {reason_=why;return;}
        written_=true;writtenSerial_=serial;writtenEpoch_=core_.getResetEpoch();lastLocal_=core_.getLastLocationId();
        AcceptedImageReceipt receipt;
        if(core_.getAcceptedImageReceipt(lastLocal_,receipt) && receipt.input.headerStampNs==source_) {
            lastAcceptedPose_=pose;
            for(auto tag:currentTags_) {
                if(tag.imageSha256!=receipt.input.pixelSha256 || !receiptHex(tag.pixelRecordSha256,64) || tag.tagId<0)
                    throw std::runtime_error("recorded Tag is not bound to the accepted pixels");
                tag.signatureId=lastLocal_;tags_[tag.tagId][lastLocal_]=tag;
            }
            if(!currentTags_.empty()){lastReferencePose_=pose;lastReferenceNs_=source_;}
        }
        {TimedPhase phase(freezeMs_);
        if(!core_.validateAndFreezeCommittedEpochForSafeRelocalization(why))throw std::runtime_error("frozen graph failed: "+why);}
        gate_.coreValidated(ticket,core_.getResetEpoch(),core_.isCurrentMapFrameRegistered(),true,source_,source_);
        if(core_.isCurrentMapFrameRegistered() && gate_.mainMapAllowed(core_.getResetEpoch(),source_,source_)) {
            lastTrusted_=lastLocal_;verifiedThrough_=source_;
        } else if(!core_.isCurrentMapFrameRegistered() && current(gate_.ticket(source_))) {
            const auto preparation=gate_.ticket(source_);Transform proposed;
            pendingTag_=prepareTag(preparation,proposed,why);
            if((pendingTag_ || core_.prepareVisualRecoveryWithAcceptedImages(limits_,context_,[&]{return current(preparation);},proposed,why)) &&
               gate_.beginCandidateObservation(preparation,core_.getResetEpoch(),source_,source_)) {
                pending_=true;pendingTicket_=preparation;pendingSerial_=serial;seed_=lastCorrection_=proposed;observations_.clear();
            }
        }
        reason_=why;
    }
    std::vector<int> independentTagNodes(const std::map<int,AcceptedTagCornerObservation> &nodes,
            const std::map<int,Transform> &poses,bool currentEpoch) const {
        std::vector<int> selected;
        for(auto it=nodes.rbegin();it!=nodes.rend();++it) {
            const int id=it->first;
            if(!poses.count(id) || core_.isRecoveryArchivedNode(id) ||
               (currentEpoch && core_.getSignatureResetEpoch(id)!=core_.getResetEpoch()))continue;
            AcceptedImageReceipt receipt;if(!core_.getAcceptedImageReceipt(id,receipt))continue;
            bool independent=true;
            for(int old:selected) {
                AcceptedImageReceipt previous;if(!core_.getAcceptedImageReceipt(old,previous))return {};
                if(std::abs(receipt.input.headerStampNs-previous.input.headerStampNs)<200000000LL ||
                   (poses.at(id).getDistance(poses.at(old))<.03 && poses.at(id).getAngle(poses.at(old))<3.*M_PI/180.))
                    independent=false;
            }
            if(independent)selected.push_back(id);
            if(selected.size()==3)break;
        }
        return selected;
    }
    bool prepareTag(Gate::Ticket ticket,Transform &proposed,std::string &why) {
        TimedPhase phase(tagPreparationMs_);
        tagLinks_.clear();tagReceipts_.clear();tagCorners_.clear();
        for(const auto &tag:tags_) {
            const auto old=independentTagNodes(tag.second,mainPoses(),false);
            const auto local=independentTagNodes(tag.second,core_._optimizedPoses,true);
            UWARN("recorded Tag selection source_ns=%lld epoch=%llu tag=%d old=%u local=%u",source_,(unsigned long long)core_.getResetEpoch(),tag.first,(unsigned)old.size(),(unsigned)local.size());
            if(old.size()!=3 || local.size()!=3)continue;
            UWARN("recorded Tag triplets source_ns=%lld tag=%d old=%d,%d,%d local=%d,%d,%d",source_,tag.first,old[0],old[1],old[2],local[0],local[1],local[2]);
            tagLinks_.clear();tagReceipts_.clear();tagCorners_.clear();
            for(int pair=0;pair<3;++pair) {
                // Endpoint selector only. Core independently measures all six
                // accepted depth/corner sets; this identity is never a bridge.
                tagLinks_.emplace_back(local[pair],old[pair],Link::kUserClosure,Transform::getIdentity(),cv::Mat::eye(6,6,CV_64F));
                for(int id:{local[pair],old[pair]}) {
                    AcceptedImageReceipt receipt;
                    if(!core_.getAcceptedImageReceipt(id,receipt))throw std::runtime_error("Tag receipt disappeared");
                    tagReceipts_.push_back(receipt);tagCorners_.push_back(tag.second.at(id));
                }
            }
            const bool prepared=core_.prepareDepthCornerRecoveryWithAcceptedImages(tagLinks_,core_.getResetEpoch(),limits_,
               tagReceipts_,tagCorners_,context_,[&]{return current(ticket);},proposed,why);
            UWARN("recorded Tag preparation source_ns=%lld epoch=%llu tag=%d accepted=%d reason=%s",source_,(unsigned long long)core_.getResetEpoch(),tag.first,prepared?1:0,why.c_str());
            if(prepared)return true;
        }
        return false;
    }
    static void corrections(const Transform &measured,const Transform &pose,const Transform &seed,const Transform &last,
            double &dt,double &dr,double &st,double &sr,Transform &correction) {
        correction=measured*pose.inverse();dt=correction.getDistance(seed);dr=correction.getAngle(seed)*180./M_PI;
        st=correction.getDistance(last);sr=correction.getAngle(last)*180./M_PI;
    }
    void observeCurrent(const SensorData &data,const Transform &pose) {
        TimedPhase phase(observationMs_);
        const auto ticket=gate_.ticket(source_);const auto epoch=core_.getResetEpoch();
        if(!gate_.observationInputAllowed(ticket,epoch,source_,source_))return;
        if(observationGeneration_!=ticket.generation || seed_.isNull()) {
            seed_=lastCorrection_=core_.getMapCorrection();observationGeneration_=ticket.generation;
        }
        Rtabmap::RecoveryObservationDiagnostics d;Transform measured;std::string why;
        const bool ok=core_.isCurrentMapFrameRegistered() ?
            core_.validateRegisteredMapObservation(data,pose,lastTrusted_,limits_,measured,why,&d):
            core_.validateLocalMapObservation(data,pose,lastLocal_,limits_,measured,why,&d);
        reason_=why;
        if(!ok && d.tested>0 && !d.predictionRejected && !d.agreementRejected && !d.firstAnchor && !d.secondAnchor) {
            gate_.waitForRegisteredMapObservation(ticket,epoch,source_,source_);return;
        }
        double dt=INFINITY,dr=INFINITY,st=INFINITY,sr=INFINITY;Transform correction;
        if(ok)corrections(measured,pose,seed_,lastCorrection_,dt,dr,st,sr,correction);
        gate_.observeRegisteredMap(ticket,epoch,ok,dt,dr,st,sr,source_,source_);
        if(ok)lastCorrection_=correction;
    }
    void observeCandidate(const SensorData &data,const Transform &pose) {
        TimedPhase phase(observationMs_);
        const auto ticket=gate_.ticket(source_);const auto epoch=core_.getResetEpoch();
        if(!gate_.observationInputAllowed(ticket,epoch,source_,source_))return;
        Transform measured;std::string why;
        if(!core_.validateProposedMapObservation(data,pose,lastTrusted_,seed_,limits_,measured,why)) {
            pending_=false;observations_.clear();gate_.rejectAlignment(why.c_str());reason_=why;return;
        }
        double dt,dr,st,sr;Transform correction;corrections(measured,pose,seed_,lastCorrection_,dt,dr,st,sr,correction);
        const bool ready=gate_.observeRegisteredMap(ticket,epoch,true,dt,dr,st,sr,source_,source_);
        if(dt>.05 || dr>3 || st>.03 || sr>2) {pending_=false;observations_.clear();return;}
        observations_.emplace_back(pose,measured);lastCorrection_=correction;
        if(!ready)return;
        const std::function<bool(const Transform &)> certificate=[&](const Transform &final) {
            if(!current(pendingTicket_) || !gate_.candidateCommitReady(pendingTicket_,epoch,source_,source_) || observations_.size()<5)return false;
            for(const auto &sample:observations_) {
                const auto predicted=final*sample.first;
                if(predicted.getDistance(sample.second)>.025 || predicted.getAngle(sample.second)>2.*M_PI/180. ||
                   std::abs(predicted.z()-sample.second.z())>.02)return false;
            }
            return true;
        };
        const bool committed=pendingTag_?
            core_.commitDepthCornerRecoveryWithObservedAlignment(tagLinks_,epoch,limits_,tagReceipts_,tagCorners_,context_,certificate,why):
            core_.commitVisualRecoveryWithObservedAlignment(limits_,context_,certificate,why);
        if(committed) {
            gate_.candidateCommitted(pendingTicket_,epoch,core_.isCurrentMapFrameRegistered(),source_,source_);++recoveries_;
        } else gate_.rejectAlignment(why.c_str());
        pending_=false;observations_.clear();reason_=why;
    }
    std::map<int,Transform> mainPoses() const {
        std::map<int,Transform> result;
        const auto &source=core_.isCurrentMapFrameRegistered()?core_._optimizedPoses:core_._safeRelocalizationTrustedPoses;
        for(const auto &p:source)if(p.first>0 && !core_.isRecoveryArchivedNode(p.first) &&
            (core_.getSignatureResetEpoch(p.first)==0 || core_._safeRelocalizationCommittedEpochs.count(core_.getSignatureResetEpoch(p.first))))result.insert(p);
        return result;
    }
    void exportTags() {
        const auto poses=mainPoses();std::ofstream out(output_/"tags.tmp");
        out<<"{\"historical_only\":true,\"frame_id\":\"deferred_map\",\"tags\":[";
        bool comma=false;
        for(const auto &tag:tags_) {
            const auto independent=independentTagNodes(tag.second,poses,false);
            int count=0;for(const auto &node:tag.second)if(poses.count(node.first))++count;
            if(!count)continue;
            if(comma)out<<',';
            comma=true;
            out<<"{\"tag_id\":"<<tag.first<<",\"distinct_main_nodes\":"<<count<<",\"independent_references\":"<<independent.size()<<",\"required_references\":3";
            // Position marker only, measured from saved depth, never a recovery
            // constraint. Core's transaction uses all corners and held-outs.
            for(auto it=tag.second.rbegin();it!=tag.second.rend();++it) {
                if(!poses.count(it->first))continue;
                auto data=core_.getMemory()->getNodeData(it->first,true,false,false,false);
                // This display-only marker uses saved depth and calibration.
                // Decode that same depth without also decoding unused RGB.
                // Re-measure each export so graph corrections and newer valid
                // observations still update the marker normally.
                cv::Mat decodedDepth;data.uncompressData(nullptr,&decodedDepth);
                if(data.cameraModels().size()!=1)continue;
                const auto &depth=data.depthRaw();const auto &model=data.cameraModels().front();
                double u=0,v=0;for(int k=0;k<4;++k){u+=it->second.cornersXY[2*k]/4;v+=it->second.cornersXY[2*k+1]/4;}
                const int x=std::lround(u),y=std::lround(v);std::vector<float> zs;
                for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx)if(x+dx>=0 && x+dx<depth.cols && y+dy>=0 && y+dy<depth.rows) {
                    const float z=depth.type()==CV_32FC1?depth.at<float>(y+dy,x+dx):depth.type()==CV_16UC1?depth.at<unsigned short>(y+dy,x+dx)*.001f:0;
                    if(std::isfinite(z) && z>=.2f && z<=1.5f)zs.push_back(z);
                }
                if(zs.size()<9)continue;
                std::sort(zs.begin(),zs.end());const float z=zs[zs.size()/2];
                const float px=(u-model.cx())*z/model.fx(),py=(v-model.cy())*z/model.fy();
                const auto pose=poses.at(it->first)*model.localTransform();const auto *m=pose.data();
                out<<",\"position\":["<<m[0]*px+m[1]*py+m[2]*z+m[3]<<','<<m[4]*px+m[5]*py+m[6]*z+m[7]<<','<<m[8]*px+m[9]*py+m[10]*z+m[11]<<']';break;
            }
            out<<'}';
        }
        out<<"],\"trajectory\":[";comma=false;
        for(const auto &node:poses) {
            AcceptedImageReceipt receipt;if(!core_.getAcceptedImageReceipt(node.first,receipt))continue;
            if(comma)out<<',';
            comma=true;
            const auto q=node.second.getQuaternionf();
            out<<"{\"node_id\":"<<node.first<<",\"source_ns\":"<<receipt.input.headerStampNs
               <<",\"epoch\":"<<core_.getSignatureResetEpoch(node.first)<<",\"position\":["
               <<node.second.x()<<','<<node.second.y()<<','<<node.second.z()<<"],\"orientation_xyzw\":["
               <<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<"]}";
        }
        out<<"],\"trajectory_edges\":[";comma=false;std::set<std::pair<int,int>> edges;
        const auto &constraints=core_.isCurrentMapFrameRegistered()?core_._constraints:core_._safeRelocalizationTrustedConstraints;
        for(const auto &item:constraints) {
            const auto &link=item.second;
            if((link.type()==Link::kNeighbor || link.type()==Link::kNeighborMerged) &&
               poses.count(link.from()) && poses.count(link.to()) &&
               core_.getSignatureResetEpoch(link.from())==core_.getSignatureResetEpoch(link.to()))
                edges.insert(std::minmax(link.from(),link.to()));
        }
        for(const auto &edge:edges){if(comma)out<<',';comma=true;out<<'['<<edge.first<<','<<edge.second<<']';}
        out<<"]}\n";out.flush();if(!out)throw std::runtime_error("Tag and trajectory export failed");out.close();
        std::filesystem::rename(output_/"tags.tmp",output_/"tags.json");
    }
    void exportCloud() {
        const auto poses=mainPoses();std::map<int,std::string> receipts;
        for(const auto &p:poses){AcceptedImageReceipt receipt;if(core_.getAcceptedImageReceipt(p.first,receipt))
            receipts[p.first]=std::to_string(receipt.input.headerStampNs)+":"+receipt.input.pixelSha256;}
        if(exported_ && receipts.size()==poses.size() && poses==exportedPoses_ && receipts==exportedReceipts_)return;
        using Point=RecordedCloudCache::Point;
        cloudCache_.retain(poses);
        cloudSamples_.retain(poses);
        RecordedCloudAssembly::Inputs inputs;
        for(const auto &item:poses) {
            RecordedCloudCache::Pose poseKey{};std::copy(item.second.data(),item.second.data()+12,poseKey.begin());
            // Missing provenance is never used as a stable cache identity.
            const auto key=receipts.count(item.first)?receipts.at(item.first):std::to_string(source_);
            inputs[item.first]={poseKey,key};
        }
        struct BuiltCloud {RecordedCloudCache::Cloud cloud;double transformMs=0;};
        using Prefetch=RecordedOrderedPrefetch<BuiltCloud>;
        // Core and both caches stay on this thread. A task owns the sampled
        // points because preparing the next node may replace a transient slot.
        auto prepareNode=[&](int id)->Prefetch::Job {
            const auto &identity=inputs.at(id);
            ++cloudNodeBuilds_;TimedPhase preparePhase(cloudPrepareMs_);
            const auto &samples=cloudSamples_.get(id,identity.input,[&] {
            RecordedCloudSamples::Data samples;
            ++cloudSampleBuilds_;
            const auto decodeStart=std::chrono::steady_clock::now();
            auto data=core_.getMemory()->getNodeData(id,true,false,false,false);
            data.uncompressData();const auto &depth=data.depthRaw();const auto &rgb=data.imageRaw();
            cloudDecodeMs_+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-decodeStart).count();
            if((depth.type()!=CV_32FC1 && depth.type()!=CV_16UC1) || rgb.empty() || data.cameraModels().size()!=1)return samples;
            const auto &camera=data.cameraModels().front();
            std::copy(camera.localTransform().data(),camera.localTransform().data()+12,samples.cameraTransform.begin());
            if(depth.size()!=rgb.size())throw std::runtime_error("export RGB/depth geometry mismatch");
            for(int y=0;y<depth.rows;y+=3)for(int x=0;x<depth.cols;x+=3) {
                const float z=depth.type()==CV_32FC1?depth.at<float>(y,x):depth.at<unsigned short>(y,x)*.001f;
                if(!std::isfinite(z) || z<.2 || z>1.5)continue;
                const float px=(x-camera.cx())*z/camera.fx(),py=(y-camera.cy())*z/camera.fy();
                Point p{px,py,z,0,0,0};
                if(rgb.type()==CV_8UC3){const auto c=rgb.at<cv::Vec3b>(y,x);p.r=c[2];p.g=c[1];p.b=c[0];}
                else p.r=p.g=p.b=rgb.at<unsigned char>(y,x);
                samples.points.push_back(p);
            }
            return samples;
            });
            const auto &c=samples.cameraTransform;
            const auto pose=poses.at(id)*Transform(c[0],c[1],c[2],c[3],c[4],c[5],c[6],c[7],c[8],c[9],c[10],c[11]);
            RecordedCloudCache::Pose matrix{};std::copy(pose.data(),pose.data()+12,matrix.begin());
            return [points=samples.points,m=matrix]() {
            const auto began=std::chrono::steady_clock::now();BuiltCloud result;auto &cloud=result.cloud;
            cloud.reserve(points.size());
            for(const auto &p:points) {
                Point world{m[0]*p.x+m[1]*p.y+m[2]*p.z+m[3],m[4]*p.x+m[5]*p.y+m[6]*p.z+m[7],m[8]*p.x+m[9]*p.y+m[10]*p.z+m[11],p.r,p.g,p.b};
                cloud.emplace(std::make_tuple(int(std::floor(world.x/.01)),int(std::floor(world.y/.01)),int(std::floor(world.z/.01))),world);
            }
            result.transformMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
            return result;
            };
        };
        std::unique_ptr<Prefetch> prefetch;
        cloudAssembly_.updatePrepared(inputs,[&](const std::vector<int> &required) {
            std::vector<int> missing;
            for(int id:required) {
                const auto &identity=inputs.at(id);
                if(!cloudCache_.matches(id,identity.pose,identity.input))missing.push_back(id);
            }
            prefetch.reset(new Prefetch(std::move(missing),prepareNode,2));
            if(prefetch->parallel())++cloudParallelExports_;
        },[&](int id)->const RecordedCloudCache::Cloud& {
            ++cloudNodeVisits_;const auto &identity=inputs.at(id);
            cloudCache_.update(id,identity.pose,identity.input,[&] {
                const bool prepared=prefetch->prepared(id);
                // A cache hit in the initial plan can be evicted by an earlier
                // node. Rebuild it here without skipping any queued result.
                auto built=prepared?prefetch->take(id):prepareNode(id)();
                if(prepared)++cloudParallelNodes_;
                cloudTransformMs_+=built.transformMs;
                return std::move(built.cloud);
            });
            return cloudCache_.points(id);
        },[&]{prefetch->finish();});
        const auto &cloud=cloudAssembly_.points();
        TimedPhase serializePhase(cloudSerializeMs_);
        const auto temporary=output_/"main_cloud.tmp";std::ofstream out(temporary,std::ios::binary);
        writeRecordedCloudPly(out,cloud);
        out.flush();if(!out)throw std::runtime_error("cloud export failed");out.close();std::filesystem::rename(temporary,output_/"main_cloud.ply");
        points_=cloud.size();exported_=true;exportedPoses_=poses;exportedReceipts_=receipts;
    }
    void trace(const OdometryInfo &info,const Gate::Evidence &e) {
        const auto s=gate_.snapshot(source_,source_);
        std::string why=s.reason+"; "+reason_;for(char &c:why)if(c==',' || c=='\n' || c=='\r')c=' ';
        trace_<<source_<<','<<info.lost<<','<<info.reg.inliers<<','<<info.reg.matches<<','<<e.usable()<<','<<core_.getResetEpoch()<<','
            <<s.cutCount<<','<<(currentInputPresent_ && s.mainMapWriteAllowed)<<','<<mainPoses().size()<<','<<core_._optimizedPoses.size()<<','<<recoveries_
            <<','<<currentInputPresent_<<','<<why<<'\n';
    }
    void writeStatus(const char *state) {
        const auto s=gate_.snapshot(source_,source_);const auto temporary=output_/"status.tmp";
        std::ofstream f(temporary);f<<"{\"schema\":\"wbrov.deferred-mapping.v1\",\"state\":\""<<state
            <<"\",\"historical_only\":true,\"navigation_authorized\":false,\"processed_source_ns\":"<<source_
            <<",\"verified_map_source_ns\":"<<verifiedThrough_<<",\"frames\":"<<frames_<<",\"input_gaps\":"<<missing_
            <<",\"odometry_compute_ms\":"<<odomMs_<<",\"mapping_compute_ms\":"<<mapMs_
            <<",\"odometry_estimation_ms\":"<<odomEstimationMs_<<",\"odometry_bundle_ms\":"<<odomBundleMs_
            <<",\"core_process_ms\":"<<coreProcessMs_<<",\"frozen_graph_check_ms\":"<<freezeMs_
            <<",\"recovery_observation_ms\":"<<observationMs_<<",\"tag_preparation_ms\":"<<tagPreparationMs_
            <<",\"cloud_export_ms\":"<<cloudExportMs_<<",\"tag_export_ms\":"<<tagExportMs_
            <<",\"cloud_node_visits\":"<<cloudNodeVisits_<<",\"cloud_node_builds\":"<<cloudNodeBuilds_
            <<",\"cloud_node_build_ms\":"<<(cloudPrepareMs_+cloudTransformMs_)<<",\"cloud_decode_ms\":"<<cloudDecodeMs_
            <<",\"cloud_prepare_ms\":"<<cloudPrepareMs_<<",\"cloud_transform_cpu_ms\":"<<cloudTransformMs_
            <<",\"cloud_parallel_exports\":"<<cloudParallelExports_<<",\"cloud_parallel_nodes\":"<<cloudParallelNodes_
            <<",\"cloud_export_workers\":2"
            <<",\"cloud_serialize_ms\":"<<cloudSerializeMs_<<",\"core_calls\":"<<coreCalls_
            <<",\"cloud_sample_builds\":"<<cloudSampleBuilds_<<",\"cloud_samples_bytes\":"<<cloudSamples_.sizeBytes()
            <<",\"redundant_core_frames_deferred\":"<<redundantCoreFrames_
            <<",\"continuity_state\":\""<<Gate::stateName(s.state)<<"\",\"main_map_write_allowed\":"<<(currentInputPresent_ && s.mainMapWriteAllowed?"true":"false")
            <<",\"input_present\":"<<(currentInputPresent_?"true":"false")
            <<",\"observation_count\":"<<s.mapObservationCount<<",\"observation_required\":"<<Gate::MAP_OBSERVATIONS
            <<",\"epoch\":"<<(closed_?closedEpoch_:core_.getResetEpoch())<<",\"cuts\":"<<s.cutCount<<",\"recoveries\":"<<recoveries_
            <<",\"main_nodes\":"<<(closed_?closedMainNodes_:mainPoses().size())<<",\"cloud_points\":"<<points_<<",\"database_closed\":"<<(closed_?"true":"false")<<",\"core_timings_ms\":{";
        bool comma=false;for(const auto &stat:coreTimings_){if(comma)f<<',';comma=true;f<<'"'<<stat.first<<"\":"<<stat.second;}f<<"}}\n";
        f.flush();if(!f)throw std::runtime_error("worker status failed");f.close();std::filesystem::rename(temporary,output_/"status.json");
    }
    Rtabmap core_;std::unique_ptr<Odometry> odom_;Gate gate_;Rtabmap::SafeRelocalizationLimits limits_{};
    AcceptedImageSourceContext context_;std::filesystem::path output_;std::ofstream trace_;
    long long source_=0,lastAttempt_=0,verifiedThrough_=0,jumpUntil_=0;
    long long lastCoreAttempt_=0;Transform lastAcceptedPose_;std::uint64_t redundantCoreFrames_=0;
    unsigned jumpSamples_=0;
    std::uint64_t frames_=0,missing_=0,points_=0,recoveries_=0,closedMainNodes_=0,closedEpoch_=0;
    bool written_=false,pending_=false,closed_=false;std::uint64_t writtenSerial_=0,writtenEpoch_=0,pendingSerial_=0,observationGeneration_=0;
    bool currentInputPresent_=false;
    int lastTrusted_=0,lastLocal_=0;Gate::Ticket pendingTicket_{};Transform seed_,lastCorrection_,previousPose_;
    std::vector<std::pair<Transform,Transform>> observations_;std::string reason_;
    std::vector<AcceptedTagCornerObservation> currentTags_,tagCorners_;
    std::map<int,std::map<int,AcceptedTagCornerObservation>> tags_;
    std::vector<AcceptedImageReceipt> tagReceipts_;std::vector<Link> tagLinks_;
    bool pendingTag_=false;long long lastReferenceNs_=0;Transform lastReferencePose_;
    double odomMs_=0,mapMs_=0;bool exported_=false;
    double odomEstimationMs_=0,odomBundleMs_=0,coreProcessMs_=0,freezeMs_=0,observationMs_=0,tagPreparationMs_=0,cloudExportMs_=0,tagExportMs_=0;
    std::uint64_t cloudNodeVisits_=0,cloudNodeBuilds_=0,cloudSampleBuilds_=0,coreCalls_=0;
    std::uint64_t cloudParallelExports_=0,cloudParallelNodes_=0;
    double cloudPrepareMs_=0,cloudTransformMs_=0,cloudDecodeMs_=0,cloudSerializeMs_=0;
    std::map<std::string,double> coreTimings_;
    std::map<int,Transform> exportedPoses_;std::map<int,std::string> exportedReceipts_;
    RecordedCloudCache cloudCache_;
    RecordedCloudAssembly cloudAssembly_;
    RecordedCloudSamples cloudSamples_;
};
}
