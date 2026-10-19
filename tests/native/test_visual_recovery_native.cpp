#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "ObservationDiagnostic.h"

using namespace rtabmap;
static void require(bool ok,const std::string & reason) {if(!ok) throw std::runtime_error(reason);}
static void requireReason(bool ok,const std::string & label,const std::string & reason) {if(!ok) throw std::runtime_error(label+reason);}
namespace rtabmap {
class ImageReceiptNativeTestAccess {
public:
    static bool process(Rtabmap & core,const SensorData & data,const Transform & pose,const ImageInputIdentity & input,std::string & reason) {
        if(!core.processWithAuthenticatedImage(data,pose,cv::Mat::eye(6,6,CV_64F)*0.001,{}, {},input,reason))return false;
        return core.validateAndFreezeCommittedEpochForSafeRelocalization(reason);
    }
    static bool processExtensionWithLoopChecks(Rtabmap & core,const SensorData & data,const Transform & pose,
            const ImageInputIdentity & input,std::string & reason) {
        if(!core.processWithAuthenticatedImage(data,pose,cv::Mat::eye(6,6,CV_64F)*0.001,{}, {},input,reason))return false;
        const auto original=snapshot(core);
        std::map<int,Transform> promoted;
        std::multimap<int,Link> promotedLinks;
        requireReason(core.evaluateCommittedEpochPromotion(core._optimizedPoses,core._constraints,promoted,promotedLinks,reason),
            "healthy extension preview failed: ",reason);
        require(original==snapshot(core),"preview modified live or trusted graph");
        const int current=core._memory->getLastWorkingSignature(true)->id(),old=38;
        const Transform exact=promoted.at(current).inverse()*promoted.at(old);
        auto candidate=core._constraints;
        candidate.emplace(current,Link(current,old,Link::kGlobalClosure,exact,cv::Mat::eye(6,6,CV_64F)*1000));
        requireReason(core.evaluateCommittedEpochPromotion(core._optimizedPoses,candidate,promoted,promotedLinks,reason),
            "consistent closure preview failed: ",reason);
        candidate=core._constraints;
        Transform wrong=exact.clone();wrong.x()+=.02f;
        candidate.emplace(current,Link(current,old,Link::kGlobalClosure,wrong,cv::Mat::eye(6,6,CV_64F)*1000));
        require(!core.evaluateCommittedEpochPromotion(core._optimizedPoses,candidate,promoted,promotedLinks,reason) &&
            reason.find("frozen-boundary constraint")!=std::string::npos,
            "statistically small loop disagreement bypassed frozen-boundary preview");
        require(original==snapshot(core) && core.isCurrentMapFrameRegistered(),"rejected candidate corrupted trusted state");
        requireReason(core.validateAndFreezeCommittedEpochForSafeRelocalization(reason),"odometry extension failed after candidate rejection: ",reason);
        std::cout<<"PASS read-only post-commit loop preview: consistent closure accepted, conflicting closure rejected before adoption, unchanged state and healthy odometry extension preserved\n";
        return true;
    }
    static std::set<int> candidateIds(const Rtabmap & core) {
        std::set<int> ids;for(const auto & candidate:core._crossMapLoopEvidence)ids.insert(candidate.newId);return ids;
    }
    static bool signatureExists(const Rtabmap & core,int id) {return core._memory->getSignature(id)!=nullptr;}
    static void checkCandidateEndpoints(const Rtabmap & core) {
        for(const auto & candidate:core._crossMapLoopEvidence) {
            AcceptedImageReceipt receipt;
            require(core._memory->getSignature(candidate.newId) && core._memory->getSignature(candidate.oldId) &&
                core.getAcceptedImageReceipt(candidate.newId,receipt) && core.getAcceptedImageReceipt(candidate.oldId,receipt),
                "native retrieval retained an evicted candidate endpoint");
        }
    }
    static int boundary(Rtabmap & core) {return core.triggerNewMapForContinuation();}
    static void clearCandidates(Rtabmap & core) {core._crossMapLoopEvidence.clear();}
    static void stationaryLocalization(Rtabmap & core,const SensorData & data,ImageInputIdentity identity) {
        const auto before=snapshot(core);
        Transform measured;Rtabmap::RecoveryObservationDiagnostics diagnostic;std::string reason;
        requireReason(core.localizeCurrentObservation(limits(),measured,diagnostic,reason,&data,&identity),
            "unsaved stationary observation failed localization: ",reason);
        require(measured.getDistance(Transform(.05f+.08f*19,0,0,0,0,0))<.025,"stationary localization incorrect");
        identity.pixelSha256=std::string(64,'a');
        require(!core.localizeCurrentObservation(limits(),measured,diagnostic,reason,&data,&identity) && measured.isNull(),
            "wrong live pixel identity passed localization");
        require(before==snapshot(core) && !core.isCurrentMapFrameRegistered(),"stationary localization changed map");
        std::cout<<"PASS source-validated stationary observation localized without retaining a new node; wrong pixels rejected\n";
    }
    static void independentLocalization(Rtabmap & core) {
        core._crossMapLoopEvidence.clear();
        const auto correction=core._mapCorrection;
        core._mapCorrection=Transform(10,0,0,0,0,0); // deliberately wrong local search prior
        const auto before=snapshot(core);
        Transform measured;Rtabmap::RecoveryObservationDiagnostics diagnostic;std::string reason;
        requireReason(core.localizeCurrentObservation(limits(),measured,diagnostic,reason),"independent localization failed: ",reason);
        require(!core.isCurrentMapFrameRegistered() && core._crossMapLoopEvidence.empty(),"localization granted graph registration");
        require(before==snapshot(core),"read-only localization changed live or trusted graph");
        require(diagnostic.globalSearch && diagnostic.firstAnchor>0 && diagnostic.secondAnchor>0 &&
            diagnostic.firstAnchor!=diagnostic.secondAnchor,"localization lacks two old anchors");
        require(measured.getDistance(Transform(.05f+.08f*19,0,0,0,0,0))<.025,"global search used incorrect local prediction");
        auto * current=const_cast<Signature *>(core._memory->getLastWorkingSignature(false));
        const auto savedLinks=current->getLinks();
        current->removeLink(current->id());
        require(!core.localizeCurrentObservation(limits(),measured,diagnostic,reason) && measured.isNull(),
            "missing measured gravity authorized localization");
        current->addLink(Link(current->id(),current->id(),Link::kGravity,Transform(0,0,0,.3f,0,0)));
        require(!core.localizeCurrentObservation(limits(),measured,diagnostic,reason) && measured.isNull(),
            "incompatible measured gravity authorized localization");
        current->removeLink(current->id());
        for(const auto & link:savedLinks)if(link.second.to()==current->id())current->addLink(link.second);
        require(before==snapshot(core),"gravity negative tests failed to restore graph");
        core._mapCorrection=correction;
        std::cout<<"PASS independent localization before three-bridge admission, wrong 10m prior, two raw old anchors, no graph mutation\n";
    }
    static Rtabmap::SafeRelocalizationLimits limits() {
        Rtabmap::SafeRelocalizationLimits l;
        l.maxHoldoutTranslation=.08f;l.maxHoldoutRotation=8;l.minBridgeTranslation=.02f;l.minBridgeRotation=2;
        l.maxAnchorZ=.02f;l.maxAnchorTilt=2;l.maxOldPoseTranslation=.005f;l.maxOldPoseRotation=.5f;
        l.maxNewRigidTranslation=.03f;l.maxNewRigidRotation=3;l.maxNewRigidZError=.02f;l.maxNewRigidTiltError=2;
        l.maxGravityResidualIncrease=.5f;l.maxInformationCondition=100000000.;return l;
    }
    static bool recover(Rtabmap & core,const AcceptedImageSourceContext & context,std::string & reason) {
        return core.tryVisualRecoveryWithAcceptedImages(limits(),context,[]{return true;},reason);
    }
    static bool localMotionFault(Rtabmap & core,Rtabmap::RecoveryLocalMotionFault & fault) {
        return core.findRecoveryLocalMotionFault(limits(),fault);
    }
    static bool cancelAfterShadow(Rtabmap & core,const AcceptedImageSourceContext & context,std::string & reason,int & calls) {
        return core.tryVisualRecoveryWithAcceptedImages(limits(),context,[&calls]{return ++calls==1;},reason);
    }
    static void candidates(Rtabmap & core,int newestOld=20) {
        core._crossMapLoopEvidence.clear();
        // Test-only endpoint selection. No transform is injected or trusted by
        // the method under test; every bridge is re-estimated from raw RGB-D.
        // All anchors are in the LAST three views. The healthy prefix has no
        // old-map bridge; its measured local trajectory is independently tested.
        for(int old=newestOld-2;old<=newestOld;++old) {
            Rtabmap::CrossMapLoopEvidence candidate{};
            const auto current=core._memory->getSignature(old+20),previous=core._memory->getSignature(old);
            require(current && previous,"candidate signatures missing");
            candidate.newId=old+20;candidate.oldId=old;
            candidate.newMapId=current->mapId();candidate.oldMapId=previous->mapId();
            candidate.newResetEpoch=1;candidate.oldResetEpoch=0;
            core._crossMapLoopEvidence.push_back(candidate);
        }
    }
    static void duplicateOld(Rtabmap & core) {core._crossMapLoopEvidence[1].oldId=core._crossMapLoopEvidence[0].oldId;}
    static void rawCandidates(Rtabmap & core) {
        core._crossMapLoopEvidence.clear();
        for(int id:{18,19,20}) {
            Signature current=*core._memory->getSignature(id+20),old=*core._memory->getSignature(id);
            // Different arbitrary cache numbering must not influence raw
            // support. The production collector must use its own solve objects.
            for(Signature * s:{&current,&old}) {
                std::multimap<int,int> shifted;
                for(const auto & word:s->getWords())shifted.emplace(word.first+1000000,word.second);
                s->setWords(shifted,s->getWordsKpts(),s->getWords3(),s->getWordsDescriptors());
            }
            std::string reason;
            require(!core.collectRawCrossMapLoopEvidence(current,old,1,0,reason),"candidate collection authorized native graph mutation");
            require(core._crossMapLoopEvidence.size()==static_cast<size_t>(id-17),"raw candidate not retained: "+reason);
            require(core._crossMapLoopEvidence.back().inlierWords.empty(),"pair-local IDs escaped into persistent candidate evidence");
        }
        core._crossMapLoopEvidence.clear();
    }
    static void trustedCandidateSearch(Rtabmap & core,const AcceptedImageSourceContext & context) {
        const auto saved=core._crossMapLoopEvidence;core._crossMapLoopEvidence.clear();
        const auto before=snapshot(core);std::string reason;
        // Native best-loop discovery is disabled in this fixture. Two earlier
        // raw views exist, but the newest observation has no native candidate.
        for(int old:{18,19}) {
            require(!core.collectRawCrossMapLoopEvidence(*core._memory->getSignature(old+20),
                *core._memory->getSignature(old),1,0,reason),"raw discovery authorized native addLink");
        }
        require(core._crossMapLoopEvidence.size()==2,"trusted search fixture requires two independent raw candidates");
        Transform proposed;
        require(!prepare(core,context,proposed,reason) && snapshot(core)==before,
            "two candidates unexpectedly prepared a merge");
        auto wrong=context;wrong.sessionTokenSha256=std::string(64,'9');
        require(!core.collectTrustedVisualRecoveryCandidates(wrong,reason) && core._crossMapLoopEvidence.size()==2,
            "wrong source context changed candidate pool");
        requireReason(core.collectTrustedVisualRecoveryCandidates(context,reason),"trusted-only retrieval failed: ",reason);
        require(core._crossMapLoopEvidence.size()==3 && core._crossMapLoopEvidence.back().newId==40 &&
            core._crossMapLoopEvidence.back().oldId<=20 && !core.isCurrentMapFrameRegistered() && snapshot(core)==before,
            "trusted retrieval failed to add current candidate or changed map authorization/geometry");
        require(core.collectTrustedVisualRecoveryCandidates(context,reason) && core._crossMapLoopEvidence.size()==3,
            "repeated observation duplicated candidates");
        requireReason(prepare(core,context,proposed,reason),"retrieved candidates did not pass unchanged preparation: ",reason);
        require(!core.isCurrentMapFrameRegistered() && snapshot(core)==before,"candidate discovery/preparation registered graph");
        core._crossMapLoopEvidence=saved;
        std::cout<<"PASS trusted-map retrieval without native loop winner: source-bound, three raw candidates, unchanged geometry and registration\n";
    }
    static bool sameEpoch(Rtabmap & core,bool wrong,std::string & reason) {
        Transform proposed=core._optimizedPoses.at(40).inverse()*core._optimizedPoses.at(38);
        if(wrong)proposed.x()+=.20f;
        return core.validateSameEpochVisualClosure(40,38,proposed,reason);
    }
    static bool observe(Rtabmap & core,const SensorData & data,const Transform & pose,std::string & reason) {
        Transform measured;return core.validateRegisteredMapObservation(data,pose,20,limits(),measured,reason);
    }
    static bool localObservation(Rtabmap & core,const SensorData & data,int cutoff,std::string & reason) {
        Transform measured;return core.validateLocalMapObservation(data,Transform(.05f+.08f*15,0,0,0,0,0),cutoff,limits(),measured,reason);
    }
    static bool prepare(Rtabmap & core,const AcceptedImageSourceContext & context,Transform & proposed,std::string & reason) {
        return core.prepareVisualRecoveryWithAcceptedImages(limits(),context,[]{return true;},proposed,reason);
    }
    static void depthProposal(Rtabmap & core,const AcceptedImageSourceContext & context) {
        std::vector<Link> endpoints;std::vector<AcceptedImageReceipt> receipts;
        std::vector<AcceptedTagCornerObservation> corners;
        for(int old:{16,18,20}) {
            // Deliberately inconsistent PnP hypotheses. These are NOT metric
            // evidence: Core must reopen its own depth and form the bridges.
            endpoints.emplace_back(old+20,old,Link::kUserClosure,
                Transform(float(old),0,0,float(old)*.05f,0,0),cv::Mat::eye(6,6,CV_64F));
            for(int id:{old+20,old}) {
                AcceptedImageReceipt receipt;require(core.getAcceptedImageReceipt(id,receipt),"depth fixture receipt missing");
                receipts.push_back(receipt);
                AcceptedTagCornerObservation corner;corner.signatureId=id;corner.tagId=13;
                corner.cornerOrder="opencv_aruco_canonical_0_1_2_3";corner.pixelRecordSha256=std::string(64,'a');
                corner.imageSha256=receipt.input.pixelSha256;
                const double x=279.5-8*((id-1)%20);
                corner.cornersXY={x-20,99.5,x+20,99.5,x+20,139.5,x-20,139.5};corners.push_back(corner);
            }
        }
        std::string why;Transform proposal;const auto before=snapshot(core);
        requireReason(core.prepareDepthCornerRecoveryWithAcceptedImages(endpoints,1,limits(),receipts,corners,
            context,[]{return true;},proposal,why),"depth proposal preparation failed: ",why);
        require(!proposal.isNull() && !core.isCurrentMapFrameRegistered() && snapshot(core)==before,
            "depth preparation mutated or registered graph");
        require(!core.commitDepthCornerRecoveryWithObservedAlignment(endpoints,1,limits(),receipts,corners,
            context,[](const Transform &){return false;},why) && snapshot(core)==before,
            "depth candidate bypassed observation certificate");
        auto wrongTag=corners;wrongTag.front().tagId=11;
        require(!core.prepareDepthCornerRecoveryWithAcceptedImages(endpoints,1,limits(),receipts,wrongTag,
            context,[]{return true;},proposal,why) && snapshot(core)==before,"mixed Tag IDs prepared");
        SensorData saved;blankCurrentDepth(core,saved,36);const auto blankSnapshot=snapshot(core);
        require(!core.prepareDepthCornerRecoveryWithAcceptedImages(endpoints,1,limits(),receipts,corners,
            context,[]{return true;},proposal,why) && snapshot(core)==blankSnapshot,
            "PnP proposal substituted for missing metric depth");
        restoreData(core,saved,36);
        require(snapshot(core)==before && !core.isCurrentMapFrameRegistered(),"depth negatives changed graph");
        std::cout<<"PASS inconsistent PnP proposal is candidate-only: native depth preparation, observation veto, mixed Tag and missing depth rejection; no graph mutation\n";
    }
    static bool observeProposal(Rtabmap & core,const SensorData & data,const Transform & pose,const Transform & proposed,Transform & measured,std::string & reason) {
        return core.validateProposedMapObservation(data,pose,20,proposed,limits(),measured,reason);
    }
    static bool observedCommit(Rtabmap & core,const AcceptedImageSourceContext & context,
            const std::vector<std::pair<Transform,Transform> > & observations,std::string & reason) {
        return core.commitVisualRecoveryWithObservedAlignment(limits(),context,[&](const Transform & finalCorrection) {
            if(observations.size()!=5)return false;
            for(size_t index=0;index<observations.size();++index) {
                const auto & sample=observations[index];
                const Transform predicted=finalCorrection*sample.first;
                if(predicted.getDistance(sample.second)>.025 || predicted.getAngle(sample.second)>2.0*3.141592653589793/180.0 ||
                   std::fabs(predicted.z()-sample.second.z())>.02) {
                    std::cout<<"observation callback rejected sample="<<index<<" distance="<<predicted.getDistance(sample.second)
                        <<" angle_deg="<<predicted.getAngle(sample.second)*180.0/3.141592653589793
                        <<" final_correction="<<finalCorrection.prettyPrint()<<" measured="<<sample.second.prettyPrint()<<'\n';
                    return false;
                }
            }
            return true;
        },reason);
    }
    static AcceptedImageReceipt removeReceipt(Rtabmap & core,int id) {
        const auto copy=core._acceptedImageReceipts.at(id);core._acceptedImageReceipts.erase(id);return copy;
    }
    static void restoreReceipt(Rtabmap & core,const AcceptedImageReceipt & receipt) {core._acceptedImageReceipts[receipt.signatureId]=receipt;}
    static void blankCurrentDepth(Rtabmap & core,SensorData & saved,int id=40) {
        auto signature=const_cast<Signature *>(core._memory->getSignature(id));saved=signature->sensorData();
        SensorData loaded=core._memory->getNodeData(id,true,false,false,false);loaded.uncompressData();
        signature->sensorData()=SensorData(loaded.imageRaw().clone(),cv::Mat::zeros(loaded.depthRaw().size(),CV_32FC1),
            loaded.cameraModels().front(),id,signature->getStamp());
    }
    static void restoreData(Rtabmap & core,const SensorData & data,int id=40) {const_cast<Signature *>(core._memory->getSignature(id))->sensorData()=data;}
    struct SavedNeighbors {Link forward,reverse;std::multimap<int,Link> graph;};
    static SavedNeighbors removeInteriorNeighbor(Rtabmap & core,int before=30) {
        const int after=before+1;
        auto a=const_cast<Signature *>(core._memory->getSignature(before));
        auto b=const_cast<Signature *>(core._memory->getSignature(after));
        require(a->getLinks().count(after)==1 && b->getLinks().count(before)==1,"fixture has no unique interior neighbor");
        SavedNeighbors saved{a->getLinks().find(after)->second,b->getLinks().find(before)->second,{}};
        require(saved.forward.type()==Link::kNeighbor && saved.reverse.type()==Link::kNeighbor,"fixture interior neighbor missing");
        a->removeLink(after);b->removeLink(before);
        for(auto row=core._constraints.begin();row!=core._constraints.end();)
            if((row->second.from()==before && row->second.to()==after)||(row->second.from()==after && row->second.to()==before)) {
                saved.graph.insert(*row);row=core._constraints.erase(row);
            }else ++row;
        return saved;
    }
    static void restoreInteriorNeighbor(Rtabmap & core,const SavedNeighbors & saved,bool corrupt=false) {
        const int before=saved.forward.from(),after=saved.forward.to();
        auto a=const_cast<Signature *>(core._memory->getSignature(before));
        auto b=const_cast<Signature *>(core._memory->getSignature(after));
        Transform forward=saved.forward.transform().clone();
        if(corrupt)forward.x()+=.10f;
        a->addLink(Link(before,after,Link::kNeighbor,forward,saved.forward.infMatrix().clone()));
        b->addLink(Link(after,before,Link::kNeighbor,forward.inverse(),saved.reverse.infMatrix().clone()));
        core._constraints.insert(saved.graph.begin(),saved.graph.end());
    }
    static std::string snapshot(const Rtabmap & core) {
        std::ostringstream out;out<<std::setprecision(17)<<core._resetEpoch<<' '<<core._safeRelocalizationTrustedSnapshotValid<<'\n';
        auto pose=[&](const Transform & p){if(p.isNull()){out<<"null";return;}for(int k=0;k<12;++k)out<<p[k]<<',';};
        auto link=[&](const Link & l){out<<l.from()<<','<<l.to()<<','<<l.type()<<',';pose(l.transform());for(int a=0;a<6;++a)for(int b=0;b<6;++b)out<<l.infMatrix().at<double>(a,b)<<',';out<<'\n';};
        out<<"mapCorrection ";pose(core._mapCorrection);out<<"\nlastLocalization ";pose(core._lastLocalizationPose);out<<'\n';
        for(auto epoch:core._safeRelocalizationCommittedEpochs)out<<"committed "<<epoch<<'\n';
        for(int id:core._safeRelocalizationArchivedIds)out<<"archived "<<id<<'\n';
        for(const auto & p:core._optimizedPoses){out<<"live "<<p.first<<' ';pose(p.second);out<<'\n';}
        for(const auto & p:core._safeRelocalizationTrustedPoses){out<<"trusted "<<p.first<<' ';pose(p.second);out<<'\n';}
        for(const auto & p:core._constraints)link(p.second);
        for(const auto & p:core._safeRelocalizationTrustedConstraints)link(p.second);
        for(int id=1;id<=core.getLastLocationId();++id)if(auto signature=core._memory->getSignature(id))for(const auto & p:signature->getLinks())link(p.second);
        return out.str();
    }
    static std::map<int,Transform> trusted(Rtabmap & core) {
        std::map<int,Transform> snapshot;
        for(const auto & pose:core._safeRelocalizationTrustedPoses)snapshot.emplace(pose.first,pose.second.clone());
        return snapshot;
    }
    static std::map<int,Transform> optimized(Rtabmap & core) {return core._optimizedPoses;}
    static bool unknownLocalization(Rtabmap & core) {return core._lastLocalizationPose.isNull();}
    static void assertScope(Rtabmap & core,int last=40) {
        const auto poses=trusted(core);
        std::set<int> expected;for(int id=1;id<=20;++id)expected.insert(id);
        for(int id=38;id<=last;++id)expected.insert(id);
        std::set<int> actual;for(const auto & pose:poses)actual.insert(pose.first);
        if(actual!=expected) {
            std::cerr<<"scope last="<<last<<" actual=";for(int id:actual)std::cerr<<id<<',';
            std::cerr<<" expected=";for(int id:expected)std::cerr<<id<<',';std::cerr<<'\n';
        }
        require(actual==expected,"registration included an excluded prefix or omitted the certified suffix");
        for(int id=21;id<38;++id) {
            AcceptedImageReceipt receipt;
            require(core.isRecoveryArchivedNode(id) && core.getAcceptedImageReceipt(id,receipt) &&
                receipt.resetEpoch==1 && core.getSignatureResetEpoch(id)==1,
                "archive changed original observation identity");
        }
        require(!core._memory->getSignature(37)->hasLink(38) && !core._memory->getSignature(38)->hasLink(37),
            "uncertified prefix is still joined to the registered suffix");
        for(const auto & row:core._constraints)
            require(!core.isRecoveryArchivedNode(row.second.from()) && !core.isRecoveryArchivedNode(row.second.to()),
                "archive edge leaked into main constraints");
        std::string why;
        require(!core.validateSameEpochVisualClosure(last,30,Transform::getIdentity(),why),
            "ordinary closure reattached an archived node");
    }
    static void diagnose(Rtabmap & core,const SensorData & current) {
        std::cout<<std::setprecision(12);
        for(int id:{16,15,9})for(bool tight:{false,true})
            observation_diagnostic::pair(current,core._memory->getNodeData(id,true,false,false,false),core._parameters,tight);
        observation_diagnostic::pair(core._memory->getNodeData(36,true,false,false,false),
            core._memory->getNodeData(16,true,false,false,false),core._parameters,false);
    }
    static void verifyStrictConsensus(Rtabmap & core,const SensorData & current) {
        const auto old=core._memory->getNodeData(16,true,false,false,false);
        const auto loose=observation_diagnostic::pair(current,old,core._parameters,false);
        const auto strict=observation_diagnostic::pair(current,old,core._parameters,true);
        require(loose.forward.above>0 || loose.reverse.above>0,
            "live-versus-stored fixture no longer contains default-10cm-consensus outliers");
        for(const auto & side:{strict.forward,strict.reverse})
            require(side.count>=400 && side.invalid==0 && side.above==0 && side.maximum<=.025 && side.rms<=.018,
                "tight 2.5cm consensus failed to preserve hundreds of good live/stored matches");
        std::cout<<"PASS strict consensus discards compression-sensitive >2.5cm correspondences while preserving >400 matches per direction; posterior thresholds unchanged\n";
    }
};
}
using Access=ImageReceiptNativeTestAccess;

int main(int argc,char ** argv) try {
    const bool prepareCloseOnly=argc==3 && std::string(argv[2])=="--prepare-close-only";
    const bool diagnoseObservation=argc==3 && std::string(argv[2])=="--diagnose-observation";
    const bool reseedPrivateOnly=argc==3 && std::string(argv[2])=="--reseed-private-only";
    const bool candidateLifecycle=argc==3 && std::string(argv[2])=="--candidate-lifecycle";
    const bool tagDepthProposal=argc==3 && std::string(argv[2])=="--tag-depth-proposal";
    require(argc==2 || prepareCloseOnly || diagnoseObservation || reseedPrivateOnly || candidateLifecycle || tagDepthProposal,"invalid native fixture arguments");
    require(std::filesystem::create_directory(argv[1]),"work directory must be new");
    cv::setNumThreads(1);cv::setRNGSeed(81015);ULogger::setLevel(ULogger::kWarning);
    ParametersMap p;
    p["Mem/GenerateIds"]="true";p["Mem/IncrementalMemory"]="true";p["Mem/BinDataKept"]="true";
    p["Mem/InitWMWithAllNodes"]="true";p["Mem/STMSize"]="100";p["Mem/BadSignaturesIgnored"]="false";
    p["Mem/RehearsalSimilarity"]="1";p["Mem/ReduceGraph"]="false";p["Mem/UseOdomGravity"]="false";
    p["Mem/ImagePreDecimation"]="1";p["Mem/ImagePostDecimation"]="1";
    p["RGBD/LinearUpdate"]="0";p["RGBD/AngularUpdate"]="0";p["RGBD/CreateOccupancyGrid"]="false";
    p["RGBD/ProximityBySpace"]="false";p["RGBD/ProximityByTime"]="false";p["RGBD/NeighborLinkRefining"]="false";
    p["Rtabmap/LoopThr"]="1";p["Rtabmap/DetectionRate"]="0";p["Kp/DetectorStrategy"]="8";
    if(candidateLifecycle) {p["Rtabmap/LoopThr"]="0.1";p["RGBD/LinearUpdate"]="0.04";p["RGBD/AngularUpdate"]="0.05";}
    p["Kp/MaxFeatures"]="1000";p["Vis/FeatureType"]="8";p["Vis/MaxFeatures"]="1000";
    p["Reg/Force3DoF"]="false";p["Optimizer/Strategy"]="1";p["Optimizer/Iterations"]="20";
    p["Optimizer/LandmarksIgnored"]="true";p["Optimizer/PriorsIgnored"]="true";p["Optimizer/GravitySigma"]="0.2";
    p["Rtabmap/WorkingDirectory"]=argv[1];
    Rtabmap core;core.init(p,"");core.setSafeRelocalizationGuardEnabled(true);
    require(core.isCurrentMapFrameRegistered(),"initial map is not registered");
    cv::Mat texture(240,512,CV_8UC1);cv::randu(texture,0,256);cv::GaussianBlur(texture,texture,cv::Size(3,3),0.6);
    const CameraModel camera(200,200,159.5,119.5,Transform::getIdentity(),0,cv::Size(320,240));
    AcceptedImageSourceContext context{std::string(64,'3'),std::string(64,'0'),std::string(48,'1'),std::string(48,'2'),"offline_native_fixture"};
    for(int index=0;index<40;++index) {
        if(index==20){require(Access::boundary(core)>=0,"boundary failed");require(!core.isCurrentMapFrameRegistered(),"unregistered segment leaked");}
        // Local tracking remains certified throughout. The 4s spacing and 76s
        // private span intentionally exceed former time-gap and length proxies.
        const int view=index%20;const double stamp=1000.0+4.0*index;
        cv::Mat rgb=texture(cv::Rect(view*8,0,320,240)).clone();cv::Mat depth(240,320,CV_32FC1,cv::Scalar(tagDepthProposal?.5:2));
        SensorData data(rgb,depth,camera,0,stamp);
        data.setIMU(IMU(cv::Vec4d(0,0,0,1),cv::Mat::eye(3,3,CV_64F)*.001,cv::Vec3d(0,0,0),
            cv::Mat::eye(3,3,CV_64F)*.001,cv::Vec3d(0,0,9.80665),cv::Mat::eye(3,3,CV_64F)*.001));
        ImageInputIdentity input;input.headerStampNs=std::int64_t(stamp*1e9);input.width=320;input.height=240;
        input.encoding="mono8";input.pixelSha256=canonicalImageSha256(rgb);input.K={200,0,159.5,0,200,119.5,0,0,1};
        input.D={0,0,0,0,0};input.sensorFrame="synthetic_camera_optical";input.provenanceScope=context.provenanceScope;
        input.sessionTokenSha256=context.sessionTokenSha256;input.databaseSha256=context.databaseSha256;
        input.rgbPublisherGidHex=context.rgbPublisherGidHex;input.infoPublisherGidHex=context.infoPublisherGidHex;
        std::string reason;require(Access::process(core,data,Transform(.05f+(tagDepthProposal?.02f:.08f)*view,0,0,0,0,0),input,reason),"input: "+reason);
        AcceptedImageReceipt receipt;require(core.getLastLocationId()==index+1 && core.getAcceptedImageReceipt(index+1,receipt),"receipt missing: "+reason);
    }
    if(tagDepthProposal) {Access::depthProposal(core,context);core.close(false);return 0;}
    if(candidateLifecycle) {
        const auto before=Access::candidateIds(core);
        require(before.size()>=2,"native retrieval did not produce candidates; lifecycle fixture is inconclusive");
        AcceptedImageReceipt previous;require(core.getAcceptedImageReceipt(40,previous),"lifecycle latest receipt missing");
        SensorData duplicate(texture(cv::Rect(19*8,0,320,240)).clone(),cv::Mat(240,320,CV_32FC1,cv::Scalar(2)),camera,0,1160.0);
        duplicate.setIMU(IMU(cv::Vec4d(0,0,0,1),cv::Mat::eye(3,3,CV_64F)*.001,cv::Vec3d(),cv::Mat(),cv::Vec3d(),cv::Mat()));
        auto identity=previous.input;identity.headerStampNs=1160000000000LL;identity.pixelSha256=canonicalImageSha256(duplicate.imageRaw());
        std::string why;
        Access::process(core,duplicate,Transform(.05f+.08f*19,0,0,0,0,0),identity,why);
        require(!Access::signatureExists(core,41),"small-displacement cleanup was not exercised");
        Access::checkCandidateEndpoints(core);
        require(!Access::candidateIds(core).count(41),"candidate retained after its same-process deletion");
        Access::stationaryLocalization(core,duplicate,identity);
        core.close(false);
        std::cout<<"PASS real process retrieval -> candidate collection -> small-displacement cleanup; retained candidates have live receipts/endpoints\n";
        return 0;
    }
    Access::independentLocalization(core);
    const auto oldPoses=Access::trusted(core);
    std::vector<Rtabmap::LocalSubmapSnapshot> local;std::string snapshotReason;
    const auto beforeLocal=Access::snapshot(core);
    requireReason(core.getUnregisteredLocalSubmaps(local,snapshotReason),"local snapshot failed: ",snapshotReason);
    require(local.size()==1 && local.front().componentId==21 && local.front().resetEpoch==1 &&
        !local.front().registeredToMain && local.front().poses.size()==20 && local.front().poses.at(21).isIdentity(),
        "private snapshot identity/registration/normalization incorrect");
    for(const auto & row:local.front().poses)require(row.first>=21 && row.first<=40,"trusted old node leaked into local snapshot");
    require(local.front().links.size()>=19,"snapshot lost measured local neighbor geometry");
    for(const auto & row:local.front().links)require(row.second.from()>=21 && row.second.to()>=21 && row.second.from()!=row.second.to(),
        "local snapshot contains non-local or absolute edge");
    local.front().poses.at(25).x()+=10;
    auto altered=local.front().links.begin()->second.transform();altered.x()+=10;
    require(beforeLocal==Access::snapshot(core),"read-only snapshot aliases live graph transforms");
    require(oldPoses.size()==20 && !oldPoses.count(21),"unregistered submap entered trusted main map");
    std::cout<<"PASS independent local snapshot: first node identity, real local links only, no main nodes, deep-copy isolation\n";
    if(reseedPrivateOnly) {
        require(Access::boundary(core)>=0 && !core.isCurrentMapFrameRegistered(),"private-to-private boundary failed");
        const auto trustedAfter=Access::trusted(core);
        require(trustedAfter.size()==oldPoses.size(),"private boundary promoted unregistered nodes");
        for(const auto & pose:oldPoses)for(int k=0;k<12;++k)require(trustedAfter.at(pose.first)[k]==pose.second[k],"private boundary changed trusted pose");
        require(core.getUnregisteredLocalSubmaps(local,snapshotReason) && local.empty(),"private boundary kept previous epoch in current snapshot");
        core.close(false);
        std::cout<<"PASS private-to-private loss boundary leaves previous submap unregistered and frozen trusted graph unchanged\n";
        return 0;
    }
    if(diagnoseObservation) {
        SensorData current(texture(cv::Rect(15*8,0,320,240)).clone(),cv::Mat(240,320,CV_32FC1,cv::Scalar(2)),camera,0,1042.5);
        Access::diagnose(core,current);core.close(false);return 0;
    }
    SensorData liveStoredFixture(texture(cv::Rect(15*8,0,320,240)).clone(),cv::Mat(240,320,CV_32FC1,cv::Scalar(2)),camera,0,1042.5);
    Access::verifyStrictConsensus(core,liveStoredFixture);
    {
        std::string why;const auto before=Access::snapshot(core);
        require(Access::localObservation(core,liveStoredFixture,40,why),
            "healthy private checkpoint failed: "+why);
        require(before==Access::snapshot(core) && !core.isCurrentMapFrameRegistered(),"local observation registered or mutated private graph");
        require(!Access::localObservation(core,liveStoredFixture,20,why),
            "historical epoch used as local continuity checkpoint");
    }
    Access::rawCandidates(core);
    std::cout<<"PASS raw candidate support uses the solved signatures despite unrelated cached word IDs\n";
    Access::trustedCandidateSearch(core,context);
    std::string reason;
    auto reject=[&](const AcceptedImageSourceContext & source,const std::string & label) {
        const auto before=Access::snapshot(core);require(!Access::recover(core,source,reason),label+" unexpectedly committed");
        require(before==Access::snapshot(core),label+" mutated the graph");require(!core.isCurrentMapFrameRegistered(),label+" registered the segment");
        std::cout<<"PASS "<<label<<": "<<reason<<'\n';
    };
    // The real native detector may already have populated this buffer while
    // processing the synthetic return. Establish the negative precondition.
    Access::clearCandidates(core);reject(context,"no native candidates");
    // Candidate age is source-time based. A newer measured frame need not
    // itself retrieve an old view, but the full intervening suffix must pass.
    Access::candidates(core,19);
    Transform recentProposal;const auto beforeRecent=Access::snapshot(core);
    requireReason(Access::prepare(core,context,recentProposal,reason),"recent candidate behind latest failed: ",reason);
    require(beforeRecent==Access::snapshot(core),"recent preparation mutated graph");
    const auto missingRecentEdge=Access::removeInteriorNeighbor(core,39);
    const auto beforeMissingRecent=Access::snapshot(core);
    require(!Access::prepare(core,context,recentProposal,reason) && beforeMissingRecent==Access::snapshot(core),
        "recent candidate bypassed its missing edge to the latest node");
    Access::restoreInteriorNeighbor(core,missingRecentEdge);
    Access::candidates(core,18);const auto beforeStale=Access::snapshot(core);
    require(!Access::prepare(core,context,recentProposal,reason) && beforeStale==Access::snapshot(core) &&
        reason.find("source-stale")!=std::string::npos,"stale source-time candidate was not explicitly rejected");
    std::cout<<"PASS recent nonlatest candidate certifies complete suffix; missing continuation and stale source time rejected\n";
    Access::candidates(core);Access::duplicateOld(core);reject(context,"duplicate old endpoint");
    Access::candidates(core);auto missing=Access::removeReceipt(core,18);reject(context,"missing accepted anchor image");Access::restoreReceipt(core,missing);
    missing=Access::removeReceipt(core,21);reject(context,"missing first unanchored local image");Access::restoreReceipt(core,missing);
    auto badContext=context;badContext.sessionTokenSha256=std::string(64,'9');reject(badContext,"wrong session");
    SensorData saved;Access::blankCurrentDepth(core,saved);reject(context,"zero depth cannot relocalize");Access::restoreData(core,saved);
    Access::blankCurrentDepth(core,saved,30);reject(context,"untrusted interior raw depth cannot be backfilled");Access::restoreData(core,saved,30);
    Rtabmap::RecoveryLocalMotionFault fault;
    const auto beforeHealthyMotionCheck=Access::snapshot(core);
    require(!Access::localMotionFault(core,fault) && fault.fromId==0 &&
        beforeHealthyMotionCheck==Access::snapshot(core),"healthy raw local chain reported a motion fault or changed graph");
    const auto neighbors=Access::removeInteriorNeighbor(core);
    reject(context,"disconnected interior neighbor cannot be guessed from late anchors");
    require(!Access::localMotionFault(core,fault),"missing neighbor was treated as a measured motion disagreement");
    requireReason(core.getUnregisteredLocalSubmaps(local,snapshotReason),"split local snapshot failed: ",snapshotReason);
    require(local.size()==2 && local[0].componentId==21 && local[1].componentId==31 &&
        local[0].poses.at(21).isIdentity() && local[1].poses.at(31).isIdentity(),"disconnected submaps were implicitly connected in display");
    Access::restoreInteriorNeighbor(core,neighbors,true);
    reject(context,"false interior motion edge cannot be certified by valid late anchors");
    const auto beforeFault=Access::snapshot(core);
    require(Access::localMotionFault(core,fault) && fault.resetEpoch==core.getResetEpoch() &&
        fault.fromId==30 && fault.toId==31 && beforeFault==Access::snapshot(core),
        "raw-supported interior fault was not identified read-only with exact epoch/endpoints");
    Access::blankCurrentDepth(core,saved,30);
    require(!Access::localMotionFault(core,fault) && fault.fromId==0 && fault.toId==0,
        "missing raw depth reused a previous motion fault");
    Access::restoreData(core,saved,30);
    Access::removeInteriorNeighbor(core);Access::restoreInteriorNeighbor(core,neighbors);
    const auto beforeLoop=Access::snapshot(core);
    require(Access::sameEpoch(core,false,reason),"correct same-epoch loop failed two-anchor validation: "+reason);
    require(!Access::sameEpoch(core,true,reason) && beforeLoop==Access::snapshot(core),"incorrect same-epoch loop was accepted or mutated graph");
    int ticketChecks=0;const auto beforeCancel=Access::snapshot(core);
    require(!Access::cancelAfterShadow(core,context,reason,ticketChecks) && ticketChecks==2 && beforeCancel==Access::snapshot(core),
        "loss during raw geometry/shadow solve was not rejected at final commit: "+reason);
    std::cout<<"PASS continuity loss immediately before first graph mutation: "<<reason<<'\n';
    // The exact failure seen in v8: an earlier raw/recorded contradiction must
    // still veto full backfill, but cannot veto a separately certified suffix.
    Access::removeInteriorNeighbor(core);Access::restoreInteriorNeighbor(core,neighbors,true);
    reject(context,"full historical backfill remains rejected by an earlier motion contradiction");
    Transform proposed;
    const auto suffixNeighbors=Access::removeInteriorNeighbor(core,38);
    const auto missingSuffix=Access::snapshot(core);
    require(!Access::prepare(core,context,proposed,reason) && missingSuffix==Access::snapshot(core),
        "current reanchor invented an unmeasured edge inside its suffix");
    Access::restoreInteriorNeighbor(core,suffixNeighbors);
    Access::removeInteriorNeighbor(core,25);
    const auto beforePreparation=Access::snapshot(core);
    requireReason(Access::prepare(core,context,proposed,reason),"valid preparation failed: ",reason);
    require(!proposed.isNull() && !core.isCurrentMapFrameRegistered() && beforePreparation==Access::snapshot(core),
        "prepare-only recovery changed graph/correction/registration");
    std::cout<<"PASS current reanchor prepares the final three views despite an earlier unregistered motion contradiction\n";
    if(prepareCloseOnly) {
        const std::string path=std::string(argv[1])+"/prepared_closed.db";
        core.close(true,path);
        require(std::filesystem::exists(path),"prepared close did not save private test database");
        Rtabmap reopened;reopened.init(p,path);reopened.setSafeRelocalizationGuardEnabled(true);
        const auto savedPoses=Access::optimized(reopened);
        require(savedPoses.size()==oldPoses.size(),"prepared close saved private candidate optimized poses");
        for(const auto & pose:oldPoses) {
            require(savedPoses.count(pose.first),"prepared close lost trusted pose");
            for(int k=0;k<12;++k)require(savedPoses.at(pose.first)[k]==pose.second[k],"prepared close changed trusted pose");
        }
        require(Access::unknownLocalization(reopened),"uncommitted prepared close persisted a localized robot pose");
        reopened.close(false);
        std::cout<<"PASS prepare then close/reopen: saved optimized graph contains only unchanged old trusted poses and unknown localization. Raw private observations may remain isolated in the test database.\n";
        return 0;
    }
    std::vector<std::pair<Transform,Transform> > observed;
    require(!Access::observedCommit(core,context,observed,reason) && beforePreparation==Access::snapshot(core),
        "zero observations incorrectly allowed a commit");
    for(int view=15;view<20;++view) {
        SensorData fresh(texture(cv::Rect(view*8,0,320,240)).clone(),cv::Mat(240,320,CV_32FC1,cv::Scalar(2)),camera,0,1041.0+.1*view);
        const Transform odom(.05f+.08f*view,0,0,0,0,0);Transform measured;
        requireReason(Access::observeProposal(core,fresh,odom,proposed,measured,reason),"precommit observation failed: ",reason);
        observed.emplace_back(odom,measured);
    }
    require(!core.isCurrentMapFrameRegistered() && beforePreparation==Access::snapshot(core),"observations changed unregistered graph");
    // Transform's default copy shares its cv::Mat. Deep-copy before fault
    // injection so the following positive certificate remains unchanged.
    const Transform originalObservation=observed[2].second.clone();
    auto wrongObserved=observed;wrongObserved[2].second=observed[2].second.clone();wrongObserved[2].second.x()+=.10f;
    for(int k=0;k<12;++k)require(observed[2].second[k]==originalObservation[k],"negative certificate modified original observation through shallow copy");
    require(!Access::observedCommit(core,context,wrongObserved,reason) && beforePreparation==Access::snapshot(core),
        "invalid observation certificate changed graph");
    requireReason(Access::observedCommit(core,context,observed,reason),"valid observed RGB-D recovery failed: ",reason);
    require(core.isCurrentMapFrameRegistered(),"successful recovery did not register");
    require(core.getUnregisteredLocalSubmaps(local,snapshotReason) && local.size()==2 &&
        local[0].poses.size()==5 && local[0].poses.count(21) && local[0].poses.at(21).isIdentity() &&
        local[1].poses.size()==12 && local[1].poses.count(26) && local[1].poses.at(26).isIdentity() && !local[1].poses.count(38),
        "excluded historical prefix was lost or relabeled as registered");
    Access::assertScope(core);
    const auto now=Access::trusted(core);
    for(const auto & pose:oldPoses){require(now.count(pose.first),"old pose disappeared");for(int k=0;k<12;++k)require(now.at(pose.first)[k]==pose.second[k],"old pose moved");}
    const auto beforeRepeat=Access::snapshot(core);require(!Access::recover(core,context,reason) && beforeRepeat==Access::snapshot(core),"duplicate recovery changed graph");
    SensorData observation(texture(cv::Rect(19*8,0,320,240)).clone(),cv::Mat(240,320,CV_32FC1,cv::Scalar(2)),camera,0,1041.0);
    requireReason(Access::observe(core,observation,Transform(.05f+.08f*19,0,0,0,0,0),reason),"registered read-only two-anchor observation failed: ",reason);
    require(!Access::observe(core,observation,Transform(.15f+.08f*19,0,0,0,0,0),reason),"incorrect observation pose passed");
    require(beforeRepeat==Access::snapshot(core),"read-only map observations mutated graph");
    // Continue normal mapping, then save/reopen: neither graph traversal nor
    // persisted neighbor links may resurrect the isolated historical prefix.
    SensorData extension(texture(cv::Rect(20*8,0,320,240)).clone(),cv::Mat(240,320,CV_32FC1,cv::Scalar(2)),camera,0,1160.0);
    extension.setIMU(IMU(cv::Vec4d(0,0,0,1),cv::Mat::eye(3,3,CV_64F)*.001,cv::Vec3d(0,0,0),
        cv::Mat::eye(3,3,CV_64F)*.001,cv::Vec3d(0,0,9.80665),cv::Mat::eye(3,3,CV_64F)*.001));
    AcceptedImageReceipt lastReceipt;require(core.getAcceptedImageReceipt(40,lastReceipt),"last receipt absent");
    auto input=lastReceipt.input;input.headerStampNs=1160000000000LL;input.pixelSha256=canonicalImageSha256(extension.imageRaw());
    requireReason(Access::processExtensionWithLoopChecks(core,extension,Transform(.05f+.08f*20,0,0,0,0,0),input,reason),"continuation failed: ",reason);
    Access::assertScope(core,41);
    const auto expectedSaved=Access::trusted(core);
    const std::string savedPath=std::string(argv[1])+"/reanchored_closed.db";
    core.close(true,savedPath);
    Rtabmap reopened;reopened.init(p,savedPath);reopened.setSafeRelocalizationGuardEnabled(true);
    const auto persisted=Access::optimized(reopened);
    require(persisted.size()==expectedSaved.size(),"save/reopen changed registration scope");
    for(const auto & pose:expectedSaved)require(persisted.count(pose.first) &&
        persisted.at(pose.first).getDistance(pose.second)<1e-6 && persisted.at(pose.first).getAngle(pose.second)<1e-6,
        "save/reopen changed a registered pose");
    reopened.close(false);
    std::cout<<"PASS native no-Tag raw RGB-D transaction: preparation and observation do not mutate graph, certificate-gated registration, old poses unchanged, negatives unchanged, repeat rejected. Synthetic texture/depth/odometry/IMU and test-selected candidate IDs; no hardware or recorded retrieval proof.\n";
    return 0;
} catch(const std::exception & error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
