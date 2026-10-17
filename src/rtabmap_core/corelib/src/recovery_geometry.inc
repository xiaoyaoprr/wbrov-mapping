// Pure raw-observation checks shared by cache certification and recovery
// observation. No Memory, graph, receipt, or global transform is modified.
namespace {
struct RecoveryPairResult {
    Transform forward,reverse;
};

bool strictRecoveryPair(SensorData fromData,SensorData toData,const ParametersMap & parameters,
        double maximumCondition,RecoveryPairResult & result,std::string & reason)
{
    fromData.uncompressData();toData.uncompressData();
    if(fromData.imageRaw().empty() || toData.imageRaw().empty() || fromData.depthRaw().empty() ||
       toData.depthRaw().empty() || fromData.cameraModels().size()!=1 || toData.cameraModels().size()!=1)
    {reason="raw bidirectional RGB-D observations unavailable";return false;}
    fromData.setFeatures({}, {},cv::Mat());toData.setFeatures({}, {},cv::Mat());
    ParametersMap p=parameters;
    int minimum=25,cells=4,configured=Parameters::defaultVisMinInliers();
    float span=0.02f,hessian=0.0001f;
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinInliers(),minimum);
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinGridCells(),cells);
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMin3DSpan(),span);
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinHessianRatio(),hessian);
    Parameters::parse(parameters,Parameters::kVisMinInliers(),configured);
    minimum=std::max(25,std::max(minimum,configured));cells=std::max(4,cells);
    span=std::max(.02f,span);hessian=std::max(.0001f,hessian);
    float correspondenceDistance=Parameters::defaultVisInlierDistance();
    Parameters::parse(parameters,Parameters::kVisInlierDistance(),correspondenceDistance);
    if(!std::isfinite(correspondenceDistance) || correspondenceDistance<=0)
    {reason="raw RGB-D correspondence distance is invalid";return false;}
    p[Parameters::kRegStrategy()]="0";p[Parameters::kRegForce3DoF()]="false";
    p[Parameters::kVisEstimationType()]="0";p[Parameters::kVisMinInliers()]=uNumber2Str(minimum);
    // RANSAC must classify correspondences using the same strict metric cap
    // as the posterior certificate. Its default 10cm consensus includes a few
    // compression-sensitive mismatches among hundreds of valid 2.5cm matches.
    // Tighten selection; retain every posterior residual/coverage/SPD check.
    p[Parameters::kVisInlierDistance()]=uNumber2Str(std::min(.025f,correspondenceDistance));
    auto validate=[&](const Signature & a,const Signature & b,const Transform & transform,const RegistrationInfo & info)->bool {
        if(!safeRigidTransform(transform) || info.inliers<minimum) return false;
        std::set<int> words(info.inliersIDs.begin(),info.inliersIDs.end());
        if(int(words.size())<minimum) return false;
        for(const Signature * s:{&a,&b}) {
            int width=0,height=0;double extent=0.0,conditioning=0.0;
            const auto points=validInlierPoints(*s,words);
            if(!signatureImageSize(*s,width,height) || occupiedGridCells(*s,words,width,height)<cells ||
               int(points.size())<minimum || !pointSetGeometry(points,extent,conditioning) ||
               extent<span || conditioning<hessian) return false;
        }
        double linear=0.0,angular=0.0;cv::Mat eigenvalues;
        if(!std::isfinite(maximumCondition) || maximumCondition<1 ||
           !covarianceVariances(info.covariance,linear,angular) || !cv::checkRange(info.covariance) ||
           cv::norm(info.covariance-info.covariance.t(),cv::NORM_INF)>1e-9 ||
           !cv::eigen(info.covariance,eigenvalues) || eigenvalues.at<double>(5)<=0 ||
           eigenvalues.at<double>(0)/eigenvalues.at<double>(5)>maximumCondition) return false;
        double squared=0.0,maximum=0.0;int count=0;
        for(int word:words) {
            const auto x=a.getWords().equal_range(word),y=b.getWords().equal_range(word);
            if(std::distance(x.first,x.second)!=1 || std::distance(y.first,y.second)!=1) return false;
            const int i=x.first->second,j=y.first->second;
            if(i<0 || j<0 || i>=int(a.getWords3().size()) || j>=int(b.getWords3().size())) return false;
            const auto & q=a.getWords3()[i];const auto & r=b.getWords3()[j];
            if(!finitePoint(q) || !finitePoint(r)) return false;
            double error=cv::norm(cv::Point3d(q.x,q.y,q.z)-transformPoint(transform,cv::Point3d(r.x,r.y,r.z)));
            if(!std::isfinite(error)) return false;
            squared+=error*error;maximum=std::max(maximum,error);++count;
        }
        return count>=minimum && std::sqrt(squared/count)<=.018 && maximum<=.025;
    };
    Signature a(fromData),b(toData),reverseB(toData),reverseA(fromData);
    std::unique_ptr<Registration> forward(Registration::create(p)),reverse(Registration::create(p));
    if(!forward || !reverse || !forward->isImageRequired() || !reverse->isImageRequired())
    {reason="raw visual registration unavailable";return false;}
    RegistrationInfo f,r;
    result.forward=forward->computeTransformationMod(a,b,Transform(),&f);
    result.reverse=reverse->computeTransformationMod(reverseB,reverseA,Transform(),&r);
    const bool forwardValid=validate(a,b,result.forward,f),reverseValid=validate(reverseB,reverseA,result.reverse,r);
    if(!forwardValid || !reverseValid ||
       result.forward.getDistance(result.reverse.inverse())>.025 ||
       result.forward.getAngle(result.reverse.inverse())>2.0*M_PI/180.0)
    {reason=uFormat("raw bidirectional RGB-D check failed (f=%d,r=%d,finliers=%d,rinliers=%d,fbackend=%s,rbackend=%s)",
        forwardValid?1:0,reverseValid?1:0,f.inliers,r.inliers,f.rejectedMsg.c_str(),r.rejectedMsg.c_str());return false;}
    return true;
}

bool strictRecoveryPoseAgreement(const Transform & a,const Transform & b,
        const Rtabmap::SafeRelocalizationLimits & limits)
{
    return safeRigidTransform(a) && safeRigidTransform(b) &&
        a.getDistance(b)<=std::min(.025,double(limits.maxHoldoutTranslation)) &&
        a.getAngle(b)<=std::min(2.0,double(limits.maxHoldoutRotation))*M_PI/180.0 &&
        std::fabs(a.z()-b.z())<=std::min(.02,double(limits.maxAnchorZ)) &&
        safeTiltError(a,b)<=std::min(2.0,double(limits.maxAnchorTilt))*M_PI/180.0;
}
}

bool Rtabmap::validateRecoveryCache(const std::set<int> & ids,const std::vector<Link> & evidence,
        const SafeRelocalizationLimits & limits,std::string & reason) const
{
    // A locally observed, continuous submap can be aligned by three anchors
    // anywhere in it. That is different from inventing a trajectory through a
    // tracking gap: every retained neighbor below needs independent raw RGB-D
    // evidence. Wrapper starts a new epoch after any loss of local confidence.
    // Unknown intervals are never filled by this transaction.
    if(ids.size()<3 || evidence.size()!=3)
    {reason="submap alignment requires a complete continuous segment and three independent bridges";return false;}
    std::vector<int> sequence(ids.begin(),ids.end());
    std::vector<Transform> forward(sequence.size(),Transform::getIdentity()),backward(sequence.size(),Transform::getIdentity());
    std::vector<RecoveryPairResult> pairs;
    double lastStamp=0.0;std::int64_t lastInputStamp=0;
    AcceptedImageReceipt firstReceipt;
    if(!getAcceptedImageReceipt(sequence.front(),firstReceipt)){reason="submap first receipt missing";return false;}
    for(int id:sequence) {
        const Signature * signature=_memory->getSignature(id);AcceptedImageReceipt receipt;
        if(!signature || !getAcceptedImageReceipt(id,receipt) || receipt.resetEpoch!=_resetEpoch ||
           getSignatureResetEpoch(id)!=_resetEpoch || signature->mapId()!=firstReceipt.mapId || receipt.mapId!=firstReceipt.mapId ||
           !std::isfinite(signature->getStamp()) || signature->getStamp()<=lastStamp ||
           receipt.input.headerStampNs<=lastInputStamp ||
           receipt.input.sessionTokenSha256!=firstReceipt.input.sessionTokenSha256 ||
           receipt.input.databaseSha256!=firstReceipt.input.databaseSha256 ||
           receipt.input.rgbPublisherGidHex!=firstReceipt.input.rgbPublisherGidHex ||
           receipt.input.infoPublisherGidHex!=firstReceipt.input.infoPublisherGidHex ||
           receipt.input.provenanceScope!=firstReceipt.input.provenanceScope)
        {reason=uFormat("submap node %d has missing, reordered, cross-epoch/map or changed-source observations",id);return false;}
        lastStamp=signature->getStamp();lastInputStamp=receipt.input.headerStampNs;
    }
    for(size_t index=1;index<sequence.size();++index) {
        const int before=sequence[index-1],after=sequence[index];
        const Signature * a=_memory->getSignature(before),*b=_memory->getSignature(after);
        const Link * recordedForward=nullptr,*recordedReverse=nullptr;
        for(const auto & edge:a->getLinks())if(edge.second.to()==after && edge.second.type()==Link::kNeighbor) {
            if(recordedForward){reason="submap has duplicated neighbor measurements";return false;}recordedForward=&edge.second;
        }
        for(const auto & edge:b->getLinks())if(edge.second.to()==before && edge.second.type()==Link::kNeighbor) {
            if(recordedReverse){reason="submap has duplicated reverse neighbor measurements";return false;}recordedReverse=&edge.second;
        }
        if(!recordedForward || !recordedReverse)
        {reason=uFormat("submap continuity breaks at %d->%d: missing measured neighbor; uncertain gap must remain separate",before,after);return false;}
        RecoveryPairResult pair;
        if(!strictRecoveryPair(_memory->getNodeData(before,true,false,false,false),
            _memory->getNodeData(after,true,false,false,false),_parameters,limits.maxInformationCondition,pair,reason)) {
            reason=uFormat("submap continuity %d->%d lacks raw evidence; no gap backfill: %s",before,after,reason.c_str());return false;
        }
        const Transform odomRelative=a->getPose().inverse()*b->getPose();
        if(!strictRecoveryPoseAgreement(pair.forward,odomRelative,limits) ||
           !strictRecoveryPoseAgreement(pair.forward,recordedForward->transform(),limits) ||
           !strictRecoveryPoseAgreement(pair.reverse,recordedReverse->transform(),limits))
        {reason=uFormat("submap continuity %d->%d raw motion disagrees with its odometry or measured neighbor edge",before,after);return false;}
        pairs.push_back(pair);forward[index]=forward[index-1]*pair.forward;
    }
    for(size_t index=sequence.size()-1;index>0;--index)backward[index-1]=backward[index]*pairs[index-1].reverse;
    std::vector<Transform> firstAlignments,lastAlignments;
    for(const Link & bridge:evidence) {
        const auto node=std::lower_bound(sequence.begin(),sequence.end(),bridge.from());
        const auto old=_safeRelocalizationTrustedPoses.find(bridge.to());
        if(node==sequence.end() || *node!=bridge.from() || old==_safeRelocalizationTrustedPoses.end())
        {reason="submap bridge endpoint is outside the current segment or trusted main map";return false;}
        const size_t index=std::distance(sequence.begin(),node);
        const Transform mainAtAnchor=old->second*bridge.transform().inverse();
        firstAlignments.push_back(mainAtAnchor*forward[index].inverse());
        lastAlignments.push_back(mainAtAnchor*backward[index].inverse());
    }
    for(size_t i=0;i<firstAlignments.size();++i)for(size_t j=0;j<i;++j)
        if(!strictRecoveryPoseAgreement(firstAlignments[i],firstAlignments[j],limits) ||
           !strictRecoveryPoseAgreement(lastAlignments[i],lastAlignments[j],limits))
        {reason="submap raw trajectory yields inconsistent main-map alignments from the three independent anchors";return false;}
    const Transform mainFirst=firstAlignments.front(),mainLast=lastAlignments.back();
    for(size_t index=0;index<sequence.size();++index) {
        if(!strictRecoveryPoseAgreement(mainFirst*forward[index],mainLast*backward[index],limits))
        {reason="submap forward/backward trajectories disagree under independent main-map anchors";return false;}
        const Transform stored=mainFirst*_optimizedPoses.at(sequence.front()).inverse()*_optimizedPoses.at(sequence[index]);
        if(!strictRecoveryPoseAgreement(mainFirst*forward[index],stored,limits))
        {reason="submap raw trajectory disagrees with stored local geometry";return false;}
    }
    return true;
}

bool Rtabmap::findRecoveryLocalMotionFault(const SafeRelocalizationLimits & limits,
        RecoveryLocalMotionFault & fault) const
{
    fault=RecoveryLocalMotionFault{};
    if(!_safeRelocalizationGuardEnabled || !_memory || isCurrentMapFrameRegistered())return false;
    // Recheck a rejected cache independently of its old-map/Tag hypothesis.
    // A wrong loop, missing depth, weak registration or stale proposal is not
    // proof that the local segment itself has a discontinuity. This read-only
    // check can only report a contradiction supported by a valid raw pair.
    const Signature * previous=nullptr;
    for(const auto & entry:_optimizedPoses) {
        if(entry.first<=0 || getSignatureResetEpoch(entry.first)!=_resetEpoch)continue;
        const Signature * current=_memory->getSignature(entry.first);
        AcceptedImageReceipt receipt;
        if(!current || !getAcceptedImageReceipt(entry.first,receipt) || receipt.resetEpoch!=_resetEpoch ||
           receipt.mapId!=current->mapId())return false;
        if(previous) {
            if(previous->mapId()!=current->mapId() || previous->getStamp()>=current->getStamp())return false;
            const Link * forward=nullptr,*reverse=nullptr;
            for(const auto & edge:previous->getLinks())if(edge.second.to()==current->id() && edge.second.type()==Link::kNeighbor) {
                if(forward)return false;forward=&edge.second;
            }
            for(const auto & edge:current->getLinks())if(edge.second.to()==previous->id() && edge.second.type()==Link::kNeighbor) {
                if(reverse)return false;reverse=&edge.second;
            }
            if(!forward || !reverse)return false;
            RecoveryPairResult pair;std::string why;
            if(!strictRecoveryPair(_memory->getNodeData(previous->id(),true,false,false,false),
                _memory->getNodeData(current->id(),true,false,false,false),_parameters,limits.maxInformationCondition,pair,why))return false;
            if(!strictRecoveryPoseAgreement(pair.forward,previous->getPose().inverse()*current->getPose(),limits) ||
               !strictRecoveryPoseAgreement(pair.forward,forward->transform(),limits) ||
               !strictRecoveryPoseAgreement(pair.reverse,reverse->transform(),limits)) {
                fault.resetEpoch=_resetEpoch;fault.fromId=previous->id();fault.toId=current->id();return true;
            }
        }
        previous=current;
    }
    return false;
}

bool Rtabmap::validateObservationAgainstReferences(const SensorData & observation,const Transform & predicted,
        const std::map<int,Transform> & references,int excludedId,int requiredAnchor,
        const SafeRelocalizationLimits & limits,Transform & measured,std::string & reason) const
{
    if(!_memory || !safeRigidTransform(predicted)) {reason="missing map or predicted observation";return false;}
    std::vector<std::pair<float,int> > ordered;
    for(const auto & pose:references) {
        if(pose.first<=0 || pose.first==excludedId || !_memory->getSignature(pose.first)) continue;
        const float distance=pose.second.getDistance(predicted);
        if(distance<=1.0f)ordered.emplace_back(pose.first==requiredAnchor?-1.0f:distance,pose.first);
    }
    std::sort(ordered.begin(),ordered.end());
    if(requiredAnchor>0 && (ordered.empty() || ordered.front().second!=requiredAnchor))
    {reason="required old-loop anchor is outside the compatible map neighborhood";return false;}
    int firstId=0;Transform firstMeasured;
    int rawRejected=0,predictionRejected=0,baselineRejected=0,agreementRejected=0;
    std::string lastFailure;
    for(size_t i=0;i<ordered.size() && i<12;++i) {
        const int id=ordered[i].second;const Signature * reference=_memory->getSignature(id);
        if(!std::isfinite(reference->getStamp()) || reference->getStamp()<=0.0)continue;
        if(firstId) {
            const Signature * firstSignature=_memory->getSignature(firstId);
            if(std::fabs(reference->getStamp()-firstSignature->getStamp())<.2 ||
               (references.at(id).getDistance(references.at(firstId))<std::max(.03,double(limits.minBridgeTranslation)) &&
                references.at(id).getAngle(references.at(firstId))<std::max(3.0,double(limits.minBridgeRotation))*M_PI/180.0)) {++baselineRejected;continue;}
        }
        RecoveryPairResult pair;
        if(!strictRecoveryPair(observation,_memory->getNodeData(id,true,false,false,false),
                _parameters,limits.maxInformationCondition,pair,reason)) {
            ++rawRejected;lastFailure=uFormat("reference=%d %s",id,reason.c_str());
            if(id==requiredAnchor)return false;
            continue;
        }
        const Transform candidate=references.at(id)*pair.forward.inverse();
        if(!strictRecoveryPoseAgreement(candidate,predicted,limits)) {
            ++predictionRejected;lastFailure=uFormat("reference=%d prediction distance=%.6f angle_deg=%.6f measured=%s predicted=%s",id,
                candidate.getDistance(predicted),candidate.getAngle(predicted)*180.0/M_PI,candidate.prettyPrint().c_str(),predicted.prettyPrint().c_str());
            if(id==requiredAnchor){reason="required old anchor disagrees with current map pose";return false;}
            continue;
        }
        if(!firstId){firstId=id;firstMeasured=candidate;continue;}
        if(!strictRecoveryPoseAgreement(candidate,firstMeasured,limits)){++agreementRejected;continue;}
        measured=firstMeasured;reason="two source-time/viewpoint-independent old RGB-D anchors agree";return true;
    }
    reason=uFormat("two independent old-keyframe RGB-D observations unavailable (references=%d,nearby=%d,first=%d,raw_rejected=%d,prediction_rejected=%d,baseline_rejected=%d,agreement_rejected=%d; %s)",
        int(references.size()),int(ordered.size()),firstId,rawRejected,predictionRejected,baselineRejected,agreementRejected,lastFailure.c_str());return false;
}

bool Rtabmap::validateLocalMapObservation(const SensorData & data,const Transform & odomPose,
        int lastLocalNodeId,const SafeRelocalizationLimits & limits,Transform & measuredLocalPose,std::string & reason) const
{
    const Signature * checkpoint=_memory?_memory->getSignature(lastLocalNodeId):nullptr;
    if(!_safeRelocalizationGuardEnabled || isCurrentMapFrameRegistered() || !checkpoint ||
       lastLocalNodeId<=0 || getSignatureResetEpoch(lastLocalNodeId)!=_resetEpoch ||
       !safeRigidTransform(odomPose) || !safeRigidTransform(_mapCorrection))
    {reason="same-epoch frozen local checkpoint unavailable";return false;}
    std::map<int,Transform> local;
    for(const auto & pose:_optimizedPoses) {
        const Signature * node=_memory->getSignature(pose.first);
        if(pose.first>0 && pose.first<=lastLocalNodeId && getSignatureResetEpoch(pose.first)==_resetEpoch &&
           node && node->mapId()==checkpoint->mapId())local.insert(pose);
    }
    // Same raw bidirectional geometry and independent-view requirements as main
    // recovery. This read-only result never registers the epoch or adds an edge.
    return validateObservationAgainstReferences(data,_mapCorrection*odomPose,local,0,0,limits,measuredLocalPose,reason);
}

bool Rtabmap::validateRegisteredMapObservation(const SensorData & data,const Transform & odomPose,
        int lastTrustedNodeId,const SafeRelocalizationLimits & limits,Transform & measuredMainPose,std::string & reason) const
{
    if(!_safeRelocalizationGuardEnabled || !isCurrentMapFrameRegistered() || lastTrustedNodeId<=0 ||
       !safeRigidTransform(odomPose))
    {reason="registered frozen old-map observation is unavailable";return false;}
    std::map<int,Transform> old;
    for(const auto & pose:_optimizedPoses) {
        const auto epoch=getSignatureResetEpoch(pose.first);
        if(pose.first<=lastTrustedNodeId && (epoch==0 || _safeRelocalizationCommittedEpochs.count(epoch)))old.insert(pose);
    }
    return validateObservationAgainstReferences(data,_mapCorrection*odomPose,old,0,0,limits,measuredMainPose,reason);
}

bool Rtabmap::validateProposedMapObservation(const SensorData & data,const Transform & odomPose,
        int lastTrustedNodeId,const Transform & proposedMapCorrection,const SafeRelocalizationLimits & limits,
        Transform & measuredMainPose,std::string & reason) const
{
    if(!_safeRelocalizationGuardEnabled || isCurrentMapFrameRegistered() ||
       !_safeRelocalizationTrustedSnapshotValid || lastTrustedNodeId<=0 ||
       !safeRigidTransform(odomPose) || !safeRigidTransform(proposedMapCorrection))
    {reason="uncommitted recovery proposal or frozen trusted map is unavailable";return false;}
    // The private graph is still in its own coordinates. Never use it, even
    // when one of its IDs falls below the caller's historical cutoff.
    std::map<int,Transform> old;
    for(const auto & pose:_safeRelocalizationTrustedPoses) {
        const auto epoch=getSignatureResetEpoch(pose.first);
        if(pose.first<=lastTrustedNodeId && epoch<_resetEpoch &&
           (epoch==0 || _safeRelocalizationCommittedEpochs.count(epoch)))old.insert(pose);
    }
    return validateObservationAgainstReferences(data,proposedMapCorrection*odomPose,old,0,0,limits,measuredMainPose,reason);
}

bool Rtabmap::getUnregisteredLocalSubmaps(std::vector<LocalSubmapSnapshot> & snapshots,std::string & reason) const
{
    snapshots.clear();reason.clear();
    if(!_safeRelocalizationGuardEnabled || !_memory)
    {reason="managed local submap owner is unavailable";return false;}
    if(isCurrentMapFrameRegistered()) {reason="current epoch is registered; no unregistered local snapshot";return true;}
    typedef std::pair<unsigned long long,int> Identity;
    std::map<Identity,std::set<int> > groups;
    std::map<int,Identity> membership;
    for(const auto & pose:_optimizedPoses) {
        if(pose.first<=0 || getSignatureResetEpoch(pose.first)!=_resetEpoch)continue;
        const Signature * node=_memory->getSignature(pose.first);
        if(!node || !safeRigidTransform(pose.second))
        {reason="unregistered local snapshot has a missing node or invalid local pose";return false;}
        const Identity identity(_resetEpoch,node->mapId());
        groups[identity].insert(pose.first);membership.emplace(pose.first,identity);
    }
    std::map<int,std::set<int> > adjacent;
    for(const auto & row:_constraints) {
        const Link & link=row.second;
        if(link.from()<=0 || link.to()<=0 || link.from()==link.to() || link.type()==Link::kVirtualClosure)continue;
        const auto from=membership.find(link.from()),to=membership.find(link.to());
        if(from==membership.end() && to==membership.end())continue;
        if(from==membership.end() || to==membership.end() || from->second!=to->second || !safeRigidTransform(link.transform()))
        {reason="unregistered local snapshot contains a cross-epoch/map or invalid inter-node edge";return false;}
        adjacent[link.from()].insert(link.to());adjacent[link.to()].insert(link.from());
    }
    std::vector<LocalSubmapSnapshot> prepared;
    for(const auto & group:groups) {
        std::set<int> remaining=group.second;
        while(!remaining.empty()) {
            const int first=*remaining.begin();std::set<int> component;std::queue<int> todo;
            todo.push(first);remaining.erase(first);
            while(!todo.empty()) {
                const int id=todo.front();todo.pop();component.insert(id);
                for(int next:adjacent[id])if(remaining.erase(next))todo.push(next);
            }
            LocalSubmapSnapshot snapshot;
            snapshot.mapId=group.first.second;snapshot.resetEpoch=group.first.first;
            snapshot.componentId=first;snapshot.registeredToMain=false;
            const Transform localOrigin=_optimizedPoses.at(first).inverse();
            for(int id:component)snapshot.poses.emplace(id,id==first?Transform::getIdentity():(localOrigin*_optimizedPoses.at(id)).clone());
            for(const auto & row:_constraints) {
                const Link & link=row.second;
                // A display snapshot contains measured relative geometry only.
                // Absolute gravity/pose priors and virtual planning edges are
                // not silently transformed into this independent local frame.
                if(link.from()!=link.to() && link.type()!=Link::kVirtualClosure &&
                   component.count(link.from()) && component.count(link.to()))
                    snapshot.links.emplace(link.from(),Link(link.from(),link.to(),link.type(),link.transform().clone(),link.infMatrix().clone()));
            }
            prepared.push_back(snapshot);
        }
    }
    snapshots.swap(prepared);
    reason=uFormat("%u current unregistered components in independent component-local frames",unsigned(snapshots.size()));return true;
}

bool Rtabmap::validateSameEpochVisualClosure(int currentId,int oldId,const Transform & proposed,std::string & reason) const
{
    if(!_safeRelocalizationGuardEnabled)return true;
    if(!_memory || !_optimizedPoses.count(currentId) || !_optimizedPoses.count(oldId) || currentId==oldId)
    {reason="same-epoch closure lacks both registered graph endpoints";return false;}
    SafeRelocalizationLimits limits{};
    limits.maxHoldoutTranslation=.025f;limits.maxHoldoutRotation=2;limits.minBridgeTranslation=.03f;limits.minBridgeRotation=3;
    limits.maxAnchorZ=.02f;limits.maxAnchorTilt=2;limits.maxInformationCondition=100000000.0;
    std::map<int,Transform> old;
    for(const auto & pose:_optimizedPoses)
        if(pose.first<currentId && getSignatureResetEpoch(pose.first)==getSignatureResetEpoch(currentId))old.insert(pose);
    Transform measured;
    if(!validateObservationAgainstReferences(_memory->getNodeData(currentId,true,false,false,false),
        _optimizedPoses.at(currentId),old,currentId,oldId,limits,measured,reason))return false;
    if(!strictRecoveryPoseAgreement(measured,_optimizedPoses.at(oldId)*proposed.inverse(),limits))
    {reason="native loop proposal disagrees with two raw old-keyframe observations";return false;}
    return true;
}
