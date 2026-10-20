// Pure raw-observation checks shared by cache certification and recovery
// observation. No Memory, graph, receipt, or global transform is modified.
namespace {
struct RecoveryPairResult {
    Transform forward,reverse;
};

// Forward/reverse solves share the same images and depth. Their agreement is
// internal consistency, not independent sensor evidence or proof that either
// this estimate or the separately recorded odometry is ground truth.

// Metric RANSAC labels precede reprojection BA. Reclassify only that
// original support under the FINAL transform, once, for every raw recovery
// path. Original-support RMS is also required, so trimming cannot conceal
// broadly inconsistent depth. This does not add matches or refine the pose.
bool finalRecoveryMetricSupport(const Signature & from,const Signature & to,
        const Transform & transform,const RegistrationInfo & info,int minimum,
        double metricCap,std::set<int> & words,std::string & diagnostic)
{
    words.clear();
    if(!std::isfinite(metricCap) || metricCap<=0.0 || metricCap>.025 || minimum<25)
    { diagnostic="invalid_metric_support_limits"; return false; }
    const std::set<int> proposed(info.inliersIDs.begin(),info.inliersIDs.end());
    if(int(proposed.size())<minimum)
    { diagnostic="insufficient_original_unique_support"; return false; }
    double originalSquared=0.0,squared=0.0,maximum=0.0;
    for(int word:proposed) {
        const auto a=from.getWords().equal_range(word),b=to.getWords().equal_range(word);
        if(std::distance(a.first,a.second)!=1 || std::distance(b.first,b.second)!=1)
        { diagnostic="ambiguous_word_correspondence"; return false; }
        const int ai=a.first->second,bi=b.first->second;
        if(ai<0 || bi<0 || ai>=int(from.getWords3().size()) || bi>=int(to.getWords3().size()))
        { diagnostic="invalid_3d_word_index"; return false; }
        const auto & p=from.getWords3()[ai];const auto & q=to.getWords3()[bi];
        if(!finitePoint(p) || !finitePoint(q))
        { diagnostic="nonfinite_3d_correspondence"; return false; }
        const double residual=cv::norm(cv::Point3d(p.x,p.y,p.z)-
            transformPoint(transform,cv::Point3d(q.x,q.y,q.z)));
        if(!std::isfinite(residual))
        { diagnostic="nonfinite_metric_residual"; return false; }
        originalSquared+=residual*residual;
        if(residual<=metricCap) {
            words.insert(word);squared+=residual*residual;maximum=std::max(maximum,residual);
        }
    }
    const double originalRms=std::sqrt(originalSquared/proposed.size());
    const double rms=words.empty()?std::numeric_limits<double>::infinity():std::sqrt(squared/words.size());
    diagnostic=uFormat("final_metric_count=%u/%d original_count=%u original_rms=%.6f/0.018000 rms=%.6f/0.018000 max=%.6f/%.6f",
        unsigned(words.size()),minimum,unsigned(proposed.size()),originalRms,rms,maximum,metricCap);
    return int(words.size())>=minimum && originalRms<=.018 && rms<=.018 && maximum<=metricCap;
}

// Fit translation and yaw from metric correspondences in gravity-aligned
// local/main frames. Does not reuse an inverted solve as reverse evidence.
// Caller must still validate full raw registration, both directions, measured
// gravity coverage, three independent bridges and every shadow/holdout gate.
bool gravityAlignedMetricFit(const Signature & from,const Signature & to,
        const Transform & fromPose,const Transform & toPose,
        const Transform & raw,const RegistrationInfo & info,int minimum,
        double metricCap,Transform & measured,std::string & reason)
{
    measured.setNull();
    if(!safeRigidTransform(fromPose) || !safeRigidTransform(toPose) || !safeRigidTransform(raw))
    {reason="invalid gravity-aligned fit frame";return false;}
    std::set<int> words;
    if(!finalRecoveryMetricSupport(from,to,raw,info,minimum,metricCap,words,reason))return false;
    std::vector<cv::Point3d> a,b;
    cv::Point3d ca(0,0,0),cb(0,0,0);
    for(int word:words) {
        const auto pa=from.getWords3().at(from.getWords().find(word)->second);
        const auto pb=to.getWords3().at(to.getWords().find(word)->second);
        a.push_back(transformPoint(fromPose,cv::Point3d(pa.x,pa.y,pa.z)));
        b.push_back(transformPoint(toPose,cv::Point3d(pb.x,pb.y,pb.z)));
        ca+=a.back();cb+=b.back();
    }
    ca*=1.0/a.size();cb*=1.0/b.size();
    double dot=0,cross=0;
    for(std::size_t i=0;i<a.size();++i) {
        const auto p=a[i]-ca,q=b[i]-cb;
        dot+=p.x*q.x+p.y*q.y;cross+=q.x*p.y-q.y*p.x;
    }
    if(!std::isfinite(dot) || !std::isfinite(cross) || std::hypot(dot,cross)<1e-12)
    {reason="gravity-aligned fit has no observable yaw";return false;}
    const double yaw=std::atan2(cross,dot),c=std::cos(yaw),s=std::sin(yaw);
    const cv::Point3d t=ca-cv::Point3d(c*cb.x-s*cb.y,s*cb.x+c*cb.y,cb.z);
    const Transform world(float(t.x),float(t.y),float(t.z),0.0f,0.0f,float(yaw));
    measured=fromPose.inverse()*world*toPose;
    if(!safeRigidTransform(measured) || measured.getDistance(raw)>.025 ||
       measured.getAngle(raw)>2.0*M_PI/180.0)
    {reason="gravity-aligned fit disagrees with raw six-axis registration";return false;}
    // Recheck the complete original support RMS, final metric support count,
    // and the original maximum error cap under the constrained solution.
    return finalRecoveryMetricSupport(from,to,measured,info,minimum,metricCap,words,reason);
}

bool strictRecoveryPair(SensorData fromData,SensorData toData,const ParametersMap & parameters,
        double maximumCondition,RecoveryPairResult & result,std::string & reason,
        const Transform * fromGravity=nullptr,const Transform * toGravity=nullptr,
        RecoveryPairResult * gravityResult=nullptr)
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
    // These GFTT descriptor paths were checked against raw first-pass features.
    // Prepare each image once for the pair; matching, RANSAC, refinement and
    // all posterior checks still run independently in both directions. Keep
    // other detector/matcher/optical-flow configurations on their existing path.
    int feature=Parameters::defaultVisFeatureType(),correspondences=Parameters::defaultVisCorType();
    int matcher=Parameters::defaultVisCorNNType();bool spatial=Parameters::defaultVisSSC();
    Parameters::parse(p,Parameters::kVisFeatureType(),feature);
    Parameters::parse(p,Parameters::kVisCorType(),correspondences);
    Parameters::parse(p,Parameters::kVisCorNNType(),matcher);
    Parameters::parse(p,Parameters::kVisSSC(),spatial);
    if(!spatial && correspondences==0 && (feature==6 || feature==8) && (matcher==1 || matcher==3)) {
        std::unique_ptr<Registration> preparation(Registration::create(p));
        if(!preparation || !preparation->isImageRequired())
        {reason="raw visual feature preparation unavailable";return false;}
        Signature preparedFrom(fromData),preparedTo(toData),emptyFrom,emptyTo;
        if(!preparation->computeTransformationMod(emptyFrom,preparedFrom).isNull() ||
           !preparation->computeTransformationMod(emptyTo,preparedTo).isNull())
        {reason="unpaired raw feature preparation unexpectedly produced motion";return false;}
        const auto &a=preparedFrom.sensorData();const auto &b=preparedTo.sensorData();
        fromData.setFeatures(a.keypoints(),a.keypoints3D(),a.descriptors().clone());
        toData.setFeatures(b.keypoints(),b.keypoints3D(),b.descriptors().clone());
    }
    auto validate=[&](const Signature & a,const Signature & b,const Transform & transform,const RegistrationInfo & info)->bool {
        if(!safeRigidTransform(transform) || info.inliers<minimum) return false;
        std::set<int> words;std::string diagnostic;
        if(!finalRecoveryMetricSupport(a,b,transform,info,minimum,
            std::min(.025,double(correspondenceDistance)),words,diagnostic))return false;
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
        return true; // shared final metric support, coverage and covariance all passed
    };
    // Refinement may replace feature storage. Reverse matching owns its own
    // descriptors and cannot observe mutations made by the forward solve.
    SensorData reverseFrom=fromData,reverseTo=toData;
    reverseFrom.setFeatures(fromData.keypoints(),fromData.keypoints3D(),fromData.descriptors().clone());
    reverseTo.setFeatures(toData.keypoints(),toData.keypoints3D(),toData.descriptors().clone());
    Signature a(fromData),b(toData),reverseB(reverseTo),reverseA(reverseFrom);
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
    if(fromGravity || toGravity || gravityResult) {
        if(!fromGravity || !toGravity || !gravityResult ||
           !safeRigidTransform(*fromGravity) || !safeRigidTransform(*toGravity))
        {reason="local metric/gravity fit requires both measured gravity orientations";return false;}
        // Keep the original six-axis result for the caller's unchanged raw
        // odometry/neighbor checks. The additional result independently fits
        // both RGB-D directions under measured roll/pitch, leaving yaw and
        // translation to the image/depth correspondences.
        RecoveryPairResult constrained;
        if(!gravityAlignedMetricFit(a,b,*fromGravity,*toGravity,result.forward,f,minimum,
                std::min(.025,double(correspondenceDistance)),constrained.forward,reason) ||
           !gravityAlignedMetricFit(reverseB,reverseA,*toGravity,*fromGravity,result.reverse,r,minimum,
                std::min(.025,double(correspondenceDistance)),constrained.reverse,reason) ||
           !validate(a,b,constrained.forward,f) || !validate(reverseB,reverseA,constrained.reverse,r))
        {reason="local raw RGB-D/gravity metric fit rejected: "+reason;return false;}
        auto agree=[](const Transform & x,const Transform & y) {
            return x.getDistance(y)<=.025 && x.getAngle(y)<=2.0*M_PI/180.0 &&
                std::fabs(x.z()-y.z())<=.02 && safeTiltError(x,y)<=2.0*M_PI/180.0;
        };
        if(!agree(constrained.forward,result.forward) || !agree(constrained.reverse,result.reverse) ||
           !agree(constrained.forward,constrained.reverse.inverse()))
        {reason="local metric/gravity fit contradicts raw motion or reverse solve";return false;}
        *gravityResult=constrained;
    }
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
    // A pure visual six-axis chain discards the independently measured
    // gravity that the mapping poses and bridge solve already use. Reconstruct
    // each direction with RGB-D + that same physical gravity measurement.
    // Never use the optimized poses as the gravity measurement.
    std::map<int,Transform> measuredGravity;
    for(int id:sequence) {
        int count=0;
        for(const auto & row:_constraints) {
            const Link & link=row.second;
            if(link.type()!=Link::kGravity || link.from()!=id || link.to()!=id)continue;
            ++count;
            if(!safeRigidTransform(link.transform()) ||
               !safeInformationMatrix(link.infMatrix(),limits.maxInformationCondition,reason,false) ||
               !_optimizedPoses.count(id) || !safeRigidTransform(_optimizedPoses.at(id)))
            {reason=uFormat("submap node %d has invalid or incompatible measured gravity",id);return false;}
            const double error=safeGravityDirectionError(_optimizedPoses.at(id),link.transform());
            if(!std::isfinite(error) || error>5.0*M_PI/180.0)
            {reason=uFormat("submap node %d has incompatible measured gravity",id);return false;}
            measuredGravity[id]=link.transform();
        }
        if(count!=1){reason=uFormat("submap node %d requires one measured gravity constraint",id);return false;}
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
        RecoveryPairResult pair,constrained;
        if(!strictRecoveryPair(_memory->getNodeData(before,true,false,false,false),
            _memory->getNodeData(after,true,false,false,false),_parameters,limits.maxInformationCondition,pair,reason,
            &measuredGravity.at(before),&measuredGravity.at(after),&constrained)) {
            reason=uFormat("submap continuity %d->%d lacks raw evidence; no gap backfill: %s",before,after,reason.c_str());return false;
        }
        const Transform odomRelative=a->getPose().inverse()*b->getPose();
        if(!strictRecoveryPoseAgreement(pair.forward,odomRelative,limits) ||
           !strictRecoveryPoseAgreement(pair.forward,recordedForward->transform(),limits) ||
           !strictRecoveryPoseAgreement(pair.reverse,recordedReverse->transform(),limits))
        {reason=uFormat("submap continuity %d->%d raw motion disagrees with its odometry or measured neighbor edge",before,after);return false;}
        if(!strictRecoveryPoseAgreement(constrained.forward,odomRelative,limits) ||
           !strictRecoveryPoseAgreement(constrained.forward,recordedForward->transform(),limits) ||
           !strictRecoveryPoseAgreement(constrained.reverse,recordedReverse->transform(),limits))
        {reason=uFormat("submap continuity %d->%d metric/gravity motion disagrees with odometry or neighbor",before,after);return false;}
        pairs.push_back(constrained);forward[index]=forward[index-1]*constrained.forward;
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
        {reason=uFormat("submap raw trajectory yields inconsistent main-map alignments from the three independent anchors: pairs=(%d,%d),(%d,%d) first_d=%.6fm first_a=%.6fdeg last_d=%.6fm last_a=%.6fdeg caps=(0.025m,2deg;z=0.02m,tilt=2deg)",
            evidence[i].from(),evidence[i].to(),evidence[j].from(),evidence[j].to(),
            firstAlignments[i].getDistance(firstAlignments[j]),firstAlignments[i].getAngle(firstAlignments[j])*180./M_PI,
            lastAlignments[i].getDistance(lastAlignments[j]),lastAlignments[i].getAngle(lastAlignments[j])*180./M_PI);return false;}
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
        const SafeRelocalizationLimits & limits,Transform & measured,std::string & reason,
        RecoveryObservationDiagnostics * diagnostic,const std::vector<int> * rankedIds) const
{
    measured.setNull();
    RecoveryObservationDiagnostics d;d.references=int(references.size());d.globalSearch=rankedIds!=nullptr;
    auto finish=[&](bool result){if(diagnostic)*diagnostic=d;return result;};
    if(!_memory || (!rankedIds && !safeRigidTransform(predicted))) {reason="missing map or predicted observation";return finish(false);}
    std::vector<std::pair<float,int> > ordered;
    for(const auto & pose:references) {
        if(pose.first<=0 || pose.first==excludedId || !_memory->getSignature(pose.first)) continue;
        if(rankedIds) {
            const auto rank=std::find(rankedIds->begin(),rankedIds->end(),pose.first);
            if(rank!=rankedIds->end())ordered.emplace_back(float(std::distance(rankedIds->begin(),rank)),pose.first);
            continue;
        }
        const float distance=pose.second.getDistance(predicted);
        if(distance<=1.0f)ordered.emplace_back(pose.first==requiredAnchor?-1.0f:distance,pose.first);
    }
    std::sort(ordered.begin(),ordered.end());
    d.nearby=int(ordered.size());
    if(requiredAnchor>0 && (ordered.empty() || ordered.front().second!=requiredAnchor))
    {reason="required old-loop anchor is outside the compatible map neighborhood";return finish(false);}
    // Interleave independent old viewpoints before near-duplicate neighbors.
    // Retain the remainder as fallbacks; never discard a view for lacking baseline.
    std::vector<int> selected,deferred;
    auto independent=[&](int a,int b) {
        return std::fabs(_memory->getSignature(a)->getStamp()-_memory->getSignature(b)->getStamp())>=.2 &&
            (references.at(a).getDistance(references.at(b))>=std::max(.03,double(limits.minBridgeTranslation)) ||
             references.at(a).getAngle(references.at(b))>=std::max(3.0,double(limits.minBridgeRotation))*M_PI/180.0);
    };
    for(const auto & item:ordered) {
        bool diverse=true;for(int id:selected)if(!independent(item.second,id)){diverse=false;break;}
        (diverse?selected:deferred).push_back(item.second);
    }
    selected.insert(selected.end(),deferred.begin(),deferred.end());
    std::vector<std::pair<int,Transform>> matches;
    bool pairFound=false;
    std::string lastFailure;
    const size_t budget=rankedIds?6U:12U;
    for(size_t i=0;i<selected.size() && i<budget;++i) {
        const int id=selected[i];const Signature * reference=_memory->getSignature(id);
        if(!std::isfinite(reference->getStamp()) || reference->getStamp()<=0.0)continue;
        ++d.tested;
        RecoveryPairResult pair;
        if(!strictRecoveryPair(observation,_memory->getNodeData(id,true,false,false,false),
                _parameters,limits.maxInformationCondition,pair,reason)) {
            ++d.rawRejected;lastFailure=uFormat("reference=%d %s",id,reason.c_str());
            if(id==requiredAnchor)return finish(false);
            continue;
        }
        const Transform candidate=references.at(id)*pair.forward.inverse();
        if(!predicted.isNull() && !strictRecoveryPoseAgreement(candidate,predicted,limits)) {
            ++d.predictionRejected;lastFailure=uFormat("reference=%d prediction distance=%.6f angle_deg=%.6f measured=%s predicted=%s",id,
                candidate.getDistance(predicted),candidate.getAngle(predicted)*180.0/M_PI,candidate.prettyPrint().c_str(),predicted.prettyPrint().c_str());
            if(id==requiredAnchor){reason="required old anchor disagrees with current map pose";return finish(false);}
            continue;
        }
        ++d.matched;
        for(const auto & previous:matches) {
            if(requiredAnchor>0 && previous.first!=requiredAnchor && id!=requiredAnchor)continue;
            if(!independent(id,previous.first)){++d.baselineRejected;continue;}
            if(!strictRecoveryPoseAgreement(candidate,previous.second,limits)){++d.agreementRejected;continue;}
            if(pairFound && !strictRecoveryPoseAgreement(measured,candidate,limits)) {
                ++d.agreementRejected;measured.setNull();reason="multiple independent old-anchor pairs imply different main-map positions";return finish(false);
            }
            if(!pairFound){measured=previous.second;d.firstAnchor=previous.first;d.secondAnchor=id;pairFound=true;}
            reason="two source-time/viewpoint-independent old RGB-D anchors agree";
            if(!rankedIds)return finish(true);
        }
        // A poor first match must not prevent later independent pairs succeeding.
        matches.emplace_back(id,candidate);
    }
    if(pairFound)return finish(true);
    reason=uFormat("two independent old-keyframe RGB-D observations unavailable (references=%d,nearby=%d,first=%d,raw_rejected=%d,prediction_rejected=%d,baseline_rejected=%d,agreement_rejected=%d; %s)",
        d.references,d.nearby,matches.empty()?0:matches.front().first,d.rawRejected,d.predictionRejected,d.baselineRejected,d.agreementRejected,lastFailure.c_str());return finish(false);
}

bool Rtabmap::localizeCurrentObservation(const SafeRelocalizationLimits & limits,
        Transform & measured,RecoveryObservationDiagnostics & diagnostic,std::string & reason,
        const SensorData * liveData,const ImageInputIdentity * liveIdentity) const
{
    measured.setNull();diagnostic=RecoveryObservationDiagnostics{};
    if(!_safeRelocalizationGuardEnabled || !_memory || isCurrentMapFrameRegistered() || !_safeRelocalizationTrustedSnapshotValid)
    {reason="read-only main-map localization unavailable";return false;}
    const Signature * current=_memory->getLastWorkingSignature(false);
    AcceptedImageReceipt input;
    if(!current || !getAcceptedImageReceipt(current->id(),input) || input.resetEpoch!=_resetEpoch)
    {reason="current authenticated observation unavailable";return false;}
    SensorData stored;Transform measuredGravity;
    if(liveData || liveIdentity) {
        // A current source-validated observation need not become a saved node.
        // The Wrapper checks source identity, freshness and epoch continuity;
        // Core additionally binds these exact pixels/camera to the same source.
        if(!liveData || !liveIdentity || !receiptIdentityMatches(*liveData,*liveIdentity,reason) ||
           liveIdentity->headerStampNs<input.input.headerStampNs ||
           liveIdentity->sessionTokenSha256!=input.input.sessionTokenSha256 ||
           liveIdentity->databaseSha256!=input.input.databaseSha256 ||
           liveIdentity->rgbPublisherGidHex!=input.input.rgbPublisherGidHex ||
           liveIdentity->infoPublisherGidHex!=input.input.infoPublisherGidHex) {
            reason="live localization observation identity mismatch: "+reason;return false;
        }
        const auto & imu=liveData->imu();const auto q=imu.orientation();
        const double norm=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
        if(imu.empty() || !safeRigidTransform(imu.localTransform()) || !std::isfinite(norm) || std::abs(norm-1.)>.002) {
            reason="live localization lacks valid measured IMU attitude";return false;
        }
        measuredGravity=Transform(0,0,imu.localTransform().theta()) *
            Transform(0,0,0,q[0],q[1],q[2],q[3]) * imu.localTransform().rotation().inverse();
    } else {
        stored=_memory->getNodeData(current->id(),true,false,false,false);
        const auto gravity=graph::findLink(current->getLinks(),current->id(),current->id(),false,Link::kGravity);
        if(!_memory->isOdomGravityUsed() && gravity!=current->getLinks().end())measuredGravity=gravity->second.transform();
    }
    if(!safeRigidTransform(measuredGravity)){reason="read-only localization lacks measured gravity";return false;}
    std::map<int,Transform> references;
    std::vector<std::pair<float,int>> ranking;
    for(const auto & pose:_safeRelocalizationTrustedPoses) {
        const auto epoch=getSignatureResetEpoch(pose.first);
        const Signature * old=_memory->getSignature(pose.first);AcceptedImageReceipt receipt;
        if(pose.first<=0 || !old || epoch>=_resetEpoch || (epoch!=0 && !_safeRelocalizationCommittedEpochs.count(epoch)) ||
           !getAcceptedImageReceipt(pose.first,receipt) || receipt.resetEpoch!=epoch)continue;
        references.insert(pose);
        // Shared persistent dictionary is only a retrieval hint. Every accepted
        // match is re-extracted from raw RGB-D in its own registration dictionary.
        ranking.emplace_back(-current->compareTo(*old),pose.first);
    }
    std::sort(ranking.begin(),ranking.end());
    std::vector<int> ids;for(const auto & item:ranking) {if(ids.size()==48)break;ids.push_back(item.second);}
    if(!validateObservationAgainstReferences(liveData?*liveData:stored,
            Transform(),references,0,0,limits,measured,reason,&diagnostic,&ids))return false;
    // Localization does not assume a translation/yaw prior from the lost frame.
    // The current measured gravity direction must still agree with the main map.
    // Memory persists IMU attitude as a gravity self-link, not in the RGB-D
    // payload returned by getNodeData(). An odometry-derived link is not an
    // independent measurement. Compare body-frame gravity, invariant to yaw.
    if(safeGravityDirectionError(measured,measuredGravity)>std::min(2.0,double(limits.maxAnchorTilt))*M_PI/180.0) {
        measured.setNull();reason="read-only localization disagrees with measured gravity or lacks IMU";return false;
    }
    return true;
}

bool Rtabmap::validateLocalMapObservation(const SensorData & data,const Transform & odomPose,
        int lastLocalNodeId,const SafeRelocalizationLimits & limits,Transform & measuredLocalPose,std::string & reason,
        RecoveryObservationDiagnostics * diagnostic) const
{
    if(diagnostic)*diagnostic=RecoveryObservationDiagnostics{};
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
    return validateObservationAgainstReferences(data,_mapCorrection*odomPose,local,0,0,limits,measuredLocalPose,reason,diagnostic);
}

bool Rtabmap::validateRegisteredMapObservation(const SensorData & data,const Transform & odomPose,
        int lastTrustedNodeId,const SafeRelocalizationLimits & limits,Transform & measuredMainPose,std::string & reason,
        RecoveryObservationDiagnostics * diagnostic) const
{
    if(diagnostic)*diagnostic=RecoveryObservationDiagnostics{};
    if(!_safeRelocalizationGuardEnabled || !isCurrentMapFrameRegistered() || lastTrustedNodeId<=0 ||
       !safeRigidTransform(odomPose))
    {reason="registered frozen old-map observation is unavailable";return false;}
    std::map<int,Transform> old;
    for(const auto & pose:_optimizedPoses) {
        const auto epoch=getSignatureResetEpoch(pose.first);
        if(pose.first<=lastTrustedNodeId && (epoch==0 || _safeRelocalizationCommittedEpochs.count(epoch)))old.insert(pose);
    }
    return validateObservationAgainstReferences(data,_mapCorrection*odomPose,old,0,0,limits,measuredMainPose,reason,diagnostic);
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
    std::vector<LocalSubmapSnapshot> archivedCopies;
    for(const auto & saved:_safeRelocalizationArchives) {
        LocalSubmapSnapshot copy=saved;copy.poses.clear();copy.links.clear();
        for(const auto & pose:saved.poses)copy.poses.emplace(pose.first,pose.second.clone());
        for(const auto & row:saved.links) {const auto & edge=row.second;
            copy.links.emplace(row.first,Link(edge.from(),edge.to(),edge.type(),edge.transform().clone(),edge.infMatrix().clone()));}
        archivedCopies.push_back(copy);
    }
    if(isCurrentMapFrameRegistered()) {
        snapshots.swap(archivedCopies);
        reason="current frame registered; excluded historical prefixes remain unregistered";return true;
    }
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
    std::vector<LocalSubmapSnapshot> prepared;prepared.swap(archivedCopies);
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
    if(!_memory || isRecoveryArchivedNode(currentId) || isRecoveryArchivedNode(oldId) ||
       !_optimizedPoses.count(currentId) || !_optimizedPoses.count(oldId) || currentId==oldId)
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
