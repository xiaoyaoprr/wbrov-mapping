#include "recovery_geometry.inc"

bool Rtabmap::collectTrustedVisualRecoveryCandidates(const AcceptedImageSourceContext & context,
        std::string & reason)
{
    if(!_safeRelocalizationGuardEnabled || !_memory || !_memory->isIncremental() ||
       isCurrentMapFrameRegistered() || !_safeRelocalizationTrustedSnapshotValid || _resetEpoch==0)
    {reason="trusted candidate search unavailable";return false;}
    const Signature * current=_memory->getLastWorkingSignature(false);
    auto sameSource=[&](const AcceptedImageReceipt & receipt) {
        const auto & input=receipt.input;
        return input.sessionTokenSha256==context.sessionTokenSha256 && input.databaseSha256==context.databaseSha256 &&
            input.rgbPublisherGidHex==context.rgbPublisherGidHex && input.infoPublisherGidHex==context.infoPublisherGidHex &&
            input.provenanceScope==context.provenanceScope;
    };
    AcceptedImageReceipt input;
    if(!current || !getAcceptedImageReceipt(current->id(),input) || input.resetEpoch!=_resetEpoch ||
       input.mapId!=current->mapId() || !_optimizedPoses.count(current->id()) || !sameSource(input))
    {reason="trusted candidate search requires current owned source receipt";return false;}
    for(const auto & candidate:_crossMapLoopEvidence)if(candidate.newId==current->id()) {
        reason="current observation already has a retained visual candidate";return true;
    }
    std::vector<std::pair<float,int>> ranking;
    std::set<int> retainedOld;
    for(const auto & candidate:_crossMapLoopEvidence)retainedOld.insert(candidate.oldId);
    for(const auto & pose:_safeRelocalizationTrustedPoses) {
        const auto epoch=getSignatureResetEpoch(pose.first);
        const Signature * old=_memory->getSignature(pose.first);AcceptedImageReceipt receipt;
        if(pose.first<=0 || !old || epoch>=_resetEpoch || (epoch!=0 && !_safeRelocalizationCommittedEpochs.count(epoch)) ||
           !getAcceptedImageReceipt(pose.first,receipt) || receipt.resetEpoch!=epoch ||
           receipt.mapId!=old->mapId() || !sameSource(receipt))continue;
        const float similarity=current->compareTo(*old);
        if(std::isfinite(similarity) && similarity>0)ranking.emplace_back(-similarity,pose.first);
    }
    std::sort(ranking.begin(),ranking.end());
    // The dictionary only ranks raw checks. Prefer unused old endpoints among
    // the best twelve so one repeated best match cannot monopolize the pool.
    // Six actual solves per retained current keyframe bound added work.
    const auto references=ranking.size();if(ranking.size()>12)ranking.resize(12);
    std::stable_partition(ranking.begin(),ranking.end(),[&](const auto & item){return !retainedOld.count(item.second);});
    unsigned tested=0;std::string detail;
    for(const auto & item:ranking) {
        if(tested==6)break;
        const Signature * old=_memory->getSignature(item.second);++tested;
        collectRawCrossMapLoopEvidence(*current,*old,_resetEpoch,getSignatureResetEpoch(old->id()),detail);
        if(std::any_of(_crossMapLoopEvidence.begin(),_crossMapLoopEvidence.end(),
                [&](const CrossMapLoopEvidence & candidate){return candidate.newId==current->id();})) {
            reason=uFormat("trusted candidate search current=%d references=%u tested=%u selected=%d retained=%u; candidate only",
                current->id(),unsigned(references),tested,old->id(),unsigned(_crossMapLoopEvidence.size()));return true;
        }
    }
    reason=uFormat("trusted candidate search current=%d references=%u tested=%u retained=%u rejected=%s",
        current->id(),unsigned(references),tested,unsigned(_crossMapLoopEvidence.size()),detail.c_str());return false;
}

// Preserve the existing Tag transaction symbol. The candidate wrapper uses
// this overload so a late odometry loss/timeout rejects after shadow solving.
bool Rtabmap::addDepthCornerLinksWithAcceptedImagesAtomically(
        const std::vector<Link> & endpointEvidence, unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits, const std::vector<AcceptedImageReceipt> & receipts,
        const std::vector<AcceptedTagCornerObservation> & corners,
        const AcceptedImageSourceContext & context, const std::function<bool()> & commitAllowed,
        std::string & reason)
{
    if(!commitAllowed || !commitAllowed())
    { reason="Tag recovery continuity ticket is no longer current"; return false; }
    if(!validateAcceptedImagesForTransaction(endpointEvidence,receipts,sourceResetEpoch,context,reason)) return false;
    std::vector<Link> measuredEvidence;
    if(!buildDepthCornerEvidence(endpointEvidence,receipts,corners,measuredEvidence,reason)) return false;
    return addLinksAtomicallyImpl(measuredEvidence,sourceResetEpoch,limits,&receipts,&context,reason,&commitAllowed);
}

bool Rtabmap::prepareDepthCornerRecoveryWithAcceptedImages(
        const std::vector<Link> & endpointEvidence, unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits, const std::vector<AcceptedImageReceipt> & receipts,
        const std::vector<AcceptedTagCornerObservation> & corners,
        const AcceptedImageSourceContext & context, const std::function<bool()> & preparationAllowed,
        Transform & proposedMapCorrection, std::string & reason)
{
    proposedMapCorrection.setNull();
    if(!preparationAllowed || !preparationAllowed())
    { reason="Tag recovery preparation ticket is no longer current"; return false; }
    if(!validateAcceptedImagesForTransaction(endpointEvidence,receipts,sourceResetEpoch,context,reason)) return false;
    std::vector<Link> measuredEvidence;
    if(!buildDepthCornerEvidence(endpointEvidence,receipts,corners,measuredEvidence,reason)) return false;
    return addLinksAtomicallyImpl(measuredEvidence,sourceResetEpoch,limits,&receipts,&context,reason,
        &preparationAllowed,&proposedMapCorrection,nullptr);
}

bool Rtabmap::commitDepthCornerRecoveryWithObservedAlignment(
        const std::vector<Link> & endpointEvidence, unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits, const std::vector<AcceptedImageReceipt> & receipts,
        const std::vector<AcceptedTagCornerObservation> & corners,
        const AcceptedImageSourceContext & context, const std::function<bool(const Transform &)> & commitAllowed,
        std::string & reason)
{
    if(!commitAllowed) { reason="Tag recovery requires a final observation certificate"; return false; }
    if(!validateAcceptedImagesForTransaction(endpointEvidence,receipts,sourceResetEpoch,context,reason)) return false;
    std::vector<Link> measuredEvidence;
    if(!buildDepthCornerEvidence(endpointEvidence,receipts,corners,measuredEvidence,reason)) return false;
    return addLinksAtomicallyImpl(measuredEvidence,sourceResetEpoch,limits,&receipts,&context,reason,
        nullptr,nullptr,&commitAllowed);
}

// Called only by the managed wrapper on its serialized Core mutation lane,
// after processWithAuthenticatedImage has retained the current input receipt.
// The existing native loop is a candidate selector, never an authorization.
bool Rtabmap::tryVisualRecoveryWithAcceptedImages(
        const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context,
        const std::function<bool()> & commitAllowed, std::string & reason)
{
    return tryVisualRecoveryImpl(limits,context,commitAllowed,nullptr,nullptr,reason);
}

bool Rtabmap::prepareVisualRecoveryWithAcceptedImages(const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context, const std::function<bool()> & preparationAllowed,
        Transform & proposedMapCorrection, std::string & reason)
{
    proposedMapCorrection.setNull();
    return tryVisualRecoveryImpl(limits,context,preparationAllowed,&proposedMapCorrection,nullptr,reason);
}

bool Rtabmap::commitVisualRecoveryWithObservedAlignment(const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context, const std::function<bool(const Transform &)> & commitAllowed,
        std::string & reason)
{
    if(!commitAllowed) { reason="visual recovery requires a final observation certificate"; return false; }
    const std::function<bool()> preparing=[] { return true; };
    return tryVisualRecoveryImpl(limits,context,preparing,nullptr,&commitAllowed,reason);
}

bool Rtabmap::tryVisualRecoveryImpl(const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context, const std::function<bool()> & commitAllowed,
        Transform * preparedMapCorrection, const std::function<bool(const Transform &)> * observedAlignmentAllowed,
        std::string & reason)
{
    reason.clear();
    if(!_safeRelocalizationGuardEnabled || !_memory || !_memory->isIncremental() ||
       _resetEpoch==0 || isCurrentMapFrameRegistered() ||
       !_safeRelocalizationTrustedSnapshotValid || _crossMapLoopEvidence.size()!=3)
    { reason=uFormat("local-submap alignment waiting for three independent trusted-map candidates (retained=%u epoch=%llu registered=%d trusted_snapshot=%d)",
        unsigned(_crossMapLoopEvidence.size()),_resetEpoch,isCurrentMapFrameRegistered()?1:0,_safeRelocalizationTrustedSnapshotValid?1:0); return false; }
    if(!commitAllowed || !commitAllowed())
    { reason="visual recovery continuity ticket is no longer current"; return false; }
    const Signature * last=_memory->getLastWorkingSignature(false);
    const auto & newest=_crossMapLoopEvidence.back();
    const Signature * bridge=_memory->getSignature(newest.newId);
    AcceptedImageReceipt latestReceipt,bridgeReceipt;
    if(!last || !bridge || newest.newId>last->id() || newest.newResetEpoch!=_resetEpoch ||
       bridge->mapId()!=last->mapId() || !getAcceptedImageReceipt(last->id(),latestReceipt) ||
       !getAcceptedImageReceipt(newest.newId,bridgeReceipt) ||
       latestReceipt.resetEpoch!=_resetEpoch || bridgeReceipt.resetEpoch!=_resetEpoch ||
       latestReceipt.input.headerStampNs<bridgeReceipt.input.headerStampNs ||
       latestReceipt.input.headerStampNs-bridgeReceipt.input.headerStampNs>6000000000LL)
    { reason=uFormat("local-submap newest bridge is missing or source-stale (current=%d newest_candidate=%d)",last?last->id():0,newest.newId); return false; }
    // This age check is only admission. The transaction certifies EVERY retained
    // neighbor from the earliest bridge through last, including newer nodes.

    std::vector<Link> endpointEvidence;
    std::vector<AcceptedImageReceipt> receipts;
    std::set<int> endpoints;
    std::set<std::int64_t> currentStamps,oldStamps;
    std::map<int,AcceptedImageReceipt> byId;
    const cv::Mat placeholderInformation=cv::Mat::eye(6,6,CV_64FC1);
    for(const CrossMapLoopEvidence & candidate:_crossMapLoopEvidence)
    {
        if(candidate.newResetEpoch!=_resetEpoch || candidate.oldResetEpoch>=_resetEpoch ||
           !endpoints.insert(candidate.newId).second || !endpoints.insert(candidate.oldId).second)
        { reason="visual recovery candidate identities are repeated or from a different epoch"; return false; }
        AcceptedImageReceipt current,old;
        if(!getAcceptedImageReceipt(candidate.newId,current) || !getAcceptedImageReceipt(candidate.oldId,old) ||
           current.resetEpoch!=candidate.newResetEpoch || old.resetEpoch!=candidate.oldResetEpoch ||
           current.mapId!=candidate.newMapId || old.mapId!=candidate.oldMapId ||
           !currentStamps.insert(current.input.headerStampNs).second || !oldStamps.insert(old.input.headerStampNs).second)
        { reason="visual recovery lacks six independent authenticated image receipts"; return false; }
        receipts.push_back(current);receipts.push_back(old);
        byId.emplace(current.signatureId,current);byId.emplace(old.signatureId,old);
        endpointEvidence.push_back(Link(candidate.newId,candidate.oldId,Link::kUserClosure,
            Transform::getIdentity(),placeholderInformation));
    }
    if(currentStamps.size()!=3 || oldStamps.size()!=3 ||
       *currentStamps.rbegin()-*currentStamps.begin()<200000000 ||
       *oldStamps.rbegin()-*oldStamps.begin()<200000000)
    { reason="visual recovery requires source-time-separated views on both sides"; return false; }
    if(!validateAcceptedImagesForTransaction(endpointEvidence,receipts,_resetEpoch,context,reason)) return false;

    // Reload the six image/depth observations through Core's owner. Never use
    // a transform, descriptor set, image, or depth array supplied by a caller.
    std::map<int,SensorData> observations;
    for(const auto & item:byId)
    {
        SensorData data=_memory->getNodeData(item.first,true,false,false,false);
        data.uncompressData();
        if(data.imageRaw().empty() || data.depthRaw().empty() ||
           data.cameraModels().size()!=1 || !receiptCameraMatches(data.cameraModels().front(),item.second.input))
        { reason="visual recovery raw RGB-D/camera data is unavailable"; return false; }
        data.setFeatures(std::vector<cv::KeyPoint>(),std::vector<cv::Point3f>(),cv::Mat());
        observations.emplace(item.first,data);
    }

    const int minimumInliers=std::max(25,_crossMapLoopMinInliers);
    const int minimumCells=std::max(4,_crossMapLoopMinGridCells);
    const double minimumSpan=std::max(0.02,double(_crossMapLoopMin3DSpan));
    const double minimumHessian=std::max(0.0001,double(_crossMapLoopMinHessianRatio));
    ParametersMap registrationParameters=_parameters;
    registrationParameters[Parameters::kRegStrategy()]="0";
    registrationParameters[Parameters::kRegForce3DoF()]="false";
    registrationParameters[Parameters::kVisEstimationType()]="0";
    int configuredInliers=Parameters::defaultVisMinInliers();
    Parameters::parse(_parameters,Parameters::kVisMinInliers(),configuredInliers);
    registrationParameters[Parameters::kVisMinInliers()]=uNumber2Str(std::max(minimumInliers,configuredInliers));
    float correspondenceDistance=Parameters::defaultVisInlierDistance();
    Parameters::parse(_parameters,Parameters::kVisInlierDistance(),correspondenceDistance);
    if(!std::isfinite(correspondenceDistance) || correspondenceDistance<=0)
    { reason="visual recovery correspondence distance is invalid"; return false; }
    registrationParameters[Parameters::kVisInlierDistance()]=uNumber2Str(std::min(.025f,correspondenceDistance));

    auto geometricEvidence=[&](const Signature & from,const Signature & to,
            const Transform & transform,const RegistrationInfo & info,std::string & diagnostic)->bool {
        if(!safeRigidTransform(transform) || info.inliers<minimumInliers)
        { diagnostic=uFormat("transform_or_inliers valid=%d inliers=%d/%d backend=%s",safeRigidTransform(transform)?1:0,info.inliers,minimumInliers,info.rejectedMsg.c_str()); return false; }
        const std::set<int> words(info.inliersIDs.begin(),info.inliersIDs.end());
        if(int(words.size())<minimumInliers)
        { diagnostic=uFormat("unique_inliers=%u/%d",unsigned(words.size()),minimumInliers); return false; }
        for(const Signature * signature:{&from,&to})
        {
            int width=0,height=0;
            if(!signatureImageSize(*signature,width,height))
            { diagnostic="missing_image_dimensions"; return false; }
            const int cells=occupiedGridCells(*signature,words,width,height);
            const std::vector<cv::Point3f> points=validInlierPoints(*signature,words);
            double span=0.0,hessian=0.0;
            const bool geometry=pointSetGeometry(points,span,hessian);
            if(cells<minimumCells || int(points.size())<minimumInliers || !geometry ||
               span<minimumSpan || hessian<minimumHessian)
            { diagnostic=uFormat("side=%s unique=%u cells=%d/%d depth=%u/%d geometry=%d span=%.6f/%.6f hessian=%.9g/%.9g",
                signature==&from?"from":"to",unsigned(words.size()),cells,minimumCells,unsigned(points.size()),minimumInliers,
                geometry?1:0,span,minimumSpan,hessian,minimumHessian); return false; }
        }
        double linear=0.0,angular=0.0;
        if(!covarianceVariances(info.covariance,linear,angular))
        { diagnostic="invalid_covariance_diagonal"; return false; }
        cv::Mat covarianceEigenvalues;
        if(!cv::checkRange(info.covariance) || cv::norm(info.covariance-info.covariance.t(),cv::NORM_INF)>1e-9 ||
           !cv::eigen(info.covariance,covarianceEigenvalues) || covarianceEigenvalues.at<double>(5)<=0.0 ||
           covarianceEigenvalues.at<double>(0)/covarianceEigenvalues.at<double>(5)>limits.maxInformationCondition)
        { diagnostic="invalid_covariance_spectrum"; return false; }
        double squared=0.0,maximum=0.0;
        int count=0;
        for(int word:words)
        {
            const auto a=from.getWords().equal_range(word),b=to.getWords().equal_range(word);
            if(std::distance(a.first,a.second)!=1 || std::distance(b.first,b.second)!=1)
            { diagnostic="ambiguous_word_correspondence"; return false; }
            const int ai=a.first->second,bi=b.first->second;
            if(ai<0 || bi<0 || ai>=int(from.getWords3().size()) || bi>=int(to.getWords3().size()))
            { diagnostic="invalid_3d_word_index"; return false; }
            const cv::Point3f & p=from.getWords3()[ai];
            const cv::Point3f & q=to.getWords3()[bi];
            if(!finitePoint(p) || !finitePoint(q))
            { diagnostic="nonfinite_3d_correspondence"; return false; }
            const double residual=cv::norm(cv::Point3d(p.x,p.y,p.z)-
                transformPoint(transform,cv::Point3d(q.x,q.y,q.z)));
            if(!std::isfinite(residual))
            { diagnostic="nonfinite_metric_residual"; return false; }
            squared+=residual*residual;maximum=std::max(maximum,residual);++count;
        }
        const double rms=count?std::sqrt(squared/double(count)):std::numeric_limits<double>::infinity();
        diagnostic=uFormat("metric_count=%d/%d rms=%.6f/0.018000 max=%.6f/0.025000",count,minimumInliers,rms,maximum);
        return count>=minimumInliers && rms<=0.018 && maximum<=0.025;
    };

    std::vector<Link> measuredEvidence;
    for(const Link & endpoint:endpointEvidence)
    {
        // Separate objects and feature extraction for both directions avoid
        // treating an inverse of the first solution as independent evidence.
        Signature forwardCurrent(observations.at(endpoint.from()));
        Signature forwardOld(observations.at(endpoint.to()));
        Signature reverseOld(observations.at(endpoint.to()));
        Signature reverseCurrent(observations.at(endpoint.from()));
        std::unique_ptr<Registration> forward(Registration::create(registrationParameters));
        std::unique_ptr<Registration> reverse(Registration::create(registrationParameters));
        if(!forward || !reverse || !forward->isImageRequired() || !reverse->isImageRequired())
        { reason="visual recovery registration backend is unavailable"; return false; }
        RegistrationInfo forwardInfo,reverseInfo;
        const Transform currentFromOld=forward->computeTransformationMod(forwardCurrent,forwardOld,Transform(),&forwardInfo);
        const Transform oldFromCurrent=reverse->computeTransformationMod(reverseOld,reverseCurrent,Transform(),&reverseInfo);
        std::string forwardDiagnostic,reverseDiagnostic;
        const bool forwardValid=geometricEvidence(forwardCurrent,forwardOld,currentFromOld,forwardInfo,forwardDiagnostic);
        const bool reverseValid=geometricEvidence(reverseOld,reverseCurrent,oldFromCurrent,reverseInfo,reverseDiagnostic);
        if(!forwardValid || !reverseValid)
        { reason=uFormat("visual recovery raw bidirectional RGB-D geometry failed pair=(%d,%d) forward=(%s) reverse=(%s)",
            endpoint.from(),endpoint.to(),forwardDiagnostic.c_str(),reverseDiagnostic.c_str()); return false; }
        const Transform reversePrediction=oldFromCurrent.inverse();
        if(currentFromOld.getDistance(reversePrediction)>std::min(0.025,double(limits.maxHoldoutTranslation)) ||
           currentFromOld.getAngle(reversePrediction)>std::min(2.0,double(limits.maxHoldoutRotation))*M_PI/180.0)
        { reason="visual recovery bidirectional RGB-D transformations disagree"; return false; }
        // Conservative covariance floor, at least the established 2cm/3deg
        // bridge floor and no stronger than either measured direction.
        double fLinear=0.0,fAngular=0.0,rLinear=0.0,rAngular=0.0;
        if(!covarianceVariances(forwardInfo.covariance,fLinear,fAngular) ||
           !covarianceVariances(reverseInfo.covariance,rLinear,rAngular)) return false;
        const double linear=std::max(0.0004,std::max(fLinear,rLinear));
        const double angular=std::max(std::pow(3.0*M_PI/180.0,2.0),std::max(fAngular,rAngular));
        cv::Mat information=cv::Mat::eye(6,6,CV_64FC1);
        for(int axis=0;axis<3;++axis) information.at<double>(axis,axis)=1.0/linear;
        for(int axis=3;axis<6;++axis) information.at<double>(axis,axis)=1.0/angular;
        measuredEvidence.push_back(Link(endpoint.from(),endpoint.to(),Link::kUserClosure,currentFromOld,information));
    }
    if(!addLinksAtomicallyImpl(measuredEvidence,_resetEpoch,limits,&receipts,&context,reason,&commitAllowed,
        preparedMapCorrection,observedAlignmentAllowed)) return false;
    if(preparedMapCorrection) return true;
    _crossMapLoopEvidence.clear();
    reason="registered by three independent raw bidirectional RGB-D bridges and fixed-old shadow/holdout transaction";
    return true;
}
