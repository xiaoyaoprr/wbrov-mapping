#include "recovery_geometry.inc"

namespace {
bool screenVisualRecoveryReference(SensorData currentData,SensorData oldData,
        const Transform & currentPose,const Transform & oldPose,
        const ParametersMap & parameters,std::string & reason)
{
    currentData.uncompressData();oldData.uncompressData();
    if(!safeRigidTransform(currentPose) || !safeRigidTransform(oldPose) ||
       currentData.imageRaw().empty() || currentData.depthRaw().empty() ||
       oldData.imageRaw().empty() || oldData.depthRaw().empty() ||
       currentData.cameraModels().size()!=1 || oldData.cameraModels().size()!=1)
    {reason="candidate screen lacks raw observations or poses";return false;}
    int minimum=25,configured=Parameters::defaultVisMinInliers();
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinInliers(),minimum);
    Parameters::parse(parameters,Parameters::kVisMinInliers(),configured);
    minimum=std::max(25,std::max(minimum,configured));
    float cap=Parameters::defaultVisInlierDistance();
    Parameters::parse(parameters,Parameters::kVisInlierDistance(),cap);
    if(!std::isfinite(cap) || cap<=0){reason="candidate screen invalid metric cap";return false;}
    const double metricCap=std::min(.025,double(cap));
    ParametersMap p=parameters;
    p[Parameters::kRegStrategy()]="0";p[Parameters::kRegForce3DoF()]="false";
    p[Parameters::kVisEstimationType()]="0";p[Parameters::kVisMinInliers()]=uNumber2Str(minimum);
    p[Parameters::kVisInlierDistance()]=uNumber2Str(metricCap);
    currentData.setFeatures({}, {},cv::Mat());oldData.setFeatures({}, {},cv::Mat());
    Transform measured[2];
    for(int direction=0;direction<2;++direction) {
        Signature from(direction?oldData:currentData),to(direction?currentData:oldData);
        std::unique_ptr<Registration> registration(Registration::create(p));
        if(!registration || !registration->isImageRequired())
        {reason="candidate screen raw registration unavailable";return false;}
        RegistrationInfo info;
        const Transform raw=registration->computeTransformationMod(from,to,Transform(),&info);
        if(!safeRigidTransform(raw) || info.inliers<minimum)
        {reason="candidate screen raw registration rejected: "+info.rejectedMsg;return false;}
        if(!gravityAlignedMetricFit(from,to,direction?oldPose:currentPose,
            direction?currentPose:oldPose,raw,info,minimum,metricCap,measured[direction],reason))return false;
    }
    if(measured[0].getDistance(measured[1].inverse())>.025 ||
       measured[0].getAngle(measured[1].inverse())>2.0*M_PI/180.0)
    {reason="candidate screen bidirectional fits disagree";return false;}
    reason="raw metric/gravity candidate screen passed; not authorization";
    return true;
}
}

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
    // Coarse native loop candidates may have entered the pool before raw
    // bidirectional/gravity checks. Do not let one rejected old endpoint pin
    // a valid current view forever. Reconsider that SAME authenticated current
    // observation against bounded distinct references; never retry one pair
    // until it passes and never grant registration here.
    std::map<int,std::set<int>> failedPairs;
    std::vector<int> searchSources;
    auto ownedCurrent=[&](int id)->bool {
        const Signature * signature=_memory->getSignature(id);AcceptedImageReceipt receipt;
        return signature && _optimizedPoses.count(id) && getAcceptedImageReceipt(id,receipt) &&
            receipt.resetEpoch==_resetEpoch && receipt.mapId==current->mapId() &&
            receipt.mapId==signature->mapId() && sameSource(receipt);
    };
    auto screen=[&](int sourceId,int oldId,std::string & why)->bool {
        const Signature * old=_memory->getSignature(oldId);AcceptedImageReceipt receipt;
        const auto epoch=getSignatureResetEpoch(oldId);
        if(!ownedCurrent(sourceId) || !old || !_safeRelocalizationTrustedPoses.count(oldId) ||
           epoch>=_resetEpoch || (epoch!=0 && !_safeRelocalizationCommittedEpochs.count(epoch)) ||
           !getAcceptedImageReceipt(oldId,receipt) || receipt.resetEpoch!=epoch ||
           receipt.mapId!=old->mapId() || !sameSource(receipt))
        {why="candidate screen endpoint ownership mismatch";return false;}
        return screenVisualRecoveryReference(_memory->getNodeData(sourceId,true,false,false,false),
            _memory->getNodeData(oldId,true,false,false,false),_optimizedPoses.at(sourceId),
            _safeRelocalizationTrustedPoses.at(oldId),_parameters,why);
    };
    // At most three retained pairs and six distinct alternatives per source.
    for(auto it=_crossMapLoopEvidence.begin();it!=_crossMapLoopEvidence.end();) {
        std::string why;
        if(screen(it->newId,it->oldId,why)){++it;continue;}
        UWARN("visual candidate endpoint rejected current=%d old=%d reason=%s; searching distinct old references, candidate only",
            it->newId,it->oldId,why.c_str());
        failedPairs[it->newId].insert(it->oldId);
        if(ownedCurrent(it->newId))searchSources.push_back(it->newId);
        it=_crossMapLoopEvidence.erase(it);
    }
    if(std::none_of(_crossMapLoopEvidence.begin(),_crossMapLoopEvidence.end(),
        [&](const CrossMapLoopEvidence & candidate){return candidate.newId==current->id();}))
        searchSources.push_back(current->id());
    std::sort(searchSources.begin(),searchSources.end());
    searchSources.erase(std::unique(searchSources.begin(),searchSources.end()),searchSources.end());
    unsigned totalTested=0;std::string detail;
    for(int sourceId:searchSources) {
        const Signature * source=_memory->getSignature(sourceId);
        if(!source || !ownedCurrent(sourceId))continue;
        std::set<int> retainedOld;
        for(const auto & candidate:_crossMapLoopEvidence)retainedOld.insert(candidate.oldId);
        std::vector<std::pair<float,int>> ranking;
        for(const auto & pose:_safeRelocalizationTrustedPoses) {
            const auto epoch=getSignatureResetEpoch(pose.first);
            const Signature * old=_memory->getSignature(pose.first);AcceptedImageReceipt receipt;
            if(pose.first<=0 || !old || failedPairs[sourceId].count(pose.first) ||
               epoch>=_resetEpoch || (epoch!=0 && !_safeRelocalizationCommittedEpochs.count(epoch)) ||
               !getAcceptedImageReceipt(pose.first,receipt) || receipt.resetEpoch!=epoch ||
               receipt.mapId!=old->mapId() || !sameSource(receipt))continue;
            const float similarity=source->compareTo(*old);
            if(std::isfinite(similarity) && similarity>0)ranking.emplace_back(-similarity,pose.first);
        }
        std::sort(ranking.begin(),ranking.end());
        const auto references=ranking.size();if(ranking.size()>12)ranking.resize(12);
        std::stable_partition(ranking.begin(),ranking.end(),[&](const auto & item){return !retainedOld.count(item.second);});
        unsigned tested=0;
        for(const auto & item:ranking) {
            if(tested==6)break;
            ++tested;++totalTested;
            if(!screen(sourceId,item.second,detail)) {
                UWARN("visual candidate alternative rejected current=%d old=%d reason=%s; candidate only",
                    sourceId,item.second,detail.c_str());continue;
            }
            const Signature * old=_memory->getSignature(item.second);
            collectRawCrossMapLoopEvidence(*source,*old,_resetEpoch,getSignatureResetEpoch(old->id()),detail);
            // Replacing a historical endpoint must not make it appear newer
            // than the newest bridge used by the source-age admission check.
            std::sort(_crossMapLoopEvidence.begin(),_crossMapLoopEvidence.end(),
                [](const CrossMapLoopEvidence & a,const CrossMapLoopEvidence & b){return a.newId<b.newId;});
            if(std::any_of(_crossMapLoopEvidence.begin(),_crossMapLoopEvidence.end(),
                    [&](const CrossMapLoopEvidence & candidate){return candidate.newId==sourceId;})) {
                UWARN("trusted candidate search current=%d references=%u tested=%u selected=%d retained=%u; candidate only",
                    sourceId,unsigned(references),tested,old->id(),unsigned(_crossMapLoopEvidence.size()));break;
            }
        }
    }
    reason=uFormat("trusted candidate search latest=%d replaced_sources=%u tested=%u retained=%u detail=%s; candidate only",
        current->id(),unsigned(failedPairs.size()),totalTested,unsigned(_crossMapLoopEvidence.size()),detail.c_str());
    return !_crossMapLoopEvidence.empty();
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

    int configuredInliers=Parameters::defaultVisMinInliers();
    Parameters::parse(_parameters,Parameters::kVisMinInliers(),configuredInliers);
    const int minimumInliers=std::max(25,std::max(_crossMapLoopMinInliers,configuredInliers));
    const int minimumCells=std::max(4,_crossMapLoopMinGridCells);
    const double minimumSpan=std::max(0.02,double(_crossMapLoopMin3DSpan));
    const double minimumHessian=std::max(0.0001,double(_crossMapLoopMinHessianRatio));
    ParametersMap registrationParameters=_parameters;
    registrationParameters[Parameters::kRegStrategy()]="0";
    registrationParameters[Parameters::kRegForce3DoF()]="false";
    registrationParameters[Parameters::kVisEstimationType()]="0";
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
        std::set<int> words;
        if(!finalRecoveryMetricSupport(from,to,transform,info,minimumInliers,
            std::min(.025,double(correspondenceDistance)),words,diagnostic))return false;
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
        return true; // same final-transform metric support as local-chain/observation checks
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
        // Both map frames are required to have measured gravity below. Fit
        // each direction separately from its own raw metric support, constraining
        // the world-frame correction to yaw+translation as on the Tag path.
        // A visual tilt conflicting with gravity is not allowed to rotate the
        // whole current map. Never bypass the original six-axis raw checks.
        const Transform & currentPose=_optimizedPoses.at(endpoint.from());
        const Transform & oldPose=_safeRelocalizationTrustedPoses.at(endpoint.to());
        auto measuredGravity=[&](int id,const Transform & pose,const std::multimap<int,Link> & links)->bool {
            int count=0;
            for(const auto & row:links) {
                const Link & link=row.second;
                if(link.type()!=Link::kGravity || link.from()!=id || link.to()!=id)continue;
                ++count;
                if(!safeRigidTransform(link.transform()) || !safeInformationMatrix(link.infMatrix(),
                    limits.maxInformationCondition,reason,false))return false;
                const double error=safeGravityDirectionError(pose,link.transform());
                if(!std::isfinite(error) || error>5.0*M_PI/180.0)return false;
            }
            return count==1;
        };
        if(!measuredGravity(endpoint.from(),currentPose,_constraints) ||
           !measuredGravity(endpoint.to(),oldPose,_safeRelocalizationTrustedConstraints))
        {reason="visual recovery gravity-aligned fit requires both measured gravity frames";return false;}
        Transform fittedForward,fittedReverse;
        if(!gravityAlignedMetricFit(forwardCurrent,forwardOld,currentPose,oldPose,currentFromOld,
                forwardInfo,minimumInliers,std::min(.025,double(correspondenceDistance)),fittedForward,forwardDiagnostic) ||
           !gravityAlignedMetricFit(reverseOld,reverseCurrent,oldPose,currentPose,oldFromCurrent,
                reverseInfo,minimumInliers,std::min(.025,double(correspondenceDistance)),fittedReverse,reverseDiagnostic) ||
           !geometricEvidence(forwardCurrent,forwardOld,fittedForward,forwardInfo,forwardDiagnostic) ||
           !geometricEvidence(reverseOld,reverseCurrent,fittedReverse,reverseInfo,reverseDiagnostic) ||
           !strictRecoveryPoseAgreement(fittedForward,currentFromOld,limits) ||
           !strictRecoveryPoseAgreement(fittedReverse,oldFromCurrent,limits) ||
           !strictRecoveryPoseAgreement(fittedForward,fittedReverse.inverse(),limits))
        {reason=uFormat("visual recovery gravity-aligned metric fit failed pair=(%d,%d) forward=(%s) reverse=(%s)",
            endpoint.from(),endpoint.to(),forwardDiagnostic.c_str(),reverseDiagnostic.c_str());return false;}
        UWARN("visual recovery gravity-aligned raw fit pair=(%d,%d) forward_change_m=%.6f deg=%.6f reverse_change_m=%.6f deg=%.6f; original and constrained raw gates passed",
            endpoint.from(),endpoint.to(),currentFromOld.getDistance(fittedForward),currentFromOld.getAngle(fittedForward)*180.0/M_PI,
            oldFromCurrent.getDistance(fittedReverse),oldFromCurrent.getAngle(fittedReverse)*180.0/M_PI);
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
        measuredEvidence.push_back(Link(endpoint.from(),endpoint.to(),Link::kUserClosure,fittedForward,information));
    }
    if(!addLinksAtomicallyImpl(measuredEvidence,_resetEpoch,limits,&receipts,&context,reason,&commitAllowed,
        preparedMapCorrection,observedAlignmentAllowed)) return false;
    if(preparedMapCorrection) return true;
    _crossMapLoopEvidence.clear();
    reason="registered by three independent raw bidirectional RGB-D bridges and fixed-old shadow/holdout transaction";
    return true;
}
