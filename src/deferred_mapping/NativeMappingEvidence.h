#pragma once
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap_slam/MappingContinuityGate.h>
#include <sstream>
namespace wbrov {
// Same evidence thresholds as the managed live wrapper. No elapsed processing
// time, replay speed, Tag pose or IMU-only prediction can supply geometry here.
inline rtabmap_slam::MappingContinuityGate::Evidence mappingEvidence(
        const rtabmap::OdometryInfo &info,int width,int height) {
    using Gate=rtabmap_slam::MappingContinuityGate;
    Gate::Evidence e;e.lost=info.lost;
    if(info.type==0 && std::isfinite(info.interval) && info.interval>0 && info.interval<=.75)
        e.measuredIntervalNs=std::llround(info.interval*1e9);
    e.finiteMotion=!info.transform.isNull();
    if(e.finiteMotion)for(int i=0;i<12;++i)e.finiteMotion=e.finiteMotion && std::isfinite(info.transform.data()[i]);
    if(e.finiteMotion) {
        const double norm=info.transform.getQuaterniond().squaredNorm();
        e.finiteMotion=std::isfinite(norm) && std::abs(norm-1.)<=1e-3;
    }
    std::array<double,36> covariance{};
    if(info.reg.covariance.type()==CV_64FC1 && info.reg.covariance.rows==6 && info.reg.covariance.cols==6)
        for(int i=0;i<36;++i)covariance[i]=info.reg.covariance.at<double>(i/6,i%6);
    std::string why;
    e.covarianceFinitePD=Gate::covarianceFinitePositiveDefinite(covariance);
    e.covarianceValid=Gate::covarianceHealthy(covariance);
    bool seen=false,dimensions=width>0 && height>0 && info.localBundleModels.size()==info.localBundlePoses.size();
    for(const auto &bundle:info.localBundleModels)for(const auto &camera:bundle.second) {
        dimensions=dimensions && info.localBundlePoses.count(bundle.first)!=0;
        seen=true;dimensions=dimensions && camera.imageWidth()==width && camera.imageHeight()==height;
    }
    e.dimensionsValid=seen && dimensions;
    const double ratio=double(info.reg.inliers)/std::max(1,info.reg.matches);
    e.visualGeometry=info.reg.inliers>=15 && info.reg.matches>=info.reg.inliers && ratio>=.25;
    if(e.visualGeometry) {
        const std::set<int> inliers(info.reg.inliersIDs.begin(),info.reg.inliersIDs.end());
        std::set<int> used;std::vector<std::array<double,5>> points;
        for(const auto &word:info.words) {
            auto xyz=info.localMap.find(word.first);const auto &p=word.second.pt;
            if(std::isfinite(p.x) && std::isfinite(p.y) && inliers.count(word.first) && xyz!=info.localMap.end() && used.insert(word.first).second)
                points.push_back({p.x,p.y,xyz->second.x,xyz->second.y,xyz->second.z});
        }
        e.visualGeometry=Gate::visualSupportHealthy(points,{0.,double(width),0.,double(height)},&why);
    }
    const auto &r=info.reg;
    e.depthGeometry=info.type==0 && r.inliers==0 && r.matches==0 && r.icpCorrespondences>=80 &&
        std::isfinite(r.icpInliersRatio) && r.icpInliersRatio>=.55f && r.icpInliersRatio<=1.f &&
        std::isfinite(r.icpStructuralComplexity) && r.icpStructuralComplexity>=1e-4f && r.icpStructuralComplexity<=1.f &&
        std::isfinite(r.icpStructuralDistribution) && r.icpStructuralDistribution>=.5f && r.icpStructuralDistribution<=1.f;
    if(!e.dimensionsValid)e.visualGeometry=e.depthGeometry=false;
    e.measuredRegistration=(r.inliers>=15 && r.matches>=r.inliers && ratio>=.15) || e.depthGeometry;
    e.recoveryGeometry=(e.visualGeometry && r.inliers>=25 && ratio>=.35) ||
        (e.depthGeometry && r.icpCorrespondences>=100 && r.icpInliersRatio>=.65f && r.icpStructuralComplexity>=2e-4f && r.icpStructuralDistribution>=.67f);
    std::ostringstream s;s<<"inliers="<<r.inliers<<" matches="<<r.matches<<" dimensions="<<e.dimensionsValid
        <<" covariance="<<e.covarianceValid<<" visual="<<e.visualGeometry<<" depth="<<e.depthGeometry<<" "<<why;
    e.qualityReason=s.str();return e;
}
}
