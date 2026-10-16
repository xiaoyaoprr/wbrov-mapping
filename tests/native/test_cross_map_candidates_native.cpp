#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/core.hpp>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>

using namespace rtabmap;
static void require(bool ok,const std::string & message) { if(!ok)throw std::runtime_error(message); }
namespace rtabmap {
class ImageReceiptNativeTestAccess {
public:
    static std::size_t count(const Rtabmap & core) { return core._crossMapLoopEvidence.size(); }
    static void clear(Rtabmap & core) { core._crossMapLoopEvidence.clear(); }
    static void guarded(Rtabmap & core,bool enabled) { core._safeRelocalizationGuardEnabled=enabled; }
    static void floor(Rtabmap & core,double linear,double angular) {
        core._parameters[Parameters::kRtabmapCrossMapLoopMinLinearSigma()]=std::to_string(linear);
        core._parameters[Parameters::kRtabmapCrossMapLoopMinAngularSigma()]=std::to_string(angular);
    }
    static bool add(Rtabmap & core,int index,double error,double angle,cv::Mat covariance,
                    int depthCount,int idCount,bool spread,std::string & reason) {
        CameraModel camera(200,200,159.5,119.5,Transform::getIdentity(),0,cv::Size(320,240));
        SensorData data(cv::Mat::zeros(240,320,CV_8UC1),cv::Mat(240,320,CV_32FC1,cv::Scalar(1)),camera);
        const Transform pose(float(index)*.10f,0,0,0,0,0);
        Signature current(100+index,1,0,1000+index,"",pose,Transform(),data);
        Signature old(10+index,0,0,900+index,"",pose,Transform(),data);
        std::multimap<int,int> words;std::vector<cv::KeyPoint> pixels;std::vector<cv::Point3f> points;
        RegistrationInfo info;info.inliers=idCount;info.covariance=covariance;
        for(int i=0;i<25;++i) {
            words.emplace(i+1,i);pixels.emplace_back(spread?float(20+(i%5)*65):20.f,spread?float(20+(i/5)*45):20.f,1.f);
            points.emplace_back(float(i%5-2)*.06f,float(i/5-2)*.06f,i<depthCount?1.f:std::numeric_limits<float>::quiet_NaN());
            if(i<idCount)info.inliersIDs.push_back(i+1);
        }
        current.setWords(words,pixels,points,cv::Mat());old.setWords(words,pixels,points,cv::Mat());
        // Produce a controlled fixed-anchor discrepancy despite the changing
        // source poses. This exercises the real candidate collector, not a copy.
        Transform anchor(float(error),0,0,0,0,float(angle));
        Transform measured=pose.inverse()*anchor*pose;
        return core.acceptCrossMapLoopEvidence(current,old,1,0,measured,info,reason);
    }
};
}
int main() try {
    ULogger::setLevel(ULogger::kWarning);
    using A=ImageReceiptNativeTestAccess;
    Rtabmap core;A::guarded(core,true);std::string reason;
    cv::Mat covariance=cv::Mat::eye(6,6,CV_64FC1)*1e-8;
    for(int k=3;k<6;++k)covariance.at<double>(k,k)=3e-8;
    auto add=[&](int i,double d,double degrees,int depth=25,int ids=25,bool spread=true) {
        return A::add(core,i,d,degrees*3.141592653589793/180.,covariance,depth,ids,spread,reason);
    };
    require(Parameters::getDefaultParameters().count("Rtabmap/CrossMapLoopMinLinearSigma")==1,"new parameters absent from runtime registry");
    require(!add(1,0,0) && A::count(core)==1,"exactly 25 unique inliers should retain one guarded candidate: "+reason);
    require(!add(2,.001998,.285697) && A::count(core)==2,"epsilon covariance still erases small measured discrepancy: "+reason);
    require(!add(3,.003,.35) && A::count(core)==3,"three candidates must be retained without authorizing native addLink: "+reason);
    A::clear(core);A::floor(core,0,0);
    add(1,0,0);add(2,.001998,.285697);
    require(A::count(core)==1 && reason.find("inconsistent fixed-anchor")!=std::string::npos,"zero floor must preserve raw consensus behavior");
    A::floor(core,.02,3);A::clear(core);add(1,0,0);add(2,.101,0);
    require(A::count(core)==1 && reason.find("inconsistent fixed-anchor")!=std::string::npos,"absolute translation limit bypassed");
    A::clear(core);add(1,0,0);add(2,0,10.1);
    require(A::count(core)==1 && reason.find("inconsistent fixed-anchor")!=std::string::npos,"absolute rotation limit bypassed");
    A::clear(core);add(1,0,0,5);
    require(A::count(core)==0 && reason.find("valid 3D inliers")!=std::string::npos,"depth threshold was relaxed");
    add(1,0,0,25,24);require(A::count(core)==0,"24 inliers unexpectedly retained");
    add(1,0,0,25,25,false);require(A::count(core)==0 && reason.find("grid coverage")!=std::string::npos,"coverage threshold was relaxed");
    cv::Mat bad=covariance.clone();bad.at<double>(0,1)=bad.at<double>(1,0)=1;
    require(!A::add(core,1,0,0,bad,25,25,true,reason) && A::count(core)==0,"floor masked indefinite covariance");
    bad=covariance.clone();bad.at<double>(0,1)=std::numeric_limits<double>::quiet_NaN();
    require(!A::add(core,1,0,0,bad,25,25,true,reason) && A::count(core)==0,"floor masked nonfinite covariance");
    A::floor(core,.021,3);add(1,0,0);require(A::count(core)==0,"out-of-range floor retained candidate");
    A::floor(core,.02,3);A::guarded(core,false);add(1,0,0);add(2,.001998,.285697);
    require(A::count(core)==1 && reason.find("inconsistent fixed-anchor")!=std::string::npos,"ordinary native closure inherited guarded-only floor");
    std::cout<<"PASS real Core collector: 25 unique inliers; small epsilon-covariance candidates retained without native authorization; strict absolute/depth/coverage/invalid-covariance/parameter/legacy negatives\n";
    return 0;
} catch(const std::exception & e) { std::cerr<<"FAIL "<<e.what()<<'\n';return 1; }
