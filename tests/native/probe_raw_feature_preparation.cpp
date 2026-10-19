// Offline experiment, not a runtime switch. Inspect the first registration
// pass because Reg/RepeatOnce may replace raw features with pair-dependent
// unique words. Never reuse features extracted from that refined result.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/utilite/ULogger.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <chrono>
#include <iostream>
#include <cstring>
using namespace rtabmap;
struct InspectFirstPass:RegistrationVis {
    using RegistrationVis::RegistrationVis;
    using RegistrationVis::computeTransformationImpl;
};
bool sameMat(const cv::Mat &a,const cv::Mat &b) {
    if(a.type()!=b.type() || a.size()!=b.size())return false;
    for(int y=0;y<a.rows;++y)if(std::memcmp(a.ptr(y),b.ptr(y),a.cols*a.elemSize()))return false;
    return true;
}
bool sameFeatures(const SensorData &a,const SensorData &b) {
    if(a.keypoints().size()!=b.keypoints().size() || a.keypoints3D().size()!=b.keypoints3D().size() || !sameMat(a.descriptors(),b.descriptors()))return false;
    for(std::size_t i=0;i<a.keypoints().size();++i) {
        const auto &x=a.keypoints()[i],&y=b.keypoints()[i];
        if(x.pt!=y.pt || x.size!=y.size || x.angle!=y.angle || x.response!=y.response || x.octave!=y.octave || x.class_id!=y.class_id)return false;
    }
    for(std::size_t i=0;i<a.keypoints3D().size();++i)if(std::memcmp(&a.keypoints3D()[i],&b.keypoints3D()[i],sizeof(cv::Point3f)))return false;
    return true;
}
SensorData detachedFeatures(const SensorData &d) {
    SensorData result=d;result.setFeatures(d.keypoints(),d.keypoints3D(),d.descriptors().clone());return result;
}
struct Solve {Transform pose;RegistrationInfo info;};
Solve solve(const ParametersMap &p,const SensorData &a,const SensorData &b) {
    RegistrationVis r(p);Signature from(detachedFeatures(a)),to(detachedFeatures(b));Solve result;
    cv::setRNGSeed(81019);result.pose=r.computeTransformationMod(from,to,Transform(),&result.info);return result;
}
bool sameSolve(const Solve &a,const Solve &b) {
    return a.pose==b.pose && a.info.inliers==b.info.inliers && a.info.matches==b.info.matches &&
        a.info.inliersIDs==b.info.inliersIDs && sameMat(a.info.covariance,b.info.covariance);
}
int main(int argc,char **argv)try {
    if(argc!=3)throw std::runtime_error("closed mapping directory and mapping parameters required");
    const std::filesystem::path root=argv[1];cv::FileStorage status((root/"status.json").string(),cv::FileStorage::READ);
    if(std::string(status["state"])!="closed" || int(status["database_closed"])!=1)throw std::runtime_error("refusing active DB");
    cv::FileStorage tags((root/"tags.json").string(),cv::FileStorage::READ);std::set<int> ids;
    for(const auto &n:tags["trajectory"])ids.insert(int(n["node_id"]));
    if(ids.size()!=std::size_t(int(status["main_nodes"])))throw std::runtime_error("main-node inventory mismatch");
    ParametersMap p;std::ifstream file(argv[2]);std::string line;
    while(std::getline(file,line)){auto at=line.find('=');if(at!=std::string::npos)p[line.substr(0,at)]=line.substr(at+1);}
    p["Reg/Strategy"]="0";p["Reg/Force3DoF"]="false";p["Vis/EstimationType"]="0";p["Vis/MinInliers"]="25";p["Vis/InlierDistance"]="0.025";
    cv::setNumThreads(1);ULogger::setLevel(ULogger::kError);
    std::unique_ptr<DBDriver> db(DBDriver::create());if(!db->openConnection((root/"map.db").string(),false,true))throw std::runtime_error("read-only DB open failed");
    std::vector<int> sequence(ids.begin(),ids.end());
    int count=0,featureMismatch=0,bruteMismatch=0,bruteControlMismatch=0,approxControlMismatch=0,approxReuseMismatch=0;
    double rawMs=0,reuseMs=0,prepareMs=0;
    for(std::size_t i=1;i<sequence.size() && count<100;++i)for(int stride:{1,7}) {
        if(i<std::size_t(stride) || count>=100)continue;
        SensorData a,b;db->getNodeData(sequence[i],a,true,false,false,false);db->getNodeData(sequence[i-stride],b,true,false,false,false);
        a.uncompressData();b.uncompressData();a.setFeatures({}, {},cv::Mat());b.setFeatures({}, {},cv::Mat());
        Signature firstA(a),firstB(b);RegistrationInfo info;InspectFirstPass inspect(p);
        inspect.computeTransformationImpl(firstA,firstB,Transform(),info);
        auto start=std::chrono::steady_clock::now();RegistrationVis prepare(p);Signature emptyA,emptyB,preparedA(a),preparedB(b);
        if(!prepare.computeTransformationMod(emptyA,preparedA).isNull() || !prepare.computeTransformationMod(emptyB,preparedB).isNull())throw std::runtime_error("single-image preparation produced pose");
        const auto pa=detachedFeatures(preparedA.sensorData()),pb=detachedFeatures(preparedB.sensorData());
        prepareMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        if(!sameFeatures(firstA.sensorData(),pa) || !sameFeatures(firstB.sensorData(),pb))++featureMismatch;
        if(!sameMat(a.imageRaw(),pa.imageRaw()) || !sameMat(b.imageRaw(),pb.imageRaw()) || !sameMat(a.depthRaw(),pa.depthRaw()) || !sameMat(b.depthRaw(),pb.depthRaw()))throw std::runtime_error("raw pixels changed");
        for(int direction=0;direction<2;++direction) {
            const auto &from=direction?b:a,&to=direction?a:b,&preparedFrom=direction?pb:pa,&preparedTo=direction?pa:pb;
            start=std::chrono::steady_clock::now();const auto raw=solve(p,from,to);
            rawMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            start=std::chrono::steady_clock::now();const auto reused=solve(p,preparedFrom,preparedTo);
            reuseMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            if(!sameSolve(raw,solve(p,from,to)))++approxControlMismatch;
            if(!sameSolve(raw,reused))++approxReuseMismatch;
            // A deterministic matcher is used only as an equivalence control;
            // the runtime parameter profile is never modified by this probe.
            auto exact=p;exact["Vis/CorNNType"]="3";
            const auto expected=solve(exact,from,to);
            if(!sameSolve(expected,solve(exact,from,to)))++bruteControlMismatch;
            if(!sameSolve(expected,solve(exact,preparedFrom,preparedTo)))++bruteMismatch;
        }
        ++count;
    }
    db->closeConnection(false);
    std::cout<<"{\"pairs\":"<<count<<",\"directions\":"<<count*2<<",\"feature_mismatch\":"<<featureMismatch
        <<",\"brute_force_control_mismatch\":"<<bruteControlMismatch<<",\"brute_force_reuse_mismatch\":"<<bruteMismatch
        <<",\"approximate_control_mismatch\":"<<approxControlMismatch<<",\"approximate_reuse_mismatch\":"<<approxReuseMismatch
        <<",\"raw_bidirectional_ms\":"<<rawMs<<",\"prepared_bidirectional_ms\":"<<reuseMs<<",\"prepare_ms\":"<<prepareMs<<"}"<<std::endl;
    return featureMismatch || bruteMismatch || bruteControlMismatch?2:0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
