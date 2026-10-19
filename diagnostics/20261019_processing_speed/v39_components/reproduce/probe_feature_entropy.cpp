// OFFLINE DIAGNOSTIC EXECUTABLE ONLY. The exported symbol replays entropy in
// this process to control A/A comparisons. Never link it into Core or a worker.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/utilite/ULogger.h>
#include <dlfcn.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>
using namespace rtabmap;
namespace {
enum class Mode {None,Record,Replay};
Mode mode=Mode::None;std::vector<unsigned> entropy;std::size_t consumed=0;std::thread::id owner;
void begin(bool record) {owner=std::this_thread::get_id();consumed=0;if(record)entropy.clear();mode=record?Mode::Record:Mode::Replay;}
std::size_t finish() {
    const auto prior=mode;mode=Mode::None;
    if(prior==Mode::Replay && consumed!=entropy.size())throw std::runtime_error("entropy replay consumed a different call count");
    return entropy.size();
}
}
extern "C" unsigned diagnosticEntropy(void *) asm("_ZNSt13random_device9_M_getvalEv");
extern "C" unsigned diagnosticEntropy(void *self) {
    using Original=unsigned(*)(void*);
    static const auto original=reinterpret_cast<Original>(dlsym(RTLD_NEXT,"_ZNSt13random_device9_M_getvalEv"));
    if(!original)throw std::runtime_error("original libstdc++ entropy symbol unavailable");
    if(mode==Mode::None)return original(self);
    if(std::this_thread::get_id()!=owner)throw std::runtime_error("unexpected concurrent entropy consumer");
    if(mode==Mode::Record){const auto value=original(self);entropy.push_back(value);return value;}
    if(consumed>=entropy.size())throw std::runtime_error("entropy replay exhausted");
    return entropy[consumed++];
}
struct InspectFirstPass:RegistrationVis {using RegistrationVis::RegistrationVis;using RegistrationVis::computeTransformationImpl;};
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
SensorData detached(const SensorData &data) {SensorData copy=data;copy.setFeatures(data.keypoints(),data.keypoints3D(),data.descriptors().clone());return copy;}
struct Solve {Transform pose;RegistrationInfo info;std::size_t calls=0;double ms=0;};
Solve solve(const ParametersMap &p,const SensorData &a,const SensorData &b,bool record) {
    cv::setRNGSeed(81019);std::srand(81019);begin(record);
    const auto started=std::chrono::steady_clock::now();RegistrationVis registration(p);
    Signature from(detached(a)),to(detached(b));Solve result;
    result.pose=registration.computeTransformationMod(from,to,Transform(),&result.info);
    result.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();result.calls=finish();return result;
}
struct Difference {
    bool pose=false,covariance=false,inliers=false,matches=false,ids=false,rejection=false;
    double matrixMaximum=0,covarianceMaximum=0;
    bool exact() const{return !pose&&!covariance&&!inliers&&!matches&&!ids&&!rejection;}
};
Difference difference(const Solve &a,const Solve &b) {
    Difference d;d.pose=a.pose!=b.pose;d.covariance=!sameMat(a.info.covariance,b.info.covariance);
    d.inliers=a.info.inliers!=b.info.inliers;d.matches=a.info.matches!=b.info.matches;d.ids=a.info.inliersIDs!=b.info.inliersIDs;d.rejection=a.pose.isNull()!=b.pose.isNull();
    if(!a.pose.isNull() && !b.pose.isNull())for(int i=0;i<12;++i)d.matrixMaximum=std::max(d.matrixMaximum,std::abs(double(a.pose.data()[i])-b.pose.data()[i]));
    if(a.info.covariance.size()==b.info.covariance.size() && !a.info.covariance.empty() && cv::checkRange(a.info.covariance) && cv::checkRange(b.info.covariance))
        d.covarianceMaximum=cv::norm(a.info.covariance-b.info.covariance,cv::NORM_INF);
    return d;
}
void print(const Difference &d) {
    std::cout<<"{\"exact\":"<<(d.exact()?"true":"false")<<",\"pose\":"<<d.pose<<",\"covariance\":"<<d.covariance<<",\"inliers\":"<<d.inliers<<",\"matches\":"<<d.matches
             <<",\"ids\":"<<d.ids<<",\"rejection\":"<<d.rejection<<",\"matrix_max_abs\":"<<d.matrixMaximum<<",\"covariance_max_abs\":"<<d.covarianceMaximum<<'}';
}
int main(int argc,char **argv)try {
    if(argc!=4 || std::string(argv[3])!="--diagnostic-entropy-replay")throw std::runtime_error("explicit offline entropy diagnostic flag required");
    const std::filesystem::path root=argv[1];cv::FileStorage status((root/"status.json").string(),cv::FileStorage::READ);
    if(std::string(status["state"])!="closed" || int(status["database_closed"])!=1)throw std::runtime_error("refusing active DB");
    cv::FileStorage tags((root/"tags.json").string(),cv::FileStorage::READ);std::set<int> ids;
    for(const auto &n:tags["trajectory"])ids.insert(int(n["node_id"]));
    if(ids.size()!=std::size_t(int(status["main_nodes"])))throw std::runtime_error("main-node inventory mismatch");
    ParametersMap p;std::ifstream file(argv[2]);std::string line;
    while(std::getline(file,line)){auto at=line.find('=');if(at!=std::string::npos)p[line.substr(0,at)]=line.substr(at+1);}
    p["Reg/Strategy"]="0";p["Reg/Force3DoF"]="false";p["Vis/EstimationType"]="0";p["Vis/MinInliers"]="25";p["Vis/InlierDistance"]="0.025";
    cv::setNumThreads(1);ULogger::setLevel(ULogger::kError);std::cout.precision(12);
    std::unique_ptr<DBDriver> db(DBDriver::create());if(!db->openConnection((root/"map.db").string(),false,true))throw std::runtime_error("read-only DB open failed");
    std::vector<int> sequence(ids.begin(),ids.end());int pairs=0,features=0,controls=0,reuses=0;std::size_t totalEntropy=0;
    for(std::size_t i=1;i<sequence.size() && pairs<24;++i)for(int stride:{1,7}) {
        if(i<std::size_t(stride) || pairs>=24)continue;
        SensorData a,b;db->getNodeData(sequence[i],a,true,false,false,false);db->getNodeData(sequence[i-stride],b,true,false,false,false);
        a.uncompressData();b.uncompressData();a.setFeatures({}, {},cv::Mat());b.setFeatures({}, {},cv::Mat());
        const auto aRgb=a.imageRaw().clone(),aDepth=a.depthRaw().clone(),bRgb=b.imageRaw().clone(),bDepth=b.depthRaw().clone();
        Signature firstA(a),firstB(b);RegistrationInfo firstInfo;InspectFirstPass inspect(p);inspect.computeTransformationImpl(firstA,firstB,Transform(),firstInfo);
        const auto began=std::chrono::steady_clock::now();RegistrationVis prepare(p);Signature emptyA,emptyB,preparedA(a),preparedB(b);
        if(!prepare.computeTransformationMod(emptyA,preparedA).isNull() || !prepare.computeTransformationMod(emptyB,preparedB).isNull())throw std::runtime_error("single-image preparation produced pose");
        const auto pa=detached(preparedA.sensorData()),pb=detached(preparedB.sensorData());
        const auto prepareMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
        const bool featureEqual=sameFeatures(firstA.sensorData(),pa)&&sameFeatures(firstB.sensorData(),pb);if(!featureEqual)++features;
        for(int matcher=0;matcher<2;++matcher)for(int direction=0;direction<2;++direction) {
            auto parameters=p;if(matcher)parameters["Vis/CorNNType"]="3";
            const auto &from=direction?b:a,&to=direction?a:b,&preparedFrom=direction?pb:pa,&preparedTo=direction?pa:pb;
            const auto raw=solve(parameters,from,to,true);const auto tape=entropy;totalEntropy+=tape.size();
            const auto control=solve(parameters,from,to,false);const auto reused=solve(parameters,preparedFrom,preparedTo,false);
            const auto controlDiff=difference(raw,control),reuseDiff=difference(raw,reused);if(!controlDiff.exact())++controls;if(!reuseDiff.exact())++reuses;
            std::cout<<"{\"pair\":"<<pairs<<",\"from\":"<<sequence[i]<<",\"to\":"<<sequence[i-stride]<<",\"direction\":"<<direction<<",\"exact_matcher_control\":"<<matcher
                     <<",\"feature_equal\":"<<(featureEqual?"true":"false")<<",\"prepare_ms\":"<<prepareMs<<",\"raw_ms\":"<<raw.ms<<",\"reused_ms\":"<<reused.ms<<",\"entropy\":[";
            for(std::size_t n=0;n<tape.size();++n){if(n)std::cout<<',';std::cout<<tape[n];}
            std::cout<<"],\"control\":";print(controlDiff);std::cout<<",\"reuse\":";print(reuseDiff);std::cout<<'}'<<std::endl;
        }
        if(!sameMat(aRgb,a.imageRaw())||!sameMat(aDepth,a.depthRaw())||!sameMat(bRgb,b.imageRaw())||!sameMat(bDepth,b.depthRaw()))throw std::runtime_error("registration changed raw pixels");
        ++pairs;
    }
    db->closeConnection(false);
    if(!totalEntropy)throw std::runtime_error("Core did not use the diagnostic entropy symbol");
    std::cout<<"{\"summary\":true,\"pairs\":"<<pairs<<",\"feature_mismatch\":"<<features<<",\"control_mismatch\":"<<controls<<",\"reuse_mismatch\":"<<reuses<<",\"entropy_calls\":"<<totalEntropy<<"}"<<std::endl;
    return features||controls||reuses?2:0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
