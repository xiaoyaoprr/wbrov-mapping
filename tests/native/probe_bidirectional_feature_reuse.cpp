// Offline experiment only: compare fresh extraction in the reverse solve with
// reusing raw per-image features extracted in the forward solve. No word IDs,
// matches, transformations, or acceptance results cross the two solves.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <chrono>
#include <iostream>
#include <cstring>
using namespace rtabmap;
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
    std::vector<int> sequence(ids.begin(),ids.end());double rawMs=0,reuseMs=0;int count=0,featureMismatch=0,registrationMismatch=0,accepted=0;
    for(std::size_t i=1;i<sequence.size() && count<100;++i)for(int stride:{1,7}) {
        if(i<std::size_t(stride) || count>=100)continue;
        SensorData a,b;db->getNodeData(sequence[i],a,true,false,false,false);db->getNodeData(sequence[i-stride],b,true,false,false,false);
        a.uncompressData();b.uncompressData();a.setFeatures({}, {},cv::Mat());b.setFeatures({}, {},cv::Mat());
        Signature forwardA(a),forwardB(b),rawB(b),rawA(a);RegistrationInfo fi,ri,ci;
        std::unique_ptr<Registration> forward(Registration::create(p)),raw(Registration::create(p)),reuse(Registration::create(p));
        cv::setRNGSeed(81019+count);forward->computeTransformationMod(forwardA,forwardB,Transform(),&fi);
        const auto rng=cv::theRNG().state;
        auto start=std::chrono::steady_clock::now();const auto before=raw->computeTransformationMod(rawB,rawA,Transform(),&ri);
        rawMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        Signature preparedB(forwardB.sensorData()),preparedA(forwardA.sensorData());
        cv::theRNG().state=rng;start=std::chrono::steady_clock::now();const auto after=reuse->computeTransformationMod(preparedB,preparedA,Transform(),&ci);
        reuseMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        const bool features=sameFeatures(rawB.sensorData(),preparedB.sensorData()) && sameFeatures(rawA.sensorData(),preparedA.sensorData());
        const bool match=before==after && ri.inliers==ci.inliers && ri.matches==ci.matches && ri.inliersIDs==ci.inliersIDs &&
            rawB.getWords()==preparedB.getWords() && rawA.getWords()==preparedA.getWords() && sameMat(ri.covariance,ci.covariance);
        if(!features)++featureMismatch;if(!match)++registrationMismatch;if(!before.isNull())++accepted;++count;
    }
    db->closeConnection(false);
    std::cout<<"{\"pairs\":"<<count<<",\"raw_reverse_ms\":"<<rawMs<<",\"reuse_reverse_ms\":"<<reuseMs
        <<",\"feature_mismatch\":"<<featureMismatch<<",\"registration_mismatch\":"<<registrationMismatch<<",\"raw_nonnull\":"<<accepted<<"}"<<std::endl;
    return featureMismatch || registrationMismatch?2:0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
