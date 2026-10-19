// Diagnostic restart test against a CLOSED, privately copied saved map.
// Input is held-out RGB-D from a sealed recording; no ROS/control publication.
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/utilite/ULogger.h>
#include "rgbd_spool.hpp"
#include "NativeMappingEvidence.h"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <set>
using namespace rtabmap;
static ParametersMap parameters(const char *path) {
    std::ifstream f(path);if(!f)throw std::runtime_error("parameters missing");
    ParametersMap result;std::string s;
    while(std::getline(f,s)){auto at=s.find('=');if(at!=std::string::npos)result[s.substr(0,at)]=s.substr(at+1);}
    return result;
}
static SensorData sensor(const wbrov_spool::Frame &f,bool low) {
    const auto &r=low?f.low:f.map;if(!r.width)return SensorData();
    const Transform bodyFromCamera(0,0,0,-.5,.5,-.5,.5);
    CameraModel model(r.K[0],r.K[1],r.K[2],r.K[3],bodyFromCamera,0,cv::Size(r.width,r.height));
    cv::Mat raw(r.height,r.width,CV_8UC3,const_cast<std::uint8_t*>(r.bgr.data())),image;
    if(low)cv::cvtColor(raw,image,cv::COLOR_BGR2GRAY);else image=raw.clone();
    cv::Mat depth(r.height,r.width,CV_32FC1,const_cast<std::uint8_t*>(r.depth.data()));
    SensorData data(image,depth.clone(),model,0,f.sourceNs/1e9);const auto &a=f.imu;
    data.setIMU(IMU(cv::Vec4d(a[0],a[1],a[2],a[3]),cv::Mat(),cv::Vec3d(a[4],a[5],a[6]),cv::Mat(),cv::Vec3d(a[7],a[8],a[9]),cv::Mat(),bodyFromCamera));
    return data;
}
int main(int argc,char **argv) try {
    if(argc!=6)throw std::runtime_error("probe config odom_parameters map_parameters output.csv working_directory");
    cv::setNumThreads(1);cv::setRNGSeed(81018);ULogger::setLevel(ULogger::kWarning);
    cv::FileStorage config(argv[1],cv::FileStorage::READ);if(!config.isOpened())throw std::runtime_error("probe config missing");
    std::set<int> savedIds;std::set<long long> savedStamps;
    for(const auto &id:config["saved_ids"])savedIds.insert(int(id));
    for(const auto &stamp:config["saved_source_ns"])savedStamps.insert(std::stoll(std::string(stamp)));
    if(savedIds.empty())throw std::runtime_error("saved main component missing");
    auto op=parameters(argv[2]),mp=parameters(argv[3]);
    if(op["Vis/MinInliers"]!="15" || mp["Vis/MinInliers"]!="25")throw std::runtime_error("unexpected geometry thresholds");
    op["Odom/FillInfoData"]="true";
    mp["Mem/IncrementalMemory"]="false";mp["Mem/LocalizationReadOnly"]="true";mp["Mem/InitWMWithAllNodes"]="true";
    mp["Rtabmap/DetectionRate"]="0";mp["Rtabmap/WorkingDirectory"]=argv[5];mp["Mem/UseOdomGravity"]="false";
    Rtabmap core;core.init(mp,std::string(config["database"]));
    std::unique_ptr<Odometry> odom(Odometry::create(op));
    std::ofstream trace(argv[4]);trace<<"source_ns,lost,usable,held_out,processed,reference_id,matched_saved_id,inliers,covariance_valid,x,y,z\n";
    int frames=0,queries=0,matches=0;std::set<int> matched;
    for(const auto &entry:config["segments"]) {
        std::ifstream input{std::string(entry["path"]),std::ios::binary};std::array<char,11> magic{};input.read(magic.data(),magic.size());
        if(!input || std::string(magic.data(),magic.size())!="WBROVRGBD1\n")throw std::runtime_error("recorded source mismatch");
        for(int n=0;n<int(entry["frames"]);++n) {
            const auto f=wbrov_spool::readFrame(input);++frames;
            if(!f.imuValid)continue;
            auto low=sensor(f,true);OdometryInfo info;auto pose=odom->process(low,&info);
            const auto quality=wbrov::mappingEvidence(info,low.imageRaw().cols,low.imageRaw().rows);
            if(!f.map.width)continue;
            // Match observations not stored as any saved DB frame, not a
            // self-match of an image copied directly out of the reference DB.
            const auto stored=savedStamps.lower_bound(static_cast<long long>(f.sourceNs)-1000);
            const bool heldOut=stored==savedStamps.end() || *stored>static_cast<long long>(f.sourceNs)+1000;
            bool processed=false;
            int reference=0,target=0,inliers=0;bool covariance=false;Transform location;
            if(heldOut && quality.usable() && !pose.isNull()) {
                ++queries;processed=core.process(sensor(f,false),pose,info.reg.covariance);
                if(processed) {
                    const auto &s=core.getStatistics();reference=s.refImageId();
                    target=s.loopClosureId()>0?s.loopClosureId():s.proximityDetectionId();
                    auto value=s.data().find("Loop/Visual_inliers/");if(value!=s.data().end())inliers=value->second;
                    const auto &cov=s.localizationCovariance();std::array<double,36> c{};
                    if(cov.type()==CV_64FC1 && cov.rows==6 && cov.cols==6)for(int i=0;i<36;++i)c[i]=cov.at<double>(i/6,i%6);
                    covariance=rtabmap_slam::MappingContinuityGate::covarianceHealthy(c);
                    if(savedIds.count(target) && covariance){++matches;matched.insert(target);location=core.getMapCorrection()*pose;}
                }
            }
            trace<<f.sourceNs<<','<<info.lost<<','<<quality.usable()<<','<<heldOut<<','<<processed<<','<<reference<<','<<target<<','<<inliers<<','<<covariance<<',';
            if(!location.isNull())trace<<location.x()<<','<<location.y()<<','<<location.z();else trace<<",,";
            trace<<'\n';
        }
        if(input.peek()!=std::char_traits<char>::eof())throw std::runtime_error("unaccounted recorded suffix");
    }
    core.close(false);trace.close();
    std::cout<<"{\"frames\":"<<frames<<",\"held_out_queries\":"<<queries<<",\"saved_map_matches\":"<<matches<<",\"distinct_saved_targets\":"<<matched.size()<<",\"hardware_started\":false,\"navigation_authorized\":false}"<<std::endl;
    return matches>=3?0:2;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
