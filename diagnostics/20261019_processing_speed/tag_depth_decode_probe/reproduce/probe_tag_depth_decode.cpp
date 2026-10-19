// Closed-map display-export experiment. Does not run SLAM or modify Core.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/ULogger.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>
using namespace rtabmap;
using Clock=std::chrono::steady_clock;
struct Observation {int node=0,tag=0;std::array<double,8> corners{};};
struct Position {bool valid=false;std::array<float,3> xyz{};};
static Position measure(const SensorData &data,const Transform &node,const Observation &observation) {
    Position result;if(data.cameraModels().size()!=1)return result;
    const auto &depth=data.depthRaw();const auto &model=data.cameraModels().front();
    double u=0,v=0;for(int k=0;k<4;++k){u+=observation.corners[2*k]/4;v+=observation.corners[2*k+1]/4;}
    const int x=std::lround(u),y=std::lround(v);std::vector<float> zs;
    for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx)if(x+dx>=0 && x+dx<depth.cols && y+dy>=0 && y+dy<depth.rows) {
        const float z=depth.type()==CV_32FC1?depth.at<float>(y+dy,x+dx):depth.type()==CV_16UC1?depth.at<unsigned short>(y+dy,x+dx)*.001f:0;
        if(std::isfinite(z) && z>=.2f && z<=1.5f)zs.push_back(z);
    }
    if(zs.size()<9)return result;
    std::sort(zs.begin(),zs.end());const float z=zs[zs.size()/2];
    const float px=(u-model.cx())*z/model.fx(),py=(v-model.cy())*z/model.fy();
    const auto pose=node*model.localTransform();const auto *m=pose.data();result.valid=true;
    result.xyz={m[0]*px+m[1]*py+m[2]*z+m[3],m[4]*px+m[5]*py+m[6]*z+m[7],m[8]*px+m[9]*py+m[10]*z+m[11]};return result;
}
static void decode(SensorData &data,bool depthOnly) {
    if(depthOnly){cv::Mat depth;data.uncompressData(nullptr,&depth);}
    else data.uncompressData();
}
int main(int argc,char **argv) try {
    if(argc!=3)throw std::runtime_error("closed mapping directory and exact observation manifest required");
    const std::filesystem::path root=argv[1];cv::setNumThreads(1);
    cv::FileStorage status((root/"status.json").string(),cv::FileStorage::READ);
    if(std::string(status["state"])!="closed" || int(status["database_closed"])!=1)throw std::runtime_error("refusing active DB");
    cv::FileStorage manifest(argv[2],cv::FileStorage::READ);
    std::map<int,std::map<int,Observation>> tags;std::map<int,SensorData> compressed;
    std::unique_ptr<DBDriver> db(DBDriver::create());
    if(!db->openConnection((root/"map.db").string(),false,true))throw std::runtime_error("read-only open failed");
    const auto poses=db->loadOptimizedPoses();
    for(const auto &entry:manifest["observations"]) {
        Observation o;o.node=int(entry["node"]);o.tag=int(entry["tag"]);const auto xy=entry["corners"];
        if(xy.size()!=8 || !poses.count(o.node))throw std::runtime_error("invalid observation");
        for(int i=0;i<8;++i){o.corners[i]=double(xy[i]);if(!std::isfinite(o.corners[i]))throw std::runtime_error("nonfinite corner");}
        tags[o.tag][o.node]=o;
        if(!compressed.count(o.node))db->getNodeData(o.node,compressed[o.node],true,false,false,false);
    }
    db->closeConnection(false);
    if(tags.empty() || compressed.empty())throw std::runtime_error("no bound observations");
    std::size_t observations=0,valid=0;
    for(const auto &tag:tags)for(const auto &entry:tag.second) {
        auto all=compressed.at(entry.first),depthOnly=all;decode(all,false);decode(depthOnly,true);
        const auto &a=all.depthRaw(),&b=depthOnly.depthRaw();
        if(a.size()!=b.size() || a.type()!=b.type())throw std::runtime_error("depth format changed");
        for(int row=0;row<a.rows;++row)if(std::memcmp(a.ptr(row),b.ptr(row),a.cols*a.elemSize()))throw std::runtime_error("depth bits changed");
        const auto before=measure(all,poses.at(entry.first),entry.second),after=measure(depthOnly,poses.at(entry.first),entry.second);
        if(before.valid!=after.valid || (before.valid && std::memcmp(before.xyz.data(),after.xyz.data(),12)))throw std::runtime_error("marker changed");
        ++observations;if(before.valid)++valid;
    }
    // Replay a fixed ascending node inventory and re-evaluate latest-valid
    // marker fallback at each export. Fresh SensorData copies avoid warming
    // either branch with the other branch's uncompressed data.
    std::vector<int> checkpoints;for(const auto &node:compressed)checkpoints.push_back(node.first);
    auto exports=[&](bool depthOnly) {
        std::ostringstream output;
        for(std::size_t c=0;c<checkpoints.size();c+=std::max(std::size_t(1),checkpoints.size()/25)) {
            output<<"checkpoint="<<checkpoints[c]<<';';
            for(const auto &tag:tags) {
                output<<tag.first<<':';
                for(auto it=tag.second.rbegin();it!=tag.second.rend();++it)if(it->first<=checkpoints[c]) {
                    auto data=compressed.at(it->first);decode(data,depthOnly);
                    const auto p=measure(data,poses.at(it->first),it->second);
                    if(p.valid){output<<p.xyz[0]<<','<<p.xyz[1]<<','<<p.xyz[2];break;}
                }
                output<<';';
            }
        }
        return output.str();
    };
    for(int round=0;round<6;++round) {
        double before=0,after=0;std::string a,b;
        auto run=[&](bool depthOnly){const auto began=Clock::now();auto data=exports(depthOnly);const double elapsed=std::chrono::duration<double,std::milli>(Clock::now()-began).count();if(depthOnly){after=elapsed;b=std::move(data);}else{before=elapsed;a=std::move(data);}};
        run(round%2);run(!(round%2));if(a!=b)throw std::runtime_error("display export changed");
        std::cout<<"{\"round\":"<<round<<",\"tags\":"<<tags.size()<<",\"nodes\":"<<compressed.size()<<",\"observations\":"<<observations<<",\"valid\":"<<valid<<",\"before_ms\":"<<before<<",\"after_ms\":"<<after<<",\"depth_and_marker_exact\":true,\"export_bytes_equal\":true}"<<std::endl;
    }
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
