// Read-only closed-map benchmark. Both containers transform the same sampled
// RGB-D points with identical arithmetic and compare every output coordinate
// and color. No graph, sampling, voxel size or recovery setting is changed.
#include "RecordedCloudCache.h"
#include "NarrowVoxel40.h"
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/ULogger.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <unordered_map>
#include <algorithm>
using Clock=std::chrono::steady_clock;
using Point=RecordedCloudCache::Point;
using Key=RecordedCloudCache::Key;
struct Node {std::vector<Point> points;rtabmap::Transform pose;};
struct Result {double transformMs=0,unionMs=0,sortMs=0;std::vector<std::pair<Key,Point>> points;};
double ms(Clock::time_point from){return std::chrono::duration<double,std::milli>(Clock::now()-from).count();}
template<class Cloud> Result assemble(const std::vector<Node> &nodes) {
    Result result;Cloud full;
    for(const auto &node:nodes) {
        auto start=Clock::now();Cloud part;part.reserve(node.points.size());const auto *m=node.pose.data();
        for(const auto &p:node.points) {
            Point world{m[0]*p.x+m[1]*p.y+m[2]*p.z+m[3],m[4]*p.x+m[5]*p.y+m[6]*p.z+m[7],m[8]*p.x+m[9]*p.y+m[10]*p.z+m[11],p.r,p.g,p.b};
            part.emplace(std::make_tuple(int(std::floor(world.x/.01)),int(std::floor(world.y/.01)),int(std::floor(world.z/.01))),world);
        }
        result.transformMs+=ms(start);start=Clock::now();full.insert(part.begin(),part.end());result.unionMs+=ms(start);
    }
    const auto start=Clock::now();result.points.assign(full.begin(),full.end());
    std::sort(result.points.begin(),result.points.end(),[](const auto &a,const auto &b){return a.first<b.first;});result.sortMs=ms(start);return result;
}
int main(int argc,char **argv) try {
    if(argc!=2)throw std::runtime_error("closed mapping directory required");
    const std::filesystem::path root=argv[1];
    cv::FileStorage status((root/"status.json").string(),cv::FileStorage::READ);
    if(std::string(status["state"])!="closed" || int(status["database_closed"])!=1)throw std::runtime_error("refusing active DB");
    cv::FileStorage tags((root/"tags.json").string(),cv::FileStorage::READ);
    std::set<int> ids;for(const auto &n:tags["trajectory"])ids.insert(int(n["node_id"]));
    if(ids.size()!=std::size_t(int(status["main_nodes"])))throw std::runtime_error("main-node inventory mismatch");
    ULogger::setLevel(ULogger::kWarning);std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
    if(!db->openConnection((root/"map.db").string(),false,true))throw std::runtime_error("read-only open failed");
    const auto poses=db->loadOptimizedPoses();std::vector<Node> nodes;
    for(int id:ids) {
        rtabmap::SensorData data;db->getNodeData(id,data,true,false,false,false);data.uncompressData();
        const auto &depth=data.depthRaw(),&rgb=data.imageRaw();
        if((depth.type()!=CV_32FC1 && depth.type()!=CV_16UC1) || rgb.empty() || data.cameraModels().size()!=1)continue;
        const auto &camera=data.cameraModels().front();Node node;node.pose=poses.at(id)*camera.localTransform();
        if(depth.size()!=rgb.size())throw std::runtime_error("geometry mismatch");
        for(int y=0;y<depth.rows;y+=3)for(int x=0;x<depth.cols;x+=3) {
            const float z=depth.type()==CV_32FC1?depth.at<float>(y,x):depth.at<unsigned short>(y,x)*.001f;
            if(!std::isfinite(z) || z<.2 || z>1.5)continue;
            Point p{float((x-camera.cx())*z/camera.fx()),float((y-camera.cy())*z/camera.fy()),z,0,0,0};
            if(rgb.type()==CV_8UC3){const auto c=rgb.at<cv::Vec3b>(y,x);p.r=c[2];p.g=c[1];p.b=c[0];}
            else p.r=p.g=p.b=rgb.at<unsigned char>(y,x);
            node.points.push_back(p);
        }
        nodes.push_back(std::move(node));
    }
    db->closeConnection(false);
    using Legacy=RecordedVoxelCloud<Point>;
    using Candidate=NarrowVoxel40<Point>;
    for(int round=0;round<8;++round) {
        Result before,after;
        if(round%2){after=assemble<Candidate>(nodes);before=assemble<Legacy>(nodes);}
        else{before=assemble<Legacy>(nodes);after=assemble<Candidate>(nodes);}
        if(before.points.size()!=after.points.size())throw std::runtime_error("count changed");
        for(std::size_t i=0;i<before.points.size();++i) {
            const auto &a=before.points[i],&b=after.points[i];
            if(a.first!=b.first || std::memcmp(&a.second.x,&b.second.x,12) || a.second.r!=b.second.r || a.second.g!=b.second.g || a.second.b!=b.second.b)throw std::runtime_error("output changed");
        }
        std::cout<<"{\"round\":"<<round<<",\"nodes\":"<<nodes.size()<<",\"points\":"<<after.points.size()<<",\"byte_exact\":true,\"before_ms\":["<<before.transformMs<<','<<before.unionMs<<','<<before.sortMs<<"],\"after_ms\":["<<after.transformMs<<','<<after.unionMs<<','<<after.sortMs<<"]}"<<std::endl;
    }
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
