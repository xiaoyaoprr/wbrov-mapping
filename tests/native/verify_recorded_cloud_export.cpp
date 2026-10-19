// Closed-run differential audit: reproduce the original ordered-map exporter
// from stored RGB-D and exact optimized poses, then compare every PLY byte.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/ULogger.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <tuple>
#include <memory>
#include <cstring>
#include <array>
#include <set>
#include <cmath>
using namespace rtabmap;
int main(int argc,char **argv) try {
    if(argc!=2)throw std::runtime_error("closed mapping directory required");
    const std::filesystem::path root=argv[1];
    cv::FileStorage status((root/"status.json").string(),cv::FileStorage::READ);
    if(std::string(status["state"])!="closed" || int(status["database_closed"])!=1)
        throw std::runtime_error("refusing active or unconfirmed database");
    cv::FileStorage tags((root/"tags.json").string(),cv::FileStorage::READ);
    std::set<int> ids;for(const auto &n:tags["trajectory"])ids.insert(int(n["node_id"]));
    if(ids.size()!=std::size_t(int(status["main_nodes"])))throw std::runtime_error("incomplete main-node inventory");
    ULogger::setLevel(ULogger::kWarning);
    std::unique_ptr<DBDriver> db(DBDriver::create());
    if(!db->openConnection((root/"map.db").string(),false,true))throw std::runtime_error("read-only DB open failed");
    const auto poses=db->loadOptimizedPoses();
    struct Point{float x,y,z;unsigned char r,g,b;};
    std::map<std::tuple<int,int,int>,Point> full;
    for(int id:ids) {
        SensorData data;db->getNodeData(id,data,true,false,false,false);data.uncompressData();
        const auto &depth=data.depthRaw();const auto &rgb=data.imageRaw();
        if((depth.type()!=CV_32FC1 && depth.type()!=CV_16UC1) || rgb.empty() || data.cameraModels().size()!=1)continue;
        const auto &camera=data.cameraModels().front();const auto pose=poses.at(id)*camera.localTransform();
        if(depth.size()!=rgb.size())throw std::runtime_error("geometry mismatch");
        for(int y=0;y<depth.rows;y+=3)for(int x=0;x<depth.cols;x+=3) {
            const float z=depth.type()==CV_32FC1?depth.at<float>(y,x):depth.at<unsigned short>(y,x)*.001f;
            if(!std::isfinite(z) || z<.2 || z>1.5)continue;
            const float px=(x-camera.cx())*z/camera.fx(),py=(y-camera.cy())*z/camera.fy();
            const auto *m=pose.data();Point p{m[0]*px+m[1]*py+m[2]*z+m[3],m[4]*px+m[5]*py+m[6]*z+m[7],m[8]*px+m[9]*py+m[10]*z+m[11],0,0,0};
            if(rgb.type()==CV_8UC3){const auto c=rgb.at<cv::Vec3b>(y,x);p.r=c[2];p.g=c[1];p.b=c[0];}
            else p.r=p.g=p.b=rgb.at<unsigned char>(y,x);
            full.emplace(std::make_tuple(int(std::floor(p.x/.01)),int(std::floor(p.y/.01)),int(std::floor(p.z/.01))),p);
        }
    }
    db->closeConnection(false);
    std::ifstream ply(root/"main_cloud.ply",std::ios::binary);std::string line;std::size_t declared=0;
    while(std::getline(ply,line)){if(line.rfind("element vertex ",0)==0)declared=std::stoull(line.substr(15));if(line=="end_header")break;}
    if(!ply || declared!=full.size())throw std::runtime_error("point count differs from full rebuild");
    for(const auto &entry:full) {
        const auto &p=entry.second;std::array<char,15> expected{},actual{};
        std::memcpy(expected.data(),&p.x,4);std::memcpy(expected.data()+4,&p.y,4);std::memcpy(expected.data()+8,&p.z,4);
        expected[12]=p.r;expected[13]=p.g;expected[14]=p.b;ply.read(actual.data(),actual.size());
        if(!ply || actual!=expected)throw std::runtime_error("point geometry, color or order differs from full rebuild");
    }
    if(ply.peek()!=std::char_traits<char>::eof())throw std::runtime_error("unaccounted PLY suffix");
    std::cout<<"{\"passed\":true,\"nodes\":"<<ids.size()<<",\"points\":"<<full.size()<<",\"byte_exact\":true}"<<std::endl;
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
