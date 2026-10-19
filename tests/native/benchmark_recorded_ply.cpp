#include "RecordedCloudPly.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <random>
#include <cmath>
int main(int argc,char **argv)try {
    if(argc!=2)throw std::runtime_error("saved binary PLY required");
    std::ifstream in(argv[1],std::ios::binary);std::string line;std::size_t count=0;
    while(std::getline(in,line)){if(line.rfind("element vertex ",0)==0)count=std::stoull(line.substr(15));if(line=="end_header")break;}
    if(!in || count>5000000)throw std::runtime_error("invalid PLY header");
    std::vector<RecordedCloudCache::Point> points(count);
    for(auto &p:points){in.read(reinterpret_cast<char*>(&p.x),4);in.read(reinterpret_cast<char*>(&p.y),4);in.read(reinterpret_cast<char*>(&p.z),4);p.r=in.get();p.g=in.get();p.b=in.get();}
    if(!in || in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("invalid PLY payload");
    std::mt19937 random(81019);std::shuffle(points.begin(),points.end(),random);
    RecordedCloudCache::Cloud cloud;cloud.reserve(count);
    for(const auto &p:points)cloud.emplace({int(std::floor(p.x/.01)),int(std::floor(p.y/.01)),int(std::floor(p.z/.01))},p);
    if(cloud.size()!=count)throw std::runtime_error("duplicate saved voxel");
    for(int round=0;round<5;++round) {
        auto start=std::chrono::steady_clock::now();std::ostringstream before(std::ios::binary);
        before<<"ply\nformat binary_little_endian 1.0\ncomment historical verified map; not current localization\nelement vertex "<<cloud.size()
           <<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
        std::vector<RecordedCloudCache::Key> ordered;ordered.reserve(cloud.size());
        for(const auto &entry:cloud)ordered.push_back(entry.first);std::sort(ordered.begin(),ordered.end());
        for(const auto &key:ordered){const auto &p=cloud.at(key);before.write(reinterpret_cast<const char*>(&p.x),4);before.write(reinterpret_cast<const char*>(&p.y),4);before.write(reinterpret_cast<const char*>(&p.z),4);before.put(p.r);before.put(p.g);before.put(p.b);}
        const auto oldMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        start=std::chrono::steady_clock::now();std::ostringstream after(std::ios::binary);writeRecordedCloudPly(after,cloud);
        const auto newMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        if(before.str()!=after.str())throw std::runtime_error("serialized bytes differ");
        std::cout<<"{\"round\":"<<round<<",\"points\":"<<count<<",\"before_ms\":"<<oldMs<<",\"after_ms\":"<<newMs<<",\"byte_exact\":true}"<<std::endl;
    }
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
