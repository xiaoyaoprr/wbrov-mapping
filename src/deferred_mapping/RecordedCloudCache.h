#pragma once
#include <map>
#include <tuple>
#include <array>
#include <string>
#include <utility>
#include "RecordedVoxelCloud.h"

// Display/export cache only. All points still come from Core's accepted RGB-D
// and exact optimized poses. It cannot add graph nodes or authorize recovery.
// Deterministic winner: lowest node ID, then first sampled pixel in that node.
class RecordedCloudCache {
public:
    struct Point {float x,y,z;unsigned char r,g,b;};
    using Key=std::tuple<int,int,int>;
    struct KeyHash {
        std::size_t operator()(const Key &key) const {
            std::size_t h=0;
            for(int v:{std::get<0>(key),std::get<1>(key),std::get<2>(key)})
                h^=std::hash<int>{}(v)+std::size_t(0x9e3779b9)+(h<<6)+(h>>2);
            return h;
        }
    };
    // Lookup order does not select the winner: callers visit nodes by ID and
    // pixels in raster order. Export sorts voxel keys for deterministic bytes.
    using Cloud=RecordedVoxelCloud<Point>;
    using Pose=std::array<float,12>;
    explicit RecordedCloudCache(std::size_t capacity=500000):capacity_(capacity) {}
    bool matches(int id,const Pose &pose,const std::string &pixelHash) const {
        const auto old=nodes_.find(id);
        return old!=nodes_.end() && old->second.pose==pose && old->second.pixelHash==pixelHash;
    }
    template<class Builder>
    void update(int id,const Pose &pose,const std::string &pixelHash,Builder build) {
        const auto old=nodes_.find(id);
        if(old!=nodes_.end() && old->second.pose==pose && old->second.pixelHash==pixelHash){old->second.used=++used_;return;}
        Entry entry;entry.pose=pose;entry.pixelHash=pixelHash;entry.points=build();entry.used=++used_;
        if(old!=nodes_.end())points_-=old->second.points.size();
        points_+=entry.points.size();
        nodes_[id]=std::move(entry);
        // The current entry remains valid until the caller has assembled it.
        // Eviction only affects future recomputation, never the output cloud.
        while(points_>capacity_ && nodes_.size()>1) {
            auto oldest=nodes_.end();
            for(auto it=nodes_.begin();it!=nodes_.end();++it)if(it->first!=id &&
                (oldest==nodes_.end() || it->second.used<oldest->second.used))oldest=it;
            points_-=oldest->second.points.size();nodes_.erase(oldest);
        }
    }
    const Cloud &points(int id) const {return nodes_.at(id).points;}
    template<class Nodes> void retain(const Nodes &nodes) {
        for(auto it=nodes_.begin();it!=nodes_.end();) {
            if(!nodes.count(it->first)){points_-=it->second.points.size();it=nodes_.erase(it);}else ++it;
        }
    }
    Cloud assemble() const {
        Cloud result;
        for(const auto &node:nodes_)result.insert(node.second.points.begin(),node.second.points.end());
        return result;
    }
private:
    struct Entry {Pose pose{};std::string pixelHash;Cloud points;std::size_t used=0;};
    std::map<int,Entry> nodes_;
    std::size_t capacity_,points_=0,used_=0;
};
