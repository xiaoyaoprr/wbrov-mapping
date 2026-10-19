#pragma once
#include "RecordedCloudCache.h"
#include <vector>

// Immutable sampled camera points, before optimized world pose or voxelization.
// Pose corrections need no image/depth decompression. This is an export cache,
// never a graph or recovery input. Keep a stable resident set across full-map
// scans: LRU eviction during a sequential scan would evict every node before
// its next visit. Nonresident data uses one transient slot and is rebuilt from
// accepted RGB-D. Removing graph nodes releases capacity for new admissions.
class RecordedCloudSamples {
public:
    struct Data {
        std::vector<RecordedCloudCache::Point> points;
        RecordedCloudCache::Pose cameraTransform{};
    };
    explicit RecordedCloudSamples(std::size_t capacityBytes=512*1024*1024):capacity_(capacityBytes) {}
    template<class Builder> const Data &get(int id,const std::string &identity,Builder build) {
        auto old=entries_.find(id);
        if(old!=entries_.end() && old->second.identity==identity) {
            return old->second.data;
        }
        Entry entry{identity,build()};
        // Build first: a decoding error must not discard the previous entry.
        if(old!=entries_.end()){bytes_-=bytes(old->second);entries_.erase(old);}
        const auto size=bytes(entry);
        if(size>capacity_-bytes_) {
            transient_=std::move(entry.data);return transient_;
        }
        bytes_+=size;entries_[id]=std::move(entry);
        return entries_.at(id).data;
    }
    template<class Nodes> void retain(const Nodes &nodes) {
        for(auto it=entries_.begin();it!=entries_.end();)
            if(!nodes.count(it->first)){bytes_-=bytes(it->second);it=entries_.erase(it);}else ++it;
    }
    std::size_t sizeBytes() const {return bytes_;}
private:
    struct Entry {std::string identity;Data data;};
    static std::size_t bytes(const Entry &entry) {
        return sizeof(Entry)+entry.identity.capacity()+entry.data.points.capacity()*sizeof(RecordedCloudCache::Point);
    }
    std::map<int,Entry> entries_;
    Data transient_;
    std::size_t capacity_,bytes_=0;
};
