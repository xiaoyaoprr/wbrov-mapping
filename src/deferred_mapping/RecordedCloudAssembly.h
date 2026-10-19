#pragma once
#include "RecordedCloudCache.h"

// The already assembled cloud can be extended only when every old node keeps
// its exact pose and input identity, and all additions have larger IDs. Any
// pose correction, rewritten input, removal, or lower-ID insertion rebuilds
// the union. This preserves the lowest-node-ID voxel winner of a full export.
class RecordedCloudAssembly {
public:
    using Cloud=RecordedCloudCache::Cloud;
    struct Identity {
        RecordedCloudCache::Pose pose{};
        std::string input;
        bool operator==(const Identity &other) const {return pose==other.pose && input==other.input;}
    };
    using Inputs=std::map<int,Identity>;

    template<class Builder> bool update(const Inputs &inputs,Builder build) {
        return updatePrepared(inputs,[](const std::vector<int>&){},build,[]{});
    }
    template<class Prepare,class Builder,class Finish>
    bool updatePrepared(const Inputs &inputs,Prepare prepare,Builder build,Finish finish) {
        if(initialized_ && inputs==inputs_)return false;
        bool append=initialized_;
        for(const auto &old:inputs_) {
            const auto next=inputs.find(old.first);
            if(next==inputs.end() || !(next->second==old.second)){append=false;break;}
        }
        if(append && !inputs_.empty())for(const auto &next:inputs) {
            if(!inputs_.count(next.first) && next.first<inputs_.rbegin()->first){append=false;break;}
        }
        Cloud staged=append?cloud_:Cloud{};
        // Optimized poses usually move a cloud without changing its size much.
        // Reserve the prior union size to avoid repeated full-index rehashing.
        if(!append && !inputs.empty())staged.reserve(cloud_.size());
        std::vector<int> required;
        for(const auto &next:inputs)if(!append || !inputs_.count(next.first))required.push_back(next.first);
        prepare(required);
        for(int id:required) {
            const auto &points=build(id);
            staged.insert(points.begin(),points.end());
        }
        // Publish only a complete union. Builder exceptions retain the prior
        // assembly, which may still be needed by the shutdown/export path.
        finish();Inputs nextInputs=inputs;
        cloud_.swap(staged);inputs_.swap(nextInputs);initialized_=true;return true;
    }
    const Cloud &points() const {return cloud_;}
private:
    Inputs inputs_;Cloud cloud_;bool initialized_=false;
};
