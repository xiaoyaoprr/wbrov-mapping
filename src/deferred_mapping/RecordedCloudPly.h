#pragma once
#include "RecordedCloudCache.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <ostream>
#include <vector>
#include <cstdint>
#include <limits>

// Lossless ordering key if the observed coordinate ranges
// fit in 63 bits; otherwise compare the original complete signed tuple.
inline void writeRecordedCloudPly(std::ostream &out,const RecordedCloudCache::Cloud &cloud) {
    out<<"ply\nformat binary_little_endian 1.0\ncomment historical verified map; not current localization\nelement vertex "<<cloud.size()
       <<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    if(cloud.empty())return;
    using Entry=RecordedCloudCache::Cloud::value_type;
    struct Item {std::uint64_t key;const Entry *entry;};
    std::array<std::int64_t,3> low,high;
    low.fill(std::numeric_limits<int>::max());high.fill(std::numeric_limits<int>::min());
    for(const auto &entry:cloud) {
        const std::array<std::int64_t,3> v{std::get<0>(entry.first),std::get<1>(entry.first),std::get<2>(entry.first)};
        for(int a=0;a<3;++a){low[a]=std::min(low[a],v[a]);high[a]=std::max(high[a],v[a]);}
    }
    std::array<unsigned,3> width{};
    for(int a=0;a<3;++a)for(std::uint64_t range=high[a]-low[a];range;range>>=1)++width[a];
    const bool packed=width[0]+width[1]+width[2]<=63;
    std::vector<Item> ordered;ordered.reserve(cloud.size());
    for(const auto &entry:cloud) {
        std::uint64_t key=0;
        if(packed)key=(std::uint64_t(std::int64_t(std::get<0>(entry.first))-low[0])<<(width[1]+width[2])) |
            (std::uint64_t(std::int64_t(std::get<1>(entry.first))-low[1])<<width[2]) |
            std::uint64_t(std::int64_t(std::get<2>(entry.first))-low[2]);
        ordered.push_back({key,&entry});
    }
    if(packed)std::sort(ordered.begin(),ordered.end(),[](const Item &a,const Item &b){return a.key<b.key;});
    else std::sort(ordered.begin(),ordered.end(),[](const Item &a,const Item &b){return a.entry->first<b.entry->first;});
    std::array<char,15*4096> bytes{};std::size_t used=0;
    for(const auto &item:ordered) {
        const auto &p=item.entry->second;
        std::memcpy(bytes.data()+used,&p.x,4);std::memcpy(bytes.data()+used+4,&p.y,4);std::memcpy(bytes.data()+used+8,&p.z,4);
        bytes[used+12]=p.r;bytes[used+13]=p.g;bytes[used+14]=p.b;used+=15;
        if(used==bytes.size()){out.write(bytes.data(),used);used=0;}
    }
    if(used)out.write(bytes.data(),used);
}
