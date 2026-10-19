#pragma once
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

// Export-only voxel lookup. Entries stay contiguous and retain first-inserted
// winners; the index has no per-voxel allocation. Coordinates are full signed
// ints, not packed/truncated spatial keys. Iteration order never selects the
// winner, and the exporter still sorts the original keys before serialization.
template<class Point> class RecordedVoxelCloud {
public:
    using Key=std::tuple<int,int,int>;
    using value_type=std::pair<Key,Point>;
    using const_iterator=typename std::vector<value_type>::const_iterator;
    using iterator=const_iterator;
    RecordedVoxelCloud()=default;
    RecordedVoxelCloud(std::initializer_list<value_type> values) {reserve(values.size());insert(values.begin(),values.end());}
    std::size_t size() const {return entries_.size();}
    bool empty() const {return entries_.empty();}
    iterator begin(){return entries_.cbegin();}
    iterator end(){return entries_.cend();}
    const_iterator begin() const{return entries_.begin();}
    const_iterator end() const{return entries_.end();}
    void reserve(std::size_t points) {
        const auto buckets=bucketCount(points);
        if(buckets>index_.size())rehash(buckets);
        entries_.reserve(points);
    }
    std::pair<iterator,bool> emplace(const Key &key,const Point &point) {
        if(index_.empty())rehash(16);
        auto slot=findSlot(key);
        if(index_[slot]!=missing())return {entries_.begin()+index_[slot],false};
        // At most 75% load. Probe termination is guaranteed by an empty slot.
        if(entries_.size()+1>index_.size()-index_.size()/4) {
            rehash(bucketCount(entries_.size()+1));slot=findSlot(key);
        }
        entries_.emplace_back(key,point);
        index_[slot]=entries_.size()-1;
        return {entries_.end()-1,true};
    }
    template<class Input> void insert(Input first,Input last) {
        for(;first!=last;++first)emplace(first->first,first->second);
    }
    const Point &at(const Key &key) const {
        if(index_.empty())throw std::out_of_range("voxel key");
        const auto id=index_[findSlot(key)];
        if(id==missing())throw std::out_of_range("voxel key");
        return entries_[id].second;
    }
    Point &at(const Key &key) {return const_cast<Point&>(static_cast<const RecordedVoxelCloud&>(*this).at(key));}
    void swap(RecordedVoxelCloud &other) noexcept {entries_.swap(other.entries_);index_.swap(other.index_);}
private:
    static constexpr std::size_t missing(){return std::numeric_limits<std::size_t>::max();}
    static std::size_t bucketCount(std::size_t points) {
        std::size_t n=16;
        while(points>n-n/4) {
            if(n>std::numeric_limits<std::size_t>::max()/2)throw std::length_error("voxel index capacity");
            n*=2;
        }
        return n;
    }
    static std::uint64_t mix(std::uint64_t n) {
        n=(n^(n>>30))*UINT64_C(0xbf58476d1ce4e5b9);
        n=(n^(n>>27))*UINT64_C(0x94d049bb133111eb);
        return n^(n>>31);
    }
    static std::size_t hash(const Key &key) {
        const auto xy=(std::uint64_t(std::uint32_t(std::get<0>(key)))<<32)|std::uint32_t(std::get<1>(key));
        return std::size_t(mix(mix(xy)^std::uint32_t(std::get<2>(key))));
    }
    std::size_t findSlot(const Key &key) const {
        auto slot=hash(key)&(index_.size()-1);
        while(index_[slot]!=missing() && entries_[index_[slot]].first!=key)slot=(slot+1)&(index_.size()-1);
        return slot;
    }
    void rehash(std::size_t count) {
        std::vector<std::size_t> next(count,missing());
        for(std::size_t id=0;id<entries_.size();++id) {
            auto slot=hash(entries_[id].first)&(count-1);
            while(next[slot]!=missing())slot=(slot+1)&(count-1);
            next[slot]=id;
        }
        index_.swap(next);
    }
    std::vector<value_type> entries_;
    std::vector<std::size_t> index_;
};
