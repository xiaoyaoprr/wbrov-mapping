#pragma once
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

// Candidate export-only lookup: narrow entry IDs reduce the bucket working
// set. Keys and points are unchanged. Large clouds automatically use size_t
// buckets; NarrowLimit is lowered only by the fallback regression fixture.
template<class Point, std::size_t NarrowLimit=std::numeric_limits<std::uint32_t>::max()>
class NarrowVoxel40 {
    static_assert(NarrowLimit>0 && NarrowLimit<=std::numeric_limits<std::uint32_t>::max(), "invalid narrow limit");
public:
    using Key=std::tuple<int,int,int>;
    using value_type=std::pair<Key,Point>;
    using const_iterator=typename std::vector<value_type>::const_iterator;
    using iterator=const_iterator;
    NarrowVoxel40()=default;
    NarrowVoxel40(std::initializer_list<value_type> values){reserve(values.size());insert(values.begin(),values.end());}
    std::size_t size()const{return entries_.size();}
    bool empty()const{return entries_.empty();}
    iterator begin(){return entries_.cbegin();}
    iterator end(){return entries_.cend();}
    const_iterator begin()const{return entries_.begin();}
    const_iterator end()const{return entries_.end();}
    void reserve(std::size_t points){
        const auto buckets=bucketCount(points);
        if(!wide_.empty() || points>=NarrowLimit){
            if(wide_.empty())upgrade(buckets);
            else if(buckets>wide_.size())rehash(wide_,buckets);
        }else if(buckets>narrow_.size())rehash(narrow_,buckets);
        entries_.reserve(points);
    }
    std::pair<iterator,bool> emplace(const Key&key,const Point&point){
        if(wide_.empty() && entries_.size()>=NarrowLimit)upgrade(bucketCount(entries_.size()+1));
        return wide_.empty()?emplaceInto(narrow_,key,point):emplaceInto(wide_,key,point);
    }
    template<class Input>void insert(Input first,Input last){for(;first!=last;++first)emplace(first->first,first->second);}
    const Point&at(const Key&key)const{return wide_.empty()?atIn(narrow_,key):atIn(wide_,key);}
    Point&at(const Key&key){return const_cast<Point&>(static_cast<const NarrowVoxel40&>(*this).at(key));}
    void swap(NarrowVoxel40&other)noexcept{entries_.swap(other.entries_);narrow_.swap(other.narrow_);wide_.swap(other.wide_);}
    bool usesWideIndex()const{return !wide_.empty();}
    std::size_t indexBytes()const{return narrow_.capacity()*sizeof(std::uint32_t)+wide_.capacity()*sizeof(std::size_t);}
private:
    template<class I>static constexpr I missing(){return std::numeric_limits<I>::max();}
    static std::size_t bucketCount(std::size_t points){
        std::size_t n=16;
        while(points>n-n/4){if(n>std::numeric_limits<std::size_t>::max()/2)throw std::length_error("voxel index capacity");n*=2;}
        return n;
    }
    static std::uint64_t mix(std::uint64_t n){
        n=(n^(n>>30))*UINT64_C(0xbf58476d1ce4e5b9);
        n=(n^(n>>27))*UINT64_C(0x94d049bb133111eb);
        return n^(n>>31);
    }
    static std::size_t hash(const Key&key){
        const auto xy=(std::uint64_t(std::uint32_t(std::get<0>(key)))<<32)|std::uint32_t(std::get<1>(key));
        return std::size_t(mix(mix(xy)^std::uint32_t(std::get<2>(key))));
    }
    template<class I>std::size_t findSlot(const std::vector<I>&index,const Key&key)const{
        auto slot=hash(key)&(index.size()-1);
        while(index[slot]!=missing<I>() && entries_[index[slot]].first!=key)slot=(slot+1)&(index.size()-1);
        return slot;
    }
    template<class I>const Point&atIn(const std::vector<I>&index,const Key&key)const{
        if(index.empty())throw std::out_of_range("voxel key");
        const auto id=index[findSlot(index,key)];
        if(id==missing<I>())throw std::out_of_range("voxel key");
        return entries_[id].second;
    }
    template<class I>std::pair<iterator,bool>emplaceInto(std::vector<I>&index,const Key&key,const Point&point){
        if(index.empty())rehash(index,16);
        auto slot=findSlot(index,key);
        if(index[slot]!=missing<I>())return {entries_.begin()+index[slot],false};
        if(entries_.size()+1>index.size()-index.size()/4){rehash(index,bucketCount(entries_.size()+1));slot=findSlot(index,key);}
        entries_.emplace_back(key,point);index[slot]=static_cast<I>(entries_.size()-1);
        return {entries_.end()-1,true};
    }
    template<class I>void rehash(std::vector<I>&index,std::size_t count){
        if(entries_.size()>std::numeric_limits<I>::max())throw std::length_error("voxel index capacity");
        std::vector<I>next(count,missing<I>());
        for(std::size_t id=0;id<entries_.size();++id){
            auto slot=hash(entries_[id].first)&(count-1);
            while(next[slot]!=missing<I>())slot=(slot+1)&(count-1);
            next[slot]=static_cast<I>(id);
        }
        index.swap(next);
    }
    void upgrade(std::size_t buckets){
        rehash(wide_,buckets);
        std::vector<std::uint32_t>().swap(narrow_);
    }
    std::vector<value_type>entries_;
    std::vector<std::uint32_t>narrow_;
    std::vector<std::size_t>wide_;
};
