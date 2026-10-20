#include "RecordedVoxelCloud.h"
#include "NarrowVoxel40.h"
#include <map>
#include <random>
#include <cstring>
#include <climits>
#include <cstdio>
struct Point{float x,y,z;unsigned char r,g,b;};
using Key=std::tuple<int,int,int>;
float number(uint32_t bits){float f;std::memcpy(&f,&bits,4);return f;}
bool same(const Point&a,const Point&b){return std::memcmp(&a.x,&b.x,12)==0 && a.r==b.r && a.g==b.g && a.b==b.b;}
template<class Cloud>void check(const Cloud&cloud,const std::map<Key,Point>&expected){
    if(cloud.size()!=expected.size())throw std::runtime_error("point count differs");
    for(const auto&item:expected)if(!same(cloud.at(item.first),item.second))throw std::runtime_error("first point bits differ");
}
template<class Cloud>void exercise(bool expectWide){
    Cloud cloud;RecordedVoxelCloud<Point>base;std::map<Key,Point>expected;
    std::mt19937 rng(40001);
    for(int i=0;i<120000;++i){
        Key key=i%3==0?Key{int(rng()%151)-75,int(rng()%151)-75,int(rng()%21)-10}:i%3==1?Key{INT_MIN+i,INT_MAX-i,-i}:Key{i%13,-(i%7),i%5};
        Point point{number(rng()),number(rng()),number(rng()),static_cast<unsigned char>(i),2,3};
        auto a=cloud.emplace(key,point),b=base.emplace(key,point);const auto c=expected.emplace(key,point);
        if(a.second!=c.second || b.second!=c.second || !same(a.first->second,c.first->second) || a.first->first!=c.first->first)throw std::runtime_error("insert result differs");
        if(i%1024==0)cloud.reserve(cloud.size()+10);
    }
    check(cloud,expected);check(base,expected);
    if(cloud.usesWideIndex()!=expectWide)throw std::runtime_error("fallback was not exercised");
    auto ai=cloud.begin(),bi=base.begin();for(;ai!=cloud.end();++ai,++bi)if(ai->first!=bi->first || !same(ai->second,bi->second))throw std::runtime_error("iteration order differs");
    auto copy=cloud;check(copy,expected);copy.reserve(400000);check(copy,expected);
    Cloud swapped;swapped.swap(copy);check(swapped,expected);if(!copy.empty())throw std::runtime_error("swap failed");
    swapped.at(expected.begin()->first).r=99;check(cloud,expected);
    auto moved=std::move(cloud);check(moved,expected);
    bool missing=false;try{moved.at({INT_MAX,INT_MAX,INT_MAX});}catch(const std::out_of_range&){missing=true;}
    if(!missing)throw std::runtime_error("missing key accepted");
    copy={{Key{1,2,3},Point{number(0x80000000),0,0,1,2,3}},{Key{1,2,3},Point{1,0,0,8,9,10}}};
    if(copy.size()!=1 || !same(copy.at({1,2,3}),Point{number(0x80000000),0,0,1,2,3}))throw std::runtime_error("initializer first winner changed");
}
int main(){try{
    exercise<NarrowVoxel40<Point>>(false);
    exercise<NarrowVoxel40<Point,17>>(true);
    // Cross-mode swaps and incremental promotion without reserve.
    using Forced=NarrowVoxel40<Point,4>;Forced wide,narrow;
    for(int n=0;n<8;++n)wide.emplace({n,0,0},{float(n),0,0,1,2,3});
    narrow.emplace({99,0,0},{99,0,0,1,2,3});wide.swap(narrow);
    if(wide.usesWideIndex() || !narrow.usesWideIndex() || wide.at({99,0,0}).x!=99 || narrow.at({7,0,0}).x!=7)throw std::runtime_error("cross-mode swap failed");
    for(int n=0;n<8;++n)wide.emplace({n,0,0},{float(n),0,0,1,2,3});
    if(!wide.usesWideIndex() || wide.size()!=9)throw std::runtime_error("promotion failed");
    std::puts("{\"insert_operations\":240000,\"exact_keys_point_bits_iteration\":true,\"narrow_and_wide_fallback\":true}");return 0;
}catch(const std::exception&e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}}
