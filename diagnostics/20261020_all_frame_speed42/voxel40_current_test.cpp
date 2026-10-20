#include "RecordedCloudCache.h"
#include "RecordedCloudAssembly.h"
#include "RecordedCloudSamples.h"
#include "RecordedCloudPly.h"
#include <cassert>
#include <set>
#include <stdexcept>
#include <random>
#include <climits>
#include <cstring>
#include <cmath>
#include <sstream>
int main() {
    // Compare the export container against an ordered reference on signed,
    // boundary, highly repetitive and spatially adjacent voxel coordinates.
    // Duplicate insertion must keep the first point, including float bits.
    RecordedCloudCache::Cloud dense;
    std::map<RecordedCloudCache::Key,RecordedCloudCache::Point> expected;
    std::mt19937 rng(381);
    for(int i=0;i<120000;++i) {
        RecordedCloudCache::Key key;
        if(i%3==0)key={int(rng()%151)-75,int(rng()%151)-75,int(rng()%21)-10};
        else if(i%3==1)key={INT_MIN+i,INT_MAX-i,-i};
        else key={i%13,-(i%7),i%5};
        RecordedCloudCache::Point point{float(i),-float(i),.01f*i,static_cast<unsigned char>(i),2,3};
        const bool inserted=dense.emplace(key,point).second;
        assert(inserted==expected.emplace(key,point).second);
    }
    auto checkDense=[&](const RecordedCloudCache::Cloud &actual) {
        assert(actual.size()==expected.size());
        for(const auto &item:expected) {
            const auto &p=actual.at(item.first),&q=item.second;
            assert(std::memcmp(&p.x,&q.x,12)==0 && p.r==q.r && p.g==q.g && p.b==q.b);
        }
    };
    checkDense(dense);auto copied=dense;checkDense(copied);
    std::ostringstream ply(std::ios::binary),reference(std::ios::binary);
    writeRecordedCloudPly(ply,dense);
    reference<<"ply\nformat binary_little_endian 1.0\ncomment historical verified map; not current localization\nelement vertex "<<expected.size()
       <<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for(const auto &item:expected) {
        const auto &p=item.second;
        reference.write(reinterpret_cast<const char*>(&p.x),4);reference.write(reinterpret_cast<const char*>(&p.y),4);reference.write(reinterpret_cast<const char*>(&p.z),4);
        reference.put(p.r);reference.put(p.g);reference.put(p.b);
    }
    assert(ply.str()==reference.str()); // multiple full batches and a partial tail
    copied.reserve(400000);checkDense(copied);
    RecordedCloudCache::Cloud swapped;swapped.swap(copied);checkDense(swapped);assert(copied.empty());
    bool missing=false;try{dense.at({INT_MAX,INT_MAX,INT_MAX});}catch(const std::out_of_range&){missing=true;}assert(missing);
    dense.emplace({INT_MAX,INT_MAX,INT_MAX},{-0.0f,0,0,1,2,3});
    assert(std::signbit(dense.at({INT_MAX,INT_MAX,INT_MAX}).x));
    assert(swapped.size()==expected.size()); // independent copy
    std::ostringstream emptyPly;writeRecordedCloudPly(emptyPly,{});
    assert(emptyPly.str().find("element vertex 0\n")!=std::string::npos && emptyPly.str().substr(emptyPly.str().size()-11)=="end_header\n");

    RecordedCloudCache cache;RecordedCloudCache::Pose pose{};
    int calls=0;
    auto points=[&] {++calls;return RecordedCloudCache::Cloud{{{1,2,3},{1,2,3,10,20,30}}};};
    cache.update(2,pose,"pixelsA",points);cache.update(2,pose,"pixelsA",points);
    assert(calls==1 && cache.assemble().size()==1);
    cache.update(1,pose,"pixelsB",[]{return RecordedCloudCache::Cloud{{{1,2,3},{1,2,3,99,20,30}}};});
    assert(cache.assemble().begin()->second.r==99);
    cache.retain(std::set<int>{2});assert(cache.assemble().begin()->second.r==10);
    pose[3]=.1f;cache.update(2,pose,"pixelsA",points);assert(calls==2);
    cache.update(2,pose,"pixelsC",points);assert(calls==3);
    cache.retain(std::set<int>{});assert(cache.assemble().empty());
    RecordedCloudCache bounded(1);RecordedCloudCache::Cloud assembled;
    for(int id=1;id<=3;++id) {
        bounded.update(id,pose,"same",[&]{return RecordedCloudCache::Cloud{{{id,0,0},{float(id),0,0,1,2,3}}};});
        assembled.insert(bounded.points(id).begin(),bounded.points(id).end());
    }
    assert(assembled.size()==3 && bounded.assemble().size()==1);

    RecordedCloudAssembly unionCache;RecordedCloudAssembly::Inputs inputs;
    std::map<int,RecordedCloudCache::Cloud> data;
    int builds=0;
    auto build=[&](int id)->const RecordedCloudCache::Cloud& {++builds;return data.at(id);};
    auto compare=[&] {
        RecordedCloudCache::Cloud full;
        for(const auto &entry:inputs)full.insert(data.at(entry.first).begin(),data.at(entry.first).end());
        const auto &incremental=unionCache.points();assert(full.size()==incremental.size());
        for(const auto &entry:full) {
            const auto &actual=incremental.at(entry.first),&expected=entry.second;
            assert(actual.x==expected.x && actual.y==expected.y && actual.z==expected.z &&
                actual.r==expected.r && actual.g==expected.g && actual.b==expected.b);
        }
    };
    data[2]={{{0,0,0},{0,0,0,22,0,0}}};inputs[2]={pose,"source2"};
    assert(unionCache.update(inputs,build));compare();assert(builds==1);
    assert(!unionCache.update(inputs,build) && builds==1);
    data[3]={{{0,0,0},{0,0,0,33,0,0}},{{1,0,0},{1,0,0,33,0,0}}};inputs[3]={pose,"source3"};
    unionCache.update(inputs,build);compare();assert(builds==2); // only append
    data[1]={{{0,0,0},{0,0,0,11,0,0}}};inputs[1]={pose,"source1"};
    unionCache.update(inputs,build);compare();assert(builds==5); // lower ID changes winner
    inputs.erase(1);unionCache.update(inputs,build);compare(); // deleted winner reveals node 2
    inputs[2].pose[3]+=.2f;data[2]={{{2,0,0},{2,0,0,22,0,0}}};
    unionCache.update(inputs,build);compare(); // pose correction removes old voxel
    inputs[3].input="different depth source";data[3].at({0,0,0}).r=99;
    unionCache.update(inputs,build);compare(); // same pose, changed source
    auto changed=inputs;changed[4]={pose,"source4"};
    bool failed=false;
    try {unionCache.update(changed,build);}catch(const std::out_of_range &){failed=true;}
    assert(failed);compare();assert(!unionCache.update(inputs,build)); // failed union is not published

    RecordedCloudSamples samples(1024);int decodes=0;
    auto decode=[&] {++decodes;RecordedCloudSamples::Data d;d.points.push_back({1,2,3,4,5,6});return d;};
    assert(samples.get(1,"image1",decode).points[0].r==4);
    samples.get(1,"image1",decode);assert(decodes==1);
    samples.get(1,"image2",decode);assert(decodes==2);
    bool decodeFailed=false;
    try{samples.get(1,"bad",[]()->RecordedCloudSamples::Data{throw std::runtime_error("decode");});}
    catch(const std::runtime_error &){decodeFailed=true;}
    assert(decodeFailed);samples.get(1,"image2",decode);assert(decodes==2);
    for(int id=2;id<=30;++id)samples.get(id,"image",decode);
    assert(samples.sizeBytes()<=1024);samples.get(1,"image2",decode);assert(decodes==31);
    // Full-map scans larger than the cache must retain useful resident nodes.
    // This reproduces the 490-node field map that previously decoded every
    // node again at each checkpoint despite having a 128 MiB cache.
    for(int scan=0;scan<3;++scan) {
        for(int id=2;id<=30;++id)assert(samples.get(id,"image",decode).points[0].x==1);
        const int before=decodes;samples.get(1,"image2",decode);assert(decodes==before);
        assert(samples.sizeBytes()<=1024);
    }
    samples.retain(std::set<int>{});assert(samples.sizeBytes()==0);
    RecordedCloudSamples noResident(0);
    assert(noResident.get(1,"image",decode).points.size()==1 && noResident.sizeBytes()==0);
}
