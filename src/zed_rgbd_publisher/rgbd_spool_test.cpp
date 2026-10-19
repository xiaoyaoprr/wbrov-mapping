#include "rgbd_spool.hpp"
#include <cassert>
#include <sstream>
#include <iostream>
#include <future>
using namespace wbrov_spool;
static void retainedWorkspace(const Frame &expected) {
    std::ostringstream plain;writeFrame(plain,expected);
    std::ostringstream encoded;writeRecord(encoded,expected);const auto bytes=encoded.str();
    Frame retained;
    for(int turn=0;turn<8;++turn) {
        auto invalid=bytes;invalid.back()^=1;std::istringstream corrupt(invalid);bool rejected=false;
        try{readRecord(corrupt,true);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
        std::istringstream stream(bytes+bytes);
        for(int copy=0;copy<2;++copy) {
            auto decoded=readRecord(stream,true);std::ostringstream restored;writeFrame(restored,decoded);
            assert(restored.str()==plain.str());
            if(turn==0 && copy==0)retained=std::move(decoded);
        }
        assert(stream.peek()==std::char_traits<char>::eof());
    }
    // Later decodes and stream-buffer destruction cannot overwrite a Frame
    // already returned to the asynchronous odometry/mapping lanes.
    std::ostringstream saved;writeFrame(saved,retained);assert(saved.str()==plain.str());
}
int main() {
    Frame f;f.sourceNs=1791653820000000123ULL;f.imuValid=1;f.imu[3]=1;
    f.low.width=8;f.low.height=6;f.low.K={10,10,4,3};f.low.bgr.assign(144,42);f.low.depth.assign(192,0);
    std::ostringstream output(std::ios::binary);writeFrame(output,f);auto bytes=output.str();
    std::istringstream input(bytes,std::ios::binary);auto read=readFrame(input);
    assert(read.sourceNs==f.sourceNs && read.low.bgr==f.low.bgr && read.low.depth==f.low.depth && read.imu==f.imu && read.map.width==0);
    for(std::size_t size:{std::size_t(0),std::size_t(9),bytes.size()-1}) {
        std::istringstream truncated(bytes.substr(0,size),std::ios::binary);bool rejected=false;
        try{readFrame(truncated);}catch(const std::runtime_error &){rejected=true;}assert(rejected);
    }
    // Preserve arbitrary float bit patterns (including NaNs and signed zero),
    // both image resolutions, integer nanoseconds and IMU bytes exactly.
    f.map=f.low;f.map.depth.resize(f.low.depth.size());
    for(std::size_t i=0;i<f.low.depth.size();++i){f.low.depth[i]=static_cast<unsigned char>((i*71)%256);f.map.depth[i]=static_cast<unsigned char>((i*37)%256);}
    for(auto *r:{&f.low,&f.map})for(std::size_t i=0;i<r->bgr.size();++i)r->bgr[i]=static_cast<unsigned char>((i*29)%256);
    std::ostringstream plain;writeFrame(plain,f);
    std::ostringstream compressed;writeRecord(compressed,f);
    std::istringstream packed(compressed.str());std::ostringstream restored;writeFrame(restored,readRecord(packed,true));
    assert(plain.str()==restored.str());
    for(std::size_t size:{std::size_t(0),std::size_t(7),compressed.str().size()-1}) {
        std::istringstream bad(compressed.str().substr(0,size));bool rejected=false;
        try{readRecord(bad,true);}catch(const std::runtime_error &){rejected=true;}assert(rejected);
    }
    for(int kind=0;kind<3;++kind) {
        auto data=compressed.str();
        if(kind==0){const std::uint32_t huge=0xffffffffU;std::memcpy(data.data(),&huge,4);}
        else if(kind==1)data.back()^=1; // Zstandard checksum corruption
        else {std::uint32_t length;std::memcpy(&length,data.data()+4,4);++length;std::memcpy(data.data()+4,&length,4);data+='x';}
        std::istringstream bad(data);bool rejected=false;try{readRecord(bad,true);}catch(const std::runtime_error &){rejected=true;}assert(rejected);
    }
    std::istringstream v1("WBROVRGBD1\n"+plain.str()),v2("WBROVRGBD2\n"+compressed.str());
    assert(!readSegmentHeader(v1) && readSegmentHeader(v2));
    assert(readRecord(v1,false).sourceNs==readRecord(v2,true).sourceNs);
    auto other=f;other.sourceNs+=100;other.map=Raster();other.low.bgr.assign(other.low.bgr.size(),73);
    auto reader=std::async(std::launch::async,[other]{retainedWorkspace(other);});
    retainedWorkspace(f);reader.get();
    auto maximum=f;maximum.low.width=1920;maximum.low.height=1200;
    maximum.low.bgr.resize(1920*1200*3,19);maximum.low.depth.resize(1920*1200*4,42);maximum.map=maximum.low;
    std::ostringstream maxPlain,maxPacked,maxRestored;writeFrame(maxPlain,maximum);writeRecord(maxPacked,maximum);
    std::istringstream maxInput(maxPacked.str());writeFrame(maxRestored,readRecord(maxInput,true));
    assert(maxRestored.str()==maxPlain.str() && maxInput.peek()==std::char_traits<char>::eof());
    f.low.width=1921;bool rejected=false;try{writeFrame(output,f);}catch(const std::runtime_error &){rejected=true;}assert(rejected);
    Frame missing;missing.sourceNs=1791653820000000456ULL;
    std::ostringstream gap;writeFrame(gap,missing);std::istringstream gapIn(gap.str());assert(readFrame(gapIn).imuValid==0);
    std::ostringstream packedGap;writeRecord(packedGap,missing);std::istringstream packedGapIn(packedGap.str());assert(readRecord(packedGapIn,true).imuValid==0);
    std::cout<<"PASS exact integer source time, pixel roundtrip, explicit IMU gap, truncation and allocation bounds\n";
}
