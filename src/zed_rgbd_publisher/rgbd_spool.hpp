#pragma once
#include "bounded_async_writer.hpp"
#include "rgbd_frame_codec.hpp"
#include <iostream>
#include <atomic>
#include <array>
#include <vector>
#include <filesystem>
#include <fstream>
#include <memory>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <streambuf>
#include <cstdlib>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>

namespace wbrov_spool {
struct Raster {
    std::uint32_t width=0,height=0;
    std::array<double,4> K{}; // fx,fy,cx,cy, rectified
    std::vector<std::uint8_t> bgr,depth; // packed BGR8 and little-endian float32 Z metres
};
struct Frame {
    std::uint64_t sourceNs=0;
    std::uint32_t imuValid=0;
    std::array<double,10> imu{}; // world_from_camera xyzw, angular velocity, linear acceleration
    Raster low,map;
};
template<class T> inline void put(std::ostream &s,const T &v) {s.write(reinterpret_cast<const char*>(&v),sizeof(v));}
template<class T> inline void get(std::istream &s,T &v) {
    s.read(reinterpret_cast<char*>(&v),sizeof(v));if(!s)throw std::runtime_error("truncated RGB-D spool record");
}
inline void validate(const Raster &r,bool required) {
    if(!required && !r.width && !r.height && r.bgr.empty() && r.depth.empty())return;
    if(r.width<1 || r.height<1 || r.width>1920 || r.height>1200 ||
       r.bgr.size()!=std::size_t(r.width)*r.height*3 || r.depth.size()!=std::size_t(r.width)*r.height*4)
        throw std::runtime_error("invalid RGB-D spool dimensions or strides");
    for(double value:r.K)if(!std::isfinite(value))throw std::runtime_error("nonfinite spool calibration");
    if(r.K[0]<=0 || r.K[1]<=0)throw std::runtime_error("invalid spool focal length");
}
inline void writeRaster(std::ostream &s,const Raster &r) {
    validate(r,false);put(s,r.width);put(s,r.height);put(s,r.K);
    if(r.width){s.write(reinterpret_cast<const char*>(r.bgr.data()),r.bgr.size());s.write(reinterpret_cast<const char*>(r.depth.data()),r.depth.size());}
}
inline void readRaster(std::istream &s,Raster &r) {
    get(s,r.width);get(s,r.height);get(s,r.K);
    if(!r.width && !r.height)return;
    if(!r.width || !r.height || r.width>1920 || r.height>1200)throw std::runtime_error("spool dimensions exceed bound");
    r.bgr.resize(std::size_t(r.width)*r.height*3);r.depth.resize(std::size_t(r.width)*r.height*4);
    s.read(reinterpret_cast<char*>(r.bgr.data()),r.bgr.size());s.read(reinterpret_cast<char*>(r.depth.data()),r.depth.size());
    if(!s)throw std::runtime_error("truncated RGB-D spool pixels");
}
inline void writeFrame(std::ostream &s,const Frame &f) {
    if(!f.sourceNs || f.imuValid>1)throw std::runtime_error("invalid spool timestamp or IMU flag");
    validate(f.low,f.imuValid==1);put(s,f.sourceNs);put(s,f.imuValid);put(s,f.imu);
    writeRaster(s,f.low);writeRaster(s,f.map);
    if(!s)throw std::runtime_error("RGB-D spool write failed");
}
inline Frame readFrame(std::istream &s) {
    Frame f;get(s,f.sourceNs);get(s,f.imuValid);get(s,f.imu);readRaster(s,f.low);readRaster(s,f.map);
    if(!f.sourceNs || f.imuValid>1)throw std::runtime_error("invalid spool timestamp or IMU flag");
    validate(f.low,f.imuValid==1);validate(f.map,false);return f;
}
inline void writeRecord(std::ostream &s,const Frame &f) {
    std::ostringstream raw(std::ios::binary);writeFrame(raw,f);writeCompressedFrame(s,raw.str());
}
inline Frame readRecord(std::istream &s,bool compressed) {
    if(!compressed)return readFrame(s);
    // C++17 istringstream copies the complete decoded frame. The buffer below
    // instead views this call's owned string; readFrame still performs all
    // field/pixel validation and returns independently owned Raster vectors.
    class DecodedBuffer final : public std::streambuf {
    public:
        explicit DecodedBuffer(std::string &data){setg(data.data(),data.data(),data.data()+data.size());}
    };
    auto decoded=readCompressedFrame(s);DecodedBuffer buffer(decoded);std::istream raw(&buffer);auto f=readFrame(raw);
    if(raw.peek()!=std::char_traits<char>::eof())throw std::runtime_error("unaccounted decoded frame suffix");
    return f;
}
inline void syncFile(const std::filesystem::path &p) {
    const int fd=::open(p.c_str(),O_RDONLY);if(fd<0)throw std::runtime_error("spool sync open failed");
    const int rc=::fsync(fd);::close(fd);if(rc)throw std::runtime_error("spool fsync failed");
}
inline void atomicText(const std::filesystem::path &p,const std::string &text) {
    const auto tmp=p.string()+".tmp";
    {std::ofstream out(tmp);out<<text;out.flush();if(!out)throw std::runtime_error("spool status write failed");}
    syncFile(tmp);std::filesystem::rename(tmp,p);syncFile(p.parent_path());
}
class Writer {
public:
    void start(const std::string &calibration) {
        const char *dir=std::getenv("WBROV_RGBD_SPOOL_DIR");if(!dir || !*dir)return;
        if(const char *segment=std::getenv("WBROV_RAW_SEGMENT_SECONDS"))if(*segment)
            throw std::runtime_error("continuous RGB-D spool must not rotate the SDK recorder");
        const std::uint32_t endian=1;
        if(*reinterpret_cast<const std::uint8_t*>(&endian)!=1)throw std::runtime_error("spool requires little-endian host");
        root_=dir;
        if(!root_.is_absolute() || std::filesystem::exists(root_))throw std::runtime_error("new absolute spool directory required");
        std::filesystem::create_directory(root_);
        std::filesystem::copy_file(calibration,root_/"water_calibration.yml");
        const char *raw=std::getenv("WBROV_RAW_RECORDING_DIR");
        if(!raw || !*raw)throw std::runtime_error("RGB-D spool requires continuous raw SVO recording");
        for(const char *file:{"init.json","runtime.json","sdk_version.txt","camera_imu_extrinsics.json"})
            std::filesystem::copy_file(std::filesystem::path(raw)/file,root_/file);
        atomicText(root_/"metadata.json","{\"schema\":\"wbrov.rgbd-spool.v1\",\"historical_only\":true,\"depth_units\":\"m\",\"depth_type\":\"float32le\",\"frame_encoding\":\"zstd-depth-byte-planes-v2\",\"body_from_camera_xyzw\":[-0.5,0.5,-0.5,0.5],\"chunk_source_span_ns\":1000000000}\n");
        status("recording",false);
        writer_=std::make_unique<BoundedAsyncWriter<Frame>>(64,[this](const Frame &f){consume(f);});
        enabled_=true;
    }
    bool enabled() const{return enabled_;}
    void append(Frame f){
        if(!enabled_)return;
        try {
            const auto begin=std::chrono::steady_clock::now();
            if(writer_->append_wait(std::move(f),std::chrono::milliseconds(0))) {
                std::cerr<<"[RGBD_SPOOL_BACKPRESSURE] retained_frame=true capacity=64 wait_ms="
                         <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()<<std::endl;
            }
        } catch(const std::exception &e) {
            throw std::runtime_error(std::string("RGB-D spool: ")+e.what()+" stage="+std::to_string(stage_.load()));
        }
    }
    void finish(bool clean) {
        if(!enabled_)return;
        try {writer_->close();seal();status("closed",clean);enabled_=false;}
        catch(...) {enabled_=false;try {status("failed",false);}catch(...) {}throw;}
    }
    ~Writer(){try {finish(false);}catch(...) {}}
private:
    std::string name() const {std::ostringstream s;s<<"segment-"<<std::setw(8)<<std::setfill('0')<<sequence_;return s.str();}
    void consume(const Frame &f) {
        stage_=1;
        if(f.sourceNs<=last_)throw std::runtime_error("spool source stamps must increase");
        if(!stream_.is_open()) {
            if(std::filesystem::space(root_).available<4ULL*1024*1024*1024)throw std::runtime_error("spool less than 4 GiB reserve");
            stream_.open(root_/(name()+".partial"),std::ios::binary);stream_.write("WBROVRGBD2\n",11);first_=f.sourceNs;count_=0;
        }
        stage_=2;writeRecord(stream_,f);last_=f.sourceNs;++count_;++total_;
        if(last_-first_>=1000000000ULL)seal();
        stage_=0;
    }
    void seal() {
        if(!stream_.is_open())return;
        const auto begin=std::chrono::steady_clock::now();stage_=3;
        stream_.flush();if(!stream_)throw std::runtime_error("spool flush failed");stream_.close();
        const auto partial=root_/(name()+".partial"),final=root_/(name()+".frames");
        const auto flushed=std::chrono::steady_clock::now();stage_=4;
        syncFile(partial);std::filesystem::rename(partial,final);
        const auto synced=std::chrono::steady_clock::now();stage_=5;
        std::ostringstream receipt;receipt<<"{\"schema\":\"wbrov.rgbd-spool-segment.v1\",\"sequence\":"<<sequence_
            <<",\"frames\":"<<count_<<",\"first_source_ns\":"<<first_<<",\"last_source_ns\":"<<last_
            <<",\"bytes\":"<<std::filesystem::file_size(final)<<"}\n";
        atomicText(root_/(name()+".json"),receipt.str());++sequence_;status("recording",false);
        const auto done=std::chrono::steady_clock::now();
        if(std::chrono::duration<double>(done-begin).count()>.5)
            std::cerr<<"[RGBD_SPOOL_SEAL_LATENCY] sequence="<<sequence_-1
                     <<" flush_ms="<<std::chrono::duration<double,std::milli>(flushed-begin).count()
                     <<" data_sync_ms="<<std::chrono::duration<double,std::milli>(synced-flushed).count()
                     <<" receipt_sync_ms="<<std::chrono::duration<double,std::milli>(done-synced).count()<<std::endl;
        stage_=0;
    }
    void status(const char *state,bool clean) {
        std::ostringstream s;s<<"{\"schema\":\"wbrov.rgbd-spool.v1\",\"state\":\""<<state
            <<"\",\"clean_shutdown\":"<<(clean?"true":"false")<<",\"sealed_segments\":"<<sequence_
            <<",\"last_source_ns\":"<<last_<<",\"frames\":"<<total_<<",\"navigation_authorized\":false}\n";
        atomicText(root_/"status.json",s.str());
    }
    std::filesystem::path root_;std::ofstream stream_;std::unique_ptr<BoundedAsyncWriter<Frame>> writer_;
    bool enabled_=false;std::uint64_t first_=0,last_=0,count_=0,total_=0,sequence_=0;
    std::atomic<int> stage_{0};
};
}
