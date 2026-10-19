#pragma once
#include "bounded_async_writer.hpp"
#include "rgbd_frame_codec.hpp"
#include "spool_file_commit.hpp"
#include "spool_io_observation.hpp"
#include "segment_commit_queue.hpp"
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
class Writer {
public:
    // The hook is for deterministic offline fault tests, not a runtime option.
    explicit Writer(std::function<void(const char *)> beforeCommit={}):beforeCommit_(std::move(beforeCommit)){}
    // Compressed bytes waiting for the ONLY spool disk writer. Buffered write()
    // itself can stall on writeback; isolating fsync alone is insufficient.
    // Account string capacity (including in-flight data), not just payload size.
    // This RAM is volatile until the unchanged durability barrier completes.
    static constexpr std::size_t MAX_PENDING_SEGMENTS=128;
    static constexpr std::uint64_t MAX_PENDING_BYTES=2ULL*1024*1024*1024;
    static constexpr std::size_t MAX_SEGMENT_BYTES=32ULL*1024*1024;
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
        telemetry_.open(root_/"seal_latency.jsonl");
        committer_=std::make_unique<SegmentCommitQueue<Segment>>(MAX_PENDING_SEGMENTS,MAX_PENDING_BYTES,[this](const Segment &s){commit(s);});
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
        std::exception_ptr failure;
        try {writer_->close();seal();}catch(...){failure=std::current_exception();}
        // Always join BOTH workers before reporting failure or destroying their
        // state. Prior complete segments may still be safely committed.
        try {committer_->close();}catch(...){if(!failure)failure=std::current_exception();}
        enabled_=false;
        if(failure){try{status("failed",false);}catch(...){}std::rethrow_exception(failure);}
        status("closed",clean);
    }
    ~Writer(){try {finish(false);}catch(...) {}}
private:
    struct Segment {
        std::uint64_t sequence=0,first=0,last=0,count=0,total=0,bytes=0;
        std::string payload;
    };
    static std::string name(std::uint64_t sequence) {std::ostringstream s;s<<"segment-"<<std::setw(8)<<std::setfill('0')<<sequence;return s.str();}
    void consume(const Frame &f) {
        stage_=1;
        committer_->check();
        if(f.sourceNs<=last_)throw std::runtime_error("spool source stamps must increase");
        stage_=2;std::ostringstream encoded(std::ios::binary);writeRecord(encoded,f);
        const auto record=encoded.str();
        if(record.size()>MAX_SEGMENT_BYTES-11)throw std::runtime_error("encoded spool frame exceeds segment bound");
        if(!payload_.empty() && record.size()>MAX_SEGMENT_BYTES-payload_.size())seal();
        if(payload_.empty()){payload_="WBROVRGBD2\n";first_=f.sourceNs;count_=0;}
        payload_.append(record);last_=f.sourceNs;++count_;++total_;
        if(last_-first_>=1000000000ULL)seal();
        stage_=0;
    }
    void seal() {
        if(payload_.empty())return;
        stage_=3;
        Segment segment;segment.sequence=sequence_;segment.first=first_;segment.last=last_;
        segment.count=count_;segment.total=total_;segment.bytes=payload_.size();segment.payload.swap(payload_);
        const auto chargedBytes=segment.payload.capacity()+sizeof(Segment);
        stage_=4;committer_->append(std::move(segment),chargedBytes);++sequence_;stage_=0;
    }
    void commit(const Segment &segment) {
        const auto before=observeIo();const auto begin=std::chrono::steady_clock::now();
        if(segment.sequence!=durableSegments_)throw std::runtime_error("out-of-order durability commit");
        const auto stem=name(segment.sequence);
        if(std::filesystem::space(root_).available<4ULL*1024*1024*1024)throw std::runtime_error("spool less than 4 GiB reserve");
        if(beforeCommit_)beforeCommit_("data_write");
        const auto writeBegin=std::chrono::steady_clock::now();
        std::ofstream stream(root_/(stem+".partial"),std::ios::binary);
        stream.write(segment.payload.data(),segment.payload.size());stream.flush();
        if(!stream)throw std::runtime_error("spool write/flush failed");
        stream.close();if(!stream)throw std::runtime_error("spool close failed");
        const double writeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-writeBegin).count();
        std::ostringstream receipt;receipt<<"{\"schema\":\"wbrov.rgbd-spool-segment.v1\",\"sequence\":"<<segment.sequence
            <<",\"frames\":"<<segment.count<<",\"first_source_ns\":"<<segment.first<<",\"last_source_ns\":"<<segment.last
            <<",\"bytes\":"<<segment.bytes<<"}\n";
        const auto times=commitSegment(root_/(stem+".partial"),root_/(stem+".frames"),root_/(stem+".json"),receipt.str(),beforeCommit_);
        ++durableSegments_;durableLast_=segment.last;durableTotal_=segment.total;
        // Replaceable telemetry, published only AFTER the durability barrier.
        // Startup and final status remain durable; no uncommitted input is
        // advertised, even if the latest recording status is lost on poweroff.
        status("recording",false,false);
        const auto done=std::chrono::steady_clock::now();
        const auto after=observeIo();
        const double totalMs=std::chrono::duration<double,std::milli>(done-begin).count();
        const auto queue=writer_->snapshot();
        const auto pending=committer_->snapshot();
        telemetry_<<"{\"sequence\":"<<segment.sequence<<",\"last_source_ns\":"<<segment.last
            <<",\"flush_ms\":"<<writeMs<<",\"data_write_ms\":"<<writeMs<<",\"data_fsync_ms\":"<<times.dataSyncMs<<",\"data_rename_ms\":"<<times.dataRenameMs
            <<",\"receipt_write_ms\":"<<times.receiptWriteMs<<",\"receipt_fsync_ms\":"<<times.receiptSyncMs
            <<",\"receipt_rename_ms\":"<<times.receiptRenameMs<<",\"directory_fsync_ms\":"<<times.directorySyncMs
            <<",\"total_ms\":"<<totalMs<<",\"queue_depth\":"<<queue.queued<<",\"queue_high_water\":"<<queue.highWater
            <<",\"producer_waits\":"<<queue.waits<<",\"producer_wait_ms\":"<<queue.waitMs
            <<",\"pending_segments\":"<<pending.pending<<",\"pending_bytes\":"<<pending.bytes
            <<",\"pending_high_water_bytes\":"<<pending.highWaterBytes<<",\"scheduler_thread\":\"segment_committer\""
            <<",\"scheduler_available\":"<<(before.schedulerAvailable&&after.schedulerAvailable?"true":"false")
            <<",\"writer_cpu_ns\":"<<counterDelta(before.runtimeNs,after.runtimeNs,before.schedulerAvailable&&after.schedulerAvailable)
            <<",\"writer_runqueue_wait_ns\":"<<counterDelta(before.runqueueNs,after.runqueueNs,before.schedulerAvailable&&after.schedulerAvailable)
            <<",\"memory_available\":"<<(before.memoryAvailable&&after.memoryAvailable?"true":"false")
            <<",\"dirty_kib_before\":"<<before.dirtyKiB<<",\"dirty_kib_after\":"<<after.dirtyKiB
            <<",\"writeback_kib_before\":"<<before.writebackKiB<<",\"writeback_kib_after\":"<<after.writebackKiB
            <<",\"io_psi_available\":"<<(before.pressureAvailable&&after.pressureAvailable?"true":"false")
            <<",\"io_some_us\":"<<counterDelta(before.ioSomeUs,after.ioSomeUs,before.pressureAvailable&&after.pressureAvailable)
            <<",\"io_full_us\":"<<counterDelta(before.ioFullUs,after.ioFullUs,before.pressureAvailable&&after.pressureAvailable)<<"}\n";
        telemetry_.flush(); // diagnostics are not part of the input commit
        if(std::chrono::duration<double>(done-begin).count()>.5)
            std::cerr<<"[RGBD_SPOOL_SEAL_LATENCY] sequence="<<segment.sequence
                     <<" data_write_ms="<<writeMs
                     <<" data_fsync_ms="<<times.dataSyncMs<<" data_rename_ms="<<times.dataRenameMs
                     <<" receipt_fsync_ms="<<times.receiptSyncMs<<" directory_fsync_ms="<<times.directorySyncMs<<std::endl;
    }
    void status(const char *state,bool clean,bool durable=true) {
        std::ostringstream s;s<<"{\"schema\":\"wbrov.rgbd-spool.v1\",\"state\":\""<<state
            <<"\",\"clean_shutdown\":"<<(clean?"true":"false")<<",\"sealed_segments\":"<<durableSegments_
            <<",\"durable_segments\":"<<durableSegments_<<",\"last_source_ns\":"<<durableLast_<<",\"frames\":"<<durableTotal_<<",\"navigation_authorized\":false";
        if(writer_) {const auto q=writer_->snapshot();s<<",\"queue_depth\":"<<q.queued<<",\"queue_high_water\":"<<q.highWater
            <<",\"accepted_frames\":"<<q.accepted<<",\"producer_waits\":"<<q.waits<<",\"producer_wait_ms\":"<<q.waitMs<<",\"producer_max_wait_ms\":"<<q.maxWaitMs;}
        if(committer_){const auto q=committer_->snapshot();s<<",\"pending_segments\":"<<q.pending<<",\"pending_bytes\":"<<q.bytes
            <<",\"pending_high_water_bytes\":"<<q.highWaterBytes<<",\"pending_byte_limit\":"<<MAX_PENDING_BYTES<<",\"pending_segment_limit\":"<<MAX_PENDING_SEGMENTS
            <<",\"pending_storage\":\"compressed_ram\",\"pending_bytes_accounting\":\"capacity_including_inflight\"";}
        s<<"}\n";atomicText(root_/"status.json",s.str(),durable);
    }
    std::filesystem::path root_;std::string payload_;std::ofstream telemetry_;std::unique_ptr<BoundedAsyncWriter<Frame>> writer_;
    std::unique_ptr<SegmentCommitQueue<Segment>> committer_;std::function<void(const char *)> beforeCommit_;
    bool enabled_=false;std::uint64_t first_=0,last_=0,count_=0,total_=0,sequence_=0;
    std::uint64_t durableSegments_=0,durableLast_=0,durableTotal_=0;
    std::atomic<int> stage_{0};
};
}
