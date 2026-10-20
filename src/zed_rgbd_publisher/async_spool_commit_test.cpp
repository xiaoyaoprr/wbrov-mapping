#include "rgbd_spool.hpp"
#include <cassert>
#include <future>
#include <iterator>

static std::string read(const std::filesystem::path &p){std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
int main(){
    namespace fs=std::filesystem;using namespace wbrov_spool;using namespace std::chrono_literals;
    // In-flight bytes count, and a failed sink wakes an already blocked
    // producer. No later segment can be reported as successfully committed.
    std::promise<void> entered,release;auto releaseFuture=release.get_future().share();
    SegmentCommitQueue<int> queue(2,10,[&](const int &){entered.set_value();releaseFuture.wait();throw std::runtime_error("disk failed");});
    queue.append(1,8);entered.get_future().wait();
    auto pending=std::async(std::launch::async,[&]{try{queue.append(2,3);return false;}catch(const std::runtime_error &){return true;}});
    assert(pending.wait_for(20ms)==std::future_status::timeout);
    assert(queue.snapshot().bytes==8 && queue.snapshot().pending==1);
    release.set_value();assert(pending.get());bool failed=false;try{queue.close();}catch(...){failed=true;}assert(failed);

    // Model the measured field burst (49 closed files, about 1.2 GB), using
    // metadata byte accounting only. No gigabytes of image RAM or disk writes.
    std::promise<void> burstEntered,burstRelease;auto burstReady=burstRelease.get_future().share();
    unsigned committed=0;
    SegmentCommitQueue<int> burst(Writer::MAX_PENDING_SEGMENTS,Writer::MAX_PENDING_BYTES,[&](const int &id){
        if(id==0){burstEntered.set_value();burstReady.wait();}assert(id==int(committed++));
    });
    constexpr std::uint64_t fileBytes=24ULL*1024*1024;
    burst.append(0,fileBytes);burstEntered.get_future().wait();
    auto files=std::async(std::launch::async,[&]{for(int i=1;i<50;++i)burst.append(i,fileBytes);});
    const bool burstAccepted=files.wait_for(2s)==std::future_status::ready;
    const auto burstSnapshot=burst.snapshot();
    burstRelease.set_value();files.get();burst.close();
    assert(burstAccepted && burstSnapshot.pending==50 && burstSnapshot.bytes==50*fileBytes);
    assert(committed==50 && burst.snapshot().bytes==0);

    char temp[]="/tmp/wbrov-async-spool-XXXXXX";assert(::mkdtemp(temp));const fs::path root=temp,raw=root/"raw";fs::create_directory(raw);
    for(const char *file:{"init.json","runtime.json","sdk_version.txt","camera_imu_extrinsics.json","water_calibration.yml"})writeText(raw/file,"fixture");
    assert(!::setenv("WBROV_RAW_RECORDING_DIR",raw.c_str(),1));assert(!::unsetenv("WBROV_RAW_SEGMENT_SECONDS"));
    Frame f;f.sourceNs=1000000000;f.imuValid=1;f.imu[3]=1;
    f.low.width=8;f.low.height=6;f.low.K={10,10,4,3};f.low.bgr.assign(144,42);f.low.depth.assign(192,0);
    for(const char *blockedStep:{"data_write","data_sync"}) {
    const auto spool=root/blockedStep;assert(!::setenv("WBROV_RGBD_SPOOL_DIR",spool.c_str(),1));
    std::promise<void> held,unblock;auto unblockFuture=unblock.get_future().share();bool first=true;
    Writer writer([&](const char *step){if(first && std::string(step)==blockedStep){first=false;held.set_value();unblockFuture.wait();}});
    writer.start((raw/"water_calibration.yml").string());
    auto producer=std::async(std::launch::async,[&]{for(int i=0;i<1500;++i){auto frame=f;frame.sourceNs+=i*50000000ULL;writer.append(std::move(frame));}});
    held.get_future().wait();const bool continued=producer.wait_for(2s)==std::future_status::ready;
    // Even the first buffered disk write has not started: the compressor can
    // consume >64 frames without touching the filesystem or publishing input.
    const auto pausedStatus=read(spool/"status.json");bool premature=false;unsigned partials=0;
    for(const auto &entry:fs::directory_iterator(spool)){
        premature|=entry.path().extension()==".frames";
        partials+=entry.path().extension()==".partial";
    }
    unblock.set_value();producer.get();writer.finish(true);
    assert(continued && partials==(std::string(blockedStep)=="data_sync"?1U:0U) && !premature && pausedStatus.find("\"sealed_segments\":0")!=std::string::npos);
    std::vector<fs::path> paths;for(const auto &entry:fs::directory_iterator(spool))if(entry.path().extension()==".frames")paths.push_back(entry.path());
    std::sort(paths.begin(),paths.end());unsigned frames=0;
    for(const auto &p:paths){std::ifstream in(p,std::ios::binary);const bool compressed=readSegmentHeader(in);
        while(in.peek()!=std::char_traits<char>::eof()){const auto actual=readRecord(in,compressed);assert(actual.sourceNs==f.sourceNs+frames*50000000ULL);assert(actual.low.bgr==f.low.bgr && actual.low.depth==f.low.depth);++frames;}}
    assert(frames==1500);const auto closed=read(spool/"status.json");
    for(const char *field:{"\"state\":\"closed\"","\"frames\":1500","\"accepted_frames\":1500","\"pending_segments\":0","\"pending_bytes\":0"})assert(closed.find(field)!=std::string::npos);
    }

    // Complete Writer failure, including close and destructor, must join both
    // stages and retain only the earlier durable high-water mark.
    for(const char *failedStep:{"data_write","data_sync"}) {
    const auto broken=root/(std::string("broken_")+failedStep);assert(!::setenv("WBROV_RGBD_SPOOL_DIR",broken.c_str(),1));unsigned commits=0;failed=false;
    {Writer sink([&](const char *step){if(std::string(step)==failedStep && ++commits==2)throw std::runtime_error("injected disk error");});
     sink.start((raw/"water_calibration.yml").string());
     try{for(int i=0;i<90;++i){auto frame=f;frame.sourceNs+=i*50000000ULL;sink.append(std::move(frame));}sink.finish(true);}catch(...){failed=true;}}
    assert(failed);const auto brokenStatus=read(broken/"status.json");
    assert(brokenStatus.find("\"state\":\"failed\"")!=std::string::npos && brokenStatus.find("\"sealed_segments\":1")!=std::string::npos);
    }
    fs::remove_all(root);
    std::cout<<"PASS blocked disk write permits capture; durability/byte limits, failure wakeup and close preserve all frames\n";
}
