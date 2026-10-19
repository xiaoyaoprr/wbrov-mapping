#include "rgbd_spool.hpp"
#include <cassert>
#include <algorithm>
#include <iostream>
#include <iterator>

static std::string read(const std::filesystem::path &p) {
    std::ifstream in(p);return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
}
int main() {
    namespace fs=std::filesystem;using namespace wbrov_spool;
    char name[]="/tmp/wbrov-spool-commit-XXXXXX";const char *made=::mkdtemp(name);assert(made);
    const fs::path root=made;
    // Interrupt at EVERY commit boundary. The published high-water mark must
    // not advance before data + receipt + directory are durable. This tests
    // ordering/error propagation, not storage hardware power-loss behavior.
    const std::vector<std::string> steps={"data_sync","data_rename","receipt_write","receipt_sync","receipt_rename","directory_sync"};
    for(std::size_t stop=0;stop<=steps.size();++stop) {
        const auto dir=root/std::to_string(stop);fs::create_directory(dir);
        atomicText(dir/"status.json","sealed=0");writeText(dir/"segment.partial","frame bytes");
        std::vector<std::string> visited;bool failed=false;
        try {
            commitSegment(dir/"segment.partial",dir/"segment.frames",dir/"segment.json","receipt",[&](const char *step){
                assert(visited.size()<steps.size() && step==steps.at(visited.size()));
                if(visited.size()==stop)throw std::runtime_error("injected commit failure");
                visited.push_back(step);
            });
            atomicText(dir/"status.json","sealed=1",false);
        }catch(const std::runtime_error &){failed=true;}
        assert(failed==(stop<steps.size()));
        assert(read(dir/"status.json")== (failed?"sealed=0":"sealed=1"));
        if(!failed)assert(read(dir/"segment.frames")=="frame bytes" && read(dir/"segment.json")=="receipt");
    }
    const auto raw=root/"raw";fs::create_directory(raw);
    for(const char *file:{"init.json","runtime.json","sdk_version.txt","camera_imu_extrinsics.json","water_calibration.yml"})writeText(raw/file,"fixture");
    const auto spool=root/"spool";
    assert(!::setenv("WBROV_RGBD_SPOOL_DIR",spool.c_str(),1));assert(!::setenv("WBROV_RAW_RECORDING_DIR",raw.c_str(),1));
    assert(!::unsetenv("WBROV_RAW_SEGMENT_SECONDS"));
    Frame f;f.sourceNs=1000000000;f.imuValid=1;f.imu[3]=1;
    f.low.width=8;f.low.height=6;f.low.K={10,10,4,3};f.low.bgr.assign(144,42);f.low.depth.assign(192,0);
    {Writer writer;writer.start((raw/"water_calibration.yml").string());
     for(int i=0;i<90;++i){f.sourceNs=1000000000ULL+i*50000000ULL;writer.append(f);}writer.finish(true);}
    std::vector<fs::path> segments;
    for(const auto &entry:fs::directory_iterator(spool))if(entry.path().extension()==".frames")segments.push_back(entry.path());
    std::sort(segments.begin(),segments.end());assert(segments.size()==5);
    unsigned frames=0;
    for(const auto &path:segments) {
        assert(fs::exists(fs::path(path).replace_extension(".json")));
        std::ifstream input(path,std::ios::binary);const bool compressed=readSegmentHeader(input);
        while(input.peek()!=std::char_traits<char>::eof()) {
            auto restored=readRecord(input,compressed);
            assert(restored.sourceNs==1000000000ULL+frames*50000000ULL);
            assert(restored.low.bgr==f.low.bgr && restored.low.depth==f.low.depth);++frames;
        }
    }
    assert(frames==90);const auto status=read(spool/"status.json");
    for(const char *field:{"\"state\":\"closed\"","\"clean_shutdown\":true","\"sealed_segments\":5","\"durable_segments\":5","\"frames\":90","\"accepted_frames\":90","\"queue_depth\":0"})assert(status.find(field)!=std::string::npos);
    const auto metrics=read(spool/"seal_latency.jsonl");assert(std::count(metrics.begin(),metrics.end(),'\n')==5);
    assert(metrics.find("data_fsync_ms")!=std::string::npos && metrics.find("writer_runqueue_wait_ns")!=std::string::npos);
    fs::remove_all(root);
    std::cout<<"PASS interrupted commit never advertised; ordered all-frame drain, durable final status and per-segment diagnostics\n";
}
