#include "rgbd_spool.hpp"
#include <algorithm>
#include <thread>

// Manual, offline disk test. No SDK or camera is linked or opened. Preload a
// bounded closed fixture, replay its original pacing, then verify every byte.
int main(int argc,char **argv) try {
    if(argc!=3)throw std::runtime_error("usage: pace_rgbd_spool_writer closed-spool new-output");
    namespace fs=std::filesystem;using Clock=std::chrono::steady_clock;
    const fs::path source=fs::absolute(argv[1]),output=fs::absolute(argv[2]);
    if(fs::is_symlink(source) || fs::exists(output) || fs::is_symlink(output))
        throw std::runtime_error("input must be direct; output must not exist");
    std::vector<fs::path> segments;
    for(const auto &item:fs::directory_iterator(source))
        if(item.path().extension()==".frames" && !item.is_symlink())segments.push_back(item.path());
    std::sort(segments.begin(),segments.end());
    std::vector<wbrov_spool::Frame> frames;std::size_t bytes=0;
    for(const auto &path:segments) {
        std::ifstream input(path,std::ios::binary);const bool compressed=wbrov_spool::readSegmentHeader(input);
        while(input.peek()!=std::char_traits<char>::eof()) {
            auto frame=wbrov_spool::readRecord(input,compressed);
            if(!frames.empty() && frame.sourceNs<=frames.back().sourceNs)throw std::runtime_error("unordered source");
            bytes+=172+frame.low.bgr.size()+frame.low.depth.size()+frame.map.bgr.size()+frame.map.depth.size();
            if(bytes>3ULL*1024*1024*1024)throw std::runtime_error("fixture exceeds 3 GiB preload bound");
            frames.push_back(std::move(frame));
        }
    }
    if(frames.empty())throw std::runtime_error("empty fixture");
    const auto span=frames.back().sourceNs-frames.front().sourceNs;
    if(span>120000000000ULL)throw std::runtime_error("fixture exceeds two-minute pacing bound");
    if(::setenv("WBROV_RGBD_SPOOL_DIR",output.c_str(),1) ||
       ::setenv("WBROV_RAW_RECORDING_DIR",source.c_str(),1) || ::unsetenv("WBROV_RAW_SEGMENT_SECONDS"))
        throw std::runtime_error("cannot configure offline output");
    wbrov_spool::Writer writer;writer.start((source/"water_calibration.yml").string());
    std::vector<double> waits;double maximumLagMs=0;std::size_t late300=0;
    const auto start=Clock::now();
    for(const auto &frame:frames) {
        const auto due=start+std::chrono::nanoseconds(frame.sourceNs-frames.front().sourceNs);
        std::this_thread::sleep_until(due);
        const double lag=std::chrono::duration<double,std::milli>(Clock::now()-due).count();
        maximumLagMs=std::max(maximumLagMs,lag);if(lag>300)++late300;
        const auto begin=Clock::now();writer.append(frame);
        waits.push_back(std::chrono::duration<double,std::milli>(Clock::now()-begin).count());
    }
    const auto submitted=Clock::now();writer.finish(true);const auto closed=Clock::now();
    segments.clear();for(const auto &item:fs::directory_iterator(output))
        if(item.path().extension()==".frames")segments.push_back(item.path());
    std::sort(segments.begin(),segments.end());std::size_t index=0,encodedBytes=0;
    for(const auto &path:segments) {
        encodedBytes+=fs::file_size(path);
        std::ifstream input(path,std::ios::binary);const bool compressed=wbrov_spool::readSegmentHeader(input);
        if(!compressed)throw std::runtime_error("writer did not emit compressed input");
        while(input.peek()!=std::char_traits<char>::eof()) {
            if(index>=frames.size())throw std::runtime_error("unexpected extra frame");
            std::ostringstream actual,expected;
            wbrov_spool::writeFrame(actual,wbrov_spool::readRecord(input,compressed));
            wbrov_spool::writeFrame(expected,frames.at(index++));
            if(actual.str()!=expected.str())throw std::runtime_error("paced recording changed input bytes");
        }
    }
    if(index!=frames.size())throw std::runtime_error("paced recording lost frames");
    std::sort(waits.begin(),waits.end());
    std::cout<<"{\"scope\":\"offline RGB-D writer only, no concurrent SVO or camera\",\"frames\":"<<index
        <<",\"byte_exact\":true,\"source_seconds\":"<<double(span)/1e9
        <<",\"producer_seconds\":"<<std::chrono::duration<double>(submitted-start).count()
        <<",\"closed_seconds\":"<<std::chrono::duration<double>(closed-start).count()
        <<",\"append_p99_ms\":"<<waits.at((waits.size()-1)*99/100)<<",\"append_max_ms\":"<<waits.back()
        <<",\"pacing_lag_max_ms\":"<<maximumLagMs<<",\"frames_over_300ms_late\":"<<late300
        <<",\"raw_bytes\":"<<bytes<<",\"packed_bytes\":"<<encodedBytes<<"}\n";
    return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
