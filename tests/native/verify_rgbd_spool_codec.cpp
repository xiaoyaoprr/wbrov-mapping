#include "rgbd_spool.hpp"
#include <filesystem>
#include <chrono>
#include <iostream>

// Offline verification/transcoding only. Original inputs are never modified.
// '-' verifies in memory; another output path must not already exist.
int main(int argc,char **argv) try {
    if(argc!=3)throw std::runtime_error("usage: verify_rgbd_spool_codec input.frames output.frames|-");
    namespace fs=std::filesystem;
    const auto began=std::chrono::steady_clock::now();
    const fs::path source=argv[1];if(fs::is_symlink(source))throw std::runtime_error("redirected input");
    std::ifstream input(source,std::ios::binary);const bool compressed=wbrov_spool::readSegmentHeader(input);
    std::ofstream output;
    if(std::string(argv[2])!="-") {
        if(fs::exists(argv[2]) || fs::is_symlink(argv[2]))throw std::runtime_error("output already exists");
        output.open(argv[2],std::ios::binary);output.write("WBROVRGBD2\n",11);
    }
    std::uint64_t count=0,rawBytes=11,packedBytes=11,first=0,last=0;
    while(input.peek()!=std::char_traits<char>::eof()) {
        const auto before=input.tellg();const auto frame=wbrov_spool::readRecord(input,compressed);const auto after=input.tellg();
        if(frame.sourceNs<=last)throw std::runtime_error("source time regressed");
        if(!count)first=frame.sourceNs;
        last=frame.sourceNs;
        std::ostringstream raw;wbrov_spool::writeFrame(raw,frame);
        if(!compressed) {
            input.seekg(before);std::string original(static_cast<std::size_t>(after-before),'\0');input.read(original.data(),original.size());
            if(original!=raw.str())throw std::runtime_error("legacy input bytes changed during decoding");
        }
        std::ostringstream encoded;wbrov_spool::writeRecord(encoded,frame);
        std::istringstream roundtrip(encoded.str());const auto decoded=wbrov_spool::readRecord(roundtrip,true);
        std::ostringstream restored;wbrov_spool::writeFrame(restored,decoded);
        if(restored.str()!=raw.str())throw std::runtime_error("lossless codec roundtrip mismatch");
        if(output.is_open())output<<encoded.str();
        ++count;rawBytes+=raw.str().size();packedBytes+=encoded.str().size();
    }
    if(!count)throw std::runtime_error("empty source segment");
    if(output.is_open()){output.flush();if(!output)throw std::runtime_error("transcode output failed");output.close();}
    std::cout<<"{\"frames\":"<<count<<",\"raw_bytes\":"<<rawBytes<<",\"packed_bytes\":"<<packedBytes
        <<",\"first_source_ns\":\""<<first<<"\",\"last_source_ns\":\""<<last
        <<"\",\"byte_exact\":true,\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<<"}\n";
    return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
