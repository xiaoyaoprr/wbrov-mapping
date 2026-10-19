#include "rgbd_frame_codec.hpp"
#include <cassert>
#include <sstream>
#include <random>
#include <vector>

static std::string raster(std::uint32_t w,std::uint32_t h) {
    std::string s(40+std::size_t(w)*h*7,'\0');std::memcpy(s.data(),&w,4);std::memcpy(s.data()+4,&h,4);
    std::mt19937 random(381);for(std::size_t i=8;i<s.size();++i)s[i]=random();
    const std::uint32_t special[]{0x80000000u,0x7fc12345u,0x7fa00001u,0x7f800000u};
    for(std::size_t p=0;p<4 && p<std::size_t(w)*h;++p)std::memcpy(s.data()+40+std::size_t(w)*h*3+p*4,&special[p],4);
    return s;
}
static std::string frame(std::uint32_t w,std::uint32_t h,std::uint32_t w2=0,std::uint32_t h2=0) {
    return std::string(92,'x')+raster(w,h)+raster(w2,h2);
}
// Independent scalar reference for valid test frames. Never interpret depth
// bytes as floats: negative zero and signaling/quiet NaN payloads must survive.
static std::string reference(const std::string &raw) {
    auto result=raw;std::size_t offset=92;
    for(int raster=0;raster<2;++raster) {
        std::uint32_t w,h;std::memcpy(&w,raw.data()+offset,4);std::memcpy(&h,raw.data()+offset+4,4);
        const std::size_t pixels=std::size_t(w)*h;offset+=40+pixels*3;
        for(std::size_t byte=0;byte<4;++byte)for(std::size_t pixel=0;pixel<pixels;++pixel)
            result[offset+byte*pixels+pixel]=raw[offset+4*pixel+byte];
        offset+=pixels*4;
    }
    return result;
}
template<class Function> static bool rejects(Function f) {try{f();return false;}catch(const std::exception&){return true;}}
static void check(const std::string &raw) {
    auto data=raw;wbrov_spool::depthBytePlanes(data,false);assert(data==reference(raw));
    wbrov_spool::depthBytePlanes(data,true);assert(data==raw);
    std::ostringstream encoded;wbrov_spool::writeCompressedFrame(encoded,raw);const auto bytes=encoded.str();
    std::uint32_t plain,packed;std::memcpy(&plain,bytes.data(),4);std::memcpy(&packed,bytes.data()+4,4);
    assert(plain==raw.size() && packed+8==bytes.size());
    std::string uncompressed(plain,'\0');const auto size=ZSTD_decompress(uncompressed.data(),plain,bytes.data()+8,packed);
    assert(!ZSTD_isError(size) && size==plain && uncompressed==reference(raw));
    std::istringstream input(bytes);assert(wbrov_spool::readCompressedFrame(input)==raw);
    auto corrupt=bytes;corrupt.back()^=1;std::istringstream damaged(corrupt);
    assert(rejects([&]{wbrov_spool::readCompressedFrame(damaged);}));
}
int main() {
    for(unsigned n=0;n<=65;++n)check(frame(n,n?1:0,3,7)); // all SIMD tails and plane alignments
    check(frame(320,180,640,360));check(frame(1920,1200));
    for(auto bad:std::vector<std::string>{std::string(171,'\0'),frame(1,1).substr(0,178),frame(1,1)+"x",frame(1921,1),frame(1,0)}) {
        auto copy=bad;assert(rejects([&]{wbrov_spool::depthBytePlanes(bad,false);}) && rejects([&]{wbrov_spool::depthBytePlanes(copy,true);}));
    }
}
