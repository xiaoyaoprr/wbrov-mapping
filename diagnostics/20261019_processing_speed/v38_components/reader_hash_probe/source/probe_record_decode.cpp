#include "rgbd_spool.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <functional>
#include <random>
#include <streambuf>

// Isolated experiment: retain all decompression, checksum, layout and Raster
// validation. Compare removing the istringstream copy and reusing only a ZSTD
// decoder's workspace. Neither variant changes any recording or map input.
class MemoryInput final : public std::streambuf {
public:
    explicit MemoryInput(std::string &data) {setg(data.data(),data.data(),data.data()+data.size());}
};
static std::string decodeWorkspace(std::istream &input) {
    using namespace wbrov_spool;
    std::uint32_t plain=0,packed=0;input.read(reinterpret_cast<char*>(&plain),4);input.read(reinterpret_cast<char*>(&packed),4);
    if(!input || plain<172 || plain>maxFrameBytes || !packed || packed>ZSTD_compressBound(plain))
        throw std::runtime_error("compressed spool length exceeds bound");
    std::string encoded(packed,'\0');input.read(encoded.data(),packed);
    if(!input)throw std::runtime_error("truncated compressed spool frame");
    const auto frameSize=ZSTD_findFrameCompressedSize(encoded.data(),encoded.size());checkZstd(frameSize);
    if(frameSize!=packed || ZSTD_getFrameContentSize(encoded.data(),encoded.size())!=plain)
        throw std::runtime_error("compressed spool frame size mismatch");
    static thread_local std::unique_ptr<ZSTD_DCtx,decltype(&ZSTD_freeDCtx)> context(ZSTD_createDCtx(),ZSTD_freeDCtx);
    if(!context)throw std::runtime_error("cannot allocate RGB-D decoder");
    std::string decoded(plain,'\0');const auto size=ZSTD_decompressDCtx(context.get(),decoded.data(),decoded.size(),encoded.data(),encoded.size());checkZstd(size);
    if(size!=plain)throw std::runtime_error("decoded spool size mismatch");
    depthBytePlanes(decoded,true);return decoded;
}
static wbrov_spool::Frame read(std::istream &input,bool compressed,int mode) {
    if(mode==0 || !compressed)return wbrov_spool::readRecord(input,compressed);
    auto decoded=mode==1?wbrov_spool::readCompressedFrame(input):decodeWorkspace(input);
    MemoryInput buffer(decoded);std::istream raw(&buffer);auto result=wbrov_spool::readFrame(raw);
    if(raw.peek()!=std::char_traits<char>::eof())throw std::runtime_error("unaccounted decoded frame suffix");
    return result;
}
static bool equal(const wbrov_spool::Frame &a,const wbrov_spool::Frame &b) {
    const auto raster=[](const auto &x,const auto &y) {
        return x.width==y.width && x.height==y.height && std::memcmp(x.K.data(),y.K.data(),sizeof(x.K))==0 && x.bgr==y.bgr && x.depth==y.depth;
    };
    return a.sourceNs==b.sourceNs && a.imuValid==b.imuValid && std::memcmp(a.imu.data(),b.imu.data(),sizeof(a.imu))==0 && raster(a.low,b.low) && raster(a.map,b.map);
}
static wbrov_spool::Frame frame(unsigned width,unsigned height,bool map=true) {
    wbrov_spool::Frame f;f.sourceNs=1791653800000000000ULL+width;f.imuValid=1;f.imu[3]=1;
    auto fill=[&](auto &r) {
        r.width=width;r.height=height;r.K={270.,271.,width/2.,height/2.};r.bgr.resize(std::size_t(width)*height*3);
        r.depth.resize(std::size_t(width)*height*4);
        for(std::size_t i=0;i<r.bgr.size();++i)r.bgr[i]=static_cast<unsigned char>(i*37+width);
        const std::array<std::uint32_t,8> patterns{0,0x80000000,0x7fc00013,0x7f800000,0xff800000,0x3f000000,0x3f800000,0xdeadbeef};
        for(std::size_t i=0;i<r.depth.size()/4;++i)std::memcpy(r.depth.data()+i*4,&patterns[i%patterns.size()],4);
    };
    fill(f.low);if(map)fill(f.map);return f;
}
static std::string pack(const wbrov_spool::Frame &f) {
    std::ostringstream out(std::ios::binary);wbrov_spool::writeRecord(out,f);return out.str();
}
static std::string packMalformed(std::string raw) {
    // Intentionally encode malformed *plain* layout, bypassing the writer's
    // validation, to verify every reader rejects it after checksum verification.
    std::unique_ptr<ZSTD_CCtx,decltype(&ZSTD_freeCCtx)> context(ZSTD_createCCtx(),ZSTD_freeCCtx);
    wbrov_spool::checkZstd(ZSTD_CCtx_setParameter(context.get(),ZSTD_c_checksumFlag,1));
    std::string packed(ZSTD_compressBound(raw.size()),'\0');
    const auto count=ZSTD_compress2(context.get(),packed.data(),packed.size(),raw.data(),raw.size());wbrov_spool::checkZstd(count);
    packed.resize(count);std::ostringstream out(std::ios::binary);
    wbrov_spool::put(out,static_cast<std::uint32_t>(raw.size()));wbrov_spool::put(out,static_cast<std::uint32_t>(count));out<<packed;return out.str();
}
static void checkRejected(const std::string &bytes) {
    for(int mode=0;mode<3;++mode) {
        bool rejected=false;try {std::istringstream input(bytes,std::ios::binary);read(input,true,mode);}catch(const std::exception&){rejected=true;}
        assert(rejected);
    }
}
static void tests() {
    std::vector<wbrov_spool::Frame> cases;
    for(unsigned i=1;i<=33;++i)cases.push_back(frame(i,1+i%3,i%2));
    wbrov_spool::Frame absent;absent.sourceNs=1;cases.push_back(absent);
    cases.push_back(frame(1920,1200));
    for(const auto &expected:cases) {
        const auto bytes=pack(expected);
        for(int mode=0;mode<3;++mode) {
            std::istringstream input(bytes+bytes,std::ios::binary);
            assert(equal(read(input,true,mode),expected));assert(equal(read(input,true,mode),expected));
            assert(input.peek()==std::char_traits<char>::eof());
        }
    }
    auto bytes=pack(frame(7,5));
    for(std::size_t i=0;i<bytes.size();++i)checkRejected(bytes.substr(0,i));
    bytes.back()^=1;checkRejected(bytes);
    std::ostringstream raw(std::ios::binary);wbrov_spool::writeFrame(raw,frame(7,5));
    const auto valid=raw.str();
    auto mutate=[&](std::size_t offset,const void *data,std::size_t count) {auto invalid=valid;std::memcpy(invalid.data()+offset,data,count);checkRejected(packMalformed(invalid));};
    std::uint64_t zero=0;mutate(0,&zero,8);
    std::uint32_t badFlag=2;mutate(8,&badFlag,4);
    std::uint32_t badWidth=1921;mutate(92,&badWidth,4);
    double badFocal=0;mutate(100,&badFocal,8);
    std::uint64_t nan=0x7ff8000000000001ULL;mutate(108,&nan,8);
    checkRejected(packMalformed(valid+'x'));
    checkRejected(packMalformed(valid.substr(0,valid.size()-1)));
    // A failed decode must not poison the retained ZSTD workspace.
    for(int i=0;i<10;++i){std::istringstream input(pack(frame(3,7)),std::ios::binary);assert(equal(read(input,true,2),frame(3,7)));}
    std::cout<<"native reader equality, checksum, invalid layout, truncation and workspace reuse tests passed"<<std::endl;
}
static std::vector<std::filesystem::path> segments(const std::filesystem::path &root) {
    std::vector<std::filesystem::path> result;
    for(const auto &p:std::filesystem::directory_iterator(root))if(p.path().extension()==".frames")result.push_back(p.path());
    std::sort(result.begin(),result.end());assert(!result.empty());return result;
}
static void benchmark(const std::filesystem::path &root) {
    const auto files=segments(root);std::size_t checked=0;
    for(const auto &p:files) {
        std::ifstream a(p,std::ios::binary),b(p,std::ios::binary),c(p,std::ios::binary);
        const bool compressed=wbrov_spool::readSegmentHeader(a);assert(compressed);
        assert(wbrov_spool::readSegmentHeader(b) && wbrov_spool::readSegmentHeader(c));
        while(a.peek()!=std::char_traits<char>::eof()) {const auto expected=read(a,true,0);assert(equal(expected,read(b,true,1)) && equal(expected,read(c,true,2)));++checked;}
        assert(b.peek()==std::char_traits<char>::eof() && c.peek()==std::char_traits<char>::eof());
    }
    for(int round=0;round<6;++round)for(int slot=0;slot<3;++slot) {
        const int mode=(round+slot)%3;std::size_t count=0;std::uint64_t stamps=0,bytes=0;
        const auto start=std::chrono::steady_clock::now();
        for(const auto &p:files) {
            std::ifstream input(p,std::ios::binary);assert(wbrov_spool::readSegmentHeader(input));
            while(input.peek()!=std::char_traits<char>::eof()) {
                const auto f=read(input,true,mode);++count;stamps^=f.sourceNs;
                bytes+=f.low.bgr.size()+f.low.depth.size()+f.map.bgr.size()+f.map.depth.size();
            }
        }
        const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        assert(count==checked);
        std::cout<<"{\"round\":"<<round<<",\"mode\":"<<mode<<",\"frames\":"<<count<<",\"bytes\":"<<bytes<<",\"stamp_xor\":"<<stamps<<",\"ms\":"<<ms<<",\"all_frame_bytes_equal\":true}"<<std::endl;
    }
}
int main(int argc,char **argv) {tests();if(argc==2)benchmark(argv[1]);}
