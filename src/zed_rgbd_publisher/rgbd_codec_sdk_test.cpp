#include "rgbd_spool.hpp"
#include <sl/Camera.hpp>
#include <dlfcn.h>
#include <cassert>
#include <iostream>

// Load the real SDK, without opening a camera. The standalone codec test
// cannot detect symbol interposition between the SDK and system ZSTD.
int main() {
    std::cout << "SDK=" << sl::Camera::getSDKVersion().c_str() << std::endl;
    for (auto symbol : {reinterpret_cast<void *>(ZSTD_createCCtx),
                        reinterpret_cast<void *>(ZSTD_freeCCtx),
                        reinterpret_cast<void *>(ZSTD_CCtx_setParameter),
                        reinterpret_cast<void *>(ZSTD_compress2),
                        reinterpret_cast<void *>(ZSTD_createDCtx),
                        reinterpret_cast<void *>(ZSTD_decompressDCtx)}) {
        Dl_info info{};
        assert(dladdr(symbol, &info) && info.dli_fname);
        assert(std::string(info.dli_fname).find("rgbd_codec_sdk_test") != std::string::npos);
    }
    wbrov_spool::Frame f;f.sourceNs=1;f.imuValid=1;
    const auto raster=[](unsigned w,unsigned h) {
        wbrov_spool::Raster r;r.width=w;r.height=h;r.K={500,500,100,100};
        r.bgr.resize(std::size_t(w)*h*3);r.depth.resize(std::size_t(w)*h*4);
        std::uint32_t state=0x12345678;
        for(auto *v:{&r.bgr,&r.depth})for(auto &b:*v){state^=state<<13;state^=state>>17;state^=state<<5;b=state&255;}
        return r;
    };
    f.low=raster(384,240);f.map=raster(960,600);
    for(int n=0;n<4;++n) {
        std::ostringstream expected(std::ios::binary);wbrov_spool::writeFrame(expected,f);
        std::ostringstream encoded(std::ios::binary);wbrov_spool::writeRecord(encoded,f);
        std::istringstream input(encoded.str(),std::ios::binary);const auto restored=wbrov_spool::readRecord(input,true);
        std::ostringstream actual(std::ios::binary);wbrov_spool::writeFrame(actual,restored);
        assert(actual.str()==expected.str());++f.sourceNs;
    }
    std::cout << "SDK loaded; private codec; full RGB-D byte-exact roundtrip PASS\n";
}
