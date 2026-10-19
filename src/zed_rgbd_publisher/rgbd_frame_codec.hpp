#pragma once
#include <zstd.h>
#include <cstdint>
#include <cstring>
#include <istream>
#include <ostream>
#include <memory>
#include <stdexcept>
#include <string>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace wbrov_spool {
constexpr std::size_t maxFrameBytes=172+2*1920*1200*7;
inline void checkZstd(std::size_t code) {
    if(ZSTD_isError(code))throw std::runtime_error(std::string("RGB-D compression: ")+ZSTD_getErrorName(code));
}
// Reorder float32 depth BYTES, never float values. NaN payloads, signed zeros,
// calibration, colour, IMU and source timestamps therefore remain bit exact.
inline void depthBytePlanes(std::string &data,bool inverse) {
    if(data.size()<172 || data.size()>maxFrameBytes)throw std::runtime_error("spool decoded size exceeds bound");
    std::size_t offset=92;
    for(int raster=0;raster<2;++raster) {
        if(data.size()-offset<40)throw std::runtime_error("truncated spool raster header");
        std::uint32_t w,h;std::memcpy(&w,data.data()+offset,4);std::memcpy(&h,data.data()+offset+4,4);
        if(w>1920 || h>1200 || ((w==0)!=(h==0)))throw std::runtime_error("spool raster exceeds codec bounds");
        offset+=40;const std::size_t pixels=std::size_t(w)*h;
        if(data.size()-offset<pixels*7)throw std::runtime_error("truncated spool raster bytes");
        offset+=pixels*3;const std::string depth=data.substr(offset,pixels*4);
        std::size_t pixel=0;
#if defined(__aarch64__)
        auto *destination=reinterpret_cast<std::uint8_t*>(data.data()+offset);
        const auto *source=reinterpret_cast<const std::uint8_t*>(depth.data());
        if(inverse) {
            for(;pixel+16<=pixels;pixel+=16) {
                uint8x16x4_t lanes;
                for(int byte=0;byte<4;++byte)lanes.val[byte]=vld1q_u8(source+byte*pixels+pixel);
                vst4q_u8(destination+4*pixel,lanes);
            }
        }else {
            for(;pixel+16<=pixels;pixel+=16) {
                const uint8x16x4_t lanes=vld4q_u8(source+4*pixel);
                for(int byte=0;byte<4;++byte)vst1q_u8(destination+byte*pixels+pixel,lanes.val[byte]);
            }
        }
#endif
        for(;pixel<pixels;++pixel)for(std::size_t byte=0;byte<4;++byte) {
            if(inverse)data[offset+4*pixel+byte]=depth[byte*pixels+pixel];
            else data[offset+byte*pixels+pixel]=depth[4*pixel+byte];
        }
        offset+=pixels*4;
    }
    if(offset!=data.size())throw std::runtime_error("unaccounted decoded spool suffix");
}
inline bool readSegmentHeader(std::istream &input) {
    char magic[11];input.read(magic,sizeof(magic));
    if(!input)throw std::runtime_error("truncated spool header");
    const std::string value(magic,sizeof(magic));
    if(value=="WBROVRGBD1\n")return false;
    if(value=="WBROVRGBD2\n")return true;
    throw std::runtime_error("spool format mismatch");
}
inline void writeCompressedFrame(std::ostream &out,std::string data) {
    depthBytePlanes(data,false);
    std::unique_ptr<ZSTD_CCtx,decltype(&ZSTD_freeCCtx)> context(ZSTD_createCCtx(),ZSTD_freeCCtx);
    if(!context)throw std::runtime_error("cannot allocate RGB-D compressor");
    checkZstd(ZSTD_CCtx_setParameter(context.get(),ZSTD_c_compressionLevel,1));
    checkZstd(ZSTD_CCtx_setParameter(context.get(),ZSTD_c_checksumFlag,1));
    std::string encoded(ZSTD_compressBound(data.size()),'\0');
    const auto size=ZSTD_compress2(context.get(),encoded.data(),encoded.size(),data.data(),data.size());checkZstd(size);
    const auto plain=static_cast<std::uint32_t>(data.size()),packed=static_cast<std::uint32_t>(size);
    out.write(reinterpret_cast<const char*>(&plain),4);out.write(reinterpret_cast<const char*>(&packed),4);
    out.write(encoded.data(),packed);if(!out)throw std::runtime_error("compressed spool write failed");
}
inline std::string readCompressedFrame(std::istream &input) {
    std::uint32_t plain=0,packed=0;input.read(reinterpret_cast<char*>(&plain),4);input.read(reinterpret_cast<char*>(&packed),4);
    if(!input || plain<172 || plain>maxFrameBytes || !packed || packed>ZSTD_compressBound(plain))
        throw std::runtime_error("compressed spool length exceeds bound");
    std::string encoded(packed,'\0');input.read(encoded.data(),packed);
    if(!input)throw std::runtime_error("truncated compressed spool frame");
    const auto frameSize=ZSTD_findFrameCompressedSize(encoded.data(),encoded.size());checkZstd(frameSize);
    if(frameSize!=packed || ZSTD_getFrameContentSize(encoded.data(),encoded.size())!=plain)
        throw std::runtime_error("compressed spool frame size mismatch");
    // Retain only the decoder workspace, independently for each reader thread.
    // Each call still verifies and decodes the complete frame, including its
    // checksum; no previous input or decoded bytes are used as evidence.
    static thread_local std::unique_ptr<ZSTD_DCtx,decltype(&ZSTD_freeDCtx)> context(ZSTD_createDCtx(),ZSTD_freeDCtx);
    if(!context)throw std::runtime_error("cannot allocate RGB-D decoder");
    std::string decoded(plain,'\0');const auto size=ZSTD_decompressDCtx(context.get(),decoded.data(),decoded.size(),encoded.data(),encoded.size());checkZstd(size);
    if(size!=plain)throw std::runtime_error("decoded spool size mismatch");
    depthBytePlanes(decoded,true);return decoded;
}
}
