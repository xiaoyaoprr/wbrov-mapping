"""Bounded, lossless WBROVRGBD2 reader; WBROVRGBD1 remains supported."""
import ctypes
import ctypes.util
import struct

MAX_FRAME_BYTES=172+2*1920*1200*7
_library=None


def exact(stream,size):
    data=stream.read(size)
    if len(data)!=size:raise ValueError('truncated recorded RGB-D packet')
    return data


def library():
    global _library
    if _library is None:
        name=ctypes.util.find_library('zstd')
        if not name:raise RuntimeError('libzstd is required for lossless RGB-D spools')
        lib=ctypes.CDLL(name);size=ctypes.c_size_t;ptr=ctypes.c_void_p
        for name,args,result in [
            ('ZSTD_compressBound',[size],size),
            ('ZSTD_findFrameCompressedSize',[ptr,size],size),
            ('ZSTD_getFrameContentSize',[ptr,size],ctypes.c_ulonglong),
            ('ZSTD_decompress',[ptr,size,ptr,size],size),
            ('ZSTD_isError',[size],ctypes.c_uint)]:
            fn=getattr(lib,name);fn.argtypes=args;fn.restype=result
        _library=lib
    return _library


def read_compressed_frame(stream):
    plain,packed=struct.unpack('<II',exact(stream,8))
    if not 172<=plain<=MAX_FRAME_BYTES or not packed:
        raise ValueError('compressed spool length exceeds bound')
    lib=library()
    if packed>lib.ZSTD_compressBound(plain):raise ValueError('compressed spool allocation exceeds bound')
    encoded=exact(stream,packed)
    frame_size=lib.ZSTD_findFrameCompressedSize(encoded,len(encoded))
    if lib.ZSTD_isError(frame_size) or frame_size!=packed or lib.ZSTD_getFrameContentSize(encoded,len(encoded))!=plain:
        raise ValueError('compressed spool frame size mismatch')
    decoded=ctypes.create_string_buffer(plain)
    size=lib.ZSTD_decompress(decoded,plain,encoded,packed)
    if lib.ZSTD_isError(size) or size!=plain:raise ValueError('invalid compressed spool bytes or checksum')
    data=bytearray(decoded.raw);offset=92
    for _ in range(2):
        if len(data)-offset<40:raise ValueError('truncated decoded raster header')
        width,height=struct.unpack_from('<II',data,offset);offset+=40
        if width>1920 or height>1200 or (width==0)!=(height==0):raise ValueError('decoded raster exceeds bounds')
        pixels=width*height
        if len(data)-offset<pixels*7:raise ValueError('truncated decoded raster')
        offset+=pixels*3;depth=data[offset:offset+pixels*4]
        for byte in range(4):data[offset+byte:offset+pixels*4:4]=depth[byte*pixels:(byte+1)*pixels]
        offset+=pixels*4
    if offset!=len(data):raise ValueError('unaccounted decoded frame suffix')
    return bytes(data)
