"""Bounded, lossless WBROVRGBD2 reader; WBROVRGBD1 remains supported."""
import ctypes
import ctypes.util
import struct
import threading

MAX_FRAME_BYTES=172+2*1920*1200*7
_library=None
_contexts=threading.local()


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
            ('ZSTD_createDCtx',[],ptr),
            ('ZSTD_freeDCtx',[ptr],size),
            ('ZSTD_decompressDCtx',[ptr,ptr,size,ptr,size],size),
            ('ZSTD_isError',[size],ctypes.c_uint)]:
            fn=getattr(lib,name);fn.argtypes=args;fn.restype=result
        _library=lib
    return _library



class _DecodeContext:
    def __init__(self,lib):
        self.pointer=lib.ZSTD_createDCtx()
        self.free=lib.ZSTD_freeDCtx
        if not self.pointer:raise MemoryError('cannot allocate spool decoder workspace')

    def __del__(self):
        if self.pointer:self.free(self.pointer)


def _decode_context(lib):
    context=getattr(_contexts,'decoder',None)
    if context is None:
        context=_DecodeContext(lib);_contexts.decoder=context
    return context.pointer


def _read_shuffled_frame(stream):
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
    size=lib.ZSTD_decompressDCtx(_decode_context(lib),decoded,plain,encoded,packed)
    if lib.ZSTD_isError(size) or size!=plain:raise ValueError('invalid compressed spool bytes or checksum')
    return decoded.raw


def _raster_layout(data):
    # Validate BOTH rasters and the entire decompressed payload, even when a
    # caller needs only the map image. No corrupt depth payload is ignored.
    offset=92;rasters=[]
    for _ in range(2):
        if len(data)-offset<40:raise ValueError('truncated decoded raster header')
        width,height,*intrinsics=struct.unpack_from('<II4d',data,offset);offset+=40
        if width>1920 or height>1200 or (width==0)!=(height==0):raise ValueError('decoded raster exceeds bounds')
        pixels=width*height
        if len(data)-offset<pixels*7:raise ValueError('truncated decoded raster')
        rasters.append((width,height,intrinsics,offset,offset+pixels*3,offset+pixels*7))
        offset+=pixels*7
    if offset!=len(data):raise ValueError('unaccounted decoded frame suffix')
    return rasters


def read_compressed_frame(stream):
    data=bytearray(_read_shuffled_frame(stream))
    for width,height,_,_,begin,end in _raster_layout(data):
        pixels=width*height;depth=data[begin:end]
        for byte in range(4):data[begin+byte:end:4]=depth[byte*pixels:(byte+1)*pixels]
    return bytes(data)


def read_compressed_map_pixels(stream):
    """Validated original header and map RGB for Tag detection only.

    Full ZSTD content/checksum and BOTH raster layouts are checked. Metric
    depth remains byte-planed in this temporary buffer because this caller
    uses no depth. The native mapping worker still decodes every RGB-D frame.
    """
    data=_read_shuffled_frame(stream)
    width,height,intrinsics,begin,end,_=_raster_layout(data)[1]
    image=(width,height,intrinsics,data[begin:end]) if width and height else None
    return data[:92],image
