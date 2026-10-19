import ctypes
import importlib.util
import io
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock
import spool_codec as candidate
import baseline_spool_codec as baseline
import deferred_tag_observations as candidate_tags

spec=importlib.util.spec_from_file_location('baseline_tags',Path(__file__).with_name('baseline_deferred_tag_observations.py'))
baseline_tags=importlib.util.module_from_spec(spec)
with mock.patch.dict('sys.modules',{'spool_codec':baseline}):spec.loader.exec_module(baseline_tags)


def packet(raw,shuffle=True):
    lib=baseline.library();raw=bytearray(raw)
    if shuffle:
        offset=92
        for _ in range(2):
            w,h=struct.unpack_from('<II',raw,offset);offset+=40+w*h*3
            depth=bytes(raw[offset:offset+w*h*4]);raw[offset:offset+w*h*4]=b''.join(depth[i::4] for i in range(4));offset+=w*h*4
    ptr=ctypes.c_void_p;size=ctypes.c_size_t
    for name,args,result in [('ZSTD_createCCtx',[],ptr),('ZSTD_freeCCtx',[ptr],size),
        ('ZSTD_CCtx_setParameter',[ptr,ctypes.c_int,ctypes.c_int],size),
        ('ZSTD_compress2',[ptr,ptr,size,ptr,size],size)]:
        fn=getattr(lib,name);fn.argtypes=args;fn.restype=result
    ctx=lib.ZSTD_createCCtx();assert ctx
    try:
        assert not lib.ZSTD_isError(lib.ZSTD_CCtx_setParameter(ctx,100,1))
        assert not lib.ZSTD_isError(lib.ZSTD_CCtx_setParameter(ctx,201,1))
        raw=bytes(raw);dest=ctypes.create_string_buffer(lib.ZSTD_compressBound(len(raw)))
        n=lib.ZSTD_compress2(ctx,dest,len(dest),raw,len(raw));assert not lib.ZSTD_isError(n)
        return struct.pack('<II',len(raw),n)+dest.raw[:n]
    finally:lib.ZSTD_freeCCtx(ctx)


def raster(w,h):
    # All float bit patterns are legal raw depth storage, including signed
    # zero, NaN payloads and infinities. No float conversions belong here.
    n=w*h;pattern=bytes.fromhex('000000800000807f3412c07fffffffff7f')
    return struct.pack('<II4d',w,h,123.5,124.5,1.25,2.5)+bytes((i*17+3)%256 for i in range(n*3))+(pattern*((n*4+15)//16))[:n*4]


def frame(stamp=1791653820032937000,valid=1,sizes=((17,3),(63,1))):
    return struct.pack('<QI10d',stamp,valid,*[i+.25 for i in range(10)])+b''.join(raster(w,h) for w,h in sizes)


def pixels(module,data,count):
    with tempfile.TemporaryDirectory() as tmp:
        p=Path(tmp)/'sealed.frames';p.write_bytes(data)
        return list(module.map_pixels(p,count))


class ImageReader(unittest.TestCase):
    def test_full_canonical_and_tag_image_are_bit_exact(self):
        for sizes in (((0,0),(0,0)),((1,1),(1,1)),((17,3),(63,1)),((0,0),(1920,1)),((65,1),(0,0)),((320,180),(640,360))):
            with self.subTest(sizes=sizes):
                raw=frame(sizes=sizes);encoded=packet(raw)
                self.assertEqual(baseline.read_compressed_frame(io.BytesIO(encoded)),raw)
                self.assertEqual(candidate.read_compressed_frame(io.BytesIO(encoded)),raw)
                header,image=candidate.read_compressed_map_pixels(io.BytesIO(encoded))
                self.assertEqual(header,raw[:92])
                legacy=pixels(baseline_tags,b'WBROVRGBD1\n'+raw,1)
                self.assertEqual(pixels(candidate_tags,b'WBROVRGBD2\n'+encoded,1),legacy)
                self.assertEqual(image,legacy[0][1] if legacy else None)

    def test_validity_and_time_rules_do_not_change(self):
        for stamps,valid in [([20,30],[1,0]),([20,20],[1,1]),([30,20],[1,1]),([20],[2]),([0],[1])]:
            raw=b''.join(frame(stamp=s,valid=v) for s,v in zip(stamps,valid))
            encoded=b''.join(packet(frame(stamp=s,valid=v)) for s,v in zip(stamps,valid))
            if stamps==[20,30]:
                self.assertEqual(pixels(candidate_tags,b'WBROVRGBD2\n'+encoded,2),pixels(baseline_tags,b'WBROVRGBD1\n'+raw,2))
            else:
                for module,data in ((candidate_tags,b'WBROVRGBD2\n'+encoded),(baseline_tags,b'WBROVRGBD1\n'+raw)):
                    with self.assertRaises(ValueError):pixels(module,data,len(stamps))

    def test_all_zstd_and_length_errors_rejected(self):
        encoded=packet(frame());plain,packed=struct.unpack_from('<II',encoded)
        bad_checksum=bytearray(encoded);bad_checksum[-1]^=1
        invalid=[encoded[:i] for i in (0,4,8,9,len(encoded)-1)]
        invalid += [bytes(bad_checksum),struct.pack('<II',candidate.MAX_FRAME_BYTES+1,1)+b'x',
            struct.pack('<II',171,packed)+encoded[8:],struct.pack('<II',plain,0),
            struct.pack('<II',plain,2**32-1),struct.pack('<II',plain+1,packed)+encoded[8:],
            struct.pack('<II',plain,packed+1)+encoded[8:]+b'x',
            struct.pack('<II',plain,packed*2)+encoded[8:]*2,
            struct.pack('<II',plain,packed)+bytes(packed)]
        for item in invalid:
            for decoder in (baseline.read_compressed_frame,candidate.read_compressed_frame,candidate.read_compressed_map_pixels):
                with self.assertRaises(ValueError):decoder(io.BytesIO(item))

    def test_unused_depth_corruption_and_both_raster_bounds_still_rejected(self):
        raw=frame();second=92+40+17*3*7
        malformed=[raw+b'x',raw[:-1]]
        for begin in (92,second):
            for w,h in ((0,1),(1,0),(1921,1),(1,1201),(2**32-1,2**32-1)):
                data=bytearray(raw);struct.pack_into('<II',data,begin,w,h);malformed.append(bytes(data))
        for raw in malformed:
            encoded=packet(raw,shuffle=False)
            for decoder in (baseline.read_compressed_frame,candidate.read_compressed_frame,candidate.read_compressed_map_pixels):
                with self.assertRaises(ValueError):decoder(io.BytesIO(encoded))
        # A valid ZSTD packet with corrupt stored checksum must be rejected
        # even if the map RGB itself would still decode successfully.
        encoded=bytearray(packet(frame()));encoded[-2]^=0x80
        with self.assertRaises(ValueError):candidate.read_compressed_map_pixels(io.BytesIO(encoded))

    def test_segment_count_and_suffix_still_checked(self):
        encoded=b'WBROVRGBD2\n'+packet(frame())
        for data,count in ((encoded,0),(encoded,2),(encoded+b'x',1),(encoded[:-1],1)):
            with self.assertRaises(ValueError):pixels(candidate_tags,data,count)

    def test_full_bounds_frame(self):
        raw=frame(sizes=((1920,1200),(1920,1200)));encoded=packet(raw)
        self.assertEqual(len(raw),candidate.MAX_FRAME_BYTES)
        self.assertEqual(candidate.read_compressed_frame(io.BytesIO(encoded)),raw)
        header,image=candidate.read_compressed_map_pixels(io.BytesIO(encoded))
        self.assertEqual(header,raw[:92]);self.assertEqual(image[:2],(1920,1200))
        self.assertEqual(len(image[3]),1920*1200*3)


if __name__=='__main__':unittest.main()
