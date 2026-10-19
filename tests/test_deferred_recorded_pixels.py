import io
import struct
import sys
import tempfile
import unittest
import ctypes
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from deferred_tag_observations import map_pixels
from spool_codec import library,read_compressed_frame,read_compressed_map_pixels,MAX_FRAME_BYTES


def compressed_frame(data,shuffle=True):
    lib=library();data=bytearray(data);offset=92
    if shuffle:
        for _ in range(2):
            w,h=struct.unpack_from('<II',data,offset);offset+=40+w*h*3
            depth=bytes(data[offset:offset+w*h*4])
            data[offset:offset+w*h*4]=b''.join(depth[byte::4] for byte in range(4));offset+=w*h*4
    ptr=ctypes.c_void_p;size=ctypes.c_size_t
    for name,args,result in [('ZSTD_createCCtx',[],ptr),('ZSTD_freeCCtx',[ptr],size),
        ('ZSTD_CCtx_setParameter',[ptr,ctypes.c_int,ctypes.c_int],size),
        ('ZSTD_compress2',[ptr,ptr,size,ptr,size],size)]:
        fn=getattr(lib,name);fn.argtypes=args;fn.restype=result
    context=lib.ZSTD_createCCtx();assert context
    try:
        assert not lib.ZSTD_isError(lib.ZSTD_CCtx_setParameter(context,100,1))
        assert not lib.ZSTD_isError(lib.ZSTD_CCtx_setParameter(context,201,1))
        raw=bytes(data);dest=ctypes.create_string_buffer(lib.ZSTD_compressBound(len(raw)))
        size=lib.ZSTD_compress2(context,dest,len(dest),raw,len(raw));assert not lib.ZSTD_isError(size)
        return struct.pack('<II',len(raw),size)+dest.raw[:size]
    finally:lib.ZSTD_freeCCtx(context)

def raster(w,h,pixels):
    return struct.pack('<II4d',w,h,100.,100.,1.,1.)+pixels+bytes(w*h*4)

def frame(stamp,valid=1):
    return struct.pack('<QI10d',stamp,valid,*([0.]*10))+raster(1,1,b'abc')+raster(2,1,b'ABCDEF')

class RecordedPixels(unittest.TestCase):
    def require_codec(self):
        try:library()
        except RuntimeError:self.skipTest('libzstd unavailable on this test host')

    def run_bytes(self,data,count):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'sealed.frames';p.write_bytes(data)
            return list(map_pixels(p,count))

    def test_exact_time_and_pixels_and_missing_imu(self):
        result=self.run_bytes(b'WBROVRGBD1\n'+frame(1791653820032937000)+frame(1791653820049593000,0),2)
        self.assertEqual(len(result),1)
        self.assertEqual(result[0][0],1791653820032937000)
        self.assertEqual(result[0][1][3],b'ABCDEF')

    def test_truncation_regression_suffix_and_bounds_rejected(self):
        valid=b'WBROVRGBD1\n'+frame(10)
        for data,count in [(valid[:-1],1),(valid+b'x',1),(valid+frame(10),2),
            (b'WBROVRGBD1\n'+struct.pack('<QI10d',10,1,*([0.]*10))+struct.pack('<II4d',90000,1,100.,100.,1.,1.),1)]:
            with self.assertRaises(ValueError):self.run_bytes(data,count)

    def test_compressed_frame_and_legacy_produce_identical_observations(self):
        self.require_codec()
        raw=frame(1791653820032937000)
        # Include nonzero depth bits: a decoder that merely skips the byte-plane
        # reversal must fail this byte-for-byte assertion.
        raw=raw[:-8]+bytes.fromhex('0000807f3412c07f')
        packed=compressed_frame(raw)
        self.assertEqual(read_compressed_frame(io.BytesIO(packed)),raw)
        header,image=read_compressed_map_pixels(io.BytesIO(packed))
        self.assertEqual(header,raw[:92])
        self.assertEqual(image,(2,1,[100.,100.,1.,1.],b'ABCDEF'))
        self.assertEqual(self.run_bytes(b'WBROVRGBD2\n'+packed,1),self.run_bytes(b'WBROVRGBD1\n'+raw,1))
        for data in [packed[:-1],struct.pack('<II',MAX_FRAME_BYTES+1,1)+b'x',struct.pack('<II',len(raw),0),
                     struct.pack('<II',len(raw),len(packed)-7)+packed[8:]+b'x']:
            with self.assertRaises(ValueError):read_compressed_frame(io.BytesIO(data))

    def test_map_only_reader_preserves_full_header_empty_rasters_and_bounds(self):
        self.require_codec()
        header=struct.pack('<QI10d',1791653820032937000,1,*[i+.25 for i in range(10)])
        for sizes in (((0,0),(0,0)),((17,3),(63,1)),((65,1),(0,0)),((1920,1200),(1920,1200))):
            rasters=[]
            for w,h in sizes:
                n=w*h;pattern=bytes.fromhex('000000800000807f3412c07fffffffff7f')
                rasters.append(struct.pack('<II4d',w,h,123.5,124.5,1.25,2.5)+b'RGB'*n+(pattern*((n*4+15)//16))[:n*4])
            raw=header+b''.join(rasters);packed=compressed_frame(raw)
            self.assertEqual(read_compressed_frame(io.BytesIO(packed)),raw)
            actual_header,image=read_compressed_map_pixels(io.BytesIO(packed))
            self.assertEqual(actual_header,header)
            w,h=sizes[1]
            self.assertEqual(image,(w,h,[123.5,124.5,1.25,2.5],b'RGB'*(w*h)) if w else None)

    def test_unused_depth_still_requires_checksum_and_valid_layout(self):
        self.require_codec();raw=frame(10);second=92+40+7
        malformed=[raw+b'x',raw[:-1]]
        for offset in (92,second):
            for w,h in ((0,1),(1,0),(1921,1),(1,1201),(2**32-1,2**32-1)):
                data=bytearray(raw);struct.pack_into('<II',data,offset,w,h);malformed.append(bytes(data))
        for data in malformed:
            packed=compressed_frame(data,shuffle=False)
            for reader in (read_compressed_frame,read_compressed_map_pixels):
                with self.assertRaises(ValueError):reader(io.BytesIO(packed))
        packed=bytearray(compressed_frame(raw));packed[-1]^=1
        for reader in (read_compressed_frame,read_compressed_map_pixels):
            with self.assertRaises(ValueError):reader(io.BytesIO(packed))

    def test_map_reader_checks_complete_zstd_frame_and_packet_size(self):
        self.require_codec();packed=compressed_frame(frame(10));plain,size=struct.unpack_from('<II',packed)
        invalid=[packed[:i] for i in (0,4,8,9,len(packed)-1)]
        invalid+=[struct.pack('<II',MAX_FRAME_BYTES+1,1)+b'x',struct.pack('<II',171,size)+packed[8:],
            struct.pack('<II',plain,0),struct.pack('<II',plain,2**32-1),
            struct.pack('<II',plain+1,size)+packed[8:],struct.pack('<II',plain,size+1)+packed[8:]+b'x',
            struct.pack('<II',plain,size*2)+packed[8:]*2,struct.pack('<II',plain,size)+bytes(size)]
        for data in invalid:
            with self.assertRaises(ValueError):read_compressed_map_pixels(io.BytesIO(data))

    def test_compressed_segment_time_count_validity_and_suffix(self):
        self.require_codec()
        good=b'WBROVRGBD2\n'+compressed_frame(frame(20))+compressed_frame(frame(30,0))
        self.assertEqual(self.run_bytes(good,2),self.run_bytes(b'WBROVRGBD1\n'+frame(20)+frame(30,0),2))
        for data,count in ((good,0),(good,3),(good+b'x',2),(good[:-1],2),
            (b'WBROVRGBD2\n'+compressed_frame(frame(20))+compressed_frame(frame(20)),2),
            (b'WBROVRGBD2\n'+compressed_frame(frame(0)),1),
            (b'WBROVRGBD2\n'+compressed_frame(frame(20,2)),1)):
            with self.assertRaises(ValueError):self.run_bytes(data,count)

    def test_returned_pixels_own_storage_across_reads_and_failure(self):
        self.require_codec()
        from spool_codec import _read_shuffled_frame
        raw=frame(10);packed=compressed_frame(raw)
        held=read_compressed_map_pixels(io.BytesIO(packed))
        full=read_compressed_frame(io.BytesIO(packed))
        private=_read_shuffled_frame(io.BytesIO(packed));before=bytes(private)
        bad=bytearray(packed);bad[-1]^=1
        with self.assertRaises(ValueError):read_compressed_map_pixels(io.BytesIO(bad))
        for n in range(20):
            different=frame(n+100)[:-14]+b'UVWXYZ'+bytes(8)
            self.assertEqual(read_compressed_frame(io.BytesIO(compressed_frame(different))),different)
        self.assertIs(type(held[0]),bytes);self.assertIs(type(held[1][3]),bytes)
        self.assertIs(type(full),bytes);self.assertEqual(full,raw)
        self.assertEqual(held,(raw[:92],(2,1,[100.,100.,1.,1.],b'ABCDEF')))
        self.assertEqual(bytes(private),before)

    def test_concurrent_decoding_returns_independent_bytes(self):
        self.require_codec()
        cases=[(frame(n+100),compressed_frame(frame(n+100))) for n in range(32)]
        def decode(case):
            raw,packed=case
            return raw,read_compressed_frame(io.BytesIO(packed)),read_compressed_map_pixels(io.BytesIO(packed))
        with ThreadPoolExecutor(max_workers=4) as pool:result=list(pool.map(decode,cases))
        for raw,full,(header,image) in result:
            self.assertEqual(full,raw);self.assertEqual(header,raw[:92])
            self.assertEqual(image[3],b'ABCDEF')
            self.assertIs(type(full),bytes);self.assertIs(type(header),bytes);self.assertIs(type(image[3]),bytes)

if __name__=='__main__':unittest.main()
