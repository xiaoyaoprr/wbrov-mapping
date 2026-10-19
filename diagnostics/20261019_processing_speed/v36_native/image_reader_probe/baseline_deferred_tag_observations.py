"""Reuse the live measured-corner verifier on immutable recorded map pixels.

No pose is estimated here. Original time, pixels and camera calibration are
bound to each observation; Core must still reopen its own metric depth and
validate three independent pairs plus held-out observations before any merge.
"""
from pathlib import Path
import struct
import sys
import io
from spool_codec import read_compressed_frame


def exact(stream, size):
    data=stream.read(size)
    if len(data)!=size:raise ValueError('truncated recorded RGB-D packet')
    return data


def raster(stream, retain=False):
    width,height,*intrinsics=struct.unpack('<II4d',exact(stream,40))
    if width==height==0:return None
    if not 1<=width<=1920 or not 1<=height<=1200:raise ValueError('recorded raster exceeds bounds')
    rgb=exact(stream,width*height*3)
    exact(stream,width*height*4)
    return (width,height,intrinsics,rgb) if retain else None


def map_pixels(path, expected_count):
    previous=0
    with path.open('rb') as stream:
        magic=exact(stream,11)
        if magic not in (b'WBROVRGBD1\n',b'WBROVRGBD2\n'):raise ValueError('incorrect spool magic')
        for _ in range(expected_count):
            packet=io.BytesIO(read_compressed_frame(stream)) if magic==b'WBROVRGBD2\n' else stream
            stamp,valid,*imu=struct.unpack('<QI10d',exact(packet,92))
            if stamp<=previous or valid not in (0,1):raise ValueError('invalid recorded timestamp or IMU flag')
            previous=stamp
            raster(packet)
            image=raster(packet,True)
            if packet is not stream and packet.read(1):raise ValueError('unaccounted decoded packet suffix')
            if image is not None and valid:yield stamp,image
        if stream.read(1):raise ValueError('unaccounted recorded packet suffix')


class TagObserver:
    def __init__(self, modules):
        import cv2
        cv2.setNumThreads(1)
        sys.path.insert(0,str(modules.resolve(strict=True)))
        from verified_pixel_observer import VerifiedPixelObserver
        self.observer=VerifiedPixelObserver()

    def observe(self, path, count):
        import numpy as np
        result=[]
        for stamp,(width,height,intrinsics,pixels) in map_pixels(path,count):
            fx,fy,cx,cy=intrinsics
            measured=self.observer.observe(np.frombuffer(pixels,dtype=np.uint8).reshape(height,width,3),
                stamp_ns=stamp,camera_frame='recorded_camera_optical',
                K=[fx,0.,cx,0.,fy,cy,0.,0.,1.],D=[0.]*5,side_m=.08,min_side_px=8)
            for observation in measured['observations']:
                pixel=observation['pixel_observation']
                result.append(dict(source_ns=str(stamp),tag_id=observation['tag_id'],
                    image_sha256=pixel['image']['sha256'],pixel_record_sha256=pixel['sha256'],
                    corner_order=pixel['corner_order'],corners_xy=[v for xy in pixel['corners_xy'] for v in xy]))
        return result
