"""Decoded measured corners, independent of any single-frame PnP branch."""
from collections import Counter
import math
from tag_pixel_record import PixelRecordError, image_record, make_pixel_record

PRODUCER = 'wbrov_apriltag_decoded_pixel_observer'

def decoded_pixel_metadata(stamp_ns,camera_frame,tag_ids,corners,*,width,height,
                           encoding,pixels,K,D,side_m,min_side_px):
    if (not isinstance(tag_ids,list) or not isinstance(corners,list) or
            len(tag_ids)!=len(corners) or
            any(type(i) is not int or i<0 for i in tag_ids)):
        raise PixelRecordError('invalid decoded ID/corner arrays')
    if type(min_side_px) not in (int,float) or not math.isfinite(min_side_px) or min_side_px<=0:
        raise PixelRecordError('invalid minimum observed side')
    output=dict(schema_version=1,producer=PRODUCER,image_stamp_ns=stamp_ns,
        camera_frame=camera_frame,observations=[],rejections=[],
        pose_estimate_provided=False,graph_update_authorized=False,
        requires_multiview_validation=True)
    image=image_record(stamp_ns,camera_frame,width,height,encoding,pixels)
    counts=Counter(tag_ids)
    for tag_id,points in zip(tag_ids,corners):
        if counts[tag_id]!=1:
            output['rejections'].append(dict(tag_id=tag_id,reason='duplicate_tag_id'))
            continue
        try:
            record=make_pixel_record(image,tag_id,side_m,points,K,D)
            if min(math.hypot(points[(i+1)%4][0]-points[i][0],
                              points[(i+1)%4][1]-points[i][1]) for i in range(4))<min_side_px:
                raise PixelRecordError('tag_too_small')
            output['observations'].append(dict(tag_id=tag_id,pixel_observation=record))
        except PixelRecordError as exc:
            output['rejections'].append(dict(tag_id=tag_id,reason=str(exc)))
    return output
