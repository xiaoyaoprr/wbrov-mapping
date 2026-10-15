"""Measured corners verified against image edges and decoded code/order.

This independent observer never publishes a pose or authorizes a graph update.
The original single-image pose channel is evaluated separately and unchanged.
"""
from collections import Counter
import cv2,numpy as np
from decoded_pixel_metadata import decoded_pixel_metadata
from verified_corners import VerifiedCorners

class VerifiedPixelObserver:
    def __init__(self):
        self.refiner=VerifiedCorners()
        self.parameters=cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_NONE

    def observe(self,bgr,*,stamp_ns,camera_frame,K,D,side_m,min_side_px):
        if not isinstance(bgr,np.ndarray) or bgr.dtype!=np.uint8 or bgr.ndim not in (2,3):
            raise ValueError('Expected canonical mono8 or BGR8 pixels')
        if bgr.ndim==3 and bgr.shape[2]!=3:raise ValueError('Expected three BGR channels')
        gray=bgr if bgr.ndim==2 else cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        corners,ids,_=cv2.aruco.detectMarkers(gray,self.refiner.dictionary,parameters=self.parameters)
        ids=[] if ids is None else list(map(int,ids.flatten()))
        counts=Counter(ids);accepted_ids=[];accepted_corners=[];details=[];rejected=[]
        for raw,tag in zip(corners,ids):
            if counts[tag]!=1:
                rejected.append({'tag_id':tag,'reason':'duplicate_tag_id'});continue
            raw=np.asarray(raw).reshape(4,2)
            result=self.refiner.refine(gray,raw,tag)
            details.append({'tag_id':tag,'raw_corners_xy':raw.astype(float).tolist(),'refinement':result})
            if result['accepted']:
                accepted_ids.append(tag);accepted_corners.append(result['corners_px'])
            else:rejected.append({'tag_id':tag,'reason':result['reason']})
        payload=decoded_pixel_metadata(stamp_ns,camera_frame,accepted_ids,accepted_corners,
            width=int(bgr.shape[1]),height=int(bgr.shape[0]),encoding='mono8' if bgr.ndim==2 else 'bgr8',
            pixels=bgr.tobytes(order='C'),K=K,D=D,side_m=side_m,min_side_px=min_side_px)
        payload['rejections'].extend(rejected)
        payload['corner_measurement_method']='verified_image_edges_or_bounded_module_subpix_v1'
        payload['refinement_details']=details
        return payload
