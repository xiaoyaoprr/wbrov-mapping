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

    def candidate_groups(self,gray):
        # A coarse image may expose a contour missed by the fixed adaptive
        # threshold window at full resolution. It supplies seeds only: code,
        # ordering, refinement, source hash and camera geometry use the original
        # image. No confidence or corner-displacement limit is relaxed.
        groups=[]
        for scale in (1.,.5):
            if scale==1.:
                sample=gray
            else:
                if min(gray.shape)<64:continue
                sample=cv2.resize(gray,(gray.shape[1]//2,gray.shape[0]//2),interpolation=cv2.INTER_AREA)
            corners,ids,_=cv2.aruco.detectMarkers(sample,self.refiner.dictionary,parameters=self.parameters)
            if ids is None:continue
            ratio=np.array([gray.shape[1]/sample.shape[1],gray.shape[0]/sample.shape[0]])
            for raw,tag in zip(corners,map(int,ids.flatten())):
                raw=(np.asarray(raw,dtype=float).reshape(4,2)+.5)*ratio-.5
                seed={'scale':scale,'corners':raw}
                matches=[]
                if scale!=1.:
                    for group in groups:
                        if group['tag_id']!=tag or any(s['scale']==scale for s in group['seeds']):continue
                        original=group['seeds'][0]['corners']
                        side=float(np.min(np.linalg.norm(np.roll(original,-1,axis=0)-original,axis=1)))
                        if np.max(np.linalg.norm(original-raw,axis=1))<=max(3.,.05*side):matches.append(group)
                if len(matches)==1:matches[0]['seeds'].append(seed)
                else:groups.append({'tag_id':tag,'seeds':[seed]})
        return groups

    def observe(self,bgr,*,stamp_ns,camera_frame,K,D,side_m,min_side_px):
        if not isinstance(bgr,np.ndarray) or bgr.dtype!=np.uint8 or bgr.ndim not in (2,3):
            raise ValueError('Expected canonical mono8 or BGR8 pixels')
        if bgr.ndim==3 and bgr.shape[2]!=3:raise ValueError('Expected three BGR channels')
        gray=bgr if bgr.ndim==2 else cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        groups=self.candidate_groups(gray)
        counts=Counter(g['tag_id'] for g in groups);accepted_ids=[];accepted_corners=[];details=[];rejected=[]
        for group in groups:
            tag=group['tag_id']
            if counts[tag]!=1:
                rejected.append({'tag_id':tag,'reason':'duplicate_tag_id'});continue
            trials=[]
            for seed in group['seeds']:
                raw=seed['corners'];result=self.refiner.refine(gray,raw,tag)
                trials.append({'detection_scale':seed['scale'],'raw_corners_xy':raw.tolist(),'refinement':result})
                if result['accepted']:break
            details.append({'tag_id':tag,'raw_corners_xy':raw.tolist(),'refinement':result,
                            'detection_scale':seed['scale'],'seed_trials':trials})
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
