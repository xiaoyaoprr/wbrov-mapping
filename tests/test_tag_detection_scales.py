import hashlib
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).parents[1]/'runtime/libexec/wbrov_corner_recovery'))
from verified_pixel_observer import VerifiedPixelObserver


class DetectionScaleTests(unittest.TestCase):
    def setUp(self):
        self.observer=VerifiedPixelObserver()
        self.detect=cv2.aruco.detectMarkers

    def scene(self,locations=((600,250),),shape=(601,961)):
        frame=np.full(shape,255,np.uint8)
        marker=cv2.aruco.drawMarker(self.observer.refiner.dictionary,13,120)
        for x,y in locations:frame[y:y+120,x:x+120]=marker
        return frame

    def observe(self,frame):
        return self.observer.observe(frame,stamp_ns=123456789,camera_frame='camera',
            K=[500.,0.,480.,0.,500.,300.,0.,0.,1.],D=[0.]*5,side_m=.08,min_side_px=8)

    def test_same_tag_at_two_scales_is_one_original_pixel_observation(self):
        frame=self.scene();result=self.observe(frame)
        self.assertEqual([x['tag_id'] for x in result['observations']],[13])
        pixel=result['observations'][0]['pixel_observation']
        self.assertEqual(pixel['image']['sha256'],hashlib.sha256(frame.tobytes()).hexdigest())
        self.assertEqual((pixel['image']['width'],pixel['image']['height']),(961,601))
        self.assertFalse(result['graph_update_authorized'])
        self.assertFalse(result['pose_estimate_provided'])

    def test_full_resolution_contour_miss_recovers_only_after_original_verification(self):
        frame=self.scene()
        def missed(image,*args,**kwargs):
            return ([],None,[]) if image.shape==frame.shape else self.detect(image,*args,**kwargs)
        with patch.object(cv2.aruco,'detectMarkers',side_effect=missed):result=self.observe(frame)
        self.assertEqual([x['tag_id'] for x in result['observations']],[13])
        detail=result['refinement_details'][0]
        self.assertEqual(detail['detection_scale'],.5)
        self.assertEqual(detail['refinement']['decode']['hamming_distance'],0)
        corners=np.asarray(result['observations'][0]['pixel_observation']['corners_xy'])
        expected=np.array([[600,250],[719,250],[719,369],[600,369]])
        self.assertLess(np.max(np.linalg.norm(corners-expected,axis=1)),2.)

    def test_coarse_wrong_id_cannot_authorize_original_pixels(self):
        frame=self.scene()
        def forged(image,*args,**kwargs):
            if image.shape==frame.shape:return [],None,[]
            corners,ids,rejected=self.detect(image,*args,**kwargs)
            return corners,ids+1,rejected
        with patch.object(cv2.aruco,'detectMarkers',side_effect=forged):result=self.observe(frame)
        self.assertEqual(result['observations'],[])
        self.assertEqual(result['rejections'][0]['reason'],'raw_code_or_order_mismatch')

    def test_duplicate_physical_tags_remain_rejected(self):
        result=self.observe(self.scene(((150,250),(600,250))))
        self.assertEqual(result['observations'],[])
        self.assertTrue(result['rejections'])
        self.assertTrue(all(x['reason']=='duplicate_tag_id' for x in result['rejections']))

    def test_coarse_detection_of_second_physical_tag_also_rejects_duplicate(self):
        frame=self.scene(((150,250),(600,250)))
        def hidden(image,*args,**kwargs):
            corners,ids,rejected=self.detect(image,*args,**kwargs)
            if image.shape==frame.shape:return corners[:1],ids[:1],rejected
            return corners,ids,rejected
        with patch.object(cv2.aruco,'detectMarkers',side_effect=hidden):result=self.observe(frame)
        self.assertEqual(result['observations'],[])
        self.assertTrue(all(x['reason']=='duplicate_tag_id' for x in result['rejections']))

    def test_blank_image_has_no_observations(self):
        result=self.observe(np.full((600,960),127,np.uint8))
        self.assertEqual(result['observations'],[])
        self.assertEqual(result['rejections'],[])


if __name__=='__main__':unittest.main()
