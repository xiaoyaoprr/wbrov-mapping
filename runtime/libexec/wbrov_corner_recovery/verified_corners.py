"""Image-only corner refinement with post-refinement code/order validation.

Original OpenCV 4.5.4 ArUco decoding limits are retained. Prefer bounded edge
intersections from the unrefined contour; use module-sized SubPix only when
the edge proposal fails. Never return a proposal that changes the code/order.
No depth, trajectory, PnP root, physical Tag size, or target pose is an input.
"""
import cv2,numpy as np
from edge_refinement import refine_quad

class VerifiedCorners:
    def __init__(self):
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.codebook=np.stack([(cv2.aruco.drawMarker(self.dictionary,i,8)[1:7,1:7]>0).astype(np.uint8)
                               for i in range(len(self.dictionary.bytesList))])

    def decode(self,gray,corners):
        # _extractBits / _identifyOneCandidate at opencv_contrib 4.5.4.
        dst=np.array([[0,0],[31,0],[31,31],[0,31]],np.float32)
        H=cv2.getPerspectiveTransform(np.asarray(corners,np.float32),dst)
        patch=cv2.warpPerspective(gray,H,(32,32),flags=cv2.INTER_NEAREST)
        if np.std(patch[2:-2,2:-2])<5:return {'accepted':False,'reason':'insufficient_contrast'}
        _,binary=cv2.threshold(patch,125,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
        bits=(binary.reshape(8,4,8,4).sum(axis=(1,3))>8*255).astype(np.uint8)
        border=int(bits[0].sum()+bits[-1].sum()+bits[1:-1,0].sum()+bits[1:-1,-1].sum())
        distances=np.stack([np.count_nonzero(self.codebook!=np.rot90(bits[1:-1,1:-1],k)[None],axis=(1,2)) for k in range(4)])
        matches=np.argwhere(distances<=int(self.dictionary.maxCorrectionBits*.6))
        if len(matches)!=1:return {'accepted':False,'reason':'code_not_unique_within_original_hamming_limit',
                                   'matches':len(matches),'border_errors':border}
        rotation,tag=map(int,matches[0])
        return {'accepted':bool(border<=int(36*.35)),'tag_id':tag,'rotation':rotation,
                'border_errors':border,'hamming_distance':int(distances[rotation,tag])}

    def refine(self,gray,raw,tag_id):
        p=np.asarray(raw,dtype=float)
        if gray.ndim!=2 or gray.dtype!=np.uint8 or p.shape!=(4,2) or not np.isfinite(p).all():
            return {'accepted':False,'reason':'invalid_input'}
        vectors=np.roll(p,-1,axis=0)-p
        cross=vectors[:,0]*np.roll(vectors,-1,axis=0)[:,1]-vectors[:,1]*np.roll(vectors,-1,axis=0)[:,0]
        if np.min(cross)<=0 or np.min(np.linalg.norm(vectors,axis=1))<18:
            return {'accepted':False,'reason':'insufficient_or_reversed_quad'}
        if np.any(p<0) or np.any(p[:,0]>gray.shape[1]-1) or np.any(p[:,1]>gray.shape[0]-1):
            return {'accepted':False,'reason':'quad_outside_image'}
        same=lambda d:bool(d.get('accepted') and d.get('tag_id')==int(tag_id) and d.get('rotation')==0)
        raw_code=self.decode(gray,p)
        if not same(raw_code):return {'accepted':False,'reason':'raw_code_or_order_mismatch','raw_decode':raw_code}
        edge=refine_quad(gray,p);attempts=[{'method':'raw_edges','edge':{k:v for k,v in edge.items() if k!='edges'}}]
        if edge['accepted']:
            code=self.decode(gray,edge['corners']);attempts[-1]['decode']=code
            if same(code):return {'accepted':True,'method':'raw_edges','corners_px':edge['corners'],
                                  'decode':code,'raw_decode':raw_code,'attempts':attempts}
        module=float(np.mean(np.linalg.norm(vectors,axis=1)))/8.
        window=max(1,min(5,int(round(module*.3))))
        scaled=p.astype(np.float32)
        cv2.cornerSubPix(gray,scaled,(window,window),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
        code=self.decode(gray,scaled)
        shift=float(np.max(np.linalg.norm(scaled-p,axis=1)))
        v=np.roll(scaled,-1,axis=0)-scaled
        convex=bool(np.min(v[:,0]*np.roll(v,-1,axis=0)[:,1]-v[:,1]*np.roll(v,-1,axis=0)[:,0])>0)
        attempts.append({'method':'module_subpix','window_px':window,'decode':code,'max_corner_shift_px':shift})
        inside=bool(np.isfinite(scaled).all() and np.min(scaled)>=0 and
                    np.max(scaled[:,0])<=gray.shape[1]-1 and np.max(scaled[:,1])<=gray.shape[0]-1)
        if same(code) and convex and inside and shift<=2.5 and np.min(np.linalg.norm(v,axis=1))>=18:
            return {'accepted':True,'method':'module_subpix','corners_px':scaled.astype(float).tolist(),
                    'decode':code,'raw_decode':raw_code,'attempts':attempts}
        return {'accepted':False,'reason':'no_verified_refinement','raw_decode':raw_code,'attempts':attempts}
