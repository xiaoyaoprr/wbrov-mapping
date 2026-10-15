"""Three independent pixel-derived links; no fitted graph pose becomes odometry.

Every link uses only its own two original four-corner measurements. Other
views select consistent planar branches; they never change a link's metric
estimate. Weights are conditional on branch, calibration and nominal Tag size,
with 1 px variance at 960-pixel reference width, not calibrated sensor noise.
"""
from itertools import combinations, product
import math
import cv2
import numpy as np

INFORMATION_MAX=1e6

B=np.eye(4);B[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
BI=np.linalg.inv(B)

def matrix(row):
    x,y,z,w=np.asarray(row['quaternion_xyzw'],float)/np.linalg.norm(row['quaternion_xyzw'])
    T=np.eye(4);T[:3,3]=row['translation']
    T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
              [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
              [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    return T

def distance(a,b):
    angle=np.linalg.norm(cv2.Rodrigues(a[:3,:3].T@b[:3,:3])[0])*180/np.pi
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(angle)

def tilt(a,b):
    za=a[:3,:3].T@np.array([0.,0.,1.]);zb=b[:3,:3].T@np.array([0.,0.,1.])
    return float(np.arccos(np.clip(za@zb,-1.,1.))*180/np.pi)

def independent(rows):
    return all((distance(a['body'],b['body'])[0]>=.03 or distance(a['body'],b['body'])[1]>=3.)
               for a,b in combinations(rows,2))

def agreement(ts):
    return all(distance(a,b)[0]<=.08 and distance(a,b)[1]<=8. and
               abs(a[2,3]-b[2,3])<=.02 and tilt(a,b)<=2. for a,b in combinations(ts,2))

def pose(rv,tv):
    T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=np.asarray(tv).ravel();return T

def refine(objects,uv,K,D,rv,tv):
    ok,r,t=cv2.solvePnP(objects,uv,K,D,np.asarray(rv).copy(),np.asarray(tv).copy(),True,cv2.SOLVEPNP_ITERATIVE)
    if not ok:raise ValueError('PnP refinement failed')
    return r,t

def observation(binding,body):
    p=binding['pixel_observation'];side=float(p['side_m']);s=side/2
    objects=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],float)
    uv=np.array(p['corners_xy'],float);K=np.array(p['camera']['K'],float).reshape(3,3);D=np.array(p['camera']['D'],float)
    scale=960./p['image']['width'];res=cv2.solvePnPGeneric(objects,uv,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    branches=[]
    if res[0]:
        for rv,tv in zip(res[1],res[2]):
            rv,tv=refine(objects,uv,K,D,rv,tv);T=pose(rv,tv)
            xyz=objects@T[:3,:3].T+T[:3,3]
            projected=cv2.projectPoints(objects,rv,tv,K,D)[0].reshape(4,2)
            rms=float(np.sqrt(np.mean(np.sum(((projected-uv)*scale)**2,axis=1))))
            if np.min(xyz[:,2])<=.01 or not np.isfinite(T).all() or rms>3:continue
            if any(distance(T,b['camera_tag'])[0]<1e-6 and distance(T,b['camera_tag'])[1]<1e-4 for b in branches):continue
            branches.append(dict(camera_tag=T,body_tag=B@T,rv=rv,tv=tv,rms=rms))
    return dict(id=binding['keyframe_id'],tag_id=binding['tag_id'],stamp=int(binding['image_stamp_ns']),
                epoch=int(binding['reset_epoch']),body=matrix(body),binding=binding,objects=objects,uv=uv,K=K,D=D,
                scale=scale,area=float(abs(cv2.contourArea(uv.astype(np.float32)))),branches=branches)

def best_old_triplet(rows):
    # Select reference geometry solely from the completed old component.
    # Prefer well-resolved quads; deterministic stamp tie-break, no future input.
    eligible=sorted((r for r in rows if r['branches']),key=lambda r:(-r['area'],r['stamp']))[:24]
    for triplet in combinations(eligible,3):
        if not independent(triplet):continue
        modes=[]
        for bs in product(*(r['branches'] for r in triplet)):
            tag_world=[r['body']@b['body_tag'] for r,b in zip(triplet,bs)]
            if agreement(tag_world):modes.append(bs)
        if modes:return tuple(triplet),modes
    return None

def relative(current_branch,old_branch):
    return current_branch['body_tag']@np.linalg.inv(old_branch['body_tag'])

def rpy_error(T):
    R=T[:3,:3];return np.r_[T[:3,3],math.atan2(R[2,1],R[2,2]),
        math.atan2(-R[2,0],math.hypot(R[0,0],R[1,0])),math.atan2(R[1,0],R[0,0])]

def information(current,old,cb,ob,epsilon=.001):
    nominal=relative(cb,ob);inverse=np.linalg.inv(nominal);columns=[]
    for row,branch,is_current in ((current,cb,True),(old,ob,False)):
        for dim in range(8):
            vals=[]
            for sign in (-1,1):
                uv=row['uv'].copy();uv.flat[dim]+=sign*epsilon
                rv,tv=refine(row['objects'],uv,row['K'],row['D'],branch['rv'],branch['tv'])
                changed=dict(body_tag=B@pose(rv,tv));delta=distance(changed['body_tag'],branch['body_tag'])
                if delta[0]>.02 or delta[1]>8.:raise ValueError('local covariance changed planar branch')
                link=relative(changed,ob) if is_current else relative(cb,changed)
                vals.append(rpy_error(inverse@link))
            columns.append((vals[1]-vals[0])/(2*epsilon)/row['scale'])
    J=np.array(columns).T;cov=J@J.T;cov=(cov+cov.T)/2
    eigen=np.linalg.eigvalsh(cov)
    if not np.isfinite(cov).all() or eigen[0]<=0 or eigen[-1]/eigen[0]>1e8:raise ValueError('conditional covariance singular')
    raw=np.linalg.inv(cov);raw=(raw+raw.T)/2
    values,vectors=np.linalg.eigh(raw)
    # The Core caps the largest information eigenvalue at 1e6. Remove excess
    # information conservatively, never amplify any measurement direction.
    # A tiny numerical margin avoids round-trip eigensolver overshoot.
    bounded=np.minimum(values,INFORMATION_MAX*(1.-1e-9))
    H=(vectors*bounded)@vectors.T;H=(H+H.T)/2
    return H,dict(conditional_covariance=cov.tolist(),condition=float(eigen[-1]/eigen[0]),
                  raw_information_row_major=raw.ravel().tolist(),raw_information_eigenvalues=values.tolist(),
                  effective_information_eigenvalues=np.linalg.eigvalsh(H).tolist(),
                  information_eigenvalue_cap=INFORMATION_MAX,information_only_reduced=True,
                  uncertainty_policy='conditional_pixel_covariance_with_Core_information_cap',
                  pixel_sigma_at_960=1.,nominal_tag_side_m=float(current['binding']['pixel_observation']['side_m']),
                  measurement_dimensions=16,coordinate='nominal_link_inverse_times_perturbed_link_xyz_rpy',
                  calibration_and_size_uncertainty_included=False,physical_covariance=False)

def propose(old_reference,current_rows):
    if old_reference is None:return None,'no_independent_consistent_old_triplet'
    olds,old_modes=old_reference
    now=max(r['stamp'] for r in current_rows)
    recent=[r for r in current_rows if r['branches'] and now-r['stamp']<=8*10**9][-12:]
    # Require the newest measured image; no repeated proposal from unchanged data.
    if len(recent)<3 or recent[-1]['stamp']!=now:return None,'fewer_than_three_current_views'
    ranked=[];ambiguous=False
    for pair in combinations(recent[:-1],2):
        currents=(*pair,recent[-1])
        if not independent(currents):continue
        options=[]
        for ob in old_modes:
            for cb in product(*(r['branches'] for r in currents)):
                links=[relative(c,o) for c,o in zip(cb,ob)]
                alignments=[o['body']@np.linalg.inv(l)@np.linalg.inv(c['body']) for c,o,l in zip(currents,olds,links)]
                if agreement(alignments):
                    pairwise=[distance(a,b) for a,b in combinations(alignments,2)]
                    score=(max(v[0] for v in pairwise),max(v[1] for v in pairwise),
                           max(abs(a[2,3]-b[2,3]) for a,b in combinations(alignments,2)),
                           max(tilt(a,b) for a,b in combinations(alignments,2)))
                    options.append((score,sum(b['rms']**2 for b in (*ob,*cb)),cb,ob,links,alignments))
        if not options:continue
        options.sort(key=lambda x:(x[0],x[1]));best=options[0]
        if any(distance(best[5][0],v[5][0])[0]>.01 or distance(best[5][0],v[5][0])[1]>1. for v in options[1:]):
            ambiguous=True;continue
        ranked.append((best[0],best[1],tuple(r['id'] for r in currents),currents,best))
    if ranked:
        # The old implementation returned the first valid combination in
        # timestamp order. That arbitrary order hid a substantially more
        # consistent three-view set in the recorded loss. Rank every bounded
        # combination before asking Core to reopen and validate metric depth.
        ranked.sort(key=lambda x:(x[0],x[1],x[2]));_,_,_,currents,best=ranked[0]
        evidence=[];weights=[]
        for c,o,cb,ob,l in zip(currents,olds,best[2],best[3],best[4]):
            H,diagnostic=information(c,o,cb,ob)
            # Quaternion from OpenCV Rodrigues via a stable rotation-vector formula.
            rv=cv2.Rodrigues(l[:3,:3])[0].ravel();angle=np.linalg.norm(rv)
            q=np.r_[rv*(math.sin(angle/2)/angle if angle>1e-12 else .5),math.cos(angle/2)]
            evidence.append(dict(from_id=c['id'],to_id=o['id'],translation=l[:3,3].tolist(),quaternion_xyzw=q.tolist(),information_row_major=H.ravel().tolist()))
            weights.append(diagnostic)
        return dict(evidence=evidence,weights=weights,old_ids=[r['id'] for r in olds],current_ids=[r['id'] for r in currents],
                    old_stamps=[r['stamp'] for r in olds],current_stamps=[r['stamp'] for r in currents],
                    corner_rms_at_960=[b['rms'] for b in (*best[3],*best[2])],
                    alignment_consistency=dict(max_translation_m=best[0][0],max_rotation_deg=best[0][1],
                                               max_z_m=best[0][2],max_tilt_deg=best[0][3]),
                    link_measurements_use_only_own_endpoint_pixels=True,shared_fit_used=False), 'three_independent_measured_links'
    if ambiguous:return None,'multiple_consistent_planar_branches'
    return None,'three_view_alignment_disagreement_or_baseline'
