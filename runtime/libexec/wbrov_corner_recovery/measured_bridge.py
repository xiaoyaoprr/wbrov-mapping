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

def rigid(value):
    T=np.asarray(value,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-9,rtol=0):
        raise ValueError('invalid finite rigid alignment/pose')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-6,rtol=0) or abs(np.linalg.det(R)-1)>1e-6:
        raise ValueError('nonrigid alignment/pose')
    return T

def body_support_points(poses):
    if len(poses)!=3:raise ValueError('three observed body poses required')
    return np.stack([rigid(p)[:,3] for p in poses],axis=1)

def alignment_metrics(alignments,points):
    """Compare actions on ALL observed positions, not on an arbitrary origin.

    For a source-coordinate change D, A'=A D^-1 and P'=D P, so A'P'=AP.
    No graph pose, measurement, link, or alignment is fitted or modified here.
    Destination z is the existing old-map gravity axis; yaw/translation of
    that frame leaves these tests unchanged. An arbitrary destination tilt
    would also require transforming gravity and is not this frame contract.
    """
    if len(alignments)<2:raise ValueError('at least two alignments required')
    points=np.asarray(points,dtype=float)
    if points.shape!=(4,3) or not np.isfinite(points).all() or not np.allclose(points[3],1,atol=1e-9,rtol=0):
        raise ValueError('invalid observed support points')
    transforms=[rigid(a) for a in alignments];actions=[(a@points)[:3] for a in transforms]
    if not all(np.isfinite(p).all() for p in actions):raise ValueError('nonfinite alignment action')
    deltas=[a-b for a,b in combinations(actions,2)]
    result=dict(max_translation_m=max(float(np.max(np.linalg.norm(d,axis=0))) for d in deltas),
        max_rotation_deg=max(distance(a,b)[1] for a,b in combinations(transforms,2)),
        max_z_m=max(float(np.max(np.abs(d[2]))) for d in deltas),
        max_tilt_deg=max(tilt(a,b) for a,b in combinations(transforms,2)))
    if not all(math.isfinite(v) for v in result.values()):raise ValueError('nonfinite alignment metric')
    return result

def normalized_alignment(metrics):
    return tuple(metrics[k]/bound for k,bound in (
        ('max_translation_m',.08),('max_rotation_deg',8.),('max_z_m',.02),('max_tilt_deg',2.)))

def alignment_passes(metrics):
    return all(math.isfinite(v) and 0<=v<=1 for v in normalized_alignment(metrics))

def alignment_ambiguous(a,b,points):
    metric=alignment_metrics([a,b],points)
    return metric['max_translation_m']>.01 or metric['max_rotation_deg']>1.

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

def reference_reservoir(rows,capacity=24):
    """Keep useful view diversity even after many larger static Tag images."""
    eligible=sorted((r for r in rows if r['branches']),key=lambda r:(-r['area'],r['stamp'],r['id']))
    distinct=[];duplicates=[]
    for row in eligible:
        if all(independent((row,other)) for other in distinct):distinct.append(row)
        else:duplicates.append(row)
    return (distinct+duplicates)[:capacity]

def best_old_triplet(rows):
    # Select reference geometry solely from the completed old component.
    # Prefer well-resolved quads; deterministic stamp tie-break, no future input.
    eligible=reference_reservoir(rows)
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

def proposal_measurements(currents,olds,current_modes,old_modes,links,metric):
    evidence=[];weights=[]
    for c,o,cb,ob,l in zip(currents,olds,current_modes,old_modes,links):
        H,diagnostic=information(c,o,cb,ob)
        # Quaternion from OpenCV Rodrigues via a stable rotation-vector formula.
        rv=cv2.Rodrigues(l[:3,:3])[0].ravel();angle=np.linalg.norm(rv)
        q=np.r_[rv*(math.sin(angle/2)/angle if angle>1e-12 else .5),math.cos(angle/2)]
        evidence.append(dict(from_id=c['id'],to_id=o['id'],translation=l[:3,3].tolist(),quaternion_xyzw=q.tolist(),information_row_major=H.ravel().tolist()))
        weights.append(diagnostic)
    return dict(evidence=evidence,weights=weights,old_ids=[r['id'] for r in olds],current_ids=[r['id'] for r in currents],
                old_stamps=[r['stamp'] for r in olds],current_stamps=[r['stamp'] for r in currents],
                corner_rms_at_960=[b['rms'] for b in (*old_modes,*current_modes)],
                alignment_consistency=metric,
                alignment_evaluation='all_three_current_body_positions',
                alignment_support_body_positions=[r['body'][:3,3].tolist() for r in currents],
                link_measurements_use_only_own_endpoint_pixels=True,shared_fit_used=False)

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
        support=body_support_points([c['body'] for c in currents])
        options=[]
        for ob in old_modes:
            for cb in product(*(r['branches'] for r in currents)):
                links=[relative(c,o) for c,o in zip(cb,ob)]
                alignments=[o['body']@np.linalg.inv(l)@np.linalg.inv(c['body']) for c,o,l in zip(currents,olds,links)]
                metric=alignment_metrics(alignments,support)
                if alignment_passes(metric):
                    score=tuple(metric[k] for k in ('max_translation_m','max_rotation_deg','max_z_m','max_tilt_deg'))
                    options.append((score,sum(b['rms']**2 for b in (*ob,*cb)),cb,ob,links,alignments))
        if not options:continue
        options.sort(key=lambda x:(x[0],x[1]));best=options[0]
        if any(alignment_ambiguous(best[5][0],v[5][0],support) for v in options[1:]):
            ambiguous=True;continue
        ranked.append((best[0],best[1],tuple(r['id'] for r in currents),currents,best))
    if ranked:
        # The old implementation returned the first valid combination in
        # timestamp order. That arbitrary order hid a substantially more
        # consistent three-view set in the recorded loss. Rank every bounded
        # combination before asking Core to reopen and validate metric depth.
        ranked.sort(key=lambda x:(x[0],x[1],x[2]));_,_,_,currents,best=ranked[0]
        metric=dict(zip(('max_translation_m','max_rotation_deg','max_z_m','max_tilt_deg'),best[0]))
        return proposal_measurements(currents,olds,best[2],best[3],best[4],metric), 'three_independent_measured_links'
    if ambiguous:return None,'multiple_consistent_planar_branches'
    return None,'three_view_alignment_disagreement_or_baseline'


def propose_depth_candidate(old_reference,current_rows):
    """Select endpoints for Core depth validation; NEVER certify a PnP alignment.

    The schema-3 bridge fields remain individual pixel estimates for protocol
    compatibility. Core discards these transforms/information and rebuilds
    metric links from its owned RGB-D corners before any prepare or commit.
    Planar pose disagreement is diagnostic here; depth/holdout/IMU and all
    subsequent observations still have to pass in Core.
    """
    if old_reference is None or not current_rows:return None,'no_depth_candidate_reference'
    olds,old_modes=old_reference
    now=max(r['stamp'] for r in current_rows)
    recent=sorted((r for r in current_rows if r['branches'] and now-r['stamp']<=8*10**9),
                  key=lambda r:(r['stamp'],r['id']))[-12:]
    if len(recent)<3 or recent[-1]['stamp']!=now:return None,'fewer_than_three_current_views'
    if len(olds)!=3 or not old_modes or not independent(olds):return None,'invalid_depth_candidate_reference'
    ranked=[]
    for pair in combinations(recent[:-1],2):
        currents=(*pair,recent[-1]);all_rows=(*olds,*currents)
        if len({r['id'] for r in all_rows})!=6 or len({r['tag_id'] for r in all_rows})!=1:continue
        if len({r['epoch'] for r in olds})!=1 or len({r['epoch'] for r in currents})!=1 or olds[0]['epoch']>=currents[0]['epoch']:continue
        if any(len({r['stamp'] for r in side})!=3 or max(r['stamp'] for r in side)-min(r['stamp'] for r in side)<200000000 for side in (olds,currents)):continue
        if not independent(currents):continue
        # Deterministic geometry-independent scheduling. Do not search for a
        # PnP branch that merely happens to satisfy an alignment threshold.
        ranked.append((currents[-1]['stamp']-currents[0]['stamp'],
                       -min(r['area'] for r in currents),tuple(r['id'] for r in currents),currents))
    if not ranked:return None,'no_independent_depth_candidate'
    ranked.sort(key=lambda x:x[:3]);currents=ranked[0][3]
    cb=tuple(min(r['branches'],key=lambda b:b['rms']) for r in currents)
    ob=min(old_modes,key=lambda branches:sum(b['rms']**2 for b in branches))
    links=[relative(c,o) for c,o in zip(cb,ob)]
    aligns=[o['body']@np.linalg.inv(l)@np.linalg.inv(c['body']) for c,o,l in zip(currents,olds,links)]
    metric=alignment_metrics(aligns,body_support_points([r['body'] for r in currents]))
    result=proposal_measurements(currents,olds,cb,ob,links,metric)
    result.update(candidate_only=True,requires_core_depth_verification=True,
                  pnp_alignment_certified=False,selection='independent_recent_endpoints_for_depth_verification')
    return result,'three_independent_tag_views_for_core_depth_verification'
