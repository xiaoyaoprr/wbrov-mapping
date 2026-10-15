"""Refine an already decoded quad from image gradients in OpenCV pixel centers.

Mechanism reference: AprilRobotics/apriltag v3.4.3 refine_edges. This private
implementation uses exact vectorized bilinear sampling, retains the original
decoded corner order, and reports explicit failures instead of stale corners.
No depth, metric marker size, trajectory, or target reprojection is an input.
"""
import numpy as np

CONFIG={'search_radius_px':2.,'step_px':.25,'gradient_half_width_px':1.,
        'minimum_samples_per_edge':16,'minimum_valid_fraction':.75,
        'maximum_line_residual_p95_px':.75,'maximum_corner_shift_px':2.5,
        'minimum_side_px':18.,'minimum_intersection_determinant':.1,
        'pixel_centers':'integer OpenCV coordinates; no fitted half-pixel offset'}

def bilinear(gray,xy):
    x=xy[...,0];y=xy[...,1]
    ix=np.floor(x).astype(int);iy=np.floor(y).astype(int)
    valid=(ix>=0)&(iy>=0)&(ix+1<gray.shape[1])&(iy+1<gray.shape[0])
    ix=np.clip(ix,0,gray.shape[1]-2);iy=np.clip(iy,0,gray.shape[0]-2)
    a=x-ix;b=y-iy
    value=(1-a)*(1-b)*gray[iy,ix]+a*(1-b)*gray[iy,ix+1]+(1-a)*b*gray[iy+1,ix]+a*b*gray[iy+1,ix+1]
    return value,valid

def refine_quad(gray,corners):
    original=np.asarray(corners,dtype=float)
    if gray.ndim!=2 or gray.dtype!=np.uint8 or original.shape!=(4,2) or not np.isfinite(original).all():
        return {'accepted':False,'reason':'invalid_input'}
    vectors=np.roll(original,-1,axis=0)-original
    cross=vectors[:,0]*np.roll(vectors,-1,axis=0)[:,1]-vectors[:,1]*np.roll(vectors,-1,axis=0)[:,0]
    if np.min(np.linalg.norm(vectors,axis=1))<18 or np.min(cross)<=0:
        return {'accepted':False,'reason':'insufficient_or_reversed_quad'}
    center=original.mean(axis=0);lines=[];diagnostics=[]
    offsets=np.arange(-2.,2.0001,.25)
    for edge,(a,b) in enumerate(zip(original,np.roll(original,-1,axis=0))):
        length=np.linalg.norm(b-a);normal=np.array([b[1]-a[1],a[0]-b[0]])/length
        if normal@(center-(a+b)/2)>0:normal=-normal
        count=max(16,int(length/8));alpha=(1+np.arange(count))/(count+1)
        along=a[None]+alpha[:,None]*(b-a)[None]
        outside=along[:,None,:]+(offsets[None,:,None]+1)*normal[None,None,:]
        inside=along[:,None,:]+(offsets[None,:,None]-1)*normal[None,None,:]
        high,v1=bilinear(gray,outside);low,v2=bilinear(gray,inside)
        gradient=np.maximum(high-low,0);weights=gradient**2*(v1&v2)
        total=weights.sum(axis=1);valid=total>1e-8
        if valid.sum()<max(4,int(np.ceil(.75*count))):
            return {'accepted':False,'reason':'insufficient_positive_edge_gradient','edge':edge,'valid_samples':int(valid.sum())}
        shift=(weights[valid]@offsets)/total[valid]
        points=along[valid]+shift[:,None]*normal
        centroid=points.mean(axis=0);_,singular,vt=np.linalg.svd(points-centroid,full_matrices=False)
        fitted_normal=vt[-1]
        residual=np.abs((points-centroid)@fitted_normal);p95=float(np.percentile(residual,95))
        if p95>.75:return {'accepted':False,'reason':'nonlinear_or_noisy_edge','edge':edge,'residual_p95_px':p95}
        lines.append((centroid,fitted_normal))
        diagnostics.append({'edge':edge,'samples':count,'valid_samples':int(valid.sum()),'sample_points':points.tolist(),
                            'sample_offset_px':shift.tolist(),'sample_gradient_energy':total[valid].tolist(),
                            'residual_p95_px':p95,'centroid':centroid.tolist(),'normal':fitted_normal.tolist(),
                            'line_singular_values_px':singular.tolist()})
    refined=[]
    for i in range(4):
        a,na=lines[(i-1)%4];b,nb=lines[i];A=np.array([na,nb])
        if abs(np.linalg.det(A))<.1:return {'accepted':False,'reason':'ill_conditioned_line_intersection','corner':i}
        refined.append(np.linalg.solve(A,np.array([na@a,nb@b])))
    refined=np.array(refined);change=np.linalg.norm(refined-original,axis=1)
    if np.max(change)>2.5:return {'accepted':False,'reason':'excessive_corner_change','corner_shift_px':change.tolist()}
    vectors=np.roll(refined,-1,axis=0)-refined
    cross=vectors[:,0]*np.roll(vectors,-1,axis=0)[:,1]-vectors[:,1]*np.roll(vectors,-1,axis=0)[:,0]
    if np.min(cross)<=0 or np.min(np.linalg.norm(vectors,axis=1))<18:
        return {'accepted':False,'reason':'refined_quad_invalid'}
    if np.any(refined<0) or np.any(refined[:,0]>gray.shape[1]-1) or np.any(refined[:,1]>gray.shape[0]-1):
        return {'accepted':False,'reason':'refined_quad_outside_image'}
    return {'accepted':True,'reason':'image_edges_refined','corners':refined.tolist(),
            'corner_shift_px':change.tolist(),'edges':diagnostics}
