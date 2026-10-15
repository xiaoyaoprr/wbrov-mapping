"""Receipt-bound Tag display geometry. Never a localization or graph-update vote."""
from collections import OrderedDict,defaultdict
from copy import deepcopy
import math,re

FAMILY='tag36h11'
SCHEMA='wbrov.tag-visualization.v1'

def require(ok,why):
    if not ok:raise ValueError(why)

def integer(v,name,minimum=0):
    require(type(v) is int and minimum<=v<=2**63-1,'invalid '+name)
    return v

def stamp(v):
    if type(v) is str:
        require(re.fullmatch('[1-9][0-9]{0,18}',v) is not None,'invalid source stamp')
        v=int(v)
    return integer(v,'source stamp',1)

def finite_vector(v,n,name):
    require(type(v) in (list,tuple) and len(v)==n and all(type(x) in (int,float) and math.isfinite(x) for x in v),'invalid '+name)
    return [float(x) for x in v]

def pose_matrix(row):
    t=finite_vector(row['translation'],3,'node position')
    q=finite_vector(row['quaternion_xyzw'],4,'node rotation');norm=sum(x*x for x in q)
    require(abs(norm-1)<.002,'nonunit node rotation')
    x,y,z,w=[v/math.sqrt(norm) for v in q]
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),t[0]],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),t[1]],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),t[2]]]

def extrinsic_matrix(values):
    v=finite_vector(values,12,'body_from_camera');m=[v[i:i+4] for i in (0,4,8)]
    for i in range(3):
        for j in range(3):
            require(abs(sum(m[i][k]*m[j][k] for k in range(3))-(i==j))<1e-5,'nonrigid camera extrinsic')
    det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
    require(abs(det-1)<1e-5,'reflected camera extrinsic')
    return m

def transform(matrix,point):
    return [sum(matrix[i][k]*point[k] for k in range(3))+matrix[i][3] for i in range(3)]

def center(corners):return [sum(p[k] for p in corners)/4 for k in range(3)]
def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def estimate_camera_corners(pixel):
    """Use real measured pixels; reject separated planar branches rather than choose one."""
    import cv2
    import numpy as np
    s=float(pixel['side_m'])/2
    objects=np.asarray([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],dtype=float)
    uv=np.asarray(pixel['corners_xy'],dtype=float)
    K=np.asarray(pixel['camera']['K'],dtype=float).reshape(3,3)
    D=np.asarray(pixel['camera']['D'],dtype=float)
    try:solved=cv2.solvePnPGeneric(objects,uv,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:return dict(status='no_valid_planar_pose',corners_camera=None)
    hypotheses=[]
    if solved[0]:
        for rv,tv in zip(solved[1],solved[2]):
            try:ok,rv,tv=cv2.solvePnP(objects,uv,K,D,rv.copy(),tv.copy(),True,cv2.SOLVEPNP_ITERATIVE)
            except cv2.error:continue
            if not ok:continue
            R=cv2.Rodrigues(rv)[0];xyz=objects@R.T+np.asarray(tv).reshape(1,3)
            projected=cv2.projectPoints(objects,rv,tv,K,D)[0].reshape(4,2)
            rms=float(np.sqrt(np.mean(np.sum(((projected-uv)*(960./pixel['image']['width']))**2,axis=1))))
            if not np.isfinite(xyz).all() or np.min(xyz[:,2])<=.01 or not math.isfinite(rms) or rms>3:continue
            hypotheses.append((rms,xyz.tolist()))
    if not hypotheses:return dict(status='no_valid_planar_pose',corners_camera=None)
    spread=max((max(distance(a,b) for a,b in zip(x[1],y[1])) for x in hypotheses for y in hypotheses),default=0.)
    if spread>.02:return dict(status='ambiguous_planar_pose',corners_camera=None,planar_branch_spread_m=spread)
    rms,corners=min(hypotheses,key=lambda x:x[0])
    return dict(status='single_view_position_estimate',corners_camera=corners,
                reprojection_rms_at_960=rms,planar_branch_spread_m=spread)

def registry_index(registry,session):
    require(type(registry) is dict and registry.get('schema')=='wbrov.submaps.v1','registry schema')
    require(registry.get('session_token_sha256')==session and registry.get('session_short_hash')==session[:16],'registry session')
    revision=integer(registry.get('revision'),'registry revision')
    require(type(registry.get('main_frame_id')) is str and bool(registry['main_frame_id']),'registry main frame')
    groups=registry.get('submaps');require(type(groups) is list and len(groups)<=4096,'registry submaps')
    nodes={};keys=set()
    for group in groups:
        require(type(group) is dict,'registry submap object')
        mid=integer(group.get('map_id'),'map ID');epoch=integer(group.get('reset_epoch'),'epoch')
        component=integer(group.get('component_id'),'component',1)
        key=f'm{mid}_e{epoch}_c{component}'
        require(group.get('key')==key and key not in keys,'submap key');keys.add(key)
        require(type(group.get('registered')) is bool and type(group.get('active')) is bool,'submap registration')
        expected=registry['main_frame_id'] if group['registered'] else f'wbrov_submap_{session[:16]}_{key}'
        require(group.get('frame_id')==expected,'submap frame')
        rows=group.get('nodes');require(type(rows) is list and len(rows)<=20000,'submap nodes')
        for row in rows:
            require(type(row) is dict,'node object');node=integer(row.get('id'),'node ID',1)
            require(node not in nodes,'node in multiple submaps')
            node_stamp=stamp(row.get('source_stamp_ns'));matrix=pose_matrix(row)
            nodes[node]=(group,node_stamp,matrix)
        require(len(nodes)<=20000,'registry node capacity')
    return revision,nodes

class TagPlacementStore:
    """Bounded evidence cache keyed by original node/Tag, regrouped by actual component."""
    def __init__(self,config,estimator=estimate_camera_corners):
        self.config=config;self.session=config['session_token_sha256']
        require(type(self.session) is str and re.fullmatch('[0-9a-f]{64}',self.session) is not None,'configured session hash')
        self.body_from_camera=extrinsic_matrix(config['body_from_camera']);self.estimator=estimator
        self.records=OrderedDict();self.conflicted=set();self.dirty=True

    def add(self,binding):
        from core_pixel_join import compare_accepted_image
        require(type(binding) is dict,'bound Tag object')
        receipt=binding['accepted_image_receipt'];pixel=binding['pixel_observation']
        # Repeat the actual image, dimensions, K/D, stamp and extrinsic identity
        # comparison. Callback/source authentication is additionally enforced by
        # the live adapter; this pure value check never grants transport trust.
        compare_accepted_image(receipt,pixel,expected_session_sha256=self.session,
            expected_database_sha256=self.config['database_sha256'],expected_body_from_camera=self.config['body_from_camera'])
        require(binding['keyframe_id']==receipt['node_id'] and binding['tag_id']==pixel['tag_id'] and
                stamp(binding['image_stamp_ns'])==stamp(receipt['header_stamp_ns']) and
                int(binding['reset_epoch'])==int(receipt['reset_epoch']),'binding identity mismatch')
        proof=binding['transport_proof']
        require(proof.get('core_receipt_transport_verified') is True and proof.get('pixel_transport_verified') is True and
                proof.get('session_token_sha256')==self.session and proof.get('database_sha256')==self.config['database_sha256'],'binding transport proof missing')
        key=(receipt['node_id'],pixel['tag_id'])
        old=self.records.get(key)
        if old is not None:
            if old['binding']!=binding:self.conflicted.add(key);self.dirty=True
            return
        estimate=self.estimator(pixel)
        corners=estimate.get('corners_camera')
        if corners is not None:
            require(type(corners) is list and len(corners)==4,'estimated corner count')
            corners=[transform(self.body_from_camera,finite_vector(c,3,'estimated corner')) for c in corners]
        self.records[key]=dict(binding=deepcopy(binding),estimate=deepcopy(estimate),body_corners=corners)
        same=[k for k,r in self.records.items() if
              (r['binding']['accepted_image_receipt']['map_id'],r['binding']['reset_epoch'],k[1])==
              (receipt['map_id'],binding['reset_epoch'],pixel['tag_id'])]
        for evicted in same[:-64]:
            del self.records[evicted];self.conflicted.discard(evicted)
        while len(self.records)>4096:
            evicted,_=self.records.popitem(last=False);self.conflicted.discard(evicted)
        self.dirty=True

    def snapshot(self,registry):
        revision,nodes=registry_index(registry,self.session);grouped=defaultdict(list)
        for key,record in self.records.items():
            binding=record['binding'];receipt=binding['accepted_image_receipt'];node=receipt['node_id']
            item=dict(family=FAMILY,tag_id=binding['tag_id'],map_id=receipt['map_id'],reset_epoch=int(receipt['reset_epoch']),
                component_id=None,submap_key=None,frame_id=None,node_id=node,source_stamp_ns=receipt['header_stamp_ns'],
                placement_status='unplaced',geometry_status='receipt_node_missing',relocalization_verified=False,
                observation_is_historical=True,pixel_record_sha256=binding['pixel_observation']['sha256'])
            located=nodes.get(node)
            group=None;matrix=None
            if located is not None:
                group,node_stamp,matrix=located
                if (group['map_id'],group['reset_epoch'],node_stamp)!=(receipt['map_id'],int(receipt['reset_epoch']),stamp(receipt['header_stamp_ns'])):
                    group=None;item['geometry_status']='receipt_registry_identity_conflict'
                else:
                    item.update(component_id=group['component_id'],submap_key=group['key'],frame_id=group['frame_id'])
                    item['geometry_status']=record['estimate']['status']
            if key in self.conflicted:item['geometry_status']='conflicting_same_node_observation'
            elif group is not None and record['body_corners'] is not None:
                item['corners3d']=[transform(matrix,c) for c in record['body_corners']]
                item['center']=center(item['corners3d'])
                item['placement_status']='main_registered' if group['registered'] else 'local_submap'
                for metric in ('reprojection_rms_at_960','planar_branch_spread_m'):
                    if metric in record['estimate']:item[metric]=record['estimate'][metric]
            groupkey=(item['submap_key'] or ('missing',item['map_id'],item['reset_epoch']),FAMILY,item['tag_id'])
            grouped[groupkey].append(item)
        tags=[]
        for rows in grouped.values():
            # At most one marker per family/ID/component. Every cached measured
            # point is transformed again under the same current graph snapshot.
            rows.sort(key=lambda r:stamp(r['source_stamp_ns']))
            latest=deepcopy(rows[-1]);placed=[r for r in rows if r['placement_status']!='unplaced']
            bad=any(r['geometry_status'] in ('conflicting_same_node_observation','receipt_registry_identity_conflict') for r in rows)
            spatial_conflict=any(distance(a['center'],b['center'])>.10 for a in placed for b in placed)
            if bad or spatial_conflict:
                latest.update(placement_status='unplaced',geometry_status='conflicting_tag_placement')
                latest.pop('center',None);latest.pop('corners3d',None)
            latest['observation_count']=len(rows);tags.append(latest)
        tags.sort(key=lambda t:(t['map_id'],t['reset_epoch'],t['submap_key'] or '',t['family'],t['tag_id']))
        self.dirty=False
        return dict(schema=SCHEMA,session_short_hash=self.session[:16],session_token_sha256=self.session,
                    registry_revision=revision,visualization_valid=True,tags=tags)
