"""Behavior tests for evidence identity, coordinate placement and display invalidation."""
from copy import deepcopy
from pathlib import Path
import json,math,sys,time,types,unittest
sys.path.insert(0,str(Path(__file__).parent/'baseline'))
from tag_pixel_record import image_record,make_pixel_record
from tag_visualization_geometry import TagPlacementStore,registry_index,estimate_camera_corners
from tag_visualization import TagVisualizationPublisher

SESSION='a'*64;DATABASE='0'*64
I=[1,0,0,0,0,1,0,0,0,0,1,0]

def config(extrinsic=None):return dict(session_token_sha256=SESSION,database_sha256=DATABASE,body_from_camera=extrinsic or I)

def binding(node=1,map_id=0,epoch=0,tag_id=7,ns=None,extrinsic=None):
    ns=ns or node*1_000_000_000
    image=image_record(ns,'camera_optical',384,240,'bgr8',bytes(384*240*3))
    K=[400.,0.,192.,0.,400.,120.,0.,0.,1.];D=[0.,0.,0.,0.,0.]
    pixel=make_pixel_record(image,tag_id,.08,[[100.,100.],[140.,100.],[140.,140.],[100.,140.]],K,D)
    receipt=dict(schema_version=1,producer='wbrov_core_accepted_image_input',node_id=node,map_id=map_id,
        header_stamp_ns=str(ns),reset_epoch=str(epoch),image_width=384,image_height=240,encoding='bgr8',
        rgb_sha256=image['sha256'],camera_frame='camera_optical',K=K,D=D,body_from_camera=extrinsic or I,
        rgb_publisher_gid='1'*32,camera_info_publisher_gid='2'*32,session_token_sha256=SESSION,
        database_sha256=DATABASE,source_scope='managed_ros_rgbd',pixel_role='authenticated_pre_memory_input',saved_jpeg_bytes_are_original_pixels=0)
    return dict(keyframe_id=node,tag_id=tag_id,reset_epoch=str(epoch),image_stamp_ns=str(ns),
        accepted_image_receipt=receipt,pixel_observation=pixel,
        transport_proof=dict(core_receipt_transport_verified=True,pixel_transport_verified=True,
            session_token_sha256=SESSION,database_sha256=DATABASE,current_endpoints={}))

def measured(_):return dict(status='single_view_position_estimate',corners_camera=[[-.04,-.04,1],[.04,-.04,1],[.04,.04,1],[-.04,.04,1]],reprojection_rms_at_960=.1,planar_branch_spread_m=.002)

def group(bindings,registered=False,translation=None,component=None):
    first=bindings[0]['accepted_image_receipt'];mid=first['map_id'];epoch=int(first['reset_epoch']);component=component or first['node_id']
    key=f'm{mid}_e{epoch}_c{component}'
    return dict(key=key,map_id=mid,reset_epoch=epoch,component_id=component,registered=registered,active=True,status='active',
        frame_id='map' if registered else f'wbrov_submap_{SESSION[:16]}_{key}',
        graph_topic='/wbrov/submaps/'+key+'/graph',cloud_topic='/wbrov/submaps/'+key+'/cloud',display_topic='/wbrov/submaps/'+key+'/display',
        nodes=[dict(id=b['keyframe_id'],source_stamp_ns=b['image_stamp_ns'],translation=translation or [0,0,0],quaternion_xyzw=[0,0,0,1]) for b in bindings])

def registry(groups,revision=1):return dict(schema='wbrov.submaps.v1',session_short_hash=SESSION[:16],session_token_sha256=SESSION,revision=revision,main_frame_id='map',submaps=groups)

class PlacementTests(unittest.TestCase):
    def setUp(self):self.store=TagPlacementStore(config(),measured)
    def test_unregistered_tag_stays_in_its_actual_component_frame(self):
        b=binding();self.store.add(b);r=registry([group([b])]);tag=self.store.snapshot(r)['tags'][0]
        self.assertEqual(tag['placement_status'],'local_submap');self.assertEqual(tag['center'],[0.,0.,1.])
        self.assertNotEqual(tag['frame_id'],'map');self.assertIs(tag['relocalization_verified'],False)
    def test_exact_stamp_map_and_epoch_required_without_pose_extrapolation(self):
        b=binding();self.store.add(b)
        for field,value in [('source_stamp_ns','2000000000')]:
            r=registry([group([b])]);r['submaps'][0]['nodes'][0][field]=value
            tag=self.store.snapshot(r)['tags'][0];self.assertEqual(tag['placement_status'],'unplaced');self.assertNotIn('center',tag)
        other=group([binding(map_id=2,epoch=1)])
        tag=self.store.snapshot(registry([other]))['tags'][0]
        self.assertEqual(tag['geometry_status'],'conflicting_tag_placement');self.assertNotIn('corners3d',tag)
    def test_missing_pose_has_no_coordinates(self):
        b=binding();self.store.add(b);g=group([b]);g['nodes']=[]
        tag=self.store.snapshot(registry([g]))['tags'][0]
        self.assertEqual(tag['geometry_status'],'receipt_node_missing');self.assertIsNone(tag['submap_key']);self.assertNotIn('center',tag)
    def test_same_id_in_different_maps_or_components_is_never_merged(self):
        rows=[binding(1),binding(2,map_id=1,epoch=1),binding(3)]
        for b in rows:self.store.add(b)
        tags=self.store.snapshot(registry([group([b],translation=[i,0,0]) for i,b in enumerate(rows)]))['tags']
        self.assertEqual(len(tags),3);self.assertEqual(len({t['submap_key'] for t in tags}),3)
        self.assertTrue(all(t['placement_status']=='local_submap' for t in tags))
    def test_repeated_detections_replace_one_component_marker(self):
        rows=[binding(1),binding(2)]
        for b in rows:self.store.add(b);self.store.add(deepcopy(b))
        tags=self.store.snapshot(registry([group(rows)]))['tags']
        self.assertEqual(len(tags),1);self.assertEqual(tags[0]['node_id'],2);self.assertEqual(tags[0]['observation_count'],2)
    def test_optimization_recomputes_pose_and_promotion_requires_registry(self):
        b=binding();self.store.add(b)
        a=self.store.snapshot(registry([group([b],translation=[1,0,0])]))['tags'][0]
        b2=self.store.snapshot(registry([group([b],translation=[3,0,0])],2))['tags'][0]
        self.assertEqual(a['center'],[1,0,1]);self.assertEqual(b2['center'],[3,0,1]);self.assertEqual(b2['placement_status'],'local_submap')
        c=self.store.snapshot(registry([group([b],registered=True,translation=[8,0,0])],3))['tags'][0]
        self.assertEqual(c['frame_id'],'map');self.assertEqual(c['placement_status'],'main_registered');self.assertEqual(c['center'],[8,0,1])
    def test_actual_extrinsic_applied_once_in_correct_direction(self):
        B=[0,0,1,.2,-1,0,0,0,0,-1,0,0];store=TagPlacementStore(config(B),measured);b=binding(extrinsic=B);store.add(b)
        tag=store.snapshot(registry([group([b],translation=[4,0,0])]))['tags'][0]
        self.assertAlmostEqual(tag['center'][0],5.2);self.assertAlmostEqual(tag['center'][1],0);self.assertAlmostEqual(tag['center'][2],0)
    def test_dimension_image_hash_or_extrinsic_mismatch_rejected(self):
        for mutate in (lambda r:r.update(image_width=960),lambda r:r.update(rgb_sha256='f'*64),lambda r:r.update(body_from_camera=[1,0,0,1,0,1,0,0,0,0,1,0])):
            b=binding();mutate(b['accepted_image_receipt'])
            with self.assertRaises(ValueError):self.store.add(b)
    def test_conflict_removes_placement(self):
        rows=[binding(1),binding(2)]
        for b in rows:self.store.add(b)
        g=group(rows);g['nodes'][1]['translation']=[1,0,0]
        tag=self.store.snapshot(registry([g]))['tags'][0]
        self.assertEqual(tag['placement_status'],'unplaced');self.assertEqual(tag['geometry_status'],'conflicting_tag_placement');self.assertNotIn('center',tag)
    def test_ambiguous_planar_pose_never_becomes_a_map_coordinate(self):
        store=TagPlacementStore(config(),lambda _:dict(status='ambiguous_planar_pose',corners_camera=None));b=binding();store.add(b)
        tag=store.snapshot(registry([group([b],registered=True)]))['tags'][0]
        self.assertEqual(tag['placement_status'],'unplaced');self.assertNotIn('corners3d',tag)
    def test_conflicting_same_node_observation_remains_invalid(self):
        b=binding();self.store.add(b);bad=deepcopy(b);bad['transport_proof']['conflicting_extra']='x';self.store.add(bad)
        tag=self.store.snapshot(registry([group([b])]))['tags'][0]
        self.assertEqual(tag['placement_status'],'unplaced')
    def test_invalid_registry_frame_session_duplicate_node_rejected(self):
        b=binding();g=group([b]);r=registry([g]);r['session_token_sha256']='b'*64
        with self.assertRaises(ValueError):self.store.snapshot(r)
        r=registry([g]);r['submaps'][0]['frame_id']='map'
        with self.assertRaises(ValueError):self.store.snapshot(r)
        g1=group([b]);g2=group([binding(2)]);g2['nodes'][0]['id']=1
        with self.assertRaises(ValueError):self.store.snapshot(registry([g1,g2]))
    def test_registry_accepts_more_than_128_components_but_remains_bounded(self):
        sample=binding();groups=[]
        for node in range(1,130):
            b=deepcopy(sample);b['keyframe_id']=node;b['accepted_image_receipt']['node_id']=node
            groups.append(group([b]))
        _,nodes=registry_index(registry(groups),SESSION);self.assertEqual(len(nodes),129)
        with self.assertRaises(ValueError):registry_index(registry([groups[0]]*4097),SESSION)

class Source:
    def __init__(self):self.valid=True
    def check(self,role,incoming):
        if role!='registry' or incoming!='owned_gid':raise ValueError('publisher identity')
    def all_current(self):
        if not self.valid:raise ValueError('publisher absent/replaced')
        return {}

class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.adapter=TagVisualizationPublisher.__new__(TagVisualizationPublisher)
        a=self.adapter;a.closed=False;a.config=config();a.sources=Source();a.consumer=types.SimpleNamespace(sources=Source())
        a.store=TagPlacementStore(config(),measured);a.registry=None;a.last_registry_ns=0;a.last_revision=-1;a.invalidated=None;a.max_age_ns=3_000_000_000
        self.published=[];a.publish=lambda payload:self.published.append(deepcopy(payload))
        self.b=binding();a.store.add(self.b)
    def send(self,row=None,gid='owned_gid'):
        self.adapter.receive_registry(types.SimpleNamespace(data=json.dumps(row or registry([group([self.b])]))),gid)
    def test_authenticated_actual_registry_required(self):
        self.send(gid='wrong_gid');self.assertEqual(self.published[-1]['tags'],[])
        self.send();self.assertEqual(len(self.published[-1]['tags']),1)
    def test_stale_or_replaced_source_clears_old_markers(self):
        self.send();self.adapter.last_registry_ns=time.monotonic_ns()-4_000_000_000;self.adapter.tick()
        self.assertFalse(self.published[-1]['visualization_valid']);self.assertEqual(self.published[-1]['tags'],[])
        self.send();self.adapter.sources.valid=False;self.adapter.tick();self.assertEqual(self.published[-1]['tags'],[])
    def test_invalid_new_registry_cannot_fall_back_to_old_pose(self):
        self.send();bad=registry([group([self.b])],2);bad['submaps'][0]['nodes'][0]['quaternion_xyzw']=[0,0,0,0]
        self.send(bad);self.adapter.store.dirty=True;self.adapter.tick()
        self.assertIsNone(self.adapter.registry);self.assertEqual(self.published[-1]['tags'],[])
    def test_identical_revision_heartbeat_refreshes_only_source_liveness(self):
        self.send();previous=deepcopy(self.published[-1])
        self.adapter.last_registry_ns=time.monotonic_ns()-4_000_000_000
        self.send()
        self.assertLess(time.monotonic_ns()-self.adapter.last_registry_ns,1_000_000_000)
        self.assertEqual(self.published[-1],previous)
        self.assertIs(self.published[-1]['tags'][0]['relocalization_verified'],False)
    def test_same_revision_mutation_and_older_revision_are_rejected(self):
        self.send(registry([group([self.b])],3))
        changed=registry([group([self.b],translation=[8,0,0])],3)
        self.send(changed);self.assertEqual(self.published[-1]['tags'],[])
        self.send(registry([group([self.b])],4));self.assertEqual(len(self.published[-1]['tags']),1)
        self.send(registry([group([self.b])],2));self.assertEqual(self.published[-1]['tags'],[])
    def test_conflicting_consumer_evidence_clears_and_stays_invalid(self):
        self.send();self.adapter.consume_event(dict(event='rejected',reason='conflicting bound pixel observation'))
        self.send(registry([group([self.b])],2));self.assertEqual(self.published[-1]['tags'],[])
    def bind_actual_adapter(self):
        self.adapter.consumer.bindings={(1,7):self.b}
        self.adapter.consumer.check_receipt_context=lambda receipt:None
        self.adapter.consume_event(dict(event='bound',keyframe_id=1,tag_id=7))
    def test_bound_evidence_is_retained_when_registry_arrives_later(self):
        self.adapter.store=TagPlacementStore(config(),measured)
        self.bind_actual_adapter();self.assertEqual(len(self.adapter.store.records),1)
        self.adapter.tick();self.assertEqual(self.published[-1]['tags'],[])
        self.send();self.assertEqual(self.published[-1]['tags'][0]['placement_status'],'local_submap')
    def test_registry_first_then_bound_callback_is_rendered_on_timer(self):
        self.adapter.store=TagPlacementStore(config(),measured)
        self.send();self.assertEqual(self.published[-1]['tags'],[])
        self.bind_actual_adapter();self.adapter.tick()
        self.assertEqual(self.published[-1]['tags'][0]['placement_status'],'local_submap')
    def test_observation_survives_temporary_missing_node_without_extrapolation(self):
        g=group([self.b]);g['nodes']=[];self.send(registry([g]))
        self.assertEqual(self.published[-1]['tags'][0]['placement_status'],'unplaced')
        self.assertNotIn('center',self.published[-1]['tags'][0])
        self.send(registry([group([self.b])],2))
        self.assertEqual(self.published[-1]['tags'][0]['placement_status'],'local_submap')

try:
    import cv2,numpy as np
    OPENCV=True
except ImportError:OPENCV=False

@unittest.skipUnless(OPENCV,'OpenCV/numpy unavailable locally; run this behavior test on the native candidate runtime')
class NativeProjectionTests(unittest.TestCase):
    def test_real_ippe_ambiguous_distant_view_has_no_position(self):
        pixel=binding()['pixel_observation'];s=.04
        obj=np.asarray([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],float)
        rv=np.asarray([math.pi-.2,.03,.02],float);tv=np.asarray([.02,-.01,.8],float)
        K=np.asarray(pixel['camera']['K']).reshape(3,3);D=np.asarray(pixel['camera']['D'])
        pixel['corners_xy']=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(4,2).tolist()
        result=estimate_camera_corners(pixel)
        self.assertEqual(result['status'],'ambiguous_planar_pose')
        self.assertIsNone(result['corners_camera'])
        self.assertGreater(result['planar_branch_spread_m'],.02)
    def test_real_ippe_recovers_projected_metric_corners(self):
        pixel=binding()['pixel_observation'];s=.04
        obj=np.asarray([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],float)
        rv=np.asarray([math.pi-.7,.03,.02],float);tv=np.asarray([.02,-.01,.35],float)
        K=np.asarray(pixel['camera']['K']).reshape(3,3);D=np.asarray(pixel['camera']['D'])
        pixel['corners_xy']=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(4,2).tolist()
        result=estimate_camera_corners(pixel)
        self.assertEqual(result['status'],'single_view_position_estimate')
        expected=obj@cv2.Rodrigues(rv)[0].T+tv
        self.assertLess(float(np.max(np.linalg.norm(np.asarray(result['corners_camera'])-expected,axis=1))),1e-4)
    def test_real_ippe_camera_body_submap_promotion_and_optimization(self):
        B=[0,0,1,.2,-1,0,0,0,0,-1,0,0];b=binding(extrinsic=B);pixel=b['pixel_observation'];s=.04
        obj=np.asarray([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],float)
        rv=np.asarray([math.pi-.7,.03,.02],float);tv=np.asarray([.02,-.01,.35],float)
        K=np.asarray(pixel['camera']['K']).reshape(3,3);D=np.asarray(pixel['camera']['D'])
        uv=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(4,2).tolist()
        b['pixel_observation']=make_pixel_record(pixel['image'],7,.08,uv,pixel['camera']['K'],pixel['camera']['D'])
        store=TagPlacementStore(config(B));store.add(b)
        g=group([b],translation=[2,3,4]);g['nodes'][0]['quaternion_xyzw']=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
        local=store.snapshot(registry([g]))['tags'][0]
        camera=obj@cv2.Rodrigues(rv)[0].T+tv
        body=camera@np.asarray([[0,0,1],[-1,0,0],[0,-1,0]],float).T+np.asarray([.2,0,0])
        expected=body@np.asarray([[0,-1,0],[1,0,0],[0,0,1]],float).T+np.asarray([2,3,4])
        self.assertEqual(local['placement_status'],'local_submap')
        self.assertLess(float(np.max(np.linalg.norm(np.asarray(local['corners3d'])-expected,axis=1))),1e-4)
        g['registered']=True;g['frame_id']='map';g['nodes'][0]['translation']=[5,3,4]
        aligned=store.snapshot(registry([g],2))['tags'][0]
        self.assertEqual(aligned['placement_status'],'main_registered');self.assertEqual(aligned['frame_id'],'map')
        self.assertLess(float(np.max(np.linalg.norm(np.asarray(aligned['corners3d'])-(expected+[3,0,0]),axis=1))),1e-4)
        g['nodes'][0]['translation']=[5,4,4]
        optimized=store.snapshot(registry([g],3))['tags'][0]
        self.assertLess(float(np.max(np.linalg.norm(np.asarray(optimized['corners3d'])-(expected+[3,1,0]),axis=1))),1e-4)

if __name__=='__main__':unittest.main()
