from copy import deepcopy
import ast
from pathlib import Path
import unittest
from submap_replay_audit import submap_audit

SESSION='a'*64
def group(epoch,ids,registered=False):
    key=f'm{epoch}_e{epoch}_c{min(ids)}'
    return dict(key=key,map_id=epoch,reset_epoch=epoch,component_id=min(ids),registered=registered,active=True,
        frame_id='map' if registered else f'wbrov_submap_{SESSION[:16]}_{key}',status='registered_main' if registered else 'local_tracking',
        nodes=[dict(id=n,source_stamp_ns=n*10) for n in ids])
def registry(ns,revision,groups):
    return dict(arrival_steady_ns=ns,registry=dict(schema='wbrov.submaps.v1',session_token_sha256=SESSION,
        session_short_hash=SESSION[:16],revision=revision,source_stamp_ns=ns,main_frame_id='map',submaps=groups))
class SubmapTests(unittest.TestCase):
    def sample(self):
        return dict(expected_session_token_sha256=SESSION,
            registries=[registry(20,1,[group(0,[1],True),group(1,[2])]),registry(30,2,[group(0,[1],True),group(1,[2,3])])],
            continuity=[dict(status=dict(source_stamp_ns=30,local_tracking_healthy=True,local_submap_write_allowed=True,epoch=1))],
            graphs=[dict(ns=30,node_ids=[1])])
    def test_healthy_local_growth_does_not_become_main(self):
        out=submap_audit(self.sample());self.assertEqual(out['local_healthy_submap_growth']['status'],'observed')
        self.assertEqual(out['main_graph_registered_membership']['status'],'observed_consistent')
    def test_local_growth_needs_live_quality_not_registry_label(self):
        sample=self.sample();sample['continuity']=[]
        self.assertEqual(submap_audit(sample)['local_healthy_submap_growth']['status'],'not_proven')
    def test_private_node_publication_is_violation(self):
        sample=self.sample();sample['graphs'][0]['node_ids'].append(2)
        self.assertEqual(submap_audit(sample)['main_graph_registered_membership']['status'],'violation')
    def test_future_registration_does_not_authorize_earlier_main_output(self):
        sample=self.sample();sample['graphs'][0]['node_ids'].append(2)
        sample['registries'].append(registry(40,3,[group(0,[1],True),group(1,[2,3],True)]))
        self.assertEqual(submap_audit(sample)['main_graph_registered_membership']['status'],'violation')
    def test_independent_frame_and_no_tf(self):
        sample=self.sample();sample['submap_tf_edges']=[dict(parent='map',child='wbrov_submap_bad')]
        self.assertEqual(submap_audit(sample)['independent_submap_frames']['status'],'violation')
        sample=self.sample();sample['registries'][1]['registry']['submaps'][1]['frame_id']='map'
        self.assertEqual(submap_audit(sample)['independent_submap_frames']['status'],'violation')
    def test_lost_frame_receipt_rejected_by_audit(self):
        sample=self.sample();sample.update(info=[dict(ns=30,lost=True)],accepted=[dict(receipt=dict(header_stamp_ns=30,node_id=3))])
        self.assertEqual(submap_audit(sample)['untrusted_frame_write_gate']['status'],'violation')
    def test_actual_loss_requires_new_private_epoch(self):
        sample=self.sample();sample['info']=[dict(ns=25,lost=True)]
        self.assertEqual(submap_audit(sample)['loss_creates_separate_local_epoch']['status'],'violation')
        sample['registries'][1]['registry']['submaps'][1]=group(2,[3])
        self.assertEqual(submap_audit(sample)['loss_creates_separate_local_epoch']['status'],'observed_consistent')
    def test_graph_and_cloud_must_match_registry(self):
        sample=self.sample();g=sample['registries'][1]['registry']['submaps'][1]
        sample['submap_graphs']=[dict(ns=30,key=g['key'],frame_id=g['frame_id'],node_ids=[2,3],links=[[2,3]])]
        sample['submap_clouds']=[dict(ns=30,key=g['key'],frame_id=g['frame_id'])]
        self.assertEqual(submap_audit(sample)['submap_graph_registry_match']['status'],'observed_consistent')
        sample['submap_clouds'][0]['frame_id']='map'
        self.assertEqual(submap_audit(sample)['submap_cloud_registry_frame']['status'],'violation')
    def test_tag_placement_binds_exact_session_revision_node_frame(self):
        sample=self.sample();g=sample['registries'][1]['registry']['submaps'][1]
        tag=dict(node_id=3,submap_key=g['key'],map_id=1,reset_epoch=1,frame_id=g['frame_id'],source_stamp_ns=30,
            placement_status='local_submap',center=[0,0,1],corners3d=[[0,0,1]]*4,relocalization_verified=False)
        sample['tag_states']=[dict(state=dict(visualization_valid=True,session_token_sha256=SESSION,registry_revision=2,tags=[tag]))]
        self.assertEqual(submap_audit(sample)['tag_registry_identity']['status'],'observed_consistent')
        tag['frame_id']='map'
        self.assertEqual(submap_audit(sample)['tag_registry_identity']['status'],'violation')
    def test_registration_accepts_only_empty_clear_in_known_previous_frame(self):
        sample=self.sample();g=sample['registries'][1]['registry']['submaps'][1]
        sample['registries'].append(registry(40,3,[group(0,[1],True),group(1,[2,3],True)]))
        sample['submap_clouds']=[dict(ns=40,key=g['key'],frame_id=g['frame_id'],width=0,data_bytes=0)]
        self.assertEqual(submap_audit(sample)['submap_cloud_registry_frame']['status'],'observed_consistent')
        sample['submap_clouds'][0].update(width=1,data_bytes=16)
        self.assertEqual(submap_audit(sample)['submap_cloud_registry_frame']['status'],'violation')
    def test_missing_evidence_is_not_success(self):
        out=submap_audit({})
        for key in ('local_healthy_submap_growth','main_graph_registered_membership','tag_registry_identity'):
            self.assertEqual(out[key]['status'],'not_proven')
    def test_scripts_parse_without_running_ros_or_processes(self):
        root=Path(__file__).resolve().parent
        # Repository layout: inspect every archived replay module and the local
        # test entry point. Task-specific packaging/remote launch tools are not
        # part of this source repository and are never executed by this test.
        for path in list(root.glob('*.py'))+[root.parents[1]/'tools'/'test_mapping_candidate.py']:
            ast.parse(path.read_text(encoding='utf-8'),filename=str(path))
        driver=(root/'replay_single_map.py').read_text(encoding='utf-8')
        self.assertIn("TASK=ROOT/'submap_tag_20261015'",driver)
        self.assertIn("str(tagroot/'consumer.py')",driver)
        self.assertIn("'--rviz-output-dir',str(views)",driver)
        self.assertNotIn("'rviz2'",driver)

if __name__=='__main__':unittest.main(verbosity=2)
