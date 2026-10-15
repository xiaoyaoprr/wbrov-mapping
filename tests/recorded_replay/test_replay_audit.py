import unittest
from replay_audit import audit,graph_components

class AuditTests(unittest.TestCase):
    def base(self):
        return dict(info=[],graphs=[dict(ns=5,arrival_steady_ns=5,node_ids=[1,2],components=1)],
            continuity=[dict(arrival_steady_ns=t,status=dict(source_stamp_ns=t,
                main_map_write_allowed=False,generation=1,epoch=1,state='PAUSED')) for t in (10,20)])
    def test_public_graph_growth_is_not_hidden(self):
        s=self.base();s['graphs'].append(dict(ns=15,arrival_steady_ns=15,node_ids=[1,2,3],components=1))
        self.assertEqual(audit(s)['main_graph_growth_during_monitored_blocked_intervals']['status'],'violation')
    def test_transition_gap_not_claimed(self):
        s=self.base();s['continuity'][1]['status']['generation']=2
        self.assertEqual(audit(s)['main_graph_growth_during_monitored_blocked_intervals']['status'],'not_proven')
    def test_missing_evidence_not_success(self):
        r=audit({});self.assertEqual(r['native_depth_fallback']['status'],'not_proven')
        self.assertEqual(r['second_disconnected_public_map']['status'],'not_proven')
    def test_connected_component_detection(self):
        self.assertEqual(graph_components([1,2,3],[(1,2)]),2)
        self.assertEqual(graph_components([1,2,3],[(1,2),(2,3)]),1)
    def test_quiescent_output_claim_is_limited(self):
        self.assertEqual(audit(self.base())['main_graph_growth_during_monitored_blocked_intervals']['status'],'observed_no_growth')
    def test_fake_visual_depth_vote_not_counted(self):
        s=self.base();s['info']=[dict(lost=False,inliers=20,icp_correspondences=100)]
        self.assertEqual(audit(s)['native_depth_fallback']['status'],'not_proven')
    def test_startup_pause_is_not_loss_recovery(self):
        s=self.base();s['continuity'].append(dict(arrival_steady_ns=25,status=dict(source_stamp_ns=25,
            main_map_write_allowed=True,generation=1,epoch=1,state='NORMAL',registered=True)))
        self.assertEqual(audit(s)['resume_after_pause']['status'],'not_proven')
    def test_uncommitted_candidate_observation_is_reported_without_claiming_resume(self):
        s=self.base();s['continuity'][0]['status'].update(recovery_commit_pending=True,observing_registered_map=False)
        out=audit(s)
        self.assertEqual(out['registered_map_observation']['status'],'observed')
        self.assertEqual(out['registered_map_observation']['completion'],'not_proven')
        self.assertEqual(out['resume_after_pause']['status'],'not_proven')
    def test_loss_before_main_bootstrap_cannot_count_as_recovery(self):
        s=self.base();s['info']=[dict(ns=1,lost=True)]
        s['continuity'].append(dict(arrival_steady_ns=25,status=dict(source_stamp_ns=25,registered=True,main_map_write_allowed=True)))
        s['graphs'].append(dict(ns=30,arrival_steady_ns=30,node_ids=[1,2,3],components=1))
        self.assertEqual(audit(s)['resume_after_pause']['status'],'not_proven')
    def test_recovery_requires_public_growth_after_real_post_bootstrap_loss(self):
        s=self.base();s['continuity'].insert(0,dict(arrival_steady_ns=6,status=dict(source_stamp_ns=6,registered=True,main_map_write_allowed=True)))
        s['info']=[dict(ns=9,lost=True)]
        s['continuity'].append(dict(arrival_steady_ns=25,status=dict(source_stamp_ns=25,registered=True,main_map_write_allowed=True)))
        self.assertEqual(audit(s)['resume_after_pause']['status'],'not_proven')
        s['graphs'].append(dict(ns=30,arrival_steady_ns=30,node_ids=[1,2,3],components=1))
        self.assertEqual(audit(s)['resume_after_pause']['status'],'observed')
    def test_epoch_change_alone_is_not_native_reset_proof(self):
        s=self.base();s['continuity'][1]['status']['epoch']=2
        self.assertEqual(audit(s)['native_reset']['status'],'not_proven')
        s['native_reset_log_lines']=['Odometry automatically reset to latest pose']
        self.assertEqual(audit(s)['native_reset']['status'],'observed')
    def test_controlled_depth_success_requires_real_geometric_quality(self):
        s=self.base();r=dict(ns=12,lost=False,inliers=0,icp_correspondences=100,icp_inliers_ratio=.8,
            icp_structural_complexity=.001,icp_structural_distribution=.75,fault_phase='blank_rgb_depth_preserved')
        s['info']=[r]
        self.assertEqual(audit(s)['controlled_short_depth_fallback']['status'],'observed')
        r['icp_structural_complexity']=0
        self.assertEqual(audit(s)['controlled_short_depth_fallback']['status'],'not_proven')
    def test_fault_before_bootstrap_is_explicit(self):
        s=self.base();s['faults']=[dict(ns=12,kind='blank_rgb_and_depth')]
        self.assertFalse(audit(s)['fault_bootstrap_preconditions'][0]['main_map_established_before_fault'])
        s['continuity'].insert(0,dict(arrival_steady_ns=6,status=dict(source_stamp_ns=6,registered=True,main_map_write_allowed=True)))
        self.assertTrue(audit(s)['fault_bootstrap_preconditions'][0]['main_map_established_before_fault'])
    def test_cloud_payload_change_is_reported_without_inventing_node_growth(self):
        s=self.base();s['clouds']=[dict(ns=t,arrival_steady_ns=t,frame_id='map',width=2,height=1,
            data_bytes=24,data_sha256=str(t)) for t in (5,15)]
        r=audit(s)
        self.assertEqual(r['public_cloud_payload_during_monitored_blocked_intervals']['status'],'changed')
        self.assertEqual(r['main_graph_growth_during_monitored_blocked_intervals']['status'],'observed_no_growth')

if __name__=='__main__':unittest.main()
