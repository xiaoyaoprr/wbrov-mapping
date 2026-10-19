from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/visualization'))
from deferred_display_state import mapping_label,trajectory_data,pose_axes,recovery_lines

class DisplayState(unittest.TestCase):
    def test_no_tag_is_not_a_visual_recovery_veto(self):
        lines=recovery_lines({'recovery_route':'visual_keyframes','main_map_write_allowed':False,
                              'detected_tags_in_processed_frame':1},{'tags':[]})
        self.assertIn('主图 Tag：0 个，参考齐备：0 个',lines)
        self.assertIn('主图无 Tag，仍可用旧关键帧尝试恢复',lines)
        self.assertTrue(any('不等于已绑定' in line for line in lines))
        self.assertTrue(any('不要求 Tag' in line for line in lines))
        self.assertNotIn('主图 Tag：0 个，参考齐备：0 个',recovery_lines({},None))

    def test_all_tag_reference_counts_and_current_diagnostic(self):
        tags=[dict(tag_id=i,independent_references=min(i,3),distinct_main_nodes=5) for i in range(5)]
        mapping={'recovery_attempt_source_ns':10,'observation_diagnostic':{'source_ns':10,'baseline_rejected':1}}
        lines=recovery_lines(mapping,{'tags':tags})
        self.assertIn('主图 Tag：5 个，参考齐备：2 个',lines)
        self.assertEqual(sum(line.startswith('Tag ') for line in lines),5)
        self.assertTrue(any('旧参考视角不够独立' in line for line in lines))
        mapping['recovery_attempt_source_ns']=11
        self.assertFalse(any('旧参考视角不够独立' in line for line in recovery_lines(mapping,{'tags':tags})))

    def test_failure_subitem_survives_presentation(self):
        mapping={'quality':{'input_present':True,'usable':False,'covariance_valid':False}}
        self.assertTrue(any('位姿不确定度' in line for line in recovery_lines(mapping,{'tags':[]})))
        mapping['main_map_write_allowed']=True
        self.assertFalse(any('位姿不确定度' in line for line in recovery_lines(mapping,{'tags':[]})))

    def test_body_axes_rotate_with_pose(self):
        from math import sqrt
        node={'position':[1.,2.,3.],'orientation_xyzw':[0.,0.,sqrt(.5),sqrt(.5)]}
        axes=pose_axes(node,1.)
        for (label,color,tip),expected in zip(axes,[[1.,3.,3.],[0.,2.,3.],[1.,2.,4.]]):
            for actual,want in zip(tip,expected):self.assertAlmostEqual(actual,want)
        self.assertEqual([x[0] for x in axes],list('XYZ'))
        node['orientation_xyzw']=[0.,0.,0.,0.]
        with self.assertRaises(ValueError):pose_axes(node)

    def test_processing_does_not_imply_map_writes(self):
        self.assertIn('暂停',mapping_label({'state':'processing','mapping':{'continuity_state':'PAUSED','main_map_write_allowed':False}})[0])
        self.assertEqual(mapping_label({'state':'processing','mapping':{'main_map_write_allowed':True}})[0],'建图中')
        self.assertIn('核验',mapping_label({'state':'processing','mapping':{'continuity_state':'RECOVERING','main_map_write_allowed':False}})[0])
        self.assertEqual(mapping_label({'state':'closed','mapping':{'main_map_write_allowed':False}})[0],'已保存')

    def test_trajectory_has_no_bridge_over_unaligned_gap(self):
        def node(i,epoch):return dict(node_id=i,source_ns=i,epoch=epoch,position=[i,0.,0.],orientation_xyzw=[0.,0.,0.,1.])
        payload={'trajectory':[node(1,0),node(2,0),node(3,1)],'trajectory_edges':[[1,2]]}
        edges,last=trajectory_data(payload);self.assertEqual(len(edges),1);self.assertEqual(last['node_id'],3)
        payload['trajectory_edges'].append([2,3])
        with self.assertRaises(ValueError):trajectory_data(payload)

if __name__=='__main__':unittest.main()
