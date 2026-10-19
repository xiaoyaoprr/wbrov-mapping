from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/visualization'))
from deferred_display_state import mapping_label,trajectory_data,pose_axes

class DisplayState(unittest.TestCase):
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
