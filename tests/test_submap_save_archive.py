from pathlib import Path
import copy
import json
import tempfile
import unittest

LAUNCHER = Path(__file__).resolve().parents[1]/'launch/deployed_20261015/start_rtabmap_calib_ab.sh'
source = LAUNCHER.read_text(encoding='utf-8').split("<<'SAFE_EXPORT_PY'", 1)[1].split('\nSAFE_EXPORT_PY', 1)[0]
scope = {'__name__':'safe_export_test'}
exec(compile(source, str(LAUNCHER), 'exec'), scope)


def fixture():
    pose = [1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.]
    nodes = {i:{'map_id':0 if i<3 else 1, 'stamp':float(i),'pose':pose.copy()} for i in range(1,5)}
    nodes[3]['pose'][3] = 20.
    nodes[4]['pose'][3] = 20.2
    return {'node_inventory':nodes,'saved_ids':[1,2], 'metric_edges':[(1,2),(3,4)],
            'rgb_depth_available_ids':[1,2,3,4],'source_snapshot':{'sha256':'a'*64},
            'coverage':{'dangling_metric_links':[],'invalid_node_pose_ids':[],
                        'weight_excluded_positive_ids':[5]}}


class ArchiveTests(unittest.TestCase):
    def run_archive(self, data):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);out=root/'.safe_export.test';out.mkdir()
            (root/'corner_recovery').mkdir()
            (root/'corner_recovery/rejected.json').write_text(json.dumps({'evidence_pairs':[[3,1],[4,2]],'reason':'insufficient independent views'}))
            report=scope['archive_submaps'](out,data)
            self.assertEqual(report,json.loads((out/'submaps.json').read_text()))
            return report

    def test_partial_main_and_local_component_remain_separate(self):
        report=self.run_archive(fixture())
        self.assertTrue(report['complete'])
        self.assertFalse(report['automatic_continuation_allowed'])
        self.assertEqual(report['saved_main_node_ids'],[1,2])
        child=report['submaps'][0]
        self.assertFalse(child['attached'])
        self.assertEqual(child['node_ids'],[3,4])
        self.assertEqual(child['poses']['3'][3],0.)
        self.assertAlmostEqual(child['poses']['4'][3],.2)
        self.assertEqual(report['weight_excluded_node_ids_retained_in_database'],[5])
        self.assertEqual(len(report['relations_and_tag_bindings']),1)

    def test_missing_nodes_inside_main_component_are_not_partial_submaps(self):
        data=fixture();data['metric_edges'].append((2,3))
        with self.assertRaises(scope['GuardError']):self.run_archive(data)

    def test_broken_data_or_graph_is_not_success(self):
        for kind in ('missing_data','invalid_pose','dangling'):
            data=copy.deepcopy(fixture())
            if kind=='missing_data':data['rgb_depth_available_ids'].remove(4)
            if kind=='invalid_pose':data['coverage']['invalid_node_pose_ids']=[4]
            if kind=='dangling':data['coverage']['dangling_metric_links']=[[4,99,0]]
            with self.assertRaises(scope['GuardError']):self.run_archive(data)


if __name__=='__main__':unittest.main()
