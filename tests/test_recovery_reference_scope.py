import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]/'runtime/libexec/wbrov_corner_recovery'))
import reference_policy


class ReferenceScopeTests(unittest.TestCase):
    def live(self, retained, committed=(1,)):
        rows = {(1, node, 8): {'id': node, 'binding': {'accepted_image_receipt': {'node_id': node, 'reset_epoch': 1}}}
                for node in (11, 12, 13, 14)}
        queried = []
        def pose(receipt):
            self.assertIn(receipt['node_id'], retained)
            queried.append(receipt['node_id'])
            return {}, 10
        graph = SimpleNamespace(has_pose_for_receipt=lambda r: r['node_id'] in retained, pose_for_receipt=pose)
        return SimpleNamespace(rows=rows, graph=graph, committed_epochs=set(committed)), queried

    def options(self, live):
        with patch.object(reference_policy, 'matrix', lambda _: 'verified_pose'), patch.object(
                reference_policy, 'best_old_triplet', lambda rows: (rows, None) if len(rows)>=3 else None):
            return reference_policy.historical_options(live, 8, 2)

    def test_committed_epoch_does_not_authorize_excluded_nodes(self):
        live, queried = self.live({11, 13, 14})
        result = self.options(live)
        self.assertEqual([r['id'] for r in result[0]['reference'][0]], [11, 13, 14])
        self.assertNotIn(12, queried)

    def test_missing_references_wait_without_reading_stale_poses(self):
        live, queried = self.live({13, 14})
        self.assertEqual(self.options(live), [])
        self.assertEqual(queried, [13, 14])

    def test_uncommitted_history_is_not_a_main_reference(self):
        live, queried = self.live({11, 12, 13, 14}, committed=())
        self.assertEqual(self.options(live), [])
        self.assertEqual(queried, [])

    def test_readiness_excludes_retired_and_uncommitted_references(self):
        live,_=self.live({11,13})
        with patch.object(reference_policy,'matrix',lambda _: 'verified_pose'), patch.object(
                reference_policy,'best_old_triplet',lambda rows:(rows,None) if len(rows)>=3 else None):
            result=reference_policy.reference_readiness(live)
        self.assertEqual(result[0]['distinct_saved_nodes'],2)
        self.assertFalse(result[0]['independent_consistent_triplet'])
        self.assertFalse(result[0]['recovery_authorized'])
        live.committed_epochs.clear()
        self.assertEqual(reference_policy.reference_readiness(live),[])

    def test_static_larger_images_do_not_evict_independent_views(self):
        import numpy as np
        from measured_bridge import reference_reservoir,independent
        def row(i,x,area):
            body=np.eye(4);body[0,3]=x
            return dict(id=i,stamp=i*10**9,body=body,area=area,branches=[{}])
        rows=[row(i,0,1000+i) for i in range(1,40)]+[row(40,.04,100),row(41,.08,90)]
        kept=reference_reservoir(rows)
        self.assertEqual(len(kept),24)
        self.assertTrue({40,41}.issubset({r['id'] for r in kept}))
        self.assertTrue(independent(kept[:3]))

    def test_readiness_source_loss_clears_display_without_stopping_consumer(self):
        import json,tempfile
        from live_bridge import LiveBridge
        with tempfile.TemporaryDirectory() as directory:
            live=LiveBridge.__new__(LiveBridge)
            def absent():raise ValueError('publisher absent')
            live.consumer=SimpleNamespace(sources=SimpleNamespace(all_current=absent))
            live.graph=SimpleNamespace(current={'state':{'reset_epoch':1}},last_revision=12)
            live.config={'session_token_sha256':'a'*64}
            live.readiness_tags=[{'tag_id':8}];live.readiness_key='old'
            live.String=SimpleNamespace;messages=[]
            live.readiness_publisher=SimpleNamespace(publish=messages.append)
            live.readiness_path=Path(directory)/'readiness.json'
            live.publish_readiness()
            payload=json.loads(messages[-1].data)
            self.assertFalse(payload['source_valid']);self.assertEqual(payload['tags'],[])
            self.assertFalse(payload['graph_update_authorized']);self.assertIsNone(live.readiness_key)


if __name__ == '__main__':
    unittest.main()
