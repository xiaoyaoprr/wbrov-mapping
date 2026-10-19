import json
import ast
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).parents[1]/'runtime/libexec/wbrov_corner_recovery'))
from managed_graph_state import ManagedGraphState


class RetirementTests(unittest.TestCase):
    def graph(self):
        graph=ManagedGraphState.__new__(ManagedGraphState)
        graph.consumer=SimpleNamespace(sources=SimpleNamespace(check=lambda *a,**k:'authenticated-test-gid'),
            rejected=lambda e:self.fail(str(e)),last_receipt_epoch=2)
        graph.config={'database_sha256':'a'*64,'session_token_sha256':'b'*64}
        graph.record=lambda *args:None
        graph.by_epoch={};graph.retired_by_epoch={};graph.current=None
        graph.last_revision=graph.last_epoch=-1;graph.received=graph.rejected=0
        return graph

    def receive(self,graph,revision,ids):
        state=dict(schema_version=3,producer='wbrov_rtabmap_safe_relocalization_core',reset_epoch=1,
            epoch_start_stamp_ns=1,graph_revision=revision,database_sha256='a'*64,
            database_binding_valid=True,session_token_sha256='b'*64,graph_snapshot_valid=True,
            graph_poses=[dict(id=i,translation=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.]) for i in ids])
        graph.receive(SimpleNamespace(data=json.dumps(state)),None)

    def test_removed_prefix_is_retired_but_future_receipt_waits(self):
        graph=self.graph();self.receive(graph,1,[21,22,23])
        self.receive(graph,2,[22,23])
        self.assertTrue(graph.receipt_retired(dict(reset_epoch=1,node_id=21)))
        self.assertFalse(graph.receipt_retired(dict(reset_epoch=1,node_id=24)))
        self.assertFalse(graph.has_pose_for_receipt(dict(reset_epoch=1,node_id=21)))

    def test_reappearing_authenticated_node_and_reclaimed_receipt(self):
        graph=self.graph();self.receive(graph,1,[21,22,23]);self.receive(graph,2,[23])
        graph.compact([dict(reset_epoch=1,node_id=21)])
        self.assertTrue(graph.receipt_retired(dict(reset_epoch=1,node_id=21)))
        self.assertFalse(graph.receipt_retired(dict(reset_epoch=1,node_id=22)))
        self.receive(graph,3,[21,23])
        self.assertFalse(graph.receipt_retired(dict(reset_epoch=1,node_id=21)))
        self.assertTrue(graph.has_pose_for_receipt(dict(reset_epoch=1,node_id=21)))

    def test_real_ready_loop_retires_only_removed_nodes_without_publishing(self):
        graph=self.graph();self.receive(graph,1,[21,22,23]);self.receive(graph,2,[22,23])
        source=Path(__file__).parents[1]/'runtime/libexec/wbrov_corner_recovery/consumer.py'
        function=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='ready')
        consumer=SimpleNamespace(receipts={n:{'receipt':dict(reset_epoch=1,node_id=i)} for n,i in [(100,21),(200,24)]},last_receipt_epoch=1)
        live=SimpleNamespace(completed_for_retention=set(),allow_ready=Mock())
        publish=Mock();emit=Mock();compact=Mock()
        env=dict(ready_count=0,last_retention_key=None,images={},infos={},consumer=consumer,
            pixel_consumed={100,200},pending_ready={100:[],200:[]},graph=graph,live=live,
            ready_pub=SimpleNamespace(publish=publish),emit=emit,compact=compact)
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),env)
        env['ready']()
        self.assertEqual(env['pending_ready'],{200:[]})
        self.assertEqual(env['pixel_consumed'],{200})
        self.assertEqual(live.completed_for_retention,{100})
        self.assertEqual(emit.call_args.args[0]['event'],'frame_retired_from_core_graph')
        publish.assert_not_called();live.allow_ready.assert_not_called();compact.assert_called_once()


if __name__=='__main__':unittest.main()
