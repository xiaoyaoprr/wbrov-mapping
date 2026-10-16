"""Exercise the real bridge ACK method and validators without starting ROS."""
from copy import deepcopy
import importlib.util,json,sys,time,types,unittest
from pathlib import Path
from unittest.mock import patch
from bridge_status import validate_terminal,validate_observing,require

def load_live_bridge():
    # ACK tests do not run geometry or ROS constructors; their imports remain
    # isolated while the real production status method/validators are executed.
    replacements={}
    for name,attributes in {
        'numpy':{},
        'measured_bridge':dict(observation=None,best_old_triplet=None,propose=None),
        'core_pixel_consumer':dict(strict_json=lambda data,limit:json.loads(data),require=require),
        'deferred_retry_window':dict(DeferredRetryWindow=None),
        'reference_policy':dict(historical_options=None,choose_candidate=None),
    }.items():
        module=types.ModuleType(name);module.__dict__.update(attributes);replacements[name]=module
    spec=importlib.util.spec_from_file_location('candidate_live_bridge_ack',Path(__file__).with_name('live_bridge.py'))
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,replacements):spec.loader.exec_module(module)
    return module.LiveBridge

LiveBridge=load_live_bridge()

class Sources:
    def check(self,role,incoming):
        require(role=='status' and incoming=='authenticated_core_gid','status publisher identity')
        return incoming

class ObservingAckTests(unittest.TestCase):
    def test_early_retry_is_independent_of_retired_core_switch(self):
        self.assertFalse(LiveBridge.early_retry_enabled({}))
        self.assertTrue(LiveBridge.early_retry_enabled({'WBROV_PRIVATE_EARLY_TAG_RETRY':'1'}))
        self.assertTrue(LiveBridge.early_retry_enabled({'WBROV_PRIVATE_EARLY_TAG_RETRY':'1','WBROV_PRIVATE_EARLY_GRAVITY_CERTIFICATE':'0'}))
        with self.assertRaisesRegex(ValueError,'invalid private early retry flag'):
            LiveBridge.early_retry_enabled({'WBROV_PRIVATE_EARLY_TAG_RETRY':'yes'})

    def test_current_core_certificate_rejection_can_be_retried(self):
        self.bridge.early_retry=True
        self.bridge.deferred=[]
        self.bridge.deferred_score=lambda geometry:(0.,0.,0.)
        status=deepcopy(self.status)
        status.update(result='rejected',reason='temporary recovery segment has not passed source-time/measured-gravity stability (8 nodes); main map remains frozen')
        self.send(status)
        self.assertEqual(len(self.bridge.deferred),1)
        self.assertIsNone(self.bridge.pending)
        self.assertFalse(self.bridge.committed_epochs)

    def setUp(self):
        self.config={'session_token_sha256':'session-hash'}
        self.proposal=dict(transaction_token='owned-token',tag_id=42,source_reset_epoch=2,
            observer_graph_revision=10,anchor_database_sha256='database-hash',created_monotonic_s=time.monotonic(),
            evidence=[dict(from_id=11,to_id=1),dict(from_id=12,to_id=2),dict(from_id=13,to_id=3)],
            accepted_images=['unchanged accepted receipts'],corner_observations=['unchanged raw corner evidence'])
        self.status=dict(schema_version=1,producer='wbrov_rtabmap_safe_relocalization_core',result='observing',
            reason='geometry prepared; graph remains uncommitted',transaction_token='owned-token',tag_id=42,
            source_reset_epoch=2,observer_graph_revision=10,core_reset_epoch=2,core_graph_revision=10,
            database_sha256='database-hash',session_token_sha256='session-hash',evidence_pairs=[[11,1],[12,2],[13,3]])
        self.bridge=LiveBridge.__new__(LiveBridge)
        self.bridge.consumer=types.SimpleNamespace(sources=Sources())
        self.bridge.config=self.config;self.bridge.pending=deepcopy(self.proposal)
        self.bridge.pending_geometry={'real_geometry_preserved':True};self.bridge.terminals={}
        self.bridge.committed_epochs=set();self.bridge.required_revision=0;self.bridge.deferred=[{'unchanged':True}]
        self.bridge.graph=types.SimpleNamespace(last_revision=10)
        self.events=[];self.bridge.event=lambda kind,**kw:self.events.append((kind,deepcopy(kw)))
    def send(self,status=None,gid='authenticated_core_gid'):
        self.bridge.status(types.SimpleNamespace(data=json.dumps(self.status if status is None else status)),gid)
    def test_observing_retains_pending_and_never_becomes_terminal(self):
        pending=self.bridge.pending;geometry=self.bridge.pending_geometry
        self.send()
        self.assertIs(self.bridge.pending,pending);self.assertIs(self.bridge.pending_geometry,geometry)
        self.assertEqual(self.bridge.pending,self.proposal);self.assertEqual(self.bridge.terminals,{})
        self.assertEqual(self.bridge.committed_epochs,set());self.assertEqual(self.bridge.required_revision,0)
        self.assertFalse(self.bridge.allow_ready(100,13,2))
    def test_repeated_observing_does_not_extend_original_timeout(self):
        created=self.bridge.pending['created_monotonic_s'];self.send();self.send()
        self.assertEqual(self.bridge.pending['created_monotonic_s'],created)
        self.assertEqual(sum(kind=='observing' for kind,_ in self.events),2)
        self.assertEqual(self.bridge.terminals,{})
        self.bridge.pending['created_monotonic_s']=time.monotonic()-13
        with self.assertRaisesRegex(ValueError,'Core proposal did not finish'):self.bridge.allow_ready(100,13,2)
    def test_observing_rejects_unowned_token_and_wrong_session_epoch_revision(self):
        for key,value in [('transaction_token','other'),('session_token_sha256','other'),
                          ('database_sha256','other'),('source_reset_epoch',3),('core_reset_epoch',3),
                          ('observer_graph_revision',11),('core_graph_revision',11),('tag_id',43)]:
            with self.subTest(key=key):
                invalid=dict(self.status,**{key:value})
                with self.assertRaises(ValueError):self.send(invalid)
                self.assertIsNotNone(self.bridge.pending);self.assertEqual(self.bridge.terminals,{})
    def test_observing_requires_owned_pending(self):
        self.bridge.pending=None
        with self.assertRaisesRegex(ValueError,'unowned terminal result'):self.send()
    def test_observing_keeps_source_authentication(self):
        with self.assertRaisesRegex(ValueError,'status publisher identity'):self.send(gid='unowned_gid')
        self.assertEqual(self.events,[])
    def test_observing_requires_exact_full_three_edges_and_schema(self):
        for pairs in ([],[[11,1]],[[11,1],[12,2],[13,4]],[[11,1],[12,2],[True,3]]):
            with self.subTest(pairs=pairs):
                with self.assertRaises(ValueError):self.send(dict(self.status,evidence_pairs=pairs))
        with self.assertRaises(ValueError):self.send(dict(self.status,unknown_field=True))
    def test_committed_once_after_observing_and_identical_terminal_retry(self):
        self.send();self.send()
        committed=dict(self.status,result='committed',core_graph_revision=11,reason='observations passed')
        self.send(committed);self.send(committed)
        self.assertEqual(self.bridge.terminals,{'owned-token':committed})
        self.assertIsNone(self.bridge.pending);self.assertIsNone(self.bridge.pending_geometry)
        self.assertEqual(self.bridge.committed_epochs,{2});self.assertEqual(self.bridge.required_revision,11)
        self.assertEqual(self.bridge.deferred,[])
        self.assertEqual(sum(kind=='terminal' for kind,_ in self.events),1)
    def test_conflicting_terminal_or_nonterminal_after_commit_rejected(self):
        self.send(dict(self.status,result='committed',core_graph_revision=11))
        for other in (self.status,dict(self.status,result='rejected')):
            with self.assertRaisesRegex(ValueError,'conflicting repeated terminal result'):self.send(other)
    def test_rejection_after_observing_clears_pending_without_commit(self):
        self.send();rejected=dict(self.status,result='rejected',reason='observation geometry failed')
        self.send(rejected)
        self.assertIsNone(self.bridge.pending);self.assertEqual(self.bridge.committed_epochs,set())
        self.assertEqual(self.bridge.terminals,{'owned-token':rejected})
    def test_terminal_validator_is_not_relaxed_for_observing(self):
        with self.assertRaisesRegex(ValueError,'terminal state'):validate_terminal(self.status,self.proposal,self.config)
        validate_observing(self.status,self.proposal,self.config)
        with self.assertRaises(ValueError):validate_observing(dict(self.status,result='committed'),self.proposal,self.config)

if __name__=='__main__':unittest.main()
