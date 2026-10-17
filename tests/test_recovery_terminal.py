"""Clean shutdown is independent of whether isolated maps could be registered."""
from copy import deepcopy
from pathlib import Path
import unittest

launcher = Path(__file__).resolve().parents[1]/'launch/deployed_20261015/start_rtabmap_calib_ab.sh'
body = launcher.read_text(encoding='utf-8').split("<<'CORNER_RECOVERY_STATE_PY'\n", 1)[1]
check = body.split("counts = result.get('counts')\n", 1)[1].split('\nCORNER_RECOVERY_STATE_PY', 1)[0]
check = "counts = result.get('counts')\n" + check
TOKEN = 'a'*64
DATABASE = '0'*64


def closed_result():
    # Captured shape from the human-assisted run: retained epochs 0 and 11,
    # zero proposals, intact authenticated sources and a completed shutdown.
    return dict(state='closed', fixture_mode=False, image_buffer_capacity=128,
                buffer_dropped={'rgb':149, 'camera_info':149}, ready_frames=119,
                graph_states=361, source_compromised=False, graph_state_rejections=0,
                counts={'bound':70, 'rejected':0}, graph_epochs=[0,11],
                committed_epochs=[], bridge_proposals=0, bridge_terminals={})


def terminal(outcome, epoch=2):
    return dict(transaction_token='test-transaction',
                producer='wbrov_rtabmap_safe_relocalization_core', schema_version=1,
                session_token_sha256=TOKEN, database_sha256=DATABASE,
                result=outcome, core_reset_epoch=epoch)


class RecoveryTerminalTests(unittest.TestCase):
    def accepts(self, result, cleanup=None):
        ns = dict(result=result, token_sha=TOKEN, database_sha=DATABASE,
                  cleanup=cleanup or {'primary_exception':None, 'cleanup_errors':[], 'bridge_complete':True})
        try:
            exec(compile(check, str(launcher), 'exec'), ns)
        except SystemExit as exc:
            return exc.code == 0
        self.fail('terminal checker did not exit')

    def test_unaligned_maps_are_a_clean_terminal_outcome(self):
        self.assertTrue(self.accepts(closed_result()))

    def test_authenticated_rejection_does_not_fail_cleanup(self):
        r=closed_result();r.update(bridge_proposals=1,
                                  bridge_terminals={'test-transaction':terminal('rejected')})
        self.assertTrue(self.accepts(r))
        self.assertEqual(r['committed_epochs'], [])

    def test_compacted_committed_epoch_is_accounted_by_terminal(self):
        r=closed_result();r.update(bridge_proposals=1, committed_epochs=[2],
                                  bridge_terminals={'test-transaction':terminal('committed')})
        self.assertTrue(self.accepts(r))
        r['committed_epochs']=[]
        self.assertFalse(self.accepts(r))

    def test_inflight_or_missing_terminal_is_not_clean(self):
        r=closed_result();r['bridge_proposals']=1
        self.assertFalse(self.accepts(r))
        r['bridge_terminals']={'test-transaction':terminal('observing')}
        self.assertFalse(self.accepts(r))

    def test_wrong_terminal_identity_is_rejected(self):
        base=closed_result();base.update(bridge_proposals=1,
            bridge_terminals={'test-transaction':terminal('rejected')})
        for key,value in [('session_token_sha256','wrong'),('database_sha256','wrong'),
                          ('transaction_token','other'),('producer','other'),('core_reset_epoch',True)]:
            r=deepcopy(base);r['bridge_terminals']['test-transaction'][key]=value
            with self.subTest(key=key):self.assertFalse(self.accepts(r))

    def test_source_or_cleanup_failure_remains_a_failure(self):
        for field,value in [('source_compromised',True),('graph_state_rejections',1),
                            ('graph_epochs',[0,True]),('committed_epochs',[True])]:
            r=closed_result();r[field]=value
            with self.subTest(field=field):self.assertFalse(self.accepts(r))
        self.assertFalse(self.accepts(closed_result(),
            {'primary_exception':None,'cleanup_errors':[],'bridge_complete':False}))


if __name__=='__main__':unittest.main()
