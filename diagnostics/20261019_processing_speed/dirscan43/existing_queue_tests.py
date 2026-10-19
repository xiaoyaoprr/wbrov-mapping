import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
import threading
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from deferred_mapping import OrderedReceipts,closed_inputs,atomic_json,atomic_status_snapshot,descriptor,PreparedInput,sealed_lookahead,SealedLookaheadWindow

class DeferredQueueTest(unittest.TestCase):
    def test_progress_snapshot_is_complete_atomic_telemetry_without_sync(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'status.json'
            for state in ('waiting_for_recording','processing'):
                value={'state':state,'acknowledged_source_ns':1791653820000000123,'mapping':{'database_closed':False}}
                with patch('deferred_mapping.os.fsync',side_effect=AssertionError('progress must not force a sync')):
                    atomic_status_snapshot(path,value)
                self.assertEqual(json.loads(path.read_text()),value)
                self.assertFalse(path.with_suffix('.tmp').exists())
            before=path.read_bytes()
            with self.assertRaises(ValueError):atomic_status_snapshot(path,{'state':'processing','bad':float('nan')})
            self.assertEqual(path.read_bytes(),before)

    def test_terminal_or_unknown_status_sync_failure_keeps_previous_snapshot(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'status.json';atomic_status_snapshot(path,{'state':'processing'})
            before=path.read_bytes()
            for state in ('closed','interrupted_saved','failed','future_terminal',None):
                with self.subTest(state=state),patch('deferred_mapping.os.fsync',side_effect=OSError('injected storage failure')):
                    with self.assertRaises(OSError):atomic_status_snapshot(path,{'state':state})
                self.assertEqual(path.read_bytes(),before)
            with patch('deferred_mapping.os.fsync',wraps=os.fsync) as synced:
                atomic_status_snapshot(path,{'state':'closed','completed_segments':4})
            self.assertEqual(synced.call_count,1 if os.name=='nt' else 2)
            self.assertEqual(json.loads(path.read_text())['state'],'closed')

    def test_command_receipt_session_and_save_writes_remain_durable_by_default(self):
        with tempfile.TemporaryDirectory() as td:
            for name in ('input-00000000.json','receipt-00000000.json','session.json','save.json'):
                path=Path(td)/name;path.write_text('{"old":true}\n')
                with self.subTest(name=name),patch('deferred_mapping.os.fsync',side_effect=OSError('injected storage failure')):
                    with self.assertRaises(OSError):atomic_json(path,{'state':'processing','new':True})
                self.assertEqual(json.loads(path.read_text()),{'old':True})

    def test_wider_lookahead_keeps_contiguous_pinned_overlap_and_never_acks(self):
        window=SealedLookaheadWindow(3);paths=[Path(str(n)) for n in range(8)];receipts=OrderedReceipts()
        def describe(path,n,verifier):return {'kind':'rgbd_spool','frames':30,'sequence':n,'sha256':{str(path):str(n)}}
        with patch('deferred_mapping.closed_inputs',return_value=(paths,False,{})),patch('deferred_mapping.descriptor',side_effect=describe) as reader:
            first=window.collect(Path('.'),0,None);self.assertEqual([d['sequence'] for d in first],[1,2,3])
            second=window.collect(Path('.'),1,None);self.assertEqual([d['sequence'] for d in second],[2,3,4])
            self.assertIs(first[1],second[0]);self.assertEqual(reader.call_count,4)
            self.assertEqual(receipts.next_sequence,0)
            last=window.collect(Path('.'),7,None);self.assertEqual(last,[]);self.assertEqual(window.cache,{})

    def test_wider_lookahead_does_not_jump_over_unusable_segment(self):
        paths=[Path(str(n)) for n in range(5)];window=SealedLookaheadWindow(3)
        def describe(path,n,verifier):return {'kind':'rgbd_spool','frames':65 if n==2 else 30,'sequence':n}
        with patch('deferred_mapping.closed_inputs',return_value=(paths,False,{})),patch('deferred_mapping.descriptor',side_effect=describe) as reader:
            self.assertEqual([d['sequence'] for d in window.collect(Path('.'),0,None)],[1]);self.assertEqual(reader.call_count,2)
        for width in (0,4,-1):
            with self.assertRaises(ValueError):SealedLookaheadWindow(width)

    def test_native_lookahead_is_one_sealed_bounded_input_and_never_an_ack(self):
        receipts=OrderedReceipts();paths=[Path('first'),Path('second')]
        with patch('deferred_mapping.closed_inputs',return_value=(paths,False,{})),patch('deferred_mapping.descriptor') as describe:
            describe.return_value={'kind':'rgbd_spool','frames':64,'sequence':1}
            self.assertEqual(sealed_lookahead(Path('.'),0,None)['sequence'],1)
            describe.assert_called_once_with(paths[1],1,None)
            self.assertIsNone(sealed_lookahead(Path('.'),1,None))
            describe.return_value={'kind':'rgbd_spool','frames':65}
            self.assertIsNone(sealed_lookahead(Path('.'),0,None))
            self.assertEqual(receipts.next_sequence,0)

    def test_preparation_overlaps_without_advancing_or_skipping_receipts(self):
        entered=threading.Event();release=threading.Event();receipts=OrderedReceipts()
        def prepare(path,sequence):
            entered.set();assert release.wait(2);return (path,sequence)
        pending=PreparedInput(prepare)
        try:
            pending.schedule(0,Path('sealed'));self.assertTrue(entered.wait(1))
            self.assertEqual(receipts.next_sequence,0)
            with self.assertRaises(ValueError):pending.take(1,Path('sealed'))
            with self.assertRaises(ValueError):pending.schedule(1,Path('next'))
            release.set();self.assertEqual(pending.take(0,Path('sealed')),(Path('sealed'),0))
            self.assertEqual(receipts.next_sequence,0)
        finally:release.set();pending.close()

    def test_preparation_failure_cannot_be_treated_as_completed_input(self):
        def fail(*_):raise ValueError('changed input')
        pending=PreparedInput(fail)
        try:
            pending.schedule(0,Path('sealed'))
            with self.assertRaisesRegex(ValueError,'changed input'):pending.take(0,Path('sealed'))
        finally:pending.close()
    def test_startup_waits_and_only_sealed_spool_is_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            self.assertEqual(closed_inputs(root)[:2],([],False))
            state={'schema':'wbrov.rgbd-spool.v1','state':'recording','sealed_segments':0}
            (root/'segment-00000000.partial').write_bytes(b'unfinished')
            atomic_json(root/'status.json',state)
            self.assertEqual(closed_inputs(root)[:2],([],False))
            state['sealed_segments']=1;atomic_json(root/'status.json',state)
            with self.assertRaises(ValueError):closed_inputs(root)
            (root/'segment-00000000.json').write_text('{}')
            self.assertEqual(closed_inputs(root)[:2],([root/'segment-00000000.json'],False))
            state['state']='failed';atomic_json(root/'status.json',state)
            with self.assertRaises(ValueError):closed_inputs(root)

    def test_configuration_change_and_regressed_interval_cannot_advance(self):
        q=OrderedReceipts()
        pins={n:'a'*64 for n in ['init.json','runtime.json','water_calibration.yml','camera_imu_extrinsics.json','sdk_version.txt']}
        d={'sequence':0,'first_source_ns':'10','last_source_ns':'20','sha256':pins}
        q.accept(d,'WBROV_SEGMENT_COMPLETE 0 20')
        changed=dict(pins);changed['water_calibration.yml']='b'*64
        with self.assertRaises(ValueError):q.validate_input({'sequence':1,'first_source_ns':'21','last_source_ns':'30','sha256':changed})
        with self.assertRaises(ValueError):q.validate_input({'sequence':1,'first_source_ns':'21','last_source_ns':'19','sha256':pins})
        self.assertEqual(q.source_ns,20)

    def test_exact_ack_only_and_no_skipped_input(self):
        q=OrderedReceipts()
        d={'sequence':0,'first_source_ns':'1000000001','last_source_ns':'2000000002'}
        for wrong in ['WBROV_SEGMENT_COMPLETE 1 2000000002','WBROV_SEGMENT_COMPLETE 0 2000000003','WBROV_WORKER_CLOSED']:
            with self.assertRaises(ValueError):q.accept(d,wrong)
            self.assertEqual(q.next_sequence,0)
            self.assertEqual(q.source_ns,0)
        q.accept(d,'WBROV_SEGMENT_COMPLETE 0 2000000002')
        with self.assertRaises(ValueError):q.validate_input(d)
        with self.assertRaises(ValueError):q.validate_input({'sequence':2,'first_source_ns':'3000000000'})
        with self.assertRaises(ValueError):q.validate_input({'sequence':1,'first_source_ns':'2000000002'})
        q.validate_input({'sequence':1,'first_source_ns':'2000000003'})

    def test_closed_only_and_hole_is_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            state={'schema':'wbrov.raw-segments.v1','state':'recording','sealed_segments':0}
            atomic_json(root/'status.json',state)
            (root/'segment-00000000').mkdir()
            self.assertEqual(closed_inputs(root)[:2],([],False))
            state['sealed_segments']=1;atomic_json(root/'status.json',state)
            self.assertEqual(closed_inputs(root)[:2],([root/'segment-00000000'],False))
            state['sealed_segments']=2;atomic_json(root/'status.json',state)
            with self.assertRaises(ValueError):closed_inputs(root)
            (root/'segment-00000001').mkdir()
            state['state']='closed';atomic_json(root/'status.json',state)
            self.assertEqual(len(closed_inputs(root)[0]),2)
            self.assertTrue(closed_inputs(root)[1])

    def test_unclosed_legacy_recording_is_not_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            atomic_json(root/'status.json',{'schema':'wbrov.raw-recording.v1','state':'recording'})
            self.assertEqual(closed_inputs(root)[:2],([],False))
            atomic_json(root/'status.json',{'schema':'wbrov.raw-recording.v1','state':'closed'})
            self.assertEqual(closed_inputs(root)[:2],([root],True))

if __name__=='__main__':unittest.main()
