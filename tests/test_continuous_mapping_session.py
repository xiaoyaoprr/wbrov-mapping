"""Linux integration of capture stop -> backlog drain -> closed-save checks."""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))


@unittest.skipUnless(sys.platform=='linux','session ownership requires Linux /proc')
class SessionTest(unittest.TestCase):
    def test_optional_tag_configuration_reaches_parser_as_disabled(self):
        from continuous_mapping_session import processing_command
        import deferred_mapping
        config={n:'/fixture/'+n for n in ['supervisor','worker','verifier','odom_parameters','map_parameters']}
        for tag in (None,'','/fixture/tag_modules'):
            for omitted in (False,True):
                c=dict(config)
                if not omitted:c['tag_modules']=tag
                command=processing_command(c,Path('/out'),Path('/state'))
                with self.subTest(tag=tag,omitted=omitted),patch.object(sys,'argv',command[2:]),patch.object(deferred_mapping,'run',return_value=0) as run:
                    self.assertEqual(deferred_mapping.main(),0)
                    args=run.call_args.args[0]
                    self.assertEqual(args.tag_modules,Path(tag) if tag and not omitted else None)
                    self.assertTrue(args.save_closed_prefix)
                    self.assertEqual(args.worker_pid_file,Path('/state/slam.pid'))

    def test_interrupted_capture_drains_sealed_prefix_and_closes_worker(self):
        from deferred_mapping import run,atomic_json
        from argparse import Namespace
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);capture=root/'capture';capture.mkdir()
            for name in ['metadata.json','water_calibration.yml','init.json','runtime.json','sdk_version.txt','camera_imu_extrinsics.json']:
                (capture/name).write_text('{}')
            (capture/'segment-00000000.frames').write_bytes(b'input fixture')
            atomic_json(capture/'segment-00000000.json',{'schema':'wbrov.rgbd-spool-segment.v1','sequence':0,'frames':1,'bytes':13,'first_source_ns':10,'last_source_ns':10})
            atomic_json(capture/'status.json',{'schema':'wbrov.rgbd-spool.v1','state':'closed','clean_shutdown':False,'sealed_segments':1,'last_source_ns':10})
            worker=root/'worker';worker.write_text('''#!/usr/bin/python3
from pathlib import Path
import sys,json
out=Path(sys.argv[1]);out.mkdir()
for line in sys.stdin:
 if line.strip()=='CLOSE':
  (out/'map.db').write_bytes(b'closed test database')
  (out/'frames.csv').write_text('source_ns\\n10\\n')
  (out/'status.json').write_text(json.dumps({'database_closed':True}))
  print('WBROV_WORKER_CLOSED',flush=True);break
 d=json.loads(Path(line.strip()).read_text())
 (out/'main_cloud.ply').write_bytes(b'cloud fixture')
 print('WBROV_SEGMENT_COMPLETE 0 10',flush=True)
''');worker.chmod(0o700)
            parameter=root/'params';parameter.write_text('')
            args=Namespace(recording=capture,output=root/'processing',worker=worker,verifier=worker,
                odom_parameters=parameter,map_parameters=parameter,cv_threads=1,feature_prefetch=False,
                tag_modules=None,first_index=0,last_index=None,segment_timeout=5,save_closed_prefix=True)
            self.assertEqual(run(args),2)
            saved=json.loads((args.output/'save.json').read_text())
            self.assertFalse(saved['complete']);self.assertTrue(saved['artifacts_saved'])
            self.assertEqual(json.loads((args.output/'status.json').read_text())['state'],'interrupted_saved')
            self.assertEqual((args.output/'mapping/map.db').read_bytes(),b'closed test database')

    def test_stop_does_not_kill_processing_and_save_checks_closed_inputs(self):
        from continuous_mapping_session import Session,atomic_json
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);state=root/'state';state.mkdir();out=root/'run';out.mkdir()
            s=Session({},state,out,os.getpid())
            # A real child takes longer than the stop request to finish. SIGUSR1
            # is handled by the owner, never forwarded to the processing child.
            child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(.3)'])
            s.children['processing']=child;s.request_stop();s.request_stop()
            self.assertIsNone(child.poll());self.assertEqual(child.wait(timeout=5),0)
            s.lifecycle('stopping');v=json.loads((state/'lifecycle.json').read_text())
            self.assertFalse(v['export_succeeded']);self.assertEqual(v['launcher_pid'],os.getpid())
            processing=out/'processing';mapping=processing/'mapping';mapping.mkdir(parents=True)
            capture=out/'rgbd_spool';capture.mkdir()
            import sqlite3
            db=sqlite3.connect(mapping/'map.db');db.execute('create table test(x)');db.commit();db.close()
            (mapping/'main_cloud.ply').write_bytes(b'closed cloud')
            pins={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in mapping.iterdir()}
            atomic_json(processing/'save.json',{'complete':True,'sha256':pins})
            st={'state':'closed','completed_segments':2,'mapping':{'database_closed':True,'cloud_points':10}}
            atomic_json(processing/'status.json',st)
            atomic_json(capture/'status.json',{'state':'closed','sealed_segments':3,'clean_shutdown':True})
            with self.assertRaisesRegex(ValueError,'unprocessed'):s.verify_save()
            st['completed_segments']=3;atomic_json(processing/'status.json',st)
            self.assertTrue(s.verify_save()['complete'])
            (mapping/'main_cloud.ply').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'identity'):s.verify_save()

    def test_exit_wait_observes_cmdline_disappearance_without_signalling(self):
        from render_continuous_entry import render
        repo=Path(__file__).resolve().parents[1]
        _,wrapper=render(Path('/tmp/frozen-map'),'a'*64,'b'*64,(repo/'launch/candidate_receipt_resume/run_water_mapping_with_light.sh').read_text(),'c'*64)
        function=wrapper[wrapper.index('wait_same() {'):wrapper.index('\nwait_mapping_launcher()')]
        cases=[('transient',0,1),('unknown',2,3),('running',1,3),('gone',0,0)]
        for mode,expected,sleeps in cases:
            script=function+'''
n=0
same_process() { [[ "$mode" == running ]]; }
pid_may_be_running() { [[ "$mode" == unknown || "$mode" == running || ( "$mode" == transient && "$n" == 0 ) ]]; }
sleep() { n=$((n+1)); }
kill() { echo forbidden-signal; exit 88; }
wait_same 123 456 pattern 3
result=$?
printf '%s %s\\n' "$result" "$n"
'''
            q=subprocess.run(['/usr/bin/bash','-c',script],env=dict(os.environ,mode=mode),capture_output=True,text=True,timeout=5)
            self.assertEqual(q.returncode,0,q.stderr)
            self.assertEqual(q.stdout.strip(),f'{expected} {sleeps}')

    def test_renderer_keeps_teleop_exit_before_unbounded_mapping_drain(self):
        from render_continuous_entry import render
        repo=Path(__file__).resolve().parents[1]
        launcher,wrapper=render(Path('/tmp/frozen-map'),'a'*64,'b'*64,(repo/'launch/candidate_receipt_resume/run_water_mapping_with_light.sh').read_text(),'c'*64)
        self.assertIn("readonly FINAL_RESET_RUNTIME_NODE_SHA256='"+'c'*64+"'",wrapper)
        self.assertLess(wrapper.index("' TERM; then",wrapper.index('cleanup()')),wrapper.index('if wait_mapping_launcher 0; then'))
        self.assertIn('! pid_may_be_running "$mapping_pid" && ! signal_mapping_remainders',wrapper)
        self.assertIn('expected_mapping_children=(bridge processing graph_display slam rviz)',wrapper)
        self.assertNotIn('enqueue --run',wrapper)
        with tempfile.TemporaryDirectory() as tmp:
            for name,data in [('launcher.sh',launcher),('wrapper.sh',wrapper)]:
                p=Path(tmp)/name;p.write_text(data)
                self.assertEqual(subprocess.run(['/usr/bin/bash','-n',str(p)],capture_output=True).returncode,0)


if __name__=='__main__':unittest.main()
