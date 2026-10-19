#!/usr/bin/env python3
"""Consume closed recordings in source order; retain all input and failed work.

This is a recording-time mapper, not a realtime pose publisher. A restart uses
a NEW attempt directory and replays from the first input: an odometry process
cannot resume from a JSON cursor without reconstructing its internal state.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import queue
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from verify_raw_recording import sha, verify


def atomic_json(path, value, *, durable=True):
    temp = path.with_suffix('.tmp')
    with temp.open('w', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, allow_nan=False, indent=2)
        f.write('\n'); f.flush()
        if durable: os.fsync(f.fileno())
    os.replace(temp, path)
    if durable and os.name != 'nt':
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)


def atomic_status_snapshot(path, value):
    # Progress is replaceable telemetry. Restart always creates a fresh attempt;
    # its authority comes from input/segment receipts and the final save record.
    # Terminal and unknown states retain the same durable publication contract.
    atomic_json(path, value, durable=value.get('state') not in ('processing','waiting_for_recording'))


def closed_inputs(root):
    """An open or absent final segment is never treated as an EOF."""
    if not (root/'status.json').exists():
        return [], False, {}
    status = json.loads((root/'status.json').read_text())
    if status['schema'] == 'wbrov.raw-recording.v1':
        return ([root] if status['state'] == 'closed' else []), status['state'] == 'closed', status
    if status['schema'] not in ('wbrov.raw-segments.v1','wbrov.rgbd-spool.v1'):
        raise ValueError('unsupported recording schema')
    count = status['sealed_segments']
    if type(count) is not int or count < 0:
        raise ValueError('invalid sealed segment count')
    spool=status['schema']=='wbrov.rgbd-spool.v1'
    if status['state']=='failed':raise ValueError('recording spool writer failed; incomplete input retained')
    paths = [root/(f'segment-{i:08d}.json' if spool else f'segment-{i:08d}') for i in range(count)]
    # Fresh directory inventory on every call; never cache across seals or polls.
    # Only names/type checks are batched. Per-segment content hashing and native
    # before/after consumption checks remain unchanged.
    entries = {}
    if paths:
        expected_names = {p.name for p in paths}
        with os.scandir(root) as directory:
            for entry in directory:
                if entry.name in expected_names:
                    entries[entry.name] = entry
                    if len(entries) == len(expected_names):
                        break
    for p in paths:
        entry = entries.get(p.name)
        if (entry is None or entry.is_symlink() or
                not (entry.is_file() if spool else entry.is_dir()) or p.parent != root):
            raise ValueError('missing or redirected recording segment: '+p.name)
    return paths, status['state'] == 'closed', status


def descriptor(root, sequence, verifier, first=0, last=None):
    if root.suffix=='.json':
        if first or last is not None:raise ValueError('diagnostic frame selection cannot skip a live spool')
        seal=json.loads(root.read_text());directory=root.parent
        if seal.get('schema')!='wbrov.rgbd-spool-segment.v1' or seal.get('sequence')!=sequence or seal.get('frames',0)<=0:
            raise ValueError('invalid RGB-D spool seal')
        frame=root.with_suffix('.frames')
        if frame.is_symlink() or not frame.is_file():raise ValueError('invalid spool data file')
        if frame.stat().st_size!=seal['bytes']:raise ValueError('spool size differs from sealed receipt')
        names=[root.name,frame.name,'metadata.json','water_calibration.yml','init.json','runtime.json','sdk_version.txt','camera_imu_extrinsics.json']
        return {'kind':'rgbd_spool','sequence':sequence,'directory':str(directory),'data_file':frame.name,
                'sha256':{n:sha(directory/n) for n in names},'frames':seal['frames'],
                'first_source_ns':str(seal['first_source_ns']),'last_source_ns':str(seal['last_source_ns'])}
    v = verify(root, verifier)
    stamps = []
    with (root/'capture.jsonl').open() as f:
        for line in f:
            stamp = json.loads(line)['stamp_ns']
            if stamps and stamp < stamps[-1]:
                raise ValueError('source time regressed in capture ledger')
            if not stamps or stamp != stamps[-1]: stamps.append(stamp)
    if len(stamps) != v['frames']['frames']:
        raise ValueError('indexed frame count does not match source ledger')
    if last is None: last = len(stamps)-1
    if not 0 <= first <= last < len(stamps): raise ValueError('invalid replay interval')
    return {'sequence':sequence, 'directory':str(root), 'sdk_version':v['sdk_version'],
            'sha256':v['sha256'], 'first_index':first, 'last_index':last,
            # Strings preserve all original integer nanoseconds in OpenCV JSON.
            'first_source_ns':str(stamps[first]), 'last_source_ns':str(stamps[last])}


def sealed_lookahead(root, sequence, verifier):
    paths,_,_=closed_inputs(root)
    if sequence+1<len(paths):
        following=descriptor(paths[sequence+1],sequence+1,verifier)
        if following.get('kind')=='rgbd_spool' and following['frames']<=64:return following
    return None


class SealedLookaheadWindow:
    """Bounded pinned descriptors only; current input is still freshly hashed.

    Cache overlapping future descriptors so widening speculative odometry does
    not multiply large-file hashing. Native checks every newly prepared input
    before/after estimation, and again before/after actual map consumption.
    """
    def __init__(self, width):
        if width not in (1,2,3):raise ValueError('invalid lookahead segment budget')
        self.width=width;self.cache={}

    def collect(self, root, sequence, verifier):
        paths,_,_=closed_inputs(root)
        self.cache={n:v for n,v in self.cache.items() if sequence<n<=sequence+self.width}
        result=[]
        for n in range(sequence+1,min(len(paths),sequence+self.width+1)):
            if n in self.cache:
                path,d=self.cache[n]
                if path!=paths[n]:raise ValueError('prepared future segment path changed')
            else:
                d=descriptor(paths[n],n,verifier)
                if d.get('kind')!='rgbd_spool' or not 1<=d['frames']<=64:break
                self.cache[n]=(paths[n],d)
            result.append(d)
        return result


class OrderedReceipts:
    def __init__(self):
        self.next_sequence = 0
        self.source_ns = 0
        self.configuration = None

    def validate_input(self, d):
        if d['sequence'] != self.next_sequence or int(d['first_source_ns']) <= self.source_ns:
            raise ValueError('out-of-order or overlapping recording input')
        if int(d.get('last_source_ns',d['first_source_ns'])) < int(d['first_source_ns']):
            raise ValueError('recorded interval regressed')
        if 'sha256' in d:
            configuration={n:d['sha256'][n] for n in ['init.json','runtime.json','water_calibration.yml',
                'camera_imu_extrinsics.json','sdk_version.txt']}
            if self.configuration is not None and configuration != self.configuration:
                raise ValueError('recording configuration changed between segments')

    def accept(self, d, line):
        self.validate_input(d)
        expected = f"WBROV_SEGMENT_COMPLETE {self.next_sequence} {d['last_source_ns']}"
        if line != expected:
            raise ValueError('worker completion does not match exact recording interval')
        self.next_sequence += 1
        self.source_ns = int(d['last_source_ns'])
        if 'sha256' in d:
            self.configuration={n:d['sha256'][n] for n in ['init.json','runtime.json','water_calibration.yml',
                'camera_imu_extrinsics.json','sdk_version.txt']}


class PreparedInput:
    """At most one sealed input prepared ahead; completion never acknowledges it."""
    def __init__(self, prepare):
        self.prepare=prepare;self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='sealed-input')
        self.pending=None

    def schedule(self, sequence, path):
        if self.pending is not None:raise ValueError('unconsumed prepared input')
        self.pending=(sequence,path,self.pool.submit(self.prepare,path,sequence))

    def take(self, sequence, path):
        if self.pending is None:self.schedule(sequence,path)
        expected,source,future=self.pending
        if (sequence,path)!=(expected,source):raise ValueError('prepared input sequence mismatch')
        result=future.result();self.pending=None;return result

    def close(self):
        self.pool.shutdown(wait=True,cancel_futures=True)


def run(args):
    root=args.recording.resolve(strict=True)
    out=args.output.resolve()
    out.mkdir(mode=0o700)  # no reuse of cursor with a fresh odometry instance
    (out/'optimizer_trace').mkdir(mode=0o700)
    bindings={n:sha(p) for n,p in [('worker',args.worker),('verifier',args.verifier),
                                 ('odom_parameters',args.odom_parameters),('map_parameters',args.map_parameters)]}
    bindings['cv_threads']=args.cv_threads
    bindings['feature_prefetch']=args.feature_prefetch
    lookahead_segments=getattr(args,'lookahead_segments',1)
    bindings['lookahead_segments']=lookahead_segments
    future_descriptors=SealedLookaheadWindow(lookahead_segments)
    tag_observer=None
    if args.tag_modules:
        from deferred_tag_observations import TagObserver
        tag_observer=TagObserver(args.tag_modules)
        bindings['tag_modules']={p.name:sha(p) for p in sorted(args.tag_modules.glob('*.py'))}
    session_hash=hashlib.sha256(json.dumps({'inputs':str(root),'attempt':str(out),'bindings':bindings},sort_keys=True).encode()).hexdigest()
    atomic_json(out/'session.json',{'schema':'wbrov.deferred-session.v1','session_sha256':session_hash,'bindings':bindings,
        'historical_only':True,'navigation_authorized':False,'restart_policy':'rebuild_from_first_recorded_frame',
        'diagnostic_range':args.first_index!=0 or args.last_index is not None})
    receipts=OrderedReceipts();process=None;events=queue.Queue();reader_thread=None
    log=(out/'worker.log').open('x',encoding='utf-8')
    latest=0
    interrupted=False
    waiting_since=time.monotonic()

    def prepare(directory,sequence):
        d=descriptor(directory,sequence,args.verifier,args.first_index,args.last_index)
        if d.get('kind')=='rgbd_spool' and tag_observer:
            if bindings['tag_modules']!={p.name:sha(p) for p in sorted(args.tag_modules.glob('*.py'))}:
                raise ValueError('measured-corner implementation changed during processing')
            d['tag_observations']=tag_observer.observe(Path(d['directory'])/d['data_file'],d['frames'])
        if d.get('kind')=='rgbd_spool':
            if lookahead_segments==1:
                following=sealed_lookahead(root,sequence,args.verifier)
                if following is not None:d['odometry_lookahead']=following
            else:
                following=future_descriptors.collect(root,sequence,args.verifier)
                if following:d['odometry_lookahead_window']=following
        return d
    prepared=PreparedInput(prepare)

    def status(state, error=None):
        nonlocal latest
        capture_state={}
        capture_path=root/'status.json'
        if capture_path.exists():
            capture_state=json.loads(capture_path.read_text())
            latest=max(latest,int(capture_state.get('last_source_ns',0)))
            if capture_state.get('schema')=='wbrov.raw-segments.v1' and capture_state.get('state')=='recording':
                active=root/f"segment-{capture_state['sealed_segments']:08d}"/'status.json'
                if active.exists():latest=max(latest,int(json.loads(active.read_text()).get('last_source_ns',0)))
        worker_path=out/'mapping/status.json'
        worker=json.loads(worker_path.read_text()) if worker_path.exists() else {}
        done=int(worker.get('processed_source_ns',receipts.source_ns))
        atomic_status_snapshot(out/'status.json',{'schema':'wbrov.deferred-queue.v1','state':state,'error':error,
            'historical_only':True,'navigation_authorized':False,'recorded_source_ns':latest,
            'recording_state':capture_state.get('state','waiting'),
            'processed_source_ns':done,'acknowledged_source_ns':receipts.source_ns,
            'verified_map_source_ns':worker.get('verified_map_source_ns',0),
            'backlog_seconds':max(0,latest-done)/1e9 if done else None,
            'completed_segments':receipts.next_sequence,'mapping':worker})

    def reader():
        try:
            for line in process.stdout:
                log.write(line);log.flush()
                value=line.strip()
                if value.startswith('WBROV_SEGMENT_COMPLETE ') or value=='WBROV_WORKER_CLOSED':events.put(value)
        finally:events.put(None)

    try:
        status('waiting_for_recording')
        while True:
            paths,closed,capture=closed_inputs(root)
            latest=int(capture.get('last_source_ns',latest))
            if receipts.next_sequence == len(paths):
                if closed:
                    if capture.get('clean_shutdown') is not True:
                        if not getattr(args,'save_closed_prefix',False):raise ValueError('capture was not cleanly closed; prefix retained')
                        interrupted=True
                    break
                if process is None and time.monotonic()-waiting_since > args.segment_timeout:
                    raise TimeoutError('no sealed recording input before startup deadline')
                status('waiting_for_recording');time.sleep(.2);continue
            directory=paths[receipts.next_sequence]
            if capture['schema']=='wbrov.raw-segments.v1' and (args.first_index or args.last_index is not None):
                raise ValueError('diagnostic ranges require a single closed SVO')
            d=prepared.take(receipts.next_sequence,directory)
            receipts.validate_input(d)
            command=out/f'input-{receipts.next_sequence:08d}.json';atomic_json(command,d)
            if process is None:
                for name,path in [('worker',args.worker),('odom_parameters',args.odom_parameters),('map_parameters',args.map_parameters)]:
                    if bindings[name]!=sha(path):raise ValueError(name+' changed before start')
                env=dict(os.environ,WBROV_DEPTH_RECOVERY='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
                         WBROV_PRIVATE_OPTIMIZER_TRACE=str(out/'optimizer_trace'))
                env.pop('DISPLAY',None);env.pop('WAYLAND_DISPLAY',None)
                process=subprocess.Popen([str(args.worker),str(out/'mapping'),str(args.odom_parameters),str(args.map_parameters),
                    session_hash,d['sha256'].get('stereo.svo2',d['sha256'].get('metadata.json')),str(args.cv_threads),'1' if args.feature_prefetch else '0'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                    text=True,encoding='utf-8',errors='replace',bufsize=1,env=env)
                if getattr(args,'worker_pid_file',None):
                    target=args.worker_pid_file
                    temp=target.with_suffix('.tmp')
                    with temp.open('w') as f:
                        os.fchmod(f.fileno(),0o600);f.write(str(process.pid)+'\n');f.flush();os.fsync(f.fileno())
                    temp.replace(target)
                reader_thread=threading.Thread(target=reader,daemon=True);reader_thread.start()
            process.stdin.write(str(command)+'\n');process.stdin.flush()
            # Overlap hashing/corner extraction with native processing. Native
            # ACK order and before/after input verification remain unchanged.
            # SDK/SVO verifier calls stay serial; only immutable RGB-D spools
            # are prepared concurrently, with one bounded pending descriptor.
            following=receipts.next_sequence+1
            if d.get('kind')=='rgbd_spool' and following<len(paths):prepared.schedule(following,paths[following])
            started=time.monotonic()
            while True:
                status('processing')
                try: event=events.get(timeout=.5)
                except queue.Empty:
                    if time.monotonic()-started > args.segment_timeout:raise TimeoutError('segment computation timed out; input retained')
                    continue
                if event is None:raise RuntimeError('worker exited before segment completion')
                # Rehash after consumption. Do not acknowledge changed inputs.
                for name,pin in d['sha256'].items():
                    if sha(Path(d['directory'])/name)!=pin:raise ValueError('recorded input changed during replay')
                receipts.accept(d,event)
                receipt={'schema':'wbrov.deferred-segment-receipt.v1','sequence':d['sequence'],
                    'last_source_ns':d['last_source_ns'],'input_manifest_sha256':sha(command),
                    'worker_sha256':bindings['worker'],'cloud_sha256':sha(out/'mapping/main_cloud.ply'),
                    'tags_sha256':sha(out/'mapping/tags.json') if (out/'mapping/tags.json').exists() else None,
                    'historical_only':True,'database_closed':False}
                atomic_json(out/f'receipt-{d["sequence"]:08d}.json',receipt)
                break
        if process:
            process.stdin.write('CLOSE\n');process.stdin.flush();process.stdin.close()
            process.wait(timeout=120)
            if process.returncode:raise RuntimeError('worker close failed')
            saved=json.loads((out/'mapping/status.json').read_text())
            if saved.get('database_closed') is not True:raise RuntimeError('database close not confirmed')
            artifacts=['map.db','main_cloud.ply','frames.csv']
            if (out/'mapping/tags.json').exists():artifacts.append('tags.json')
            for name in artifacts:
                with (out/'mapping'/name).open('rb') as f:os.fsync(f.fileno())
            atomic_json(out/'save.json',{'complete':not interrupted,'artifacts_saved':True,'recording_complete':not interrupted,
                'partial_reason':'capture interrupted; only sealed segments processed' if interrupted else None,
                'historical_only':True,'recovery_acceptance_passed':False,
                'sha256':{n:sha(out/'mapping'/n) for n in artifacts}})
        else:raise ValueError('no closed recording data')
        status('interrupted_saved' if interrupted else 'closed');return 2 if interrupted else 0
    except BaseException as e:
        status('failed',f'{type(e).__name__}: {e}')
        if process and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        raise
    finally:
        prepared.close()
        # reader has drained before EOF/exit in normal operation; no raw input
        # is removed, including failed and not-yet-acknowledged segments.
        if reader_thread:reader_thread.join(timeout=10)
        if process and process.stdout:process.stdout.close()
        log.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ['recording','output','worker','verifier','odom-parameters','map-parameters']:
        p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--segment-timeout',type=float,default=600)
    p.add_argument('--save-closed-prefix',action='store_true',help='close and hash a sealed prefix after interrupted capture; never report complete')
    p.add_argument('--first-index',type=int,default=0)
    p.add_argument('--last-index',type=int)
    p.add_argument('--tag-modules',type=Path)
    p.add_argument('--cv-threads',type=int,choices=(1,2,4),default=1)
    p.add_argument('--feature-prefetch',action='store_true',help='prepare sealed RGB-D features on a bounded separate lane')
    p.add_argument('--lookahead-segments',type=int,choices=[1,2,3],default=1,help='bounded sealed segments for speculative odometry; each frame still processed and individually checked')
    p.add_argument('--worker-pid-file',type=Path,help='private launcher PID receipt; does not change frame processing')
    return run(p.parse_args())

if __name__=='__main__':raise SystemExit(main())
