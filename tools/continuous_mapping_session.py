#!/usr/bin/env python3
"""Own capture, ordered processing, display and save. Never owns robot control.

Stopping seals capture first. The worker drains all sealed input before database
verification. Closing RViz requests the same path; repeated stop signals do not
terminate the processing queue. The outer teleop wrapper stops control first.
"""
from pathlib import Path
import argparse
import fcntl
import hashlib
import json
import os
import signal
import shutil
import sqlite3
import subprocess
import time
from deferred_mapping import atomic_json


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def ticks(pid):
    return int(Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19])


def secure_directory(path):
    path=Path(path)
    path.mkdir(mode=0o700,parents=True,exist_ok=True)
    if path.is_symlink() or path.resolve()!=path or path.stat().st_uid!=os.getuid() or path.stat().st_mode&0o777!=0o700:
        raise ValueError('private directory identity/permissions invalid: '+str(path))
    return path


def atomic_pid(path,pid):
    temp=path.with_suffix('.tmp')
    with temp.open('w') as f:
        os.fchmod(f.fileno(),0o600);f.write(str(pid)+'\n');f.flush();os.fsync(f.fileno())
    temp.replace(path)


def load_config(path,pin):
    if path.is_symlink() or sha(path)!=pin:raise ValueError('session configuration changed')
    c=json.loads(path.read_text())
    if c['schema']!='wbrov.continuous-entry.v1':raise ValueError('unsupported entry schema')
    for name,digest in c['sha256'].items():
        p=Path(name)
        if not p.is_absolute() or not p.is_file() or p.is_symlink() or sha(p)!=digest:
            raise ValueError('entry input changed: '+name)
    for name in ['camera','worker','verifier','supervisor','display','rviz','rviz_config','raw_verify','odom_parameters','map_parameters','calibration']:
        if c[name] not in c['sha256']:raise ValueError('unbound entry input: '+name)
    return c


def processing_command(c,out,state):
    command=['/usr/bin/python3','-B',c['supervisor'],'--recording',str(out/'rgbd_spool'),
        '--output',str(out/'processing'),'--worker',c['worker'],'--verifier',c['verifier'],
        '--odom-parameters',c['odom_parameters'],'--map-parameters',c['map_parameters'],
        '--worker-pid-file',str(state/'slam.pid'),'--save-closed-prefix']
    # argparse's Path('') becomes '.', which is truthy and would import the
    # Tag observer even though the configuration explicitly disabled it.
    if c.get('tag_modules'):
        command+=['--tag-modules',c['tag_modules']]
    return command


class Session:
    def __init__(self,config,state,output,launcher_pid):
        self.config=config;self.state=state;self.output=output;self.launcher_pid=launcher_pid
        self.launcher_ticks=ticks(launcher_pid);self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.stop_requested=False;self.children={};self.logs=[];self.errors=[];self.phase='starting'
        self.saved=False;self.partial_saved=False;self.start=time.monotonic()
        self.capture_source=0;self.capture_updated=self.start;self.capture_state='starting'

    def request_stop(self,*_):
        self.stop_requested=True

    def lifecycle(self,phase,complete=False):
        value={'schema_version':1,'launcher_pid':self.launcher_pid,'process_start_ticks':self.launcher_ticks,
            'boot_id':self.boot,'phase':phase,'exit_code':int(bool(self.errors)),
            'nodes_stopped':complete,'export_succeeded':complete,'cleanup_succeeded':complete,
            'run_dir':str(self.output),'entry':'continuous_all_frames','errors':self.errors}
        atomic_json(self.state/'lifecycle.json',value)
        atomic_json(self.output/'lifecycle.json',value)

    def progress(self):
        capture=self.output/'rgbd_spool/status.json'
        if capture.exists():
            observed=json.loads(capture.read_text());source=int(observed.get('last_source_ns',0))
            if source!=self.capture_source:self.capture_source=source;self.capture_updated=time.monotonic()
            self.capture_state=observed.get('state','unknown')
        atomic_json(self.output/'entry_status.json',{'schema':'wbrov.continuous-entry-status.v1','phase':self.phase,
            'elapsed_s':time.monotonic()-self.start,'stop_requested':self.stop_requested,'saved':self.saved,
            'partial_saved':self.partial_saved,
            'capture_state':self.capture_state,'sealed_source_ns':self.capture_source,
            'seconds_without_new_seal':time.monotonic()-self.capture_updated,
            'errors':self.errors,'pids':{n:p.pid for n,p in self.children.items()},
            'exits':{n:p.poll() for n,p in self.children.items()},'historical_only':True,'navigation_authorized':False})

    def spawn(self,name,command,env):
        log=(self.output/(name+'.log')).open('x');self.logs.append(log)
        p=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,env=env)
        self.children[name]=p;atomic_pid(self.state/(name+'.pid'),p.pid);self.progress();return p

    def close_child(self,name,seconds=20):
        p=self.children.get(name)
        if p is None:return
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=seconds)
            except subprocess.TimeoutExpired:
                self.errors.append(name+' did not stop cleanly')
                p.terminate()
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    p.kill();p.wait()
        if p.returncode not in (0,-signal.SIGINT):self.errors.append(name+' exit='+str(p.returncode))

    def verify_save(self,require_complete=True):
        root=self.output/'processing';s=json.loads((root/'save.json').read_text())
        state=json.loads((root/'status.json').read_text())
        capture=json.loads((self.output/'rgbd_spool/status.json').read_text())
        if require_complete and (not s.get('complete') or state['state']!='closed' or not capture.get('clean_shutdown')):
            raise ValueError('capture/processing was not cleanly closed')
        if not require_complete and (not s.get('artifacts_saved') or state['state']!='interrupted_saved'):
            raise ValueError('no explicitly saved interrupted prefix')
        if capture['state']!='closed':raise ValueError('capture still owns unclosed data')
        if state['completed_segments']!=capture['sealed_segments']:raise ValueError('unprocessed recorded segments')
        if not state['mapping'].get('database_closed'):raise ValueError('database close was not confirmed')
        for name,pin in s['sha256'].items():
            if Path(name).name!=name or sha(root/'mapping'/name)!=pin:raise ValueError('saved artifact identity mismatch')
        # Only the terminated native worker's closed database is opened here.
        with sqlite3.connect((root/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:
            if db.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise ValueError('saved database check failed')
        if state['mapping'].get('cloud_points',0)<=0:raise ValueError('saved map has no verified cloud points')
        return s

    def run(self):
        c=self.config;out=self.output
        env={k:v for k,v in os.environ.items() if k in ('HOME','USER','LOGNAME','LANG','LC_ALL','DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR','DBUS_SESSION_BUS_ADDRESS')}
        env.update(c['environment']);env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        gui=dict(env);capture_env=dict(env)
        capture_env.pop('DISPLAY',None);capture_env.pop('WAYLAND_DISPLAY',None)
        capture_env.update(WBROV_RAW_RECORDING_DIR=str(out/'raw_recording'),WBROV_RGBD_SPOOL_DIR=str(out/'rgbd_spool'),ZED_OPENCV_CALIBRATION_FILE=c['calibration'])
        self.lifecycle('starting');self.progress()
        try:
            if shutil.disk_usage(out).free<30*1024**3:raise RuntimeError('less than 30 GiB free before capture')
            cap=self.spawn('bridge',[c['camera']],capture_env)
            deadline=time.monotonic()+90
            while True:
                if self.stop_requested:raise RuntimeError('stopped before camera produced input')
                if cap.poll() is not None:raise RuntimeError('camera exited during startup')
                status=out/'rgbd_spool/status.json'
                if status.exists() and json.loads(status.read_text()).get('sealed_segments',0)>0:break
                if time.monotonic()>deadline:raise TimeoutError('camera input did not seal within 90 seconds')
                time.sleep(.2)
            proc=self.spawn('processing',processing_command(c,out,self.state),env)
            deadline=time.monotonic()+15
            while not (out/'processing').is_dir():
                if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('processing did not start')
                time.sleep(.1)
            self.spawn('graph_display',['/usr/bin/python3','-B',c['display'],str(out/'processing')],gui)
            viewer=self.spawn('rviz',[c['rviz'],'-d',c['rviz_config']],gui)
            self.phase='capturing';self.progress()
            while not self.stop_requested and not (out/'stop_capture').exists():
                if viewer.poll() is not None:
                    if viewer.returncode:raise RuntimeError('RViz crashed: '+str(viewer.returncode))
                    self.request_stop();break
                for name in ['bridge','processing','graph_display']:
                    if self.children[name].poll() is not None:raise RuntimeError(name+' exited unexpectedly')
                if shutil.disk_usage(out).free<8*1024**3:
                    self.errors.append('disk low; capture stopped before reserve exhausted');break
                self.progress();time.sleep(.5)
        except Exception as e:self.errors.append(str(e))
        finally:
            self.phase='draining';self.lifecycle('stopping');self.progress()
            # The outer wrapper sees stopping immediately and closes teleop.
            # Camera closure seals the final segment. NEVER signal the queue to
            # implement normal stop, even if it has many minutes of backlog.
            self.close_child('bridge',60)
            proc=self.children.get('processing')
            if proc is not None:
                capture=out/'rgbd_spool/status.json'
                closed=capture.exists() and json.loads(capture.read_text()).get('state')=='closed'
                if not closed and proc.poll() is None:
                    # A failed writer cannot produce a trustworthy EOF. Retain
                    # every input, fail explicitly, and stop this failed queue.
                    self.errors.append('capture did not seal; partial input retained')
                    proc.terminate()
                while proc.poll() is None:
                    self.progress();time.sleep(.5)
                if proc.returncode==2:
                    try:
                        receipt=self.verify_save(require_complete=False)
                        atomic_json(out/'partial_saved_map.json',receipt);self.partial_saved=True
                        self.errors.append('capture interrupted; sealed observations saved, recording incomplete')
                    except Exception as e:self.errors.append(str(e))
                elif proc.returncode:self.errors.append('processing exit='+str(proc.returncode))
                else:
                    try:
                        receipt=self.verify_save()
                        verify=self.spawn('recording_verify',['/usr/bin/python3','-B',c['raw_verify'],'--directory',str(out/'raw_recording'),'--verifier',c['verifier']],env)
                        # Recorded duration can exceed any fixed short timeout.
                        while verify.poll() is None:self.progress();time.sleep(.5)
                        if verify.returncode:raise RuntimeError('raw SVO verification failed')
                        self.saved=True;atomic_json(out/'saved_map.json',receipt)
                    except Exception as e:self.errors.append(str(e))
            # Keep the verified SAVED overlay visible briefly when RViz remains.
            if self.saved:time.sleep(2)
            self.close_child('rviz');self.close_child('graph_display')
            self.phase='saved' if self.saved and not self.errors else 'failed'
            self.progress();self.lifecycle('stopped',complete=self.saved and not self.errors)
            for log in self.logs:log.close()
        return int(bool(self.errors) or not self.saved)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--config-sha256',required=True)
    p.add_argument('--check-only',action='store_true');p.add_argument('--state',type=Path)
    p.add_argument('--output-root',type=Path);p.add_argument('--launcher-pid',type=int)
    a=p.parse_args();c=load_config(a.config,a.config_sha256)
    if a.check_only:print(json.dumps({'checked':True,'all_frames':True,'profile':'continuous_new_map','navigation_authorized':False}));return 0
    if not a.state or not a.output_root or not a.launcher_pid:raise ValueError('missing launcher identity/directories')
    os.umask(0o077);state=secure_directory(a.state);root=secure_directory(a.output_root)
    fd=os.open(state/'launcher.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Same lock as the legacy entry. Do not overwrite its state while in use.
    out=root/('continuous_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(a.launcher_pid));out.mkdir(mode=0o700)
    with (state/'run_dir.tmp').open('w') as f:f.write(str(out)+'\n');f.flush();os.fsync(f.fileno())
    (state/'run_dir.tmp').replace(state/'run_dir')
    for name in ['bridge','processing','graph_display','rviz','slam']:
        old=state/(name+'.pid')
        if old.is_symlink():raise ValueError('redirected PID file')
        old.unlink(missing_ok=True)
    session=Session(c,state,out,a.launcher_pid)
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP,signal.SIGUSR1]:signal.signal(sig,session.request_stop)
    try:return session.run()
    finally:os.close(fd)


if __name__=='__main__':raise SystemExit(main())
