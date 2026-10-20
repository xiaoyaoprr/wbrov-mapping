from pathlib import Path
import os,pty,select,json,time,signal,hashlib,subprocess,fcntl
H=Path('/home/exocean');R=H/'.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018'
OUT=R/'round6_field_notags_v6';OUT.mkdir(mode=0o700)
receipt=json.loads((R/'package_round6_candidate6_result.json').read_text())
assert receipt['tag_detection_enabled'] is False
assert json.loads((R/'round6_baseline5/result.json').read_text())['phase']=='completed'
ENTRY=Path(receipt['package'])/'run_continuous_mapping_with_light.sh'
assert hashlib.sha256(ENTRY.read_bytes()).hexdigest()==receipt['wrapper_sha256']
assert not json.loads((Path(receipt['package'])/'entry.json').read_text())['tag_modules']
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
 except OSError:continue
 assert not any(v in cmd for v in ['bin/zed_rgbd_publisher_rate_ab','bin/wbrov_replay_worker','tools/continuous_mapping_session.py']),cmd
assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
fd=os.open(H/'.cache/wbrov_water_mapping/orchestrator.lock',os.O_RDWR|os.O_NOFOLLOW);fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);os.close(fd)
env={'HOME':str(H),'USER':'exocean','LOGNAME':'exocean','SHELL':'/usr/bin/bash','PATH':'/usr/bin:/bin','LANG':'C.UTF-8','TERM':'xterm-256color','DISPLAY':':0','XAUTHORITY':'/run/user/1000/gdm/Xauthority','XDG_RUNTIME_DIR':'/run/user/1000','DBUS_SESSION_BUS_ADDRESS':'unix:path=/run/user/1000/bus'}
started=time.monotonic();pid,master=pty.fork()
if pid==0:os.execve('/usr/bin/bash',['/usr/bin/bash','--noprofile','--norc',str(ENTRY)],env)
ticks=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
report={'entry':str(ENTRY),'entry_sha256':hashlib.sha256(ENTRY.read_bytes()).hexdigest(),'wrapper_pid':pid,'wrapper_ticks':ticks,'start_monotonic':started,'phase':'starting','motion_keys_sent':False,'inputs':[],'scope':'candidate no-Tag ordinary movement check after live RGB/cloud verification; no commanded movement','tag_detection_enabled':False}
last_startup_sample=0.0
buffer='';selected=False;confirmed=False;ready=None;stopping=False;exitstatus=None;footer_closed=False

def observe_startup():
 global last_startup_sample
 now=time.monotonic()
 if ready or now-last_startup_sample<.5:return
 last_startup_sample=now
 row={'monotonic_s':now,'elapsed_s':now-started}
 try:
  state_path=H/'.cache/wbrov_control/mapping_light/ready.json'
  raw=state_path.read_bytes();v=json.loads(raw);row['state']=v
  row['state_sha256']=hashlib.sha256(raw).hexdigest()
  row['state_age_s']=now-v.get('updated_at_monotonic_s',0)
  tp=int(v['pid']);proc=Path(f'/proc/{tp}')
  if proc.exists():
   row['proc_ticks']=proc.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
   row['cmdline']=proc.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
   row['cwd']=str(proc.joinpath('cwd').resolve())
   e=dict(x.split(b'=',1) for x in proc.joinpath('environ').read_bytes().split(b'\0') if b'=' in x)
   row['environment']={k.decode():e.get(k,b'').decode() for k in [b'DRY_RUN',b'WBROV_LIGHT_ON_START',b'WBROV_LIGHT_BRIGHTNESS',b'WBROV_LIGHT_READY_FILE']}
   row['sockets']=subprocess.check_output(['ss','-H','-u','-a','-n','-p','sport = :3333'],text=True,timeout=2)
   args=['/usr/bin/python3',str(H/'Desktop/机器人键盘遥控/.tools/validate_mapping_light_state.py'),str(state_path),'--pid',str(tp),'--start-ticks',row['proc_ticks'],'--boot-id',Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'--brightness','30','--phase','startup','--max-age','2','--not-before',str(started)]
   check=subprocess.run(args,capture_output=True,text=True,timeout=2)
   row['light_validator']={'exit':check.returncode,'stderr':check.stderr}
 except Exception as ex:row['read_error']=str(ex)
 with (OUT/'startup_observations.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')

def save():
 report['elapsed_s']=time.monotonic()-started
 t=OUT/'result.tmp';t.write_text(json.dumps(report,ensure_ascii=False,indent=2));t.replace(OUT/'result.json')
def stop():
 global stopping
 if stopping:return
 current=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
 assert current==ticks
 if report.get('mapping_run') and ready:
  run=Path(report['mapping_run']);state=json.loads((run/'entry_status.json').read_text());assert state['phase']=='capturing'
  (run/'stop_capture').touch(exist_ok=False)
 else:os.kill(pid,signal.SIGTERM)
 stopping=True;report['normal_stop_requested']=True;report['phase']='draining';save()
save()
with (OUT/'terminal.log').open('xb',buffering=0) as log:
 while True:
  if select.select([master],[],[],.25)[0]:
   try:data=os.read(master,65536)
   except OSError:data=b''
   if data:
    log.write(data);buffer=(buffer+data.decode('utf-8',errors='replace'))[-30000:]
  if not selected and '输入 1、2 或 3：' in buffer:
   os.write(master,b'1\n');selected=True;report['inputs'].append('new map menu: 1');save()
  if not confirmed and '确认后输入 OK：' in buffer:
   os.write(master,b'OK\n');confirmed=True;report['inputs'].append('authorized existing entry confirmation: OK');save()
  if ready is None and '建图各必需子进程身份' in buffer:
   ready=time.monotonic();report.update(phase='running',ready_elapsed_s=ready-started);save()
  life=H/'.cache/rtabmap_dense_calib_ab/lifecycle.json'
  if life.exists():
   v=json.loads(life.read_text())
   if int(v.get('launcher_pid',0))>pid and v.get('entry')=='continuous_all_frames':report['mapping_run']=v['run_dir']
  if not footer_closed and '按回车关闭此窗口' in buffer:
   os.write(master,b'\n');footer_closed=True
  child,status=os.waitpid(pid,os.WNOHANG)
  if child:exitstatus=os.waitstatus_to_exitcode(status);break
  observe_startup()
  now=time.monotonic()
  if not stopping and ((OUT/'stop').exists() or (not ready and now-started>240)):stop()
  save()
report.update(phase='closed',wrapper_exit=exitstatus);save();os.close(master)

