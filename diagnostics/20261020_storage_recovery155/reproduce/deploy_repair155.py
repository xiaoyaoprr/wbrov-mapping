from pathlib import Path
import os,json,hashlib,stat,subprocess,fcntl,time,shutil,traceback
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
pkg=r/'mapping_round6_candidate155_recovery';out=r/'repair155_deployment'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for folder in ['repair155_acceptance','repair154_regression','repair155_package']:
    assert json.loads((r/folder/'result.json').read_text())['phase']=='completed',folder
suite=json.loads((r/'repair155_acceptance/result.json').read_text())
assert all(sha(p)==pin for p,pin in suite['receipt_sha256'].items())
display=json.loads((r/'repair155_display_rgb2/result.json').read_text())
assert display.get('render_ready') and display.get('recorded_rgb_published') and display.get('screenshot_exit')==0 and not display.get('error')
assert not out.exists();out.mkdir(mode=0o700)
tools=Path('/home/exocean/Desktop/机器人键盘遥控/.tools')
target=tools/'run_water_mapping_with_light.sh'
legacy=tools/'run_water_mapping_with_light.pre_combo42.sh'
oldpin='d1581976c17bc6a68b32aa45ff4eac73c7ec3930b392b79e04c82b6061a6436a'
assert sha(target)==oldpin and sha(legacy)=='34cf9b3c35fa4f7977721bb94f4786d59bf662c6c552d202c14123ab06bb03d2'
pack=json.loads((r/'repair155_package/result.json').read_text())
assert sha(pkg/'entry.json')==pack['entry_sha256']
config=json.loads((pkg/'entry.json').read_text())
assert all(sha(p)==pin for p,pin in config['sha256'].items())
protected=[Path('/home/exocean/Desktop/机器人键盘遥控/8-水下遥控开灯加建图_一键启动'),
    tools/'run_water.sh',tools/'run_water_calibration.sh',legacy,
    Path('/home/exocean/code/poa-aco-integration/integration/teleop_keyboard.py'),
    Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/start_rtabmap_calib_ab.sh'),
    Path('/home/exocean/.cache/wbrov_control/rtabmap_xy_hold/run_rtabmap_xy_hold.sh'),
    Path('/home/exocean/.cache/kcap_navigation_candidates/20260920/open_navigation_v5.sh'),
    Path(config['calibration'])]
before={str(p):sha(p) for p in protected}
old=target.read_bytes();mode=stat.S_IMODE(target.stat().st_mode)
text=old.decode()
oldroute=str(r/'mapping_combo42_camera6/run_continuous_mapping_with_light.sh')
oldroutepin='727683484e856fc393e494810fe8d2a929a8bec740046162f82ff658e11ee1ec'
assert text.count(oldroute)==1 and text.count(oldroutepin)==1
wrapper=pkg/'run_continuous_mapping_with_light.sh'
assert sha(wrapper)==pack['wrapper_sha256']
data=text.replace(oldroute,str(wrapper)).replace(oldroutepin,sha(wrapper)).encode()
assert b'continue|localize-old)' in data
newpin=hashlib.sha256(data).hexdigest()
(out/'before_dispatcher.sh').write_bytes(old);(out/'candidate_dispatcher.sh').write_bytes(data)
v={'status':'preflight','scope':'saved-data recovery and RViz verified; file-only installation; no live camera/control acceptance',
   'before_outer':oldpin,'after_outer':newpin,'package':str(pkg),'protected':before,'checks':[],
   'production_switched':False}
fds=[];changed=False
def save():
    p=out/'receipt.tmp';p.write_text(json.dumps(v,ensure_ascii=False,indent=2));p.replace(out/'deployment_receipt.json')
def idle():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
        try:
            args=(proc/'cmdline').read_bytes().split(b'\0')
            names=[Path(a.decode(errors='replace')).name for a in args]
            assert not any(any(s in n for s in ['teleop','run_water','zed_rgb','rgbd_odometry','wbrov_replay','deferred_mapping.py','continuous_mapping_session.py','rviz2']) for n in names),'active mapping/control'
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
def locks():
    for p in [r/'speed_optimization.lock',Path('/home/exocean/.cache/wbrov_water_mapping/orchestrator.lock'),
              Path('/home/exocean/.cache/rtabmap_dense_calib_ab/launcher.lock')]:
        fd=os.open(p,os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600);fds.append(fd)
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
def release():
    while fds:os.close(fds.pop())
def atomic(payload):
    tmp=target.with_name('.'+target.name+'.repair155-'+str(time.monotonic_ns()))
    fd=os.open(tmp,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,mode)
    with os.fdopen(fd,'wb') as f:f.write(payload);f.flush();os.fsync(f.fileno())
    os.chmod(tmp,mode);os.replace(tmp,target)
    fd=os.open(target.parent,os.O_RDONLY|os.O_DIRECTORY);os.fsync(fd);os.close(fd)
try:
    save();idle();assert shutil.disk_usage(r).free>=30*2**30
    subprocess.run(['/usr/bin/bash','-n',str(out/'candidate_dispatcher.sh')],check=True)
    env=dict(os.environ,DISPLAY=':0',XAUTHORITY='/run/user/1000/gdm/Xauthority',XDG_RUNTIME_DIR='/run/user/1000',
             DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/1000/bus')
    for stage,path in [('candidate',out/'candidate_dispatcher.sh'),('installed',target)]:
        if stage=='installed':
            locks();idle();assert sha(target)==oldpin
            assert target.resolve()==target and target.stat().st_uid==os.getuid() and target.stat().st_nlink==1
            atomic(data);changed=True;v['production_switched']=True;save();release()
        for m in ['new','continue','localize-old']:
            q=subprocess.run(['/usr/bin/bash',str(path),'--check-only','--mode',m],env=env,capture_output=True,text=True,timeout=90)
            (out/(stage+'_'+m+'.log')).write_text(q.stdout+q.stderr)
            v['checks'].append({'stage':stage,'mode':m,'exit':q.returncode});save()
            assert q.returncode==0,(stage,m)
    assert sha(target)==newpin and {str(p):sha(p) for p in protected}==before
    assert all(sha(p)==pin for p,pin in config['sha256'].items())
    idle();v.update(status='deployed',protected_unchanged=True,free_bytes=shutil.disk_usage(r).free)
except BaseException as e:
    v.update(status='failed',error=repr(e),traceback=traceback.format_exc())
    if changed:
        release();locks();idle();assert sha(target)==newpin
        atomic(old);v.update(rollback='passed',production_switched=False)
    raise
finally:
    release();save();print(json.dumps(v,ensure_ascii=False))
