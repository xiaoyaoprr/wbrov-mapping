from pathlib import Path
import os,json,hashlib,shutil,subprocess,sys,time,traceback,fcntl
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
old=r/'mapping_round6_candidate42_voxel';p=r/'mapping_round6_candidate155_recovery'
out=r/'repair155_package';out.mkdir(mode=0o700)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
v={'phase':'preflight','scope':'isolated package of verified saved-data repair; no hardware or production switch','steps':[]}
def save():
    f=out/'result.tmp';f.write_text(json.dumps(v,indent=2));f.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert json.loads((r/'repair152_reference_fixture/result.json').read_text())['phase']=='completed'
    replay=json.loads((r/'repair153_replay/result.json').read_text())
    suite=json.loads((r/'repair155_acceptance/result.json').read_text())
    assert suite['phase']=='completed' and all(sha(p)==pin for p,pin in suite['receipt_sha256'].items())
    assert replay['phase']=='validation_failed' and replay['prior_storage_evidence']['passed']
    assert suite['core_sha256']==replay['core_sha256'] and suite['worker_sha256']==replay['worker_sha256']
    regression=json.loads((r/'repair154_regression/result.json').read_text())
    assert regression['phase']=='completed' and regression['recovery_continuation_passed']
    assert json.loads((r/'repair153_replay/field/mapping/status.json').read_text())['visual_candidate_deferred_retained_searches']>0
    assert replay['frames']==9771 and replay['all_input_stamps_equal'] and replay['save_verified'] and replay['database_check']=='ok'
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    assert shutil.disk_usage(r).free>30*2**30
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        assert not any(x in cmd for x in ['/bin/zed_rgbd_publisher_rate_ab','/bin/wbrov_replay_worker','/repair130_build/wbrov_replay_worker','/tools/continuous_mapping_session.py']),cmd
    dispatcher=Path('/home/exocean/Desktop/机器人键盘遥控/.tools/run_water_mapping_with_light.sh')
    production_pin='d1581976c17bc6a68b32aa45ff4eac73c7ec3930b392b79e04c82b6061a6436a'
    assert sha(dispatcher)==production_pin
    source_core=r/'repair152_build/librtabmap_core.so.0.23.7'
    source_worker=r/'repair130_build/wbrov_replay_worker'
    assert sha(source_core)==replay['core_sha256'] and sha(source_worker)==replay['worker_sha256']
    c0=json.loads((old/'entry.json').read_text());old_pin=sha(old/'entry.json')
    assert all(sha(n)==h for n,h in c0['sha256'].items()) and not p.exists()
    assert sum(f.stat().st_size for f in old.rglob('*') if f.is_file() and not f.is_symlink())<50*2**20
    shutil.copytree(old,p,symlinks=True)
    for f in p.rglob('*'):
        if f.is_symlink():assert f.resolve().is_relative_to(p.resolve())
    core=p/'lib/librtabmap_core.so.0.23.7';worker=p/'bin/wbrov_replay_worker'
    for dest,source in [(core,source_core),(worker,source_worker)]:
        assert not dest.is_symlink();dest.chmod(0o700);shutil.copyfile(source,dest);dest.chmod(0o500)
    c=json.loads(json.dumps(c0).replace(str(old),str(p)))
    assert not any(k in c['environment'] for k in ['LD_PRELOAD','WBROV_OFFLINE_RANDOM_DIAGNOSTIC','WBROV_PRIVATE_OPTIMIZER_TRACE'])
    c['sha256']={n:sha(n) for n in c['sha256']}
    for n,h in c0['sha256'].items():
        new=n.replace(str(old),str(p))
        expected=replay['core_sha256'] if Path(new).resolve()==core.resolve() else replay['worker_sha256'] if Path(new).resolve()==worker.resolve() else h
        assert c['sha256'][new]==expected
    (p/'entry.json').write_text(json.dumps(c,indent=2))
    sys.dont_write_bytecode=True
    sys.path.insert(0,str(p/'tools'));from render_continuous_entry import render
    launcher,wrapper=render(p,sha(p/'entry.json'),sha(p/'tools/continuous_mapping_session.py'),
        (p/'launch/candidate_receipt_resume/run_water_mapping_with_light.sh').read_text(),sha(worker))
    for name,data in [('start_rtabmap_continuous.sh',launcher),('run_continuous_mapping_with_light.sh',wrapper)]:
        f=p/name;f.chmod(0o700);f.write_text(data);f.chmod(0o500)
        subprocess.run(['/usr/bin/bash','-n',str(f)],check=True)
    env=dict(os.environ);env.update(c['environment']);env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    def run(name,cmd,timeout=90,cwd=None):
        v['phase']=name;save();t=time.monotonic()
        with (out/(name+'.log')).open('x') as f:
            q=subprocess.run(cmd,env=env,cwd=cwd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
        v['steps'].append({'name':name,'seconds':time.monotonic()-t,'exit':q.returncode});save();assert q.returncode==0,name
    run('worker_bindings',['ldd','-r',str(worker)])
    binding=(out/'worker_bindings.log').read_text()
    assert str(p/'lib/librtabmap_core.so') in binding and 'not found' not in binding and 'undefined symbol' not in binding
    for name in ['test_deferred_mapping','test_continuous_mapping_session','test_deferred_display_state','test_deferred_recorded_pixels']:
        run(name,['/usr/bin/python3','-B','-m','unittest','discover','-s','tests','-p',name+'.py'],60,p)
    run('entry_check_only',[str(p/'start_rtabmap_continuous.sh'),'--check-only'],90)
    assert sha(dispatcher)==production_pin and sha(old/'entry.json')==old_pin
    assert all(sha(n)==h for n,h in c0['sha256'].items()) and all(sha(n)==h for n,h in c['sha256'].items())
    v.update(phase='completed',package=str(p),core_sha256=sha(core),worker_sha256=sha(worker),
        entry_sha256=sha(p/'entry.json'),wrapper_sha256=sha(p/'run_continuous_mapping_with_light.sh'),
        camera_sha256=sha(c['camera']),tag_detection_enabled=True,production_changed=False,field_acceptance=False)
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
