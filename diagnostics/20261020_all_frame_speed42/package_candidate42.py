from pathlib import Path
import os,json,hashlib,shutil,subprocess,sys,time,traceback,fcntl,signal
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
old=r/'mapping_round6_candidate22_parallel';p=r/'mapping_round6_candidate42_voxel';out=r/'candidate42_package_validation'
out.mkdir(mode=0o700)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
v={'phase':'prepare','scope':'independent candidate package only; same Core20/Tag+visual recovery, narrow export index worker41; no hardware or production switch','steps':[]}
def save():
    f=out/'result.tmp';f.write_text(json.dumps(v,indent=2));f.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        assert not any(x in cmd for x in ['/bin/wbrov_replay_worker','/bin/zed_rgbd_publisher_rate_ab','/tools/continuous_mapping_session.py','/voxel41_build/wbrov_replay_worker']),cmd
    assert shutil.disk_usage(r).free>10*2**30
    fixed=json.loads((r/'voxel41_validation/result.json').read_text());plain=json.loads((r/'voxel42_plain/result.json').read_text())
    assert fixed['phase']=='completed' and fixed['output_equivalence_passed'] and fixed['recovery_continuation_passed']
    assert plain['phase']=='completed' and plain['all_inputs_and_tags_equal'] and plain['recovery_continuation_passed']
    assert all(s['save_verified'] and s['database_check']=='ok' for s in plain['steps'])
    assert fixed['comparisons']['field']['time_saved_fraction']>.02 and fixed['comparisons']['short']['time_saved_fraction']>=-.02
    assert plain['ordinary_timing_qualifies'] and plain['comparisons']['short']['time_saved_fraction']>=-.02
    dispatcher=Path('/home/exocean/Desktop/机器人键盘遥控/.tools/run_water_mapping_with_light.sh')
    production_pin='d1581976c17bc6a68b32aa45ff4eac73c7ec3930b392b79e04c82b6061a6436a';assert sha(dispatcher)==production_pin
    source_core=r/'parallel20_build/librtabmap_core.so.0.23.7'
    expected_core='7bed7a7da1178a2f33121408091cc789fbcc085be51108d8bf22c2ad065a310a';assert sha(source_core)==expected_core
    source_worker=r/'voxel41_build/wbrov_replay_worker'
    expected_worker='489980de6f910aacd4319d400ffa6b1a7796955ed860cb2d57d496334efe165f';assert sha(source_worker)==expected_worker
    entry_old_pin=sha(old/'entry.json');c0=json.loads((old/'entry.json').read_text());assert all(sha(n)==pin for n,pin in c0['sha256'].items())
    assert c0['tag_modules'] and not p.exists()
    total=sum(f.stat().st_size for f in old.rglob('*') if f.is_file() and not f.is_symlink());assert total<50*2**20,total
    shutil.copytree(old,p,symlinks=True)
    for f in p.rglob('*'):
        if f.is_symlink():assert f.resolve().is_relative_to(p.resolve()),str(f)
    core=p/'lib/librtabmap_core.so.0.23.7';assert not core.is_symlink() and sha(core)==expected_core
    worker=p/'bin/wbrov_replay_worker';assert not worker.is_symlink();worker.chmod(0o700);shutil.copyfile(source_worker,worker);worker.chmod(0o500)
    c=json.loads(json.dumps(c0).replace(str(old),str(p)));c['environment']['OMP_WAIT_POLICY']='PASSIVE'
    assert not any(k in c['environment'] for k in ['LD_PRELOAD','WBROV_OFFLINE_RANDOM_DIAGNOSTIC','WBROV_OFFLINE_PHASE_DIAGNOSTIC','WBROV_PRIVATE_OPTIMIZER_TRACE'])
    c['sha256']={n:sha(n) for n in c['sha256']}
    # Every original runtime file keeps identical content except this single worker.
    for n,pin in c0['sha256'].items():
        new=n.replace(str(old),str(p))
        assert c['sha256'][new]==(expected_worker if Path(new).resolve()==worker.resolve() else pin),(n,new)
    (p/'entry.json').write_text(json.dumps(c,indent=2))
    sys.path.insert(0,str(p/'tools'));from render_continuous_entry import render
    launcher,wrapper=render(p,sha(p/'entry.json'),sha(p/'tools/continuous_mapping_session.py'),(p/'launch/candidate_receipt_resume/run_water_mapping_with_light.sh').read_text(),sha(c['worker']))
    for name,data in [('start_rtabmap_continuous.sh',launcher),('run_continuous_mapping_with_light.sh',wrapper)]:
        f=p/name;f.chmod(0o700);f.write_text(data);f.chmod(0o500);subprocess.run(['/usr/bin/bash','-n',str(f)],check=True)
    env=dict(os.environ);env.update(c['environment']);env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    assert 'LD_PRELOAD' not in env and 'WBROV_OFFLINE_RANDOM_DIAGNOSTIC' not in env
    def run(name,args,timeout=180,extra=None,cwd=None):
        v['phase']=name;save();start=time.monotonic();e=env.copy()
        if extra:e.update(extra)
        with (out/(name+'.log')).open('x') as log:
            proc=subprocess.Popen(args,env=e,cwd=cwd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:rc=proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                raise
        v['steps'].append({'name':name,'seconds':time.monotonic()-start,'exit_code':rc,'command':args});save();assert rc==0,name
    run('worker_ldd',['ldd','-r',c['worker']])
    ld=(out/'worker_ldd.log').read_text();assert str(p/'lib/librtabmap_core.so') in ld and not any(x in ld for x in ['not found','undefined symbol'])
    for name in ['test_deferred_mapping','test_continuous_mapping_session','test_deferred_display_state','test_deferred_recorded_pixels']:
        run(name,['/usr/bin/python3','-B','-m','unittest','discover','-s','tests','-p',name+'.py'],60,cwd=p)
    run('entry_check_only',[str(p/'start_rtabmap_continuous.sh'),'--check-only'],60)
    test=r/'round6_recovery9_build/test_visual'
    run('test_ldd',['ldd',str(test)]);assert str(p/'lib/librtabmap_core.so') in (out/'test_ldd.log').read_text()
    trace=out/'trace';trace.mkdir()
    for label,args in [('visual',[]),('tag',['--tag-depth-proposal'])]:
        run('core_'+label,[str(test),str(out/('fixture_'+label)),*args],180,{'WBROV_PRIVATE_OPTIMIZER_TRACE':str(trace)})
    assert sha(dispatcher)==production_pin and sha(old/'entry.json')==entry_old_pin
    assert all(sha(n)==pin for n,pin in c0['sha256'].items()) and all(sha(n)==pin for n,pin in c['sha256'].items())
    v.update(phase='completed',package=str(p),core_sha256=sha(core),entry_sha256=sha(p/'entry.json'),worker_sha256=sha(c['worker']),camera_sha256=sha(c['camera']),wrapper_sha256=sha(p/'run_continuous_mapping_with_light.sh'),tag_detection_enabled=True,source_file_content_preserved=True,production_changed=False,field_acceptance=False,free_gib=shutil.disk_usage(r).free/2**30)
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
