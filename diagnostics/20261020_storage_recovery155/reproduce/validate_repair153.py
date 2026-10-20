from pathlib import Path
import os,json,subprocess,hashlib,time,traceback,fcntl,csv,sqlite3,shutil
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'repair153_replay';out.mkdir(mode=0o700)
io_process=None;io_log=None
v={'phase':'prepare','scope':'one complete saved field119 replay; Core152 bounded distinct-reference fallback with unchanged raw/gravity and transaction checks, worker130 actual retained-keyframe identity scheduling fix; same parameters, Tags, density, complete original input and quality gates; no live hardware','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
    p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    build=r/'repair130_build';built=json.loads((build/'result.json').read_text());assert built['phase']=='completed'
    core=r/'repair152_build';core_result=json.loads((core/'result.json').read_text());assert core_result['phase']=='completed'
    assert sha(core/'librtabmap_core.so.0.23.7')==core_result['core_sha256']
    c=json.loads((r/'mapping_round6_candidate42_voxel/entry.json').read_text());assert all(sha(p)==pin for p,pin in c['sha256'].items())
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    assert shutil.disk_usage(r).free>30*2**30
    run=Path(json.loads((r/'field119_candidate42_v2/result.json').read_text())['mapping_run'])
    recording=run/'rgbd_spool';capture=json.loads((recording/'status.json').read_text())
    assert capture['state']=='closed' and capture['clean_shutdown'] and capture['frames']==9771
    input_pin=sha(recording/'status.json');worker=build/'wbrov_replay_worker';assert sha(worker)==built['worker_sha256']
    env=dict(os.environ);env.update(c['environment']);env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE')
    assert 'LD_PRELOAD' not in env and not any(k.startswith('WBROV_OFFLINE_') for k in env)
    env['LD_LIBRARY_PATH']=str(core)+':'+env['LD_LIBRARY_PATH']
    binding=subprocess.check_output(['ldd','-r',str(worker)],env=env,text=True)
    (out/'worker_bindings.log').write_text(binding)
    assert str(core/'librtabmap_core.so') in binding and 'not found' not in binding and 'undefined symbol' not in binding
    v['core_sha256']=core_result['core_sha256']
    target=out/'field'
    args=['/usr/bin/python3','-B',c['supervisor'],'--recording',str(recording),'--output',str(target),
        '--worker',str(worker),'--verifier',c['verifier'],'--odom-parameters',c['odom_parameters'],
        '--map-parameters',c['map_parameters'],'--tag-modules',c['tag_modules'],'--cv-threads','1','--lookahead-segments','1']
    v.update(phase='field',command=args,source_status_sha256=input_pin,worker_sha256=sha(worker));save()
    started=time.monotonic()
    with (out/'field.log').open('x') as log:
        p=subprocess.run(args,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,timeout=2400)
    v.update(seconds=time.monotonic()-started,exit=p.returncode);save();assert p.returncode==0
    storage=json.loads((r/'repair125_replay/result.json').read_text())['concurrent_storage']
    assert storage['passed']
    v['prior_storage_evidence']=storage
    saved=json.loads((target/'save.json').read_text());state=json.loads((target/'mapping/status.json').read_text())
    final=json.loads((target/'status.json').read_text())
    assert saved['complete'] and state['database_closed'] and final['state']=='closed' and final['tag_detection_enabled']
    assert final['completed_segments']==capture['sealed_segments'] and int(final['acknowledged_source_ns'])==int(capture['last_source_ns'])
    for name,pin in saved['sha256'].items():assert Path(name).name==name and sha(target/'mapping'/name)==pin
    with sqlite3.connect((target/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:
        check=db.execute('PRAGMA quick_check').fetchone()[0]
    assert check=='ok'
    rows=list(csv.DictReader((target/'mapping/frames.csv').open()))
    source=list(csv.DictReader((run/'processing/mapping/frames.csv').open()))
    stamps=[row['source_ns'] for row in rows];assert stamps==[row['source_ns'] for row in source] and len(rows)==9771
    assert not any(row['main_write']=='1' and (row['lost']=='1' or row['input_present']=='0') for row in rows)
    events=[];last=0
    for row in rows:
        if int(row['recoveries'])>last:events.append({'source_ns':int(row['source_ns']),'main_nodes':int(row['main_nodes'])})
        last=int(row['recoveries'])
    log=(target/'mapping/core.log').read_text()
    v.update(frames=len(rows),all_input_stamps_equal=True,save_verified=True,database_check=check,
        final_continuity=state['continuity_state'],main_nodes=state['main_nodes'],cloud_points=state['cloud_points'],
        recoveries=state['recoveries'],cuts=state['cuts'],recovery_events=events,
        isolated_motion_faults=log.count('recorded recovery local motion fault'),
        saved_sha256=saved['sha256'])
    save()
    assert sha(recording/'status.json')==input_pin and sha(worker)==built['worker_sha256']
    assert all(sha(p)==pin for p,pin in c['sha256'].items())
    v['recovery_continuation_passed']=bool(events) and state['main_nodes']>events[0]['main_nodes'] and state['continuity_state']=='NORMAL'
    v['phase']='completed' if v['recovery_continuation_passed'] else 'validation_failed'
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
finally:
    if io_process is not None and io_process.poll() is None:
        io_process.terminate()
        try:io_process.wait(timeout=10)
        except subprocess.TimeoutExpired:io_process.kill();io_process.wait()
    if io_log is not None:io_log.close()
save()
