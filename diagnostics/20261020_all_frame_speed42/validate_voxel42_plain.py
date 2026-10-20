from pathlib import Path
import os,subprocess,json,time,hashlib,csv,sqlite3,traceback,shutil,fcntl,signal
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'voxel42_plain';out.mkdir(mode=0o700)
v={'phase':'prepare','scope':'fixed four ordinary all-frame replays, no entropy preload or timing wrapper; candidate22 supervisor and Core20 unchanged; only original worker10 vs narrow voxel index worker41 differs; PASSIVE wait for both; no hardware/deployment','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
    t=out/'result.tmp';t.write_text(json.dumps(v,indent=2));t.replace(out/'result.json')
def resources():
    result={'loadavg':Path('/proc/loadavg').read_text().strip(),'free_gib':shutil.disk_usage(r).free/2**30}
    result['cpu_freq_khz']={str(p):p.read_text().strip() for p in Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_cur_freq')}
    result['temperature_mC']={p.parent.joinpath('type').read_text().strip():p.read_text().strip() for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')}
    return result
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    previous=json.loads((r/'voxel41_validation/result.json').read_text())
    assert previous['phase']=='completed' and previous['output_equivalence_passed'] and previous['recovery_continuation_passed']
    assert previous['comparisons']['field']['time_saved_fraction']>.02 and previous['comparisons']['short']['time_saved_fraction']>=-.02,'insufficient fixed-condition gain to justify ordinary replay'
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        assert not any(x in cmd for x in ['/bin/zed_rgbd_publisher_rate_ab','/bin/wbrov_replay_worker','/tools/continuous_mapping_session.py','/voxel41_build/wbrov_replay_worker']),cmd
    assert shutil.disk_usage(r).free>10*2**30
    p=r/'mapping_round6_candidate22_parallel';c=json.loads((p/'entry.json').read_text())
    assert all(sha(path)==pin for path,pin in c['sha256'].items())
    build=r/'voxel41_build';built=json.loads((build/'result.json').read_text())
    core=r/'parallel20_build';core_pin='7bed7a7da1178a2f33121408091cc789fbcc085be51108d8bf22c2ad065a310a'
    assert built['phase']=='completed' and built['unchanged_other_sources']
    assert sha(build/'wbrov_replay_worker')==built['worker_sha256']=='489980de6f910aacd4319d400ffa6b1a7796955ed860cb2d57d496334efe165f'
    assert sha(core/'librtabmap_core.so.0.23.7')==core_pin
    base_env=dict(os.environ);base_env.update(c['environment']);base_env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE')
    assert 'LD_PRELOAD' not in base_env and 'WBROV_OFFLINE_RANDOM_DIAGNOSTIC' not in base_env and 'WBROV_OFFLINE_PHASE_DIAGNOSTIC' not in base_env
    v['pins']={'baseline_worker':sha(c['worker']),'candidate_worker':built['worker_sha256'],'core':core_pin,'supervisor':sha(c['supervisor'])};save()
    inputs={'short':r/'motion_spool1','field':Path('/home/exocean/maps/rtabmap_calibration_ab/continuous_20261012_083050_1300795/rgbd_spool')}
    known=previous
    # Counterbalance the variant order from the diagnostic experiment.
    for variant,input_name in [('candidate','short'),('baseline','short'),('baseline','field'),('candidate','field')]:
        name=variant+'_'+input_name;target=out/name;recording=inputs[input_name]
        capture=json.loads((recording/'status.json').read_text());assert capture['state']=='closed' and capture['clean_shutdown']
        input_pin=sha(recording/'status.json');supervisor=c['supervisor']
        env=base_env.copy()
        env['LD_LIBRARY_PATH']=str(core)+':'+env['LD_LIBRARY_PATH']
        worker=str(build/'wbrov_replay_worker') if variant=='candidate' else c['worker']
        ld=subprocess.check_output(['ldd',worker],env=env,text=True)
        expected_core=core
        assert str(expected_core/'librtabmap_core.so') in ld
        (out/(name+'.ldd')).write_text(ld)
        args=['/usr/bin/python3','-B',supervisor,'--recording',str(recording),'--output',str(target),'--worker',worker,'--verifier',c['verifier'],'--odom-parameters',c['odom_parameters'],'--map-parameters',c['map_parameters'],'--tag-modules',c['tag_modules'],'--cv-threads','1','--lookahead-segments','1']
        v['phase']=name;before=resources();save();start=time.monotonic()
        with (out/(name+'.log')).open('x') as log:
            proc=subprocess.Popen(args,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:rc=proc.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                raise
        step={'name':name,'seconds':time.monotonic()-start,'exit_code':rc,'resources_before':before,'resources_after':resources()};v['steps'].append(step);save();assert rc==0,name
        state=json.loads((target/'mapping/status.json').read_text());final=json.loads((target/'status.json').read_text());saved=json.loads((target/'save.json').read_text())
        assert saved['complete'] and state['database_closed'] and final['state']=='closed' and final['tag_detection_enabled']
        assert final['completed_segments']==capture['sealed_segments'] and int(final['acknowledged_source_ns'])==int(capture['last_source_ns'])
        assert final.get('awaiting_durable_segments',0)==0
        for file,pin in saved['sha256'].items():assert Path(file).name==file and sha(target/'mapping'/file)==pin
        with sqlite3.connect((target/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:check=db.execute('PRAGMA quick_check').fetchone()[0]
        assert check=='ok'
        rows=list(csv.DictReader((target/'mapping/frames.csv').open()));stamps=[int(x['source_ns']) for x in rows]
        assert len(rows)==capture['frames'] and all(b>a for a,b in zip(stamps,stamps[1:]))
        assert not any(x['main_write']=='1' and (x['lost']=='1' or x['input_present']=='0') for x in rows)
        stamp_sha=hashlib.sha256('\n'.join(map(str,stamps)).encode()).hexdigest()
        assert stamp_sha==next(x['stamp_sha256'] for x in known['steps'] if x['name']=='candidate_'+input_name)
        assert sha(recording/'status.json')==input_pin
        receipts=sorted(target.glob('receipt-*.json'));assert len(receipts)==capture['sealed_segments']
        observations=[]
        for n,f in enumerate(receipts):
            receipt=json.loads(f.read_text());d=target/f'input-{n:08d}.json';desc=json.loads(d.read_text())
            assert receipt['sequence']==n and receipt['input_manifest_sha256']==sha(d) and receipt['last_source_ns']==desc['last_source_ns']
            observations.extend(desc.get('tag_observations',[]))
        log=(target/'worker.log').read_text();assert 'WBROV_RANDOM_DIAGNOSTIC ' not in log
        recovery_events=[];old=0
        for row in rows:
            if int(row['recoveries'])>old:recovery_events.append({'nodes_at_recovery':int(row['main_nodes']),'nodes_at_end':int(rows[-1]['main_nodes'])})
            old=int(row['recoveries'])
        step.update(frames=len(rows),source_seconds=(stamps[-1]-stamps[0])/1e9,stamp_sha256=stamp_sha,main_nodes=state['main_nodes'],cloud_points=state['cloud_points'],cuts=state['cuts'],recoveries=state['recoveries'],recovery_events=recovery_events,
            final_continuity=state['continuity_state'],save_verified=True,database_check=check,saved_sha256=saved['sha256'],tag_observations=len(observations),tag_observations_sha256=hashlib.sha256(json.dumps(observations,sort_keys=True).encode()).hexdigest(),
            tag_ids=sorted({o['tag_id'] for o in observations}),tags=json.loads((target/'mapping/tags.json').read_text())['tags'],timings={k:value for k,value in state.items() if k.endswith('_ms')});save()
        assert all(sha(path)==pin for path,pin in c['sha256'].items())
        assert sha(core/'librtabmap_core.so.0.23.7')==core_pin and sha(build/'wbrov_replay_worker')==built['worker_sha256']
    comparisons={}
    for input_name in inputs:
        b=next(x for x in v['steps'] if x['name']=='baseline_'+input_name);a=next(x for x in v['steps'] if x['name']=='candidate_'+input_name)
        comparisons[input_name]={'baseline_seconds':b['seconds'],'candidate_seconds':a['seconds'],'time_saved_fraction':1-a['seconds']/b['seconds'],
            'same_recorded_frames':a['stamp_sha256']==b['stamp_sha256'],'same_tag_observations':a['tag_observations_sha256']==b['tag_observations_sha256'],
            'same_frames_csv':a['saved_sha256']['frames.csv']==b['saved_sha256']['frames.csv'],'same_cloud':a['saved_sha256']['main_cloud.ply']==b['saved_sha256']['main_cloud.ply']}
    v.update(phase='completed',comparisons=comparisons,
        recovery_continuation_passed=all(s['final_continuity']=='NORMAL' and any(e['nodes_at_end']>e['nodes_at_recovery'] for e in s['recovery_events']) for s in v['steps'] if s['name'].endswith('_short')),
        all_inputs_and_tags_equal=all(x['same_recorded_frames'] and x['same_tag_observations'] for x in comparisons.values()))
    assert v['recovery_continuation_passed'] and v['all_inputs_and_tags_equal'],'ordinary replay failed recovery or complete-input consistency; retain result, no retry'
    v['ordinary_timing_qualifies']=comparisons['field']['time_saved_fraction']>.02 and comparisons['short']['time_saved_fraction']>=-.02
    v['free_gib']=shutil.disk_usage(r).free/2**30
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
