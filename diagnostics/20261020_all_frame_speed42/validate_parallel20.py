from pathlib import Path
import os,json,time,hashlib,csv,sqlite3,traceback,subprocess,shutil,fcntl,shlex,signal
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'parallel20_validation';out.mkdir(mode=0o700)
v={'phase':'prepare','scope':'four fixed-order full-frame replays, projection query cores 1 versus 2; native diagnostic entropy fixed; PASSIVE OpenMP wait in both; no deployment or field acceptance','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
    p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
def thermal():
    data={}
    for p in [*Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'),*Path('/sys/class/thermal').glob('thermal_zone*/temp')]:
        try:data[str(p)]=p.read_text().strip()
        except OSError:pass
    return data
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    build=r/'parallel20_build';built=json.loads((build/'result.json').read_text());assert built['phase']=='completed'
    assert sha(build/'librtabmap_core.so.0.23.7')==built['core_sha256']
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        assert not any(x in cmd for x in ['/bin/wbrov_replay_worker','/bin/zed_rgbd_publisher_rate_ab','/tools/continuous_mapping_session.py']),cmd
    assert shutil.disk_usage(r).free>10*2**30
    package=r/'mapping_round6_candidate11_tags';c=json.loads((package/'entry.json').read_text())
    assert all(sha(p)==pin for p,pin in c['sha256'].items())
    base_env=dict(os.environ,**c['environment'],PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE')
    assert 'LD_PRELOAD' not in base_env and 'WBROV_OFFLINE_PHASE_DIAGNOSTIC' not in base_env
    random=r/'tag_speed11_fixed_random2/fixed_random_diagnostic.so'
    assert sha(random)=='d5a6470d903993a47ce515f2b2bdecc512b3fc1bfe568fd2243600b19960e0bf'
    worker=out/'offline_fixed_worker.sh'
    worker.write_text('#!/bin/sh\nexport WBROV_OFFLINE_RANDOM_DIAGNOSTIC=1\nexport LD_PRELOAD='+shlex.quote(str(random))+'\nexec '+shlex.quote(c['worker'])+' "$@"\n');worker.chmod(0o700)
    v['pins']={'candidate_core':built['core_sha256'],'baseline_core':sha(package/'lib/librtabmap_core.so.0.23.7'),'native_worker':sha(c['worker']),'supervisor':sha(c['supervisor']),'fixed_entropy':sha(random)};save()
    known=json.loads((r/'receipt14_validation/result.json').read_text());assert known['phase']=='completed' and known['output_equivalence_passed']
    inputs={'short':r/'motion_spool1','field':Path('/home/exocean/maps/rtabmap_calibration_ab/continuous_20261012_083050_1300795/rgbd_spool')}
    for variant,input_name in [('baseline','short'),('candidate','short'),('candidate','field'),('baseline','field')]:
        name=variant+'_'+input_name;recording=inputs[input_name];target=out/name
        env=base_env.copy()
        if variant=='candidate':env['LD_LIBRARY_PATH']=str(build)+':'+env['LD_LIBRARY_PATH']
        linked=subprocess.check_output(['ldd',c['worker']],env=env,text=True)
        expected_core=build if variant=='candidate' else package/'lib'
        assert str(expected_core/'librtabmap_core.so') in linked,linked
        (out/(name+'.ldd')).write_text(linked)
        capture=json.loads((recording/'status.json').read_text());input_pin=sha(recording/'status.json')
        assert capture['state']=='closed' and capture['clean_shutdown']
        v['phase']=name;save();before=thermal();start=time.monotonic()
        args=['/usr/bin/python3','-B',c['supervisor'],'--recording',str(recording),'--output',str(target),'--worker',str(worker),'--verifier',c['verifier'],'--odom-parameters',c['odom_parameters'],'--map-parameters',c['map_parameters'],'--tag-modules',c['tag_modules'],'--cv-threads','1','--lookahead-segments','1']
        with (out/(name+'.log')).open('x') as f:
            proc=subprocess.Popen(args,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:rc=proc.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                raise
        step={'name':name,'seconds':time.monotonic()-start,'exit_code':rc,'thermal_before':before,'thermal_after':thermal()};v['steps'].append(step);save();assert rc==0,name
        status=json.loads((target/'status.json').read_text());state=json.loads((target/'mapping/status.json').read_text());saved=json.loads((target/'save.json').read_text())
        assert saved['complete'] and state['database_closed'] and status['state']=='closed' and status['tag_detection_enabled']
        assert status['completed_segments']==capture['sealed_segments'] and int(status['acknowledged_source_ns'])==int(capture['last_source_ns'])
        for file,pin in saved['sha256'].items():assert Path(file).name==file and sha(target/'mapping'/file)==pin
        with sqlite3.connect((target/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:check=db.execute('PRAGMA quick_check').fetchone()[0]
        assert check=='ok'
        rows=list(csv.DictReader((target/'mapping/frames.csv').open()));stamps=[int(row['source_ns']) for row in rows]
        assert len(rows)==capture['frames'] and all(b>a for a,b in zip(stamps,stamps[1:]))
        assert not any(row['main_write']=='1' and (row['lost']=='1' or row['input_present']=='0') for row in rows)
        reference=next(s for s in known['steps'] if s['name']=='baseline_'+input_name)
        stamp_sha=hashlib.sha256('\n'.join(map(str,stamps)).encode()).hexdigest();assert stamp_sha==reference['stamp_sha256']
        assert sha(recording/'status.json')==input_pin
        receipts=sorted(target.glob('receipt-*.json'));assert len(receipts)==capture['sealed_segments']
        for n,f in enumerate(receipts):
            receipt=json.loads(f.read_text());d=target/f'input-{n:08d}.json'
            assert receipt['sequence']==n and receipt['input_manifest_sha256']==sha(d) and receipt['last_source_ns']==json.loads(d.read_text())['last_source_ns']
        text=(target/'worker.log').read_text();random_lines=[line for line in text.splitlines() if line.startswith('WBROV_RANDOM_DIAGNOSTIC ')]
        same={file:saved['sha256'][file]==reference['saved_sha256'][file] for file in ['frames.csv','main_cloud.ply','tags.json']}
        previous=0;events=[]
        for row in rows:
            if int(row['recoveries'])>previous:events.append({'nodes_at_recovery':int(row['main_nodes']),'nodes_at_end':int(rows[-1]['main_nodes'])})
            previous=int(row['recoveries'])
        step.update(frames=len(rows),stamp_sha256=stamp_sha,main_nodes=state['main_nodes'],cloud_points=state['cloud_points'],cuts=state['cuts'],recoveries=state['recoveries'],recovery_events=events,final_continuity=state['continuity_state'],save_verified=True,database_check=check,output_equivalence=same,same_random_calls=random_lines==[reference['random_diagnostic']],random_lines=random_lines,saved_sha256=saved['sha256'],timings={k:value for k,value in state.items() if k.endswith('_ms')});save()
        assert all(same.values()) and step['same_random_calls'],'output or entropy differs; retain failure and inspect, no retry-until-pass'
        assert input_name!='short' or any(e['nodes_at_end']>e['nodes_at_recovery'] for e in events),'short recovery did not continue mapping'
        assert all(sha(p)==pin for p,pin in c['sha256'].items()) and sha(build/'librtabmap_core.so.0.23.7')==built['core_sha256']
    comparisons={}
    for input_name in inputs:
        b=next(x for x in v['steps'] if x['name']=='baseline_'+input_name);a=next(x for x in v['steps'] if x['name']=='candidate_'+input_name)
        comparisons[input_name]={'baseline_seconds':b['seconds'],'candidate_seconds':a['seconds'],'time_saved_fraction':1-a['seconds']/b['seconds'],'same_frames_cloud_tags':all(b['saved_sha256'][file]==a['saved_sha256'][file] for file in ['frames.csv','main_cloud.ply','tags.json'])}
    v.update(phase='completed',comparisons=comparisons,output_equivalence_passed=True,recovery_continuation_passed=True,free_gib=shutil.disk_usage(r).free/2**30)
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
