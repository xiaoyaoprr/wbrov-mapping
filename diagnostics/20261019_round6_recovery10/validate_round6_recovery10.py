from pathlib import Path
import os,subprocess,json,time,hashlib,csv,sqlite3,traceback
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
c=r/'round6_recovery9_build';out=r/'round6_recovery10_replay';out.mkdir(mode=0o700)
report={'phase':'prepare','scope':'single all-frame duplicate-search fix after the retained fixed pair, Tags disabled, no hardware/deployment; retain every outcome','steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(report,indent=2));p.replace(out/'result.json')
save()
try:
 built=json.loads((c/'resume2_result.json').read_text());assert built['phase']=='passed'
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
 cfg=json.loads((r/'mapping_round6_candidate8_notags/entry.json').read_text())
 baseenv=dict(os.environ,**cfg['environment'],PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 tool=r/'mapping_round6_candidate8_notags/tools/deferred_mapping.py'
 sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
 def replay(name,recording,candidate,partial=False):
  report['phase']=name;save();target=out/name
  env=dict(baseenv)
  if candidate:env['LD_LIBRARY_PATH']=str(c)+':'+env['LD_LIBRARY_PATH']
  worker=r/'round6_recovery10_build/wbrov_replay_worker' if candidate else Path(cfg['worker'])
  # Bind actual loaded Core before measuring; old worker is paired only with old Core.
  loaded=subprocess.check_output(['ldd',str(worker)],env=env,text=True)
  expected=c if candidate else r/'mapping_round6_candidate8_notags/lib'
  assert str(expected/'librtabmap_core.so') in loaded,loaded
  core=expected/'librtabmap_core.so.0.23.7'
  assert sha(core)==('a6b4397cd6a5ed4cf06e54cad95d4205734297f2da31f07684b517e17cc17db4' if candidate else '0e68b8fe830df0c60b23d0043d94f2cf20a27ea61795e83726ed40a67eb6b14b')
  pins={'core':sha(core),'worker':sha(worker),'recording_status':sha(recording/'status.json')}
  argv=['/usr/bin/python3','-B',str(tool),'--recording',str(recording),'--output',str(target),'--worker',str(worker),
   '--verifier',cfg['verifier'],'--odom-parameters',cfg['odom_parameters'],'--map-parameters',cfg['map_parameters'],'--cv-threads','1','--lookahead-segments','1']
  if partial:argv+=['--save-closed-prefix']
  start=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(argv,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=2400)
  seconds=time.monotonic()-start
  step={'name':name,'seconds':seconds,'exit':q.returncode,'pins':pins,'recording':str(recording)};report['steps'].append(step);save()
  assert q.returncode==(2 if partial else 0),step
  saved=json.loads((target/'save.json').read_text());state=json.loads((target/'mapping/status.json').read_text())
  assert state['database_closed'] and saved['complete']==(not partial)
  for name,pin in saved['sha256'].items():assert Path(name).name==name and sha(target/'mapping'/name)==pin
  with sqlite3.connect((target/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:check=db.execute('PRAGMA quick_check').fetchone()[0]
  assert check=='ok'
  rows=list(csv.DictReader((target/'mapping/frames.csv').open()));stamps=[int(row['source_ns']) for row in rows]
  assert len(rows)==json.loads((recording/'status.json').read_text())['frames']
  assert all(b>a for a,b in zip(stamps,stamps[1:]))
  assert not any(row['main_write']=='1' and (row['lost']=='1' or row['input_present']=='0') for row in rows)
  assert not json.loads((target/'mapping/tags.json').read_text())['tags']
  for key,p in [('core',core),('worker',worker),('recording_status',recording/'status.json')]:assert sha(p)==pins[key]
  events=[];last=0
  for row in rows:
   if int(row['recoveries'])>last:events.append(row)
   last=int(row['recoveries'])
  step.update(frames=len(rows),stamp_sha256=hashlib.sha256('\n'.join(map(str,stamps)).encode()).hexdigest(),source_seconds=(stamps[-1]-stamps[0])/1e9,
   save_verified=True,database_check=check,final=state,recovery_events=events,complete=saved['complete']);save()
 assert json.loads((r/'round6_recovery10_build/result.json').read_text())['phase']=='passed'
 replay('candidate_motion',r/'motion_spool1',True)
 assert report['steps'][0]['stamp_sha256']==json.loads((r/'round6_recovery9_replay2/result.json').read_text())['steps'][0]['stamp_sha256']
 report['phase']='completed';report['candidate_recovery_observed']=bool(report['steps'][0]['recovery_events']);report['hardware_acceptance']=False
except BaseException as ex:report.update(phase='failed',error=repr(ex),traceback=traceback.format_exc())
save()


