from pathlib import Path
import os,subprocess,json,time,hashlib,csv,sqlite3,traceback
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');out=r/'tag_speed11_replay';out.mkdir(mode=0o700)
v={'phase':'prepare','scope':'fixed four all-frame replays, tags enabled in both controls and candidate; no hardware or deployment; no retry-until-pass','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
 p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
save()
try:
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
 assert json.loads((r/'package_tag_speed11_result.json').read_text())['tag_detection_enabled']
 inputs={'short':r/'motion_spool1','field':Path('/home/exocean/maps/rtabmap_calibration_ab/continuous_20261012_083050_1300795/rgbd_spool')}
 for variant,input_name in [('baseline','short'),('candidate','short'),('candidate','field'),('baseline','field')]:
  name=variant+'_'+input_name;v['phase']=name;save();target=out/name;recording=inputs[input_name]
  p=r/('mapping_round6_candidate11_tags' if variant=='candidate' else 'mapping_round6_candidate10_notags');c=json.loads((p/'entry.json').read_text())
  env=dict(os.environ,**c['environment'],PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
  ld=subprocess.check_output(['ldd',c['worker']],env=env,text=True);assert str(p/'lib/librtabmap_core.so') in ld
  pins={'core':sha(p/'lib/librtabmap_core.so.0.23.7'),'worker':sha(c['worker']),'recording_status':sha(recording/'status.json')}
  args=['/usr/bin/python3','-B',c['supervisor'],'--recording',str(recording),'--output',str(target),'--worker',c['worker'],'--verifier',c['verifier'],'--odom-parameters',c['odom_parameters'],'--map-parameters',c['map_parameters'],'--tag-modules',str(p/'runtime/libexec/wbrov_corner_recovery'),'--cv-threads','1','--lookahead-segments','1']
  start=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(args,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=2400)
  step={'name':name,'seconds':time.monotonic()-start,'exit':q.returncode,'recording':str(recording),'pins':pins};v['steps'].append(step);save();assert q.returncode==0,name
  state=json.loads((target/'mapping/status.json').read_text());saved=json.loads((target/'save.json').read_text());assert saved['complete'] and state['database_closed']
  for file,pin in saved['sha256'].items():assert Path(file).name==file and sha(target/'mapping'/file)==pin
  with sqlite3.connect((target/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:check=db.execute('PRAGMA quick_check').fetchone()[0]
  assert check=='ok'
  rows=list(csv.DictReader((target/'mapping/frames.csv').open()));stamps=[int(x['source_ns']) for x in rows]
  assert len(rows)==json.loads((recording/'status.json').read_text())['frames'] and all(b>a for a,b in zip(stamps,stamps[1:]))
  assert not any(x['main_write']=='1' and (x['lost']=='1' or x['input_present']=='0') for x in rows)
  assert sha(recording/'status.json')==pins['recording_status']
  tags=json.loads((target/'mapping/tags.json').read_text());detected=0;ids=set()
  for f in target.glob('input-*.json'):
   for tag in json.loads(f.read_text()).get('tag_observations',[]):detected+=1;ids.add(tag['tag_id'])
  step.update(frames=len(rows),source_seconds=(stamps[-1]-stamps[0])/1e9,stamp_sha256=hashlib.sha256('\n'.join(map(str,stamps)).encode()).hexdigest(),save_verified=True,database_check=check,main_nodes=state['main_nodes'],cuts=state['cuts'],recoveries=state['recoveries'],final_continuity=state['continuity_state'],cloud_points=state['cloud_points'],detected_observations=detected,detected_ids=sorted(ids),exported_tags=tags['tags'],timings={k:val for k,val in state.items() if k.endswith('_ms')},saved_sha256=saved['sha256']);save()
 comparisons={}
 for name in inputs:
  b=next(s for s in v['steps'] if s['name']=='baseline_'+name);c=next(s for s in v['steps'] if s['name']=='candidate_'+name)
  assert b['stamp_sha256']==c['stamp_sha256']
  comparisons[name]={'baseline_seconds':b['seconds'],'candidate_seconds':c['seconds'],'fraction_time_saved':1-c['seconds']/b['seconds'],'same_frames_csv':b['saved_sha256']['frames.csv']==c['saved_sha256']['frames.csv'],'same_cloud':b['saved_sha256']['main_cloud.ply']==c['saved_sha256']['main_cloud.ply'],'same_detected_ids':b['detected_ids']==c['detected_ids'],'same_main_nodes':b['main_nodes']==c['main_nodes'],'same_recoveries':b['recoveries']==c['recoveries']}
 v.update(phase='completed',comparisons=comparisons,hardware_acceptance=False)
except BaseException as ex:v.update(phase='failed',error=repr(ex),traceback=traceback.format_exc())
save()
