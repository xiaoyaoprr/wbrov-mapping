from pathlib import Path
import os,sys,json,zipfile,subprocess,shutil,time,hashlib,csv,sqlite3
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');out=r/'offline_status40';out.mkdir(mode=0o700)
result={'scope':'progress-snapshot durability candidate with unchanged Core5 and worker39; terminal statuses and all input/receipt/save checks retained','phase':'preparing','steps':[]}
def save():
 temp=out/'result.tmp';temp.write_text(json.dumps(result,indent=2));temp.replace(out/'result.json')
def step(name):result['phase']=name;save()
def run(cmd,log,timeout=60,env=None,cwd=None):
 with (out/log).open('x') as f:q=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=timeout,env=env,cwd=cwd)
 assert q.returncode==0,(log,q.returncode);return q
try:
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
 assert json.loads((r/'worker_build_v39.json').read_text())['stable_cache_scan_test']=='passed'
 assert json.loads((r/'status40_snapshot_probe/result.json').read_text())['phase']=='finished'
 result['core_sha256']=hashlib.sha256((r/'native_release_v5/librtabmap_core.so.0.23.7').read_bytes()).hexdigest()
 assert result['core_sha256']=='573f9a1cd6a6ac257efd5621b421b9650f14fcbfc95cfcd915f5c9f479c6eed7'
 result['supervisor_sha256']=hashlib.sha256((r/'status40_snapshot_probe/project/tools/deferred_mapping.py').read_bytes()).hexdigest();save()
 original=r/'live_candidate1/rgbd_spool'
 (out/'tools').symlink_to(r/'status40_snapshot_probe/project/tools',target_is_directory=True)
 runtime=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-439961e59d783abc416c2507a947dfe222965bd6a7ad9da4eed6e203d4d3b31b')
 env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LD_LIBRARY_PATH=str(r/'native_release_v5')+':'+str(r.parent/'round4_20261016/build')+':'+str(runtime/'lib')+':/usr/local/zed/lib:/usr/local/cuda/lib64:/opt/ros/humble/lib')
 def replay(name,recording,version,partial=False):
  step(name);output=out/name
  argv=['/usr/bin/python3','-B',str(r/'profile_supervisor40.py'),str(out/'tools'),'--recording',str(recording),'--output',str(output),'--worker',str(r/('wbrov_replay_worker_'+version)),'--verifier',str(r/'camera_build_usable4/build/wbrov_verify_svo'),'--odom-parameters',str(r/'odom_parameters.txt'),'--map-parameters',str(r/'map_parameters.txt'),'--tag-modules',str(r/'mapping_usable_v3/runtime/libexec/wbrov_corner_recovery'),'--cv-threads','1','--lookahead-segments','1']
  if partial:argv+=['--save-closed-prefix']
  began=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(argv,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=3000)
  elapsed=time.monotonic()-began;assert q.returncode==(2 if partial else 0),(name,q.returncode)
  status=json.loads((output/'status.json').read_text());saved=json.loads((output/'save.json').read_text());assert status['mapping']['database_closed']
  assert status['mapping']['cloud_export_workers']==2 and status['mapping']['cloud_parallel_nodes']>0
  for artifact,pin in saved['sha256'].items():assert Path(artifact).name==artifact and hashlib.sha256((output/'mapping'/artifact).read_bytes()).hexdigest()==pin
  with sqlite3.connect((output/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:check=db.execute('PRAGMA quick_check').fetchone()[0]
  rows=list(csv.DictReader((output/'mapping/frames.csv').open()));assert len(rows)==json.loads((recording/'status.json').read_text())['frames']
  invalid=sum(x['main_write']=='1' and (x['lost']=='1' or x['input_present']=='0') for x in rows);assert invalid==0 and check=='ok'
  events=[];previous=0
  for i,row in enumerate(rows):
   if int(row['recoveries'])>previous:events.append({'source_ns':row['source_ns'],'nodes_at_recovery':int(row['main_nodes']),'nodes_final':int(rows[-1]['main_nodes']),'continued_seconds':(int(rows[-1]['source_ns'])-int(row['source_ns']))/1e9})
   previous=int(row['recoveries'])
  q=subprocess.run([str(r/'verify_recorded_cloud_export'),str(output/'mapping')],env=env,capture_output=True,text=True,timeout=180);assert q.returncode==0,q.stdout+q.stderr
  v={'cloud_verifier_exit':q.returncode,'supervisor_exit':2 if partial else 0,'seconds':elapsed,'frames':len(rows),'invalid_write_frames':invalid,'database_check':check,'recovery_events':events,'final':status['mapping'],'cloud_differential':q.stdout+q.stderr,'complete':saved['complete']}
  v['supervisor_profile']=json.loads((output/'supervisor_timings.json').read_text());assert v['supervisor_profile']['tools_unchanged']
  v['worker_version']=version
  v['worker_sha256']=hashlib.sha256((r/('wbrov_replay_worker_'+version)).read_bytes()).hexdigest()
  v['recording']=str(recording)
  v['recovery_required']=not partial
  v['recovery_accepted']=bool(status['mapping']['main_map_write_allowed'] and any(e['nodes_final']>e['nodes_at_recovery'] for e in events))
  v['last_reason']=rows[-1]['reason']
  result['steps'].append(dict(name=name,**v));save()
 replay('full_compressed',r/'compressed_full39/rgbd_spool','v39')
 replay('large_interrupted_capture',Path('/home/exocean/maps/rtabmap_calibration_ab/continuous_20261011_092000_500034/rgbd_spool'),'v39',True)
 result['recovery_acceptance_passed']=all(not s['recovery_required'] or s['recovery_accepted'] for s in result['steps'])
 result['phase']='passed' if result['recovery_acceptance_passed'] else 'completed_with_recovery_failures'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
