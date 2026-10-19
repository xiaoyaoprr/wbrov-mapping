from pathlib import Path
import os,sys,json,zipfile,subprocess,shutil,time,hashlib,csv,sqlite3
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');out=r/'offline_v30';out.mkdir(mode=0o700)
result={'scope':'offline only; no camera, lights, control or deployment','phase':'preparing','steps':[]}
def save():
 temp=out/'result.tmp';temp.write_text(json.dumps(result,indent=2));temp.replace(out/'result.json')
def step(name):result['phase']=name;save()
def run(cmd,log,timeout=60,env=None,cwd=None):
 with (out/log).open('x') as f:q=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=timeout,env=env,cwd=cwd)
 assert q.returncode==0,(log,q.returncode);return q
try:
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
 assert json.loads((r/'usable_hardware3/result.json').read_text())['phase']=='closed'
 with zipfile.ZipFile(r/'offline_v30_tools.zip') as z:z.extractall(out)
 run(['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-pthread','-I'+str(r/'worker_source_v30'),str(out/'verify_rgbd_spool_codec.cpp'),'-lzstd','-o',str(out/'verify_rgbd_spool_codec')],'codec_build.log')
 run(['/usr/bin/python3','-B','-m','unittest','discover','-s','tests','-p','test_deferred_recorded_pixels.py'],'python_codec_test.log',cwd=out)
 sys.path.insert(0,str(out/'tools'));from deferred_tag_observations import map_pixels
 step('full_byte_exact_codec_verification')
 totals={'frames':0,'raw_bytes':0,'packed_bytes':0,'segments':0,'seconds':0}
 original=r/'live_candidate1/rgbd_spool'
 assert json.loads((original/'status.json').read_text())['clean_shutdown']
 with (out/'codec_segments.jsonl').open('x') as ledger:
  for path in sorted(original.glob('segment-????????.frames')):
   q=subprocess.run([str(out/'verify_rgbd_spool_codec'),str(path),'-'],capture_output=True,text=True,timeout=45);assert q.returncode==0,q.stderr
   v=json.loads(q.stdout);meta=json.loads(path.with_suffix('.json').read_text())
   assert v['frames']==meta['frames'] and int(v['first_source_ns'])==meta['first_source_ns'] and int(v['last_source_ns'])==meta['last_source_ns']
   ledger.write(json.dumps(dict(segment=path.name,**v))+'\n');ledger.flush()
   for k in ['frames','raw_bytes','packed_bytes','seconds']:totals[k]+=v[k]
   totals['segments']+=1;result['codec']=totals;save()
 assert totals['frames']==json.loads((original/'status.json').read_text())['frames']
 step('compressed_recovery_fixture')
 source=r/'motion_spool1';target=out/'motion_spool_v2';target.mkdir()
 for p in source.iterdir():
  if p.is_file() and not p.name.startswith('segment-'):shutil.copyfile(p,target/p.name)
 metadata=json.loads((target/'metadata.json').read_text());metadata['frame_encoding']='zstd-depth-byte-planes-v2';(target/'metadata.json').write_text(json.dumps(metadata))
 for p in sorted(source.glob('segment-????????.frames')):
  q=subprocess.run([str(out/'verify_rgbd_spool_codec'),str(p),str(target/p.name)],capture_output=True,text=True,timeout=45);assert q.returncode==0,q.stderr
  v=json.loads(q.stdout);meta=json.loads(p.with_suffix('.json').read_text());assert v['frames']==meta['frames']
  assert list(map_pixels(p,v['frames']))==list(map_pixels(target/p.name,v['frames']))
  meta['bytes']=(target/p.name).stat().st_size;(target/p.with_suffix('.json').name).write_text(json.dumps(meta))
 result['cross_language_tag_pixels']='all short-fixture observations identical';save()
 runtime=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-439961e59d783abc416c2507a947dfe222965bd6a7ad9da4eed6e203d4d3b31b')
 env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LD_LIBRARY_PATH=str(r/'native_release_v5')+':'+str(r.parent/'round4_20261016/build')+':'+str(runtime/'lib')+':/usr/local/zed/lib:/usr/local/cuda/lib64:/opt/ros/humble/lib')
 def replay(name,recording,partial=False):
  step(name);output=out/name
  argv=['/usr/bin/python3','-B',str(out/'tools/deferred_mapping.py'),'--recording',str(recording),'--output',str(output),'--worker',str(r/'wbrov_replay_worker_v30'),'--verifier',str(r/'camera_build_usable4/build/wbrov_verify_svo'),'--odom-parameters',str(r/'odom_parameters.txt'),'--map-parameters',str(r/'map_parameters.txt'),'--tag-modules',str(r/'mapping_usable_v3/runtime/libexec/wbrov_corner_recovery'),'--cv-threads','1']
  if partial:argv+=['--save-closed-prefix']
  began=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(argv,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=3000)
  elapsed=time.monotonic()-began;assert q.returncode==(2 if partial else 0),(name,q.returncode)
  status=json.loads((output/'status.json').read_text());saved=json.loads((output/'save.json').read_text());assert status['mapping']['database_closed']
  for artifact,pin in saved['sha256'].items():assert Path(artifact).name==artifact and hashlib.sha256((output/'mapping'/artifact).read_bytes()).hexdigest()==pin
  with sqlite3.connect((output/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:check=db.execute('PRAGMA quick_check').fetchone()[0]
  rows=list(csv.DictReader((output/'mapping/frames.csv').open()));assert len(rows)==json.loads((recording/'status.json').read_text())['frames']
  invalid=sum(x['main_write']=='1' and (x['lost']=='1' or x['input_present']=='0') for x in rows);assert invalid==0 and check=='ok'
  events=[];previous=0
  for i,row in enumerate(rows):
   if int(row['recoveries'])>previous:events.append({'source_ns':row['source_ns'],'nodes_at_recovery':int(row['main_nodes']),'nodes_final':int(rows[-1]['main_nodes']),'continued_seconds':(int(rows[-1]['source_ns'])-int(row['source_ns']))/1e9})
   previous=int(row['recoveries'])
  q=subprocess.run([str(r/'verify_recorded_cloud_export'),str(output/'mapping')],env=env,capture_output=True,text=True,timeout=180);assert q.returncode==0,q.stdout+q.stderr
  v={'exit':q.returncode,'seconds':elapsed,'frames':len(rows),'invalid_write_frames':invalid,'database_check':check,'recovery_events':events,'final':status['mapping'],'cloud_differential':q.stdout+q.stderr,'complete':saved['complete']}
  if not partial:assert status['mapping']['main_map_write_allowed'] and any(e['nodes_final']>e['nodes_at_recovery'] for e in events),'recovery/continued mapping not accepted'
  result['steps'].append(dict(name=name,**v));save()
 replay('short_compressed_recovery',target)
 replay('full_recovery',original)
 replay('large_interrupted_capture',Path('/home/exocean/maps/rtabmap_calibration_ab/continuous_20261011_092000_500034/rgbd_spool'),True)
 result['phase']='passed'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
