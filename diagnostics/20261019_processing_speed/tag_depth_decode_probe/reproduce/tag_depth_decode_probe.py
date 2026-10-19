from pathlib import Path
import os,subprocess,shlex,time,json,hashlib,statistics
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'tag_depth_decode_probe';out.mkdir(mode=0o700)
result={'phase':'waiting_for_existing_jobs','scope':'closed v35 DB display-only depth decoder experiment; no Core, worker, camera or production changes','steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
def run(name,cmd,timeout,env=None,cwd=None):
 result['phase']=name;save();began=time.monotonic()
 with (out/(name+'.log')).open('x') as log:q=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=timeout,env=env,cwd=cwd)
 result['steps'].append({'name':name,'exit':q.returncode,'seconds':time.monotonic()-began});save();assert q.returncode==0,name
save()
try:
 began=time.monotonic()
 while True:
  integrated=json.loads((r/'speed37_pipeline/result.json').read_text())['phase']
  hamming=json.loads((r/'hamming_neon_probe3/result.json').read_text())['phase']
  if integrated in ('passed','completed_with_recovery_failures','failed') and hamming in ('finished','failed'):break
  assert time.monotonic()-began<7200;time.sleep(5)
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 b=r.parent/'round4_20261016/build'
 runtime=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-439961e59d783abc416c2507a947dfe222965bd6a7ad9da4eed6e203d4d3b31b')
 env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LD_LIBRARY_PATH=str(r/'native_release_v5')+':'+str(b)+':'+str(runtime/'lib')+':/usr/local/zed/lib:/usr/local/cuda/lib64:/opt/ros/humble/lib')
 flags=(b/'CMakeFiles/test_visual_recovery_native.dir/flags.make').read_text();args=[]
 for name in ('CXX_DEFINES','CXX_INCLUDES'):args+=shlex.split(next(x.split('=',1)[1].strip() for x in flags.splitlines() if x.startswith(name+' =')))
 result['probe_sha256']=hashlib.sha256((r/'probe_tag_depth_decode.cpp').read_bytes()).hexdigest();save()
 run('compile',['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-I'+str(r/'native_release_v5/core/headers'),*args,'-c',str(r/'probe_tag_depth_decode.cpp'),'-o',str(out/'probe.o')],90)
 link=shlex.split((b/'CMakeFiles/test_visual_recovery_native.dir/link.txt').read_text())
 link=[str(out/'probe.o') if x.endswith('.o') else str(out/'probe') if x=='test_visual_recovery_native' else str(r/'native_release_v5/librtabmap_core.so') if 'librtabmap_core.so' in x else x for x in link]
 link+=['-Wl,-rpath-link,'+str(runtime/'lib')];run('link',link,45,cwd=b)
 result['maps']=[]
 for name in ('full_recovery','large_interrupted_capture'):
  replay=r/'offline_v35'/name;mapping=replay/'mapping'
  status=json.loads((mapping/'status.json').read_text());assert status['state']=='closed' and status['database_closed']
  tags=json.loads((mapping/'tags.json').read_text());node_by_stamp={str(x['source_ns']):x['node_id'] for x in tags['trajectory']}
  obs=[];input_pins={}
  for file in sorted(replay.glob('input-????????.json')):
   input_pins[file.name]=hashlib.sha256(file.read_bytes()).hexdigest()
   for item in json.loads(file.read_text()).get('tag_observations',[]):
    if str(item['source_ns']) in node_by_stamp:obs.append({'node':node_by_stamp[str(item['source_ns'])],'tag':item['tag_id'],'corners':item['corners_xy']})
  assert obs and len(node_by_stamp)==status['main_nodes']
  manifest=out/(name+'.json');manifest.write_text(json.dumps({'observations':obs,'input_pins':input_pins},indent=2))
  db_pin=hashlib.sha256((mapping/'map.db').read_bytes()).hexdigest()
  run(name,[str(out/'probe'),str(mapping),str(manifest)],180,env=env)
  assert hashlib.sha256((mapping/'map.db').read_bytes()).hexdigest()==db_pin
  measurements=[json.loads(line) for line in (out/(name+'.log')).read_text().splitlines() if line.startswith('{')]
  assert len(measurements)==6 and all(x['depth_and_marker_exact'] and x['export_bytes_equal'] for x in measurements)
  result['maps'].append({'name':name,'db_sha256':db_pin,'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),'measurements':measurements,'before_median_ms_excluding_warmup':statistics.median(x['before_ms'] for x in measurements[1:]),'after_median_ms_excluding_warmup':statistics.median(x['after_ms'] for x in measurements[1:])});save()
 result['phase']='finished'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
