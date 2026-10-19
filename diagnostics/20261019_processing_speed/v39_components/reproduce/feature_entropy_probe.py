from pathlib import Path
import os,subprocess,shlex,time,json,hashlib
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'feature_entropy_probe';out.mkdir(mode=0o700)
result={'phase':'waiting_for_l1','scope':'offline feature-preparation A/A diagnostic; process-local record/replay of actual random_device values only in this test executable, no Core/worker changes or deployment','steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
def run(name,cmd,timeout,env=None,cwd=None,allowed=(0,)):
 result['phase']=name;save();began=time.monotonic()
 with (out/(name+'.log')).open('x') as log:q=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=timeout,env=env,cwd=cwd)
 result['steps'].append({'name':name,'exit':q.returncode,'seconds':time.monotonic()-began});save();assert q.returncode in allowed,name
save()
try:
 began=time.monotonic()
 while True:
  phase=json.loads((r/'l1_neon_probe/result.json').read_text())['phase']
  if phase in ('finished','failed'):break
  assert time.monotonic()-began<9000;time.sleep(5)
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 b=r.parent/'round4_20261016/build'
 runtime=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-439961e59d783abc416c2507a947dfe222965bd6a7ad9da4eed6e203d4d3b31b')
 env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LD_LIBRARY_PATH=str(r/'native_release_v5')+':'+str(b)+':'+str(runtime/'lib')+':/usr/local/zed/lib:/usr/local/cuda/lib64:/opt/ros/humble/lib')
 core=r/'native_release_v5/librtabmap_core.so.0.23.7'
 core_pin=hashlib.sha256(core.read_bytes()).hexdigest();assert core_pin=='573f9a1cd6a6ac257efd5621b421b9650f14fcbfc95cfcd915f5c9f479c6eed7'
 flags=(b/'CMakeFiles/test_visual_recovery_native.dir/flags.make').read_text();args=[]
 for name in ('CXX_DEFINES','CXX_INCLUDES'):args+=shlex.split(next(x.split('=',1)[1].strip() for x in flags.splitlines() if x.startswith(name+' =')))
 result['source_sha256']=hashlib.sha256((r/'probe_feature_entropy.cpp').read_bytes()).hexdigest();save()
 run('compile',['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-I'+str(r/'native_release_v5/core/headers'),*args,'-c',str(r/'probe_feature_entropy.cpp'),'-o',str(out/'probe.o')],120)
 link=shlex.split((b/'CMakeFiles/test_visual_recovery_native.dir/link.txt').read_text())
 link=[str(out/'probe.o') if x.endswith('.o') else str(out/'probe') if x=='test_visual_recovery_native' else str(r/'native_release_v5/librtabmap_core.so') if 'librtabmap_core.so' in x else x for x in link]
 link+=['-Wl,-rpath-link,'+str(runtime/'lib'),'-Wl,--export-dynamic-symbol=_ZNSt13random_device9_M_getvalEv','-ldl'];run('link',link,60,cwd=b)
 symbols=subprocess.check_output(['nm','-D',str(out/'probe')],text=True)
 assert any(' T _ZNSt13random_device9_M_getvalEv' in line for line in symbols.splitlines()),'missing explicit diagnostic export'
 result['maps']=[]
 for name in ('full_recovery','large_interrupted_capture'):
  mapping=r/'offline_v35'/name/'mapping';status=json.loads((mapping/'status.json').read_text());assert status['state']=='closed' and status['database_closed']
  pin=hashlib.sha256((mapping/'map.db').read_bytes()).hexdigest()
  run(name,[str(out/'probe'),str(mapping),str(r/'map_parameters.txt'),'--diagnostic-entropy-replay'],300,env=env,allowed=(0,2))
  rows=[json.loads(line) for line in (out/(name+'.log')).read_text().splitlines() if line.startswith('{')]
  assert rows[-1].get('summary') and rows[-1]['pairs']==24 and rows[-1]['entropy_calls']>0
  assert hashlib.sha256((mapping/'map.db').read_bytes()).hexdigest()==pin
  result['maps'].append({'name':name,'db_sha256':pin,'summary':rows[-1]});save()
 assert hashlib.sha256(core.read_bytes()).hexdigest()==core_pin
 result['core_sha256']=core_pin;result['phase']='finished';result['adopted']=False
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
