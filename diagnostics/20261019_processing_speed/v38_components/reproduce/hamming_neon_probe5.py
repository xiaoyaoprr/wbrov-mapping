from pathlib import Path
import subprocess,time,json,hashlib,zipfile,shlex,os
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'hamming_neon_probe5';out.mkdir(mode=0o700)
result={'phase':'waiting_for_v38','scope':'same standalone Hamming experiment; minimal utilite/lz4 link after probe4 unnecessary Core dependency failure; no Core/runtime mutation','steps':[]}
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
def run(name,cmd,timeout):
 result['phase']=name;save();began=time.monotonic()
 with (out/(name+'.log')).open('x') as log:q=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=timeout,env=env)
 result['steps'].append({'name':name,'exit':q.returncode,'seconds':time.monotonic()-began});save();assert q.returncode==0,name
save()
try:
 began=time.monotonic()
 while True:
  phase=json.loads((r/'speed38_pipeline/result.json').read_text())['phase']
  if phase in ('passed','completed_with_recovery_failures','failed'):break
  assert time.monotonic()-began<7200;time.sleep(5)
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 prior=r/'hamming_neon_probe4';receipt=json.loads((prior/'result.json').read_text())
 assert receipt['steps'][0]['name']=='sanitizer_compile' and receipt['steps'][0]['exit']==0
 assert hashlib.sha256((r/'probe_hamming_neon.cpp').read_bytes()).hexdigest()==receipt['probe_sha256']
 source=prior/'source'
 with zipfile.ZipFile(r/'rtflann_probe_headers.zip') as z:
  for name,pin in json.loads(z.read('pins.json')).items():
   assert not Path(name).is_absolute() and '..' not in Path(name).parts
   assert hashlib.sha256((source/name).read_bytes()).hexdigest()==pin
 b=r.parent/'round4_20261016/build';link=shlex.split((b/'CMakeFiles/test_visual_recovery_native.dir/link.txt').read_text())
 utilite=[x for x in link if 'librtabmap_utilite' in x];assert len(utilite)==1 and Path(utilite[0]).is_file()
 result.update(probe_sha256=receipt['probe_sha256'],header_archive_sha256=receipt['header_archive_sha256'],baseline_dist_sha256=receipt['baseline_dist_sha256'],sanitizer_object_sha256=hashlib.sha256((prior/'sanitizer.o').read_bytes()).hexdigest(),utilite_sha256=hashlib.sha256(Path(utilite[0]).read_bytes()).hexdigest())
 runtime=['-llz4',utilite[0],'-pthread','-Wl,-rpath,'+str(Path(utilite[0]).parent)]
 run('sanitizer_relink',['/usr/bin/c++',str(prior/'sanitizer.o'),'-o',str(out/'sanitizer'),'-fsanitize=address,undefined',*runtime],60)
 run('sanitizer',[str(out/'sanitizer')],60)
 flags=(b/'CMakeFiles/test_visual_recovery_native.dir/flags.make').read_text();args=[]
 for name in ('CXX_DEFINES','CXX_INCLUDES'):args+=shlex.split(next(x.split('=',1)[1].strip() for x in flags.splitlines() if x.startswith(name+' =')))
 run('release_build',['/usr/bin/c++','-std=c++17','-O3','-UNDEBUG','-I'+str(source),*args,str(r/'probe_hamming_neon.cpp'),'-o',str(out/'benchmark'),*runtime],180)
 run('fixed_index_benchmark',[str(out/'benchmark'),str(out/'saved')],180)
 result['measurements']=[json.loads(line) for line in (out/'fixed_index_benchmark.log').read_text().splitlines() if line.startswith('{')]
 assert len(result['measurements'])==12 and all(x['indices_and_distances_equal'] for x in result['measurements'])
 result['phase']='finished'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
