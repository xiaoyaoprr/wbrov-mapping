from pathlib import Path
import subprocess,time,json,hashlib,zipfile,shlex,os
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'hamming_neon_probe4';out.mkdir(mode=0o700)
result={'phase':'waiting_for_tag_probe','scope':'same isolated exact Hamming experiment; correct build dependencies after probe3 compile failure; Core/runtime unchanged','steps':[]}
b=r.parent/'round4_20261016/build'
runtime=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-439961e59d783abc416c2507a947dfe222965bd6a7ad9da4eed6e203d4d3b31b')
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LD_LIBRARY_PATH=str(r/'native_release_v5')+':'+str(b)+':'+str(runtime/'lib')+':/usr/local/zed/lib:/usr/local/cuda/lib64:/opt/ros/humble/lib')
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
def run(name,cmd,timeout,cwd=None):
 result['phase']=name;save();began=time.monotonic()
 with (out/(name+'.log')).open('x') as log:q=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=timeout,cwd=cwd,env=env)
 result['steps'].append({'name':name,'exit':q.returncode,'seconds':time.monotonic()-began});save();assert q.returncode==0,name
save()
try:
 began=time.monotonic()
 while True:
  phase=json.loads((r/'tag_depth_decode_probe/result.json').read_text())['phase']
  if phase in ('finished','failed'):break
  assert time.monotonic()-began<2000;time.sleep(5)
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 source=out/'source';source.mkdir()
 with zipfile.ZipFile(r/'rtflann_probe_headers.zip') as z:
  pins=json.loads(z.read('pins.json'))
  for name,pin in pins.items():
   assert not Path(name).is_absolute() and '..' not in Path(name).parts
   data=z.read(name);assert hashlib.sha256(data).hexdigest()==pin
   p=source/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
 result['header_archive_sha256']=hashlib.sha256((r/'rtflann_probe_headers.zip').read_bytes()).hexdigest()
 result['header_source']='current repository headers; standalone experiment, not proof of the headers used to compile installed Core'
 result['baseline_dist_sha256']=hashlib.sha256((source/'rtflann/algorithms/dist.h').read_bytes()).hexdigest()
 result['probe_sha256']=hashlib.sha256((r/'probe_hamming_neon.cpp').read_bytes()).hexdigest();save()
 flags=(b/'CMakeFiles/test_visual_recovery_native.dir/flags.make').read_text();args=[]
 for name in ('CXX_DEFINES','CXX_INCLUDES'):args+=shlex.split(next(x.split('=',1)[1].strip() for x in flags.splitlines() if x.startswith(name+' =')))
 common=['/usr/bin/c++','-std=c++17','-UNDEBUG','-I'+str(source),'-I'+str(r/'native_release_v5/core/headers'),*args]
 base_link=shlex.split((b/'CMakeFiles/test_visual_recovery_native.dir/link.txt').read_text())
 for label,options in [('sanitizer',['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer']),('release',['-O3'])]:
  obj=out/(label+'.o');binary=out/label
  run(label+'_compile',common+options+['-c',str(r/'probe_hamming_neon.cpp'),'-o',str(obj)],240)
  link=[str(obj) if x.endswith('.o') else str(binary) if x=='test_visual_recovery_native' else str(r/'native_release_v5/librtabmap_core.so') if 'librtabmap_core.so' in x else x for x in base_link]
  link+=['-llz4','-Wl,-rpath-link,'+str(runtime/'lib')]
  if label=='sanitizer':link+=['-fsanitize=address,undefined']
  run(label+'_link',link,60,cwd=b)
  if label=='sanitizer':run('sanitizer',[str(binary)],60)
 run('fixed_index_benchmark',[str(out/'release'),str(out/'saved')],180)
 result['measurements']=[json.loads(line) for line in (out/'fixed_index_benchmark.log').read_text().splitlines() if line.startswith('{')]
 assert len(result['measurements'])==12 and all(x['indices_and_distances_equal'] for x in result['measurements'])
 result['phase']='finished'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
