from pathlib import Path
import subprocess,time,json,hashlib,zipfile
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'hamming_neon_probe3';out.mkdir(mode=0o700)
result={'phase':'waiting_for_integrated37','scope':'isolated exact Hamming distance and fixed saved LSH index experiment; Core/runtime unchanged','steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
def run(name,cmd,timeout):
 result['phase']=name;save();began=time.monotonic()
 with (out/(name+'.log')).open('x') as log:q=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=timeout)
 result['steps'].append({'name':name,'exit':q.returncode,'seconds':time.monotonic()-began});save();assert q.returncode==0,name
save()
try:
 began=time.monotonic()
 while True:
  phase=json.loads((r/'speed37_pipeline/result.json').read_text())['phase']
  if phase in ('passed','completed_with_recovery_failures','failed'):break
  assert time.monotonic()-began<7200;time.sleep(5)
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 source=out/'source';source.mkdir()
 with zipfile.ZipFile(r/'rtflann_probe_headers.zip') as z:
  pins=json.loads(z.read('pins.json'))
  for name,pin in pins.items():
   assert not Path(name).is_absolute() and '..' not in Path(name).parts
   data=z.read(name);assert hashlib.sha256(data).hexdigest()==pin
   p=source/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
 result['header_archive_sha256']=hashlib.sha256((r/'rtflann_probe_headers.zip').read_bytes()).hexdigest()
 result['header_source']='current repository rtflann headers; separate experiment, no runtime-library mutation' 
 assert (source/'rtflann/algorithms/dist.h').exists(),str(source)
 result['baseline_dist_sha256']=hashlib.sha256((source/'rtflann/algorithms/dist.h').read_bytes()).hexdigest()
 result['probe_sha256']=hashlib.sha256((r/'probe_hamming_neon.cpp').read_bytes()).hexdigest();save()
 common=['/usr/bin/c++','-std=c++17','-UNDEBUG','-I'+str(source),str(r/'probe_hamming_neon.cpp'),'-llz4']
 run('sanitizer_build',common+['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-o',str(out/'sanitizer')],180)
 run('sanitizer',[str(out/'sanitizer')],60)
 run('release_build',common+['-O3','-o',str(out/'benchmark')],180)
 run('fixed_index_benchmark',[str(out/'benchmark'),str(out/'saved')],180)
 result['measurements']=[json.loads(line) for line in (out/'fixed_index_benchmark.log').read_text().splitlines() if line.startswith('{')]
 assert len(result['measurements'])==12 and all(x['indices_and_distances_equal'] for x in result['measurements'])
 result['phase']='finished'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
