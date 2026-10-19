from pathlib import Path
import subprocess,time,json,hashlib,zipfile,os,statistics
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'reader_hash_probe';out.mkdir(mode=0o700)
result={'phase':'waiting_for_hamming5','scope':'offline component experiments only; native full RGB-D read equality and unchanged per-file SHA-256 checks; no Core, runtime, hardware or production mutation','steps':[]}
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
  phase=json.loads((r/'hamming_neon_probe5/result.json').read_text())['phase']
  if phase in ('finished','failed'):break
  assert time.monotonic()-began<7200;time.sleep(5)
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 source=out/'source';source.mkdir()
 with zipfile.ZipFile(r/'reader_hash_probe_source.zip') as z:
  pins=json.loads(z.read('pins.json'))
  for name,pin in pins.items():
   assert not Path(name).is_absolute() and '..' not in Path(name).parts
   data=z.read(name);assert hashlib.sha256(data).hexdigest()==pin
   p=source/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
 result['source_sha256']=pins;save()
 common=['/usr/bin/c++','-std=c++17','-UNDEBUG','-pthread','-Wall','-Wextra','-I'+str(source)]
 run('reader_sanitizer_build',[*common,'-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer',str(source/'probe_record_decode.cpp'),'-o',str(out/'reader_sanitizer'),'-lzstd'],120)
 run('reader_sanitizer',[str(out/'reader_sanitizer')],120)
 run('reader_release_build',[*common,'-O2',str(source/'probe_record_decode.cpp'),'-o',str(out/'reader'),'-lzstd'],120)
 run('hash_release_build',[*common,'-O2',str(source/'probe_input_hash.cpp'),'-o',str(out/'hash'),'-lcrypto'],120)
 root=r/'compressed_v35/motion_spool_v2';raw=r/'motion_spool1'
 # Hash all selected immutable inputs before and after. No recording is opened
 # by a camera and no SQLite database is touched.
 inputs=sorted(root.glob('*.frames'))+[sorted(raw.glob('*.frames'))[0]]
 inputs+=sorted(p for p in raw.iterdir() if p.is_file() and p.suffix!='.frames' and p.name not in ('status.json',))[:8]
 def sha(p):
  h=hashlib.sha256()
  with p.open('rb') as f:
   for data in iter(lambda:f.read(1024*1024),b''):h.update(data)
  return h.hexdigest()
 original={str(p):sha(p) for p in inputs}
 (out/'inputs.json').write_text(json.dumps(original,indent=2))
 selected=[p for p in inputs if p.parent==raw]
 (out/'hash_inputs.txt').write_text(''.join(original[str(p)]+' '+str(p)+'\n' for p in selected))
 run('hash_benchmark',[str(out/'hash'),str(out/'hash_inputs.txt')],180)
 run('reader_benchmark',[str(out/'reader'),str(root)],240)
 assert all(sha(Path(p))==pin for p,pin in original.items())
 for name in ('hash','reader'):
  rows=[json.loads(line) for line in (out/(name+'_benchmark.log')).read_text().splitlines() if line.startswith('{')]
  assert len(rows)==18 and set(x['mode'] for x in rows)=={0,1,2}
  result[name]={'rounds':rows,'median_ms_excluding_warmup':{str(mode):statistics.median(x['ms'] for x in rows if x['mode']==mode and x['round']>0) for mode in range(3)}}
 result['input_hashes_unchanged']=True;result['phase']='finished'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
