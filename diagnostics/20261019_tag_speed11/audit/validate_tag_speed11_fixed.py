from pathlib import Path
import os,subprocess,json,time,hashlib,csv,traceback
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
assert json.loads((r/'tag_speed11_replay/result.json').read_text())['phase']=='completed'
out=r/'tag_speed11_fixed_random';out.mkdir(mode=0o700)
v={'phase':'compile','scope':'offline native engine comparison with diagnostic-only fixed entropy; original captured frames and Tag observations; not end-to-end timing or production randomness','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():(out/'result.json').write_text(json.dumps(v,indent=2))
save()
try:
 lib=out/'fixed_random_diagnostic.so'
 with (out/'compile.log').open('x') as f:subprocess.run(['g++','-std=c++17','-O2','-shared','-fPIC',str(r/'fixed_random_diagnostic.cpp'),'-o',str(lib)],stdout=f,stderr=subprocess.STDOUT,check=True,timeout=60)
 inputs=sorted((r/'tag_speed11_replay/baseline_short').glob('input-*.json'));assert len(inputs)==61
 for name,package in [('baseline','mapping_round6_candidate10_notags'),('candidate','mapping_round6_candidate11_tags')]:
  v['phase']=name;save();p=r/package;c=json.loads((p/'entry.json').read_text())
  env=dict(os.environ,**c['environment'],WBROV_DEPTH_RECOVERY='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LD_PRELOAD=str(lib),WBROV_OFFLINE_RANDOM_DIAGNOSTIC='1')
  session=hashlib.sha256(str(out/name).encode()).hexdigest();identity=json.loads(inputs[0].read_text())['sha256']['metadata.json']
  args=[c['worker'],str(out/name),c['odom_parameters'],c['map_parameters'],session,identity,'1','0'];start=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(args,input='\n'.join(map(str,inputs))+'\nCLOSE\n',env=env,text=True,stdout=f,stderr=subprocess.STDOUT,timeout=600)
  step={'name':name,'exit':q.returncode,'seconds':time.monotonic()-start};v['steps'].append(step);save();assert q.returncode==0
  s=json.loads((out/name/'status.json').read_text());assert s['frames']==1581 and s['database_closed']
  log=(out/(name+'.log')).read_text();line=next(x for x in log.splitlines() if x.startswith('WBROV_RANDOM_DIAGNOSTIC '));assert 'entropy=0 ' not in line
  step.update(frames=s['frames'],main_nodes=s['main_nodes'],recoveries=s['recoveries'],cuts=s['cuts'],cloud_points=s['cloud_points'],random_diagnostic=line,sha256={n:sha(out/name/n) for n in ['frames.csv','main_cloud.ply','tags.json']},timings={k:x for k,x in s.items() if k.endswith('_ms')});save()
 b,c=v['steps'];v.update(phase='completed',same_frames=b['sha256']['frames.csv']==c['sha256']['frames.csv'],same_cloud=b['sha256']['main_cloud.ply']==c['sha256']['main_cloud.ply'],same_tags=b['sha256']['tags.json']==c['sha256']['tags.json'],native_time_saved_fraction=1-c['seconds']/b['seconds'])
except BaseException as ex:v.update(phase='failed',error=repr(ex),traceback=traceback.format_exc())
save()
