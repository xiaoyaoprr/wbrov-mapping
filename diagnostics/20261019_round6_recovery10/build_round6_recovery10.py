from pathlib import Path
import json,os,subprocess,shutil,time,hashlib,traceback
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');old=r/'round6_recovery9_build';out=r/'round6_recovery10_build'
out.mkdir(mode=0o700);v={'phase':'prepare','scope':'worker only; skip duplicate old-keyframe search, unchanged Core9 and all geometry gates','steps':[]}
def save():
 t=out/'result.tmp';t.write_text(json.dumps(v,indent=2));t.replace(out/'result.json')
save()
try:
 assert json.loads((r/'round6_recovery9_replay2/result.json').read_text())['phase']=='completed'
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
 shutil.copytree(old/'worker',out/'worker');shutil.copyfile(r/'round6_recovery10_DeferredMappingSession.h',out/'worker/DeferredMappingSession.h')
 cfg=json.loads((r/'mapping_round6_candidate8_notags/entry.json').read_text());env=dict(os.environ,**cfg['environment'],PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 env['LD_LIBRARY_PATH']=str(old)+':'+env['LD_LIBRARY_PATH']
 previous=json.loads((old/'resume2_result.json').read_text());assert previous['phase']=='passed'
 sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
 v['worker_sources']={p.name:sha(p) for p in (out/'worker').iterdir() if p.is_file()};v['core_sha256']=sha(old/'librtabmap_core.so.0.23.7');save()
 for name in ['worker_compile','worker_link','worker_ldd']:
  cmd=next(s['command'] for s in previous['steps'] if s['name']==name)
  cmd=[x.replace(str(old/'worker'),str(out/'worker')).replace(str(old/'wbrov_replay_worker'),str(out/'wbrov_replay_worker')) for x in cmd]
  v['phase']=name;save();start=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=300)
  v['steps'].append({'name':name,'command':cmd,'seconds':time.monotonic()-start,'exit':q.returncode});save();assert q.returncode==0
 ldd=(out/'worker_ldd.log').read_text();assert str(old/'librtabmap_core.so') in ldd and not any(x in ldd for x in ['not found','undefined symbol'])
 v.update(phase='passed',worker_sha256=sha(out/'wbrov_replay_worker'));save()
 with (r/'round6_recovery10_replay_driver.log').open('x') as f:
  p=subprocess.Popen(['/usr/bin/python3','-I','-B',str(r/'validate_round6_recovery10.py')],env=env,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
 v['replay_pid']=p.pid
except BaseException as ex:v.update(phase='failed',error=repr(ex),traceback=traceback.format_exc())
save()
