from pathlib import Path
import os,json,hashlib,shutil,zipfile,subprocess,sys
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');old=r/'mapping_round6_candidate10_notags';p=r/'mapping_round6_candidate11_tags'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
build=json.loads((r/'tag_speed11_build/resume_result.json').read_text());assert build['phase']=='passed'
c0=json.loads((old/'entry.json').read_text());assert all(sha(n)==pin for n,pin in c0['sha256'].items())
shutil.copytree(old,p,symlinks=True)
core=p/'lib/librtabmap_core.so.0.23.7';assert not core.is_symlink();core.chmod(0o700);shutil.copyfile(r/'tag_speed11_build/librtabmap_core.so.0.23.7',core);core.chmod(0o500);assert sha(core)==build['core_sha256']
with zipfile.ZipFile(r/'tag_speed11_sources.zip') as z:
 for name,pin in json.loads(z.read('sha256.json')).items():
  data=z.read(name);assert hashlib.sha256(data).hexdigest()==pin
  f=(p/name).resolve();assert f.is_relative_to(p.resolve()) and not f.is_symlink();f.chmod(0o600);f.write_bytes(data)
c=json.loads(json.dumps(c0).replace(str(old),str(p)));c['tag_modules']=str(p/'runtime/libexec/wbrov_corner_recovery')
c['sha256']={n:sha(n) for n in c['sha256']}
assert all(str(f) in c['sha256'] for f in Path(c['tag_modules']).glob('*.py'))
(p/'entry.json').write_text(json.dumps(c,indent=2))
sys.path.insert(0,str(p/'tools'));from render_continuous_entry import render
launcher,wrapper=render(p,sha(p/'entry.json'),sha(p/'tools/continuous_mapping_session.py'),(p/'launch/candidate_receipt_resume/run_water_mapping_with_light.sh').read_text(),sha(c['worker']))
for name,data in [('start_rtabmap_continuous.sh',launcher),('run_continuous_mapping_with_light.sh',wrapper)]:
 f=p/name;f.chmod(0o700);f.write_text(data);f.chmod(0o500);subprocess.run(['/usr/bin/bash','-n',str(f)],check=True)
env=dict(os.environ,**c['environment'],PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
checks=[]
for name in ['test_deferred_mapping','test_continuous_mapping_session','test_deferred_display_state','test_deferred_recorded_pixels']:
 q=subprocess.run(['/usr/bin/python3','-B','-m','unittest','discover','-s','tests','-p',name+'.py'],cwd=p,env=env,capture_output=True,text=True,timeout=60);checks.append({'name':name,'exit':q.returncode,'output':q.stdout+q.stderr});assert q.returncode==0,checks[-1]
q=subprocess.run([str(p/'start_rtabmap_continuous.sh'),'--check-only'],env=env,capture_output=True,text=True,timeout=60);assert q.returncode==0,q.stdout+q.stderr
ldd=subprocess.check_output(['ldd','-r',c['worker']],env=env,text=True);assert str(p/'lib/librtabmap_core.so') in ldd and not any(x in ldd for x in ['not found','undefined symbol'])
receipt={'package':str(p),'core_sha256':sha(core),'worker_sha256':sha(c['worker']),'camera_sha256':sha(c['camera']),'entry_sha256':sha(p/'entry.json'),'wrapper_sha256':sha(p/'run_continuous_mapping_with_light.sh'),'checks':checks,'check_only':q.stdout,'tag_detection_enabled':True,'production_changed':False,'field_acceptance':False,'ldd':ldd}
(r/'package_tag_speed11_result.json').write_text(json.dumps(receipt,indent=2));print(json.dumps({k:v for k,v in receipt.items() if k!='ldd'}))
