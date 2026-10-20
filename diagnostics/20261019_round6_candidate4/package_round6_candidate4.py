from pathlib import Path
import os,json,hashlib,shutil,subprocess,sys
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
old=r/'mapping_combo42_camera6';p=r/'mapping_round6_candidate4_notags';built=r/'round6_candidate4'
build=json.loads((built/'result.json').read_text());assert build['phase']=='passed'
assert json.loads((r/'round6_validation2/result.json').read_text())['phase']=='completed'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
oldconfig=json.loads((old/'entry.json').read_text());assert all(sha(n)==pin for n,pin in oldconfig['sha256'].items())
shutil.copytree(old,p,symlinks=True)
for dest,source,pin in [('bin/zed_rgbd_publisher_rate_ab',built/'camera_build/zed_rgbd_publisher_rate_ab',build['camera_sha256']),('bin/wbrov_replay_worker',built/'wbrov_replay_worker',build['worker_sha256'])]:
 f=p/dest;f.chmod(0o700);shutil.copyfile(source,f);f.chmod(0o500);assert sha(f)==pin
for name in ['deferred_map_display.py','deferred_display_state.py']:
 f=p/'src/visualization'/name;f.chmod(0o600);shutil.copyfile(built/'display'/name,f)
shutil.copyfile(built/'display/test_deferred_display_state.py',p/'tests/test_deferred_display_state.py')
for remote,relative in [('round6_test_deferred_mapping.py','tests/test_deferred_mapping.py'),('round6_test_continuous_mapping_session.py','tests/test_continuous_mapping_session.py'),('round6_continuous_mapping_session.py','tools/continuous_mapping_session.py')]:
 shutil.copyfile(r/remote,p/relative)
c=json.loads(json.dumps(oldconfig).replace(str(old),str(p)));c['sha256']={n:sha(n) for n in c['sha256']}
c['tag_modules']=''
(p/'entry.json').write_text(json.dumps(c,indent=2))
sys.path.insert(0,str(p/'tools'));from render_continuous_entry import render
launcher,wrapper=render(p,sha(p/'entry.json'),sha(p/'tools/continuous_mapping_session.py'),(p/'launch/candidate_receipt_resume/run_water_mapping_with_light.sh').read_text(),sha(p/'bin/wbrov_replay_worker'))
for name,data in [('start_rtabmap_continuous.sh',launcher),('run_continuous_mapping_with_light.sh',wrapper)]:
 f=p/name;f.chmod(0o700);f.write_text(data);f.chmod(0o500);subprocess.run(['/usr/bin/bash','-n',str(f)],check=True)
env=dict(os.environ,**c['environment'],PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
checks=[]
for name in ['test_deferred_mapping','test_continuous_mapping_session','test_deferred_display_state','test_deferred_recorded_pixels']:
 q=subprocess.run(['/usr/bin/python3','-B','-m','unittest','discover','-s','tests','-p',name+'.py'],cwd=p,env=env,capture_output=True,text=True,timeout=60)
 checks.append({'name':name,'exit':q.returncode,'output':q.stdout+q.stderr});assert q.returncode==0,name
q=subprocess.run([str(p/'start_rtabmap_continuous.sh'),'--check-only'],env=env,capture_output=True,text=True,timeout=60);assert q.returncode==0,q.stdout+q.stderr
changed=[]
for f in old.rglob('*'):
 if f.is_file() and not f.is_symlink() and sha(f)!=sha(p/f.relative_to(old)):changed.append(str(f.relative_to(old)))
receipt={'package':str(p),'before':str(old),'changed':changed,'camera_sha256':sha(p/'bin/zed_rgbd_publisher_rate_ab'),'worker_sha256':sha(p/'bin/wbrov_replay_worker'),'entry_sha256':sha(p/'entry.json'),'wrapper_sha256':sha(p/'run_continuous_mapping_with_light.sh'),'check_only':q.stdout,'checks':checks,'tag_detection_enabled':False,'production_changed':False,'field_acceptance':False}
(r/'package_round6_candidate4_result.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
