from pathlib import Path
import sys,json,subprocess,shutil,hashlib,zipfile,time,importlib.util,os
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');out=r/'status40_snapshot_probe';out.mkdir(mode=0o700)
result={'phase':'prepare','scope':'status-only durability optimization; immutable-input checks, receipts, session/save and terminal status unchanged; no hardware','steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
def test(name):
 result['phase']=name;save();started=time.monotonic()
 with (out/(name+'.log')).open('x') as log:p=subprocess.run(['/usr/bin/python3','-I','-B','-m','unittest','discover','-s',str(out/'project/tests'),'-p',name+'.py'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,timeout=90)
 result['steps'].append({'name':name,'exit':p.returncode,'seconds':time.monotonic()-started});save();assert p.returncode==0,name
def module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
save()
try:
 assert json.loads((r/'core6_replay_pipeline/result.json').read_text())['phase'] in ('passed','completed_with_recovery_failures','failed')
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 frozen=r/'offline_v37_tools/tools'
 with zipfile.ZipFile(r/'status40_source.zip') as z:
  delta=json.loads(z.read('delta.json'))
  for name,pin in delta['baseline_tool_pins'].items():assert hashlib.sha256((frozen/name).read_bytes()).hexdigest()==pin
  project=out/'project';project.mkdir();shutil.copytree(frozen,project/'tools')
  for name,pin in delta['members_sha256'].items():
   assert not Path(name).is_absolute() and '..' not in Path(name).parts
   data=z.read(name);assert hashlib.sha256(data).hexdigest()==pin
   target=project/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
  (out/'delta.json').write_text(json.dumps(delta,indent=2))
 result['source_archive_sha256']=hashlib.sha256((r/'status40_source.zip').read_bytes()).hexdigest();save()
 for name in ('test_deferred_mapping','test_continuous_mapping_session','test_deferred_display_state'):test(name)
 sys.path.insert(0,str(frozen));baseline=module('baseline_queue',frozen/'deferred_mapping.py');candidate=module('candidate_queue',project/'tools/deferred_mapping.py')
 payload=json.loads((r/'supervisor_profile39/full_compressed_profile/status.json').read_text());payload['state']='processing';payload['mapping']['database_closed']=False
 targets={}
 for kind in ('baseline','candidate'):
  directory=out/kind;directory.mkdir();targets[kind]=directory/'status.json'
 result['phase']='snapshot_benchmark';result['measurements']=[];save()
 for turn in range(6):
  values={}
  for kind in (('baseline','candidate') if turn%2==0 else ('candidate','baseline')):
   started=time.perf_counter_ns()
   for n in range(120):
    value=dict(payload,completed_segments=n)
    if kind=='baseline':baseline.atomic_json(targets[kind],value)
    else:candidate.atomic_status_snapshot(targets[kind],value)
   values[kind]=(time.perf_counter_ns()-started)/1e6
   assert json.loads(targets[kind].read_text())==value and not targets[kind].with_suffix('.tmp').exists()
  assert targets['baseline'].read_bytes()==targets['candidate'].read_bytes()
  result['measurements'].append({'round':turn,'snapshots':120,'before_ms':values['baseline'],'after_ms':values['candidate'],'outputs_equal':True});save()
 # Complete-state publication still executes the durable default path.
 final=dict(payload,state='closed');baseline.atomic_json(targets['baseline'],final);candidate.atomic_status_snapshot(targets['candidate'],final)
 assert targets['baseline'].read_bytes()==targets['candidate'].read_bytes()
 for name,pin in delta['baseline_tool_pins'].items():assert hashlib.sha256((frozen/name).read_bytes()).hexdigest()==pin
 result['phase']='finished'
except BaseException as e:result['phase']='failed';result['error']=repr(e)
save()
