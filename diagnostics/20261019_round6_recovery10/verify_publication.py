from pathlib import Path
import json,hashlib,zipfile,subprocess
repo=Path('C:/Users/41479/Documents/Codex/2026-10-15/new-chat/outputs/wbrov-mapping');sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
tracked=set(subprocess.check_output(['git','ls-files','-z'],cwd=repo).decode().split('\0'))
lines=(repo/'MANIFEST.sha256').read_text().splitlines()
for line in lines:
 pin,name=line.split('  ',1);assert sha(repo/name)==pin,name
counts=0
for name in ['20261019_round6_candidate6','20261019_round6_candidate7','20261019_round6_candidate8','20261019_round6_recovery9','20261019_round6_recovery10']:
 for p in (repo/'diagnostics'/name).rglob('sha256.json'):
  for rel,pin in json.loads(p.read_text()).items():
   target=p.parent/rel;assert sha(target)==pin,str(target);counts+=1
   if name in ['20261019_round6_recovery9','20261019_round6_recovery10']:assert target.relative_to(repo).as_posix() in tracked,str(target)+' missing from Git'
e9=repo/'diagnostics/20261019_round6_recovery9';e10=repo/'diagnostics/20261019_round6_recovery10'
with zipfile.ZipFile(e9/'source.zip') as z:
 pairs={'core/visual_recovery.inc':'src/rtabmap_core/corelib/src/visual_recovery.inc','core/recovery_geometry.inc':'src/rtabmap_core/corelib/src/recovery_geometry.inc',
 'core/headers/rtabmap/core/Rtabmap.h':'src/rtabmap_core/corelib/include/rtabmap/core/Rtabmap.h','test_visual_recovery_native.cpp':'tests/native/test_visual_recovery_native.cpp'}
 for name in z.namelist():
  if name.startswith('worker/') and name!='worker/DeferredMappingSession.h':pairs[name]='src/deferred_mapping/'+Path(name).name
 for name,path in pairs.items():assert z.read(name)==(repo/path).read_bytes(),path
assert (e10/'DeferredMappingSession.h').read_bytes()==(repo/'src/deferred_mapping/DeferredMappingSession.h').read_bytes()
b=json.loads((e10/'build/result.json').read_text());assert b['phase']=='passed'
assert b['worker_sources']['DeferredMappingSession.h']==sha(repo/'src/deferred_mapping/DeferredMappingSession.h')
for folder in [e9,e10]:
 result=json.loads((folder/'replay/result.json').read_text());assert result['phase']=='completed'
 for step in result['steps']:assert step['frames']==1581 and step['save_verified'] and step['complete'] and step['final']['database_closed']
q=subprocess.run(['git','-c','core.whitespace=cr-at-eol,-blank-at-eof','diff','--check','--','src','tools','tests','docs','README.md'],cwd=repo,capture_output=True,text=True);assert q.returncode==0,q.stdout+q.stderr
assert subprocess.check_output(['git','branch','--show-current'],cwd=repo,text=True).strip()=='codex/round6-capture-recovery'
print(json.dumps({'manifest_files':len(lines),'evidence_hashes':counts,'core9_sources_match':True,'worker10_sources_match':True,'short_recovery_passed':False,'production_deployment':False}))
