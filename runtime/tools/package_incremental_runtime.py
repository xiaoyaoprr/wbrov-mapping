"""New immutable runtime from a verified baseline and four rebuilt components.
Reuses checked Core/GUI/export/messages and all unchanged objects byte-for-byte.
It never replaces the baseline tree or any desktop entrypoint.
"""
from pathlib import Path
import hashlib,json,os,sys,subprocess,tempfile,re
root=Path(__file__).resolve().parent
assert root==Path('/home/exocean/.cache/wbrov_mapping_builds/discovery_20260905')
baseline=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-49306a31efd52dd483250cf6ec19d2224dc057d3cb7fa80c5078e6489b42317e')
base_sha='efb168b0cf45c3ef8282b68850ba2a7a077327eb4c75549a342c27e26904bb10'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(baseline/'runtime-manifest.json')==base_sha
assert (root/'build.rc').read_text().strip()=='0'
subprocess.run([sys.executable,str(baseline/'tools/check_private_runtime.py'),'tree','--runtime-root',str(baseline)],check=True)
base=json.loads((baseline/'runtime-manifest.json').read_text())
source_manifest=json.loads((root/'persistent_source_manifest.json').read_text())
for name,digest in source_manifest.items():assert sha(root/name)==digest,name
replacements={
 'lib/librtabmap_odom.so':root/'build/librtabmap_odom.so',
 'lib/librtabmap_slam_plugins.so':root/'build/librtabmap_slam_plugins.so',
 'bin/header_health_observer':root/'build/native/header_health_observer_native/header_health_observer',
 'bin/safe_reloc_gid_frontend':root/'build/native/safe_reloc_gid_frontend_native/safe_reloc_gid_frontend'}
for p in replacements.values():
 assert p.is_file() and not p.is_symlink()
 assert 'AArch64' in subprocess.check_output(['readelf','-h',str(p)],text=True)
 tags=subprocess.check_output(['readelf','-d',str(p)],text=True)
 assert '(RPATH)' not in tags and '(RUNPATH)' not in tags
replacements['libexec/safe_reloc_ipc_engine.py']=root/'native/safe_reloc_gid_frontend_native/safe_reloc_ipc_engine.py'
checker=root/'check_private_runtime.py'
text=checker.read_text()
text=text.replace('/tmp/wbrov_armtx_b916_20260904_5e72df64',str(root))
text,count=re.subn(r'("rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp":\s*")[0-9a-f]{64}(\")',lambda m:m[1]+sha(root/'source/rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp')+m[2],text)
assert count==1
for name in ['header_health_observer','safe_reloc_gid_frontend']:
 pattern=r'("'+name+r'":\s*")[0-9a-f]{64}(\")'
 text,count=re.subn(pattern,lambda m:m[1]+sha(replacements['bin/'+name])+m[2],text)
 assert count==1
text,count=re.subn(r'("safe_reloc_ipc_engine":\s*")[0-9a-f]{64}(\")',lambda m:m[1]+sha(replacements['libexec/safe_reloc_ipc_engine.py'])+m[2],text)
assert count==1
checker.write_text(text)
sys.path.insert(0,str(root))
import package_private_runtime as pack
import check_private_runtime as check
for rel,digest in check.SOURCE_HASHES.items():
 actual=root/'source'/rel
 if actual.exists():assert sha(actual)==digest,rel
 else:assert base['source_hashes'][rel]==digest,rel
parent=root/'packages';parent.mkdir(exist_ok=True,mode=0o700)
staging=parent/('stage-'+next(tempfile._get_candidate_names()))
builder=pack.RuntimeBuilder(root,root,root,root,staging)
replacements.update({'tools/check_private_runtime.py':checker,'tools/package_private_runtime.py':root/'package_private_runtime.py'})
base_entries={entry['path']:entry for entry in base['files']}
for entry in base['files']:
 rel=entry['path']
 if entry['type']!='regular':continue
 source=replacements.get(rel,baseline/rel)
 builder.copy_regular(source,rel,executable=bool((baseline/rel).stat().st_mode&0o111),provenance_kind='rebuilt-repair-component' if rel in replacements else 'verified-baseline-reuse')
 if rel not in replacements:
  assert sha(staging/rel)==entry['sha256'],rel
  builder.provenance[rel]['baseline_manifest_sha256']=base_sha
  origin=base['provenance'].get(rel,{}).get('source_path')
  if check.required_dynamic_type_support(rel) and isinstance(origin,str):
   builder.provenance[rel]['dynamic_dependency_origin']=origin
for entry in base['files']:
 if entry['type']=='symlink':builder.add_symlink(entry['path'],entry['link_target'],provenance_kind='verified-baseline-relative-alias')
builder.copy_regular(Path(__file__).resolve(),'tools/package_incremental_runtime.py',executable=True,provenance_kind='reviewed-incremental-packager')
pack.collect_dependency_closure(builder)
content,_=pack.write_manifest(builder,base['required_components'])
final=parent/('sha256-'+content);os.rename(staging,final)
subprocess.run([sys.executable,str(final/'tools/check_private_runtime.py'),'tree','--runtime-root',str(final)],check=True)
metadata={'runtime_root':str(final),'content_sha256':content,'manifest_sha256':sha(final/'runtime-manifest.json'),'checker_sha256':sha(final/'tools/check_private_runtime.py'),'entrypoints':{name:sha(final/rel) for name,rel in check.ENTRYPOINTS.items()},'components':{rel:sha(final/rel) for rel in replacements},'source_hashes':check.SOURCE_HASHES,'baseline_reuse_sha256':base_sha}
(root/'runtime_result.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata,indent=2))
