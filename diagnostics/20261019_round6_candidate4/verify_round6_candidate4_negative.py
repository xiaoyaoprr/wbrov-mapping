from pathlib import Path
import subprocess, json, resource, hashlib, time
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
assert json.loads((r/'round6_candidate4/result.json').read_text())['phase']=='passed'
out=r/'round6_candidate4_negative';out.mkdir(mode=0o700)
source=out/'async_spool_commit_test.cpp'
source.write_bytes((r/'round6_candidate4/camera/async_spool_commit_test.cpp').read_bytes())
old=r/'round6_candidate3/camera'
assert 'MAX_PENDING_SEGMENTS=32' in (old/'rgbd_spool.hpp').read_text()
assert 'MAX_PENDING_BYTES=512ULL*1024*1024' in (old/'rgbd_spool.hpp').read_text()
cmd=['/usr/bin/c++','-std=c++17','-O2','-UNDEBUG','-pthread','-I'+str(old),str(source)]
dep=subprocess.check_output(cmd+['-MM'],text=True);(out/'dependencies.txt').write_text(dep)
assert str(old/'rgbd_spool.hpp') in dep and str(r/'round6_candidate4/camera/rgbd_spool.hpp') not in dep
with (out/'compile.log').open('x') as f:
    subprocess.run(cmd+['-lzstd','-o',str(out/'test_old_limit')],stdout=f,stderr=subprocess.STDOUT,check=True,timeout=120)
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
start=time.monotonic()
with (out/'expected_failure.log').open('x') as f:
    q=subprocess.run([str(out/'test_old_limit')],stdout=f,stderr=subprocess.STDOUT,timeout=20)
report={'expected_exit':-6,'actual_exit':q.returncode,'seconds':time.monotonic()-start,'old_header_sha256':hashlib.sha256((old/'rgbd_spool.hpp').read_bytes()).hexdigest(),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'hardware_opened':False,'production_changed':False}
report['passed']=q.returncode==-6 and 'burstAccepted' in (out/'expected_failure.log').read_text()
(out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));assert report['passed']
