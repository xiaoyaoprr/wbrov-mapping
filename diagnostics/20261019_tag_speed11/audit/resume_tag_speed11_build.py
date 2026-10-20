from pathlib import Path
import os,json,hashlib,shutil,subprocess,time,traceback,resource
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');old=r/'round6_recovery9_build';out=r/'tag_speed11_build'
resource.setrlimit(resource.RLIMIT_CORE,(0,0));sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
previous=json.loads((out/'result.json').read_text());assert previous['phase']=='failed' and previous['error']=="AssertionError('probe_asan_ubsan')"
v={'phase':'prepare','scope':'retain ASAN failure; vendor-only ASAN control plus UBSAN, then candidate native build','steps':[]}
def save():
 p=out/'resume_result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'resume_result.json')
save()
try:
 cfg=json.loads((r/'mapping_round6_candidate10_notags/entry.json').read_text());env=dict(os.environ,**cfg['environment'],PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 env['LD_LIBRARY_PATH']=str(out)+':'+str(old)+':'+env['LD_LIBRARY_PATH'];env['ASAN_OPTIONS']='detect_leaks=0';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
 def run(name,cmd,timeout=300,expected=0):
  v['phase']=name;save();start=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(cmd,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
  v['steps'].append({'name':name,'command':cmd,'exit':q.returncode,'seconds':time.monotonic()-start});save()
  assert (q.returncode!=0 if expected=='nonzero' else q.returncode==expected),name
 template=next(s['command'] for s in previous['steps'] if s['name']=='probe_asan_ubsan_compile')
 source=out/'vendor_control.cpp';source.write_text('#include <rtabmap/core/CameraModel.h>\n#include <iostream>\nint main(){rtabmap::CameraModel m(420.,421.,320.,180.,rtabmap::Transform::getIdentity(),0.,cv::Size(640,360));std::cout<<"vendor control complete, no candidate cache executed"<<std::endl;return m.isValidForProjection()?0:1;}\n')
 control=[str(source) if x.endswith('/native_probe.cpp') else str(out/'vendor_control') if x==str(out/'probe_asan_ubsan') else x for x in template]
 run('vendor_control_compile',control);run('vendor_control',[str(out/'vendor_control')],expected='nonzero')
 control_log=(out/'vendor_control.log').read_text();assert 'vendor control complete' in control_log and 'libnvfnet.so' in control_log and 'alloc-dealloc-mismatch' in control_log
 v['asan_limitation']='Same alloc/dealloc mismatch reproduced at NVIDIA libnvfnet shutdown without candidate cache. ASAN is not passed; no mismatch suppression applied.';save()
 ubsan=[x.replace('-fsanitize=address,undefined','-fsanitize=undefined').replace(str(out/'probe_asan_ubsan'),str(out/'probe_ubsan')) for x in template]
 run('probe_ubsan_compile',ubsan);run('probe_ubsan',[str(out/'probe_ubsan')])
 shutil.copytree(old/'core',out/'core');shutil.copyfile(out/'proposal/OdometryF2M.candidate.cpp',out/'core/odometry/OdometryF2M.cpp')
 cmd=next(s['command'] for s in json.loads((r/'native_release_v1/result.json').read_text())['steps'] if s['name']=='OdometryF2M_compile')
 cmd=[str(out/'core/odometry/OdometryF2M.cpp') if x.endswith('/core/odometry/OdometryF2M.cpp') else x.replace(str(r/'native_release_v1'),str(out)) for x in cmd]
 run('OdometryF2M_compile',cmd)
 link=next(s['command'] for s in json.loads((old/'result.json').read_text())['steps'] if s['name']=='link')
 link=[str(out/'OdometryF2M.o') if x==str(r/'native_release_v1/OdometryF2M.o') else x for x in link];link[link.index('-o')+1]=str(out/'librtabmap_core.so.0.23.7');run('link',link)
 for name in ['librtabmap_core.so','librtabmap_core.so.0.23']:(out/name).symlink_to('librtabmap_core.so.0.23.7')
 run('ldd',['ldd','-r',str(out/'librtabmap_core.so')]);assert not any(x in (out/'ldd.log').read_text() for x in ['not found','undefined symbol'])
 trace=out/'trace';trace.mkdir();env['WBROV_PRIVATE_OPTIMIZER_TRACE']=str(trace)
 for label,extra in [('visual',[]),('prepared',['--prepare-close-only']),('lifecycle',['--candidate-lifecycle']),('tag',['--tag-depth-proposal'])]:run('test_'+label,[str(old/'test_visual'),str(out/('fixture_'+label)),*extra])
 v.update(phase='passed',core_sha256=sha(out/'librtabmap_core.so.0.23.7'),f2m_sha256=sha(out/'core/odometry/OdometryF2M.cpp'))
except BaseException as ex:v.update(phase='failed',error=repr(ex),traceback=traceback.format_exc())
save()
