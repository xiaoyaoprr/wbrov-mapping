from pathlib import Path
import os,subprocess,json,zipfile,time,hashlib,shutil,shlex,traceback
b=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015');r=b/'round7_deferred_20261018'
out=r/'round6_candidate5';out.mkdir(mode=0o700)
report={'phase':'prepare','production_changed':False,'hardware_opened':False,'steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(report,indent=2));p.replace(out/'result.json')
save()
try:
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip(),'control active'
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
  except OSError:continue
  assert not any(s in cmd for s in ['bin/zed_rgbd_publisher_rate_ab','bin/wbrov_replay_worker','tools/continuous_mapping_session.py']),cmd
 with zipfile.ZipFile(r/'round6_candidate5_source.zip') as z:
  for name in z.namelist():
   path=Path(name);assert '..' not in path.parts and path.parts[0] in ['camera','worker','display','include']
   (out/path).parent.mkdir(parents=True,exist_ok=True);(out/path).write_bytes(z.read(name))
 report['archive_sha256']=hashlib.sha256((r/'round6_candidate5_source.zip').read_bytes()).hexdigest()
 report['sources']={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file() and p.name!='result.json'}
 env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
  PYTHONPATH='/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/home/exocean/opt/rtabmap/opt/ros/humble/local/lib/python3.10/dist-packages',
  AMENT_PREFIX_PATH='/home/exocean/opt/rtabmap/opt/ros/humble:/opt/ros/humble',
  CMAKE_PREFIX_PATH='/home/exocean/.cache/wbrov_mapping_builds/discovery_20260905/vendor_shadow:/home/exocean/opt/rtabmap/opt/ros/humble:/opt/ros/humble',
  LD_LIBRARY_PATH='/usr/local/zed/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu')
 def run(name,cmd,timeout=600,cwd=None,environment=None):
  report['phase']=name;save();start=time.monotonic()
  with (out/(name+'.log')).open('x') as log:q=subprocess.run(cmd,cwd=cwd,env=environment or env,stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
  report['steps'].append({'name':name,'exit':q.returncode,'seconds':time.monotonic()-start});save();assert q.returncode==0,name
 run('camera_configure',['cmake','-S',str(out/'camera'),'-B',str(out/'camera_build'),'-DCMAKE_BUILD_TYPE=Release','-DPython3_EXECUTABLE=/usr/bin/python3','-DOpenCV_DIR=/usr/lib/cmake/opencv4'])
 run('camera_compile',['cmake','--build',str(out/'camera_build'),'-j2'])
 run('camera_tests',['ctest','--test-dir',str(out/'camera_build'),'--output-on-failure'])
 run('camera_ldd',['ldd','-r',str(out/'camera_build/zed_rgbd_publisher_rate_ab')])
 assert not any(s in (out/'camera_ldd.log').read_text() for s in ['not found','undefined symbol'])
 symbols=subprocess.check_output(['readelf','--dyn-syms','--wide',str(out/'camera_build/zed_rgbd_publisher_rate_ab')],text=True)
 assert not any(' ZSTD_' in s for s in symbols.splitlines())
 config=json.loads((r/'mapping_combo42_camera6/entry.json').read_text());runtime=dict(env,**config['environment'])
 run('sdk_runtime',['./rgbd_codec_sdk_test'],cwd=out/'camera_build',environment=runtime)
 for p in (r/'worker_source_v39').iterdir():
  if p.is_file() and not (out/'worker'/p.name).exists():shutil.copyfile(p,out/'worker'/p.name)
 build=b/'round4_20261016/build';core=r/'native_release_v6/librtabmap_core.so'
 assert core.exists()
 flags=(build/'CMakeFiles/test_visual_recovery_native.dir/flags.make').read_text();args=[]
 for name in ('CXX_DEFINES','CXX_INCLUDES'):
  args+=shlex.split(next(x.split('=',1)[1].strip() for x in flags.splitlines() if x.startswith(name+' =')))
 args+=['-I'+str(b/'round4_20261016/source/include')]
 base=['/usr/bin/c++','-I'+str(out/'include'),'-std=c++17','-O2','-Wall','-Wextra','-I'+str(r/'native_release_v6/core/headers'),*args,'-I'+str(out/'worker')]
 run('gate_compile',['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-UNDEBUG','-pthread','-I'+str(out/'include'),str(out/'worker/test_mapping_continuity_gate.cpp'),'-o',str(out/'gate_test')])
 run('gate_test',[str(out/'gate_test')])
 run('quality_compile',[*base,'-UNDEBUG',str(out/'worker/test_mapping_quality_trace.cpp'),'-o',str(out/'quality_test')])
 run('quality_test',[str(out/'quality_test')])
 json.loads((out/'quality_test.log').read_text())
 run('worker_compile',[*base,'-isystem','/usr/local/zed/include','-isystem','/usr/local/cuda/include','-c',str(out/'worker/replay_worker.cpp'),'-o',str(out/'worker.o')])
 link=shlex.split((build/'CMakeFiles/test_visual_recovery_native.dir/link.txt').read_text())
 link=[str(out/'worker.o') if x.endswith('.o') else str(out/'wbrov_replay_worker') if x=='test_visual_recovery_native' else str(core) if 'librtabmap_core.so' in x else x for x in link]
 legacy=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-439961e59d783abc416c2507a947dfe222965bd6a7ad9da4eed6e203d4d3b31b')
 link+=['-Wl,-rpath-link,'+str(legacy/'lib'),'-L/usr/local/zed/lib','-Wl,-rpath,/usr/local/zed/lib','-lsl_zed','-lcrypto','-lzstd']
 run('worker_link',link,cwd=build)
 run('worker_ldd',['ldd','-r',str(out/'wbrov_replay_worker')],environment=runtime)
 assert not any(s in (out/'worker_ldd.log').read_text() for s in ['not found','undefined symbol'])
 report['worker_sources']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (out/'worker').iterdir() if p.is_file()}
 report.update(phase='passed',camera_sha256=hashlib.sha256((out/'camera_build/zed_rgbd_publisher_rate_ab').read_bytes()).hexdigest(),worker_sha256=hashlib.sha256((out/'wbrov_replay_worker').read_bytes()).hexdigest(),core_sha256=hashlib.sha256(core.read_bytes()).hexdigest())
except BaseException as e:report.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
