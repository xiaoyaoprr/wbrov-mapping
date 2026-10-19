from pathlib import Path
import os,json,hashlib,shutil,zipfile,subprocess,time,traceback,resource
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');old=r/'round6_recovery9_build';out=r/'tag_speed11_build';out.mkdir(mode=0o700)
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
v={'phase':'prepare','scope':'frame-local exact inverse reuse; original quality, all frames, Tag recovery unchanged; offline only','steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
save()
try:
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
 cfg=json.loads((r/'mapping_round6_candidate10_notags/entry.json').read_text())
 env=dict(os.environ,**cfg['environment'],PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 env['LD_LIBRARY_PATH']=str(out)+':'+str(old)+':'+env['LD_LIBRARY_PATH'];env['ASAN_OPTIONS']='detect_leaks=0'
 def run(name,cmd,timeout=300,nonzero=False):
  v['phase']=name;save();start=time.monotonic()
  with (out/(name+'.log')).open('x') as f:q=subprocess.run(cmd,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
  v['steps'].append({'name':name,'command':cmd,'exit':q.returncode,'seconds':time.monotonic()-start});save()
  assert (q.returncode!=0 if nonzero else q.returncode==0),name
 archive=r/'tag_speed11_inverse_source.zip';assert sha(archive)=='57213561f6de5aa20f2c6ca62a69a8fa5201ceb7ea3f18b69cc70b2cce0c7817'
 with zipfile.ZipFile(archive) as z:
  for name in z.namelist():
   f=(out/'proposal'/name).resolve();assert f.is_relative_to((out/'proposal').resolve());f.parent.mkdir(parents=True,exist_ok=True);f.write_bytes(z.read(name))
 proposal=out/'proposal';manifest=json.loads((proposal/'manifest.json').read_text())
 for name,pin in manifest['members'].items():assert sha(proposal/name)==pin['sha256']
 assert sha(old/'core/odometry/OdometryF2M.cpp')==manifest['members']['OdometryF2M.baseline.cpp']['sha256']
 original=json.loads((r/'native_release_v1/result.json').read_text())
 cmd=next(s['command'] for s in original['steps'] if s['name']=='OdometryF2M_compile')
 # Reuse actual production compile includes and optimization flags; asserts stay on for probe.
 flags=cmd[1:cmd.index('-c')];idx=flags.index('-MMD');flags.pop(idx);idx=flags.index('-MF');del flags[idx:idx+2]
 flags=[x for x in flags if x!='-DNDEBUG']
 legacy=next(Path(x) for x in cfg['environment']['LD_LIBRARY_PATH'].split(':') if 'sha256-439961' in x)
 def probe(kind,sanitizer):
  extras=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if sanitizer else []
  exe=out/('probe_'+kind)
  run('probe_'+kind+'_compile',['/usr/bin/c++',*flags,'-O3','-UNDEBUG',*extras,str(proposal/'native_probe.cpp'),'-o',str(exe),'-L'+str(old),'-lrtabmap_core','-L'+str(legacy),'-lrtabmap_utilite','-lopencv_core','-lopencv_calib3d','-Wl,-rpath-link,'+str(legacy),'-Wl,-rpath-link,/usr/local/zed/lib'])
  run('probe_'+kind,[str(exe)]+([] if sanitizer else ['benchmark']))
  if not sanitizer:
   for mode in ['singular_baseline','singular_candidate']:run(mode,[str(exe),mode],nonzero=True)
 probe('release',False);probe('asan_ubsan',True)
 shutil.copytree(old/'core',out/'core');shutil.copyfile(proposal/'OdometryF2M.candidate.cpp',out/'core/odometry/OdometryF2M.cpp')
 compile_cmd=[str(out/'core/odometry/OdometryF2M.cpp') if x.endswith('/core/odometry/OdometryF2M.cpp') else x.replace(str(r/'native_release_v1'),str(out)) for x in cmd]
 run('OdometryF2M_compile',compile_cmd)
 prior=json.loads((old/'result.json').read_text());link=next(s['command'] for s in prior['steps'] if s['name']=='link')
 link=[str(out/'OdometryF2M.o') if x==str(r/'native_release_v1/OdometryF2M.o') else x for x in link]
 link[link.index('-o')+1]=str(out/'librtabmap_core.so.0.23.7');run('link',link)
 for name in ['librtabmap_core.so','librtabmap_core.so.0.23']:(out/name).symlink_to('librtabmap_core.so.0.23.7')
 run('ldd',['ldd','-r',str(out/'librtabmap_core.so')]);assert not any(x in (out/'ldd.log').read_text() for x in ['not found','undefined symbol'])
 trace=out/'trace';trace.mkdir();env['WBROV_PRIVATE_OPTIMIZER_TRACE']=str(trace)
 for label,extra in [('visual',[]),('prepared',['--prepare-close-only']),('lifecycle',['--candidate-lifecycle']),('tag',['--tag-depth-proposal'])]:
  run('test_'+label,[str(old/'test_visual'),str(out/('fixture_'+label)),*extra])
 v.update(phase='passed',core_sha256=sha(out/'librtabmap_core.so.0.23.7'),f2m_sha256=sha(out/'core/odometry/OdometryF2M.cpp'))
except BaseException as ex:v.update(phase='failed',error=repr(ex),traceback=traceback.format_exc())
save()
