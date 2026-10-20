from pathlib import Path
import os,json,hashlib,subprocess,time,traceback,fcntl,shutil,shlex,re
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'repair152_build';out.mkdir(mode=0o700)
v={'phase':'prepare','scope':'separate raw-six-axis and measured-gravity RGB-D local trajectories; original per-edge raw/odom checks and all constrained geometry checks retained; yaw/translation still measured from raw pairs','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
    t=out/'result.tmp';t.write_text(json.dumps(v,indent=2));t.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert json.loads((r/'repair123_trim/result.json').read_text())['phase']=='completed'
    assert json.loads((r/'repair123_storage/result.json').read_text())['phase']=='completed'
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    c=json.loads((r/'mapping_round6_candidate42_voxel/entry.json').read_text())
    assert all(sha(p)==pin for p,pin in c['sha256'].items())
    previous=r/'round6_recovery9_build'
    assert sha(previous/'core/recovery_geometry.inc')=='bef59c5315c22220943c49c67f91ba87c942c9aec9485205b2ceaa8b976d7bb5'
    pins=json.loads((r/'repair152_sources.json').read_text())
    for name,pin in pins.items():assert sha(r/'repair152_input'/name)==pin
    shutil.copytree(previous/'core',out/'core')
    for name in ['recovery_geometry.inc','visual_recovery.inc']:
        shutil.copyfile(r/'repair152_input'/name,out/'core'/name)
    env=dict(os.environ);env.update(c['environment']);env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE',PYTHONDONTWRITEBYTECODE='1')
    assert 'LD_PRELOAD' not in env
    trace=out/'trace_tests';trace.mkdir();env['WBROV_PRIVATE_OPTIMIZER_TRACE']=str(trace)
    env['LD_LIBRARY_PATH']=str(out)+':'+env['LD_LIBRARY_PATH']
    def run(name,args,timeout=300,cwd=None):
        v['phase']=name;save();t=time.monotonic()
        with (out/(name+'.log')).open('x') as f:
            p=subprocess.run(args,env=env,cwd=cwd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
        v['steps'].append({'name':name,'seconds':time.monotonic()-t,'exit':p.returncode,'command':args});save()
        assert p.returncode==0,name
    old=json.loads((previous/'result.json').read_text())
    cmd=next(s['command'] for s in old['steps'] if s['name']=='Rtabmap_compile')
    run('Rtabmap_compile',[a.replace(str(previous),str(out)) for a in cmd],1200)
    link=next(s['command'] for s in json.loads((r/'parallel20_build/result.json').read_text())['steps'] if s['name']=='link')
    assert str(previous/'Rtabmap.o') in link
    link=[str(out/'Rtabmap.o') if a==str(previous/'Rtabmap.o') else a for a in link]
    link[link.index('-o')+1]=str(out/'librtabmap_core.so.0.23.7')
    run('link',link,180)
    for name in ['librtabmap_core.so','librtabmap_core.so.0.23']:(out/name).symlink_to('librtabmap_core.so.0.23.7')
    run('bindings',['ldd','-r',str(out/'librtabmap_core.so')])
    assert not any(x in (out/'bindings.log').read_text() for x in ['not found','undefined symbol'])
    root=r.parent/'round4_20261016';build=root/'build'
    flags=(build/'CMakeFiles/test_visual_recovery_native.dir/flags.make').read_text();args=[]
    for name in ['CXX_DEFINES','CXX_INCLUDES']:
        args+=shlex.split(next(x.split('=',1)[1].strip() for x in flags.splitlines() if x.startswith(name+' =')))
    runtime=next(Path(p) for p in env['LD_LIBRARY_PATH'].split(':') if 'sha256-439961' in p)
    def native(name,source,include):
        run(name+'_compile',['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-UNDEBUG',
            '-I'+str(out/'core/headers'),'-I'+str(include),*args,'-c',str(source),'-o',str(out/(name+'.o'))],300)
        cmd=shlex.split((build/'CMakeFiles/test_visual_recovery_native.dir/link.txt').read_text())
        cmd=[str(out/(name+'.o')) if a.endswith('.o') else str(out/name) if a=='test_visual_recovery_native'
             else str(out/'librtabmap_core.so') if 'librtabmap_core.so' in a else a for a in cmd]
        cmd+=['-Wl,-rpath-link,'+str(runtime)]
        run(name+'_link',cmd,120,build)
    native('test_visual',previous/'test_visual_recovery_native.cpp',previous)
    for label,extra in [('visual',[]),('prepared',['--prepare-close-only']),('lifecycle',['--candidate-lifecycle']),('tag',['--tag-depth-proposal'])]:
        run('test_'+label,[str(out/'test_visual'),str(out/('fixture_'+label)),*extra],300)
    native('metric_test',r/'repair152_input/metric152_test.cpp',previous)
    run('metric_test',[str(out/'metric_test')],30)
    native('gravity_test',r/'repair152_input/gravity152_test.cpp',previous)
    run('gravity_test',[str(out/'gravity_test')],30)
    assert all(sha(p)==pin for p,pin in c['sha256'].items())
    v.update(phase='completed',core_sha256=sha(out/'librtabmap_core.so.0.23.7'),source_pins=pins,production_changed=False)
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
