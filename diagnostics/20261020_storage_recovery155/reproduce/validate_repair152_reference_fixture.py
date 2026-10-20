from pathlib import Path
import os,json,hashlib,time,subprocess,shlex,fcntl,traceback
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'repair152_reference_fixture';out.mkdir(mode=0o700)
v={'phase':'prepare','steps':[]}
def save():
    p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    core=r/'repair152_build';built=json.loads((core/'result.json').read_text())
    assert built['phase']=='completed'
    sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    assert sha(core/'librtabmap_core.so.0.23.7')==built['core_sha256']
    source=r/'test_visual_reference152.cpp'
    assert sha(source)=='29efa1489dbc9d18840b0e7194924ceeeeae2f4e499c5a84061aba2f2ab2584f'
    c=json.loads((r/'mapping_round6_candidate42_voxel/entry.json').read_text())
    env=dict(os.environ);env.update(c['environment'])
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE')
    env['LD_LIBRARY_PATH']=str(core)+':'+env['LD_LIBRARY_PATH']
    trace=out/'trace';trace.mkdir();env['WBROV_PRIVATE_OPTIMIZER_TRACE']=str(trace)
    def run(name,args,timeout):
        v['phase']=name;save();t=time.monotonic()
        with (out/(name+'.log')).open('x') as f:
            p=subprocess.run(args,env=env,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=timeout)
        v['steps'].append({'name':name,'seconds':time.monotonic()-t,'exit':p.returncode});save()
        assert p.returncode==0,name
    commands={s['name']:s['command'] for s in built['steps']}
    cmd=commands['test_visual_compile']
    cmd=[str(source) if a.endswith('/test_visual_recovery_native.cpp') else
         str(out/'test_visual.o') if a==str(core/'test_visual.o') else a for a in cmd]
    run('compile',cmd,300)
    cmd=[str(out/'test_visual.o') if a==str(core/'test_visual.o') else
         str(out/'test_visual') if a==str(core/'test_visual') else a for a in commands['test_visual_link']]
    run('link',cmd,120)
    run('fallback_test',[str(out/'test_visual'),str(out/'fixture'),'--candidate-reference-fallback'],300)
    v.update(phase='completed',core_sha256=built['core_sha256'],source_sha256=sha(source))
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
