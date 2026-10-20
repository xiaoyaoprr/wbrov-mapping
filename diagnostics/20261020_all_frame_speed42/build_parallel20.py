from pathlib import Path
import os,json,hashlib,subprocess,time,traceback,fcntl,shutil,shlex,difflib,signal
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'parallel20_build';out.mkdir(mode=0o700)
v={'phase':'prepare','scope':'candidate: only two original projection radius searches use two OpenMP query threads; original tree/math/order unchanged; no deployment','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
    p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    previous=json.loads((r/'parallel20_probe/result.json').read_text());assert previous['phase']=='completed'
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        assert not any(x in cmd for x in ['/bin/wbrov_replay_worker','/bin/zed_rgbd_publisher_rate_ab','/tools/continuous_mapping_session.py']),cmd
    assert shutil.disk_usage(r).free>10*2**30
    package=r/'mapping_round6_candidate11_tags';c=json.loads((package/'entry.json').read_text())
    assert all(sha(p)==pin for p,pin in c['sha256'].items())
    env=dict(os.environ,**c['environment'],PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    assert 'LD_PRELOAD' not in env
    base=Path('/home/exocean/.cache/wbrov_mapping_builds/gravity_core_20260908')
    source=out/'source';source.mkdir()
    original_reg=r/'native_release_v4/core/RegistrationVis.cpp'
    assert sha(original_reg)=='4d32b793662ad69f6240aca7f8c9addb5e271edfbf8c90dac93c688db36d4ddc'
    original=original_reg.read_text();text=original
    for sorted_arg,queries in [('true','pointsToMat'),('false','cornersProjectedMat')]:
        old_params='rtflann::SearchParams()' if sorted_arg=='true' else 'rtflann::SearchParams(32, 0, false)'
        needle='index.radiusSearch('+queries+', indices, dists, radius*radius, '+old_params+');'
        assert text.count(needle)==1,needle
        replacement='rtflann::SearchParams projectionSearch(32, 0, '+sorted_arg+');\n\t\t\t\t\t\t\tprojectionSearch.cores = 2;\n\t\t\t\t\t\t\tindex.radiusSearch('+queries+', indices, dists, radius*radius, projectionSearch);'
        text=text.replace(needle,replacement)
    (source/'RegistrationVis.cpp').write_text(text)
    (source/'RegistrationVis.cpp.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),text.splitlines(True),fromfile=str(original_reg),tofile='RegistrationVis.cpp')))
    v['original_sources']={str(original_reg):sha(original_reg)};save()
    def run(name,args,timeout=300):
        v['phase']=name;save();start=time.monotonic()
        with (out/(name+'.log')).open('x') as f:
            p=subprocess.Popen(args,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
                raise
        v['steps'].append({'name':name,'command':args,'seconds':time.monotonic()-start,'exit_code':code});save();assert code==0,name
    old=json.loads((r/'native_release_v4/result.json').read_text())
    cmd=next(s['command'] for s in old['steps'] if 'RegistrationVis' in s['name'])
    cmd=[str(source/'RegistrationVis.cpp') if x==str(original_reg) else x.replace(str(r/'native_release_v4/RegistrationVis.'),str(out/'RegistrationVis.')) for x in cmd]
    cmd.insert(1,'-I'+str(original_reg.parent));run('registration_compile',cmd)
    prior=json.loads((r/'tag_speed11_build/resume_result.json').read_text())
    cmd=next(s['command'] for s in prior['steps'] if s['name']=='link')
    replacements={str(r/'native_release_v4/RegistrationVis.o'):str(out/'RegistrationVis.o')}
    assert all(k in cmd for k in replacements)
    cmd=[replacements.get(x,x) for x in cmd];cmd[cmd.index('-o')+1]=str(out/'librtabmap_core.so.0.23.7');run('link',cmd)
    for name in ['librtabmap_core.so','librtabmap_core.so.0.23']:(out/name).symlink_to('librtabmap_core.so.0.23.7')
    env['LD_LIBRARY_PATH']=str(out)+':'+env['LD_LIBRARY_PATH']
    run('ldd',['ldd','-r',str(out/'librtabmap_core.so')]);assert not any(x in (out/'ldd.log').read_text() for x in ['not found','undefined symbol'])
    run('worker_ldd',['ldd',c['worker']]);assert str(out/'librtabmap_core.so') in (out/'worker_ldd.log').read_text()
    v.update(phase='completed',core_sha256=sha(out/'librtabmap_core.so.0.23.7'),source_sha256={p.name:sha(p) for p in source.iterdir()},free_gib=shutil.disk_usage(r).free/2**30)
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
