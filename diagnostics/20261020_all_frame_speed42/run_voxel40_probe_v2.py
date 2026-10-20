from pathlib import Path
import os,json,time,hashlib,subprocess,traceback,fcntl,shutil,zipfile,signal
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'voxel40_probe_v2';out.mkdir(mode=0o700)
v={'phase':'prepare','scope':'narrow export index only; synthetic point bits and forced wide fallback, current cloud regressions, closed saved-map cloud microbench; no full replay/deployment','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
    p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    previous=json.loads((r/'heap39_validation/result.json').read_text());assert previous['phase']=='completed' and previous['output_equivalence_passed']
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        assert not any(x in cmd for x in ['/bin/wbrov_replay_worker','/bin/zed_rgbd_publisher_rate_ab','/tools/continuous_mapping_session.py','/round6_recovery9_build/test_visual']),cmd
    assert shutil.disk_usage(r).free>10*2**30
    package=r/'mapping_round6_candidate22_parallel';c=json.loads((package/'entry.json').read_text());assert all(sha(p)==pin for p,pin in c['sha256'].items())
    env=dict(os.environ);env.update(c['environment']);env.update(PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE')
    assert 'LD_PRELOAD' not in env
    archive=r/'voxel40_probe_source.zip';assert sha(archive)=='77a8893608d00cb9ffbe4d8203edc53cf0cefd19a6f4d1fcf3a1fe8ba43ff1bd'
    source=out/'source';source.mkdir()
    with zipfile.ZipFile(archive)as z:
        for name in z.namelist():
            p=(source/name).resolve();assert p.is_relative_to(source.resolve());p.write_bytes(z.read(name))
    manifest=json.loads((source/'manifest.json').read_text());assert all(sha(source/p)==pin for p,pin in manifest['files'].items())
    native=r/'round6_recovery10_build';built=json.loads((native/'result.json').read_text());assert built['phase']=='passed'
    assert built['worker_sha256']==sha(c['worker'])=='f0f70bed93c81f678a71a2aae45c11481ab4e7ae548c2b466e498fae11e61d9b'
    original=native/'worker';assert sha(original/'RecordedVoxelCloud.h')==manifest['original_voxel_sha256']
    for name in ['RecordedVoxelCloud.h','RecordedCloudCache.h']:
        assert sha(original/name)==built['worker_sources'][name];shutil.copyfile(original/name,source/name)
    regression=out/'regression';regression.mkdir()
    for p in original.iterdir():
        if p.suffix in ['.h','.hpp'] or p.name=='test_recorded_cloud_cache.cpp':
            assert sha(p)==built['worker_sources'][p.name];shutil.copyfile(p,regression/p.name)
    shutil.copyfile(source/'NarrowVoxel40.h',regression/'NarrowVoxel40.h')
    (regression/'RecordedVoxelCloud.h').write_text('#pragma once\n#include "NarrowVoxel40.h"\ntemplate<class Point>using RecordedVoxelCloud=NarrowVoxel40<Point>;\n')
    def run(name,args,timeout=120,extra=None):
        v['phase']=name;save();start=time.monotonic()
        with (out/(name+'.log')).open('x')as f:
            p=subprocess.Popen(args,env=dict(env,**(extra or {})),stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:rc=p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
                raise
        v['steps'].append({'name':name,'command':args,'seconds':time.monotonic()-start,'exit_code':rc});save();assert rc==0,name
    exe=out/'correctness'
    run('correctness_compile',['g++','-std=c++17','-O2','-Wall','-Wextra',str(source/'voxel40_correctness.cpp'),'-o',str(exe)])
    run('correctness',[str(exe)])
    run('sanitizer_compile',['g++','-std=c++17','-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer',str(source/'voxel40_correctness.cpp'),'-o',str(out/'sanitized')])
    run('sanitizer',[str(out/'sanitized')],120,{'ASAN_OPTIONS':'detect_leaks=1:halt_on_error=1','UBSAN_OPTIONS':'halt_on_error=1:print_stacktrace=1'})
    run('regression_compile',['g++','-std=c++17','-O2','-UNDEBUG','-pthread',str(regression/'test_recorded_cloud_cache.cpp'),'-o',str(out/'regression_test')])
    run('regression',[str(out/'regression_test')])
    compile=next(s['command']for s in built['steps']if s['name']=='worker_compile').copy()
    compile=[str(source/'benchmark.cpp')if x==str(original/'replay_worker.cpp')else str(out/'benchmark.o')if x==str(native/'wbrov_replay_worker.o')else x for x in compile]
    run('benchmark_compile',compile,180)
    link=next(s['command']for s in built['steps']if s['name']=='worker_link').copy()
    link=[str(out/'benchmark.o')if x==str(native/'wbrov_replay_worker.o')else str(out/'benchmark')if x==str(native/'wbrov_replay_worker')else x for x in link]
    run('benchmark_link',link,120)
    run('ldd',['ldd',str(out/'benchmark')]);ld=(out/'ldd.log').read_text();assert str(package/'lib/librtabmap_core.so')in ld and 'not found'not in ld
    fixture=r/'parallel20_validation/candidate_field/mapping';state=json.loads((fixture/'status.json').read_text());assert state['state']=='closed'and state['database_closed']
    saved=json.loads((fixture.parent/'save.json').read_text());assert saved['complete']
    pins={str(fixture/name):sha(fixture/name)for name in ['status.json','map.db','tags.json','main_cloud.ply']}
    assert pins[str(fixture/'map.db')]==saved['sha256']['map.db']
    v['fixture_pins']=pins;save()
    run('benchmark',[str(out/'benchmark'),str(fixture)],240)
    rows=[json.loads(line)for line in (out/'benchmark.log').read_text().splitlines()if line.startswith('{')]
    assert len(rows)==8 and all(row['byte_exact']and row['nodes']==124 for row in rows)
    measured=rows[2:];before=sum(sum(x['before_ms'][:2])for x in measured);after=sum(sum(x['after_ms'][:2])for x in measured)
    v['measurements']=rows;v['transform_union_comparison']={'baseline_ms':before,'candidate_ms':after,'saved_fraction':1-after/before,'measured_rounds':6,'warmup_rounds':2}
    assert all(sha(p)==pin for p,pin in pins.items())and all(sha(p)==pin for p,pin in c['sha256'].items())
    assert sha('/home/exocean/Desktop/机器人键盘遥控/.tools/run_water_mapping_with_light.sh')=='d1581976c17bc6a68b32aa45ff4eac73c7ec3930b392b79e04c82b6061a6436a'
    v.update(phase='completed',synthetic_and_regression_passed=True,sanitizer_passed=True,closed_map_byte_equivalence_passed=True,free_gib=shutil.disk_usage(r).free/2**30)
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
