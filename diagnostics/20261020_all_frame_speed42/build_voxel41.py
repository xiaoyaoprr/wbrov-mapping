from pathlib import Path
import os,json,time,hashlib,subprocess,traceback,fcntl,shutil,signal
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'voxel41_build';out.mkdir(mode=0o700)
v={'phase':'prepare','scope':'worker10 only: export index uses proven NarrowVoxel40 with automatic wide fallback; Core20, all input/quality/recovery/Tag/point-density settings unchanged; no deployment','steps':[]}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save():
    p=out/'result.tmp';p.write_text(json.dumps(v,indent=2));p.replace(out/'result.json')
save()
try:
    lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    probe=r/'voxel40_probe_v2';result=json.loads((probe/'result.json').read_text())
    assert result['phase']=='completed'and result['sanitizer_passed']and result['closed_map_byte_equivalence_passed']and result['synthetic_and_regression_passed']
    assert result['transform_union_comparison']['saved_fraction']>=.10
    assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        assert not any(x in cmd for x in ['/bin/wbrov_replay_worker','/bin/zed_rgbd_publisher_rate_ab','/tools/continuous_mapping_session.py']),cmd
    assert shutil.disk_usage(r).free>10*2**30
    package=r/'mapping_round6_candidate22_parallel';c=json.loads((package/'entry.json').read_text());assert all(sha(p)==pin for p,pin in c['sha256'].items())
    env=dict(os.environ);env.update(c['environment']);env.update(PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE')
    assert 'LD_PRELOAD'not in env
    native=r/'round6_recovery10_build';old=json.loads((native/'result.json').read_text());assert old['phase']=='passed'
    assert sha(c['worker'])==old['worker_sha256']=='f0f70bed93c81f678a71a2aae45c11481ab4e7ae548c2b466e498fae11e61d9b'
    source=out/'source';source.mkdir()
    for name,pin in old['worker_sources'].items():
        p=native/'worker'/name;assert sha(p)==pin;shutil.copyfile(p,source/name)
    header=probe/'source/NarrowVoxel40.h';assert sha(header)=='845948a67c329af532837b9d5ef107d4dcb8c79759a96ae9a7c0f4ded5e8c8ae'
    shutil.copyfile(header,source/'NarrowVoxel40.h')
    shutil.copyfile(probe/'regression/RecordedVoxelCloud.h',source/'RecordedVoxelCloud.h')
    assert all(sha(source/name)==pin for name,pin in old['worker_sources'].items()if name!='RecordedVoxelCloud.h')
    v['source_changes']={name:sha(source/name)for name in ['NarrowVoxel40.h','RecordedVoxelCloud.h']};save()
    def run(name,args,timeout=180):
        v['phase']=name;save();start=time.monotonic()
        with (out/(name+'.log')).open('x')as f:
            p=subprocess.Popen(args,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:rc=p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
                raise
        v['steps'].append({'name':name,'command':args,'seconds':time.monotonic()-start,'exit_code':rc});save();assert rc==0,name
    for name in ['worker_compile','worker_link']:
        cmd=next(s['command']for s in old['steps']if s['name']==name)
        cmd=[x.replace(str(native/'worker'),str(source)).replace(str(native/'wbrov_replay_worker'),str(out/'wbrov_replay_worker'))for x in cmd]
        cmd=[str(package/'lib/librtabmap_core.so')if x.endswith('/librtabmap_core.so')else x for x in cmd]
        run(name,cmd,300)
    run('ldd',['ldd','-r',str(out/'wbrov_replay_worker')]);ld=(out/'ldd.log').read_text()
    assert str(package/'lib/librtabmap_core.so')in ld and not any(x in ld for x in ['not found','undefined symbol'])
    assert all(sha(p)==pin for p,pin in c['sha256'].items())
    assert sha('/home/exocean/Desktop/机器人键盘遥控/.tools/run_water_mapping_with_light.sh')=='d1581976c17bc6a68b32aa45ff4eac73c7ec3930b392b79e04c82b6061a6436a'
    v.update(phase='completed',worker_sha256=sha(out/'wbrov_replay_worker'),unchanged_other_sources=True,free_gib=shutil.disk_usage(r).free/2**30)
except BaseException as e:v.update(phase='failed',error=repr(e),traceback=traceback.format_exc())
save()
