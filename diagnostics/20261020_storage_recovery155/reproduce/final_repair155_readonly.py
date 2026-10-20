from pathlib import Path
import fcntl,hashlib,json,os,shutil,subprocess
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
receipt=json.loads((r/'repair155_deployment/deployment_receipt.json').read_text())
assert receipt['status']=='deployed'
entry=Path('/home/exocean/Desktop/机器人键盘遥控/.tools/run_water_mapping_with_light.sh')
assert sha(entry)==receipt['after_outer']
assert all(sha(p)==pin for p,pin in receipt['protected'].items())
pkg=Path(receipt['package']);c=json.loads((pkg/'entry.json').read_text())
assert all(sha(p)==pin for p,pin in c['sha256'].items())
active=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:names=[Path(a.decode(errors='replace')).name for a in (p/'cmdline').read_bytes().split(b'\0')]
    except OSError:continue
    if any(any(token in n for token in ['teleop','run_water','zed_rgb','rgbd_odometry','wbrov_replay','deferred_mapping.py','continuous_mapping_session.py','rviz2','preview_repair155']) for n in names):
        active.append({'pid':int(p.name),'names':names})
assert not active,active
udp=subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
assert not udp
lease=(r/'speed_optimization.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
result={'scope':'post-deployment read-only verification; no new live acceptance',
    'dispatcher_sha256':sha(entry),'package_entry_sha256':sha(pkg/'entry.json'),
    'protected_files_unchanged':True,'package_pins_verified':len(c['sha256']),
    'active_mapping_or_control':active,'udp3333':udp,'serial_lock_free':True,
    'free_bytes':shutil.disk_usage(r).free}
(r/'repair155_deployment/final_readonly.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
