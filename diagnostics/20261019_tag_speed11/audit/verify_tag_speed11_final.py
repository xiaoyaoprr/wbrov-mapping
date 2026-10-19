from pathlib import Path
import json,hashlib,subprocess,shutil
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');p=r/'mapping_round6_candidate11_tags'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
c=json.loads((p/'entry.json').read_text());assert c['tag_modules']==str(p/'runtime/libexec/wbrov_corner_recovery')
assert 'LD_PRELOAD' not in c['environment'];assert all(sha(path)==pin for path,pin in c['sha256'].items())
dispatcher=Path('/home/exocean/Desktop/机器人键盘遥控/.tools/run_water_mapping_with_light.sh')
assert sha(dispatcher)=='d1581976c17bc6a68b32aa45ff4eac73c7ec3930b392b79e04c82b6061a6436a'
prod=json.loads((r/'mapping_combo42_camera6/entry.json').read_text());assert all(sha(path)==pin for path,pin in prod['sha256'].items())
active=[]
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
 except OSError:continue
 if any(v in cmd for v in ['/bin/zed_rgbd_publisher_rate_ab','/bin/wbrov_replay_worker','/tools/continuous_mapping_session.py','/src/visualization/deferred_map_display.py']):active.append({'pid':proc.name,'cmd':cmd})
socket=subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True)
assert not active and not socket.strip()
print(json.dumps({'candidate_pins_verified':True,'tag_detection_enabled':True,'diagnostic_preload_absent':True,'production_dispatcher_sha256':sha(dispatcher),'production_pins_unchanged':True,'active_mapping_processes':active,'controller_socket':socket,'free_gib':shutil.disk_usage(r).free/2**30,'field_acceptance':False}))
for name in ['result.json','publisher.log','rviz.log']:
 q=r/'display_tag_speed11'/name;print(name,q.read_text()[-6000:])
