from pathlib import Path
import os,subprocess,json,time,signal,hashlib
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'repair155_display_rgb';out.mkdir(mode=0o700)
runtime=Path('/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-439961e59d783abc416c2507a947dfe222965bd6a7ad9da4eed6e203d4d3b31b')
pkg=r/'mapping_round6_candidate155_recovery';rviz=pkg/'bin/rviz2'
assert hashlib.sha256(rviz.read_bytes()).hexdigest()=='6ce75960305230fe63ad2ce41dbcf3f2fb865e67e6e5216059929eedc1dfce78'
assert json.loads((r/'repair155_package/result.json').read_text())['phase']=='completed'
assert subprocess.run(['pgrep','-x','rviz2'],capture_output=True).returncode==1,'user RViz active'
ogre=Path('/opt/ros/humble/opt/rviz_ogre_vendor/lib')
assert (ogre/'libOgreMain.so.1.12.1').exists()
env=dict(os.environ,ROS_DOMAIN_ID='208',ROS_LOCALHOST_ONLY='1',PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',
 DISPLAY=':0',XAUTHORITY='/run/user/1000/gdm/Xauthority',XDG_RUNTIME_DIR='/run/user/1000',DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/1000/bus',
 PYTHONPATH=str(runtime/'local/lib/python3.10/dist-packages')+':'+str(runtime/'lib/python3.10/site-packages')+':/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages',
 AMENT_PREFIX_PATH='/home/exocean/opt/rtabmap/opt/ros/humble:/opt/ros/humble',
 LD_LIBRARY_PATH=str(pkg/'lib')+':'+str(ogre)+':'+str(runtime/'lib')+':/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu')
report={'scope':'RViz display of repair153 saved complete field119 recovery replay; historical data only','hardware_started':False}
display_log=(out/'publisher.log').open('x');rviz_log=(out/'rviz.log').open('x');processes=[]
try:
 publisher=subprocess.Popen(['/usr/bin/python3','-B',str(pkg/'src/visualization/deferred_map_display.py'),str(r/'repair153_replay/field')],env=env,stdout=display_log,stderr=subprocess.STDOUT)
 processes.append(publisher)
 rgb_log=(out/'recorded_rgb.log').open('x')
 rgb=subprocess.Popen(['/usr/bin/python3','-B',str(r/'preview_repair155_saved_rgb.py')],env=env,stdout=rgb_log,stderr=subprocess.STDOUT)
 processes.append(rgb)
 config=out/'saved_data_only.rviz'
 config.write_text((pkg/'src/visualization/deferred_mapping.rviz').read_text().replace('/zed/rgb/image_rect_color','/wbrov/test/recorded_rgb').replace('Name: 相机图像','Name: 已保存RGB回放'))
 viewer=subprocess.Popen([str(rviz),'-d',str(config)],env=env,stdout=rviz_log,stderr=subprocess.STDOUT)
 processes.append(viewer)
 deadline=time.monotonic()+45
 while time.monotonic()<deadline:
  if publisher.poll() is not None or viewer.poll() is not None:raise RuntimeError('display subprocess exited')
  if '[WBROV_RVIZ_RENDER_READY]' in (out/'rviz.log').read_text() and 'points=' in (out/'publisher.log').read_text():break
  time.sleep(.5)
 else:raise RuntimeError('display admission timeout')
 time.sleep(3)
 assert rgb.poll() is None
 assert '"published": 10' in (out/'recorded_rgb.log').read_text()
 report['recorded_rgb_published']=True
 p=subprocess.run(['gnome-screenshot','-f',str(out/'rviz.png')],env=env,text=True,capture_output=True,timeout=15)
 report.update(render_ready=True,screenshot_exit=p.returncode,screenshot_error=p.stderr)
except BaseException as e:report['error']=repr(e)
finally:
 for p in reversed(processes):
  if p.poll() is None:p.send_signal(signal.SIGINT)
 for p in reversed(processes):
  try:p.wait(timeout=12)
  except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
 report['subprocess_exit_codes']=[p.returncode for p in processes]
 display_log.close();rviz_log.close();(out/'result.json').write_text(json.dumps(report,indent=2))


print(json.dumps(report))
assert report.get("render_ready") and not report.get("error")
