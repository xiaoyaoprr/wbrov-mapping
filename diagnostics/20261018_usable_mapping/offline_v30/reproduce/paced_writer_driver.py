from pathlib import Path
import os,json,subprocess,time,re
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'paced_writer_v30';out.mkdir(mode=0o700)
result={'scope':'offline RGB-D writer only; no camera, SVO or lights','phase':'waiting_for_replays'}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
save()
try:
 deadline=time.monotonic()+4000
 while True:
  phase=json.loads((r/'offline_v30/result.json').read_text())['phase']
  if phase in ['passed','failed']:break
  assert time.monotonic()<deadline,'replay wait timeout'
  time.sleep(5)
 assert phase=='passed','preceding replay acceptance failed'
 assert not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()
 result['phase']='compile';save()
 q=subprocess.run(['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-pthread','-I'+str(r/'worker_source_v30'),str(r/'pace_rgbd_spool_writer.cpp'),'-lzstd','-o',str(out/'pace_rgbd_spool_writer')],capture_output=True,text=True,timeout=120)
 (out/'compile.log').write_text(q.stdout+q.stderr);assert q.returncode==0,'compile'
 source=r/'motion_spool1';assert json.loads((source/'status.json').read_text())['clean_shutdown']
 result['phase']='paced_write';save()
 with (out/'stdout.log').open('x') as stdout,(out/'stderr.log').open('x') as stderr:
  q=subprocess.run([str(out/'pace_rgbd_spool_writer'),str(source),str(out/'rgbd_spool')],stdout=stdout,stderr=stderr,timeout=240)
 assert q.returncode==0,'paced write or byte comparison failed'
 result['measurement']=json.loads((out/'stdout.log').read_text())
 result['spool_status']=json.loads((out/'rgbd_spool/status.json').read_text())
 assert result['spool_status']['clean_shutdown'] and result['measurement']['frames']==json.loads((source/'status.json').read_text())['frames']
 log=(out/'stderr.log').read_text();result['backpressure_events']=log.count('[RGBD_SPOOL_BACKPRESSURE]')
 result['io_max_ms']={key:max([float(x) for x in re.findall(r'\b'+key+r'=([\d.]+)',log)],default=0) for key in ['wait_ms','flush_ms','data_sync_ms','receipt_sync_ms']}
 result['phase']='passed'
except BaseException as e:result.update(phase='failed',error=repr(e))
save()
