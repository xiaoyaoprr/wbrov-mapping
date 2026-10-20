from pathlib import Path
import json,csv,hashlib,sqlite3,subprocess,collections
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');o=r/'round6_field_notags_v8'
driver=json.loads((o/'result.json').read_text());assert driver['phase']=='closed'
d=Path(driver['mapping_run']);p=d/'processing';entry=json.loads((d/'entry_status.json').read_text())
state=json.loads((p/'status.json').read_text());m=state['mapping'];saved=json.loads((p/'save.json').read_text())
assert m['database_closed'] and all(not Path('/proc',str(pid)).exists() for pid in entry['pids'].values())
for name,pin in saved['sha256'].items():assert Path(name).name==name and hashlib.sha256((p/'mapping'/name).read_bytes()).hexdigest()==pin
with sqlite3.connect((p/'mapping/map.db').as_uri()+'?mode=ro&immutable=1',uri=True) as db:check=db.execute('PRAGMA quick_check').fetchone()[0]
assert check=='ok'
rows=list(csv.DictReader((p/'mapping/frames.csv').open()));stamps=[int(x['source_ns']) for x in rows]
raw=json.loads((d/'raw_recording/status.json').read_text());spool=json.loads((d/'rgbd_spool/status.json').read_text())
verification=json.loads((d/'raw_recording/verification.json').read_text())
captured=[]
for line in (d/'raw_recording/capture.jsonl').open():
 stamp=json.loads(line)['stamp_ns']
 if not captured or stamp!=captured[-1]:captured.append(stamp)
assert captured==stamps and len(rows)==raw['unique_computed_stamps']==spool['frames']==verification['frames']['frames']
assert all(b>a for a,b in zip(stamps,stamps[1:]))
assert saved['complete'] and spool['clean_shutdown'] and verification['complete']
assert state['completed_segments']==spool['sealed_segments'] and not json.loads((p/'mapping/tags.json').read_text())['tags']
assert not m['detected_tags_in_processed_frame'] and not m['main_tag_count_at_checkpoint']
assert not any(x['main_write']=='1' and (x['lost']=='1' or x['input_present']=='0') for x in rows)
intervals=[(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]
events=[];previous=int(rows[0]['cuts']);previousWrite=rows[0]['main_write']
for row in rows[1:]:
 if int(row['cuts'])>previous or previousWrite=='1' and row['main_write']=='0':events.append(row)
 previous=int(row['cuts']);previousWrite=row['main_write']
latency=[json.loads(line) for line in (d/'rgbd_spool/seal_latency.jsonl').open()]
result={'scope':'stationary candidate capture and save check with Tag disabled; no new operator movement requested and no assistant movement command',
 'run_dir':str(d),'startup_seconds':driver.get('ready_elapsed_s'),'wrapper_exit':driver['wrapper_exit'],
 'entry_errors':entry['errors'],'child_exits':entry['exits'],'all_frames':len(rows),'source_seconds':(stamps[-1]-stamps[0])/1e9,
 'all_input_stamps_equal':True,'save_hashes_match':True,'database_check':check,'raw_svo_verified':True,'tag_detection_enabled':False,
 'source_gap_max_ms':max(intervals),'source_gaps_over_300ms':sum(x>300 for x in intervals),'spool':spool,
 'fsync_max_ms':max(x['data_fsync_ms'] for x in latency),'commit_max_ms':max(x['total_ms'] for x in latency),
 'final_continuity':m['continuity_state'],'final_main_write':m['main_map_write_allowed'],'main_nodes':m['main_nodes'],
 'cloud_points':m['cloud_points'],'cuts':m['cuts'],'recoveries':m['recoveries'],'pauses_and_cuts':events,
 'controller_socket_released':not subprocess.check_output(['ss','-H','-ulpn','sport = :3333'],text=True).strip()}
for name in ['topic_probe.json','movement_requested.json','movement_completed.json']:
 if (o/name).exists():result[name]=json.loads((o/name).read_text())
(o/'audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='pauses_and_cuts'},ensure_ascii=False));print('pause_or_cut_events',len(events))
