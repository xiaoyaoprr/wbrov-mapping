from pathlib import Path
import json,sqlite3,struct,math,statistics,hashlib
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018/round6_recovery9_replay2')
for name in ['baseline_motion','candidate_motion']:
 p=r/name/'mapping';s=json.loads((p/'status.json').read_text());assert s['database_closed']
 dbpath=p/'map.db';before=hashlib.sha256(dbpath.read_bytes()).hexdigest()
 with sqlite3.connect(dbpath.as_uri()+'?mode=ro&immutable=1',uri=True) as db:
  print(name,'schema',db.execute('pragma table_info(Node)').fetchall(),db.execute('pragma table_info(Link)').fetchall())
  print('links',db.execute('select type,count(*),length(transform) from Link group by type').fetchall())
  gravity={a:struct.unpack('<12f',t) for a,t in db.execute('select from_id,transform from Link where type=9 and from_id=to_id')}
  main={x['node_id'] for x in json.loads((p/'tags.json').read_text())['trajectory']}
  def error(a,b):
   a=a[8:11];b=b[8:11];v=sum(x*y for x,y in zip(a,b))/math.sqrt(sum(x*x for x in a)*sum(x*x for x in b));return math.degrees(math.acos(min(1,max(-1,v))))
  groups={'main':[],'local':[]}
  for i,stamp,pose in db.execute('select id,stamp,pose from Node'):
   if i in gravity:groups['main' if i in main else 'local'].append({'id':i,'stamp':stamp,'raw_pose_gravity_deg':error(struct.unpack('<12f',pose),gravity[i])})
  print(json.dumps({k:{'count':len(v),'mean':statistics.mean(x['raw_pose_gravity_deg'] for x in v),'max':max(x['raw_pose_gravity_deg'] for x in v),'last':v[-12:]} for k,v in groups.items() if v}))
  optimized=[]
  for item in json.loads((p/'tags.json').read_text())['trajectory']:
   i=item['node_id'];x,y,z,w=item['orientation_xyzw']
   rotation=[0]*8+[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0]
   if i in gravity:optimized.append({'id':i,'optimized_gravity_deg':error(rotation,gravity[i])})
  print('optimized_main',json.dumps({'mean':statistics.mean(x['optimized_gravity_deg'] for x in optimized),'max':max(x['optimized_gravity_deg'] for x in optimized),'last':optimized[-15:]}))
 assert before==hashlib.sha256(dbpath.read_bytes()).hexdigest()
