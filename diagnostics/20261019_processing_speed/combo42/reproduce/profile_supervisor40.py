"""Offline wrapper: measure unchanged supervisor calls, never bypass checks."""
from pathlib import Path
import sys,json,time,threading,hashlib
tools=Path(sys.argv[1]).resolve(strict=True)
sys.path.insert(0,str(tools));sys.argv=[sys.argv[0],*sys.argv[2:]]
output=Path(sys.argv[sys.argv.index('--output')+1])
import deferred_mapping as mapping
import deferred_tag_observations as tags
import spool_codec

stats={};lock=threading.Lock()
def measure(label,function):
 def timed(*args,**kwargs):
  category=label(*args,**kwargs) if callable(label) else label
  start=time.perf_counter_ns()
  try:return function(*args,**kwargs)
  finally:
   elapsed=time.perf_counter_ns()-start
   with lock:
    entry=stats.setdefault(category,{'calls':0,'total_ns':0,'maximum_ns':0})
    entry['calls']+=1;entry['total_ns']+=elapsed;entry['maximum_ns']=max(entry['maximum_ns'],elapsed)
 return timed

def atomic_kind(path,*unused,**kwargs):
 name=Path(path).name
 return 'atomic_json/'+('input' if name.startswith('input-') else 'receipt' if name.startswith('receipt-') else name)
def hash_kind(path):
 path=Path(path)
 return 'sha/'+('recorded_frames' if path.suffix=='.frames' else 'map_db' if path.name=='map.db' else 'main_cloud' if path.name=='main_cloud.ply' else 'tags' if path.name=='tags.json' else 'other')

pins={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(tools.glob('*.py'))}
mapping.atomic_json=measure(atomic_kind,mapping.atomic_json)
mapping.sha=measure(hash_kind,mapping.sha)
mapping.closed_inputs=measure('closed_inputs',mapping.closed_inputs)
mapping.descriptor=measure('descriptor',mapping.descriptor)
mapping.PreparedInput.take=measure('prepared_take_wait',mapping.PreparedInput.take)
tags.TagObserver.observe=measure('tag_observer',tags.TagObserver.observe)
spool_codec._read_shuffled_frame=measure('python_decode',spool_codec._read_shuffled_frame)
started=time.perf_counter_ns();code=None
try:
 code=mapping.main()
finally:
 report={'scope':'diagnostic wrapper only; measures pinned implementation without additional skips or state changes; nested and concurrent times are not additive','elapsed_ns':time.perf_counter_ns()-started,'return_code':code,'measurements':stats,'tools_sha256':pins}
 report['tools_unchanged']=all(hashlib.sha256((tools/name).read_bytes()).hexdigest()==pin for name,pin in pins.items())
 if output.is_dir():
  with (output/'supervisor_timings.json').open('x') as f:json.dump(report,f,indent=2)
 assert report['tools_unchanged'],'frozen supervisor sources changed'
raise SystemExit(code)
