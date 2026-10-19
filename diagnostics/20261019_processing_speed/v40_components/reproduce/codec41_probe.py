from pathlib import Path
import sys,json,subprocess,hashlib,zipfile,time,importlib.util,io,struct,threading,weakref,gc
from concurrent.futures import ThreadPoolExecutor
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018');out=r/'codec41_probe';out.mkdir(mode=0o700)
result={'phase':'waiting_for_status40','scope':'isolated Python decoder component experiment; complete input frames/checks, no hardware/runtime/deployment change','steps':[]}
def save():
 p=out/'result.tmp';p.write_text(json.dumps(result,indent=2));p.replace(out/'result.json')
def module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
def pins(paths):
 output={}
 for p in paths:
  h=hashlib.sha256()
  with p.open('rb') as f:
   while block:=f.read(1048576):h.update(block)
  output[p.name]=h.hexdigest()
 return output
save()
try:
 started=time.monotonic()
 while True:
  if json.loads((r/'status40_pipeline/result.json').read_text())['phase'] in ('passed','completed_with_recovery_failures','failed'):break
  assert time.monotonic()-started<12000;time.sleep(5)
 assert subprocess.run(['pgrep','-f','^'+str(r/'wbrov_replay_worker')],capture_output=True).returncode==1
 result['phase']='prepare';save()
 archive=r/'codec41_source.zip';assert hashlib.sha256(archive.read_bytes()).hexdigest()=='26836d28cf6cb0dc16a93bbd6dd1ed9799d117ac5aa072ada2587c6187014925'
 with zipfile.ZipFile(archive) as z:
  assert z.testzip() is None;delta=json.loads(z.read('delta.json'))
  for name,pin in delta['sha256'].items():
   assert not Path(name).is_absolute() and '..' not in Path(name).parts
   data=z.read(name);assert hashlib.sha256(data).hexdigest()==pin
   p=out/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
  (out/'delta.json').write_text(json.dumps(delta,indent=2))
 (out/'tools/spool_codec.py').write_bytes((out/'combined/spool_codec.py').read_bytes())
 with (out/'unit.log').open('x') as log:
  p=subprocess.run(['/usr/bin/python3','-I','-B','-m','unittest','discover','-s',str(out/'tests'),'-p','test_deferred_recorded_pixels.py'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,timeout=90)
 assert p.returncode==0,'existing decoder guards'
 result['steps'].append({'name':'existing_decoder_guards','exit':p.returncode});save()
 sys.path.insert(0,str(out/'tools'));helper=module('codec41_test_helpers',out/'tests/test_deferred_recorded_pixels.py')
 kinds=['baseline','context','copy','combined'];modules={k:module('codec41_'+k,out/k/'spool_codec.py') for k in kinds}
 raw=helper.frame(1791653820032937000);packed=helper.compressed_frame(raw)
 # Exhaustive small packet truncations and recovery on the SAME per-thread context.
 for kind,m in modules.items():
  for reader in (m.read_compressed_frame,m.read_compressed_map_pixels):
   for length in range(len(packed)):
    try:reader(io.BytesIO(packed[:length]));raise AssertionError((kind,'truncation accepted',length))
    except ValueError:pass
    answer=reader(io.BytesIO(packed))
    assert answer==(raw if reader==m.read_compressed_frame else (raw[:92],(2,1,[100.,100.,1.,1.],b'ABCDEF')))
   bad=bytearray(packed);bad[-1]^=1
   try:reader(io.BytesIO(bad));raise AssertionError((kind,'checksum accepted'))
   except ValueError:pass
   reader(io.BytesIO(packed))
  a=m.read_compressed_map_pixels(io.BytesIO(packed));b=m.read_compressed_frame(io.BytesIO(packed))
  assert type(a[0]) is bytes and type(a[1][3]) is bytes and type(b) is bytes
  private=m._read_shuffled_frame(io.BytesIO(packed));held=bytes(private)
  for n in range(10):m._read_shuffled_frame(io.BytesIO(helper.compressed_frame(helper.frame(n+100))))
  assert bytes(private)==held and a[0]==raw[:92] and b==raw
 # Workspace isolation and ownership after thread destruction.
 m=modules['combined'];barrier=threading.Barrier(4)
 def threaded(n):
  own=helper.frame(1791653820032937000+n);encoded=helper.compressed_frame(own)
  first=m.read_compressed_frame(io.BytesIO(encoded));context=m._contexts.decoder;pointer=context.pointer;ref=weakref.ref(context)
  barrier.wait(timeout=10)
  for _ in range(50):assert m.read_compressed_frame(io.BytesIO(encoded))==own
  return pointer,ref,first,own
 with ThreadPoolExecutor(max_workers=4) as pool:threads=list(pool.map(threaded,range(4)))
 gc.collect();assert len({x[0] for x in threads})==4
 assert all(ref() is None and first==own for _,ref,first,own in threads)
 result['steps'].append({'name':'truncation_checksum_recovery_ownership_threads','passed':True});save()
 source=r/'compressed_v35/motion_spool_v2';seals=sorted(source.glob('segment-*.json'));paths=[p.with_suffix('.frames') for p in seals]
 original=pins(paths);expected=sum(json.loads(p.read_text())['frames'] for p in seals);assert expected==1581
 result['input_sha256']=original;result['frames']=expected;result['phase']='actual_frame_equivalence';save()
 # Baseline full RGB-D decode is independent of the candidate's map-only extraction.
 total=0
 for seal,path in zip(seals,paths):
  count=json.loads(seal.read_text())['frames']
  with path.open('rb') as f:
   assert f.read(11)==b'WBROVRGBD2\n'
   for _ in range(count):
    lengths=f.read(8);plain,size=struct.unpack('<II',lengths);encoded=lengths+f.read(size)
    full=modules['baseline'].read_compressed_frame(io.BytesIO(encoded));assert len(full)==plain
    w0,h0=struct.unpack_from('<II',full,92);offset=132+w0*h0*7
    w1,h1,*intrinsics=struct.unpack_from('<II4d',full,offset);begin=offset+40
    expected_image=(w1,h1,intrinsics,full[begin:begin+w1*h1*3]) if w1 else None
    for kind,m in modules.items():
     assert m.read_compressed_frame(io.BytesIO(encoded))==full,kind
     assert m.read_compressed_map_pixels(io.BytesIO(encoded))==(full[:92],expected_image),kind
    total+=1
   assert not f.read(1)
 assert total==expected;result['actual_frame_equivalence_passed']=True;save()
 def read_all(m):
  count=0;first=None;last=0
  for seal,path in zip(seals,paths):
   with path.open('rb') as f:
    assert f.read(11)==b'WBROVRGBD2\n'
    for _ in range(json.loads(seal.read_text())['frames']):
     header,image=m.read_compressed_map_pixels(f);stamp=struct.unpack_from('<Q',header)[0]
     assert stamp>last;last=stamp;first=stamp if first is None else first;count+=1
    assert not f.read(1)
  assert count==expected
  return count,first,last
 result['phase']='rotating_benchmark';result['measurements']=[];save();same=None
 for turn in range(6):
  values={};order=kinds[turn%4:]+kinds[:turn%4]
  if turn%2:order=list(reversed(order))
  for kind in order:
   started=time.perf_counter_ns();actual=read_all(modules[kind]);elapsed=(time.perf_counter_ns()-started)/1e6
   if same is None:same=actual
   assert actual==same;values[kind]=elapsed
  result['measurements'].append({'round':turn,'frames':expected,'order':order,'ms':values});save()
 assert pins(paths)==original;result['original_input_unchanged']=True;result['phase']='finished';result['adopted']=False
except BaseException as e:
 result['phase']='failed';result['error']=repr(e)
 import traceback
 (out/'failure.log').write_text(traceback.format_exc())
save()
