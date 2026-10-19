from pathlib import Path
import os,sys,json,subprocess,time,threading
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
p=r/'mapping_round6_candidate6_notags';out=r/'round6_candidate6_drain_test';out.mkdir(mode=0o700)
sys.path.insert(0,str(p/'tools'))
from continuous_mapping_session import Session
state=out/'state';state.mkdir();run=out/'run';run.mkdir()
ready=out/'child.ready';stopping=out/'stop.received';sealed=out/'sealed.json'
source='''from pathlib import Path
import sys,signal,time,json
ready,stopping,sealed=map(Path,sys.argv[1:])
def stop(*_):stopping.touch()
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,lambda *_:sys.exit(91))
ready.touch()
while not stopping.exists():time.sleep(.02)
time.sleep(75)
sealed.write_text(json.dumps({'synthetic_frames':100,'sealed_frames':100,'closed':True}))
'''
child=subprocess.Popen(['/usr/bin/python3','-B','-c',source,str(ready),str(stopping),str(sealed)])
s=Session({},state,run,os.getpid());s.children['bridge']=child;s.phase='draining'
deadline=time.monotonic()+5
while not ready.exists() and time.monotonic()<deadline:time.sleep(.02)
assert ready.exists()
start=time.monotonic();s.close_child('bridge',None);duration=time.monotonic()-start
result={'elapsed_s':duration,'child_exit':child.returncode,'errors':s.errors,'sealed':json.loads(sealed.read_text()),'scope':'75-second synthetic child drain with real SIGINT; no camera/SDK or robot control'}
assert duration>=75 and child.returncode==0 and not s.errors
(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
