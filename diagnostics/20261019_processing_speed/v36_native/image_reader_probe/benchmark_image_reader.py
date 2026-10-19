from pathlib import Path
import hashlib,json,struct,time,statistics
from test_image_reader import baseline_tags,candidate_tags

r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
source=r/'compressed_v35/motion_spool_v2'
segments=[(p,json.loads(p.with_suffix('.json').read_text())['frames']) for p in sorted(source.glob('segment-????????.frames'))]
assert len(segments)==61 and sum(n for _,n in segments)==1581
pins={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p,_ in segments}
report={'scope':'map RGB reader for Tag detection only; every frame is fully ZSTD validated; no mapping-worker change','rounds':[]}

def decode(module):
    rows=[];began=time.perf_counter()
    for p,n in segments:
        for stamp,(w,h,K,rgb) in module.map_pixels(p,n):
            rows.append((stamp,w,h,K,rgb))
    elapsed=time.perf_counter()-began
    h=hashlib.sha256()
    for stamp,w,height,K,rgb in rows:h.update(struct.pack('<QII4d',stamp,w,height,*K));h.update(rgb)
    return {'seconds':elapsed,'images':len(rows),'sha256':h.hexdigest()}

expected=None
for turn in range(6):
    result={'round':turn}
    order=[('baseline',baseline_tags),('candidate',candidate_tags)]
    if turn%2:order.reverse()
    for name,module in order:
        result[name]=decode(module)
        if expected is None:expected=result[name]['sha256']
        assert result[name]['sha256']==expected
    report['rounds'].append(result);print(json.dumps(result),flush=True)
for name in ('baseline','candidate'):report[name+'_median_seconds']=statistics.median(x[name]['seconds'] for x in report['rounds'][1:])
report['reduction_fraction']=1-report['candidate_median_seconds']/report['baseline_median_seconds']

observer=candidate_tags.TagObserver(r/'mapping_usable_v3/runtime/libexec/wbrov_corner_recovery')
observations=0;frames=0
for p,n in segments:
    number=int(p.stem.split('-')[1]);descriptor=json.loads((r/'offline_v35/short_recovery'/('input-%08d.json'%number)).read_text())
    actual=observer.observe(p,n);expected=descriptor['tag_observations']
    assert actual==expected,(p.name,len(actual),len(expected))
    frames+=n;observations+=len(actual)
assert frames==1581 and observations==235
assert pins=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p,_ in segments}
report.update(frames=frames,observations=observations,all_observations_equal=True,source_pins_unchanged=True)
Path('benchmark.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
