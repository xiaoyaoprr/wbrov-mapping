"""Capacity estimate only; source gaps cannot be reconstructed by this model."""
from pathlib import Path
import csv,json,collections
here=Path(__file__).resolve().parent
old=here.parent/'20261019_round6_continuity/round6_field_notags_v2/run'
sizes=json.loads((here/'closed_segment_sizes.json').read_text())
rows=list(csv.DictReader((old/'processing/mapping/frames.csv').open()))
stamps=[int(x['source_ns']) for x in rows]
# Remove each measured >300ms gap from the synthetic arrival clock. This is
# optimistic and keeps the measured service costs unchanged, not a device model.
gaps=[(b,b-a) for a,b in zip(stamps,stamps[1:]) if b-a>300000000]
latency=[json.loads(x) for x in (old/'rgbd_spool/seal_latency.jsonl').read_text().splitlines()]
queue=collections.deque();finish=0.;peak_bytes=peak_segments=0
for s in latency:
    arrival=(s['last_source_ns']-stamps[0]-sum(delta for stamp,delta in gaps if stamp<=s['last_source_ns']))/1e9
    while queue and queue[0][0]<=arrival:queue.popleft()
    finish=max(arrival,finish)+s['total_ms']/1000
    queue.append((finish,sizes['segment-%08d.frames'%s['sequence']]))
    peak_bytes=max(peak_bytes,sum(x[1] for x in queue));peak_segments=max(peak_segments,len(queue))
print(json.dumps({'segments':len(latency),'service_seconds':sum(s['total_ms'] for s in latency)/1000,'modeled_peak_pending_bytes':peak_bytes,'modeled_peak_segments':peak_segments,'model_complete_seconds':finish},indent=2))
