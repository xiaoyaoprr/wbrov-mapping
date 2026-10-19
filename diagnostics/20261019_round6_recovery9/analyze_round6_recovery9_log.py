from pathlib import Path
import re,json,csv,collections,hashlib
w=Path(__file__).parent;repo=Path('C:/Users/41479/Documents/Codex/2026-10-15/new-chat/outputs/wbrov-mapping')
log=w/'round6_v5_core_recovery9.log';lines=log.read_text(errors='replace').splitlines()
candidates=[];same=[]
for index,line in enumerate(lines,1):
 m=re.search(r'Guarded local-submap candidate current=(\d+) old=(\d+) retained=(\d+): (.*)',line)
 if m:candidates.append({'line':index,'current':int(m[1]),'old':int(m[2]),'retained':int(m[3]),'reason':m[4]})
 if 'Rejected same-epoch visual loop' in line:same.append(index)
rows=list(csv.DictReader((repo/'diagnostics/20261019_round6_candidate6/field_v5/run/processing/mapping/frames.csv').open()))
stale=[row for row in rows if 'source-stale' in row['reason']]
groups=collections.Counter()
for row in rows:
 reason=row['reason']
 if 'source-stale' in reason:groups['stale_candidate']+=1
 elif 'three independent trusted-map candidates' in reason:groups['insufficient_candidates']+=1
 elif 'inconsistent main-map alignments' in reason:groups['three_anchor_disagreement']+=1
 elif 'raw bidirectional RGB-D geometry failed' in reason:groups['raw_geometry_rejection']+=1
out={'scope':'existing closed v5 field evidence; repeated frame reasons are durations, not independent failures',
 'core_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'cross_map_candidates':len(candidates),
 'last_cross_map_candidate':candidates[-1],'later_same_epoch_rejection_count':sum(x>candidates[-1]['line'] for x in same),
 'frame_reason_occurrences':dict(groups),'stale_source_duration_s':(int(stale[-1]['source_ns'])-int(stale[0]['source_ns']))/1e9 if stale else 0,
 'candidate_events':candidates,
 'interpretation':'The native best-loop route stopped proposing trusted-main candidates after node 392 while local same-epoch loop hypotheses continued. A separate trusted-main search avoids depending on that winner; raw geometry/observation and graph authorization must stay unchanged.'}
(w/'round6_recovery9_diagnosis.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in out.items() if k!='candidate_events'},ensure_ascii=False,indent=2))
