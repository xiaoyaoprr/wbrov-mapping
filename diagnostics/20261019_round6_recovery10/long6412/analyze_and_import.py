from pathlib import Path
import json,hashlib,zipfile,csv,collections,shutil
w=Path(__file__).parent;repo=Path('C:/Users/41479/Documents/Codex/2026-10-15/new-chat/outputs/wbrov-mapping');out=repo/'diagnostics/20261019_round6_recovery10/long6412';out.mkdir()
archive=w/'round6_recovery10_long6412_evidence.zip';pin=json.loads((w/'collect_round6_recovery10_long6412_1.txt').read_text())['sha256'];assert hashlib.sha256(archive.read_bytes()).hexdigest()==pin
with zipfile.ZipFile(archive) as z:
 pins=json.loads(z.read('sha256.json'))
 for name,pin in pins.items():
  p=(out/name).resolve();assert p.is_relative_to(out.resolve());data=z.read(name);assert hashlib.sha256(data).hexdigest()==pin
  p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
 (out/'sha256.json').write_text(json.dumps(pins,indent=2))
shutil.copyfile(w/'validate_round6_recovery10_long6412.py',out/'validate.py')
result=json.loads((out/'replay/result.json').read_text());assert result['phase']=='completed';s=result['steps'][0]
rows=list(csv.DictReader((out/'replay/candidate_full/mapping/frames.csv').open()));stamps=[int(x['source_ns']) for x in rows];gaps=[(b-a)/1e9 for a,b in zip(stamps,stamps[1:])]
cut=0;causes=collections.Counter();events=[]
for index,row in enumerate(rows):
 delta=int(row['cuts'])-cut;cut=int(row['cuts'])
 if delta:
  reason=row['reason'].split(';')[0];causes[reason]+=delta;events.append({'frame':index+1,'source_ns':int(row['source_ns']),'increment':delta,'reason':reason,'lost':row['lost']})
report={k:s[k] for k in ['frames','seconds','source_seconds','save_verified','complete','stamp_sha256']}
report.update(final_state=s['final']['continuity_state'],recoveries=s['final']['recoveries'],main_nodes=s['final']['main_nodes'],cut_count=cut,
 max_source_gap_s=max(gaps),source_gaps_over_300ms=sum(x>.3 for x in gaps),lost_frames=sum(x['lost']=='1' for x in rows),
 invalid_main_writes=sum(x['main_write']=='1' and (x['lost']=='1' or x['input_present']=='0') for x in rows),cut_increments_by_reason=dict(causes),cut_events=events)
(out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='cut_events'},ensure_ascii=False,indent=2))
