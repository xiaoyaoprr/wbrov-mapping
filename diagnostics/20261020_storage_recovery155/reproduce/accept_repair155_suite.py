"""Combine distinct saved-data coverage without modifying any original verdict."""
from pathlib import Path
import csv,hashlib,json
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
out=r/'repair155_acceptance'
assert not out.exists()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
receipt_paths=[r/name/'result.json' for name in
    ['repair152_build','repair152_reference_fixture','repair153_replay','repair154_regression']]
pins={str(p):sha(p) for p in receipt_paths}
build,fixture,long,short=[json.loads(p.read_text()) for p in receipt_paths]
assert build['phase']==fixture['phase']=='completed'
assert short['phase']=='completed' and short['recovery_continuation_passed']
# Preserve this standalone test failure: the clip ends 0.917 s after recovery
# and contains no new keyframe afterwards. This is not a positive movement test.
assert long['phase']=='validation_failed' and not long['recovery_continuation_passed']
for result,count in [(long,9771),(short,1581)]:
    assert result['exit']==0 and result['frames']==count
    assert result['all_input_stamps_equal'] and result['save_verified'] and result['database_check']=='ok'
    assert result['final_continuity']=='NORMAL' and result['recoveries']==1
    assert result['core_sha256']==build['core_sha256']
    assert result['worker_sha256']=='b4e7e925a9b41fe788e5b63684fcf110fd882109d719a8e04c81a9ff1838f561'
    assert result['prior_storage_evidence']['passed']
    assert result['prior_storage_evidence']['summary']['byte_exact']
    assert result['prior_storage_evidence']['max_segment_s']<.3
def coverage(folder,result):
    base=r/folder/'field'
    rows=list(csv.DictReader((base/'mapping/frames.csv').open()))
    event=next(i for i,row in enumerate(rows) if int(row['recoveries'])==1)
    before,after=rows[event],rows[event+1:]
    assert after and all(row['main_write']=='1' and row['lost']=='0' and row['input_present']=='1' and
        row['usable']=='1' and row['covariance_valid']=='1' and row['cuts']==before['cuts'] for row in after)
    for name,pin in result['saved_sha256'].items():
        assert Path(name).name==name and sha(base/'mapping'/name)==pin
    return {'route':before['recovery_route'],'remaining_frames':len(after),
        'remaining_seconds':(int(rows[-1]['source_ns'])-int(before['source_ns']))/1e9,
        'main_nodes_at_recovery':int(before['main_nodes']),'final_main_nodes':int(rows[-1]['main_nodes']),
        'all_remaining_frames_qualified':True,'no_cut_after_recovery':True,
        'frames_sha256':sha(base/'mapping/frames.csv')}
a=coverage('repair153_replay',long);b=coverage('repair154_regression',short)
assert a['route']=='visual_keyframes' and a['remaining_frames']==22 and .9<a['remaining_seconds']<1
assert a['main_nodes_at_recovery']==a['final_main_nodes']
assert b['route']!='visual_keyframes' and b['remaining_seconds']>=15 and b['remaining_frames']>=300
assert b['final_main_nodes']-b['main_nodes_at_recovery']>=5
native=(r/'repair152_build/test_visual.log').read_text()
assert 'PASS native no-Tag raw RGB-D transaction' in native and 'FAIL ' not in native
assert all(sha(p)==pin for p,pin in pins.items())
report={'phase':'completed','scope':'combined saved-recording coverage; not new field acceptance',
    'core_sha256':build['core_sha256'],'worker_sha256':long['worker_sha256'],
    'long_recording':a,'independent_movement_recording':b,'receipt_sha256':pins,
    'original_long_standalone_verdict':'validation_failed: no post-recovery keyframe growth; preserved',
    'coverage_basis':'Long recording verifies visual recovery, 22 qualified remaining frames and durable save; independent recording verifies Tag recovery plus >15 seconds, >=5 added main nodes and durable save. Native no-Tag test additionally verifies post-commit graph extension and save/reopen. No recorded visual movement continuation is claimed.',
    'quality_gates_changed_for_acceptance':False,'runtime_parameters_changed':False,
    'field_acceptance':False,'hardware_health_permanently_resolved':False}
out.mkdir(mode=0o700);(out/'result.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
