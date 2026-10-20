from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
for folder in ['repair152_build','repair152_reference_fixture','repair155_acceptance','repair154_regression','repair155_package']:
    assert json.loads((r/folder/'result.json').read_text())['phase']=='completed',folder
assert json.loads((r/'repair155_deployment/deployment_receipt.json').read_text())['status']=='deployed'
files={}
def add(name,p):
    assert p.is_file(),str(p)
    data=p.read_bytes();assert len(data)<6*2**20
    files[name]=data
for folder in ['repair120_replay','repair121_storage','repair123_trim','repair123_storage',
    'repair124_build','repair125_replay','repair127_build','repair128_replay',
    'repair130_build','repair131_replay','repair134_build','repair135_replay',
    'repair136_regression','repair138_probe','repair138_rawpose_probe','repair138_leveled_probe',
    'repair139_build','repair140_replay','repair143_probe','repair144_build',
    'repair147_reference','repair147_reference_verified','repair152_reference_fixture',
    'repair147_build','repair148_replay','repair149_regression','repair151_probe',
    'repair152_build','repair153_replay','repair154_regression','repair155_acceptance','repair155_package','repair155_display','repair155_display_rgb','repair155_display_rgb2']:
    add('receipts/'+folder+'.json',r/folder/'result.json')
add('receipts/repair134_completed_tests.json',r/'repair134_build/tests_result.json')
for name in ['metric_test','gravity_test','test_visual','test_prepared','test_lifecycle','test_tag']:
    add('native/'+name+'.log',r/'repair152_build'/(name+'.log'))
for name in ['recovery_geometry.inc','visual_recovery.inc']:
    add('source/'+name,r/'repair152_build/core'/name)
add('source/DeferredMappingSession.h',r/'repair130_build/worker/DeferredMappingSession.h')
for name in ['metric152_test.cpp','gravity152_test.cpp']:
    add('native/'+name,r/'repair152_input'/name)
add('native/test_visual_reference152.cpp',r/'test_visual_reference152.cpp')
add('native/reference_fallback.log',r/'repair152_reference_fixture/fallback_test.log')
add('diagnostic/alternative_references.log',r/'repair143_probe/probe.log')
for folder in ['repair135_replay','repair153_replay','repair154_regression']:
    p=r/folder/'field'
    for name in ['mapping/status.json','save.json','status.json']:
        add('runs/'+folder+'/'+name,p/name)
    for name in ['core.log','frames.csv']:
        add('runs/'+folder+'/'+name,p/'mapping'/name)
for name in ['main_cloud.ply','tags.json']:
    add('runs/repair153_replay/'+name,r/'repair153_replay/field/mapping'/name)
for name in ['entry.json','start_rtabmap_continuous.sh','run_continuous_mapping_with_light.sh']:
    add('package/'+name,r/'mapping_round6_candidate155_recovery'/name)
for name in ['deployment_receipt.json','before_dispatcher.sh','candidate_dispatcher.sh']:
    add('deployment/'+name,r/'repair155_deployment'/name)
add('display/rviz.png',r/'repair155_display/rviz.png')
add('display/rviz_saved_rgb.png',r/'repair155_display_rgb2/rviz.png')
add('display/recorded_rgb.log',r/'repair155_display_rgb2/recorded_rgb.log')
pins={name:hashlib.sha256(data).hexdigest() for name,data in files.items()}
files['manifest.json']=(json.dumps({'schema':'wbrov.repair155-evidence.v1','files':pins,
    'scope':'saved-data diagnostics, selected source and file-only deployment; no field acceptance, no raw videos or databases'},indent=2)+'\n').encode()
assert sum(map(len,files.values()))<20*2**20
out=r/'repair155_evidence.zip'
with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as z:
    for name,data in sorted(files.items()):z.writestr(name,data)
print(json.dumps({'path':str(out),'sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'bytes':out.stat().st_size,'files':len(files)}))

