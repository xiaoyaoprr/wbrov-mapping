"""Derive a bounded local source/timing review from already imported receipts."""
from pathlib import Path
import hashlib
import json

work = Path(__file__).resolve().parent
repo = work.parents[2] / 'new-chat/outputs/wbrov-mapping'
diag = repo / 'diagnostics/20261019_processing_speed'
out = diag / 'f2m_inverse44'
out.mkdir()
sha = lambda b: hashlib.sha256(b).hexdigest()
sources = ['src/rtabmap_core/corelib/src/odometry/OdometryF2M.cpp',
           'src/rtabmap_core/corelib/src/Transform.cpp',
           'src/rtabmap_core/corelib/src/util3d_transforms.cpp',
           'src/rtabmap_core/corelib/src/CameraModel.cpp',
           'src/rtabmap_core/corelib/src/optimizer/OptimizerG2O.cpp',
           'src/rtabmap_core/corelib/src/Odometry.cpp',
           'src/deferred_mapping/DeferredMappingSession.h']
review = {'schema': 'wbrov.local-source-timing-review.v1', 'remote_verified_this_turn': False,
          'native_tests_run': False, 'candidate_adopted': False, 'performance_claim': None,
          'source_sha256': {p: sha((repo / p).read_bytes()) for p in sources}, 'runs': []}
for name in ('core6/full_compressed/status.json', 'status40/full_compressed/status.json',
             'v39/v39_full_compressed/status.json', 'v39/v38_full_compressed/status.json'):
    path = diag / name
    doc = json.loads(path.read_text()); mapping = doc.get('mapping', doc)
    assert mapping['database_closed'] and mapping['frames'] == 10746
    timing = {k: mapping[k] for k in ('frames', 'odometry_compute_ms', 'odometry_estimation_ms',
                                     'odometry_bundle_ms', 'mapping_compute_ms')}
    timing['reported_bundle_fraction_of_odom'] = mapping['odometry_bundle_ms'] / mapping['odometry_compute_ms']
    timing['source'] = name; timing['source_sha256'] = sha(path.read_bytes())
    review['runs'].append(timing)
review['interpretation'] = [
    'Odometry::timeEstimation includes computeTransform and therefore local BA; do not add bundle and estimation totals.',
    'Deferred frontend and map consumer can overlap; do not sum their CPU-stage totals as wall-clock duration.',
    'OdometryF2M::bundleTime surrounds optimizeBA only, excludes per-feature camera inversions in this candidate.',
    'bundleTime is assigned, not accumulated, per successful-entry attempt; it is a reported substage, not a complete exclusive timing partition.',
    'The four repeat-inverse sites are a source-level opportunity, not a measured hotspot; no percentage speedup is inferred.',
    'Keep optimizer iterations, references, robust kernels, inliers, depth, density, all frame processing and save checks unchanged.'
]
review['source_locations'] = {
    'camera_models_populated_before_cache': 'OdometryF2M.cpp:338-361',
    'inverse_calls': [549, 935, 1004, 1477],
    'bundle_timer_scope': 'OdometryF2M.cpp:565-567',
    'general_inverse_with_invertibility_check': 'Transform.cpp:198-212',
    'same_point_multiplication_order': 'util3d_transforms.cpp:211-219',
    'null_model_construction_rejected': 'CameraModel.cpp:84,129',
    'outer_estimation_timer': 'Odometry.cpp:832'
}
review['required_next_checks'] = [
    'Wait for legitimate remote permission, inspect existing combo42 receipts before any new heavy work.',
    'Build isolated f2m_inverse44_v2 probe against actual candidate RTABMap; ASan/UBSan and Release.',
    'Require bitwise equality of 90000 inverse/point observations and failure for both singular modes.',
    'Run fixed six alternating microbenchmark rounds once and preserve all outcomes, including slower results.',
    'Only if component result is useful, build isolated Core overlay and run all four fixed replays with timestamp, recovery, DB/save and own-cloud audits.',
    'No production/source adoption or 8/GitHub update based on static review or the component benchmark alone.'
]
v2 = work / 'f2m_inverse44_v2'
manifest = json.loads((v2 / 'manifest.json').read_text())
assert review['source_sha256'][sources[0]] == manifest['baseline_repository_raw_sha256']
for name in ('f2m_inverse44_source.zip', 'f2m_inverse44_v2_source.zip'):
    data = (work / name).read_bytes(); (out / name).write_bytes(data)
for name in ('prepare_f2m_inverse44.py', 'revise_inverse44_probe_fixture.py', 'analyze_odometry44.py'):
    (out / name).write_bytes((work / name).read_bytes())
(out / 'review.json').write_text(json.dumps(review, indent=2) + '\n')
(out / 'candidate_manifest.json').write_bytes((v2 / 'manifest.json').read_bytes())
(out / 'archive_verification.json').write_bytes((v2 / 'archive_verification.json').read_bytes())
print(json.dumps({'runs': len(review['runs']), 'native_tests_run': False, 'source_unchanged': True,
                  'odometry_ms': [r['odometry_compute_ms'] for r in review['runs']],
                  'bundle_ms': [r['odometry_bundle_ms'] for r in review['runs']]}))
