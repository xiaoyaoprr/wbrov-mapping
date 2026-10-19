"""Preserve the first unbuilt draft; fix its singular-camera fixture before native use."""
from pathlib import Path
import hashlib
import json
import zipfile

work = Path(__file__).resolve().parent
before = work / 'f2m_inverse44'
after = work / 'f2m_inverse44_v2'
old = json.loads((before / 'manifest.json').read_text())
assert not old['compiled'] and not old['native_tests_run'] and not old['adopted']
probe = (before / 'native_probe.cpp').read_text()
assert probe.count('model(Transform())') == 2
helper = '''static Transform singularNonNull() {
    Transform t = Transform::getIdentity();
    t(2,2) = 0.0f; // Non-null, but determinant zero: CameraModel accepts it.
    assert(!t.isNull() && !t.isInvertible());
    return t;
}
'''
probe = probe.replace('static void equivalence() {', helper + 'static void equivalence() {')
probe = probe.replace('model(Transform())', 'model(singularNonNull())')
files = {name: (before / name).read_bytes() for name in old['members']}
files['native_probe.cpp'] = probe.encode()
manifest = dict(old)
manifest['experiment'] = 'f2m_inverse44_v2'
manifest['supersedes_unbuilt_probe'] = 'f2m_inverse44'
manifest['fixture_correction'] = 'CameraModel rejects a null Transform at construction, before inverse. Use a non-null singular transform to exercise lazy inverse and fail-closed behavior. Discovered by source review; neither version has been compiled or run.'
manifest['members'] = {name: {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)} for name, data in files.items()}
for name in ('OdometryF2M.baseline.cpp', 'OdometryF2M.candidate.cpp', 'frame_inverse_cache.inc', 'candidate.patch'):
    assert manifest['members'][name] == old['members'][name]
files['manifest.json'] = (json.dumps(manifest, indent=2) + '\n').encode()
after.mkdir()
for name, data in files.items(): (after / name).write_bytes(data)
archive = work / 'f2m_inverse44_v2_source.zip'
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as z:
    for name, data in files.items(): z.writestr(name, data)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name, item in manifest['members'].items():
        assert hashlib.sha256(z.read(name)).hexdigest() == item['sha256']
receipt = {'source_archive': archive.name, 'sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
           'bytes': archive.stat().st_size, 'crc_members_verified': True,
           'compiled': False, 'adopted': False}
(after / 'archive_verification.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps(receipt))
