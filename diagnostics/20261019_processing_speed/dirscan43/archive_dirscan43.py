from pathlib import Path
import hashlib
import json
import zipfile

work = Path(__file__).resolve().parent
stage = work / 'dirscan43'
repo = work.parents[2] / 'new-chat' / 'outputs' / 'wbrov-mapping'
out = repo / 'diagnostics/20261019_processing_speed/dirscan43'
result = json.loads((stage / 'result.json').read_text())
assert result['state'] == 'local_probe_complete' and result['adopted'] is False
assert result['tests']['run'] == 20 and len(result['tests']['skipped']) == 2
manifest = json.loads((stage / 'source_manifest.json').read_text())
assert hashlib.sha256((repo / 'tools/deferred_mapping.py').read_bytes()).hexdigest() == manifest['repository_source_sha256']
files = {name: (stage / name).read_bytes() for name in
         ('baseline.py', 'candidate.py', 'source_manifest.json', 'result.json', 'tests.log')}
for name in ('prepare_dirscan43.py', 'probe_dirscan43.py', 'archive_dirscan43.py'):
    files[name] = (work / name).read_bytes()
files['existing_queue_tests.py'] = (repo / 'tests/test_deferred_mapping.py').read_bytes()
inventory = {name: {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
             for name, data in files.items()}
files['archive_manifest.json'] = (json.dumps(inventory, indent=2) + '\n').encode()
archive = work / 'dirscan43_local_evidence.zip'
with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED) as z:
    for name, data in files.items(): z.writestr(name, data)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name, item in inventory.items():
        data = z.read(name)
        assert len(data) == item['bytes'] and hashlib.sha256(data).hexdigest() == item['sha256']
out.mkdir()
for name, data in files.items(): (out / name).write_bytes(data)
verification = {'archive': archive.name, 'bytes': archive.stat().st_size,
                'sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
                'crc_and_members_verified': True, 'repository_source_unchanged': True,
                'adopted': False, 'native_jetson_validation': False}
(out / 'archive_verification.json').write_text(json.dumps(verification, indent=2) + '\n')
print(json.dumps(verification))
