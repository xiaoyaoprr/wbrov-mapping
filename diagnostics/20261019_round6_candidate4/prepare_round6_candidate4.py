from pathlib import Path
import zipfile, hashlib
w=Path(__file__).resolve().parent
repo=Path('C:/Users/41479/Documents/Codex/2026-10-15/new-chat/outputs/wbrov-mapping')
archive=w/'round6_candidate4_source.zip'
assert not archive.exists()
with zipfile.ZipFile(w/'round6_candidate3_source.zip') as old, zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as out:
    assert len(old.namelist())==len(set(old.namelist()))
    for name in old.namelist():
        group,relative=name.split('/',1)
        source=repo/'src/zed_rgbd_publisher'/relative
        current=group in ('camera','worker') and source.is_file()
        out.writestr(name,source.read_bytes() if current else old.read(name))
(w/'build_round6_candidate4.py').write_text((w/'build_round6_candidate3.py').read_text().replace('round6_candidate3','round6_candidate4'))
start=(w/'start_round6_candidate3.py').read_text().replace('round6_candidate3','round6_candidate4')
start=start.replace("assert json.loads((r/'round6_validation2/result.json').read_text())['phase'] in ['completed','failed']", "assert json.loads((r/'round6_field_notags_v2/result.json').read_text())['phase'] in ['completed','failed']")
(w/'start_round6_candidate4.py').write_text(start)
package=(w/'package_round6_candidate3_v2.py').read_text().replace('mapping_round6_candidate3_notags_v2','mapping_round6_candidate4_notags').replace('round6_candidate3','round6_candidate4').replace('package_round6_candidate4_v2_result','package_round6_candidate4_result')
(w/'package_round6_candidate4.py').write_text(package)
print(hashlib.sha256(archive.read_bytes()).hexdigest())
