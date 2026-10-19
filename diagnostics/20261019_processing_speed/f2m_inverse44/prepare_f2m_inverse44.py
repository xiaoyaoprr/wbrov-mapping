"""Prepare one isolated exact-arithmetic candidate; no build, replay or adoption."""
from pathlib import Path
import difflib
import hashlib
import json
import zipfile

work = Path(__file__).resolve().parent
repo = work.parents[2] / 'new-chat/outputs/wbrov-mapping'
source_path = repo / 'src/rtabmap_core/corelib/src/odometry/OdometryF2M.cpp'
source = source_path.read_text(encoding='utf-8')
stage = work / 'f2m_inverse44'
old = 'lastFrameModels[cameraIndex].localTransform().inverse()'
marker = '\tUDEBUG("lastFrameModels=%ld", lastFrameModels.size());\n'
assert source.count(old) == 4 and source.count(marker) == 1
cache = '''	// Camera models are complete above and read-only for this computeTransform.
	// Compute each requested inverse once, using the original Transform::inverse.
	// Keep the cache frame-local and lazy: an unused/singular model is never inverted.
	std::vector<Transform> frameCameraInverses;
	std::vector<unsigned char> frameCameraInverseReady;
	const auto frameCameraInverse = [&](int cameraIndex) -> const Transform & {
		UASSERT(cameraIndex >= 0 && cameraIndex < (int)lastFrameModels.size());
		if(frameCameraInverseReady.empty())
		{
			frameCameraInverses.resize(lastFrameModels.size());
			frameCameraInverseReady.resize(lastFrameModels.size(), 0);
		}
		if(!frameCameraInverseReady[cameraIndex])
		{
			frameCameraInverses[cameraIndex] = lastFrameModels[cameraIndex].localTransform().inverse();
			frameCameraInverseReady[cameraIndex] = 1;
		}
		return frameCameraInverses[cameraIndex];
	};
'''
candidate = source.replace(old, 'frameCameraInverse(cameraIndex)').replace(marker, marker + '\n' + cache)
# Exact reversibility restricts the draft to the cache and four call sites.
reverted = candidate.replace(marker + '\n' + cache, marker).replace('frameCameraInverse(cameraIndex)', old)
assert reverted == source
stage.mkdir()
files = {'OdometryF2M.baseline.cpp': source.encode(), 'OdometryF2M.candidate.cpp': candidate.encode(),
         'frame_inverse_cache.inc': cache.encode(),
         'candidate.patch': ''.join(difflib.unified_diff(source.splitlines(True), candidate.splitlines(True),
             fromfile='a/corelib/src/odometry/OdometryF2M.cpp', tofile='b/corelib/src/odometry/OdometryF2M.cpp')).encode(),
         'native_probe.cpp': (work / 'f2m_inverse44_probe.cpp').read_bytes(),
         'CMakeLists.txt': b'''cmake_minimum_required(VERSION 3.16)
project(wbrov_inverse44_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(RTABMap REQUIRED)
add_executable(native_probe native_probe.cpp)
target_include_directories(native_probe PRIVATE ${RTABMap_INCLUDE_DIRS})
target_link_libraries(native_probe rtabmap::core)
if(MSVC)
  target_compile_options(native_probe PRIVATE /UNDEBUG)
else()
  target_compile_options(native_probe PRIVATE -UNDEBUG)
endif()
'''}
manifest = {'schema': 'wbrov.isolated-source-proposal.v1', 'experiment': 'f2m_inverse44',
            'adopted': False, 'compiled': False, 'native_tests_run': False, 'replays_run': False,
            'performance_claim': None, 'scope': 'same inverse arithmetic and transformPoint, frame-local lazy reuse',
            'replaced_calls': 4, 'source_reversibility_verified': True,
            'baseline_repository_raw_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
            'members': {name: {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
                        for name, data in files.items()}}
files['manifest.json'] = (json.dumps(manifest, indent=2) + '\n').encode()
for name, data in files.items(): (stage / name).write_bytes(data)
archive = work / 'f2m_inverse44_source.zip'
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as z:
    for name, data in files.items(): z.writestr(name, data)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name, item in manifest['members'].items():
        assert hashlib.sha256(z.read(name)).hexdigest() == item['sha256']
assert hashlib.sha256(source_path.read_bytes()).hexdigest() == manifest['baseline_repository_raw_sha256']
receipt = {'source_archive': archive.name, 'sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
           'bytes': archive.stat().st_size, 'crc_members_verified': True,
           'source_unchanged': True, 'compiled': False, 'adopted': False}
(stage / 'archive_verification.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps(receipt))
