"""Contract comparison then a fixed local benchmark, never a replay or acceptance."""
from pathlib import Path
import contextlib
import hashlib
import importlib.util
import io
import json
import os
import platform
import statistics
import sys
import tempfile
import time
import traceback
import unittest

WORK = Path(__file__).resolve().parent
STAGE = WORK / 'dirscan43'
REPO = WORK.parents[2] / 'new-chat' / 'outputs' / 'wbrov-mapping'
sys.path.insert(0, str(REPO / 'tools'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


baseline = load('dirscan43_baseline', STAGE / 'baseline.py')
candidate = load('deferred_mapping', STAGE / 'candidate.py')


class InventoryContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=STAGE)
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def status(self, count=3, schema='wbrov.rgbd-spool.v1', state='recording'):
        (self.root / 'status.json').write_text(json.dumps(dict(schema=schema, state=state, sealed_segments=count)))

    def seal(self, i, directory=False):
        p = self.root / (f'segment-{i:08d}' + ('' if directory else '.json'))
        if directory:
            p.mkdir()
        else:
            p.write_text('{}')
        return p

    def outcome(self, module):
        try:
            return ('ok', module.closed_inputs(self.root))
        except Exception as exc:
            return (type(exc).__name__, str(exc))

    def same(self, accepted=None):
        b, c = self.outcome(baseline), self.outcome(candidate)
        self.assertEqual(b, c)
        if accepted is not None:
            self.assertEqual(c[0] == 'ok', accepted)
        return c

    def test_absent_empty_legacy_and_open_status(self):
        self.same(True)
        self.status(0); self.same(True)
        for state in ('recording', 'closed'):
            self.status(schema='wbrov.raw-recording.v1', state=state)
            result = self.same(True)[1]
            self.assertEqual(result[0], [self.root] if state == 'closed' else [])
            self.assertEqual(result[1], state == 'closed')

    def test_count_schema_writer_and_json_fail_closed(self):
        for count in (-1, True, 1.5, '3', None):
            self.status(count); self.same(False)
        self.status(0, state='failed'); self.same(False)
        self.status(0, schema='unknown'); self.same(False)
        (self.root / 'status.json').write_text('{'); self.same(False)

    def test_order_extras_open_tail_and_closed(self):
        self.status()
        for n in (2, 0, 1): self.seal(n)
        self.seal(3)  # Unsealed tail must not be exposed.
        (self.root / 'unrelated').mkdir()
        paths, closed, _ = self.same(True)[1]
        self.assertEqual([p.name for p in paths], [f'segment-{n:08d}.json' for n in range(3)])
        self.assertFalse(closed)
        self.status(state='closed'); self.assertTrue(self.same(True)[1][1])

    def test_hole_wrong_type_and_replacement_between_calls(self):
        self.status()
        self.seal(0); self.seal(2)
        self.same(False)
        p = self.seal(1); self.same(True)
        p.unlink(); p.mkdir(); self.same(False)
        p.rmdir(); p.write_text('{}'); self.same(True)
        p.unlink(); self.same(False)

    def test_raw_directory_format_and_new_seal_between_calls(self):
        self.status(1, schema='wbrov.raw-segments.v1'); self.seal(0, True)
        self.same(True)
        self.status(2, schema='wbrov.raw-segments.v1'); self.same(False)
        p = self.seal(1, True); self.same(True)
        p.rmdir(); p.write_text('{}'); self.same(False)

    def test_actual_file_and_directory_symlinks_rejected(self):
        target_file = self.root / 'actual.json'; target_file.write_text('{}')
        target_dir = self.root / 'actual-dir'; target_dir.mkdir()
        link = self.root / 'segment-00000000.json'
        try:
            link.symlink_to(target_file)
        except OSError as exc:
            if os.name == 'nt' and getattr(exc, 'winerror', None) == 1314:
                self.skipTest('Windows symlink privilege unavailable; Linux validation pending')
            raise
        self.status(1); self.same(False)
        link.unlink(); link.symlink_to(self.root / 'missing.json'); self.same(False)
        link.unlink()
        self.status(1, schema='wbrov.raw-segments.v1')
        link = self.root / 'segment-00000000'; link.symlink_to(target_dir, target_is_directory=True)
        self.same(False)
        link.unlink()  # Explicit unlink, never recurse through a test symlink.

    @unittest.skipUnless(hasattr(os, 'mkfifo'), 'POSIX FIFO type check requires Linux')
    def test_fifo_rejected(self):
        self.status(1); os.mkfifo(self.root / 'segment-00000000.json'); self.same(False)


def main():
    manifest = json.loads((STAGE / 'source_manifest.json').read_text())
    result = {'schema': 'wbrov.local-inventory-probe.v1', 'platform': platform.platform(),
              'python': sys.version, 'native_jetson_validation': False, 'adopted': False,
              'source_manifest': manifest, 'state': 'running'}
    def save():
        (STAGE / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    if (STAGE / 'result.json').exists():
        raise RuntimeError('preserve previous results; benchmark may not be silently repeated')
    save()
    try:
        original = load('dirscan43_queue_tests', REPO / 'tests/test_deferred_mapping.py')
        suite = unittest.TestSuite((unittest.defaultTestLoader.loadTestsFromModule(original),
                                  unittest.defaultTestLoader.loadTestsFromTestCase(InventoryContract)))
        log = io.StringIO()
        tests = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
        (STAGE / 'tests.log').write_text(log.getvalue())
        result['tests'] = dict(run=tests.testsRun, failures=len(tests.failures), errors=len(tests.errors),
                               skipped=[reason for _, reason in tests.skipped])
        assert tests.wasSuccessful(), log.getvalue()
        print(json.dumps(result['tests']), flush=True)
        rounds = []
        with tempfile.TemporaryDirectory(dir=STAGE) as td:
            root = Path(td)
            status = dict(schema='wbrov.rgbd-spool.v1', state='closed', sealed_segments=375)
            (root / 'status.json').write_text(json.dumps(status))
            for n in range(375):
                (root / f'segment-{n:08d}.json').write_text('{}')
                (root / f'segment-{n:08d}.frames').touch()
            for name in ('metadata.json', 'init.json', 'runtime.json', 'water_calibration.yml',
                         'sdk_version.txt', 'camera_imu_extrinsics.json'):
                (root / name).write_text('{}')
            expected = baseline.closed_inputs(root)
            for index in range(6):
                row = {'round': index, 'scans_per_variant': 120}
                for name, module in ((('baseline', baseline), ('candidate', candidate)) if index % 2 == 0 else
                                     (('candidate', candidate), ('baseline', baseline))):
                    start = time.perf_counter_ns()
                    for _ in range(120):
                        assert module.closed_inputs(root) == expected
                    row[name + '_ms'] = (time.perf_counter_ns() - start) / 1e6
                rounds.append(row)
                result['rounds'] = rounds; save()
                print(json.dumps(row), flush=True)
        result['median_excluding_round0_ms'] = {name: statistics.median(r[name + '_ms'] for r in rounds[1:])
                                                for name in ('baseline', 'candidate')}
        repo_hash = hashlib.sha256((REPO / 'tools/deferred_mapping.py').read_bytes()).hexdigest()
        assert repo_hash == manifest['repository_source_sha256']
        result['repository_source_unchanged'] = True
        result['state'] = 'local_probe_complete'
        result['limits'] = ['Windows synthetic directory inventory only, not recorded pixels or a replay',
                            'No whole-pipeline speed claim; Jetson/Linux and mutation checks still required',
                            'Candidate is isolated and not applied to the repository or production']
        save(); print(json.dumps(result['median_excluding_round0_ms']), flush=True)
    except BaseException:
        result['state'] = 'failed'; result['failure'] = traceback.format_exc(); save()
        raise


if __name__ == '__main__': main()
