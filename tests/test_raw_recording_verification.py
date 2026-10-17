"""Post-stop completeness must fail closed independently of recorder shutdown."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

path = Path(__file__).resolve().parents[1]/'tools/verify_raw_recording.py'
spec = importlib.util.spec_from_file_location('verify_raw_recording', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RecordingVerification(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ('stereo.svo2', 'capture.jsonl', 'init.json', 'runtime.json',
                     'recording.json', 'water_calibration.yml', 'camera_imu_extrinsics.json'):
            (self.root/name).write_text('{}')
        (self.root/'sdk_version.txt').write_text('5.0.0\n')
        self.status = dict(state='closed', clean_shutdown=True, write_errors=0,
                           grab_errors=0, sdk_frames_ingested=10, first_source_ns=100,
                           last_source_ns=1000, unique_computed_stamps=10)
        self.decoded = dict(frames=10, first_source_ns=100, last_source_ns=1000)
        self.save_status()

    def save_status(self):
        (self.root/'status.json').write_text(json.dumps(self.status))

    def invoke(self, version='5.0.0', mutate=False):
        def run(command, **kwargs):
            if '--sdk-version' in command:
                return subprocess.CompletedProcess(command, 0, version+'\n')
            if mutate:
                (self.root/'water_calibration.yml').write_text('changed')
            return subprocess.CompletedProcess(command, 0, json.dumps(self.decoded))
        with patch.object(module.subprocess, 'run', side_effect=run):
            return module.verify(self.root, Path('fake_verifier'))

    def test_closed_file_with_matching_coverage(self):
        result = self.invoke()
        self.assertTrue(result['complete'])
        self.assertFalse(result['sensor_capture_losslessness_proven'])
        self.assertEqual(len(result['sha256']), 9)

    def test_unclean_and_capture_errors_are_partial(self):
        for key, value in [('state', 'recording'), ('clean_shutdown', False),
                           ('write_errors', 1), ('grab_errors', 1), ('sdk_frames_ingested', 0),
                           ('unique_computed_stamps', 11)]:
            with self.subTest(key=key):
                old = self.status[key]
                self.status[key] = value
                self.save_status()
                with self.assertRaises(ValueError):
                    self.invoke()
                self.status[key] = old

    def test_index_and_time_range_must_cover_ingestion(self):
        for key, value in [('frames', 9), ('frames', 11), ('first_source_ns', 101), ('last_source_ns', 999)]:
            with self.subTest(key=key):
                old = self.decoded[key]
                self.decoded[key] = value
                with self.assertRaises(ValueError):
                    self.invoke()
                self.decoded[key] = old

    def test_transient_sdk_write_status_requires_complete_final_coverage(self):
        self.status.update(status_false_samples=22, duplicate_grabs=19)
        self.save_status()
        self.assertTrue(self.invoke()['complete'])
        self.decoded['frames'] = 9
        with self.assertRaises(ValueError):
            self.invoke()

    def test_sdk_mismatch_and_changed_calibration_rejected(self):
        with self.assertRaises(ValueError):
            self.invoke(version='different')
        with self.assertRaises(ValueError):
            self.invoke(mutate=True)

    def test_replay_contract_rejects_mismatched_planned_configuration(self):
        (self.root/'verification.json').write_text(json.dumps(self.invoke()))
        planned = self.root/'planned_init.json'
        planned.write_bytes((self.root/'init.json').read_bytes())
        with patch.object(module.subprocess, 'run', return_value=subprocess.CompletedProcess([],0,'5.0.0\n')):
            arguments = (self.root, Path('fake_verifier'), self.root/'water_calibration.yml',
                         planned, self.root/'runtime.json')
            self.assertTrue(module.check_replay_contract(*arguments)['compatible_inputs_verified'])
            planned.write_text('{"depth_mode":"different"}')
            with self.assertRaises(ValueError):
                module.check_replay_contract(*arguments)


if __name__ == '__main__':
    unittest.main()
