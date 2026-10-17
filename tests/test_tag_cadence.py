import importlib.util
from pathlib import Path
import sys
import unittest

p = Path(__file__).resolve().parents[1] / 'src/apriltag/detector/tag_pose_guard.py'
spec = importlib.util.spec_from_file_location('tag_pose_guard', p)
g = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = g
spec.loader.exec_module(g)


class TagCadenceTests(unittest.TestCase):
    def make(self, period):
        return g.TemporalPoseGuard(g.GuardConfig(max_gap_s=g.cadence_gap_s(period)))

    def observe(self, guard, stamp):
        return guard.update(8, stamp, (0., 0., .5), (0., 0., 0., 1.))

    def test_configured_one_and_six_hz_confirm_three_distinct_frames(self):
        for period in (1., 1./6.):
            guard = self.make(period)
            results = [self.observe(guard, 10.+i*period)['accepted'] for i in range(6)]
            self.assertEqual(results, [False, False, True, True, True, True])

    def test_missing_detector_frame_restarts(self):
        guard = self.make(1.)
        self.observe(guard, 10.)
        self.observe(guard, 11.)
        guard.finish_frame(12., [])
        self.assertFalse(self.observe(guard, 13.)['accepted'])
        self.assertFalse(self.observe(guard, 14.)['accepted'])
        self.assertTrue(self.observe(guard, 15.)['accepted'])

    def test_missing_input_and_duplicates_cannot_confirm(self):
        guard = self.make(1.)
        self.observe(guard, 10.)
        self.observe(guard, 11.)
        self.assertFalse(self.observe(guard, 13.)['accepted'])
        self.assertFalse(self.observe(guard, 13.)['accepted'])
        self.assertFalse(self.observe(guard, 14.)['accepted'])

    def test_bad_cadence_rejected(self):
        for period in (-1, 1.01, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                g.cadence_gap_s(period)


if __name__ == '__main__':
    unittest.main()
