import unittest
from replay_timing import timing_evidence


class ReplayTimingTests(unittest.TestCase):
    def test_good_replay_is_not_live_sensor_proof(self):
        result = timing_evidence({'p95': .02, 'max': .1})
        self.assertTrue(result['schedule_budget_met'])
        self.assertFalse(result['live_sensor_realtime_validated'])

    def test_recorded_overrun_stays_failed_without_erasing_algorithm_results(self):
        result = timing_evidence({'p95': .03377, 'max': .12})
        self.assertFalse(result['schedule_budget_met'])
        self.assertEqual(result['runtime_source_freshness_gate'], 'unchanged_and_enforced')
        self.assertIn('reported_separately', result['algorithm_evidence_policy'])

    def test_original_boundaries_remain_strict(self):
        for p95, maximum in ((.03, .1), (.02, .15)):
            self.assertFalse(timing_evidence({'p95': p95, 'max': maximum})['schedule_budget_met'])

    def test_missing_nonfinite_negative_and_inverted_statistics_do_not_pass(self):
        for value in ({}, {'p95': float('nan'), 'max': .1}, {'p95': -.01, 'max': .1},
                      {'p95': .02, 'max': .01}, {'p95': True, 'max': .1}):
            result = timing_evidence(value)
            self.assertFalse(result['schedule_statistics_valid'])
            self.assertFalse(result['schedule_budget_met'])
