"""Separate replay scheduler fidelity from sensor and algorithm validation."""
import math


def timing_evidence(summary):
    """Keep the original strict budget; failure never validates live timing."""
    p95, maximum = summary.get('p95'), summary.get('max')
    valid = (isinstance(p95, (int, float)) and not isinstance(p95, bool)
             and isinstance(maximum, (int, float)) and not isinstance(maximum, bool)
             and math.isfinite(p95) and math.isfinite(maximum)
             and 0 <= p95 <= maximum)
    met = bool(valid and p95 < .03 and maximum < .15)
    return {
        'scope': 'recorded_replay_publication_schedule',
        'schedule_statistics_valid': bool(valid),
        'schedule_budget_met': met,
        'schedule_budget_s': {'p95_exclusive': .03, 'max_exclusive': .15},
        'publish_lateness_s': dict(summary),
        'live_sensor_realtime_validated': False,
        'live_sensor_realtime_status': 'not_tested_by_recorded_replay',
        'runtime_source_freshness_gate': 'unchanged_and_enforced',
        'algorithm_evidence_policy': 'reported_separately; late input may affect runtime recovery',
    }
