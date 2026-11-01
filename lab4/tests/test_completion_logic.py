#!/usr/bin/env python3
"""Regression cases that must not be confused with a stable complete race."""
import copy
import os
import sys
import unittest
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from lab4_metrics import evaluate_completion


class CompletionTests(unittest.TestCase):
    def setUp(self):
        self.status = [dict(time=0., trajectory_id=1, planned_duration_s=100.,
                            phase='TRACKING', reference_complete=False,
                            sample_tau_s=0., trajectory_elapsed_s=0., fault=False),
                       dict(time=100., trajectory_id=1, planned_duration_s=100.,
                            phase='POST_RACE_HOLD', reference_complete=True,
                            sample_tau_s=100., trajectory_elapsed_s=100., fault=False),
                       dict(time=171., trajectory_id=1, planned_duration_s=100.,
                            phase='FINISHED', reference_complete=True,
                            sample_tau_s=100., trajectory_elapsed_s=171., fault=False)]
        self.hold = np.column_stack((np.linspace(100., 171., 3551),
                                     np.full((3551, 4), .01)))

    def check(self, statuses=None, hold=None, passed=12, faults=()):
        return evaluate_completion(self.status if statuses is None else statuses,
                                   self.hold if hold is None else hold,
                                   passed, 12, .1, .2, faults)

    def test_complete(self):
        self.assertTrue(self.check()['actual_stable_full_route'])

    def test_half_reference(self):
        s = copy.deepcopy(self.status)
        s[-1].update(phase='TRACKING', sample_tau_s=50., trajectory_elapsed_s=50.)
        self.assertFalse(self.check(statuses=s)['reference_full_route'])

    def test_missing_gate(self):
        self.assertFalse(self.check(passed=11)['actual_stable_full_route'])

    def test_delayed_divergence(self):
        h = self.hold.copy()
        h[-100:, 1] = 17.
        self.assertFalse(self.check(hold=h)['actual_stable_full_route'])

    def test_short_hold(self):
        self.assertFalse(self.check(hold=self.hold[:1500])['actual_stable_full_route'])

    def test_changed_id_or_clock(self):
        for field, value in [('trajectory_id', 2), ('time', -1.),
                             ('trajectory_elapsed_s',0.),('sample_tau_s',0.)]:
            s = copy.deepcopy(self.status)
            s[-1][field] = value
            self.assertFalse(self.check(statuses=s)['actual_stable_full_route'])

    def test_fault_nan_and_gap(self):
        self.assertFalse(self.check(faults=['watchdog'])['actual_stable_full_route'])
        for value in (float('nan'), 1.):
            h = self.hold.copy()
            h[-1, 1] = value
            self.assertFalse(self.check(hold=h)['actual_stable_full_route'])
        h = np.delete(self.hold, np.arange(1200, 1300), axis=0)
        self.assertFalse(self.check(hold=h)['actual_stable_full_route'])


if __name__ == '__main__':
    unittest.main()
