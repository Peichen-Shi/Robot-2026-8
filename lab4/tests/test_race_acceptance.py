"""Synthetic timing fixtures; these are not new flight evidence."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'optimization'))
from race_acceptance import stable_terminal_window


class TerminalWindowTests(unittest.TestCase):
    def samples(self):
        return [(10 + i * .02, .1, .1, .1) for i in range(101)]

    def test_continuous_second(self):
        self.assertTrue(stable_terminal_window(self.samples()))

    def test_many_duplicates_do_not_make_a_second(self):
        self.assertFalse(stable_terminal_window([(10., .1, .1, .1)] * 100))

    def test_dense_short_burst_is_not_a_second(self):
        self.assertFalse(stable_terminal_window(self.samples()[-30:]))

    def test_missing_telemetry_inside_tail(self):
        rows = self.samples()
        self.assertFalse(stable_terminal_window(rows[:65] + rows[80:]))

    def test_clock_rollback(self):
        rows = self.samples()
        rows[60] = (10., .1, .1, .1)
        self.assertFalse(stable_terminal_window(rows))

    def test_nonfinite_telemetry(self):
        for field in range(4):
            for value in (math.nan, math.inf, -math.inf):
                rows = self.samples()
                bad = list(rows[-1]); bad[field] = value; rows[-1] = tuple(bad)
                self.assertFalse(stable_terminal_window(rows))

    def test_unstable_at_start_of_window(self):
        rows = self.samples()
        rows[50] = (11., .31, .1, .1)
        self.assertFalse(stable_terminal_window(rows))

    def test_recovered_old_excursion(self):
        rows = self.samples()
        rows[10] = (10.2, 5., .1, .1)
        self.assertTrue(stable_terminal_window(rows))

    def test_duplicate_with_unstable_sample_is_rejected(self):
        rows = self.samples()
        rows.insert(80, (rows[80][0], .1, 2., .1))
        self.assertFalse(stable_terminal_window(rows))

    def test_unstable_duplicate_at_window_boundary(self):
        rows = self.samples()
        rows.insert(50, (11., 1., .1, .1))
        self.assertFalse(stable_terminal_window(rows))

    def test_empty(self):
        self.assertFalse(stable_terminal_window([]))


if __name__ == '__main__':
    unittest.main()
