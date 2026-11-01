#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from lab4_metrics import find_sequential_gate_crossings, last_monotonic_epoch


class GateGeometryTest(unittest.TestCase):
    def test_forward_crossing_inside_local_yz_aperture(self):
        times = np.array([0.0, 1.0, 2.0])
        positions = np.array([[-1.0, 0.2, -0.1], [-0.2, 0.2, -0.1],
                              [0.5, 0.2, -0.1]])
        results = find_sequential_gate_crossings(
            times, positions, np.zeros((1, 3)), np.array([[0, 0, 0, 1]])
        )
        self.assertTrue(results[0].passed)
        self.assertAlmostEqual(results[0].local_y_m, 0.2)

    def test_reverse_crossing_is_rejected(self):
        times = np.array([0.0, 1.0])
        positions = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]])
        results = find_sequential_gate_crossings(
            times, positions, np.zeros((1, 3)), np.array([[0, 0, 0, 1]])
        )
        self.assertFalse(results[0].passed)

    def test_timestamp_rollback_keeps_last_epoch(self):
        times, values = last_monotonic_epoch(
            [10.0, 11.0, 1.0, 1.0, 2.0], np.array([[10], [11], [1], [2], [3]])
        )
        np.testing.assert_allclose(times, [1.0, 2.0])
        np.testing.assert_allclose(values[:, 0], [2, 3])


if __name__ == "__main__":
    unittest.main()
