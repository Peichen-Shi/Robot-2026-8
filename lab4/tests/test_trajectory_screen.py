"""Independent endpoint solver must agree with saved ROS reference samples."""
import sys
import unittest
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'optimization'))
from trajectory_screen import basis, evaluate, minimum_snap, reconstruct


class TrajectoryScreenTest(unittest.TestCase):
    def test_endpoint_constraints_and_c4_continuity(self):
        points = np.array([[0.,1.,2.], [2.,-1.,3.], [3.,0.,1.]])
        times = np.array([.4,.7])
        coefficients = minimum_snap(points,times)
        np.testing.assert_allclose(evaluate(coefficients,times,np.array([0.,.4,1.1])),points,atol=1e-7)
        for derivative in range(1,5):
            start = basis(0.,derivative) @ coefficients[0] / times[0]**derivative
            end = basis(1.,derivative) @ coefficients[-1] / times[-1]**derivative
            left = basis(1.,derivative) @ coefficients[0] / times[0]**derivative
            right = basis(0.,derivative) @ coefficients[1] / times[1]**derivative
            np.testing.assert_allclose(start,0.,atol=2e-6)
            np.testing.assert_allclose(end,0.,atol=2e-6)
            np.testing.assert_allclose(left,right,atol=2e-6)

    def test_matches_saved_certified_reference(self):
        profile = yaml.safe_load((ROOT / 'optimization/planner_profiles/sub4.yaml').read_text())
        evidence = ROOT / 'evidence/sub4_qualified/qualify_sub4_20261029_031725_015205632_1_01/analysis/numerics_input.txt'
        lines = evidence.read_text().splitlines()
        n, _ = map(int, lines[0].split())
        initial = list(map(float,lines[1].split()))[:4]
        coefficients, durations = reconstruct(profile,initial=initial)
        reference = np.array([list(map(float,line.split())) for line in lines[2+n:]])
        positions = evaluate(coefficients,durations,reference[:,0],0)
        np.testing.assert_allclose(positions[:,:3],reference[:,1:4],atol=2e-5)
        angular_error = np.arctan2(np.sin(positions[:,3]-reference[:,4]),np.cos(positions[:,3]-reference[:,4]))
        np.testing.assert_allclose(angular_error,0.,atol=2e-5)
        np.testing.assert_allclose(evaluate(coefficients,durations,reference[:,0],1)[:,:3],reference[:,5:8],atol=2e-5)
        np.testing.assert_allclose(evaluate(coefficients,durations,reference[:,0],2)[:,:3],reference[:,8:11],atol=2e-4)

    def test_reject_out_of_aperture_target(self):
        profile = yaml.safe_load((ROOT / 'optimization/planner_profiles/sub4.yaml').read_text())
        profile['gate_lateral_offsets'][0] = .71
        with self.assertRaises(ValueError):
            reconstruct(profile)


if __name__ == '__main__':
    unittest.main()
