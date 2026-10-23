from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from check_fast_result import evaluate

class SpeedAcceptanceTests(unittest.TestCase):
    def evaluate(self,**changes):
        # Deliberately synthetic unit-test fixtures, never experimental evidence.
        m=dict(actual_stable_full_route=True,passed_gates=12,
            mission_start_to_last_gate_s=12.,mission_start_to_settled_finish_s=14.,
            trajectory_start_to_last_gate_s=11.5,planned_trajectory_duration_s=13.5,
            segments={'course_through_last_crossed_gate':{'position_rmse_m':.2,'position_p95_m':.5}},
            completion_verification={'terminal_hold_duration_s':70.},collision_events_during_mission=[])
        m.update(changes)
        return evaluate(m)['passed']
    def test_complete_valid_fast_record(self):self.assertTrue(self.evaluate())
    def test_fast_reference_but_failed_physics(self):self.assertFalse(self.evaluate(actual_stable_full_route=False))
    def test_fast_gates_but_late_settling(self):self.assertFalse(self.evaluate(mission_start_to_settled_finish_s=20.01))
    def test_no_settling_timestamp(self):self.assertFalse(self.evaluate(mission_start_to_settled_finish_s=None))
    def test_missing_gate(self):self.assertFalse(self.evaluate(passed_gates=11))
    def test_collision_cannot_be_averaged_away(self):self.assertFalse(self.evaluate(collision_events_during_mission=[{'object':'wall'}]))
    def test_terminal_hold_cannot_dilute_racing_error(self):
        self.assertFalse(self.evaluate(segments={'course_through_last_crossed_gate':{'position_rmse_m':.8,'position_p95_m':.9}}))

if __name__=='__main__':unittest.main()
