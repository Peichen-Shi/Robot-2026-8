import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from lab4_gate_timer import GateTimer

class GateTimerTests(unittest.TestCase):
    def make(self):
        t=GateTimer()
        t.set_gates([[0,0,0,0,0,0,1],[2,0,0,0,0,0,1]])
        t.status('test',1,'TRACKING')
        return t
    def test_first_to_last_only_and_freeze(self):
        t=self.make()
        t.state(0,[-1,0,0]);self.assertEqual(t.result()['elapsed_ms'],0)
        for stamp,x in [(.2,1),(.4,1.5),(.6,2.5)]:t.state(stamp,[x,0,0])
        self.assertAlmostEqual(t.result()['elapsed_ms'],400)
        self.assertEqual(t.result()['phase'],'FINISHED')
        t.state(5,[3,0,0]);self.assertAlmostEqual(t.result()['elapsed_ms'],400)
    def test_reverse_not_a_start(self):
        t=self.make();t.state(0,[1,0,0]);t.state(.2,[-1,0,0])
        self.assertIsNone(t.start)
    def test_outside_aperture_invalid(self):
        t=self.make();t.state(0,[-1,2,0]);t.state(.2,[1,2,0])
        self.assertEqual(t.result()['phase'],'INVALID')
    def test_rollback_invalid(self):
        t=self.make();t.state(1,[-1,0,0]);t.state(.9,[1,0,0])
        self.assertEqual(t.result()['phase'],'INVALID')
    def test_gap_invalid(self):
        t=self.make();t.state(0,[-1,0,0]);t.state(1,[1,0,0])
        self.assertEqual(t.result()['phase'],'INVALID')
    def test_duplicate_no_extra_crossing(self):
        t=self.make();t.state(0,[-1,0,0]);t.state(.2,[1,0,0]);t.state(.2,[1,0,0])
        self.assertEqual(len(t.crossings),1)
    def test_new_trial_resets(self):
        t=self.make();t.state(0,[-1,0,0]);t.state(.2,[1,0,0]);t.status('next',2,'READY_HOLD')
        self.assertIsNone(t.start);self.assertEqual(t.result()['phase'],'WAIT_FIRST_GATE')

if __name__=='__main__':unittest.main()
