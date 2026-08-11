import unittest

from experiments.analysis.terminal_tail_bridge import _interpolate, _predict_bridge


class TerminalTailBridgeTest(unittest.TestCase):
    def test_interpolation_starts_with_zero_credit_at_24(self):
        values={28:-2.0,32:-3.0,36:-4.0,40:-5.0,48:-6.0}
        self.assertEqual(_interpolate(values,24),0.0)
        self.assertEqual(_interpolate(values,26),-1.0)

    def test_piecewise_accumulates_conservative_segments(self):
        segments={28:-1.0,32:-2.0,36:-1.0,40:-.5,48:-.5}
        self.assertEqual(_predict_bridge(segments,"piecewise_envelope",{},40),-4.5)

    def test_zero_credit_never_adds_deceleration(self):
        self.assertEqual(_predict_bridge(None,"zero_credit",{},48),0.0)


if __name__=="__main__":unittest.main()
