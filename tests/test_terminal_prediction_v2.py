import unittest
from experiments.analysis.terminal_prediction_v2 import _tail_statistics


class TerminalPredictionV2Test(unittest.TestCase):
    def test_tail_uses_additional_change_after_twelve(self):
        releases=[{"deltas":{12:-10,16:-12,20:-13,24:-14}},{"deltas":{12:-8,16:-9,20:-10,24:-11}}]
        result=_tail_statistics(releases)
        self.assertEqual(result["16"]["median"],-1.5)
        self.assertEqual(result["24"]["median"],-3.5)


if __name__=="__main__":unittest.main()
