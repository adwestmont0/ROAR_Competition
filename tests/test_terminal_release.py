import unittest
import numpy as np

from experiments.analysis.terminal_release import terminal_objective


class TerminalReleaseTest(unittest.TestCase):
    def test_finds_bottom_before_sustained_recovery(self):
        speeds=[200,190,180,170,171,176,178,180]
        profile=[{"stability_capped_planned_speed_kmh":v,"ds_m":5,"custom_waypoint_index":i} for i,v in enumerate(speeds)]
        result=terminal_objective(profile,np.asarray(speeds,dtype=float),0,7)
        self.assertEqual(result["custom_waypoint_index"],3)
        self.assertEqual(result["target_speed_kmh"],170)


if __name__=="__main__":unittest.main()
