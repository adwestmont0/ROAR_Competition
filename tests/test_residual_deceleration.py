import unittest
import numpy as np

from experiments.analysis.residual_deceleration import _predict_exponential


class ResidualDecelerationTest(unittest.TestCase):
    def test_exponential_preserves_deceleration_sign_and_decay(self):
        features=np.asarray([1,-10,-9,2,0,0,1,0],dtype=float)
        prediction=_predict_exponential(4,features)
        self.assertLess(prediction[2],0)
        self.assertLess(prediction[8],prediction[2])
        increments=[prediction[i]-prediction[i-1] for i in range(2,9)]
        self.assertGreater(increments[-1],increments[0])


if __name__=="__main__": unittest.main()
