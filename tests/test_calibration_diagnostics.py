"""实际数据诊断中发现的角点形状广播陷阱与边缘外推红旗回归。"""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'q2_calib'))
from diagnose import reprojection_rms,radial_check


class CalibrationDiagnosticsTests(unittest.TestCase):
    def test_corner_shapes_must_not_broadcast(self):
        points=np.arange(108,dtype=float).reshape(54,2)
        self.assertEqual(reprojection_rms(points[:,None,:],points),0.)
        self.assertAlmostEqual(reprojection_rms((points+[1,2])[:,None,:],points),np.sqrt(5))

    def test_unobserved_edge_fold_is_reported(self):
        K=np.array([[100.,0,100],[0,100,100],[0,0,1]])
        center=np.array([[90,90],[110,90],[110,110],[90,110]],float)
        good=radial_check(K,np.zeros(5),(200,200),center)
        bad=radial_check(K,np.array([0.,0.,0.,0.,-1.]),(200,200),center)
        self.assertTrue(good['radial_monotone_full_frame'])
        self.assertFalse(bad['radial_monotone_full_frame'])
        self.assertGreater(bad['full_frame_max_normalized_radius'],bad['observed_max_normalized_radius'])


if __name__=='__main__':unittest.main()
