"""纸平面垂距与视线夹角，避免把中心斜距当离地高度。"""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'q2_calib'))
from angle_study import plane_metrics


class PlaneMetricsTests(unittest.TestCase):
    def test_off_axis_target_not_slant_times_optical_cosine(self):
        distance,angle=plane_metrics([0,0,0],[.3,0,.4])
        self.assertAlmostEqual(distance,.4)
        self.assertAlmostEqual(angle,np.degrees(np.arccos(.8)))
        # 法线沿光轴，纸中心却偏离主点：斜距0.5不等于垂距0.4。
        self.assertNotAlmostEqual(distance,.5)

    def test_tilted_plane(self):
        distance,angle=plane_metrics([0,np.pi/3,0],[0,0,1])
        self.assertAlmostEqual(distance,.5)
        self.assertAlmostEqual(angle,60.)


if __name__=='__main__':unittest.main()
