"""地形分类与未知灰度语义回归；合成数据不冒充真实地形真值。"""
import sys
from pathlib import Path
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'qB_sentry'))
from extended_study import classify_height, save_map
from pointcloud_map import rasterize
from grid import Grid
from decision_sim import decide


class ExtendedTests(unittest.TestCase):
    def test_ramp_vs_step_vs_raised_flat(self):
        y,x=np.mgrid[:30,:40]*.05
        ramp=np.tan(np.radians(15))*x
        r=classify_height(ramp,.05)
        self.assertTrue(r['ramp_candidate'][10,15])
        self.assertFalse(r['step'].any())
        step=np.where(x<1.,0.,.20)
        s=classify_height(step,.05)
        self.assertTrue(s['step'][:,19:21].all())
        flat=classify_height(np.ones((30,40))*.5,.05)
        self.assertFalse(flat['step'].any())
        self.assertFalse(flat['ramp_candidate'].any())

    def test_unknown_never_loads_as_free(self):
        y,x=np.mgrid[:20,:20]*.05
        # 带中央大孔洞的已观测地面，孔洞中心离支持点超过10cm。
        pts=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        pts=pts[~((pts[:,0]>.2)&(pts[:,0]<.75)&(pts[:,1]>.2)&(pts[:,1]<.75))]
        image,origin,*_=rasterize(pts)
        self.assertTrue(np.any(image==128))
        with tempfile.TemporaryDirectory() as temp:
            save_map(image,origin,Path(temp))
            grid=Grid.load(Path(temp)/'map.yaml')
            np.testing.assert_array_equal(grid.free,image==254)
            self.assertFalse(grid.free[image==128].any())

    def test_missing_support_not_ramp(self):
        h=np.tile(np.arange(20)*.02,(20,1));h[10,10]=np.nan
        result=classify_height(h,.05)
        self.assertFalse(result['ramp_candidate'][10,10])
        self.assertFalse(result['ramp_candidate'][9,9])

    def test_decision_priority_and_time_boundary(self):
        self.assertEqual(decide(20,0,(1,1),10,False),'RETREAT')
        self.assertEqual(decide(80,0,(1,1),10,False),'RESUPPLY')
        self.assertEqual(decide(80,80,None,30,True),'PATROL')
        self.assertEqual(decide(80,80,None,29,True),'DEFEND')


if __name__=='__main__':unittest.main()
