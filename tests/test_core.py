"""算法性质、碰撞边界与合成几何验证。合成图绝不作为本人采集证据。"""
import sys
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'qB_sentry'),str(ROOT/'q2_calib')]
from grid import Grid
from search import search
from measure import pose,a4_points
from calibrate import object_points,fit,calibrate
from pointcloud_map import align_normal,rasterize
from register_and_compare import rigid_fit
from calibrate import compare
from measure import measure
import json


class SearchTests(unittest.TestCase):
    def test_known_cost_and_blocked_corner(self):
        free=np.ones((5,5),bool)
        a=search(free,(0,0),(4,4),.1)
        self.assertAlmostEqual(a['cost_m'],4*np.sqrt(2)*.1)
        free[:]=False; free[0,0]=free[1,1]=True
        self.assertIsNone(search(free,(0,0),(1,1),1)['path'])

    def test_astar_equals_dijkstra_on_random_maps(self):
        rng=np.random.default_rng(7)
        for _ in range(15):
            free=rng.random((18,20))>.2; free[1,1]=free[-2,-2]=True
            a=search(free,(1,1),(16,18),.05,'astar')
            d=search(free,(1,1),(16,18),.05,'dijkstra')
            self.assertEqual(a['reason'],d['reason'])
            if a['path']: self.assertAlmostEqual(a['cost_m'],d['cost_m'])

    def test_coordinates_and_continuous_collision(self):
        free=np.ones((30,30),bool); free[15,15]=False
        g=Grid(np.uint8(free)*254,.1,np.array([-3.,-4.,.4]),free)
        for rc in [(0,0),(5,9),(29,29)]:
            self.assertEqual(g.world_to_cell(g.cell_to_world(rc)),rc)
        self.assertFalse(g.disk_clear(g.cell_to_world((0,0)),.1))
        a,b=g.cell_to_world((15,10)),g.cell_to_world((15,20))
        self.assertTrue(g.disk_clear(a,.1)); self.assertTrue(g.disk_clear(b,.1))
        self.assertFalse(g.segment_clear(a,b,.1))


class GeometryTests(unittest.TestCase):
    def test_calibration_and_pnp_from_known_geometry(self):
        K=np.array([[800.,0,320],[0,805,240],[0,0,1]])
        D=np.array([-.08,.02,.001,-.002,0.])
        obj=object_points(9,6,.025); observations=[]
        rng=np.random.default_rng(12)
        for i in range(20):
            r=rng.uniform(-.4,.4,3); t=np.array([-.10+rng.uniform(-.05,.05),-.06+rng.uniform(-.04,.04),.7+i*.025])
            uv,_=cv2.projectPoints(obj,r,t,K,D); observations.append(uv.astype(np.float32))
        result=fit([obj]*len(observations),observations,(640,480))
        np.testing.assert_allclose(result['camera_matrix'],K,rtol=5e-4,atol=.02)
        r=np.array([.25,-.3,.1]); t=np.array([.1,-.05,1.7])
        corners,_=cv2.projectPoints(a4_points(),r,t,K,D)
        solution=pose(corners.reshape(4,2),K,D)
        self.assertAlmostEqual(solution['center_range_m'],np.linalg.norm(t),places=5)
        self.assertAlmostEqual(solution['optical_z_m'],1.7,places=5)
        with self.assertRaises(ValueError): pose(corners.reshape(4,2)[[0,2,1,3]],K,D)

    def test_real_detection_pipeline_on_rendered_boards(self):
        # 渲染15张已知姿态棋盘，仅测试检测与文件流程。
        with tempfile.TemporaryDirectory(prefix='rm-synthetic-calib-') as directory:
            root=Path(directory); images=root/'synthetic'; images.mkdir()
            cols,rows,square=7,5,.03
            board=np.zeros(((rows+1)*80,(cols+1)*80),np.uint8)
            for r in range(rows+1):
                for c in range(cols+1): board[r*80:(r+1)*80,c*80:(c+1)*80]=255 if (r+c)%2 else 0
            K=np.array([[800.,0,320],[0,800,240],[0,0,1]])
            bounds=np.array([[-square,-square,0],[cols*square,-square,0],
                             [cols*square,rows*square,0],[-square,rows*square,0]],float)
            src=np.float32([[0,0],[board.shape[1],0],[board.shape[1],board.shape[0]],[0,board.shape[0]]])
            for i in range(15):
                r=np.array([.3*np.sin(i),.3*np.cos(i),.08*np.sin(2*i)])
                t=np.array([-.085+.02*np.sin(i),-.05+.015*np.cos(i),.7+.02*i])
                target,_=cv2.projectPoints(bounds,r,t,K,np.zeros(5))
                H=cv2.getPerspectiveTransform(src,target.reshape(4,2).astype(np.float32))
                image=cv2.warpPerspective(board,H,(640,480),borderValue=180)
                cv2.imwrite(str(images/f'{i:02}.png'),image)
            result=calibrate(images,root/'results',cols,rows,30)
            self.assertEqual(len(result['images']),15)
            self.assertLess(result['rms_px'],.5)
            np.testing.assert_allclose(np.asarray(result['camera_matrix'])[:2,:2],K[:2,:2],rtol=.03,atol=1.)

    def test_calibration_variants_and_measurement_files(self):
        K=np.array([[800.,0,320],[0,800,240],[0,0,1]])
        obj=object_points(9,6,.025); observations=[]
        for i in range(10):
            uv,_=cv2.projectPoints(obj,np.array([.05*i,.2,.03*i]),np.array([-.1,-.06,.8+.03*i]),K,np.zeros(5))
            observations.append(uv.astype(np.float32))
        with tempfile.TemporaryDirectory(prefix='rm-synthetic-batch-') as directory:
            root=Path(directory)
            np.savez(root/'obs.npz',object_points=obj,image_points=np.array(observations),image_size=[640,480])
            compare(root/'obs.npz',root/'variants',25,20,10)
            results=json.loads((root/'variants/comparison.json').read_text())
            a=np.array(results['all']['tvecs_m']); b=np.array(results['wrong_square']['tvecs_m'])
            np.testing.assert_allclose(b,a*.8,rtol=.001,atol=.0001)
            (root/'calibration.json').write_text(json.dumps({'camera_matrix':K.tolist(),'distortion_coefficients':[0]*5,'image_size':[640,480]}))
            image=np.full((480,640,3),180,np.uint8); cv2.imwrite(str(root/'synthetic.png'),image)
            uv,_=cv2.projectPoints(a4_points(),np.array([.2,.1,0.]),np.array([0.,0.,1.]),K,np.zeros(5))
            record={'image':'synthetic.png','corners_px':uv.reshape(4,2).tolist(),
                    'reference_metric':'center_range_m','actual_distance_m':1.,'angle_deg':0}
            (root/'manifest.json').write_text(json.dumps([record]))
            measured=measure(root/'calibration.json',root/'manifest.json',root/'results')
            self.assertLess(abs(measured[0]['error_m']),1e-6)
            with self.assertRaises(ValueError):
                pose([[1,1],[2,2]],K,np.zeros(5))

    def test_landmark_registration(self):
        source=np.array([[0,0],[1,0],[.2,2],[2,3]],float)
        R=np.array([[0.,-1],[1,0]]); t=np.array([2.,-3.])
        recovered,offset,error=rigid_fit(source,source@R.T+t)
        np.testing.assert_allclose(recovered,R,atol=1e-8)
        np.testing.assert_allclose(offset,t,atol=1e-8); self.assertLess(error,1e-8)

    def test_ground_alignment_and_ramp_step(self):
        n=np.array([.34,.06,.94]); n/=np.linalg.norm(n)
        np.testing.assert_allclose(align_normal(n)@n,[0,0,1],atol=1e-8)
        # 稠密平面必须有通行区；陡直台阶边缘不能被当成连续缓坡。
        x,y=np.meshgrid(np.arange(0,2,.025),np.arange(0,1,.025))
        flat=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        image,*_=rasterize(flat)
        self.assertGreater(np.sum(image==254),500)
        stepped=flat.copy(); stepped[stepped[:,0]>1,2]=.4
        image,origin,height,slope,seen,blocked=rasterize(stepped)
        self.assertTrue(np.any(blocked[:,18:23]))


if __name__=='__main__': unittest.main(verbosity=2)
