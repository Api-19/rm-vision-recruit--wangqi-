"""本地补完阶段的输入与配准回归，不涉及学生练习或云端。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'qB_sentry'),str(ROOT/'q2_calib')]
from grid import Grid
from measure import pose,a4_points,measure
from register_and_compare import compare_transform
from auto_registration import estimate
from plan import run
from check_inputs import audit


class InputTests(unittest.TestCase):
    def make_map(self,root,negate=0,free=.25,occupied=.65):
        image=np.full((30,40),0 if negate else 255,np.uint8)
        cv2.imwrite(str(root/'map.pgm'),image)
        path=root/'map.yaml'
        path.write_text(yaml.safe_dump({'image':'map.pgm','mode':'trinary','origin':[0,0,0],
            'resolution':.1,'free_thresh':free,'occupied_thresh':occupied,'negate':negate}))
        return path

    def test_invalid_grid_settings(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);path=self.make_map(root)
            cfg=yaml.safe_load(path.read_text())
            for key,value in [('resolution',float('nan')),('free_thresh',.9),('origin',[0,0]),('negate',2)]:
                changed=dict(cfg);changed[key]=value;path.write_text(yaml.safe_dump(changed))
                with self.assertRaises(ValueError,msg=key):Grid.load(path)

    def test_invalid_disk_input(self):
        g=Grid(np.ones((10,10),np.uint8),.1,np.zeros(3),np.ones((10,10),bool))
        for xy,radius in [([float('nan'),0],.3),([0,0],-.1),([0,0],float('inf'))]:
            with self.assertRaises(ValueError):g.disk_clear(xy,radius)

    def test_invalid_camera_matrix(self):
        K=np.eye(3);K[0,0]=-1
        with self.assertRaises(ValueError):pose(np.array([[10,10],[50,10],[50,70],[10,70]]),K,np.zeros(5))

    def test_resampling_exterior_never_free(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);path=self.make_map(root,negate=1,free=.85,occupied=.95)
            result=compare_transform(path,path,np.eye(2),np.array([100.,0]),root/'mapped')
            self.assertFalse(Grid.load(result).free.any())

    def test_reflection_is_not_rigid_rotation(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);path=self.make_map(root)
            with self.assertRaises(ValueError):compare_transform(path,path,np.diag([1.,-1]),[0,0],root/'bad')

    def test_no_photos_are_not_ready(self):
        with tempfile.TemporaryDirectory() as folder:
            report=audit(Path(folder)/'missing',Path(folder)/'manifest.json')
            self.assertFalse(report['calibration_input_ready'])
            self.assertEqual(report['valid_chessboard_count'],0)
            self.assertEqual(report['valid_measurement_count'],0)

    def test_failed_or_unreachable_rerun_does_not_keep_old_path(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);path=self.make_map(root)
            queries=root/'queries.json';queries.write_text(json.dumps({'queries':[{'start':[-5,0],'goal':[1,1]}]}))
            out=root/'results';(out/'astar').mkdir(parents=True)
            for file in [out/'path1.csv',out/'astar/path1.csv',out/'astar/check1.txt']:file.write_text('old success')
            result=run(path,queries,out)
            self.assertIsNone(result[0]['length_m'])
            self.assertFalse((out/'path1.csv').exists());self.assertFalse((out/'astar/check1.txt').exists())
            self.assertEqual(json.loads((out/'run_status.json').read_text())['status'],'complete')
            queries.write_text('{"queries": []}')
            with self.assertRaises(ValueError):run(path,queries,out)
            self.assertEqual(json.loads((out/'run_status.json').read_text())['status'],'failed')

    def test_auto_registration_recovers_asymmetric_geometry(self):
        rng=np.random.default_rng(31)
        a=rng.uniform([0,0],[7,4],(150,2))
        theta=.07;R=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        t=np.array([4.,-2.]);b=a@R.T+t
        best=estimate(a,b,angles=(0,180))[0]
        np.testing.assert_allclose(best['rotation'],R,atol=.01)
        np.testing.assert_allclose(best['translation'],t,atol=.02)
        self.assertLess(best['median_distance_m'],.02)


if __name__=='__main__':unittest.main(verbosity=2)
