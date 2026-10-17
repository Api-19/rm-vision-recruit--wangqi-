"""自动几何配准：多朝向初始化、双向截断边界距离；不是人工地标真值。

SciPy cKDTree/minimize官方接口2026-10-18核验，本机1.17.1实测。
"""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import minimize
from grid import Grid
from register_and_compare import compare_transform


def boundary_points(grid, max_points=3500):
    # 对通行语义提边，不从显示灰度推断轮廓；短闭运算减少扫描孔洞。
    free=(grid.free.astype(np.uint8)*255)
    free=cv2.morphologyEx(free,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    edges=cv2.Canny(free,80,160)
    rc=np.argwhere(edges>0)
    if len(rc)<30: raise ValueError('地图边界特征太少，无法自动配准')
    if len(rc)>max_points: rc=rc[np.linspace(0,len(rc)-1,max_points,dtype=int)]
    xy=np.c_[rc[:,1]+.5,grid.image.shape[0]-rc[:,0]-.5]*grid.resolution
    return xy@grid.rotation.T+grid.origin[:2]


def estimate(source, reference, angles=(0,90,180,270)):
    """两方向距离防止部分轮廓偶然贴合；对称场景保留多个候选，不宣称唯一。"""
    source,reference=np.asarray(source,float),np.asarray(reference,float)
    if min(len(source),len(reference))<30: raise ValueError('至少30个边界点')
    tree_a,tree_b=cKDTree(source),cKDTree(reference)
    center_a=(source.min(0)+source.max(0))/2
    center_b=(reference.min(0)+reference.max(0))/2
    def transform(parameters):
        angle,dx,dy=parameters
        R=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        t=center_b-center_a@R.T+np.array([dx,dy])
        return R,t
    def distances(parameters):
        R,t=transform(parameters)
        forward=tree_b.query(source@R.T+t)[0]
        backward=tree_a.query((reference-t)@R)[0]
        return np.r_[forward,backward]
    def objective(parameters):
        return float(np.mean(np.minimum(distances(parameters),.8)))
    candidates=[]
    for degrees in angles:
        initial=np.array([math.radians(degrees),0.,0.])
        solution=minimize(objective,initial,method='Powell',
                          bounds=[(initial[0]-.20,initial[0]+.20),(-1.5,1.5),(-1.5,1.5)],
                          options={'maxiter':50,'xtol':1e-5,'ftol':1e-6})
        R,t=transform(solution.x); d=distances(solution.x)
        candidates.append({'initial_degrees':degrees,'angle_deg':float(np.degrees(solution.x[0])),
                           'rotation':R.tolist(),'translation':t.tolist(),
                           'mean_truncated_distance_m':objective(solution.x),
                           'median_distance_m':float(np.median(d)),
                           'within_015m_fraction':float(np.mean(d<.15)),
                           'optimizer_success':bool(solution.success)})
    candidates.sort(key=lambda c:c['mean_truncated_distance_m'])
    return candidates


def main(args):
    src,ref=Grid.load(args.source_map),Grid.load(args.reference_map)
    candidates=estimate(boundary_points(src),boundary_points(ref))
    best=candidates[0]
    gap=candidates[1]['mean_truncated_distance_m']-best['mean_truncated_distance_m']
    ambiguous=gap<.03
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    evidence={'method':'automatic_boundary_matching','human_landmarks':False,'candidates':candidates,
              'best_vs_second_gap_m':gap,'ambiguous_orientation':ambiguous,
              'note':'Geometric candidate only. Reference-map matching is not independent ground truth.'}
    (out/'candidates.json').write_text(json.dumps(evidence,indent=2))
    # 即使存在对称歧义也可展示最佳候选，但只在显式要求时以它作条件性规划对比。
    compare_transform(args.source_map,args.reference_map,best['rotation'],best['translation'],out,
                      args.queries if args.compare else None,evidence)
    print(json.dumps({'best':best,'ambiguous_orientation':ambiguous,'score_gap_m':gap},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-map',required=True);p.add_argument('--reference-map',required=True)
    p.add_argument('--output',default='qB_sentry/results/auto_registration');p.add_argument('--queries')
    p.add_argument('--compare',action='store_true',help='在最佳自动候选下做条件性对比，不将该候选当真值')
    a=p.parse_args()
    if a.compare and not a.queries:p.error('--compare需要--queries')
    main(a)
