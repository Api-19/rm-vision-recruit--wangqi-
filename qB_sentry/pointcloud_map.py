"""点云→候选平面→调平→局部地面高度→可通行栅格。

多层场地用局部地面，而不是统一的绝对z阈值。自动结果需要查看过程图；
单一2D高度层不能完整表达上下重叠道路，相关格子保守记为障碍/未知。
"""
import argparse
import json
from pathlib import Path
import math
import cv2
import numpy as np
import open3d as o3d
from scipy.ndimage import distance_transform_edt, median_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml


def align_normal(normal):
    """列向量旋转：把地面法线转到+z，返回3×3旋转矩阵。"""
    normal=np.asarray(normal,float); normal/=np.linalg.norm(normal)
    if normal[2]<0: normal=-normal
    z=np.array([0.,0.,1.]); v=np.cross(normal,z); c=float(normal@z)
    if np.linalg.norm(v)<1e-10: return np.eye(3)
    K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+K+K@K/(1+c)


def scatter(points, path, title, color=None):
    step=max(1,len(points)//45000); a=points[::step]
    fig=plt.figure(figsize=(9,6)); ax=fig.add_subplot(projection='3d')
    ax.scatter(a[:,0],a[:,1],a[:,2],s=.15,c=(a[:,2] if color is None else color[::step]),cmap='terrain')
    ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title=title)
    ax.set_box_aspect(np.maximum(np.ptp(a,axis=0),.1))
    fig.tight_layout(); fig.savefig(path,dpi=160); plt.close(fig)


def extract_planes(cloud, count=10, threshold=.04):
    remaining=cloud
    planes=[]
    for _ in range(count):
        if len(remaining.points)<100: break
        coefficients, ids=remaining.segment_plane(distance_threshold=threshold,ransac_n=3,num_iterations=600)
        if len(ids)<100: break
        a=np.asarray(coefficients,float); a/=np.linalg.norm(a[:3])
        if a[2]<0: a=-a
        pts=np.asarray(remaining.points)[ids]
        planes.append({'coefficients':a.tolist(),'inliers':len(ids),
                       'centroid':pts.mean(0).tolist(), 'extent':np.ptp(pts,axis=0).tolist(),
                       'tilt_from_sensor_z_deg':float(np.degrees(np.arccos(np.clip(a[2],-1,1))))})
        remaining=remaining.select_by_index(ids,invert=True)
    return planes


def rasterize(points, res=.05, max_step=.12, robot_height=.6, max_slope_deg=30.):
    # 先限制在可能的行走高度范围，避免远处飞点把地图边界撑大。
    plausible=points[(points[:,2]>-.15)&(points[:,2]<2.)]
    if len(plausible)<100: raise ValueError('调平后合理高度点太少，请检查地面选择')
    low=np.quantile(plausible[:,:2],.001,axis=0)-res
    high=np.quantile(plausible[:,:2],.999,axis=0)+res
    origin=np.floor(low/res)*res
    w,h=np.ceil((high-origin)/res).astype(int)
    if w*h>8_000_000: raise ValueError('地图过大，请核对点云单位/离群点')
    rc=np.floor((plausible[:,:2]-origin)/res).astype(int)
    valid=(rc[:,0]>=0)&(rc[:,0]<w)&(rc[:,1]>=0)&(rc[:,1]<h)
    p=plausible[valid]; rc=rc[valid]; idx=rc[:,1]*w+rc[:,0]
    # 按格分组，以低分位估计局部地面；孤立格保留不确定性。
    order=np.argsort(idx); ids=idx[order]; z=p[order,2]
    starts=np.r_[0,np.flatnonzero(np.diff(ids))+1]; ends=np.r_[starts[1:],len(ids)]
    floor=np.full(h*w,np.nan); count=np.zeros(h*w,int); obstruction=np.zeros(h*w,bool)
    multilayer=np.zeros(h*w,bool)
    for begin,end in zip(starts,ends):
        cell=ids[begin]; values=np.sort(z[begin:end]); count[cell]=len(values)
        floor[cell]=np.quantile(values,.15)
        # 机器人头顶以下有非地面返回，不能仅凭最低点判为空地。
        above=values-floor[cell]
        obstruction[cell]=np.any((above>max_step)&(above<robot_height+.05))
        # 无法用2D表达的重叠结构，保守记录。
        multilayer[cell]=np.any(above>robot_height+.05) and np.any(above<.05)
    floor=floor.reshape(h,w); count=count.reshape(h,w)
    observed=(count>=1)&np.isfinite(floor)
    if not observed.any(): raise ValueError('没有足够的局部地面支持')
    dist,nearest=distance_transform_edt(~observed,return_indices=True)
    support=dist*res<=.10
    # 仅短距离填洞，不能把整个未观测区域填为空地。
    filled=median_filter(floor[tuple(nearest)],size=3)
    filled[~support]=np.nan
    blocked=obstruction.reshape(h,w)|multilayer.reshape(h,w)
    slope=np.zeros((h,w)); step_edges=np.zeros((h,w),bool)
    for dr,dc in [(1,0),(0,1),(1,1),(1,-1)]:
        r0=slice(max(0,dr),min(h,h+dr)); r1=slice(max(0,-dr),min(h,h-dr))
        c0=slice(max(0,dc),min(w,w+dc)); c1=slice(max(0,-dc),min(w,w-dc))
        dz=np.abs(filled[r0,c0]-filled[r1,c1]); valid=np.isfinite(dz)
        angle=np.degrees(np.arctan2(np.nan_to_num(dz),res*math.hypot(dr,dc)))
        dangerous=valid&((dz>max_step)|(angle>max_slope_deg))
        step_edges[r0,c0]|=dangerous; step_edges[r1,c1]|=dangerous
        slope[r0,c0]=np.maximum(slope[r0,c0],angle)
        slope[r1,c1]=np.maximum(slope[r1,c1],angle)
    # 仅在10cm内的局部表面支持范围插值；远处未知不扩展。
    # 插值是地表连续性假设，过程图中保留observed以供检查，不能当实测真值。
    free=support & ~blocked & ~step_edges
    image=np.full((h,w),205,np.uint8)
    image[observed]=0; image[free]=254
    return np.flipud(image),origin,filled,slope,observed,blocked|step_edges


def main(args):
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    o3d.utility.random.seed(args.seed)
    cloud=o3d.io.read_point_cloud(args.cloud)
    if not len(cloud.points): raise ValueError('点云为空')
    cloud.remove_non_finite_points()
    excluded_count=0
    if args.exclude_boxes:
        boxes=json.loads(Path(args.exclude_boxes).read_text())
        coords=np.asarray(cloud.points); keep=np.ones(len(coords),bool)
        for box in boxes:
            low,high=np.asarray(box['min'],float),np.asarray(box['max'],float)
            if low.shape!=(3,) or high.shape!=(3,) or np.any(low>=high): raise ValueError('排除框min/max必须为三维且min<max')
            keep &= ~np.all((coords>=low)&(coords<=high),axis=1)
        excluded_count=int((~keep).sum()); cloud=cloud.select_by_index(np.flatnonzero(keep).tolist())
    raw=np.asarray(cloud.points).copy(); scatter(raw,output/'01_raw.png','Raw sensor coordinates')
    cloud=cloud.voxel_down_sample(args.voxel)
    before=len(cloud.points)
    cloud,inliers=cloud.remove_statistical_outlier(nb_neighbors=20,std_ratio=args.std_ratio)
    print('raw',len(raw),'voxel',before,'clean',len(cloud.points),flush=True)
    planes=extract_planes(cloud)
    candidates=[(i,p) for i,p in enumerate(planes) if p['tilt_from_sensor_z_deg']<40 and abs(p['coefficients'][3])<1.5]
    if args.ground_plane is not None:
        choice=args.ground_plane
    elif candidates:
        choice=max(candidates,key=lambda item:item[1]['inliers'])[0]
    else:
        raise ValueError('未找到符合安装高度/法线先验的地面候选，请检查点云')
    if not 0<=choice<len(planes): raise ValueError('地面候选编号越界')
    model=np.asarray(planes[choice]['coefficients']); rotation=align_normal(model[:3])
    # z=法线·原始点+d，所选地面变成z=0。
    transform=np.eye(4); transform[:3,:3]=rotation; transform[2,3]=model[3]
    if args.align_yaw:
        # 用最大竖直墙面方向对正；只确定轴方向，不确定世界坐标的正反与平移。
        wall=np.asarray(planes[0]['coefficients'][:3])@rotation.T
        if abs(wall[2])<.3:
            tangent=np.array([-wall[1],wall[0]])
            angle=-math.atan2(tangent[1],tangent[0])
            yaw=np.eye(4); yaw[:2,:2]=[[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]
            transform=yaw@transform
    if args.world_transform:
        correction=np.asarray(json.loads(Path(args.world_transform).read_text()),float)
        if correction.shape!=(4,4) or not np.allclose(correction[3],[0,0,0,1]):
            raise ValueError('world-transform须为4×4齐次变换JSON')
        if not np.allclose(correction[:3,:3].T@correction[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(correction[:3,:3])<.99:
            raise ValueError('world-transform旋转部分须为正交且det=1')
        if not np.allclose(correction[:3,2],[0,0,1],atol=1e-5) or abs(correction[2,3])>1e-5:
            raise ValueError('世界配准只允许绕z旋转与XY平移，保留已调平的地面高度')
        # 对正后的坐标到题目世界坐标，由独立配准提供，不猜测两张图已同坐标。
        transform=correction@transform
    points=np.asarray(cloud.points)@transform[:3,:3].T+transform[:3,3]
    scatter(points,output/'02_levelled.png','Levelled cloud')
    normal_distance=np.abs(np.asarray(cloud.points)@model[:3]+model[3])
    fig=plt.figure(figsize=(9,6)); ax=fig.add_subplot(projection='3d')
    a=np.asarray(cloud.points)[::12]; color=np.where(normal_distance[::12]<.04,'tab:green','lightgray')
    ax.scatter(*a.T,s=.4,c=color); ax.set_title(f'Ground candidate {choice} (green); inspect before acceptance')
    fig.tight_layout(); fig.savefig(output/'03_ground_candidate.png',dpi=160); plt.close(fig)
    image,origin,height,slope,observed,blocked=rasterize(points,args.resolution,args.max_step,args.robot_height,args.max_slope)
    cv2.imwrite(str(output/'map.pgm'),image)
    (output/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','mode':'trinary','resolution':args.resolution,
        'origin':[float(origin[0]),float(origin[1]),0.], 'negate':0,'occupied_thresh':.65,'free_thresh':.25}))
    np.savez_compressed(output/'terrain.npz',height=height,slope_deg=slope,observed=observed,blocked=blocked)
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    axes[0].imshow(np.flipud(height),cmap='terrain'); axes[0].set_title('Local support height (m)')
    axes[1].imshow(np.flipud(slope),cmap='magma',vmin=0,vmax=60); axes[1].set_title('Neighbour slope (deg)')
    axes[2].imshow(image,cmap='gray',vmin=0,vmax=255); axes[2].set_title('Free / blocked / unknown')
    fig.tight_layout(); fig.savefig(output/'04_terrain_map.png',dpi=160); plt.close(fig)
    metadata={'raw_count_after_manual_exclusion':len(raw),'manually_excluded_count':excluded_count,'voxel_count':before,'clean_count':len(cloud.points),
              'removed_statistical_count':before-len(cloud.points),'planes':planes,'selected_ground':choice,
              'sensor_to_map':transform.tolist(),'registered_to_given_world':bool(args.world_transform),
              'map_shape':list(image.shape),'free_cells':int((image==254).sum()),
              'parameters':vars(args),'limitations':['Ground candidate uses sensor mounting prior; manually inspect figures.',
              'No automatic semantic identification of spurious objects.',
              '2D height representation loses stacked-road topology; conservative unknown/blocked cells remain.',
              'Provided-map queries require independently established coordinate registration.']}
    (output/'processing.json').write_text(json.dumps(metadata,indent=2))
    print('ground',choice,model,'map',image.shape,'free',metadata['free_cells'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cloud',required=True); p.add_argument('--output',default='qB_sentry/results/cloud_map')
    p.add_argument('--voxel',type=float,default=.05); p.add_argument('--std-ratio',type=float,default=2.5)
    p.add_argument('--resolution',type=float,default=.05); p.add_argument('--max-step',type=float,default=.12)
    p.add_argument('--robot-height',type=float,default=.6); p.add_argument('--max-slope',type=float,default=30.)
    p.add_argument('--seed',type=int,default=42); p.add_argument('--ground-plane',type=int)
    p.add_argument('--align-yaw',action='store_true'); p.add_argument('--world-transform')
    p.add_argument('--exclude-boxes',help='确认是异常物体后提供传感器坐标min/max框JSON；默认不删除语义物体')
    a=p.parse_args()
    if min(a.voxel,a.resolution,a.max_step,a.robot_height,a.std_ratio)<=0: p.error('尺寸/比例必须大于0')
    main(a)
