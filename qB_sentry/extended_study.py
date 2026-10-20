"""B题发挥的可复现实验：异常几何、调平对正、地形剖面和双假设比较。

来源：Open3D PointCloud官方接口、SciPy ndimage.label官方文档，2026-10-20核验。
API安装环境Open3D0.20.0、SciPy1.17.1实跑；文档release版本可能不同。
不输出人工语义真值，不为了查询通过改变端点或障碍。
"""
import argparse
import csv
import json
import math
from pathlib import Path
import hashlib
import cv2
import numpy as np
import open3d as o3d
from scipy.ndimage import label, median_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
from pointcloud_map import rasterize, align_normal
from grid import Grid, dense_path
from search import search
from auto_registration import boundary_points, estimate
from register_and_compare import compare_transform
from decision_sim import decide, simulate


def classify_height(height, resolution, max_step=.12, max_slope=30.):
    """局部高度面诊断。台阶与坡度独立计算，不以绝对高度拒绝高地。"""
    height=np.asarray(height,float)
    if height.ndim!=2 or min(height.shape)<3 or resolution<=0:
        raise ValueError('height至少3×3，resolution须正')
    valid=np.isfinite(height)
    # 无效格不凭空填入高度，只有自身及四邻域已知时估计坡度。
    h=median_filter(np.where(valid,height,0.),size=3)
    gy,gx=np.gradient(h,resolution)
    slope=np.degrees(np.arctan(np.hypot(gx,gy)))
    interior=valid.copy();interior[[0,-1],:]=False;interior[:,[0,-1]]=False
    for dr,dc in [(dr,dc) for dr in [-1,0,1] for dc in [-1,0,1]]:
        interior &= np.roll(np.roll(valid,dr,0),dc,1)
    slope[~interior]=np.nan
    step=np.zeros_like(valid)
    for axis in [0,1]:
        d=np.abs(np.diff(height,axis=axis))
        edge=np.isfinite(d)&(d>max_step)
        if axis==0:step[:-1]|=edge;step[1:]|=edge
        else:step[:,:-1]|=edge;step[:,1:]|=edge
    ramp=interior&~step&(slope>=3)&(slope<=max_slope)
    return {'slope_deg':slope,'step':step,'ramp_candidate':ramp,
            'too_steep':interior&(slope>max_slope)}


def save_map(image,origin,output,res=.05):
    output.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(output/'map.pgm'),image)
    config={'image':'map.pgm','mode':'trinary','resolution':res,'origin':[*map(float,origin),0.],
            'negate':0,'occupied_thresh':.65,'free_thresh':.25}
    (output/'map.yaml').write_text(yaml.safe_dump(config))


def write_json(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False))


def clean_points(raw,transform,std_ratio=2.5):
    cloud=o3d.geometry.PointCloud();cloud.points=o3d.utility.Vector3dVector(raw)
    voxel=cloud.voxel_down_sample(.05);v=np.asarray(voxel.points).copy()
    _,ids=voxel.remove_statistical_outlier(nb_neighbors=20,std_ratio=std_ratio)
    keep=np.zeros(len(v),bool);keep[ids]=True
    map_v=v@transform[:3,:3].T+transform[:3,3]
    # 在已识别最低场地平面以下15cm的孤立几何异常。若换地面模型需复核此先验。
    below=map_v[:,2]<-.15
    accepted=keep&~below
    return map_v,accepted,{'voxel_points':len(v),'statistical_outliers':int((~keep).sum()),
                         'below_ground_points':int(below.sum()),'union_removed':int((~accepted).sum()),
                         'retained':int(accepted.sum()),'below_ground_threshold_m':-.15,
                         'std_ratio':std_ratio}


def axis_evidence(meta,transform,points,out):
    plane_rows=[]
    for i,plane in enumerate(meta['planes'][:4]):
        normal=np.asarray(plane['coefficients'][:3]);n=transform[:3,:3]@normal
        direction=math.degrees(math.atan2(n[1],n[0]))
        error=abs((direction+45)%90-45)
        plane_rows.append({'plane':i,'normal_after':n.tolist(),'axis_residual_deg':error})
    write_json(out/'axis_alignment.json',{'walls':plane_rows,'axis_ambiguous_by_180_degrees':True,
               'note':'Directions, not unique world orientation. Planes extracted from same cloud, not independent ground truth.'})
    selected=points[(points[:,2]>.8)&(points[:,2]<1.5)]
    normal=meta['planes'][meta['selected_ground']]['coefficients'][:3]
    yaw_rotation=transform[:3,:3]@align_normal(normal).T
    R=yaw_rotation[:2,:2]
    fig,axs=plt.subplots(1,2,figsize=(12,6))
    for ax,xy,title in [(axs[0],selected[:,:2]@R,'Leveled cloud before yaw alignment'),(axs[1],selected[:,:2],'Axis-aligned XY, measured wall normals')]:
        ax.scatter(xy[::3,0],xy[::3,1],s=.2);ax.set_aspect('equal');ax.set_title(title);ax.set(xlabel='x m',ylabel='y m');ax.grid()
    fig.tight_layout();fig.savefig(out/'axis_alignment.png',dpi=150);plt.close(fig)


def terrain_evidence(height,origin,image,out):
    terrain=classify_height(height,.05)
    np.savez_compressed(out/'terrain_cache.npz',height=height,**terrain)
    extent=[origin[0],origin[0]+height.shape[1]*.05,origin[1],origin[1]+height.shape[0]*.05]
    fig,axs=plt.subplots(1,3,figsize=(16,5))
    a=axs[0].imshow(height,origin='lower',extent=extent,cmap='terrain');fig.colorbar(a,ax=axs[0],label='height m')
    a=axs[1].imshow(terrain['slope_deg'],origin='lower',extent=extent,cmap='magma',vmin=0,vmax=45);fig.colorbar(a,ax=axs[1],label='slope deg')
    rgb=np.ones((*height.shape,3))*.8
    rgb[np.isfinite(height)]=[.9,.9,.9];rgb[terrain['ramp_candidate']]=[0,.65,.25];rgb[terrain['step']]=[.9,.1,.05]
    axs[2].imshow(rgb,origin='lower',extent=extent)
    for ax,title in zip(axs,['Local surface','Robust local slope','Green: slope candidates; red: >12cm steps']):ax.set_title(title);ax.set(xlabel='x m',ylabel='y m')
    fig.tight_layout();fig.savefig(out/'terrain_classes.png',dpi=140);plt.close(fig)
    # 数据驱动选较大连续坡面供审查，不把候选标签当独立真值。
    labs,n=label(terrain['ramp_candidate']);regions=[]
    for i in range(1,n+1):
        rc=np.argwhere(labs==i)
        if len(rc)<15:continue
        values=height[labs==i];coords=rc[:,::-1]*.05+origin+.025
        plane=np.linalg.lstsq(np.c_[coords,np.ones(len(coords))],values,rcond=None)[0]
        residual=float(np.sqrt(np.mean((np.c_[coords,np.ones(len(coords))]@plane-values)**2)))
        regions.append({'plane_slope_deg':float(np.degrees(np.arctan(np.linalg.norm(plane[:2])))),'plane_fit_rms_m':residual,'label':i,'cells':len(rc),'area_m2':len(rc)*.0025,'center_xy':coords.mean(0).tolist(),
                        'height_span_m':float(np.ptp(values)),'median_slope_deg':float(np.nanmedian(terrain['slope_deg'][labs==i]))})
    regions.sort(key=lambda r:-r['cells']);write_json(out/'ramp_candidates.json',regions)
    # 固定真实数据剖面，不根据是否通路筛选。用来区分缓坡与跳变。
    profiles=[('ramp_A',[21.765,2.3],[21.765,3.5]),('step_edge',[7.925,2.675],[8.575,2.675]),
              ('ramp_B',[11.625,.35],[11.625,.85])]
    fig,axs=plt.subplots(1,3,figsize=(14,4));stats=[]
    for ax,(name,a,b) in zip(axs,profiles):
        a,b=np.asarray(a),np.asarray(b);t=np.linspace(0,1,100);xy=a+(b-a)*t[:,None]
        rc=np.floor((xy-origin)/.05).astype(int);valid=(rc[:,0]>=0)&(rc[:,0]<height.shape[1])&(rc[:,1]>=0)&(rc[:,1]<height.shape[0]);z=np.full(100,np.nan);z[valid]=height[rc[valid,1],rc[valid,0]]
        distance=t*np.linalg.norm(b-a);ax.plot(distance,z,'.-');ax.set(title=name,xlabel='distance along section m',ylabel='height m');ax.grid()
        grid=Grid.load(out/'cloud_map/map.yaml')
        free_fraction=sum(grid.free[grid.world_to_cell(p)] for p in xy)/len(xy)
        disk_fraction=sum(grid.disk_clear(p,.3) for p in xy)/len(xy)
        stats.append({'free_center_fraction':float(free_fraction),'disk_clear_fraction':float(disk_fraction),'name':name,'from':a.tolist(),'to':b.tolist(),'samples':100,'known_samples':int(np.isfinite(z).sum()),'max_adjacent_sample_jump_m':float(np.nanmax(abs(np.diff(z))))})
    fig.tight_layout();fig.savefig(out/'terrain_profiles.png',dpi=150);plt.close(fig)
    write_json(out/'terrain_evidence.json',{'ramp_cells':int(terrain['ramp_candidate'].sum()),'step_cells':int(terrain['step'].sum()),'profiles':stats,'semantics':'geometric candidates; inspect profiles; no per-cell independent labels'})
    return terrain


def decision_evidence(out):
    scenarios=[('low_hp',20,80,None,100,True,'RETREAT'),('low_ammo',80,5,None,100,True,'RESUPPLY'),
               ('outpost_lost',80,80,None,100,False,'DEFEND'),('time_low',80,80,None,20,True,'DEFEND'),
               ('enemy_seen',80,80,(3,4),100,True,'INVESTIGATE'),('normal',80,80,None,100,True,'PATROL'),
               ('hp_over_outpost',20,80,(3,4),20,False,'RETREAT')]
    rows=[]
    for name,hp,ammo,enemy,time,outpost,expected in scenarios:
        actual=decide(hp,ammo,enemy,time,outpost);assert actual==expected
        rows.append({'scenario':name,'expected':expected,'actual':actual,'passed':actual==expected})
    simulate(out/'decision')
    with (out/'decision/simulation.csv').open() as f: states=list(csv.DictReader(f))
    transitions=[]
    for a,b in zip(states,states[1:]):
        if a['state']!=b['state']:transitions.append({'time_s':b['t_s'],'from':a['state'],'to':b['state']})
    assert len(set(r['state'] for r in states))==5
    write_json(out/'decision/scenarios.json',rows);write_json(out/'decision/transitions.json',transitions)


def main(args):
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    meta=json.loads(Path(args.metadata).read_text());T=np.array(meta['sensor_to_map'])
    original=o3d.io.read_point_cloud(args.cloud);raw=np.asarray(original.points);raw=raw[np.isfinite(raw).all(1)]
    points,keep,counts=clean_points(raw,T);clean=points[keep]
    write_json(out/'anomaly_counts.json',counts)
    np.savez_compressed(out/'leveled_cache.npz',points=points,accepted=keep)
    fig,axs=plt.subplots(1,3,figsize=(15,5))
    for ax,data,title in [(axs[0],points,'Before'),(axs[1],points[~keep],'Removed geometric anomalies'),(axs[2],clean,'After')]:
        subset=data[::max(1,len(data)//40000)];ax.scatter(subset[:,0],subset[:,1],s=.25,c=subset[:,2],cmap='terrain',vmin=0,vmax=2);ax.set_aspect('equal');ax.set_title(title);ax.set(xlabel='x m',ylabel='y m')
    fig.tight_layout();fig.savefig(out/'anomalies_before_after.png',dpi=140);plt.close(fig)
    image,origin,height,slope,observed,blocked=rasterize(clean)
    save_map(image,origin,out/'cloud_map')
    loaded=Grid.load(out/'cloud_map/map.yaml');assert np.array_equal(loaded.free,image==254)
    write_json(out/'map_semantics.json',{'pixels':{str(i):int((image==i).sum()) for i in np.unique(image)},'free_matches_254':True,'unknown_probability':1-128/255,'old_205_probability':1-205/255})
    axis_evidence(meta,T,clean,out);terrain_evidence(height,origin,image,out)
    source=Grid.load(out/'cloud_map/map.yaml');ref=Grid.load(args.reference_map)
    candidates=estimate(boundary_points(source),boundary_points(ref),angles=(0,180))
    write_json(out/'registration_candidates.json',{'candidates':candidates,'human_landmarks':False,'keep_both_hypotheses':True})
    for candidate in candidates:
        deg=candidate['initial_degrees'];folder=out/f'orientation_{deg}'
        compare_transform(out/'cloud_map/map.yaml',args.reference_map,candidate['rotation'],candidate['translation'],folder,args.queries,
                          {'method':'automatic_geometry_two_hypotheses','candidate':candidate,'human_landmarks':False})
    # 自建地图内另选安全连通分量的两个远点，验证搜索链路能工作，而非移动题目查询端点。
    safe=source.safe_cells(.3);labs,n=label(safe);sizes=np.bincount(labs.ravel());sizes[0]=0
    component=int(sizes.argmax());rc=np.argwhere(labs==component)
    a=rc[0];a=rc[np.argmax(np.linalg.norm(rc-a,axis=1))];b=rc[np.argmax(np.linalg.norm(rc-a,axis=1))]
    trial=search(safe,tuple(a),tuple(b),source.resolution)
    xy=dense_path([source.cell_to_world(x) for x in trial['path']])
    assert all(source.segment_clear(x,y,.3) for x,y in zip(xy,xy[1:]))
    np.savetxt(out/'local_demonstration_path.csv',xy,delimiter=',',header='x,y',comments='')
    fig,ax=plt.subplots(figsize=(10,6));ax.imshow(source.image,cmap='gray',vmin=0,vmax=255);path=np.asarray(trial['path']);ax.plot(path[:,1],path[:,0],'r-');ax.set_title('Additional local-map demonstration (not one of 7 exam queries)');fig.tight_layout();fig.savefig(out/'local_demonstration_path.png',dpi=150);plt.close(fig)
    write_json(out/'local_demonstration.json',{'not_an_exam_query':True,'start_world':source.cell_to_world(a).tolist(),'goal_world':source.cell_to_world(b).tolist(),'length_m':float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),'continuous_check_passed':True})
    decision_evidence(out)
    write_json(out/'run_provenance.json',{'source_sha256':hashlib.sha256(Path(args.cloud).read_bytes()).hexdigest(),'source_name':Path(args.cloud).name,'metadata':str(args.metadata),'args':{'resolution':.05,'radius_m':.3,'hypotheses':[0,180]},'note':'Agent-run study. No change to exam endpoints. No independent semantic or orientation ground truth.'})
    print('Extended study complete',counts,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cloud',required=True);p.add_argument('--reference-map',required=True);p.add_argument('--queries',required=True)
    p.add_argument('--metadata',default='qB_sentry/results/cloud_map/processing.json');p.add_argument('--output',default='qB_sentry/results/extended')
    main(p.parse_args())
