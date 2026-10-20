"""重现B题点云分层/局部观察；需先运行extended_study生成忽略缓存。"""
from pathlib import Path
import json
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parents[1]/'qB_sentry/results/extended'
p=np.load(OUT/'leveled_cache.npz')['points']
fig,axs=plt.subplots(2,3,figsize=(15,9))
for ax,(lo,hi) in zip(axs.flat,[(-.1,.12),(.12,.3),(.3,.6),(.6,1.2),(1.2,2.5),(2.5,20)]):
 b=p[(p[:,2]>=lo)&(p[:,2]<hi)];b=b[::max(1,len(b)//70000)]
 ax.scatter(b[:,0],b[:,1],s=.15);ax.set_aspect('equal');ax.set(xlim=(-4,27),ylim=(-3,15),title=f'{lo} <= z < {hi} m');ax.grid()
fig.tight_layout();fig.savefig(OUT/'height_slices.png',dpi=140);plt.close(fig)
a=p[(p[:,0]>-1.8)&(p[:,0]<25.8)&(p[:,1]>-1.7)&(p[:,1]<13.1)&(p[:,2]>.65)&(p[:,2]<4)]
c=o3d.geometry.PointCloud();c.points=o3d.utility.Vector3dVector(a);labels=np.array(c.cluster_dbscan(eps=.18,min_points=8));rows=[]
for lab in sorted(set(labels)):
 if lab<0:continue
 b=a[labels==lab]
 if len(b)<70:continue
 rows.append({'label':int(lab),'count':len(b),'min':b.min(0).tolist(),'max':b.max(0).tolist(),'center':b.mean(0).tolist()})
rows.sort(key=lambda r:-r['count']);(OUT/'cluster_candidates.json').write_text(json.dumps(rows,indent=2))
fig,ax=plt.subplots(figsize=(14,8));s=ax.scatter(a[:,0],a[:,1],c=a[:,2],s=.4,vmin=.65,vmax=4)
for r in rows:ax.text(*r['center'][:2],str(r['label']),color='red')
ax.set_aspect('equal');ax.set(xlabel='map x m',ylabel='map y m',title='Interior structures above 0.65m, cluster labels');fig.colorbar(s);fig.tight_layout();fig.savefig(OUT/'interior_clusters.png',dpi=150);plt.close(fig)
fig=plt.figure(figsize=(16,9))
for i,(x0,x1,y0,y1) in enumerate([(14.5,20,5.5,7.4),(-1.8,.5,-.5,2),(7.8,9.8,2,4.5),(14,16,8,10)]):
 b=p[(p[:,0]>x0)&(p[:,0]<x1)&(p[:,1]>y0)&(p[:,1]<y1)&(p[:,2]<2)&(p[:,2]>-.1)]
 ax=fig.add_subplot(2,2,i+1,projection='3d');ax.scatter(b[:,0],b[:,1],b[:,2],c=b[:,2],s=1,cmap='turbo',vmin=0,vmax=1.5);ax.set(xlabel='x',ylabel='y',zlabel='z',title=str((x0,x1,y0,y1)));ax.set_box_aspect((x1-x0,y1-y0,2))
fig.tight_layout();fig.savefig(OUT/'object_roi_views.png',dpi=150);plt.close(fig)
print('Saved cloud inspection evidence; clusters are geometric candidates, not entity labels.')
