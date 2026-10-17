"""选做：用人工确认的同名地标配准自建图，再在同一世界坐标规划比较。

pairs JSON: [{"source":[x,y],"reference":[x,y]}, ...]，至少3组米制坐标。
不能因为两张图像大小相近就把它们当作同坐标系。
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
from grid import Grid
from plan import run


def rigid_fit(source,reference):
    a,b=np.asarray(source,float),np.asarray(reference,float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=2 or len(a)<3: raise ValueError('至少3对二维同名点')
    if not np.isfinite(a).all() or not np.isfinite(b).all(): raise ValueError('地标必须有限')
    if np.linalg.matrix_rank(a-a.mean(0))<2 or np.linalg.matrix_rank(b-b.mean(0))<2: raise ValueError('地标不能全部共线')
    u,_,vt=np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)))
    R=vt.T@u.T
    if np.linalg.det(R)<0:
        vt[-1]*=-1; R=vt.T@u.T
    t=b.mean(0)-a.mean(0)@R.T
    rms=float(np.sqrt(np.mean(np.sum((a@R.T+t-b)**2,axis=1))))
    return R,t,rms


def compare_transform(source_map, reference_map, rotation, translation, output, queries=None, evidence=None):
    """将指定刚性变换用于栅格重采样；evidence说明来自人工地标还是自动估计。"""
    src,ref=Grid.load(source_map),Grid.load(reference_map)
    R,t=np.asarray(rotation,float),np.asarray(translation,float)
    if R.shape!=(2,2) or t.shape!=(2,) or not np.isfinite(R).all() or not np.isfinite(t).all():
        raise ValueError('配准变换必须为有限2D旋转与平移')
    if not np.allclose(R.T@R,np.eye(2),atol=1e-6) or not np.isclose(np.linalg.det(R),1,atol=1e-6):
        raise ValueError('配准矩阵必须是det=1的旋转')
    evidence={} if evidence is None else evidence
    out=Path(output); out.mkdir(parents=True,exist_ok=True)
    rr,cc=np.indices(ref.image.shape)
    local=np.stack([cc+.5,ref.image.shape[0]-rr-.5],axis=-1)*ref.resolution
    world=local@ref.rotation.T+ref.origin[:2]
    src_world=(world-t)@R
    src_local=src.local(src_world)/src.resolution
    cols=np.floor(src_local[...,0]); rows=src.image.shape[0]-1-np.floor(src_local[...,1])
    src_config=yaml.safe_load(Path(source_map).read_text())
    # 固定205在其他negate/阈值下可能被当作空闲；边界使用该配置下的未知值。
    p_unknown=(float(src_config['free_thresh'])+float(src_config['occupied_thresh']))/2
    gray_unknown=p_unknown if src_config.get('negate',0) else 1-p_unknown
    border=int(round(gray_unknown*255))
    # 若阈值窄于8bit能表达的间隔，则用占据值兜底，不能让图外成为空闲。
    border_probability=border/255 if src_config.get('negate',0) else 1-border/255
    if border_probability<float(src_config['free_thresh']):
        border=255 if src_config.get('negate',0) else 0
    mapped=cv2.remap(src.image,cols.astype(np.float32),rows.astype(np.float32),cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=border)
    cv2.imwrite(str(out/'registered.pgm'),mapped)
    src_config=yaml.safe_load(Path(source_map).read_text())
    config={'image':'registered.pgm','mode':'trinary','resolution':ref.resolution,'origin':ref.origin.tolist(),'negate':src_config.get('negate',0),'free_thresh':src_config['free_thresh'],'occupied_thresh':src_config['occupied_thresh']}
    (out/'registered.yaml').write_text(yaml.safe_dump(config))
    transform=np.eye(4); transform[:2,:2]=R; transform[:2,3]=t
    (out/'registration.json').write_text(json.dumps({'source_world_to_reference_world':transform.tolist(),'evidence':evidence},indent=2))
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    axes[0].imshow(ref.image,cmap='gray',vmin=0,vmax=255); axes[0].set_title('Provided map')
    axes[1].imshow(mapped,cmap='gray',vmin=0,vmax=255); axes[1].set_title('Registered generated map')
    overlay=np.zeros((*ref.image.shape,3),np.uint8); overlay[...,0]=ref.image; overlay[...,1]=mapped
    axes[2].imshow(overlay); axes[2].set_title('Red / green overlay; inspect registration evidence')
    fig.tight_layout(); fig.savefig(out/'registration.png',dpi=150); plt.close(fig)
    if queries:
        generated=run(out/'registered.yaml',queries,out/'generated_paths')
        reference=run(reference_map,queries,out/'reference_paths')
        comparison=[]
        for a,b in zip(generated,reference):
            comparison.append({'query':a['query'],'algorithm':a['algorithm'],'generated_length_m':a['length_m'],
                               'reference_length_m':b['length_m'],'generated_status':a['reason'],'reference_status':b['reason']})
        (out/'path_comparison.json').write_text(json.dumps(comparison,indent=2))
    print('Inspect registration.png; map agreement is not ground truth.')
    return out/'registered.yaml'


def register(source_map,reference_map,pairs,output,queries=None):
    records=json.loads(Path(pairs).read_text())
    R,t,rms=rigid_fit([p['source'] for p in records],[p['reference'] for p in records])
    if rms>.15: raise ValueError(f'地标配准RMS={rms:.3f}m过大；核对同名点')
    return compare_transform(source_map,reference_map,R,t,output,queries,
                             {'method':'supplied_landmarks','landmark_rms_m':rms,'pairs':records})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-map',required=True); p.add_argument('--reference-map',required=True)
    p.add_argument('--pairs',required=True); p.add_argument('--queries'); p.add_argument('--output',default='qB_sentry/results/registered_comparison')
    a=p.parse_args(); register(a.source_map,a.reference_map,a.pairs,a.output,a.queries)
