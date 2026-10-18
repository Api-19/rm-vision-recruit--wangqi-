"""标定质量诊断：留一图检查、模型复杂度和未观测边缘外推。

不把低重投影误差当作真实测距准确；不自动删除高误差照片。
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def reprojection_rms(predicted, observed):
    a=np.asarray(predicted,float).reshape(-1,2)
    b=np.asarray(observed,float).reshape(-1,2)
    if a.shape!=b.shape or not len(a):raise ValueError('角点形状必须匹配且非空')
    return float(np.sqrt(np.mean(np.sum((a-b)**2,axis=1))))


def radial_check(K,D,size,observations):
    """在同一归一化坐标中比较观测范围和全幅范围，检查径向折叠红旗。

    此检查不含切向项，不替代完整成像Jacobian或真实尺测。
    """
    K,D=np.asarray(K),np.asarray(D).ravel()
    width,height=size
    corners=np.array([[0,0],[width-1,0],[width-1,height-1],[0,height-1]])
    scale=np.array([K[0,0],K[1,1]])
    maximum=float(np.linalg.norm((corners-K[:2,2])/scale,axis=1).max())
    seen=float(np.linalg.norm((np.asarray(observations).reshape(-1,2)-K[:2,2])/scale,axis=1).max())
    radius=np.linspace(0,maximum,1000);v=radius**2
    derivative=1+3*D[0]*v+5*D[1]*v**2+7*D[4]*v**3
    return {'full_frame_max_normalized_radius':maximum,'observed_max_normalized_radius':seen,
            'min_radial_derivative_full_frame':float(derivative.min()),
            'radial_monotone_full_frame':bool(np.all(derivative>0))}


def diagnose(observations,output):
    data=np.load(observations);points=data['image_points'];obj=data['object_points'];size=tuple(int(v) for v in data['image_size'])
    if len(points)<4:raise ValueError('留一图诊断至少需要4个视图')
    models=[('full5',0),('k1k2',cv2.CALIB_FIX_K3),('k1',cv2.CALIB_FIX_K2|cv2.CALIB_FIX_K3)]
    results=[]
    for name,flags in models:
        rms,K,D,_,_=cv2.calibrateCamera([obj]*len(points),list(points),size,None,None,flags=flags)
        errors=[];focals=[]
        for i in range(len(points)):
            train=[p for j,p in enumerate(points) if j!=i]
            _,kk,dd,_,_=cv2.calibrateCamera([obj]*len(train),train,size,None,None,flags=flags)
            ok,r,t=cv2.solvePnP(obj,points[i],kk,dd)
            if not ok:raise ValueError(f'视图{i}的留一姿态解算失败')
            predicted,_=cv2.projectPoints(obj,r,t,kk,dd)
            errors.append(reprojection_rms(predicted,points[i]));focals.append([kk[0,0],kk[1,1]])
        result={'model':name,'flags':int(flags),'rms_px':float(rms),'K':K.tolist(),'D':D.ravel().tolist(),
                'leave_one_view_out_rms_px':errors,'leave_one_view_out_mean_px':float(np.mean(errors)),
                'leave_one_view_out_max_px':float(max(errors)),'loo_fx_range':float(np.ptp(np.asarray(focals)[:,0]))}
        result.update(radial_check(K,D,size,points));results.append(result)
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    (out/'model_diagnostics.json').write_text(json.dumps(results,indent=2))
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for r in results:
        axes[0].plot(range(1,len(points)+1),r['leave_one_view_out_rms_px'],'o-',label=r['model'])
        radius=np.linspace(0,r['full_frame_max_normalized_radius'],1000);v=radius**2;d=r['D']
        axes[1].plot(radius,1+3*d[0]*v+5*d[1]*v**2+7*d[4]*v**3,label=r['model'])
    axes[0].set(xlabel='Held-out image',ylabel='Reprojection RMS (px)')
    axes[1].axhline(0,color='black',ls='--');axes[1].axvline(results[0]['observed_max_normalized_radius'],color='gray',ls=':',label='max observed radius')
    axes[1].set(xlabel='Normalized image radius',ylabel='Radial mapping derivative')
    for ax in axes:ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'model_diagnostics.png',dpi=160);plt.close(fig)
    for result in results:
        print(result['model'],'fit_RMS',result['rms_px'],'LOO_mean',result['leave_one_view_out_mean_px'],
              'radial_monotone_full_frame',result['radial_monotone_full_frame'])
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--observations',required=True)
    p.add_argument('--output',default='q2_calib/results/diagnostics');a=p.parse_args();diagnose(a.observations,a.output)
