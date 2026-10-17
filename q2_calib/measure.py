"""在原始畸变图上标注A4四角；平面PnP→中心距离/光轴深度→真实尺测比较。"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def a4_points():
    # 纸张中心为物体系原点，故||t||表示光心到纸中心距离。
    return np.array([[-.105,-.1485,0],[.105,-.1485,0],[.105,.1485,0],[-.105,.1485,0]],np.float64)


def pose(corners,K,dist):
    corners=np.asarray(corners,np.float64)
    if corners.shape!=(4,2) or not np.isfinite(corners).all(): raise ValueError('须有4个有限二维角点')
    contour=corners.astype(np.float32)
    if not cv2.isContourConvex(contour.reshape(-1,1,2)) or abs(cv2.contourArea(contour))<25:
        raise ValueError('角点顺序交叉/面积过小；按同一纸张物理角TL TR BR BL标注')
    K=np.asarray(K,float); dist=np.asarray(dist,float)
    if K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0:
        raise ValueError('K须为有限的3×3矩阵且焦距为正')
    if not np.allclose(K[2],[0,0,1]) or dist.size not in (4,5,8,12,14) or not np.isfinite(dist).all():
        raise ValueError('相机矩阵末行或畸变参数不合法')
    obj=a4_points()
    ok,rs,ts,_=cv2.solvePnPGeneric(obj,corners,K,dist,flags=cv2.SOLVEPNP_IPPE)
    candidates=[]
    if ok:
        for r,t in zip(rs,ts):
            R,_=cv2.Rodrigues(r)
            if np.min((obj@R.T+t.reshape(3))[:,2])<=0: continue
            proj,_=cv2.projectPoints(obj,r,t,K,dist)
            error=float(np.sqrt(np.mean(np.sum((proj.reshape(-1,2)-corners)**2,axis=1))))
            candidates.append((error,r,t))
    if not candidates: raise ValueError('PnP无有效的相机前方解')
    candidates.sort(key=lambda item:item[0]); error,r,t=candidates[0]
    # 用最佳平面解初始化题目要求的solvePnP，迭代细化；保留平面双解诊断。
    refined,rr,tt=cv2.solvePnP(obj,corners,K,dist,rvec=r.copy(),tvec=t.copy(),
                              useExtrinsicGuess=True,flags=cv2.SOLVEPNP_ITERATIVE)
    if refined:
        rotation,_=cv2.Rodrigues(rr)
        uv,_=cv2.projectPoints(obj,rr,tt,K,dist)
        ee=float(np.sqrt(np.mean(np.sum((uv.reshape(-1,2)-corners)**2,axis=1))))
        if np.min((obj@rotation.T+tt.reshape(3))[:,2])>0 and ee<=error:
            error,r,t=ee,rr,tt
    R,_=cv2.Rodrigues(r)
    return {'center_range_m':float(np.linalg.norm(t)),'optical_z_m':float(t[2,0]),
            'translation_m':t.reshape(-1).tolist(),'rvec':r.reshape(-1).tolist(),
            'reprojection_rms_px':error,'estimated_inclination_deg':float(np.degrees(np.arccos(np.clip(abs(R[2,2]),0,1)))),
            'candidate_errors_px':[v[0] for v in candidates],
            'ambiguous':len(candidates)>1 and candidates[1][0]-error<.2}


def annotate(image,output):
    img=cv2.imread(image)
    if img is None: raise ValueError('图像无法读取')
    scale=min(1.,1400/img.shape[1],850/img.shape[0]); points=[]
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(points)<4: points.append([x/scale,y/scale])
    cv2.namedWindow('A4 corners'); cv2.setMouseCallback('A4 corners',click)
    print('按纸张物理TL TR BR BL点击（210mm宽、297mm高）；U撤销，S保存，Q取消。坐标来自原图。')
    try:
        while True:
            view=cv2.resize(img,None,fx=scale,fy=scale)
            for i,p in enumerate(points):
                x,y=np.round(np.array(p)*scale).astype(int)
                cv2.circle(view,(x,y),5,(0,0,255),-1); cv2.putText(view,['TL','TR','BR','BL'][i],(x+8,y),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
            cv2.imshow('A4 corners',view); k=cv2.waitKey(30)&255
            if k==ord('q'): return
            if k==ord('u') and points: points.pop()
            if k==ord('s') and len(points)==4:
                destination=Path(output); destination.parent.mkdir(parents=True,exist_ok=True)
                destination.write_text(json.dumps({'image':str(Path(image).resolve()),'corners_px':points},indent=2)); print(destination); return
    finally:
        cv2.destroyAllWindows()


def measure(calibration,manifest,output):
    cfg=json.loads(Path(calibration).read_text()); K=np.array(cfg['camera_matrix']); dist=np.array(cfg['distortion_coefficients'])
    manifest=Path(manifest); records=json.loads(manifest.read_text()); out=Path(output); out.mkdir(parents=True,exist_ok=True)
    if not records: raise ValueError('测量清单为空；须填写本人采集图像、角点及真实测距')
    results=[]
    for i,record in enumerate(records,1):
        file=Path(record['image']); file=file if file.is_absolute() else manifest.parent/file
        image=cv2.imread(str(file))
        if image is None: raise ValueError(f'图像无法读取: {file}')
        if list(image.shape[1::-1])!=cfg['image_size']: raise ValueError(f'{file}分辨率与标定不一致')
        if record.get('reference_metric') not in ('center_range_m','optical_z_m'):
            raise ValueError('reference_metric必须明确是center_range_m或optical_z_m')
        actual=float(record['actual_distance_m'])
        if not np.isfinite(actual) or actual<=0: raise ValueError('真实距离必须为正且有限')
        corners=np.asarray(record['corners_px'],float)
        if corners.shape!=(4,2) or not np.isfinite(corners).all(): raise ValueError('须有4个有限二维角点')
        if np.any(corners<0) or np.any(corners[:,0]>=image.shape[1]) or np.any(corners[:,1]>=image.shape[0]): raise ValueError('角点越界')
        result=pose(corners,K,dist); metric=record['reference_metric']
        result.update(image=str(file),actual_distance_m=actual,reference_metric=metric,
                      measured_angle_deg=record.get('angle_deg'),error_m=result[metric]-actual)
        result['relative_error_pct']=result['error_m']/actual*100
        # 对照不使用畸变系数的解，不能把前后两次解混成真值。
        result['without_distortion']=pose(corners,K,np.zeros_like(dist))
        cv2.polylines(image,[corners.astype(np.int32)],True,(0,0,255),2)
        for label,xy in zip(['TL','TR','BR','BL'],corners.astype(int)):
            cv2.putText(image,label,tuple(xy),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
        cv2.putText(image,f'range={result["center_range_m"]:.3f}m z={result["optical_z_m"]:.3f}m',(10,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,150,0),2)
        cv2.imwrite(str(out/f'measurement{i}.png'),image); results.append(result)
    (out/'measurements.json').write_text(json.dumps(results,indent=2))
    keys=['image','actual_distance_m','reference_metric','center_range_m','optical_z_m','error_m','relative_error_pct','reprojection_rms_px','measured_angle_deg','estimated_inclination_deg','ambiguous']
    with (out/'measurements.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore'); w.writeheader(); w.writerows(results)
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for metric in ['center_range_m','optical_z_m']:
        subset=[r for r in results if r['reference_metric']==metric]
        if subset:
            axes[0].scatter([r['actual_distance_m'] for r in subset],[r['error_m']*1000 for r in subset],label=metric)
    angles=[r for r in results if r['measured_angle_deg'] is not None]
    if angles: axes[1].scatter([r['measured_angle_deg'] for r in angles],[r['error_m']*1000 for r in angles])
    axes[0].set(xlabel='Actual distance (m)',ylabel='Signed error (mm)'); axes[0].axhline(0,color='gray'); axes[0].legend()
    axes[1].set(xlabel='Measured angle (deg)',ylabel='Signed error (mm)')
    fig.tight_layout(); fig.savefig(out/'measurement_errors.png',dpi=160); plt.close(fig)
    errors=np.array([r['error_m'] for r in results])
    stats={'count':len(results),'mae_m':float(np.mean(abs(errors))),'rmse_m':float(np.sqrt(np.mean(errors**2))),
           'has_at_least_5_samples':len(results)>=5,'note':'Sample count alone does not prove 0.5–3m coverage or genuine measurements.'}
    (out/'statistics.json').write_text(json.dumps(stats,indent=2)); print(stats)
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); s=p.add_subparsers(dest='command',required=True)
    a=s.add_parser('annotate'); a.add_argument('--image',required=True); a.add_argument('--output',required=True)
    a=s.add_parser('batch'); a.add_argument('--calibration',required=True); a.add_argument('--manifest',required=True); a.add_argument('--output',default='q2_calib/results/measurements')
    a=p.parse_args()
    if a.command=='annotate': annotate(a.image,a.output)
    else: measure(a.calibration,a.manifest,a.output)
