"""棋盘格标定：检测→内参/畸变→逐图重投影误差→去畸变对照。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def object_points(cols, rows, square_m):
    points=np.zeros((cols*rows,3),np.float32)
    points[:,:2]=np.mgrid[:cols,:rows].T.reshape(-1,2)*square_m
    return points


def fit(objects, corners, size):
    rms,K,dist,rotations,translations=cv2.calibrateCamera(objects,corners,tuple(size),None,None)
    errors=[]; inclinations=[]
    for obj,img,r,t in zip(objects,corners,rotations,translations):
        predicted,_=cv2.projectPoints(obj,r,t,K,dist)
        errors.append(float(np.sqrt(np.mean(np.sum((img.reshape(-1,2)-predicted.reshape(-1,2))**2,axis=1)))))
        R,_=cv2.Rodrigues(r)
        inclinations.append(float(np.degrees(np.arccos(np.clip(abs(R[2,2]),0,1)))))
    return {'camera_matrix':K.tolist(),'distortion_coefficients':dist.reshape(-1).tolist(),
            'image_size':[int(v) for v in size],'rms_px':float(rms),'per_view_rms_px':errors,
            'view_inclination_deg':inclinations,
            'rvecs':[r.reshape(-1).tolist() for r in rotations],
            'tvecs_m':[t.reshape(-1).tolist() for t in translations]}


def calibrate(input_dir, output, cols, rows, square_mm, allow_small=False):
    if min(cols,rows)<2 or square_mm<=0: raise ValueError('内角点行列至少2；方格边长须为正')
    input_dir=Path(input_dir); out=Path(output); out.mkdir(parents=True,exist_ok=True)
    files=sorted(p for p in input_dir.iterdir() if p.suffix.lower() in ('.jpg','.jpeg','.png','.bmp','.tif','.tiff'))
    accepted=[]; points=[]; skipped=[]; size=None
    for f in files:
        img=cv2.imread(str(f))
        if img is None:
            skipped.append({'file':f.name,'reason':'unreadable'}); continue
        current=img.shape[1::-1]
        if size is not None and current!=size:
            raise ValueError(f'{f.name}分辨率{current}与其他图{size}不同；禁止静默缩放')
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ok,corners=cv2.findChessboardCornersSB(gray,(cols,rows))
        if not ok:
            ok,corners=cv2.findChessboardCorners(gray,(cols,rows))
            if ok:
                corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),
                    (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001))
        if ok:
            size=current; points.append(corners.astype(np.float32)); accepted.append(str(f.resolve()))
            cv2.drawChessboardCorners(img,(cols,rows),corners,ok)
            cv2.imwrite(str(out/f'corners_{f.stem}.png'),img)
        else:
            skipped.append({'file':f.name,'reason':'corners_not_found'})
    (out/'detection_log.json').write_text(json.dumps({'accepted':accepted,'skipped':skipped},indent=2,ensure_ascii=False))
    minimum=3 if allow_small else 15
    if len(points)<minimum:
        raise ValueError(f'仅{len(points)}张有效图，需至少{minimum}张。先检查角点图，不生成伪标定。')
    obj=object_points(cols,rows,square_mm/1000)
    result=fit([obj]*len(points),points,size)
    result.update(cols=cols,rows=rows,square_mm=square_mm,images=accepted,
                  note='RMS is fit error, not ground-truth metric accuracy; inspect pose diversity.')
    (out/'calibration.json').write_text(json.dumps(result,indent=2))
    np.savez_compressed(out/'observations.npz',object_points=obj,image_points=np.array(points),image_size=size)
    # 保持图像尺寸和同一个K，只去畸变，便于对照；不把这张图拿原始畸变参数重复校正。
    img=cv2.imread(accepted[0]); K=np.asarray(result['camera_matrix']); dist=np.asarray(result['distortion_coefficients'])
    undistorted=cv2.undistort(img,K,dist,None,K)
    cv2.imwrite(str(out/'undistort_comparison.png'),np.hstack([img,undistorted]))
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    axes[0].bar(range(len(points)),result['per_view_rms_px']); axes[0].set(xlabel='view',ylabel='RMS (px)')
    axes[1].bar(range(len(points)),result['view_inclination_deg']); axes[1].set(xlabel='view',ylabel='plane inclination (deg)')
    fig.tight_layout(); fig.savefig(out/'calibration_quality.png',dpi=150); plt.close(fig)
    print(f'accepted={len(points)}, RMS={result["rms_px"]:.4f}px; {out/"calibration.json"}')
    return result


def compare(observations, output, square_mm, wrong_square_mm, frontal_deg):
    data=np.load(observations); obj=data['object_points']; points=data['image_points']; size=data['image_size']
    out=Path(output); out.mkdir(parents=True,exist_ok=True)
    if square_mm<=0 or wrong_square_mm<=0: raise ValueError('边长必须为正')
    # 由缓存真实坐标推断实际边长，防止命令行标签与已有数据不符。
    spacing=float(np.min(np.linalg.norm(obj[1:]-obj[0],axis=1)))
    if not np.isclose(spacing,square_mm/1000,rtol=1e-4): raise ValueError('square-mm与原始标定坐标不符')
    full=fit([obj]*len(points),list(points),size)
    variants={'all':full}
    if len(points)>=5: variants['first_5']=fit([obj]*5,list(points[:5]),size)
    front=np.where(np.array(full['view_inclination_deg'])<=frontal_deg)[0]
    if len(front)>=3:
        variants['frontal_only']=fit([obj]*len(front),list(points[front]),size)
        variants['frontal_only']['selected_indices']=front.tolist()
    else:
        variants['frontal_only']={'status':'insufficient_actual_frontal_views','count':len(front)}
    variants['wrong_square']=fit([obj*(wrong_square_mm/square_mm)]*len(points),list(points),size)
    variants['wrong_square']['input_square_mm']=wrong_square_mm
    (out/'comparison.json').write_text(json.dumps(variants,indent=2))
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for name,value in variants.items():
        if 'rms_px' not in value: continue
        axes[0].scatter(name,value['rms_px']); axes[1].scatter(name,value['camera_matrix'][0][0])
    axes[0].set_ylabel('fit RMS (px)'); axes[1].set_ylabel('fx (px)')
    fig.tight_layout(); fig.savefig(out/'comparison.png',dpi=160); plt.close(fig)
    print('Compared supplied observations. Wrong board scale changes translation scale, not intrinsic pixel scale.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); sub=p.add_subparsers(dest='command',required=True)
    c=sub.add_parser('fit'); c.add_argument('--images',required=True); c.add_argument('--output',default='q2_calib/results/main')
    c.add_argument('--cols',type=int,default=9); c.add_argument('--rows',type=int,default=6)
    c.add_argument('--square-mm',type=float,required=True); c.add_argument('--allow-small',action='store_true')
    c=sub.add_parser('compare'); c.add_argument('--observations',required=True); c.add_argument('--output',default='q2_calib/results/variants')
    c.add_argument('--square-mm',type=float,required=True); c.add_argument('--wrong-square-mm',type=float,required=True)
    c.add_argument('--frontal-deg',type=float,default=10.)
    a=p.parse_args()
    if a.command=='fit': calibrate(a.images,a.output,a.cols,a.rows,a.square_mm,a.allow_small)
    else: compare(a.observations,a.output,a.square_mm,a.wrong_square_mm,a.frontal_deg)
