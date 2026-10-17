"""检查本人实拍输入是否齐备；不访问相机、不生成假距离、不执行标定。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def audit(images, manifest, cols=9, rows=6):
    directory=Path(images); photos=[]; sizes=set()
    if directory.exists():
        for file in sorted(directory.iterdir()):
            if file.suffix.lower() not in ('.png','.jpg','.jpeg','.bmp','.tif','.tiff'):continue
            image=cv2.imread(str(file))
            if image is None:
                photos.append({'file':file.name,'corners_found':False,'reason':'unreadable'});continue
            size=image.shape[1::-1];sizes.add(size)
            found,_=cv2.findChessboardCornersSB(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),(cols,rows))
            photos.append({'file':file.name,'size':list(size),'corners_found':bool(found)})
    samples=[]
    if manifest and Path(manifest).exists():
        records=json.loads(Path(manifest).read_text())
        if not isinstance(records,list):raise ValueError('测量清单须为列表')
        for i,record in enumerate(records,1):
            issues=[]
            try:
                distance=float(record.get('actual_distance_m'))
                if not np.isfinite(distance) or distance<=0:issues.append('invalid_actual_distance')
            except (TypeError,ValueError):issues.append('missing_actual_distance')
            if record.get('reference_metric') not in ('center_range_m','optical_z_m'):issues.append('missing_distance_reference')
            try:
                corners=np.asarray(record.get('corners_px'),float)
                if corners.shape!=(4,2) or not np.isfinite(corners).all():issues.append('missing_four_corners')
            except (TypeError,ValueError):issues.append('invalid_corners')
            file=Path(record.get('image',''))
            file=file if file.is_absolute() else Path(manifest).parent/file
            if not file.is_file():issues.append('image_missing')
            samples.append({'sample':i,'issues':issues})
    count=sum(item['corners_found'] for item in photos)
    result={'calibration_images':photos,'valid_chessboard_count':count,'resolution_consistent':len(sizes)<=1,
            'calibration_input_ready':15<=count<=25 and len(sizes)==1,
            'measurement_samples':samples,'valid_measurement_count':sum(not s['issues'] for s in samples),
            'needs_user':['本人实际测量方格边长','至少5组约0.5–3m真实距离与对应照片',
                          '统一镜头/变焦/分辨率和距离参考点','由本人填写学习总结'],
            'note':'Readiness checks do not verify image ownership, ruler truth, pose diversity, or metric accuracy.'}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--images',default='q2_calib/input/chessboard');p.add_argument('--manifest',default='q2_calib/measurements.json')
    p.add_argument('--cols',type=int,default=9);p.add_argument('--rows',type=int,default=6)
    p.add_argument('--output',default='q2_calib/results/input_readiness.json')
    args=p.parse_args()
    if min(args.cols,args.rows)<2:p.error('内角点行列至少2')
    report=audit(args.images,args.manifest,args.cols,args.rows)
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print('棋盘有效图:',report['valid_chessboard_count'],'测距有效记录:',report['valid_measurement_count'])
    print(out)
