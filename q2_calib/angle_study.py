"""比较同一纸张不同姿态的PnP结果；严格区分用户标称条件与程序估计。

中心斜距有独立尺测才算误差；固定高度另算纸面垂距，不混淆采集条件。
"""
import argparse,csv,hashlib,json
from pathlib import Path
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from measure import pose


def plane_metrics(rvec,translation):
    """纸中心视线不一定沿光轴；垂距必须用法向量与平移的点积。"""
    R,_=cv2.Rodrigues(np.asarray(rvec,float))
    n=R[:,2];t=np.asarray(translation,float).reshape(3)
    perpendicular=float(abs(n@t));distance=float(np.linalg.norm(t))
    if distance<=0:raise ValueError('纸中心距离必须为正')
    theta=float(np.degrees(np.arccos(np.clip(perpendicular/distance,0,1))))
    return perpendicular,theta


def run(calibration,manifest,output):
    config=json.loads(Path(calibration).read_text());K=np.array(config['camera_matrix']);D=np.array(config['distortion_coefficients'])
    manifest=Path(manifest);items=json.loads(manifest.read_text());out=Path(output);out.mkdir(parents=True,exist_ok=True)
    results=[];rng=np.random.default_rng(20261020)
    for item in items:
        file=manifest.parent/item['image'];image=cv2.imread(str(file))
        if image is None or list(image.shape[1::-1])!=config['image_size']:raise ValueError('图像或分辨率不符')
        corners=np.asarray(item['corners_px'],float)
        if corners.shape!=(4,2) or np.any(corners<0) or np.any(corners[:,0]>=image.shape[1]) or np.any(corners[:,1]>=image.shape[0]):raise ValueError('四角不合法')
        value=pose(corners,K,D)
        perturb=[pose(corners+rng.uniform(-8,8,(4,2)),K,D)['center_range_m'] for _ in range(200)]
        perpendicular,center_angle=plane_metrics(value['rvec'],value['translation_m'])
        value.update(image=file.name,corners_px=corners.tolist(),reported_angle_deg=item['reported_angle_deg'],
                     reported_center_range_m=item.get('reported_center_range_m'),
                     reported_height_m=item.get('reported_height_m'),capture_status=item['capture_status'],
                     association=item['association'],annotation_source=item['annotation_source'],
                     perpendicular_plane_distance_m=perpendicular,center_ray_to_normal_angle_deg=center_angle,
                     perturbation_range_p05_p95_m=np.quantile(perturb,[.05,.95]).tolist())
        value['center_range_error_m']=None if item.get('reported_center_range_m') is None else value['center_range_m']-item['reported_center_range_m']
        value['height_error_m']=None if item.get('reported_height_m') is None else perpendicular-item['reported_height_m']
        value['without_distortion']=pose(corners,K,np.zeros_like(D));results.append(value)
    (out/'results.json').write_text(json.dumps(results,indent=2))
    keys=['image','reported_angle_deg','estimated_inclination_deg','center_ray_to_normal_angle_deg','reported_center_range_m','reported_height_m','center_range_m','optical_z_m','perpendicular_plane_distance_m','center_range_error_m','height_error_m','reprojection_rms_px','capture_status']
    with (out/'comparison.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore',lineterminator='\n');w.writeheader();w.writerows(results)
    fig,axs=plt.subplots(1,3,figsize=(15,4));labels=[Path(r['image']).stem for r in results]
    axs[0].bar(labels,[r['center_range_m']*100 for r in results]);axs[0].set_ylabel('PnP slant center range cm (not fixed at 50)')
    axs[1].bar(labels,[r['perpendicular_plane_distance_m']*100 for r in results]);axs[1].plot([1,2],[50,50],'r--',label='Reported new-view height 50cm');axs[1].set_ylabel('Perpendicular distance to paper plane cm');axs[1].legend(fontsize=8)
    axs[2].bar(labels,[r['estimated_inclination_deg'] for r in results]);axs[2].plot(labels,[r['reported_angle_deg'] for r in results],'ro',label='Reported nominal angle');axs[2].set_ylabel('Optical axis to plane normal deg');axs[2].legend(fontsize=8)
    fig.suptitle('Fixed-height capture confirmed; slant distances differ; not a fixed-slant-distance angle experiment');fig.tight_layout();fig.savefig(out/'pose_comparison.png',dpi=150);plt.close(fig)
    (out/'provenance.json').write_text(json.dumps({'calibration_sha256':hashlib.sha256(Path(calibration).read_bytes()).hexdigest(),
      'annotation_space':'original distorted pixels; landscape A4 short edge first',
      'sensitivity':'200 independent uniform +/-8px perturbations per corner coordinate; not a confidence interval',
      'status':'height_controlled_capture_confirmed; fixed_center_range_requirement_not_met','automatic_monitor':'remains paused'},indent=2))
    print('Saved',len(results),'poses; fixed height is distinguished from center slant distance.')
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--calibration',required=True);p.add_argument('--manifest',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();run(a.calibration,a.manifest,a.output)
