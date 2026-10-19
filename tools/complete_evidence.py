"""保存真实数据对比摘要、搜索回放与运行证据图；不生成实拍/尺测真值。"""
from pathlib import Path
import csv
import json
import sys
import textwrap
import numpy as np
from PIL import Image, ImageDraw, ImageFont
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'qB_sentry'))
from grid import Grid
from search import search


def log_image(source, output):
    lines = ['RUN OUTPUT RENDERING - not a desktop screenshot', source.name, '']
    lines += [part for line in source.read_text().splitlines() for part in (textwrap.wrap(line, width=125, replace_whitespace=False) or [''])]
    font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Andale Mono.ttf', 16)
    im = Image.new('RGB', (1350, max(180, 24*(len(lines)+2))), '#101820')
    d = ImageDraw.Draw(im)
    for i,line in enumerate(lines): d.text((20,20+24*i),line,fill='#e4efe8',font=font)
    im.save(output)


def main():
    folder=ROOT/'evidence/2026-10-19'
    folder.mkdir(parents=True,exist_ok=True)
    comparison=ROOT/'q2_calib/results/display_27mm_19views_2026-10-18/variants/comparison.json'
    data=json.loads(comparison.read_text())
    full=data['all']; wrong=data['wrong_square']
    rows=[]
    for label,value in data.items():
        if 'rms_px' not in value: continue
        k=np.asarray(value['camera_matrix'])
        rows.append([label,len(value['per_view_rms_px']),value['rms_px'],k[0,0],k[1,1],k[0,2],k[1,2]])
    out=comparison.parent
    with (out/'summary.csv').open('w',newline='') as f:
        writer=csv.writer(f,lineterminator='\n'); writer.writerow(['variant','views','rms_px','fx_px','fy_px','cx_px','cy_px']); writer.writerows(rows)
    ratios=np.linalg.norm(wrong['tvecs_m'],axis=1)/np.linalg.norm(full['tvecs_m'],axis=1)
    table='\n'.join('| '+ ' | '.join(str(round(x,5)) if isinstance(x,float) else str(x) for x in row)+' |' for row in rows)
    text=f'''# 19张实拍标定发挥对比

同一批真实检测角点，原始方格27mm；仅改变子集或输入尺度。由Agent运行，不代表学生独立完成。

| 条件 | 图数 | 拟合RMS px | fx px | fy px | cx px | cy px |
|---|---:|---:|---:|---:|---:|---:|
{table}

仅5张会减少姿态/空间约束；低拟合误差不证明全视场泛化。近正对子集按全量拟合的估计倾角≤10°选择，并非另采一整套严格正对数据；该条件的结果为 `{data['frontal_only'].get('status','fitted_subset')}`。正对视图深度与焦距约束弱、缺少倾斜变化会使参数不稳定，不能用本小样本推导普遍误差大小。

错误边长实验将27mm输入为20mm，预期平移缩放20/27={20/27:.8f}，实际逐图平移模长比范围{ratios.min():.8f}–{ratios.max():.8f}；fx变化{wrong['camera_matrix'][0][0]-full['camera_matrix'][0][0]:.6f}px。棋盘尺度与外参平移存在尺度自由度，不能声称错误棋盘边长必然按同比例改变fx或使用正确A4尺寸的PnP距离。

此处RMS是各自训练视图拟合误差，不是共同验证集误差，也不是尺测误差。A4至少5组真实距离、同距0°/30°/60°和误差曲线仍待采集。
'''
    (out/'REPORT.md').write_text(text)
    grid=Grid.load(Path.home()/'Downloads/给新生 2/data/map/rmuc_2025.yaml')
    cfg=json.loads((Path.home()/'Downloads/给新生 2/data/map/queries.json').read_text())
    q=cfg['queries'][0]; safe=grid.safe_cells(.3)
    start,goal=grid.world_to_cell(q['start']),grid.world_to_cell(q['goal'])
    records={a:search(safe,start,goal,grid.resolution,a) for a in ['astar','dijkstra']}
    frames=[]
    for progress in np.linspace(0,1,36):
        panels=[]
        for algorithm,result in records.items():
            rgb=np.repeat(grid.image[:,:,None],3,axis=2)
            order=np.asarray(result['order']); count=int(len(order)*progress)
            selected=order[:count]
            if count: rgb[selected[:,0],selected[:,1]]=[50,140,235]
            if progress==1 and result['path']:
                path=np.asarray(result['path']); rgb[path[:,0],path[:,1]]=[240,50,60]
            panel=Image.new('RGB',(600,370),'white')
            panel.paste(Image.fromarray(rgb).resize((583,324)),(8,38))
            ImageDraw.Draw(panel).text((10,10),f'{algorithm}: {count}/{len(order)} expansions',fill='black')
            panels.append(panel)
        frame=Image.new('RGB',(1200,370),'white'); frame.paste(panels[0],(0,0)); frame.paste(panels[1],(600,0)); frames.append(frame)
    frames[0].save(folder/'search_query1.gif',save_all=True,append_images=frames[1:],duration=120,loop=0)
    stats={k:{key:v for key,v in r.items() if key not in ['path','order']} for k,r in records.items()}
    (folder/'search_query1.json').write_text(json.dumps(stats,indent=2))
    for source in [folder/'tests.txt',folder/'environment.txt']:
        if source.exists(): log_image(source,source.with_suffix('.png'))
    diagram=Image.new('RGB',(1050,470),'white')
    draw=ImageDraw.Draw(diagram)
    for i,(condition,state) in enumerate([('HP < 30','RETREAT'),('ammo < 10','RESUPPLY'),('outpost lost OR time < 30s','DEFEND'),('enemy position available','OBSERVE'),('otherwise','PATROL')]):
        y=50+i*78
        draw.rounded_rectangle((20,y,610,y+48),radius=8,outline='black',width=2)
        draw.text((35,y+15),condition,fill='black')
        draw.line((610,y+24,710,y+24),fill='black',width=2)
        draw.text((735,y+15),state,fill='black')
        if i<4: draw.line((300,y+48,300,y+78),fill='gray',width=2)
    draw.text((20,15),'Offline FSM: evaluate top-down every step; first matching condition wins',fill='black')
    diagram.save(folder/'decision_priority.png')
    print('Saved variants summary and real-query search playback. Frame progression is expansion fraction, not wall-clock speed.')


if __name__=='__main__': main()
