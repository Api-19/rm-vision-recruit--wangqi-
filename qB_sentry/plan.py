"""批量处理7组查询，输出两种算法、图表、独立校验和原始检查器日志。"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from grid import Grid, dense_path
from search import search


def run(map_path, queries_path, output, radius=None, checker=None, experiments=False):
    grid = Grid.load(map_path)
    cfg = json.loads(Path(queries_path).read_text())
    radius = float(cfg.get('robot_radius_m', .30) if radius is None else radius)
    safe = grid.safe_cells(radius)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    variants = [('astar','octile',1.), ('dijkstra','octile',1.)]
    if experiments:
        variants += [('astar','manhattan',1.), ('astar','octile',2.)]
    summary = []
    for number, query in enumerate(cfg['queries'],1):
        a, b = np.asarray(query['start'],float), np.asarray(query['goal'],float)
        for algorithm, heuristic, weight in variants:
            label = algorithm if heuristic=='octile' and weight==1 else f'{algorithm}_{heuristic}_w{weight:g}'
            folder = output/label
            folder.mkdir(exist_ok=True)
            path_file = folder/f'path{number}.csv'
            path_file.unlink(missing_ok=True)  # 避免旧成功路径冒充本次不可达结果。
            start, goal = grid.world_to_cell(a), grid.world_to_cell(b)
            result = search(safe,start,goal,grid.resolution,algorithm,heuristic,weight)
            row = {k:v for k,v in result.items() if k not in ('path','order')}
            row.update(query=number,algorithm=label,length_m=None,checker='not_run')
            if not grid.disk_clear(a,radius) or not grid.disk_clear(b,radius):
                result['path'] = None
                issues=[]
                for name,point,cell in [('start',a,start),('goal',b,goal)]:
                    if not grid.inside(cell): issues.append(name+'_outside_map')
                    elif not grid.disk_clear(point,radius): issues.append(name+'_disk_collides_or_crosses_boundary')
                row['reason'] = ';'.join(issues)
            if result['path']:
                centers = [grid.cell_to_world(rc) for rc in result['path']]
                # 保留原始查询端点，不偷偷吸附到附近空闲格。
                points = dense_path([a,*centers,b])
                segments_ok = all(grid.segment_clear(x,y,radius) for x,y in zip(points,points[1:]))
                if not segments_ok:
                    raise RuntimeError(f'查询{number}连续底盘校验失败；不导出不安全路径')
                np.savetxt(path_file, points, delimiter=',', header='x,y',comments='',fmt='%.8f')
                row['length_m'] = float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
                if checker:
                    check = subprocess.run([sys.executable,str(checker),str(Path(map_path).resolve()),str(path_file.resolve())],
                                           capture_output=True,text=True)
                    (folder/f'check{number}.txt').write_text(check.stdout+check.stderr)
                    row['checker'] = 'passed' if check.returncode==0 and check.stdout.rstrip().endswith('\n通过') else 'failed'
                    if row['checker'] != 'passed':
                        raise RuntimeError(f'原检查器拒绝查询{number}，请查看日志')
                if label=='astar':
                    np.savetxt(output/f'path{number}.csv',points,delimiter=',',header='x,y',comments='',fmt='%.8f')
            elif label=='astar':
                (output/f'path{number}.csv').unlink(missing_ok=True)
            fig, ax = plt.subplots(figsize=(10,6))
            ax.imshow(grid.image,cmap='gray',vmin=0,vmax=255)
            mask = np.zeros((*safe.shape,4)); mask[grid.free & ~safe] = (1,.4,0,.35)
            ax.imshow(mask)
            if result['order']:
                order=np.asarray(result['order']); step=max(1,len(order)//10000)
                ax.scatter(order[::step,1],order[::step,0],s=1,c=np.arange(len(order))[::step],cmap='viridis',alpha=.3)
            if result['path']:
                p=np.asarray(result['path']); ax.plot(p[:,1],p[:,0],'r-',lw=1)
            ax.scatter([start[1],goal[1]],[start[0],goal[0]],c=['lime','red'],s=25)
            ax.set_title(f'Query {number}: {label}, {row["reason"]}; expanded={row["expanded"]}')
            ax.set_xlim(-1,grid.image.shape[1]); ax.set_ylim(grid.image.shape[0],-1)
            fig.tight_layout(); fig.savefig(folder/f'search{number}.png',dpi=140); plt.close(fig)
            summary.append(row)
            print(number,label,row['reason'],row['length_m'],row['checker'],flush=True)
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    with (output/'summary.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(summary[0])); writer.writeheader(); writer.writerows(summary)
    # 对比只针对相同地图、相同查询；计时是单次实验，不当作稳定基准。
    reachable=[i for i in range(1,len(cfg['queries'])+1) if any(r['query']==i and r['length_m'] for r in summary)]
    if reachable:
        fig, axes=plt.subplots(1,3,figsize=(14,4))
        for label in dict.fromkeys(r['algorithm'] for r in summary):
            rows=[r for r in summary if r['algorithm']==label and r['query'] in reachable]
            for ax,key in zip(axes,['expanded','seconds','length_m']):
                ax.plot([r['query'] for r in rows],[r[key] for r in rows],'o-',label=label); ax.set_title(key); ax.set_xlabel('query')
        axes[0].legend(fontsize=7); fig.tight_layout(); fig.savefig(output/'comparison.png',dpi=160); plt.close(fig)
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',required=True); p.add_argument('--queries',required=True)
    p.add_argument('--output',default='qB_sentry/results/given_map')
    p.add_argument('--radius',type=float); p.add_argument('--checker')
    p.add_argument('--experiments',action='store_true',help='额外比较曼哈顿启发式及权重2，不保证最优')
    args=p.parse_args(); run(args.map,args.queries,args.output,args.radius,args.checker,args.experiments)
