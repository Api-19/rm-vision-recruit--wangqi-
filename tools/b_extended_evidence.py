"""补充B题发挥报告所需统计与离线动画；不修改查询或地图。"""
from pathlib import Path
import csv,json,sys
import numpy as np
from PIL import Image,ImageDraw
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'qB_sentry'))
from grid import Grid
from search import search

out=ROOT/'qB_sentry/results/extended'
ref=Grid.load(Path.home()/'Downloads/给新生 2/data/map/rmuc_2025.yaml')
queries=json.loads((Path.home()/'Downloads/给新生 2/data/map/queries.json').read_text())['queries']
safe=ref.safe_cells(.3);rows=[]
for trial in range(3):
 for number in [1,2,4,5]:
  query=queries[number-1]
  for algorithm in ['astar','dijkstra']:
   r=search(safe,ref.world_to_cell(query['start']),ref.world_to_cell(query['goal']),ref.resolution,algorithm)
   rows.append({'query':number,'trial':trial+1,'algorithm':algorithm,'expanded':r['expanded'],'seconds':r['seconds'],'grid_cost_m':r['cost_m']})
with (out/'search_repeated.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=rows[0],lineterminator='\n');w.writeheader();w.writerows(rows)
fig,axs=plt.subplots(1,3,figsize=(13,4));summary=[]
for alg in ['astar','dijkstra']:
 group=[r for r in rows if r['algorithm']==alg]
 for j,key in enumerate(['expanded','seconds','grid_cost_m']):
  vals=[np.median([r[key] for r in group if r['query']==i]) for i in [1,2,4,5]]
  axs[j].plot([1,2,4,5],vals,'o-',label=alg);axs[j].set(xlabel='query',ylabel=key);axs[j].legend()
 for i in [1,2,4,5]:
  g=[r for r in group if r['query']==i];summary.append({'query':i,'algorithm':alg,'median_seconds':float(np.median([r['seconds'] for r in g])),'expanded':g[0]['expanded'],'grid_cost_m':g[0]['grid_cost_m']})
fig.suptitle('3 repeated trials; same map/start/goal/radius; grid cost excludes endpoint connectors');fig.tight_layout();fig.savefig(out/'search_repeated.png',dpi=150);plt.close(fig)
(out/'search_repeated_summary.json').write_text(json.dumps(summary,indent=2))
for i in [1,2,4,5]:
 costs=[r['grid_cost_m'] for r in rows if r['query']==i];assert max(costs)-min(costs)<1e-8
with (out/'decision/simulation.csv').open() as f:data=list(csv.DictReader(f))
frames=[]
for index in range(0,len(data),4):
 r=data[index];im=Image.new('RGB',(800,500),'white');d=ImageDraw.Draw(im)
 d.rectangle((30,60,630,420),fill='#eeeeee',outline='black');d.rectangle((318,144,354,324),fill='#333333')
 for a,b in zip(data[:index],data[1:index+1]):
  d.line((30+float(a['x_m'])*30,420-float(a['y_m'])*30,30+float(b['x_m'])*30,420-float(b['y_m'])*30),fill='#ed4b35',width=2)
 x=30+float(r['x_m'])*30;y=420-float(r['y_m'])*30;d.ellipse((x-5,y-5,x+5,y+5),fill='blue')
 d.text((30,20),f'Offline scripted FSM, t={r["t_s"]}s state={r["state"]}',fill='black')
 d.text((650,100),f'HP {float(r["hp"]):.1f}\nammo {float(r["ammo"]):.1f}\noutpost {r["outpost_alive"]}\nleft {r["seconds_left"]}s',fill='black')
 d.text((30,450),'Synthetic navigation scenario, not a real robot experiment',fill='black');frames.append(im)
frames[0].save(out/'decision/simulation.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
print('Repeated search costs match; FSM animation saved.')
