"""选做：状态机的离线二维仿真。阈值仅为可解释演示参数，不是比赛官方规则。"""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from search import search


def decide(hp,ammo,enemy,seconds_left,outpost_alive):
    if hp<30: return 'RETREAT'
    if ammo<10: return 'RESUPPLY'
    if not outpost_alive or seconds_left<30: return 'DEFEND'
    if enemy is not None: return 'INVESTIGATE'
    return 'PATROL'


def simulate(output,steps=240,seed=42):
    rng=np.random.default_rng(seed); out=Path(output); out.mkdir(parents=True,exist_ok=True)
    free=np.ones((30,50),bool); free[[0,-1],:]=False; free[:,[0,-1]]=False
    free[8:23,24:27]=False; resolution=.4
    # 本演示网格已视作底盘中心可通行图，非从真实点云推断。
    current=(25,5); home=(25,5); supply=(24,12); defend=(22,18); patrol=(5,40)
    trajectory=[]; hp=80.; ammo=60.; path=[]; previous=None
    for t in range(steps):
        enemy=(6,38) if 20<=t<60 or 150<=t<190 else None
        # 显式事件驱动测试各状态分支，不把它伪装成真实战场数据。
        if t==45: hp=20
        if t==85: hp=85; ammo=0
        if t==125: ammo=60
        outpost_alive=t<180
        state=decide(hp,ammo,enemy,steps-t,outpost_alive)
        target={'RETREAT':home,'RESUPPLY':supply,'DEFEND':defend,'INVESTIGATE':enemy,'PATROL':patrol}[state]
        if state!=previous or not path:
            result=search(free,current,target,resolution,'astar')
            path=result['path'][1:] if result['path'] else []
        if path: current=path.pop(0)
        if current==home and state=='RETREAT': hp=min(100,hp+5)
        if current==supply and state=='RESUPPLY': ammo=min(100,ammo+10)
        if enemy is not None and np.linalg.norm(np.subtract(current,enemy))*resolution<4:
            hp=max(0,hp-float(rng.uniform(.2,.6)))
        trajectory.append({'t_s':t,'x_m':current[1]*resolution,'y_m':(free.shape[0]-1-current[0])*resolution,
                           'hp':hp,'ammo':ammo,'enemy_visible':enemy is not None,'seconds_left':steps-t,
                           'outpost_alive':outpost_alive,'state':state,'path_found':bool(path) or current==target})
        previous=state
    with (out/'simulation.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=trajectory[0]); w.writeheader(); w.writerows(trajectory)
    fig,axes=plt.subplots(1,2,figsize=(12,5)); axes[0].imshow(np.flipud(free),cmap='gray',origin='lower',extent=[0,20,0,12])
    axes[0].plot([r['x_m'] for r in trajectory],[r['y_m'] for r in trajectory],'-r'); axes[0].set_title('Offline FSM trajectory (scripted events)')
    names=['RETREAT','RESUPPLY','DEFEND','INVESTIGATE','PATROL']
    axes[1].step([r['t_s'] for r in trajectory],[names.index(r['state']) for r in trajectory],where='post')
    axes[1].set_yticks(range(5),names); axes[1].set_xlabel('time (s)'); fig.tight_layout(); fig.savefig(out/'simulation.png',dpi=160); plt.close(fig)
    (out/'state_machine.md').write_text('''# 演示状态机
每次评估按从上到下的优先级。所有阈值为演示参数。
```mermaid
flowchart TD
  S[读取血量 弹量 敌位置 剩余时间 前哨状态] --> H{血量低于30?}
  H -->|是| R[撤退补血]
  H -->|否| A{弹量低于10?}
  A -->|是| L[补给]
  A -->|否| D{前哨已失或剩余时间少于30秒?}
  D -->|是| F[回防]
  D -->|否| E{发现敌方位置?}
  E -->|是| I[观察区域移动]
  E -->|否| P[巡逻]
```
状态机适于少量状态和明确转移；行为树更便于组合可复用子任务。小型LLM可解释高层意图，但存在延迟、不确定输出和错误状态推断，不能替代确定性边界与碰撞检查。本程序仅离线移动可视化。
''')
    print(out/'simulation.csv')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--output',default='qB_sentry/results/decision'); p.add_argument('--steps',type=int,default=240); p.add_argument('--seed',type=int,default=42)
    a=p.parse_args()
    if a.steps<1:p.error('steps必须为正')
    simulate(a.output,a.steps,a.seed)
