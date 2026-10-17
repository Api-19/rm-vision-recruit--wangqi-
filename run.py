"""统一入口；所有输出留在代码项目内，原始考核材料只读。"""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('task',choices=['env','planning','cloud','decision','test','all'])
    p.add_argument('--data-root',type=Path,default=Path.home()/'Downloads/给新生 2')
    a=p.parse_args(); source=a.data_root.resolve()
    commands={
        'env':['q1_env/check_environment.py'],
        'planning':['qB_sentry/plan.py','--map',str(source/'data/map/rmuc_2025.yaml'),
                    '--queries',str(source/'data/map/queries.json'),'--checker',str(source/'tools/check_path.py'),'--experiments'],
        'cloud':['qB_sentry/pointcloud_map.py','--cloud',str(source/'data/rmuc_2025_map.pcd'),'--align-yaw'],
        'decision':['qB_sentry/decision_sim.py'],
        'test':['-m','unittest','discover','-s','tests','-v'],
    }
    for task in (['env','test','planning','cloud','decision'] if a.task=='all' else [a.task]):
        print(f'Running {task}',flush=True)
        subprocess.run([sys.executable,*commands[task]],cwd=ROOT,check=True)


if __name__=='__main__':main()
