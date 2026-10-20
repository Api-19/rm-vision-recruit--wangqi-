import heapq
import math
from scipy import ndimage
import cv2
import json
import numpy as np
from pathlib import Path
import yaml
import sys
def load_map(path):
    path=Path(path)
    cfg=read_config(path)
    folder=path.parent
    name=cfg['image']
    result=folder/name
    image=cv2.imread(str(result),cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError('cant read image')  
    free=make_free_mask(image,cfg['negate'],cfg['free_thresh'])
    return image,cfg,free 

def read_config(path):
    path=Path(path)
    a=path.read_text()
    b=yaml.safe_load(a)
    return b

def make_free_mask(image,negate,free_thresh):
    a=np.array(image,dtype=np.uint8)
    b=a.astype(float)/255
    if negate==0:
        p=1-b
    else:
        p=b
    return p<free_thresh

def cell_center_world(row,col,height,resolution,origin_xy):
    origin_x,origin_y=origin_xy
    x=origin_x+(col+0.5)*resolution
    y=origin_y+(height-row-0.5)*resolution
    return x,y

def world_to_cell(x,y,height,width,resolution,origin_xy):
    origin_x,origin_y=origin_xy
    i=(x-origin_x)/resolution
    j=(y-origin_y)/resolution
    col=math.floor(i)
    j=math.floor(j)
    row=height-1-j
    if row>=0 and row<height and col>=0 and col<width:
        return (row,col)
    else:
        return None

def neighbors8(safe,row,col,resolution):
    height,width=safe.shape
    selected=[]
    directions=[(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    for dr,dc in directions:
        neighbor=(row+dr,col+dc)
        side_vertical=(row+dr,col)
        side_horizontal=(row,col+dc)
        if 0<=neighbor[0]<height and 0<=neighbor[1]<width:
            if safe[neighbor[0],neighbor[1]]:
                if dr!=0 and dc!=0:
                    if safe[side_vertical[0],side_vertical[1]] and safe[side_horizontal[0],side_horizontal[1]]:
                        cost=math.sqrt(2)*resolution
                        selected.append((neighbor,cost))
                else:
                    selected.append((neighbor,resolution))
    return selected

def dijkstra(safe, start, goal, resolution):
    dist={start:0}
    parent={}
    done=set()
    heap=[(0,start)]
    expanded=0
    while heap:
        g,cur=heapq.heappop(heap)
        if cur in done:
            continue
        done.add(cur)
        expanded+=1
        if cur==goal:
            break
        for nb,step_cost in neighbors8(safe,cur[0],cur[1],resolution):
            new_g=g+step_cost
            if nb not in dist or new_g<dist[nb]:
                dist[nb]=new_g
                parent[nb]=cur
                heapq.heappush(heap, (new_g, nb))
    if goal not in done:
        return None,None,expanded
    path=[goal]
    while path[-1]!=start:
        path.append(parent[path[-1]])
    path.reverse()
    return path,dist[goal],expanded


def heuristic(a, b, resolution):
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)) * resolution

def astar(safe, start, goal, resolution):
    dist={start:0}
    parent={}
    done=set()
    heap=[(0,0,start)]
    expanded=0
    while heap:
        f,g,cur=heapq.heappop(heap)
        if cur in done:
            continue
        done.add(cur)
        expanded+=1
        if cur==goal:
            break
        for nb,step_cost in neighbors8(safe,cur[0],cur[1],resolution):
            new_g=g+step_cost
            if nb not in dist or new_g<dist[nb]:
                h=heuristic(nb, goal, resolution)
                f=new_g+h
                dist[nb]=new_g
                parent[nb]=cur
                heapq.heappush(heap, (f,new_g, nb))
    if goal not in done:
        return None,None,expanded
    path=[goal]
    while path[-1]!=start:
        path.append(parent[path[-1]])
    path.reverse()
    return path,dist[goal],expanded

map_path = sys.argv[1]       
queries_path = sys.argv[2]   
image, cfg, free = load_map(map_path)
resolution=cfg['resolution']
d = ndimage.distance_transform_edt(free) * resolution
safe = d >= 0.30
queries=json.load(open(queries_path,'r'))
for i,q in enumerate(queries['queries'],1):
    start=world_to_cell(q['start'][0], q['start'][1], image.shape[0], image.shape[1], resolution, cfg['origin'][:2])
    goal=world_to_cell(q['goal'][0],q['goal'][1], image.shape[0], image.shape[1], resolution, cfg['origin'][:2])
    if start is None or goal is None:
        print(f'第{i}组:start or goal is out of bounds')
        continue
    elif safe[start[0],start[1]]==False or safe[goal[0],goal[1]]==False:
        print(f'第{i}组:start or goal is in obstacle')
        continue
    else:
        path, cost, expanded = astar(safe, start, goal, resolution)
        if path is None:
            print(f'第{i}组:no path found')
        else:
            with open(f'A*path{i}.csv','w') as fp:
                fp.write('x,y\n')
                for cell in path:
                    x,y=cell_center_world(cell[0],cell[1],image.shape[0],resolution,cfg['origin'][:2])
                    fp.write(f'{x:.4f},{y:.4f}\n')
                print(f'A*第{i}组:cost={cost:.4f},expanded={expanded},path saved to A*path{i}.csv')

for i,q in enumerate(queries['queries'],1):
    start=world_to_cell(q['start'][0], q['start'][1], image.shape[0], image.shape[1], resolution, cfg['origin'][:2])
    goal=world_to_cell(q['goal'][0],q['goal'][1], image.shape[0], image.shape[1], resolution, cfg['origin'][:2])
    if start is None or goal is None:
        print(f'第{i}组:start or goal is out of bounds')
        continue
    elif safe[start[0],start[1]]==False or safe[goal[0],goal[1]]==False:
        print(f'第{i}组:start or goal is in obstacle')
        continue
    else:
        path, cost, expanded = dijkstra(safe, start, goal, resolution)
        if path is None:
            print(f'第{i}组:no path found')
        else:
            with open(f'dijkstra_path{i}.csv','w') as fp:
                fp.write('x,y\n')
                for cell in path:
                    x,y=cell_center_world(cell[0],cell[1],image.shape[0],resolution,cfg['origin'][:2])
                    fp.write(f'{x:.4f},{y:.4f}\n')
                print(f'dijkstra第{i}组:cost={cost:.4f},expanded={expanded},path saved to dijkstra_path{i}.csv')