"""自行实现的Dijkstra/A*。两者共用更新逻辑，仅启发式不同。"""
import heapq
import math
import time
import numpy as np


def search(safe, start, goal, resolution, algorithm='astar', heuristic='octile', weight=1.):
    if algorithm not in ('astar', 'dijkstra') or heuristic not in ('octile', 'euclidean', 'manhattan'):
        raise ValueError('未知搜索算法或启发式')
    if weight < 0:
        raise ValueError('weight不能为负')
    begin = time.perf_counter()
    h, w = safe.shape
    def valid(p):
        return 0 <= p[0] < h and 0 <= p[1] < w and safe[p]
    if not valid(start) or not valid(goal):
        return {'path': None, 'reason': 'start_or_goal_not_in_safe_grid', 'expanded': 0,
                'seconds': time.perf_counter()-begin, 'cost_m': None, 'order': []}
    def estimate(p):
        if algorithm == 'dijkstra':
            return 0.
        dr, dc = abs(p[0]-goal[0]), abs(p[1]-goal[1])
        if heuristic == 'manhattan':
            value = dr+dc             # 8邻居下可能高估，实验用，不保证最短。
        elif heuristic == 'euclidean':
            value = math.hypot(dr, dc)
        else:
            value = max(dr, dc)+(math.sqrt(2)-1)*min(dr, dc)
        return weight*value*resolution
    queue = [(estimate(start), 0., start)]
    costs = np.full(safe.shape, np.inf)
    costs[start] = 0.
    parents, expanded_order = {}, []
    while queue:
        _, old_cost, current = heapq.heappop(queue)
        if old_cost > costs[current]+1e-12:
            continue                  # 该队列项已被更便宜的路线替代。
        expanded_order.append(current)
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parents[path[-1]])
            return {'path': path[::-1], 'reason': 'ok', 'expanded': len(expanded_order),
                    'seconds': time.perf_counter()-begin, 'cost_m': float(costs[goal]),
                    'order': expanded_order}
        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nxt = (r+dr, c+dc)
            if not valid(nxt):
                continue
            if dr and dc and (not safe[r+dr,c] or not safe[r,c+dc]):
                continue              # 禁止斜着穿过两个障碍之间的角缝。
            proposal = old_cost+math.hypot(dr,dc)*resolution
            if proposal+1e-12 < costs[nxt]:
                costs[nxt] = proposal
                parents[nxt] = current
                heapq.heappush(queue, (proposal+estimate(nxt), proposal, nxt))
    return {'path': None, 'reason': 'disconnected_on_conservative_grid',
            'expanded': len(expanded_order), 'seconds': time.perf_counter()-begin,
            'cost_m': None, 'order': expanded_order}
