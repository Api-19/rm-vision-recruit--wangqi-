"""栅格、坐标与碰撞。世界坐标以米计；数组索引为(row, col)。"""
from dataclasses import dataclass
from pathlib import Path
import math
import cv2
import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt


@dataclass
class Grid:
    image: np.ndarray
    resolution: float
    origin: np.ndarray                 # 地图左下角世界坐标与朝向角
    free: np.ndarray

    @classmethod
    def load(cls, path):
        path = Path(path)
        cfg = yaml.safe_load(path.read_text())
        if cfg.get('mode', 'trinary') != 'trinary':
            raise ValueError('本程序只支持 trinary 地图')
        image = cv2.imread(str(path.parent / cfg['image']), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f'无法读取地图: {path}')
        res = float(cfg['resolution'])
        if not np.isfinite(res) or res <= 0:
            raise ValueError('resolution 必须大于0')
        origin=np.asarray(cfg['origin'],float)
        free_threshold=float(cfg['free_thresh']); occupied_threshold=float(cfg['occupied_thresh'])
        if origin.shape!=(3,) or not np.isfinite(origin).all(): raise ValueError('origin必须为有限的x,y,yaw')
        if not 0<=free_threshold<occupied_threshold<=1: raise ValueError('阈值须满足0<=free<occupied<=1')
        if cfg.get('negate',0) not in (0,1): raise ValueError('negate只能为0或1')
        p = image.astype(float) / 255
        if not cfg.get('negate', 0):
            p = 1 - p
        # 与本题检查器一致，阈值相等时不认为可通行。
        return cls(image, res, origin, p < free_threshold)

    @property
    def rotation(self):
        a = self.origin[2]
        return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])

    def local(self, xy):
        return (np.asarray(xy, float) - self.origin[:2]) @ self.rotation

    def world_to_cell(self, xy):
        x, y = self.local(xy) / self.resolution
        return self.image.shape[0] - 1 - math.floor(y), math.floor(x)

    def cell_to_world(self, rc):
        r, c = rc
        xy = np.array([c + .5, self.image.shape[0] - r - .5]) * self.resolution
        return xy @ self.rotation.T + self.origin[:2]

    def inside(self, rc):
        r, c = rc
        return 0 <= r < self.free.shape[0] and 0 <= c < self.free.shape[1]

    def safe_cells(self, radius):
        """保守膨胀：占据格是有面积的方块，扣掉半格对角线。"""
        if not np.isfinite(radius) or radius < 0:
            raise ValueError('半径不能为负')
        padded = np.pad(self.free, 1, constant_values=False)
        distance = distance_transform_edt(padded)[1:-1, 1:-1] * self.resolution
        safe = self.free & (distance > radius + self.resolution / math.sqrt(2))
        rr, cc = np.indices(self.free.shape)
        edge = np.minimum.reduce([cc + .5, self.free.shape[1] - cc - .5,
                                  rr + .5, self.free.shape[0] - rr - .5]) * self.resolution
        return safe & (edge > radius)

    def disk_clear(self, xy, radius):
        """精确检查圆与附近占据/未知方格的相交，包含整个底盘的边界余量。"""
        xy=np.asarray(xy,float)
        if xy.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(radius) or radius<0:
            raise ValueError('坐标必须是有限二维向量，半径须非负有限')
        x, y = self.local(xy)
        res = self.resolution
        h, w = self.free.shape
        if min(x, y, w * res - x, h * res - y) <= radius:
            return False
        c0, c1 = max(0, math.floor((x-radius)/res)), min(w-1, math.floor((x+radius)/res))
        j0, j1 = max(0, math.floor((y-radius)/res)), min(h-1, math.floor((y+radius)/res))
        for j in range(j0, j1+1):
            for c in range(c0, c1+1):
                if self.free[h-1-j, c]:
                    continue
                # 点到轴对齐方格的最短距离。
                dx = max(c*res-x, 0., x-(c+1)*res)
                dy = max(j*res-y, 0., y-(j+1)*res)
                if math.hypot(dx, dy) <= radius:
                    return False
        return True

    def segment_clear(self, a, b, radius):
        """以加大半径的离散检测覆盖整段：每点负责前后半个采样间隔。"""
        a, b = np.asarray(a), np.asarray(b)
        n = max(1, math.ceil(np.linalg.norm(b-a)/(self.resolution/3)))
        half_step = np.linalg.norm(b-a)/n/2
        return all(self.disk_clear(p, radius+half_step) for p in np.linspace(a, b, n+1))


def dense_path(points, max_step=.10):
    result = [np.asarray(points[0], float)]
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(np.linalg.norm(np.asarray(b)-a)/max_step))
        result.extend(np.linspace(a, b, n+1)[1:])
    return np.asarray(result)
