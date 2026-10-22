#!/usr/bin/env python3
"""Kiểm thử nhanh lõi khám phá (scripts/frontier_core.py) ngoài ROS, trên bản đồ căn hộ đã lưu.

Mô hình lưới kiểu Explore-Bench Level-0 (Xu et al., ICRA 2022): định vị hoàn hảo, LiDAR 2D 360° ray-casting
(tầm = max_laser_range của slam_toolbox), quét sau mỗi 0.2 m, robot đi theo đường ngắn nhất trên vùng đã
biết là trống; thời gian = quãng đường / v + góc quay / w. KHÔNG mô phỏng Nav2/DWB, trôi SLAM hay độ trễ /map,
nên thời gian thật trong Gazebo sẽ dài hơn. Logic chọn mục tiêu / re-check / blacklist / kết thúc giống node.

  python3 tools/explore_level0.py                  # S1, S2 (config/trials.yaml) trên maps/home_sim_map
  python3 tools/explore_level0.py --random 10      # thêm 10 vị trí xuất phát ngẫu nhiên
Cần: numpy, scipy, pyyaml.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml
from scipy import ndimage

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import frontier_core as fc  # noqa: E402

V, W = 0.30, 0.8              # m/s, rad/s (nav2_exploration_params của robot thật)
LIDAR_RANGE = 10.0            # mapper_params_online_async.yaml: max_laser_range
SENSE_STEP = 0.2              # minimum_travel_distance
TICK = 1.0                    # tick_period của node
P = dict(robot_radius=0.30, approach_radius=0.6, min_goal_distance=0.4, recheck_radius=0.5,
         blacklist_radius=0.6, max_failures=2, min_frontier_size=8, done_patience=3,
         min_gain_cells=10)


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    raw = open(pgm, 'rb').read()
    hdr = [l for l in raw.split(b'\n')[:6] if not l.startswith(b'#')]
    w, h = map(int, hdr[1].split())
    img = np.flipud(np.frombuffer(raw[-w * h:], dtype=np.uint8).reshape(h, w))
    free = img >= 250                    # trinary: 254 = trống, 0 = chiếm chỗ, 205 = chưa biết
    return free, ~free, meta['resolution'], meta['origin'][0], meta['origin'][1]


class World(object):
    def __init__(self, free_gt, blocked_gt, res, ox, oy, start):
        self.blocked = blocked_gt
        # va chạm thật: ô có clearance THẬT < 0.25 m thì robot không vào được (Nav2 sẽ dừng/abort tại đó)
        self.gt_ok = ndimage.distance_transform_edt(free_gt) * res >= 0.25
        self.grid = fc.Grid(-np.ones(free_gt.shape, np.int16), res, ox, oy)
        self.r, self.c = self.grid.to_cell(start[0], start[1])
        self.yaw = start[2]
        self.dist = self.rot = 0.0
        ang = np.linspace(-np.pi, np.pi, 3200, endpoint=False)   # RPLIDAR S2E: 0.1125°
        steps = np.arange(0, LIDAR_RANGE / res, 0.5)
        self.dr, self.dc = np.outer(np.sin(ang), steps), np.outer(np.cos(ang), steps)
        self.sense()

    def xy(self):
        return self.grid.to_world(self.r, self.c)

    def time(self):
        return self.dist / V + self.rot / W

    def sense(self):
        g = self.grid.data
        h, w = g.shape
        rr, cc = np.rint(self.r + self.dr).astype(int), np.rint(self.c + self.dc).astype(int)
        oob = (rr < 0) | (rr >= h) | (cc < 0) | (cc >= w)
        rc, ccc = np.clip(rr, 0, h - 1), np.clip(cc, 0, w - 1)
        hit = self.blocked[rc, ccc] | oob
        first = np.where(hit.any(1), hit.argmax(1), hit.shape[1])
        before = np.arange(hit.shape[1])[None, :] < first[:, None]
        sel = g[rc[before], ccc[before]] != 100
        g[rc[before][sel], ccc[before][sel]] = 0
        i = np.nonzero(first < hit.shape[1])[0]
        hr, hc = rr[i, first[i]], cc[i, first[i]]
        ok = (hr >= 0) & (hr < h) & (hc >= 0) & (hc < w)
        g[hr[ok], hc[ok]] = 100

    def path_to(self, goal_xy):
        trav = fc.traversable(self.grid, (self.r, self.c), P['robot_radius'])
        gr, gc = self.grid.to_cell(*goal_xy)
        d = fc.geodesic_distance(trav, (gr, gc), self.grid.res)
        if not np.isfinite(d[self.r, self.c]):
            return None
        path, (r, c) = [], (self.r, self.c)
        while (r, c) != (gr, gc):
            nb = [(r + a, c + b) for a in (-1, 0, 1) for b in (-1, 0, 1)
                  if (a or b) and self.grid.inside(r + a, c + b)]
            r, c = min(nb, key=lambda p: d[p])
            path.append((r, c))
        return path


def explore(world, denom):
    bl = fc.Blacklist(P['blacklist_radius'], P['max_failures'])
    target, idle, trace = None, 0, []
    stats = dict(goals=0, failures=0, rechecks=0)

    def cov():
        return float(((world.grid.data == 0) & denom).sum()) / float(denom.sum())

    def fail(t, why):
        stats['failures'] += 1
        stats[why] = stats.get(why, 0) + 1
        bl.record_failure(t.centroid_xy)

    for _ in range(2000):
        fr = fc.detect_frontiers(world.grid, 20, P['min_frontier_size'])
        if not fr.clusters:
            return 'complete', stats, trace
        target, n_reach, _ = fc.select_target(world.grid, fr, world.xy(), bl, P['robot_radius'],
                                              P['approach_radius'], P['min_goal_distance'])
        if target is None:
            idle += 1
            if idle >= P['done_patience']:
                return 'no reachable frontier', stats, trace
            continue
        idle = 0
        stats['goals'] += 1
        known0 = fc.known_cells_near(world.grid, target.cells_xy)
        since_sense, reached, t_goal = 0.0, None, world.time()
        while reached is None:
            if world.time() - t_goal > 120.0:         # goal_timeout của node
                reached = False
                fail(target, 'goal_timeout')
                break
            # Nav2 lập lại đường mỗi chu kỳ trên bản đồ hiện tại (BT RateController 1 Hz)
            path = world.path_to(target.goal_xy)
            if path is None:
                reached = False
                fail(target, 'no_path')                          # không còn đường -> Nav2 abort
                break
            t_tick = world.time()
            for (r, c) in path:
                if not world.gt_ok[r, c]:             # đâm vào vật cản chưa thấy -> Nav2 abort
                    reached = False
                    world.sense()
                    fail(target, 'collision')
                    break
                step = math.hypot(r - world.r, c - world.c) * world.grid.res
                hd = math.atan2(r - world.r, c - world.c)
                world.rot += abs(math.atan2(math.sin(hd - world.yaw), math.cos(hd - world.yaw)))
                world.yaw, world.r, world.c = hd, r, c
                world.dist += step
                since_sense += step
                if since_sense >= SENSE_STEP:
                    world.sense()
                    since_sense = 0.0
                    trace.append((world.time(), cov()))
                if world.time() - t_tick >= TICK:     # re-check mỗi chu kỳ của node
                    fr = fc.detect_frontiers(world.grid, 20, P['min_frontier_size'])
                    if not fc.frontier_alive(world.grid, fr, target.cells_xy, P['recheck_radius']):
                        stats['rechecks'] += 1
                        reached = False
                    break
            else:
                reached = True

        world.sense()
        trace.append((world.time(), cov()))
        if reached:  # VERIFY: tới nơi, cụm vẫn mở và quanh cụm không có thêm ô đã biết -> thất bại
            world.rot += abs(math.atan2(math.sin(target.yaw - world.yaw), math.cos(target.yaw - world.yaw)))
            world.yaw = target.yaw
            fr = fc.detect_frontiers(world.grid, 20, P['min_frontier_size'])
            gain = fc.known_cells_near(world.grid, target.cells_xy) - known0
            if fc.frontier_alive(world.grid, fr, target.cells_xy, P['recheck_radius']) and gain < P['min_gain_cells']:
                fail(target, 'verify_no_gain')
    return 'max_iter', stats, trace


def observable(free_gt, blocked, res, ox, oy, start):
    """Mẫu số coverage: ô trống quan sát được từ C-space tới được (quét dày từ nhiều điểm)."""
    w = World(free_gt, blocked, res, ox, oy, start)
    clear = ndimage.distance_transform_edt(free_gt) * res
    lab, _ = ndimage.label(free_gt & (clear >= P['robot_radius']), structure=np.ones((3, 3)))
    comp = lab == lab[w.r, w.c]
    for r, c in np.argwhere(comp)[::25]:
        w.r, w.c = r, c
        w.sense()
    return w.grid.data == 0, comp


def main():
    pkg = os.path.join(HERE, '..')
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.join(pkg, 'maps', 'home_sim_map.yaml'))
    ap.add_argument('--trials', default=os.path.join(pkg, 'config', 'trials.yaml'))
    ap.add_argument('--random', type=int, default=0)
    a = ap.parse_args()
    free, blocked, res, ox, oy = load_map(a.map)
    # map frame = robot tại S1 (xem maps/home_sim_map.yaml): world (x, y) -> map (3 - x, -3 - y), yaw - pi
    starts = {}
    for s in yaml.safe_load(open(a.trials))['start_poses']:
        starts[s['id']] = (3.0 - s['x'], -3.0 - s['y'], s['yaw'] - math.pi)
    denom, comp = observable(free, blocked, res, ox, oy, starts['S1'])
    rng = np.random.RandomState(0)
    cells = np.argwhere(comp)
    for k in range(a.random):
        r, c = cells[rng.randint(len(cells))]
        starts['R%d' % k] = (c * res + ox + res / 2, r * res + oy + res / 2, rng.uniform(-math.pi, math.pi))
    print('observable free area %.1f m2' % (denom.sum() * res * res))
    print('start  end                    coverage  path_m  T_s   T90_s  T95_s  goals fail recheck')
    for name, st in starts.items():
        w = World(free, blocked, res, ox, oy, st)
        reason, s, tr = explore(w, denom)
        cov = float(((w.grid.data == 0) & denom).sum()) / float(denom.sum())
        t_at = lambda lv: next((t for t, c in tr if c >= lv), float('nan'))  # noqa: E731
        why = ' '.join('%s=%d' % (k, v) for k, v in s.items() if k not in ('goals', 'failures', 'rechecks'))
        print('%-6s %-22s %6.1f%%  %6.1f  %4.0f  %5.0f  %5.0f  %4d %4d %6d  %s'
              % (name, reason, cov * 100, w.dist, w.time(), t_at(.90), t_at(.95), s['goals'], s['failures'],
                 s['rechecks'], why), flush=True)


if __name__ == '__main__':
    main()
