#!/usr/bin/env python3
"""Kiểm tra hình học (không cần chạy Gazebo) các vị trí đặt vật và vị trí xuất phát của robot.

Bằng cách bắn tia thẳng đứng xuống mesh visual của toàn bộ căn nhà AWS (đã áp pose trong world):
  * vật: tại lưới điểm (<= 5 cm) phủ đế vật, mặt cao nhất bên dưới phải trùng z khai báo (sai lệch <= tol)
         -> vật được đỡ hoàn toàn, không lơ lửng, không bị nhúng vào đồ khác;
         đồng thời không có đồ vật nào chắn ngay phía trên (trong 1.0 m), trừ khi vật được khai báo là bị che.
  * xuất phát: trong bán kính r quanh điểm xuất phát, mọi tia đều phải chạm sàn (z <= 0.03 m).
Cần: pip3 install trimesh pycollada numpy pyyaml  (chỉ để kiểm tra, không cần cho mô phỏng)
Dùng: python3 check_placements.py <aws_repo_hoac_share> <trials.yaml>
"""
import glob
import math
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

SKIP = ('RoomCeiling', 'Chandelier', 'LightC', 'AirconditionerA', 'SecurityCamera', 'Curtain')


def unit(p):
    m = re.search(r'<unit[^>]*meter="([\d.eE+-]+)"', open(p, errors='ignore').read(5000))
    return float(m.group(1)) if m else 1.0


def tf(pose):
    r, p, y = pose[3:]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    M = np.eye(4)
    M[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    M[:3, 3] = pose[:3]
    return M


def house(root):
    world = ET.parse(root + '/worlds/small_house.world').getroot().find('world')
    parts, cache = [], {}
    for m in world.findall('model'):
        uri = m.find('include/uri')
        if uri is None:
            continue
        base = uri.text.strip().replace('model://aws_robomaker_residential_', '')
        if base.startswith(SKIP):
            continue
        f = glob.glob('%s/models/aws_robomaker_residential_%s/meshes/*visual*.DAE' % (root, base))
        if not f:
            continue
        if f[0] not in cache:
            g = trimesh.util.concatenate(trimesh.load(f[0], force='scene').dump())
            g.apply_scale(unit(f[0]))
            cache[f[0]] = g
        g = cache[f[0]].copy()
        g.apply_transform(tf([float(v) for v in m.find('pose').text.split()]))
        parts.append(g)
    return trimesh.util.concatenate(parts)


def hits_down(scene, xy, z0=1.6):
    o = np.array([[x, y, z0] for x, y in xy])
    d = np.tile([0, 0, -1.0], (len(o), 1))
    loc, idx, _ = scene.ray.intersects_location(o, d, multiple_hits=True)
    out = [[] for _ in xy]
    for p, i in zip(loc, idx):
        out[i].append(p[2])
    return [sorted(h, reverse=True) for h in out]


def main(root, trials_path, tol=0.03):
    cfg = yaml.safe_load(open(trials_path))
    dims = cfg.get('object_footprint_m', {})
    scene = house(root)
    bad = 0
    print('== Vị trí xuất phát (bán kính %.2f m phải trống tới sàn)' % cfg.get('robot_radius_m', 0.3))
    r = cfg.get('robot_radius_m', 0.3)
    for s in cfg['start_poses']:
        x, y = s['x'], s['y']
        ring = [(x + rr * math.cos(a), y + rr * math.sin(a)) for rr in (0, r / 2, r)
                for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
        top = max((h[0] if h else -9) for h in hits_down(scene, ring, 1.3))
        ok = top <= 0.03
        bad += not ok
        print('  %-8s (%.2f, %.2f)  vật cao nhất trong vòng tròn: %.2f m  %s' % (s['id'], x, y, top, 'OK' if ok else 'VƯỚNG'))
    print('== Vị trí đặt vật')
    for t in cfg['placements']:
        x, y, z, yaw = t['x'], t['y'], t['z'], t.get('yaw', 0.0)
        lx, ly = dims.get(t['object'], [0.1, 0.1])
        c, sn = math.cos(yaw), math.sin(yaw)
        nx, ny = max(2, int(math.ceil(lx / 0.05)) + 1), max(2, int(math.ceil(ly / 0.05)) + 1)
        pts = [(x + c * dx - sn * dy, y + sn * dx + c * dy)        # lưới điểm cách nhau <= 5 cm trên đế vật
               for dx in np.linspace(-lx / 2, lx / 2, nx) for dy in np.linspace(-ly / 2, ly / 2, ny)]
        hs = hits_down(scene, pts, 1.6)
        below = [max([h for h in hh if h <= z + tol] or [-9]) for hh in hs]
        above = [min([h for h in hh if h > z + tol] or [9]) for hh in hs]
        support = all(abs(b - z) <= tol for b in below)
        covered = min(above) < z + 1.0           # có mặt/vật nào nằm trên vật trong 1 m
        expect_cover = t.get('visibility') == 'occluded'
        ok = support and (expect_cover or not covered)   # vật 'visible' không được bị che từ trên
        bad += not ok
        print('  %-4s %-17s %-25s z=%.2f  mặt đỡ %.2f..%.2f (%d điểm): %s  che phía trên: %s  %s' % (
            t['id'], t['object'], t['surface'], z, min(below), max(below), len(below),
            'đủ' if support else 'THIẾU', 'có' if covered else 'không', 'OK' if ok else 'XEM LẠI'))
    print('Kết quả: %s' % ('tất cả hợp lệ' if bad == 0 else '%d mục cần xem lại' % bad))
    return bad


if __name__ == '__main__':
    sys.exit(1 if main(sys.argv[1], sys.argv[2]) else 0)
