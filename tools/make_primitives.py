#!/usr/bin/env python3
"""Sinh model SDF hình khối đơn giản cho các vật chưa có model 3D.

Quy ước chung cho mọi vật (kể cả vật GSO do fetch_gso.py sinh ra):
  * gốc model = TÂM ĐÁY vật  -> đặt pose z = độ cao bề mặt là vật nằm đúng trên mặt.
  * <static>true</static>     -> không mô phỏng vật lý, vật không rơi/trượt.
  * collision = hình khối giống visual (box/cylinder), không dùng mesh.
Kích thước là ƯỚC LƯỢNG của đồ vật thường gặp (không lấy từ nguồn đo chuẩn); chỉnh trong bảng OBJECTS.

Dùng: python3 make_primitives.py <thu_muc_models>
"""
import os
import sys

# name: (mô tả ASCII ghi vào model.config, danh sách phần [(kiểu, kích thước, pose x y z r p y, màu rgb)])
# box: (dx, dy, dz) ; cylinder: (bán kính, chiều dài)  -- pose là tâm của phần đó, tính từ tâm đáy vật
OBJECTS = {
    'obj_phone': ('Phone (box)', [
        ('box', (0.150, 0.072, 0.008), (0, 0, 0.004, 0, 0, 0), (0.05, 0.05, 0.05)),
    ]),
    'obj_tv_remote': ('TV remote (box)', [
        ('box', (0.180, 0.048, 0.022), (0, 0, 0.011, 0, 0, 0), (0.15, 0.15, 0.18)),
    ]),
    'obj_glasses': ('Glasses lying flat, temples open', [
        ('cylinder', (0.025, 0.004), (-0.033, 0, 0.002, 0, 0, 0), (0.6, 0.75, 0.85)),
        ('cylinder', (0.025, 0.004), (0.033, 0, 0.002, 0, 0, 0), (0.6, 0.75, 0.85)),
        ('box', (0.016, 0.006, 0.004), (0, 0, 0.002, 0, 0, 0), (0.1, 0.1, 0.1)),
        ('box', (0.004, 0.130, 0.006), (-0.058, 0.060, 0.003, 0, 0, 0), (0.1, 0.1, 0.1)),
        ('box', (0.004, 0.130, 0.006), (0.058, 0.060, 0.003, 0, 0, 0), (0.1, 0.1, 0.1)),
    ]),
    'obj_wallet': ('Wallet (box)', [
        ('box', (0.110, 0.090, 0.020), (0, 0, 0.010, 0, 0, 0), (0.40, 0.22, 0.10)),
    ]),
    'obj_cane': ('Walking cane lying on the floor', [
        ('cylinder', (0.0125, 0.85), (0, 0, 0.0125, 0, 1.5708, 0), (0.30, 0.18, 0.08)),
        ('box', (0.025, 0.12, 0.025), (0.425, 0.05, 0.0125, 0, 0, 0), (0.30, 0.18, 0.08)),
    ]),
    'obj_water_bottle': ('Water bottle (no drink bottle in GSO)', [
        ('cylinder', (0.034, 0.200), (0, 0, 0.100, 0, 0, 0), (0.55, 0.80, 0.95)),
        ('cylinder', (0.015, 0.030), (0, 0, 0.215, 0, 0, 0), (0.10, 0.30, 0.80)),
    ]),
    'obj_handbag': ('Handbag (no handbag in GSO)', [
        ('box', (0.300, 0.120, 0.220), (0, 0, 0.110, 0, 0, 0), (0.55, 0.15, 0.15)),
        ('box', (0.020, 0.012, 0.120), (-0.090, 0, 0.280, 0, 0, 0), (0.30, 0.08, 0.08)),
        ('box', (0.020, 0.012, 0.120), (0.090, 0, 0.280, 0, 0, 0), (0.30, 0.08, 0.08)),
        ('box', (0.200, 0.012, 0.020), (0, 0, 0.330, 0, 0, 0), (0.30, 0.08, 0.08)),
    ]),
}


def geom(kind, size):
    if kind == 'box':
        return '<box><size>%g %g %g</size></box>' % size
    return '<cylinder><radius>%g</radius><length>%g</length></cylinder>' % size


def model_sdf(name, parts):
    out = ['<?xml version="1.0"?>', '<sdf version="1.6">', '  <model name="%s">' % name,
           '    <static>true</static>', '    <link name="link">']
    for i, (kind, size, pose, rgb) in enumerate(parts):
        p = ' '.join('%g' % v for v in pose)
        c = '%g %g %g 1' % rgb
        out += ['      <collision name="c%d"><pose>%s</pose><geometry>%s</geometry></collision>' % (i, p, geom(kind, size)),
                '      <visual name="v%d"><pose>%s</pose><geometry>%s</geometry>' % (i, p, geom(kind, size)),
                '        <material><ambient>%s</ambient><diffuse>%s</diffuse><specular>0.1 0.1 0.1 1</specular></material>' % (c, c),
                '      </visual>']
    out += ['    </link>', '  </model>', '</sdf>', '']
    return '\n'.join(out)


def model_config(name, desc):
    return ('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
            '  <sdf version="1.6">model.sdf</sdf>\n  <author><name>home_sim_world</name></author>\n'
            '  <description>%s</description>\n</model>\n' % (name, desc))


if __name__ == '__main__':
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), '..', 'models')
    for name, (desc, parts) in OBJECTS.items():
        d = os.path.join(root, name)
        os.makedirs(d, exist_ok=True)
        open(os.path.join(d, 'model.sdf'), 'w').write(model_sdf(name, parts))
        open(os.path.join(d, 'model.config'), 'w').write(model_config(name, desc))
        print('->', d)
