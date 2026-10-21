#!/usr/bin/env python3
"""Sinh worlds/home_trials.world từ world gốc của AWS Small House.

Thêm vào world (KHÔNG sửa file của gói AWS):
  1. plugin gazebo_ros_state (WORLD plugin) -> /gazebo/get_entity_state, /gazebo/set_entity_state,
     /gazebo/model_states, /gazebo/link_states.
  2. mỗi vật trong config/trials.yaml một bản, đặt tại vị trí ĐẦU TIÊN của nó trong 'placements'
     (trial runner sẽ dời vật sang vị trí khác cho từng lượt).
Vật nào chưa có thư mục model trong models/ (vd. gso_mug khi chưa chạy fetch_gso.py) sẽ bị bỏ qua và in cảnh báo.
Chỉ dùng Python 3.8 + PyYAML (có sẵn khi cài ROS 2 Foxy).

Dùng (từ thư mục gói, sau khi đã source ROS và workspace có gói AWS):
  python3 tools/build_world.py
  python3 tools/build_world.py --aws-world /duong/dan/small_house.world
"""
import argparse
import os
import re
import sys

import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

PLUGIN = '''
    <!-- [home_sim_world] Ground truth: WORLD plugin, must sit directly inside <world> -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <!-- publish rate of /gazebo/model_states and /gazebo/link_states (Hz); services do not depend on it -->
      <update_rate>10.0</update_rate>
    </plugin>
'''

OBJ = '''    <include>
      <uri>model://{model}</uri>
      <name>{model}</name>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
    </include>
'''


def find_aws_world():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('aws_robomaker_small_house_world'),
                            'worlds', 'small_house.world')
    except Exception as e:  # noqa: BLE001
        sys.exit('Không tìm thấy gói aws_robomaker_small_house_world (%s).\n'
                 'Hãy source workspace (source ~/sim_ws/install/setup.bash) hoặc dùng --aws-world.' % e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--aws-world')
    ap.add_argument('--trials', default=os.path.join(PKG, 'config', 'trials.yaml'))
    ap.add_argument('--models', default=os.path.join(PKG, 'models'))
    ap.add_argument('--out', default=os.path.join(PKG, 'worlds', 'home_trials.world'))
    a = ap.parse_args()

    src = a.aws_world or find_aws_world()
    txt = open(src).read()
    if 'libgazebo_ros_state.so' in txt:
        sys.exit('World gốc đã có gazebo_ros_state -> không chèn thêm để tránh nạp 2 lần.')
    m = re.search(r"<world\s+name=['\"][^'\"]+['\"]\s*>", txt)
    if not m:
        sys.exit('Không thấy thẻ <world name=...> trong %s' % src)

    cfg = yaml.safe_load(open(a.trials))
    first = {}
    for p in cfg['placements']:
        first.setdefault(p['object'], p)
    objs, skipped = [], []
    for name, p in first.items():
        if not os.path.isfile(os.path.join(a.models, name, 'model.sdf')):
            skipped.append(name)
            continue
        objs.append(OBJ.format(model=name, x=p['x'], y=p['y'], z=p['z'], yaw=p.get('yaw', 0.0)))

    block = PLUGIN + '\n    <!-- [home_sim_world] Target objects (initial pose = first placement in config/trials.yaml) -->\n' + ''.join(objs)
    out = txt[:m.end()] + '\n' + block + txt[m.end():]
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    open(a.out, 'w').write(out)
    print('World gốc : %s' % src)
    print('Đã ghi    : %s' % a.out)
    print('Vật đã đặt: %s' % ', '.join(sorted(set(first) - set(skipped))))
    if skipped:
        print('BỎ QUA (chưa có model): %s -> chạy tools/fetch_gso.py rồi chạy lại script này.' % ', '.join(skipped))


if __name__ == '__main__':
    main()
