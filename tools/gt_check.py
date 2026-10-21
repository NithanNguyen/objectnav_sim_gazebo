#!/usr/bin/env python3
"""Kiểm tra ground truth khi Gazebo đang chạy home_trials.world.

1. /gazebo/get_entity_state cho mọi vật trong trials.yaml -> so với vị trí ban đầu (placement đầu tiên).
2. /gazebo/set_entity_state: dời 1 vật (mặc định obj_phone) sang placement thứ hai của nó, đọc lại,
   rồi trả về chỗ cũ. Mục đích: xác nhận dời được vật STATIC (cần cho trial runner).
   Hãy nhìn gzclient trong lúc chạy: vật phải nhảy sang chỗ mới rồi quay lại.

Dùng: ros2 run không áp dụng (đây là script) ->
  python3 $(ros2 pkg prefix home_sim_world)/share/home_sim_world/tools/gt_check.py
"""
import math
import os
import sys
import time

import rclpy
import yaml
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
TOL = 0.01  # m


def call(node, cli, req):
    if not cli.wait_for_service(timeout_sec=10.0):
        sys.exit('Không thấy service %s (plugin gazebo_ros_state chưa nạp?)' % cli.srv_name)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if fut.result() is None:
        sys.exit('Gọi %s bị quá thời gian' % cli.srv_name)
    return fut.result()


def get(node, cli, name):
    r = call(node, cli, GetEntityState.Request(name=name, reference_frame='world'))
    p = r.state.pose.position
    return r.success, (p.x, p.y, p.z)


def set_pose(node, cli, name, x, y, z, yaw):
    st = EntityState(name=name, reference_frame='world')
    st.pose.position.x, st.pose.position.y, st.pose.position.z = float(x), float(y), float(z)
    st.pose.orientation.z, st.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return call(node, cli, SetEntityState.Request(state=st)).success


def main():
    obj_to_move = sys.argv[1] if len(sys.argv) > 1 else 'obj_phone'
    cfg = yaml.safe_load(open(os.path.join(PKG, 'config', 'trials.yaml')))
    plist = {}
    for p in cfg['placements']:
        plist.setdefault(p['object'], []).append(p)
    rclpy.init()
    node = rclpy.create_node('gt_check')
    gcli = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    scli = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    bad = 0
    print('== get_entity_state')
    for name, ps in plist.items():
        ok, pos = get(node, gcli, name)
        if not ok:
            print('  %-17s KHÔNG CÓ trong world' % name)
            bad += 1
            continue
        e = math.dist(pos, (ps[0]['x'], ps[0]['y'], ps[0]['z']))
        bad += e > TOL
        print('  %-17s (%.3f, %.3f, %.3f)  sai lệch %.4f m  %s' % (name, *pos, e, 'OK' if e <= TOL else 'LỆCH'))
    print('== set_entity_state (%s)' % obj_to_move)
    ps = plist.get(obj_to_move, [])
    if len(ps) < 2:
        print('  %s cần ít nhất 2 placement trong trials.yaml' % obj_to_move)
    else:
        a, b = ps[0], ps[1]
        ok = set_pose(node, scli, obj_to_move, b['x'], b['y'], b['z'], b.get('yaw', 0.0))
        time.sleep(1.0)
        _, pos = get(node, gcli, obj_to_move)
        e = math.dist(pos, (b['x'], b['y'], b['z']))
        print('  dời sang %s: success=%s, sai lệch %.4f m  %s' % (b['id'], ok, e, 'OK' if ok and e <= TOL else 'LỖI'))
        bad += not (ok and e <= TOL)
        time.sleep(2.0)
        set_pose(node, scli, obj_to_move, a['x'], a['y'], a['z'], a.get('yaw', 0.0))
        print('  đã trả về %s' % a['id'])
    node.destroy_node()
    rclpy.shutdown()
    print('Kết quả: %s' % ('ĐẠT' if bad == 0 else '%d lỗi' % bad))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
