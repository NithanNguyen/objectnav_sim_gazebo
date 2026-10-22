#!/usr/bin/env python3
"""Khám phá frontier tự động (dựng bản đồ 2D căn hộ chưa biết) — ROS 2 Foxy, slam_toolbox + Nav2.

Thuật toán (lõi ở scripts/frontier_core.py):
  1. Phát hiện frontier trên /map của slam_toolbox (ô trống kề ô chưa biết), gom cụm, bỏ cụm < min_frontier_size.
  2. Chọn cụm có điểm tiếp cận gần nhất THEO ĐƯỜNG ĐI (Dijkstra trên vùng đã biết là trống, có clearance).
     Điểm tiếp cận của một cụm = ô đủ clearance, cách cụm <= approach_radius, gần tâm cụm nhất.
  3. Repetitive re-checking: mỗi chu kỳ, nếu cụm đang nhắm không còn là frontier (LiDAR đã quét tới) thì bỏ
     mục tiêu và chọn lại ngay (gửi goal mới; Nav2 Foxy preempt goal cũ). Không đổi mục tiêu chỉ vì có ứng
     viên điểm tốt hơn (tránh dao động).
  4. Tới nơi (đợi một lần cập nhật /map) mà cụm vẫn mở VÀ quanh cụm không có thêm ô đã biết, hoặc Nav2
     thất bại/quá giờ -> tính một lần thất bại; đủ max_failures -> blacklist theo bán kính quanh tâm cụm.
  5. Kết thúc khi hết frontier, hoặc còn frontier nhưng không cụm nào tới được trong done_patience chu kỳ;
     log tổng kết và phát /explore/status.

Giao diện giống node của robot thật (receptionist_robot/scripts/frontier_explorer.py):
  service /explore/start, /explore/stop (std_srvs/Trigger); action navigate_to_pose.
  Thêm: /explore/frontiers (MarkerArray), /explore/status (std_msgs/String, latched).
"""
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import frontier_core as fc  # noqa: E402

IDLE, SELECT, MOVING, VERIFY, DONE = 'IDLE', 'SELECT', 'MOVING', 'VERIFY', 'DONE'


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        p = {
            'map_topic': '/map',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'autostart': False,            # True: tự bắt đầu khi có /map, TF và Nav2 (không cần gọi /explore/start)
            'tick_period': 1.0,            # [s] chu kỳ chọn mục tiêu / re-check
            'min_frontier_size': 8,        # [ô] cụm nhỏ hơn bị bỏ (8 ô = 0.4 m ở 5 cm/ô)
            'free_thresh': 20,             # ô 0..free_thresh là trống
            'occ_thresh': 65,              # ô >= occ_thresh là chiếm chỗ
            'robot_radius': 0.30,          # [m] clearance tối thiểu của điểm/đường tiếp cận
            'approach_radius': 0.6,        # [m] điểm tiếp cận cách cụm frontier tối đa
            'min_goal_distance': 0.4,      # [m] bỏ điểm tiếp cận quá gần robot
            'recheck_radius': 0.5,         # [m] cụm còn sống nếu còn ô frontier trong bán kính này
            'blacklist_radius': 0.6,       # [m]
            'max_failures': 2,             # số lần thất bại trước khi blacklist
            'goal_timeout': 120.0,         # [s] (đồng hồ ROS) quá thời gian thì hủy goal, tính 1 thất bại
            'verify_timeout': 5.0,         # [s] chờ /map mới sau khi tới nơi trước khi đánh giá cụm
            'min_gain_cells': 10,          # tới nơi mà quanh cụm thêm < số ô đã biết này (và cụm vẫn mở) -> thất bại
            'done_patience': 3,            # [chu kỳ] liên tiếp không có cụm tới được thì kết thúc
            'exploration_timeout': 0.0,    # [s] 0 = không giới hạn
            'action_name': 'navigate_to_pose',
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        self.prm = {k: self.get_parameter(k).value for k in p}

        self.state = IDLE
        self.map_msg = None
        self.map_rx_time = None
        self.target = None
        self.goal_seq = 0
        self._goal_handle = None
        self.goal_start = None
        self.arrival_time = None
        self.known_at_send = 0
        self.no_target_ticks = 0
        self.blacklist = fc.Blacklist(self.prm['blacklist_radius'], self.prm['max_failures'])
        self.stats = {}
        self.last_pose = None
        self.start_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.prm['map_topic'], self._on_map, latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 10)
        self.status_pub = self.create_publisher(String, '/explore/status', latched)
        self.create_service(Trigger, '/explore/start', self._on_start)
        self.create_service(Trigger, '/explore/stop', self._on_stop)
        self.nav = ActionClient(self, NavigateToPose, self.prm['action_name'])
        self.create_timer(float(self.prm['tick_period']), self._tick)
        self.create_timer(0.2, self._track_distance)

        if self.prm['autostart']:
            self._reset()
            self.state = SELECT
        self._status('ready' if self.state == IDLE else 'starting')
        self.get_logger().info('frontier_explorer ready (state=%s). Start: ros2 service call /explore/start '
                               'std_srvs/srv/Trigger' % self.state)

    # ------------------------------------------------------------------ helpers
    def _now(self):
        return self.get_clock().now()

    def _secs_since(self, t):
        return (self._now() - t).nanoseconds * 1e-9

    def _status(self, text):
        self.status_pub.publish(String(data='%s: %s' % (self.state, text)))

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.prm['map_frame'], self.prm['base_frame'], Time())
        except TransformException as ex:
            self.get_logger().warn('No TF %s -> %s: %s' % (self.prm['map_frame'], self.prm['base_frame'], ex),
                                   throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _reset(self):
        self.blacklist.clear()
        self.target = None
        self.goal_seq += 1          # bỏ qua kết quả của mọi goal cũ
        self.no_target_ticks = 0
        self.stats = {'goals': 0, 'failures': 0, 'rechecks': 0, 'path_m': 0.0}
        self.last_pose = None
        self.start_time = self._now()

    def _fail(self, why):
        self.stats['failures'] += 1
        if self.target is not None:
            if self.blacklist.record_failure(self.target.centroid_xy):
                self.get_logger().warn('Blacklist frontier (%.2f, %.2f): %s' % (self.target.centroid_xy + (why,)))
            else:
                self.get_logger().warn('Frontier (%.2f, %.2f) failed: %s' % (self.target.centroid_xy + (why,)))
        self.target = None
        self.state = SELECT

    def _finish(self, reason, grid=None):
        self.state = DONE
        self.target = None
        elapsed = self._secs_since(self.start_time) if self.start_time is not None else float('nan')
        area = fc.known_free_area(grid, self.prm['free_thresh']) if grid is not None else float('nan')
        summary = ('%s | t=%.0f s, path=%.1f m, goals=%d, failures=%d, re-checks=%d, blacklisted=%d, '
                   'known free=%.1f m2' % (reason, elapsed, self.stats['path_m'], self.stats['goals'],
                                           self.stats['failures'], self.stats['rechecks'],
                                           len(self.blacklist.points), area))
        self.get_logger().info('>>> Exploration DONE: ' + summary)
        self._status(summary)

    # ------------------------------------------------------------------ callbacks
    def _on_map(self, msg):
        self.map_msg = msg
        self.map_rx_time = self._now()

    def _on_start(self, request, response):
        if self.state in (SELECT, MOVING, VERIFY):
            response.success, response.message = False, 'already running (%s)' % self.state
            return response
        self._reset()
        self.state = SELECT
        self._status('started')
        self.get_logger().info('>>> Exploration STARTED')
        response.success, response.message = True, 'exploration started'
        return response

    def _on_stop(self, request, response):
        self._cancel_current()
        self.state = IDLE
        self.target = None
        self._status('stopped by user')
        self.get_logger().info('>>> Exploration STOPPED')
        response.success, response.message = True, 'exploration stopped'
        return response

    def _track_distance(self):
        if self.state not in (SELECT, MOVING, VERIFY):
            return
        pose = self._robot_pose()
        if pose is None:
            return
        if self.last_pose is not None:
            self.stats['path_m'] += math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
        self.last_pose = pose

    # ------------------------------------------------------------------ main loop
    def _tick(self):
        if self.state in (IDLE, DONE):
            return
        if self.map_msg is None:
            self.get_logger().info('Waiting for %s ...' % self.prm['map_topic'], throttle_duration_sec=5.0)
            return
        pose = self._robot_pose()
        if pose is None:
            return
        timeout = float(self.prm['exploration_timeout'])
        if timeout > 0.0 and self._secs_since(self.start_time) > timeout:
            self._cancel_current()
            self._finish('exploration_timeout')
            return

        grid = fc.Grid.from_msg(self.map_msg)
        frontiers = fc.detect_frontiers(grid, self.prm['free_thresh'], self.prm['min_frontier_size'])

        if self.state == MOVING:
            if self._secs_since(self.goal_start) > float(self.prm['goal_timeout']):
                self._cancel_current()
                self._fail('goal timeout')
            elif not fc.frontier_alive(grid, frontiers, self.target.cells_xy, self.prm['recheck_radius']):
                self.stats['rechecks'] += 1
                self.get_logger().info('Re-check: frontier (%.2f, %.2f) already mapped -> reselect'
                                       % self.target.centroid_xy)
                self.target = None
                self.state = SELECT   # goal mới sẽ preempt goal đang chạy
            else:
                self._publish_markers(grid, frontiers)
                return

        if self.state == VERIFY:
            fresh = self.map_rx_time is not None and self.map_rx_time > self.arrival_time + Duration(seconds=0.5)
            if not fresh and self._secs_since(self.arrival_time) < float(self.prm['verify_timeout']):
                return
            gain = fc.known_cells_near(grid, self.target.cells_xy) - self.known_at_send
            if (fc.frontier_alive(grid, frontiers, self.target.cells_xy, self.prm['recheck_radius'])
                    and gain < int(self.prm['min_gain_cells'])):
                self._fail('reached, frontier still open and no new cells around it (+%d)' % gain)
            else:
                self.target = None
                self.state = SELECT

        if self.state == SELECT:
            self._select_and_send(grid, frontiers, pose)
        self._publish_markers(grid, frontiers)

    def _select_and_send(self, grid, frontiers, pose):
        if not frontiers.clusters:
            self._cancel_current()
            self._finish('complete: no frontier left', grid)
            return
        if not self.nav.server_is_ready():
            self.get_logger().info('Waiting for action server %s ...' % self.prm['action_name'],
                                   throttle_duration_sec=5.0)
            return
        target, n_reach, n_black = fc.select_target(
            grid, frontiers, pose[:2], self.blacklist, self.prm['robot_radius'], self.prm['approach_radius'],
            self.prm['min_goal_distance'], self.prm['free_thresh'], self.prm['occ_thresh'])
        if target is None:
            self._cancel_current()  # không để Nav2 tiếp tục chạy tới mục tiêu cũ không còn được theo dõi
            self.no_target_ticks += 1
            self.get_logger().info('No reachable frontier (%d clusters, %d blacklisted) [%d/%d]'
                                   % (len(frontiers.clusters), n_black, self.no_target_ticks,
                                      self.prm['done_patience']))
            if self.no_target_ticks >= int(self.prm['done_patience']):
                self._cancel_current()
                left = ', '.join('(%.1f, %.1f)' % c['centroid'] for c in frontiers.clusters)
                self._finish('no reachable frontier; remaining: %s' % left, grid)
            return
        self.no_target_ticks = 0
        self._send_goal(target, grid)

    # ------------------------------------------------------------------ Nav2 action
    def _send_goal(self, target, grid):
        self.known_at_send = fc.known_cells_near(grid, target.cells_xy)
        self.goal_seq += 1
        seq = self.goal_seq
        self.target = target
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.prm['map_frame']
        goal.pose.header.stamp = self._now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(target.goal_xy[0]), float(target.goal_xy[1])
        goal.pose.pose.orientation.z = math.sin(target.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target.yaw / 2.0)
        self.stats['goals'] += 1
        self.goal_start = self._now()
        self.state = MOVING
        self.get_logger().info('Goal #%d -> (%.2f, %.2f), frontier (%.2f, %.2f), %d cells, path %.2f m'
                               % ((self.stats['goals'],) + target.goal_xy + target.centroid_xy + (target.size, target.cost)))
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, seq))

    def _on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:
            return
        if not handle.accepted:
            self._fail('goal rejected by Nav2')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, seq))

    def _on_result(self, future, seq):
        if seq != self.goal_seq or self.state != MOVING:
            return   # goal đã bị preempt/hủy hoặc đã chọn lại
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.arrival_time = self._now()
            self.state = VERIFY
        else:
            self._fail('Nav2 status %d' % status)

    def _cancel_current(self):
        self.goal_seq += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ------------------------------------------------------------------ visualization
    def _publish_markers(self, grid, frontiers):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self._now().to_msg()
        for i, cl in enumerate(frontiers.clusters):
            m = Marker()
            m.header.frame_id = self.prm['map_frame']
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = 'frontiers', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cl['centroid'][0], cl['centroid'][1], 0.1
            m.pose.orientation.w = 1.0
            s = min(0.5, max(0.15, 0.01 * cl['size']))
            m.scale.x = m.scale.y = m.scale.z = s
            if self.blacklist.contains(cl['centroid']):
                m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.55, 0.0
            m.color.a = 0.9
            arr.markers.append(m)
        if self.target is not None:
            g = Marker()
            g.header.frame_id = self.prm['map_frame']
            g.header.stamp = stamp
            g.ns, g.id, g.type, g.action = 'target', 0, Marker.ARROW, Marker.ADD
            g.pose.position.x, g.pose.position.y = self.target.goal_xy
            g.pose.orientation.z = math.sin(self.target.yaw / 2.0)
            g.pose.orientation.w = math.cos(self.target.yaw / 2.0)
            g.scale.x, g.scale.y, g.scale.z = 0.5, 0.08, 0.08
            g.color.g, g.color.a = 1.0, 1.0
            arr.markers.append(g)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
