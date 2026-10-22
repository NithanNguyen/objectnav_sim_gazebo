"""Navigation (AMCL + Nav2) trong căn hộ ảo — bản mô phỏng của receptionist_robot/launch/nav_v3.launch.py.

ros2 launch objectnav_sim_gazebo nav_sim.launch.py                         # Gazebo + EKF + laser_filters + Nav2 + RViz, robot ở S1
ros2 launch objectnav_sim_gazebo nav_sim.launch.py x:=-5.5 y:=-0.3 yaw:=1.57   # xuất phát ở S2 (AMCL tự đổi sang frame map)
ros2 launch objectnav_sim_gazebo nav_sim.launch.py gui:=false rviz:=false
ros2 launch objectnav_sim_gazebo nav_sim.launch.py record_bag:=true        # ghi bag như robot thật
ros2 launch objectnav_sim_gazebo nav_sim.launch.py bt_xml:=<file.xml>      # đổi behavior tree

Gửi đích: nút "Navigation2 Goal" trong RViz, hoặc
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"

Ánh xạ nav_v3.launch.py (robot thật) -> mô phỏng (cùng tên topic/frame):
  robot_state_publisher, joint_state_publisher -> home_sim.launch.py (URDF v1) + gazebo_ros_joint_state_publisher
  wheel_odom_node (STM32)    -> gazebo_ros_diff_drive /odom (publish_odom_tf:=false, EKF phát TF như robot thật)
  bno055                     -> gazebo_ros_imu_sensor /imu/data
  sllidar_node (RPLIDAR S2E) -> gazebo_ros_ray_sensor /scan (frame laser_frame)
  imu_reader                 -> bỏ (chỉ đổi quaternion -> Euler để xem, Nav2 không dùng)
  jetson_sensor_bridge, ultrasonic_fusion_node, global_localizer -> bỏ (nav_v3 cũng đang tắt)
  ekf / laser_filters        -> GIỮ NGUYÊN: dùng chung config/slam/ekf.yaml, config/slam/laser_filter.yaml (chép nguyên văn)
  nav2_bringup               -> GIỮ NGUYÊN: config/nav/nav2_params.yaml (chép nav2_params_test.yaml, 4 chỗ [SIM])
  set_initial_pose (/initialpose) -> giữ, nhưng tư thế tính từ vị trí spawn thay vì tọa độ tầng e6
  navigator.py (checkpoint)  -> chưa đưa vào: cần bộ checkpoint riêng cho căn hộ ảo; đích gửi qua RViz/action.
"""
import datetime
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Các topic của nav_v3.launch.py có trong mô phỏng (bỏ /ultrasonic*, /robot/* vì không có node phát) + /clock
BAG_TOPICS = ['/clock', '/scan', '/scan_filtered', '/odom', '/odometry/filtered', '/imu/data', '/amcl_pose',
              '/cmd_vel', '/tf', '/tf_static', '/plan', '/navigate_to_pose/_action/feedback']

# Hiệp phương sai /initialpose giống nav_v3.launch.py: var(x)=var(y)=0.25 m², var(yaw)=0.068 rad²
INITIAL_POSE_COV = [0.0] * 36
INITIAL_POSE_COV[0] = INITIAL_POSE_COV[7] = 0.25
INITIAL_POSE_COV[35] = 0.068


def world_to_map(x, y, yaw, map_x, map_y, map_yaw):
    """Đổi tư thế (x, y, yaw) từ frame world của Gazebo sang frame map.

    (map_x, map_y, map_yaw) là tư thế của gốc frame map trong world. Với slam_toolbox, frame map trùng frame odom
    lúc bắt đầu dựng bản đồ, tức là vị trí spawn của robot khi chạy slam_sim.launch.py.
    """
    dx, dy = x - map_x, y - map_y
    c, s = math.cos(map_yaw), math.sin(map_yaw)
    yaw_m = math.atan2(math.sin(yaw - map_yaw), math.cos(yaw - map_yaw))
    return c * dx + s * dy, -s * dx + c * dy, yaw_m


def _initial_pose(context):
    f = {k: float(LaunchConfiguration(k).perform(context)) for k in ('x', 'y', 'yaw', 'map_x', 'map_y', 'map_yaw')}
    return world_to_map(f['x'], f['y'], f['yaw'], f['map_x'], f['map_y'], f['map_yaw'])


def _write_params(params_file, xm, ym, yawm):
    """Chép params_file, chỉ thay initial_pose_* của AMCL; trả về đường dẫn file tạm."""
    with open(params_file, 'r') as f:
        data = yaml.safe_load(f)
    amcl = data['amcl']['ros__parameters']
    amcl.update({'set_initial_pose': True, 'initial_pose_x': xm, 'initial_pose_y': ym,
                 'initial_pose_z': 0.0, 'initial_pose_yaw': yawm})
    out = tempfile.NamedTemporaryFile(mode='w', prefix='nav_sim_params_', suffix='.yaml', delete=False)
    yaml.safe_dump(data, out, sort_keys=False)
    out.close()
    return out.name


def _nav2_and_initial_pose(context, pkg):
    xm, ym, yawm = _initial_pose(context)
    params = _write_params(LaunchConfiguration('params_file').perform(context), xm, ym, yawm)
    pose_msg = yaml.safe_dump({
        'header': {'frame_id': 'map'},
        'pose': {'pose': {'position': {'x': xm, 'y': ym, 'z': 0.0},
                          'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yawm / 2), 'w': math.cos(yawm / 2)}},
                 'covariance': INITIAL_POSE_COV}}, default_flow_style=True, width=10000).strip()
    info = '[nav_sim] AMCL initial pose (map) x=%.3f y=%.3f yaw=%.3f; params -> %s' % (xm, ym, yawm, params)

    nav2 = GroupAction(scoped=True, actions=[  # scoped: xem chú thích GroupAction trong slam_sim.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
                'params_file': params,
                # nav2_bringup (Foxy) ghi đè bt_navigator.default_bt_xml_filename bằng đối số này (RewrittenYaml)
                'default_bt_xml_filename': LaunchConfiguration('bt_xml'),
                'autostart': 'true',
            }.items()),
    ])

    # nav_v3 phát /initialpose một lần sau khi Nav2 lên (ngoài set_initial_pose của AMCL); giữ hành vi đó.
    set_initial_pose = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped', pose_msg],
        output='screen', condition=IfCondition(LaunchConfiguration('pub_initial_pose')))

    return [
        LogInfo(msg=info),
        TimerAction(period=8.0, actions=[LogInfo(msg='[nav_sim] [8s] Nav2 bringup (AMCL + planner + DWB + BT)'), nav2]),
        TimerAction(period=14.0, actions=[LogInfo(msg='[nav_sim] [14s] /initialpose'), set_initial_pose]),
    ]


def _bag_record(context):
    if LaunchConfiguration('record_bag').perform(context).lower() != 'true':
        return []
    bags_dir = os.path.expanduser(LaunchConfiguration('bag_dir').perform(context))
    os.makedirs(bags_dir, exist_ok=True)
    bag_path = os.path.join(bags_dir, 'sim_nav_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    return [LogInfo(msg='[nav_sim] [bag] Recording -> ' + bag_path),
            ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_path] + BAG_TOPICS, output='screen')]


def generate_launch_description():
    pkg = get_package_share_directory('objectnav_sim_gazebo')
    slam_cfg = os.path.join(pkg, 'config', 'slam')
    sim_time = {'use_sim_time': True}

    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[os.path.join(slam_cfg, 'ekf.yaml'), sim_time])

    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain', output='screen',
        parameters=[os.path.join(slam_cfg, 'laser_filter.yaml'), sim_time])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='mở gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 'nav_sim.rviz')),
        DeclareLaunchArgument('x', default_value='3.0', description='world x lúc spawn (m), mặc định S1'),
        DeclareLaunchArgument('y', default_value='-3.0', description='world y lúc spawn (m), mặc định S1'),
        DeclareLaunchArgument('yaw', default_value='3.14', description='world yaw lúc spawn (rad), mặc định S1'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'home_sim_map.yaml')),
        DeclareLaunchArgument('map_x', default_value='3.0',
                              description='tư thế gốc frame map trong world = vị trí spawn khi dựng bản đồ (S1)'),
        DeclareLaunchArgument('map_y', default_value='-3.0'),
        DeclareLaunchArgument('map_yaw', default_value='3.14'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav', 'nav2_params.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=os.path.join(
            pkg, 'behavior_trees', 'navigate_w_replanning_and_recovery_x221.xml')),
        DeclareLaunchArgument('pub_initial_pose', default_value='true'),
        DeclareLaunchArgument('lidar_samples', default_value='3200'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_dir', default_value='~/objectnav_sim_bags'),

        LogInfo(msg='[nav_sim] === Gazebo + EKF + laser_filters + Nav2 (AMCL) ==='),

        # Gazebo + robot; EKF là nguồn TF odom->base_footprint duy nhất (như robot thật) nên tắt TF của diff-drive.
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'home_sim.launch.py')),
                launch_arguments={
                    'gui': LaunchConfiguration('gui'),
                    'rviz': 'false',
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'yaw': LaunchConfiguration('yaw'),
                    'lidar_samples': LaunchConfiguration('lidar_samples'),
                    'publish_odom_tf': 'false',
                }.items()),
        ]),

        OpaqueFunction(function=_bag_record),

        # Robot thật chờ 7 s / 11 s / 12 s / 15 s cho phần cứng; mô phỏng chỉ chờ Gazebo spawn robot (~2–3 s),
        # cùng mốc với slam_sim.launch.py.
        TimerAction(period=5.0, actions=[LogInfo(msg='[nav_sim] [5s] ekf_filter_node'), ekf]),
        TimerAction(period=6.0, actions=[LogInfo(msg='[nav_sim] [6s] scan_to_scan_filter_chain'), scan_filter]),
        OpaqueFunction(function=_nav2_and_initial_pose, args=[pkg]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
