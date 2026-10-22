"""Khám phá tự động căn hộ ảo chưa có bản đồ: Gazebo + EKF + laser_filters + slam_toolbox + Nav2 + frontier_explorer.

ros2 launch objectnav_sim_gazebo explore_sim.launch.py                        # robot ở S1, tự bắt đầu khám phá
ros2 launch objectnav_sim_gazebo explore_sim.launch.py x:=-5.5 y:=-0.3 yaw:=1.57   # xuất phát ở S2
ros2 launch objectnav_sim_gazebo explore_sim.launch.py autostart:=false       # rồi: ros2 service call /explore/start std_srvs/srv/Trigger
ros2 launch objectnav_sim_gazebo explore_sim.launch.py gui:=false rviz:=false record_bag:=true

Kết quả: log ">>> Exploration DONE: ..." và topic /explore/status (latched). Lưu bản đồ:
  ros2 run nav2_map_server map_saver_cli -f ~/explored_map --ros-args -p use_sim_time:=true

Cấu trúc như auto_exploration.launch.py của robot thật (receptionist_robot): slam_toolbox (online async, cùng file
tham số với slam_sim.launch.py) phát /map và TF map->odom, Nav2 chạy navigation_launch.py (KHÔNG AMCL, KHÔNG
map_server). Không dùng nav2_bringup bringup_launch.py slam:=True vì ở Foxy nhánh đó chạy online_sync_launch.py
của slam_toolbox với tham số mặc định, bỏ qua config/slam/mapper_params_online_async.yaml.

Ghi đè chỉ trong lúc khám phá (file config gốc không đổi):
  * slam_toolbox map_update_interval (mặc định 1.0 s thay vì 4.0 s) để re-check thấy frontier đã đóng sớm hơn.
  * Nav2 FollowPath.max_vel_x = explore_max_vel_x (mặc định 0.30 m/s như nav2_exploration_params.yaml của robot thật).
  * costmap obstacle_layer: Nav2 Foxy (0.4.7) chỉ đọc obstacle_range / raytrace_range; config/nav/nav2_params.yaml
    đang dùng tên của bản Nav2 mới hơn (obstacle_max_range / raytrace_max_range) nên bị bỏ qua và Foxy dùng mặc định
    2.5 m / 3.0 m. Launch này chép giá trị *_max_range sang tên Foxy.
"""
import copy
import datetime
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

BAG_TOPICS = ['/clock', '/scan', '/scan_filtered', '/odom', '/odometry/filtered', '/imu/data', '/cmd_vel',
              '/tf', '/tf_static', '/map', '/plan', '/explore/frontiers', '/explore/status']


def _foxy_nav2_params(src, max_vel_x):
    """Chép nav2_params.yaml: max_vel_x khi khám phá + tên tham số range của obstacle_layer theo Nav2 Foxy."""
    with open(src, 'r') as f:
        data = yaml.safe_load(f)
    data = copy.deepcopy(data)
    data['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] = max_vel_x
    for costmap in ('local_costmap', 'global_costmap'):
        layer = data[costmap][costmap]['ros__parameters'].get('obstacle_layer', {})
        for source in str(layer.get('observation_sources', '')).split():
            obs = layer.get(source, {})
            if 'obstacle_max_range' in obs:
                obs['obstacle_range'] = obs['obstacle_max_range']
            if 'raytrace_max_range' in obs:
                obs['raytrace_range'] = obs['raytrace_max_range']
    out = tempfile.NamedTemporaryFile(mode='w', prefix='explore_nav2_params_', suffix='.yaml', delete=False)
    yaml.safe_dump(data, out, sort_keys=False)
    out.close()
    return out.name


def _stack(context, pkg):
    cfg = os.path.join(pkg, 'config', 'slam')
    sim_time = {'use_sim_time': True}
    interval = float(LaunchConfiguration('map_update_interval').perform(context))
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    params = _foxy_nav2_params(LaunchConfiguration('params_file').perform(context),
                               float(LaunchConfiguration('explore_max_vel_x').perform(context)))

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        parameters=[os.path.join(cfg, 'mapper_params_online_async.yaml'), sim_time,
                    {'map_update_interval': interval}])

    nav2 = GroupAction(scoped=True, actions=[  # scoped: xem chú thích GroupAction trong slam_sim.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': params,
                'default_bt_xml_filename': LaunchConfiguration('bt_xml'),
                'autostart': 'true',
                # global_costmap.static_layer đặt true trong nav2_params.yaml; navigation_launch.py (Foxy) ghi đè
                # khóa này bằng đối số của nó (mặc định false) nên phải truyền lại. slam_toolbox phát /map
                # transient_local + reliable nên tương thích.
                'map_subscribe_transient_local': 'true',
            }.items()),
    ])

    explorer = Node(
        package='objectnav_sim_gazebo', executable='frontier_explorer.py', name='frontier_explorer',
        output='screen', parameters=[sim_time, {'autostart': autostart}])

    return [
        LogInfo(msg='[explore_sim] map_update_interval=%.2f s, Nav2 params -> %s' % (interval, params)),
        TimerAction(period=8.0, actions=[LogInfo(msg='[explore_sim] [8s] slam_toolbox (online_async)'), slam]),
        TimerAction(period=10.0, actions=[LogInfo(msg='[explore_sim] [10s] Nav2 navigation (no AMCL)'), nav2]),
        TimerAction(period=14.0, actions=[LogInfo(msg='[explore_sim] [14s] frontier_explorer'), explorer]),
    ]


def _bag_record(context):
    if LaunchConfiguration('record_bag').perform(context).lower() != 'true':
        return []
    bags_dir = os.path.expanduser(LaunchConfiguration('bag_dir').perform(context))
    os.makedirs(bags_dir, exist_ok=True)
    bag_path = os.path.join(bags_dir, 'sim_explore_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    return [LogInfo(msg='[explore_sim] [bag] Recording -> ' + bag_path),
            ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_path] + BAG_TOPICS, output='screen')]


def generate_launch_description():
    pkg = get_package_share_directory('objectnav_sim_gazebo')
    cfg = os.path.join(pkg, 'config', 'slam')
    sim_time = {'use_sim_time': True}

    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[os.path.join(cfg, 'ekf.yaml'), sim_time])

    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain', output='screen',
        parameters=[os.path.join(cfg, 'laser_filter.yaml'), sim_time])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='mở gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 'explore_sim.rviz')),
        DeclareLaunchArgument('x', default_value='3.0', description='world x lúc spawn (m), mặc định S1'),
        DeclareLaunchArgument('y', default_value='-3.0', description='world y lúc spawn (m), mặc định S1'),
        DeclareLaunchArgument('yaw', default_value='3.14', description='world yaw lúc spawn (rad), mặc định S1'),
        DeclareLaunchArgument('lidar_samples', default_value='3200'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='true: tự khám phá khi /map, TF và Nav2 sẵn sàng; false: gọi /explore/start'),
        DeclareLaunchArgument('map_update_interval', default_value='1.0',
                              description='[s] ghi đè slam_toolbox map_update_interval khi khám phá (gốc 4.0)'),
        DeclareLaunchArgument('explore_max_vel_x', default_value='0.30',
                              description='[m/s] FollowPath.max_vel_x khi vừa SLAM vừa chạy'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav', 'nav2_params.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=os.path.join(
            pkg, 'behavior_trees', 'navigate_w_replanning_and_recovery_x221.xml')),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_dir', default_value='~/objectnav_sim_bags'),

        LogInfo(msg='[explore_sim] === Gazebo + EKF + laser_filters + slam_toolbox + Nav2 + frontier_explorer ==='),

        # EKF là nguồn TF odom->base_footprint duy nhất (như robot thật) nên tắt TF của diff-drive.
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

        # Cùng mốc thời gian với slam_sim.launch.py / nav_sim.launch.py (Gazebo spawn robot sau ~2–3 s).
        TimerAction(period=5.0, actions=[LogInfo(msg='[explore_sim] [5s] ekf_filter_node'), ekf]),
        TimerAction(period=6.0, actions=[LogInfo(msg='[explore_sim] [6s] scan_to_scan_filter_chain'), scan_filter]),
        OpaqueFunction(function=_stack, args=[pkg]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
