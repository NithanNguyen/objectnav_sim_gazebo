"""SLAM trong căn hộ ảo — bản mô phỏng của receptionist_robot/launch/slam_v3_launch.py.

ros2 launch objectnav_sim_gazebo slam_sim.launch.py                    # Gazebo + EKF + laser_filters + slam_toolbox + RViz
ros2 launch objectnav_sim_gazebo slam_sim.launch.py gui:=false rviz:=false
ros2 launch objectnav_sim_gazebo slam_sim.launch.py record_bag:=true   # ghi bag như robot thật
ros2 launch objectnav_sim_gazebo slam_sim.launch.py use_ekf:=false     # bỏ EKF, TF odom lấy từ plugin diff-drive

Lái robot (terminal khác):  ros2 run teleop_twist_keyboard teleop_twist_keyboard
Lưu bản đồ:                 ros2 run nav2_map_server map_saver_cli -f ~/home_sim_map --ros-args -p use_sim_time:=true

Ánh xạ robot thật -> mô phỏng (cùng tên topic/frame):
  wheel_odom_node (STM32)   -> gazebo_ros_diff_drive  /odom (publish_odom_tf:=false, EKF phát TF như robot thật)
  bno055                    -> gazebo_ros_imu_sensor  /imu/data
  sllidar_node (RPLIDAR S2E)-> gazebo_ros_ray_sensor  /scan (frame laser_frame)
  joint_state_publisher     -> gazebo_ros_joint_state_publisher /joint_states
  robot_state_publisher     -> home_sim.launch.py (URDF v1)
  imu_reader                -> bỏ (chỉ đổi quaternion -> Euler để xem, không dùng cho SLAM)
  ekf / laser_filters / slam_toolbox -> GIỮ NGUYÊN, cấu hình chép nguyên văn ở config/slam/
"""
import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Cùng danh sách topic với slam_v3_launch.py
BAG_TOPICS = ['/scan', '/scan_filtered', '/odom', '/odometry/filtered', '/imu/data',
              '/cmd_vel', '/tf', '/tf_static', '/map']


def _bag_record(context):
    if LaunchConfiguration('record_bag').perform(context).lower() != 'true':
        return []
    bags_dir = os.path.expanduser(LaunchConfiguration('bag_dir').perform(context))
    os.makedirs(bags_dir, exist_ok=True)
    bag_path = os.path.join(bags_dir, 'sim_slam_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    return [
        LogInfo(msg='[slam_sim] [bag] Recording -> ' + bag_path),
        # use_sim_time để dấu thời gian trong bag theo đồng hồ Gazebo (/clock cũng được ghi)
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_path, '/clock'] + BAG_TOPICS,
                       output='screen'),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('objectnav_sim_gazebo')
    cfg = os.path.join(pkg, 'config', 'slam')
    sim_time = {'use_sim_time': True}

    use_ekf = LaunchConfiguration('use_ekf')

    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[os.path.join(cfg, 'ekf.yaml'), sim_time],
        condition=IfCondition(use_ekf))

    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain', output='screen',
        parameters=[os.path.join(cfg, 'laser_filter.yaml'), sim_time])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        parameters=[os.path.join(cfg, 'mapper_params_online_async.yaml'), sim_time])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='mở gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 'slam_sim.rviz')),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='true: EKF phát TF odom->base_footprint như robot thật'),
        DeclareLaunchArgument('x', default_value='3.0', description='mặc định S1'),
        DeclareLaunchArgument('y', default_value='-3.0'),
        DeclareLaunchArgument('yaw', default_value='3.14'),
        DeclareLaunchArgument('lidar_samples', default_value='3200'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_dir', default_value='~/objectnav_sim_bags'),

        LogInfo(msg='[slam_sim] === Gazebo + EKF + laser_filters + slam_toolbox ==='),

        # Gazebo + robot; khi có EKF thì tắt TF odom của diff-drive để chỉ còn 1 nguồn TF odom->base_footprint.
        # Phải bọc trong GroupAction (scoped=True): ở Foxy, IncludeLaunchDescription đặt launch_arguments thẳng vào
        # launch configuration của file cha (không có push/pop), nên 'rviz':'false' bên dưới sẽ ghi đè 'rviz' của
        # file này và RViz ở TimerAction cuối không bao giờ được mở.
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
                    'publish_odom_tf': PythonExpression(["'false' if '", use_ekf, "' == 'true' else 'true'"]),
                }.items()),
        ]),

        OpaqueFunction(function=_bag_record),

        # Robot thật dùng 7 s / 10 s / 12 s để chờ phần cứng; mô phỏng chỉ cần chờ Gazebo nạp world và spawn robot
        # (log thực tế trên laptop: robot spawn sau khoảng 2–3 s).
        TimerAction(period=5.0, actions=[LogInfo(msg='[slam_sim] [5s] ekf_filter_node'), ekf]),
        TimerAction(period=6.0, actions=[LogInfo(msg='[slam_sim] [6s] scan_to_scan_filter_chain'), scan_filter]),
        TimerAction(period=8.0, actions=[LogInfo(msg='[slam_sim] [8s] slam_toolbox (online_async)'), slam]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
