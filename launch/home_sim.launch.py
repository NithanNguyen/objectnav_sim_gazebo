"""Chạy Gazebo Classic 11 với căn hộ ảo home_trials.world và đặt robot (URDF v1) vào đó.

ros2 launch objectnav_sim_gazebo home_sim.launch.py                      # có giao diện, robot ở S1
ros2 launch objectnav_sim_gazebo home_sim.launch.py gui:=false           # chỉ gzserver (chạy hàng loạt lượt thử)
ros2 launch objectnav_sim_gazebo home_sim.launch.py x:=-5.5 y:=-0.3 yaw:=1.57   # xuất phát ở S2
ros2 launch objectnav_sim_gazebo home_sim.launch.py spawn_robot:=false   # chỉ căn hộ, không robot
ros2 launch objectnav_sim_gazebo home_sim.launch.py publish_odom_tf:=false     # khi chạy EKF riêng
ros2 launch objectnav_sim_gazebo home_sim.launch.py lidar_samples:=720         # LiDAR nhẹ hơn nếu RTF thấp
ros2 launch objectnav_sim_gazebo home_sim.launch.py rviz:=true                 # mở thêm RViz2 (rviz/home_sim.rviz)

Vị trí mặc định (x, y, yaw) = S1 trong config/trials.yaml.
Topic: /cmd_vel (vào), /odom, /scan, /imu/data, /joint_states, /tf (ra) — trùng tên với robot thật.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = 'mobile_robot'


def _robot_state_publisher(context, pkg):
    # Mesh dùng đường dẫn tuyệt đối file://<share>/meshes: Gazebo 11 (SystemPaths::FindFileURI) đọc thẳng file,
    # nên không phải thêm share/ vào GAZEBO_MODEL_PATH (tránh lỗi "Missing model.config" ở tab Insert).
    mappings = {
        'mesh_uri': 'file://' + os.path.join(pkg, 'meshes'),
        'publish_odom_tf': LaunchConfiguration('publish_odom_tf').perform(context),
        'lidar_samples': LaunchConfiguration('lidar_samples').perform(context),
    }
    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'mobile_robot.urdf.xacro'), mappings=mappings).toxml()
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])]


def generate_launch_description():
    pkg = get_package_share_directory('objectnav_sim_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # gzserver.launch.py ghi đè GAZEBO_RESOURCE_PATH bằng media path của các gói (gói AWS có export),
    # nên nếu biến môi trường thiếu thư mục cài của Gazebo thì gzserver/gzclient không tìm thấy media mặc định.
    # Thêm lại /usr/share/gazebo-11 ở đây (tương đương 'source /usr/share/gazebo/setup.sh').
    res = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    if '/usr/share/gazebo-11' not in res.split(os.pathsep):
        os.environ['GAZEBO_RESOURCE_PATH'] = os.pathsep.join(p for p in ['/usr/share/gazebo-11', res] if p)

    world = LaunchConfiguration('world')
    spawn_robot = IfCondition(LaunchConfiguration('spawn_robot'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'home_trials.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument('x', default_value='3.0', description='world x (m), mặc định S1'),
        DeclareLaunchArgument('y', default_value='-3.0', description='world y (m), mặc định S1'),
        DeclareLaunchArgument('z', default_value='0.01', description='độ cao base_footprint lúc spawn (m)'),
        DeclareLaunchArgument('yaw', default_value='3.14', description='hướng (rad), mặc định S1'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true',
                              description='diff-drive phát TF odom->base_footprint (false nếu dùng EKF)'),
        DeclareLaunchArgument('lidar_samples', default_value='3200',
                              description='số tia LiDAR mỗi vòng (3200 = 0.1125° như RPLIDAR S2)'),
        DeclareLaunchArgument('rviz', default_value='false', description='mở RViz2 với rviz/home_sim.rviz'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 'home_sim.rviz')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world,
                              'verbose': LaunchConfiguration('verbose'),
                              'pause': LaunchConfiguration('pause')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),

        # /robot_description (latched) + TF tĩnh của robot, dùng đồng hồ mô phỏng
        OpaqueFunction(function=_robot_state_publisher, args=[pkg], condition=spawn_robot),

        # RViz2 phải dùng đồng hồ mô phỏng, nếu không TF (đóng dấu giờ Gazebo) bị coi là quá cũ/quá mới
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Đọc /robot_description và gọi dịch vụ /spawn_entity của Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', ROBOT_NAME,
                       '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                       '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
            condition=spawn_robot),
    ])
