"""Chạy Gazebo Classic 11 với căn hộ ảo home_trials.world và đặt robot (URDF v1) vào đó.

ros2 launch objectnav_sim_gazebo home_sim.launch.py                      # có giao diện, robot ở S1
ros2 launch objectnav_sim_gazebo home_sim.launch.py gui:=false           # chỉ gzserver (chạy hàng loạt lượt thử)
ros2 launch objectnav_sim_gazebo home_sim.launch.py x:=-5.5 y:=-0.3 yaw:=1.57   # xuất phát ở S2
ros2 launch objectnav_sim_gazebo home_sim.launch.py spawn_robot:=false   # chỉ căn hộ, không robot

Vị trí mặc định (x, y, yaw) = S1 trong config/trials.yaml.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = 'mobile_robot'


def _prepend_env_path(var, path):
    cur = os.environ.get(var, '')
    if path not in cur.split(os.pathsep):
        os.environ[var] = os.pathsep.join(p for p in [path, cur] if p)


def generate_launch_description():
    pkg = get_package_share_directory('objectnav_sim_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # gzserver.launch.py ghi đè GAZEBO_RESOURCE_PATH bằng media path của các gói (gói AWS có export),
    # nên nếu biến môi trường thiếu thư mục cài của Gazebo thì gzserver/gzclient không tìm thấy media mặc định.
    # Thêm lại /usr/share/gazebo-11 ở đây (tương đương 'source /usr/share/gazebo/setup.sh').
    _prepend_env_path('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')

    # Mesh của robot ghi là package://objectnav_sim_gazebo/meshes/...; khi chuyển URDF -> SDF, sdformat đổi
    # thành model://objectnav_sim_gazebo/meshes/..., nên thư mục CHA của share/objectnav_sim_gazebo phải nằm
    # trong GAZEBO_MODEL_PATH. gzserver/gzclient.launch.py nối thêm giá trị môi trường này vào đường dẫn của chúng.
    _prepend_env_path('GAZEBO_MODEL_PATH', os.path.dirname(pkg))

    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'mobile_robot.urdf.xacro')).toxml()

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world,
                              'verbose': LaunchConfiguration('verbose'),
                              'pause': LaunchConfiguration('pause')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),

        # /robot_description (latched) + TF tĩnh của robot, dùng đồng hồ mô phỏng
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            condition=spawn_robot),

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
