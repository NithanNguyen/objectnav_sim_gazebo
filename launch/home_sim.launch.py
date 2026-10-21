"""Chạy Gazebo Classic 11 với căn hộ ảo home_trials.world.

ros2 launch objectnav_sim_gazebo home_sim.launch.py              # có giao diện
ros2 launch objectnav_sim_gazebo home_sim.launch.py gui:=false   # chỉ gzserver (chạy hàng loạt lượt thử)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'home_trials.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world,
                              'verbose': LaunchConfiguration('verbose'),
                              'pause': LaunchConfiguration('pause')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
    ])
