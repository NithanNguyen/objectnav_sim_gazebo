# home_sim_world

Căn hộ ảo cho mô phỏng HIL (đề tài CE407): **AWS Small House** + vật cần tìm + plugin `gazebo_ros_state`.
ROS 2 Foxy, Gazebo Classic 11, Ubuntu 20.04. Hướng dẫn đầy đủ nằm trong tài liệu "Hướng dẫn cài đặt Gazebo Classic 11 và căn hộ ảo".

## Cấu trúc
| Đường dẫn | Nội dung |
|---|---|
| `worlds/home_trials.world` | World AWS + plugin ground truth + vật cần tìm (sinh bởi `tools/build_world.py`) |
| `models/obj_*` | Vật hình khối đơn giản: điện thoại, điều khiển TV, kính, ví, gậy, chai nước, túi xách |
| `models/gso_*` | Vật GSO, sinh bởi `tools/fetch_gso.py` (chưa có sẵn) |
| `config/trials.yaml` | 2 vị trí xuất phát, 10 vị trí đặt vật (7 dễ thấy / 3 bị che) |
| `config/surfaces.yaml`, `config/floorplan.png` | Độ cao bề mặt đặt vật đo từ mesh AWS, mặt bằng nhìn từ trên |
| `launch/home_sim.launch.py` | Chạy gzserver (+ gzclient nếu `gui:=true`) |
| `launch/slam_sim.launch.py` | Gazebo + EKF + laser_filters + slam_toolbox (bản mô phỏng của `slam_v3_launch.py`) |
| `launch/nav_sim.launch.py` | Gazebo + EKF + laser_filters + Nav2/AMCL (bản mô phỏng của `nav_v3.launch.py`) |
| `config/nav/nav2_params.yaml`, `behavior_trees/` | Nav2 params và BT chép từ `receptionist_robot` (chỉ sửa chỗ `[SIM]`) |
| `maps/home_sim_map.yaml` | Bản đồ căn hộ (slam_toolbox, robot xuất phát tại S1 → gốc frame map = S1) |
| `tools/check_env.sh` | Kiểm tra môi trường |
| `tools/gt_check.py` | Kiểm tra `/gazebo/get_entity_state` và `/gazebo/set_entity_state` |
| `tools/check_placements.py`, `tools/measure_surfaces.py`, `tools/plot_floorplan.py` | Công cụ hình học (cần `pip3 install trimesh pycollada pyyaml numpy`) |

## Chạy nhanh
```bash
cd ~/sim_ws && colcon build --symlink-install && source install/setup.bash   # lần 1: để tìm thấy gói AWS
python3 src/home_sim_world/tools/fetch_gso.py src/home_sim_world/models mug=Room_Essentials_Mug_White_Yellow
python3 src/home_sim_world/tools/build_world.py
colcon build --symlink-install && source install/setup.bash                  # lần 2: cài model/world mới
ros2 launch home_sim_world home_sim.launch.py
```

Quy ước: mọi model vật có gốc ở **tâm đáy** và là `static`, nên `z` của pose = độ cao bề mặt đỡ.

## Navigation (Nav2 + AMCL)
Cài gói (ROS 2 Foxy, Ubuntu 20.04):
```bash
sudo apt update
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup \
                 ros-foxy-robot-localization ros-foxy-laser-filters ros-foxy-xacro
cd ~/sim_ws && rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy   # bổ sung theo package.xml
colcon build --symlink-install && source install/setup.bash
```
Chạy:
```bash
ros2 launch objectnav_sim_gazebo nav_sim.launch.py                            # robot ở S1
ros2 launch objectnav_sim_gazebo nav_sim.launch.py x:=-5.5 y:=-0.3 yaw:=1.57  # robot ở S2
```
Gửi đích bằng nút **Navigation2 Goal** trong RViz, hoặc:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
Đích tính trong frame **map** (gốc = S1, lệch world của Gazebo một phép quay ~π): world (x, y) ↔ map (3 − x, −3 − y).
Nếu dựng lại bản đồ từ vị trí khác, truyền `map_x:= map_y:= map_yaw:=` = vị trí spawn lúc chạy `slam_sim.launch.py`.
