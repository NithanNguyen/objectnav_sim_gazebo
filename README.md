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
| `launch/explore_sim.launch.py`, `rviz/explore_sim.rviz` | Gazebo + EKF + laser_filters + slam_toolbox + Nav2 (không AMCL) + khám phá frontier tự động |
| `scripts/frontier_explorer.py`, `scripts/frontier_core.py` | Node khám phá (ROS 2) và lõi thuật toán không phụ thuộc ROS |
| `tools/explore_level0.py` | Kiểm thử lõi khám phá ngoài ROS trên `maps/home_sim_map` (mô hình lưới) |
| `maps/home_sim_map.yaml` | Bản đồ căn hộ (slam_toolbox, robot xuất phát tại S1 → gốc frame map = S1) |
| `tools/check_env.sh` | Kiểm tra môi trường |
| `tools/gt_check.py` | Kiểm tra `/gazebo/get_entity_state` và `/gazebo/set_entity_state` |
| `tools/check_placements.py`, `tools/measure_surfaces.py`, `tools/plot_floorplan.py` | Công cụ hình học (cần `pip3 install trimesh pycollada pyyaml numpy`) |

## Chạy nhanh
```bash
cd ~/objectnav_sim_gazebo && colcon build --symlink-install && source install/setup.bash   # lần 1: để tìm thấy gói AWS
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
cd ~/objectnav_sim_gazebo && rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy   # bổ sung theo package.xml
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

## Khám phá tự động (frontier + slam_toolbox + Nav2)
Robot tự dựng bản đồ căn hộ chưa biết rồi dừng khi hết frontier tới được:
```bash
sudo apt install ros-foxy-slam-toolbox python3-scipy      # ngoài các gói ở mục Navigation
ros2 launch objectnav_sim_gazebo explore_sim.launch.py                            # S1, tự bắt đầu
ros2 launch objectnav_sim_gazebo explore_sim.launch.py x:=-5.5 y:=-0.3 yaw:=1.57  # S2
ros2 launch objectnav_sim_gazebo explore_sim.launch.py autostart:=false           # bắt đầu/dừng tay:
ros2 service call /explore/start std_srvs/srv/Trigger    # /explore/stop để dừng
ros2 topic echo /explore/status                           # kết quả cuối: thời gian, quãng đường, diện tích trống đã biết
ros2 run nav2_map_server map_saver_cli -f ~/explored_map --ros-args -p use_sim_time:=true
```
Thuật toán (`scripts/frontier_core.py`, mỗi chu kỳ 1 s trên `/map` của slam_toolbox):
1. Frontier = ô trống kề ô chưa biết; gom cụm (lân cận 8), bỏ cụm < `min_frontier_size` ô.
2. Chọn cụm có điểm tiếp cận gần nhất **theo đường đi** (Dijkstra trên vùng đã biết là trống, clearance ≥ `robot_radius`);
   điểm tiếp cận = ô đủ clearance, cách cụm ≤ `approach_radius`, gần tâm cụm nhất; hướng goal nhìn về tâm cụm.
3. Repetitive re-checking: cụm đang nhắm không còn là frontier thì chọn lại ngay (goal mới preempt goal cũ).
4. Nav2 thất bại / quá `goal_timeout`, hoặc tới nơi mà cụm vẫn mở và quanh cụm không thêm ô đã biết → 1 lần thất bại;
   `max_failures` lần → blacklist theo bán kính `blacklist_radius`.
5. Kết thúc khi hết frontier, hoặc `done_patience` chu kỳ liên tiếp không có cụm tới được (liệt kê frontier còn lại).

Tham số node: xem đầu `scripts/frontier_explorer.py`. Launch ghi đè khi khám phá (file config gốc giữ nguyên):
`map_update_interval` của slam_toolbox (mặc định 1.0 s, gốc 4.0 s), `FollowPath.max_vel_x` = `explore_max_vel_x` (0.30 m/s
như robot thật khi vừa SLAM vừa chạy), và chép `obstacle_max_range`/`raytrace_max_range` sang `obstacle_range`/`raytrace_range`
— tên mà Nav2 Foxy 0.4.7 đọc (`nav2_costmap_2d/plugins/obstacle_layer.cpp`); với `nav_sim.launch.py` các giá trị này hiện
đang bị bỏ qua (Foxy dùng mặc định 2.5 m / 3.0 m).

Kiểm thử lõi ngoài ROS (numpy, scipy, pyyaml; chạy được với numpy 1.17 / scipy 1.3 của Ubuntu 20.04):
```bash
python3 tools/explore_level0.py --random 10    # S1, S2 + 10 vị trí ngẫu nhiên trên maps/home_sim_map
```
Mô hình lưới này bỏ qua động lực học Nav2/DWB, trôi SLAM và độ trễ `/map`, nên chỉ dùng để kiểm logic; thời gian thật lấy từ Gazebo.
Diện tích trống của bản đồ tham chiếu `home_sim_map` là 153.9 m² (đối chiếu với "known free" trong `/explore/status`).

