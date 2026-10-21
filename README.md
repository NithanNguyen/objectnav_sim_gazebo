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
