#!/usr/bin/env bash
# Kiểm tra nhanh môi trường mô phỏng trên laptop. Không sửa gì trên máy.
# Dùng: bash check_env.sh   (sau khi đã source /opt/ros/foxy/setup.bash và workspace)
ok()   { printf '  [OK]   %s\n' "$*"; }
bad()  { printf '  [LỖI]  %s\n' "$*"; FAIL=1; }
warn() { printf '  [CHÚ Ý] %s\n' "$*"; }
FAIL=0

echo "== Hệ điều hành"
. /etc/os-release
[ "$VERSION_ID" = "20.04" ] && ok "Ubuntu $VERSION_ID" || bad "Cần Ubuntu 20.04, đang là $PRETTY_NAME"
grep -qi microsoft /proc/version && bad "Đang chạy trong WSL -> cần Ubuntu cài trực tiếp" || ok "Không phải WSL"
systemd-detect-virt -q 2>/dev/null && warn "Đang chạy trong máy ảo ($(systemd-detect-virt))" || ok "Không phải máy ảo"

echo "== GPU"
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  ok "$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader)"
else bad "nvidia-smi không chạy -> chưa có driver NVIDIA"; fi
command -v prime-select >/dev/null && echo "  prime-select: $(prime-select query 2>/dev/null)"
if command -v glxinfo >/dev/null; then
  R=$(glxinfo -B 2>/dev/null | grep -i "OpenGL renderer" | cut -d: -f2)
  echo "$R" | grep -qi nvidia && ok "OpenGL renderer:$R" || bad "OpenGL renderer:$R (không phải NVIDIA)"
else warn "Chưa có glxinfo (sudo apt install mesa-utils)"; fi

echo "== ROS 2 / Gazebo"
[ "$ROS_DISTRO" = "foxy" ] && ok "ROS_DISTRO=foxy" || bad "ROS_DISTRO='$ROS_DISTRO' (chưa source /opt/ros/foxy/setup.bash?)"
command -v gazebo >/dev/null && ok "$(gazebo --version 2>/dev/null | head -1)" || bad "Chưa có lệnh gazebo"
for p in ros-foxy-desktop ros-foxy-gazebo-ros-pkgs gazebo11 python3-colcon-common-extensions python3-rosdep; do
  dpkg -s "$p" >/dev/null 2>&1 && ok "$p" || bad "Thiếu gói $p"
done
echo "$GAZEBO_RESOURCE_PATH" | tr ':' '\n' | grep -qx /usr/share/gazebo-11 \
  && ok "GAZEBO_RESOURCE_PATH có /usr/share/gazebo-11" \
  || warn "GAZEBO_RESOURCE_PATH chưa có /usr/share/gazebo-11 (thêm 'source /usr/share/gazebo/setup.sh' vào ~/.bashrc)"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<mặc định>}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

echo "== Navigation (launch/nav_sim.launch.py)"
for p in ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-robot-localization ros-foxy-laser-filters; do
  dpkg -s "$p" >/dev/null 2>&1 && ok "$p" || bad "Thiếu gói $p (sudo apt install $p)"
done

echo "== Workspace"
for p in aws_robomaker_small_house_world objectnav_sim_gazebo; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && ok "Gói $p" || bad "Không thấy gói $p (chưa build hoặc chưa source install/setup.bash)"
done
[ $FAIL = 0 ] && echo "Kết quả: ĐẠT" || echo "Kết quả: còn lỗi, xem các dòng [LỖI]"
exit $FAIL
