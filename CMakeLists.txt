cmake_minimum_required(VERSION 3.5)
project(objectnav_sim_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models config tools urdf meshes rviz maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Node khám phá frontier (launch/explore_sim.launch.py); frontier_core.py nằm cùng thư mục để node import được.
install(PROGRAMS scripts/frontier_explorer.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/frontier_core.py DESTINATION lib/${PROJECT_NAME})

ament_package()
