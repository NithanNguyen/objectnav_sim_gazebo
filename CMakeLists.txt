cmake_minimum_required(VERSION 3.5)
project(objectnav_sim_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models config tools urdf meshes rviz maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
