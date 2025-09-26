cmake_minimum_required(VERSION 3.0.2)
project(clover_payload)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  gazebo_msgs
  gazebo_ros_link_attacher
  tf2_ros
  geometry_msgs
)

catkin_package()

# Установка скриптов
catkin_install_python(PROGRAMS
  scripts/payload_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Установка моделей Gazebo
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

# Установка launch-файлов
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)