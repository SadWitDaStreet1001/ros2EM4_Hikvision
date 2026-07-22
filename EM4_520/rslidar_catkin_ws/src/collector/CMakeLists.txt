cmake_minimum_required(VERSION 3.0.2)
project(collector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/camera_node.py
  scripts/collector_node.py
  scripts/zed/zed_node.py
  scripts/zed/zed_collector_node.py
  scripts/hikvision/hikvision_node.py
  scripts/hikvision/hikvision_collector_node.py
  scripts/hikvision/hikvision_photo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
