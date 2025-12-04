cmake_minimum_required(VERSION 3.0.2)
project(drone)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  vision_msgs
  mavros_msgs
  cv_bridge
  std_srvs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/node_yolo.py
    scripts/node_analyze.py
    scripts/node_server.py
    scripts/node_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
