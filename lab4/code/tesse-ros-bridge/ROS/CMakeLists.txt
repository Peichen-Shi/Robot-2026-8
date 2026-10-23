cmake_minimum_required(VERSION 2.8.3)
project(tesse_ros_bridge)

find_package(catkin_simple REQUIRED)

catkin_python_setup()
catkin_simple()

cs_install()

# Install the launchable Python nodes with executable wrappers. This remains
# reliable after extracting a zip archive on a filesystem that does not retain
# Unix execute bits.
catkin_install_python(PROGRAMS
  src/tesse_ros_bridge/tesse_ros_node.py
  src/tesse_ros_bridge/quadrotor_control_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cs_export()
