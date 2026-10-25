cmake_minimum_required(VERSION 2.8.3)
project(controller_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs
  mav_msgs
  nav_msgs
  trajectory_msgs
  visualization_msgs
  tf
  tf2
  tf2_eigen
  tf2_geometry_msgs
  message_generation
  trajectory_generation
)

add_message_files(
  FILES
  ControllerDiagnostics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs
  mav_msgs
  nav_msgs
  trajectory_msgs
  visualization_msgs
  tf
  tf2
  tf2_eigen
  tf2_geometry_msgs
  message_runtime
  trajectory_generation
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_executable(controller_node src/controller_node.cpp)
add_executable(feedforward_test src/feedforward_test.cpp)
add_executable(reference_prediction_test src/reference_prediction_test.cpp)
target_link_libraries(reference_prediction_test ${catkin_LIBRARIES})
target_link_libraries(feedforward_test ${catkin_LIBRARIES})
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node ${catkin_LIBRARIES})

add_executable(traj_publisher src/traj_publisher.cpp)
add_dependencies(traj_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_publisher ${catkin_LIBRARIES})

add_executable(lab3_visualization_node src/lab3_visualization_node.cpp)
add_dependencies(lab3_visualization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lab3_visualization_node ${catkin_LIBRARIES})

add_executable(lab4_visualization_node src/lab4_visualization_node.cpp)
add_dependencies(lab4_visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lab4_visualization_node ${catkin_LIBRARIES})

add_executable(controller_math_test src/controller_math_test.cpp)
add_dependencies(controller_math_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_math_test ${catkin_LIBRARIES})

add_executable(deliverable_1_api_test src/deliverable_1_api_test.cpp)
add_dependencies(deliverable_1_api_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(deliverable_1_api_test ${catkin_LIBRARIES})

install(TARGETS controller_node traj_publisher lab3_visualization_node
  lab4_visualization_node controller_math_test deliverable_1_api_test feedforward_test
  reference_prediction_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/lab4_diagnostic_reference.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  enable_testing()
  add_test(NAME allocation_math COMMAND controller_math_test)
  add_test(NAME quaternion_conversions COMMAND deliverable_1_api_test)
  add_test(NAME so3_feedforward COMMAND feedforward_test)
  add_test(NAME reference_prediction COMMAND reference_prediction_test)
endif()
