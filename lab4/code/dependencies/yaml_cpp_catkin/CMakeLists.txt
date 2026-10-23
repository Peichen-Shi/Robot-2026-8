cmake_minimum_required(VERSION 3.0.2)
project(yaml_cpp_catkin)

find_package(catkin REQUIRED)
find_package(yaml-cpp REQUIRED)

# Ubuntu 20.04 provides yaml-cpp 0.6.2.  Export the system library under the
# historic catkin package name expected by this VNAV2021 dependency snapshot.
catkin_package(LIBRARIES yaml-cpp)
