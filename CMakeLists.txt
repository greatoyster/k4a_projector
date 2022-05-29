cmake_minimum_required(VERSION 3.0.2)
project(k4a_projector)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rosbag
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(k4a REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
  ${OpenCV_LIBRARY_DIRS}
)

add_executable(k4a_projector src/main.cpp)

target_link_libraries(k4a_projector k4a yaml-cpp ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES})
