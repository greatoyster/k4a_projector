cmake_minimum_required(VERSION 3.0.2)
project(k4a_projector)
find_package(catkin REQUIRED)
set(CMAKE_CXX_STANDARD 17)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_CXX_FLAGS "-O3 -fopenmp")
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
find_package(k4a REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} k4a::k4a Eigen3::Eigen)

cs_install()