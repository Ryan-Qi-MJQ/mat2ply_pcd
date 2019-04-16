cmake_minimum_required(VERSION 3.13)
project(mat_point)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})

add_executable(mat_point main.cpp pcd_select.cpp)
target_link_libraries(mat_point ${PCL_LIBRARIES} ${OpenCV_LIBS})
