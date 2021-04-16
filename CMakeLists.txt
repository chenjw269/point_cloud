cmake_minimum_required(VERSION 3.15)
project(software_engineering)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        roscpp
        )

catkin_package(
        CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
)

find_package(PCL 1.8 REQUIRED)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(software_engineering main.cpp pointcloud_op.cpp pointcloud_op.h)

target_link_libraries(
        software_engineering
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)