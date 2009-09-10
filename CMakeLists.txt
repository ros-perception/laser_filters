cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(laser_filters)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library(pointcloud_filters src/pointcloud_filters.cpp)

rospack_add_library(laser_scan_filters src/laser_scan_filters.cpp src/median_filter.cpp)

rospack_add_executable(scan_to_cloud_filter_chain src/scan_to_cloud.cpp)
target_link_libraries (scan_to_cloud_filter_chain laser_scan)


rospack_add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
target_link_libraries (generic_laser_filter_node laser_scan)

