cmake_minimum_required(VERSION 2.4.6)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

rosbuild_add_library(pointcloud_filters src/pointcloud_filters.cpp)

rosbuild_add_library(laser_scan_filters src/laser_scan_filters.cpp src/median_filter.cpp src/array_filter.cpp)
rosbuild_link_boost(laser_scan_filters system)

rosbuild_add_executable(scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
rosbuild_link_boost(scan_to_cloud_filter_chain system signals)
rosbuild_add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
rosbuild_link_boost(scan_to_scan_filter_chain system signals)

rosbuild_add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
rosbuild_link_boost(generic_laser_filter_node system signals)

#test scan filter chain filter
rosbuild_add_executable(test_scan_filter_chain EXCLUDE_FROM_ALL test/test_scan_filter_chain.cpp)
target_link_libraries(test_scan_filter_chain laser_scan_filters)
rosbuild_add_gtest_build_flags(test_scan_filter_chain)
rosbuild_add_rostest(test/test_scan_filter_chain.launch)
