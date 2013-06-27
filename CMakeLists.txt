cmake_minimum_required(VERSION 2.8.3)
project(laser_filters)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS sensor_msgs roscpp tf filters message_filters
  laser_geometry pluginlib pcl_ros)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS} rostest)
find_package(Boost REQUIRED COMPONENTS system signals)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_filters laser_scan_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
  )

##############################################################################
# Build
##############################################################################

add_library(pointcloud_filters src/pointcloud_filters.cpp)
target_link_libraries(pointcloud_filters ${catkin_LIBRARIES})

add_library(laser_scan_filters src/laser_scan_filters.cpp src/median_filter.cpp src/array_filter.cpp)
target_link_libraries(laser_scan_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES})

add_executable(scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
target_link_libraries(scan_to_cloud_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES})

add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
target_link_libraries(scan_to_scan_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES})

add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
target_link_libraries(generic_laser_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES})

catkin_add_gtest(test_scan_filter_chain EXCLUDE_FROM_ALL test/test_scan_filter_chain.cpp)
target_link_libraries(test_scan_filter_chain laser_scan_filters)

add_rostest(test/test_scan_filter_chain.launch)

##############################################################################
# Install
##############################################################################

install(TARGETS pointcloud_filters laser_scan_filters 
  scan_to_cloud_filter_chain
  scan_to_scan_filter_chain
  generic_laser_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES laser_filters_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
