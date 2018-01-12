cmake_minimum_required(VERSION 3.5)
project(laser_filters)

##############################################################################
# Find dependencies
##############################################################################

find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(filters REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(angles REQUIRED)
find_package(rostime REQUIRED)

find_package(Eigen3 REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS system signals thread filesystem)

find_package(TinyXML REQUIRED HINTS "*.lib") # add hint seems to allow debug lib to be found when appropriate

include_directories(include 
  ${pluginlib_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${filters_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS}
  ${angles_INCLUDE_DIRS}
  ${rostime_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pluginlib_LIBRARY_DIRS}
  ${class_loader_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS}
  ${filters_LIBRARY_DIRS}
  ${message_filters_LIBRARY_DIRS}
  ${tf2_ros_LIBRARY_DIRS}
  ${laser_geometry_LIBRARY_DIRS}
  ${angles_LIBRARY_DIRS}
  ${rostime_LIBRARY_DIRS}
)

add_definitions(-DTRANSFORM_LISTENER_NOT_IMPLEMENTED)


set(${PROJECT_NAME}_LIBRARIES 
  ${pluginlib_LIBRARIES}
  ${class_loader_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${filters_LIBRARIES}
  ${message_filters_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${laser_geometry_LIBRARIES}
  ${angles_LIBRARIES}
  ${rostime_LIBRARIES}
  ${Boost_LIBRARIES}
)

##############################################################################
# Build
##############################################################################

add_library(pointcloud_filters src/pointcloud_filters.cpp)
target_link_libraries(pointcloud_filters ${${PROJECT_NAME}_LIBRARIES})

add_library(laser_scan_filters src/laser_scan_filters.cpp src/median_filter.cpp src/array_filter.cpp src/box_filter.cpp)
target_link_libraries(laser_scan_filters ${${PROJECT_NAME}_LIBRARIES})

add_executable(scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
target_link_libraries(scan_to_cloud_filter_chain ${${PROJECT_NAME}_LIBRARIES})

add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
target_link_libraries(scan_to_scan_filter_chain ${${PROJECT_NAME}_LIBRARIES})

add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
target_link_libraries(generic_laser_filter_node ${${PROJECT_NAME}_LIBRARIES})

pluginlib_export_plugin_description_file(laser_filters laser_filters_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(
  pointcloud_filters 
  laser_scan_filters 
  scan_to_cloud_filter_chain
  scan_to_scan_filter_chain
  generic_laser_filter_node
)
ament_package()

##############################################################################
# Install
##############################################################################

install(TARGETS pointcloud_filters laser_scan_filters 
  scan_to_cloud_filter_chain
  scan_to_scan_filter_chain
  generic_laser_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

install(FILES laser_filters_plugins.xml
    DESTINATION share
)
