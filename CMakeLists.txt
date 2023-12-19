cmake_minimum_required(VERSION 3.5)
project(laser_filters)

##############################################################################
# Find dependencies
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(DEPENDENCIES
  sensor_msgs 
  rclcpp 
  tf2 
  tf2_ros
  filters 
  message_filters
  laser_geometry 
  pluginlib 
  angles 
  ddynamic_reconfigure
)

find_package(ament_cmake REQUIRED)
foreach (dependency IN ITEMS ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach ()


##############################################################################
# Define package
##############################################################################

# # dynamic reconfigure
# generate_dynamic_reconfigure_options(
#   cfg/IntensityFilter.cfg
#   cfg/PolygonFilter.cfg
#   cfg/RangeFilter.cfg
#   cfg/ScanShadowsFilter.cfg
#   cfg/SpeckleFilter.cfg
#   cfg/SectorFilter.cfg
#   cfg/BoxFilter.cfg
# )

##############################################################################
# Build
##############################################################################

add_library(pointcloud_filters src/pointcloud_filters.cpp)
target_link_libraries(pointcloud_filters ${catkin_LIBRARIES})
ament_target_dependencies(pointcloud_filters ${DEPENDENCIES})

add_library(laser_scan_filters
  src/laser_scan_filters.cpp
  src/median_filter.cpp
  src/array_filter.cpp
  src/box_filter.cpp
  src/polygon_filter.cpp
  src/scan_shadow_detector.cpp
  src/scan_shadows_filter.cpp
  src/speckle_filter.cpp
  src/intensity_filter.cpp
  src/sector_filter.cpp
)
ament_target_dependencies(laser_scan_filters ${DEPENDENCIES})
target_link_libraries(laser_scan_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
ament_target_dependencies(scan_to_cloud_filter_chain ${DEPENDENCIES})
target_link_libraries(scan_to_cloud_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
ament_target_dependencies(scan_to_scan_filter_chain ${DEPENDENCIES})
target_link_libraries(scan_to_scan_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_library(${PROJECT_NAME}_nodelets
#   src/scan_to_cloud_filter_chain.cpp
#   src/scan_to_scan_filter_chain.cpp)
# target_link_libraries(${PROJECT_NAME}_nodelets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# target_compile_definitions(${PROJECT_NAME}_nodelets PRIVATE BUILDING_NODELET=1)

add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
ament_target_dependencies(generic_laser_filter_node ${DEPENDENCIES})
target_link_libraries(generic_laser_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(laser_scan_filters ${PROJECT_NAME}_gencfg)

# if (CATKIN_ENABLE_TESTING)
#   if (CATKIN_ENABLE_PERFORMANCE_TESTING)
#     message(STATUS "Enabling performance tests")
#     add_definitions(-DENABLE_PERFORMANCE)
#   else()
#     message(STATUS "Disabled performance tests")
#   endif()

#   find_package(rostest)

#   add_rostest_gtest(test_scan_filter_chain test/test_scan_filter_chain.launch test/test_scan_filter_chain.cpp)
#   target_link_libraries(test_scan_filter_chain laser_scan_filters ${catkin_LIBRARIES})
#   add_rostest(test/test_polygon_filter.launch)
#   add_rostest(test/test_speckle_filter.launch)

#   catkin_add_gtest(test_shadow_detector test/test_shadow_detector.cpp)
#   target_link_libraries(test_shadow_detector laser_scan_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES})

#   catkin_add_gtest(test_speckle_filter test/test_speckle_filter.cpp)
#   target_link_libraries(test_speckle_filter laser_scan_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES})

#   catkin_add_gtest(test_scan_shadows_filter test/test_scan_shadows_filter.cpp)
#   target_link_libraries(test_scan_shadows_filter laser_scan_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES})
# endif()

##############################################################################
# Install
##############################################################################

pluginlib_export_plugin_description_file(filters laser_filters_plugins.xml)
# ament_auto_package(INSTALL_TO_SHARE examples)

install(TARGETS 
  pointcloud_filters 
  laser_scan_filters 
  scan_to_cloud_filter_chain
  scan_to_scan_filter_chain
  # ${PROJECT_NAME}_nodelets
  # generic_laser_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(FILES laser_filters_plugins.xml nodelets.xml
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

