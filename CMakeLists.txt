cmake_minimum_required(VERSION 3.5)
project(laser_filters)

##############################################################################
# Find dependencies
##############################################################################
find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

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
)

find_package(ament_cmake REQUIRED)
foreach (dependency IN ITEMS ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach ()

##############################################################################
# Build
##############################################################################

include_directories(include ${Boost_INCLUDE_DIRS})

add_library(pointcloud_filters src/pointcloud_filters.cpp)
ament_target_dependencies(pointcloud_filters ${DEPENDENCIES})

add_library(laser_scan_filters
  src/laser_scan_filters.cpp
  src/median_filter.cpp
  src/array_filter.cpp
  src/box_filter.cpp
  src/polygon_filter.cpp
  src/scan_shadows_filter.cpp
  src/speckle_filter.cpp
  src/intensity_filter.cpp
  src/sector_filter.cpp
)
ament_target_dependencies(laser_scan_filters ${DEPENDENCIES})
# target_link_libraries(laser_scan_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
ament_target_dependencies(scan_to_cloud_filter_chain ${DEPENDENCIES})
target_link_libraries(scan_to_cloud_filter_chain laser_scan_filters)

add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
ament_target_dependencies(scan_to_scan_filter_chain ${DEPENDENCIES})
target_link_libraries(scan_to_scan_filter_chain laser_scan_filters)


add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
ament_target_dependencies(generic_laser_filter_node ${DEPENDENCIES})
target_link_libraries(generic_laser_filter_node laser_scan_filters)


##############################################################################
# Install
##############################################################################

pluginlib_export_plugin_description_file(filters laser_filters_plugins.xml)
ament_auto_package(INSTALL_TO_SHARE examples)

install(TARGETS 
  pointcloud_filters 
  laser_scan_filters 
  scan_to_cloud_filter_chain
  scan_to_scan_filter_chain
  generic_laser_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES laser_filters_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##############################################################################
# Test
##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  # if (CATKIN_ENABLE_PERFORMANCE_TESTING)
#     message(STATUS "Enabling performance tests")
#     add_definitions(-DENABLE_PERFORMANCE)
#   else()
#     message(STATUS "Disabled performance tests")
#   endif()

#   find_package(rostest)

    ament_add_gtest_executable(test_scan_filter_chain test/test_scan_filter_chain.cpp)
    target_link_libraries(test_scan_filter_chain laser_scan_filters)
    ament_target_dependencies(test_scan_filter_chain filters pluginlib rclcpp sensor_msgs)
    ament_add_test(
      test_scan_filter_chain
      COMMAND
        $<TARGET_FILE:test_scan_filter_chain>
        --ros-args --params-file ${PROJECT_SOURCE_DIR}/test/test_scan_filter_chain.yaml
    )
#   add_rostest_gtest(test_scan_filter_chain test/test_scan_filter_chain.launch test/test_scan_filter_chain.cpp)
#   target_link_libraries(test_scan_filter_chain laser_scan_filters ${catkin_LIBRARIES})
#   add_rostest(test/test_polygon_filter.launch)
#   add_rostest(test/test_speckle_filter.launch)

    # ament_add_gtest_executable(test_shadow_detector test/test_shadow_detector.cpp)
    # target_link_libraries(test_shadow_detector laser_scan_filters)

    # ament_add_gtest_executable(test_speckle_filter test/test_speckle_filter.cpp)
    # target_link_libraries(test_speckle_filter laser_scan_filters)

#   catkin_add_gtest(test_scan_shadows_filter test/test_scan_shadows_filter.cpp)
#   target_link_libraries(test_scan_shadows_filter laser_scan_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES})
endif()
