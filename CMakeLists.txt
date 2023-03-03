cmake_minimum_required(VERSION 3.5)
project(laser_filters)

##############################################################################
# Find dependencies
##############################################################################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

##############################################################################
# Build
##############################################################################

ament_auto_add_library(laser_scan_filters SHARED src/laser_scan_filters.cpp)

set(FILTER_CHAINS
    scan_to_cloud_filter_chain
    scan_to_scan_filter_chain
    generic_laser_filter_node
)
foreach(FILTER_CHAIN ${FILTER_CHAINS})
    ament_auto_add_executable(${FILTER_CHAIN} src/${FILTER_CHAIN}.cpp)
endforeach()

##############################################################################
# Install
##############################################################################

pluginlib_export_plugin_description_file(filters laser_filters_plugins.xml)
ament_auto_package(INSTALL_TO_SHARE examples)

##############################################################################
# Test
##############################################################################

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    set(TEST_NAME test_scan_filter_chain)
    set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
    ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
    ament_target_dependencies(${TEST_NAME} filters pluginlib rclcpp sensor_msgs)
    ament_add_test(
        ${TEST_NAME}
        COMMAND
            $<TARGET_FILE:${TEST_NAME}>
            --ros-args --params-file ${PROJECT_SOURCE_DIR}/test/${TEST_NAME}.yaml
            --gtest_output=xml:${RESULT_FILENAME}
        RESULT_FILE ${RESULT_FILENAME}
    )

    set(TEST_NAME test_shadow_detector)
    set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
    ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
    ament_target_dependencies(${TEST_NAME} angles)
    target_link_libraries(${TEST_NAME} laser_scan_filters)
    ament_add_test(
        ${TEST_NAME}
        COMMAND
            $<TARGET_FILE:${TEST_NAME}>
            --ros-args
            --gtest_output=xml:${RESULT_FILENAME}
        RESULT_FILE ${RESULT_FILENAME}
    )
endif()
