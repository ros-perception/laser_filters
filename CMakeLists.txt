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

if(${rclcpp_VERSION_MAJOR} GREATER_EQUAL 20)
    add_compile_definitions(RCLCPP_SUPPORTS_MATCHED_CALLBACKS)
endif()

ament_auto_add_library(laser_scan_filters SHARED src/laser_scan_filters.cpp)
ament_auto_add_library(laser_filter_chains SHARED
                         src/scan_to_cloud_filter_chain.cpp
                         src/scan_to_scan_filter_chain.cpp)

# The rclcpp_components_register_node macro registers an rclcpp component for the node with
# the ament resource index AND creates a templated executable that spins the given node.

rclcpp_components_register_node(
    laser_filter_chains
    PLUGIN "ScanToCloudFilterChain"
    EXECUTABLE "scan_to_cloud_filter_chain"
)

rclcpp_components_register_node(
    laser_filter_chains
    PLUGIN "ScanToScanFilterChain"
    EXECUTABLE "scan_to_scan_filter_chain"
)

ament_auto_add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)

##############################################################################
# Install
##############################################################################

pluginlib_export_plugin_description_file(filters laser_filters_plugins.xml)
ament_auto_package(INSTALL_TO_SHARE examples test)

##############################################################################
# Test
##############################################################################

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    set(TEST_NAME test_scan_filter_chain)
    set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
    ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME}
        pluginlib::pluginlib
        rclcpp::rclcpp
        filters::filter_chain
        ${sensor_msgs_TARGETS}
    )
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
    target_link_libraries(${TEST_NAME}
        angles::angles
        laser_scan_filters
    )
    ament_add_test(
        ${TEST_NAME}
        COMMAND
            $<TARGET_FILE:${TEST_NAME}>
            --ros-args
            --gtest_output=xml:${RESULT_FILENAME}
        RESULT_FILE ${RESULT_FILENAME}
    )

    set(TEST_NAME test_speckle_filter)
    set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
    ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} angles::angles laser_scan_filters)
    ament_add_test(
        ${TEST_NAME}
        COMMAND
            $<TARGET_FILE:${TEST_NAME}>
            --ros-args
            --gtest_output=xml:${RESULT_FILENAME}
        RESULT_FILE ${RESULT_FILENAME}
    )

    set(TEST_NAME test_scan_shadows_filter)
    set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
    ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME}
        laser_scan_filters
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
    )
    ament_add_test(
        ${TEST_NAME}
        COMMAND
            $<TARGET_FILE:${TEST_NAME}>
            --ros-args
            --gtest_output=xml:${RESULT_FILENAME}
        RESULT_FILE ${RESULT_FILENAME}
    )

    find_package(launch_testing_ament_cmake)
    add_launch_test(test/test_polygon_filter.test.py)
    add_launch_test(test/test_speckle_filter.test.py)
endif()
