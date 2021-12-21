#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cmath>
#include "laser_filters/scan_shadows_filter.h"
#include "sensor_msgs/LaserScan.h"

sensor_msgs::LaserScan create_message(
    float ranges[], int num_beams
) {
    sensor_msgs::LaserScan msg;

    std::vector<float> v_range(ranges, ranges + num_beams);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser";
    // Use a small beam so that angle_increment remains small (realistic) also with few points
    msg.angle_min = -M_PI / 12;
    msg.angle_max = M_PI / 12;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / (num_beams - 1);
    msg.time_increment = 0.1 / num_beams;
    msg.scan_time = 0.1;
    msg.range_min = 0.1;
    msg.range_max = 10;
    msg.ranges = v_range;

    return msg;
}

/**
 * Verifies that two vectors of range values are the same. Allows the case
 * where corresponding values are both NaN.
 */
void expect_ranges_equal(const std::vector<float> &actual, const float expected[], int expected_len) {
    EXPECT_EQ(expected_len, actual.size());
    for (int i = 0; i < expected_len; i++) {
        if (std::isnan(expected[i])) {
            EXPECT_TRUE(std::isnan(actual[i])) << "Mismatch at index " << i << std::endl;
        }
        else {
            EXPECT_NEAR(expected[i], actual[i], 1e-6) << "Mismatch at index " << i << std::endl;
        }
    }
}

void expect_ranges_equal(const std::vector<float> &actual, const std::vector<float> expected) {
    expect_ranges_equal(actual, &expected[0], expected.size());
}

TEST(ScanShadowsFilter, NoShadows) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 1;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    float ranges[] = {9, 9, 9, 9, 9, 9, 9, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {9, 9, 9, 9, 9, 9, 9, 9, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, DistanceDeltaWithoutShadow) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 0;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    // This input data is very much simplified. The range-5 points represent a nearby object, and
    // the range-9 points a wall.
    float ranges[] = {5, 5, 5, 5, 5, 9, 9, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Below is what is expected given the filter's current logic. However, it shows that the
    // filter is primitive and can filter out point that are not shadows.
    float expected[] = {5, 5, 5, 5, NAN, NAN, 9, 9, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, DistanceDeltaWithoutShadow_Angles_8_172) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 8.0;
    config.max_angle = 172.0;
    config.neighbors = 0;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    float ranges[] = {5, 5, 5, 5, 5, 9, 9, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Increasing the angle range still results in the same filter behaviour.
    float expected[] = {5, 5, 5, 5, NAN, NAN, 9, 9, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, DistanceDeltaWithoutShadow_Angles_5_175) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 5.0;
    config.max_angle = 175.0;
    config.neighbors = 0;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    float ranges[] = {5, 5, 5, 5, 5, 9, 9, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Increasing the angle range more, filters out fewer points
    float expected[] = {5, 5, 5, 5, 5, NAN, 9, 9, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, DistanceDeltaWithoutShadowFlipped_Angles_5_175) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 5.0;
    config.max_angle = 175.0;
    config.neighbors = 0;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    float ranges[] = {9, 9, 9, 9, 9, 5, 5, 5, 5, 5};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Reversing the input results in reversed output
    float expected[] = {9, 9, 9, 9, NAN, 5, 5, 5, 5, 5};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, DistanceDeltaWithoutShadow_Angles_3_177) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 3.0;
    config.max_angle = 177.0;
    config.neighbors = 0;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    float ranges[] = {5, 5, 5, 5, 5, 9, 9, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Increasing the range even more, no more points are filtered out
    float expected[] = {5, 5, 5, 5, 5, 9, 9, 9, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, SingleBackwardShadow_NoNeighbours) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 0;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    // This input data is very much simplified. The range-5 points represent a nearby object, and
    // the range-9 points a wall. The range-7 point is a shadow.
    float ranges[] = {5, 5, 5, 5, 5, 9, 7, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // The shadow is filtered out, as well as some other points
    float expected[] = {5, 5, 5, 5, NAN, NAN, NAN, NAN, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, SingleBackwardShadow_OneNeighbour) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 1;
    config.window = 1;
    config.remove_shadow_start_point = false;

    filter.reconfigureCB(config, 0);

    float ranges[] = {5, 5, 5, 5, 5, 9, 7, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Below is what is expected given the filter's current logic. Note, this configuration does
    // not filter out the simulated shadow but some other points.
    float expected[] = {5, 5, 5, 5, 5, NAN, 7, NAN, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, SingleForwardShadow_NoNeighbours) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 0;
    config.window = 1;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    // This input data is very much simplified. The range-5 points represent a nearby object, and
    // the range-9 points a wall. The range-3 point is a shadow.
    float ranges[] = {5, 5, 5, 5, 5, 9, 3, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // The shadow is filtered out, as well as some other points
    float expected[] = {5, 5, 5, 5, NAN, NAN, NAN, NAN, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, SingleForwardShadow_OneNeighbour) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 1;
    config.window = 1;
    config.remove_shadow_start_point = false;

    filter.reconfigureCB(config, 0);

    float ranges[] = {5, 5, 5, 5, 5, 9, 3, 9, 9, 9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Below is what is expected given the filter's current logic. Note, this configuration does
    // not filter out the simulated shadow but some other points.
    float expected[] = {5, 5, 5, 5, 5, NAN, 3, NAN, 9, 9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(ScanShadowsFilter, SingleForwardShadow_AngleIncrementChanged) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 1;
    config.window = 1;
    config.remove_shadow_start_point = false;

    filter.reconfigureCB(config, 0);

    float ranges[] = {5, 5, 5, 5, 5, 4, 4, 4, 4, 4};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    // Below is what is expected given the filter's current logic. Note, this configuration does
    // not filter out the simulated shadow but some other points.
    float expected[] = {5, 5, 5, 5, 5, 4, 4, 4, 4, 4};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));

    // Smaller angle_increment, results in steeper angles and shadow detection
    input_scan.angle_increment /= 2;    
    filter.update(input_scan, output_scan);

    float new_expected[] = {5, 5, 5, 5, NAN, 4, 4, 4, 4, 4};

    expect_ranges_equal(output_scan.ranges, new_expected, sizeof(new_expected) / sizeof(float));
}

#ifdef ENABLE_PERFORMANCE
TEST(ScanShadowsFilter, Performance) {
    laser_filters::ScanShadowsFilter filter;
    laser_filters::ScanShadowsFilterConfig config;

    config.min_angle = 15.0;
    config.max_angle = 165.0;
    config.neighbors = 2;
    config.window = 2;
    config.remove_shadow_start_point = true;

    filter.reconfigureCB(config, 0);

    float ranges[] = {};
    sensor_msgs::LaserScan input_scan[2];
    input_scan[0] = create_message(ranges, 0);
    int num_samples = 1024;
    int next_index = 0;
    int pending = 0;
    float range_val = 5;
    while (input_scan[0].ranges.size() < num_samples) {
        if (pending == 0) {
            range_val = 5 + next_index;
            pending = next_index + 1;

            if (next_index == 10) {
                range_val = range_val + 20;
                next_index = 0;
            } else {
                ++next_index;
            }
        }

        input_scan[0].ranges.push_back(range_val);
        --pending;
    }

    // Create second input set. A mirror of the first
    input_scan[1] = input_scan[0];
    std::reverse(input_scan[1].ranges.begin(), input_scan[1].ranges.end());

    sensor_msgs::LaserScan output_scan;
    sensor_msgs::LaserScan expected_scan[2];
    float expected_data[2][num_samples];

    // Create expected output by running filter once.

    // The main purpose is to check that when executing repeatedly, the expected output
    // alternates.
    for (int j = 0; j < 2; j++) {
        filter.update(input_scan[j], expected_scan[j]);
    }

    for (int i = 0; i < 10000; i++) {
        int j = i % 2;
        filter.update(input_scan[j], output_scan);

        expect_ranges_equal(output_scan.ranges, expected_scan[j].ranges);
    }
}
#endif

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_scan_shadows_filter");
    ros::Time::init();
    return RUN_ALL_TESTS();
}