#include <gtest/gtest.h>
#include <ros/ros.h>
#include <laser_filters/speckle_filter.h>
#include "sensor_msgs/LaserScan.h"

/*
Notes:
- The tests below only use/test the Distance window validator. There are no tests for the radius
  outlier window validator yet.
*/

sensor_msgs::LaserScan create_message(
    float ranges[], int num_beams
) {
    sensor_msgs::LaserScan msg;

    std::vector<float> v_range(ranges, ranges + num_beams);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser";
    msg.angle_min = -.5;
    msg.angle_max = .5;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / (num_beams - 1);
    msg.time_increment = 0.1 / num_beams;
    msg.scan_time = 0.1;
    msg.range_min = 0.5;
    msg.range_max = 1.5;
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

TEST(SpeckleFilter_Distance, SingleSpeckle) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1, 1, 1, 1, 0.5, 1, 1, 1, 1, 1, 1};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {1, 1, 1, 1, NAN, 1, 1, 1, 1, 1, 1};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, TwoDistantSpeckles) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1, 1, 1, 1, 0.5, 1, 1, 1, 0.5, 1, 1};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {1, 1, 1, 1, NAN, 1, 1, 1, NAN, 1, 1};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, TwoNearSpeckles) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1, 1, 1, 1, 0.5, 1, 0.5, 1, 1, 1, 1};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {1, 1, 1, 1, NAN, NAN, NAN, 1, 1, 1, 1};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, TwoSpecklesAtEdge) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1.2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1.2};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {NAN, 1, 1, 1, 1, 1, 1, 1, 1, 1, NAN};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, SinglePeak) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1, 1, 1, 1, 3, 1, 1, 1, 1, 1, 1};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {1, 1, 1, 1, 3, 1, 1, 1, 1, 1, 1};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, TwoFarPeaks) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1, 1, 1, 1, 3, 1, 1, 1, 3, 1, 1};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {1, 1, 1, 1, 3, 1, 1, 1, 3, 1, 1};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, TwoNearPeaks) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1, 1, 1, 1, 3, 1, 3, 1, 1, 1, 1};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {1, 1, 1, 1, 3, NAN, 3, 1, 1, 1, 1};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, TwoPeaksAtEdge) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, MultiplePlateausNoSpeckles) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 2;

    filter.configure(config);

    float ranges[] = {1, 1, 1.2, 1.2, 1.2, 0.7, 0.7, 0.9, 0.9, 0.9, 0.9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {1, 1, 1.2, 1.2, 1.2, 0.7, 0.7, 0.9, 0.9, 0.9, 0.9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

TEST(SpeckleFilter_Distance, MultiplePlateausBiggerWindow) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 3;

    filter.configure(config);

    float ranges[] = {1, 1, 1.2, 1.2, 1.2, 0.7, 0.7, 0.9, 0.9, 0.9, 0.9};
    sensor_msgs::LaserScan input_scan = create_message(
        ranges, sizeof(ranges) / sizeof(float)
    );
    sensor_msgs::LaserScan output_scan;

    filter.update(input_scan, output_scan);

    float expected[] = {NAN, NAN, 1.2, 1.2, 1.2, NAN, NAN, 0.9, 0.9, 0.9, 0.9};

    expect_ranges_equal(output_scan.ranges, expected, sizeof(expected) / sizeof(float));
}

#ifdef ENABLE_PERFORMANCE
TEST(SpeckleFilter_Distance, Performance) {
    laser_filters::LaserScanSpeckleFilter filter;
    laser_filters::SpeckleFilterConfig config;

    config.filter_type = laser_filters::SpeckleFilter_Distance;
    config.max_range = 2.0;
    config.max_range_difference = 0.1;
    config.filter_window = 8;

    filter.configure(config);

    float ranges[] = {};
    sensor_msgs::LaserScan input_scan[2];
    input_scan[0] = create_message(ranges, 0);
    int num_samples = 1024;
    int next_index = 0;
    int pending = 0;
    float range_val = 0;
    while (input_scan[0].ranges.size() < num_samples) {
        if (pending == 0) {
            range_val = 0.5 + next_index / 10.0f;
            pending = next_index + 1;

            if (next_index == 10) {
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
    ros::init(argc, argv, "test_speckle_filter");
    ros::Time::init();
    return RUN_ALL_TESTS();
}
