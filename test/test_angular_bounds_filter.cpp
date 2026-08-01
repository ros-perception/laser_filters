#include <gtest/gtest.h>
#include <laser_filters/angular_bounds_filter_in_place.h>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pluginlib/class_loader.hpp>

using sensor_msgs::msg::LaserScan;

static LaserScan makeScanMsg(
    float angle_min,
    float angle_increment,
    int num_beams,
    float range_value = 1.0f,
    float intensity_value = 5.0f)
{
    LaserScan msg;
    msg.header.frame_id = "laser";
    msg.angle_min = angle_min;
    msg.angle_increment = angle_increment;
    msg.angle_max = angle_min + angle_increment * (num_beams - 1);
    msg.time_increment = 0.1;
    msg.scan_time = 0.1;
    msg.range_min = 0.5;
    msg.range_max = 10.0;

    msg.ranges.assign(num_beams, range_value);
    msg.intensities.assign(num_beams, intensity_value);
    return msg;
}

static void checkRangeVectors(const std::vector<float> &actual, const std::vector<float> &expected)
{
    ASSERT_EQ(actual.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
    {
        if(std::isnan(expected[i]))
        {
            EXPECT_TRUE(std::isnan(actual[i])) << "index " << i;
        }
        else
        {
            EXPECT_NEAR(actual[i], expected[i], 1e-6) << "index " << i;
        }
    }
}

TEST(AngularBoundsFilterInPlace, NonWrappedRange)
{
    laser_filters::LaserScanAngularBoundsFilterInPlace filter;

    filter.lower_angle_ = -0.15;
    filter.upper_angle_ = 0.15;
    filter.replace_with_nan_ = true;
    filter.wrap_angle_ = true;

    LaserScan input = makeScanMsg(-0.5f, 0.1f, 10);
    LaserScan output;

    ASSERT_TRUE(filter.update(input, output));

    std::vector<float> expected = {
        1.0f, 1.0f, 1.0f, 1.0f,
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        1.0f, 1.0f, 1.0f
    };

    checkRangeVectors(output.ranges, expected);

    EXPECT_FLOAT_EQ(output.intensities[4], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[5], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[6], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[0], 5.0f);

}

TEST(AngularBoundsFilterInPlace, WrappedRange)
{
    laser_filters::LaserScanAngularBoundsFilterInPlace filter;

    filter.lower_angle_ = 2.9;
    filter.upper_angle_ = -2.9;
    filter.replace_with_nan_ = true;
    filter.wrap_angle_ = true;

    LaserScan input = makeScanMsg(-3.0f, 0.5f, 13);
    LaserScan output;

    ASSERT_TRUE(filter.update(input, output));

    std::vector<float> expected = {
        std::numeric_limits<float>::quiet_NaN(),
        1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f,
        std::numeric_limits<float>::quiet_NaN()
    };

    checkRangeVectors(output.ranges, expected);

    EXPECT_FLOAT_EQ(output.intensities.front(), 0.0f);
    EXPECT_FLOAT_EQ(output.intensities.back(), 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[1], 5.0f);
}

TEST(AngularBoundsFilterInPlace, NonWrappedRangeReplaceWithMax)
{
    laser_filters::LaserScanAngularBoundsFilterInPlace filter;

    filter.lower_angle_ = -0.15;
    filter.upper_angle_ = 0.15;
    filter.replace_with_nan_ = false;
    filter.wrap_angle_ = true;

    LaserScan input = makeScanMsg(-0.5f, 0.1f, 10);
    LaserScan output;

    ASSERT_TRUE(filter.update(input, output));

    std::vector<float> expected = {
        1.0f, 1.0f, 1.0f, 1.0f,
        input.range_max + 1.0f,
        input.range_max + 1.0f,
        input.range_max + 1.0f,
        1.0f, 1.0f, 1.0f
    };

    checkRangeVectors(output.ranges, expected);

    EXPECT_FLOAT_EQ(output.intensities[4], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[5], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[6], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[0], 5.0f);
}

TEST(AngularBoundsFilterInPlace, WrapBoundaries)
{
    laser_filters::LaserScanAngularBoundsFilterInPlace filter;

    filter.lower_angle_ = -0.15;
    filter.upper_angle_ = 0.15;
    filter.replace_with_nan_ = false;
    filter.wrap_angle_ = true;

    LaserScan input = makeScanMsg(-0.5f, 0.1f, 10);
    LaserScan output;

    ASSERT_TRUE(filter.update(input, output));

    std::vector<float> expected = {
        1.0f, 1.0f, 1.0f, 1.0f,
        input.range_max + 1.0f,
        input.range_max + 1.0f,
        input.range_max + 1.0f,
        1.0f, 1.0f, 1.0f
    };

    checkRangeVectors(output.ranges, expected);

    EXPECT_FLOAT_EQ(output.intensities[4], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[5], 0.0f);
    EXPECT_FLOAT_EQ(output.intensities[6], 0.0f);

    EXPECT_FLOAT_EQ(output.intensities[3], 5.0f);
    EXPECT_FLOAT_EQ(output.intensities[7], 5.0f);
}

TEST(AngularBoundsFilterInPlace, WrapAngleParamBehavior)
{
    LaserScan input = makeScanMsg(-3.1415f, 0.1f, 63);
    const size_t idx = static_cast<size_t>(std::round((3.05f - (-3.1415f)) / 0.1f)); // should end up being around 63 beams

    // wrapped interval (3.0 .. -3.0) should filter the sample near +3.05
    {
        laser_filters::LaserScanAngularBoundsFilterInPlace filter;
        filter.lower_angle_ = 3.0;
        filter.upper_angle_ = -3.0;
        filter.replace_with_nan_ = true;
        filter.wrap_angle_ = true;

        LaserScan output;
        ASSERT_TRUE(filter.update(input, output));
        EXPECT_TRUE(std::isnan(output.ranges[idx]));
        EXPECT_FLOAT_EQ(output.intensities[idx], 0.0f);
    }

    // legacy (non-wrap) behavior; same numeric angles not filtered
    {
        laser_filters::LaserScanAngularBoundsFilterInPlace filter;
        filter.lower_angle_ = 3.0;
        filter.upper_angle_ = -3.0;
        filter.replace_with_nan_ = true;
        filter.wrap_angle_ = false;

        LaserScan output;
        ASSERT_TRUE(filter.update(input, output));
        EXPECT_FALSE(std::isnan(output.ranges[idx]));
        EXPECT_FLOAT_EQ(output.ranges[idx], 1.0f); // original range value from makeScanMsg
        EXPECT_FLOAT_EQ(output.intensities[idx], 5.0f);
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}