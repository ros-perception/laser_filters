#ifndef SCAN_MEDIAN_SHADOW_FILTER_H
#define SCAN_MEDIAN_SHADOW_FILTER_H

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace laser_filters {

class LaserScanMeanShadowFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
public:
    unsigned int window_size_;
    double percent_max_dist_;
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr;


    bool configure() {
        ros::NodeHandle nh_("~/scan_median_shadow_filter");
        ROS_INFO("Configuring filter");
//        ddr = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);

        getParam("window_size", window_size_);
        getParam("percent_max_dist", percent_max_dist_);

//        ddr->registerVariable<unsigned int>("Window size", &window_size_, "Filter window size", 0, 16);
//        ddr->registerVariable<double>("Percent max dist", &percent_max_dist_, "Percentage of the max distance in the window that will create the threshold", 0, 1);
//        ROS_INFO("Filter configured");
        return true;
    }

    virtual ~LaserScanMeanShadowFilter() {

    }

    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {
        double current_max_range;
        double absolute_mean_difference;

        // Rebuild header of output scan
        filtered_scan = input_scan;

        // Going through the vector
        for(unsigned int i=0; i < input_scan.ranges.size(); i=i+window_size_) {
            current_max_range = 0;
            // Going through the window
            for(unsigned int j=i; j < i+window_size_ - 1; j++) {
                if(input_scan.ranges[j] > current_max_range) {
                    current_max_range = input_scan.ranges[j];
                }
                absolute_mean_difference = abs(input_scan.ranges[j+1] - input_scan.ranges[j]);
            }
            absolute_mean_difference = absolute_mean_difference/window_size_;
            double delta_treshold = current_max_range * percent_max_dist_;

            if(absolute_mean_difference < delta_treshold) {
                for(unsigned int j=i; j < i+window_size_; j++) {
                    filtered_scan.ranges[j] = input_scan.ranges[j];
                }
            } else {
                for(unsigned int j=i; j < i+window_size_; j++) {
                    filtered_scan.ranges[j] = input_scan.range_max+1;
                }
            }
        }
        return true;
    }

};
}

#endif // SCAN_MEDIAN_SHADOW_FILTER_H
