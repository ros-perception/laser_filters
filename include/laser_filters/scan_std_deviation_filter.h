#ifndef SCAN_STD_DEVIATION_FILTER_H
#define SCAN_STD_DEVIATION_FILTER_H

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace laser_filters {

class LaserScanStdDeviationFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
public:
    int window_size_;
    double basic_std_dev_threshold_;

    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr;


    bool configure() {
        ros::NodeHandle nh_("~/scan_std_deviation_filter");

        ddr = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);

        getParam("window_size", window_size_);
        getParam("basic_std_dev_threshold", basic_std_dev_threshold_);

        ddr->registerVariable<int>("window_size", &window_size_, "Filter window size", 0, 16);
        ddr->registerVariable<double>("basic_std_dev_threshold", &basic_std_dev_threshold_, "Basic std deviation threshold for max range of LIDAR", 0, 3);
        ddr->publishServicesTopics();

        return true;
    }

    virtual ~LaserScanStdDeviationFilter() {}

    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {

        filtered_scan = input_scan;

        for(unsigned int i=0; i < input_scan.ranges.size(); i=i+window_size_) {
            double current_max_range=0;
            // Going through the window to calculate the mean of the data set
            double sum = 0;
            for(unsigned int j=i; j < i+window_size_; j++) {
                if(input_scan.ranges[j] > current_max_range) {
                    current_max_range = input_scan.ranges[j];
                }
                sum = sum + input_scan.ranges[j];
            }
            double mean = sum / window_size_;

            // Calculating variance
            double var_sum = 0;
            for(unsigned int k=i; k < i+window_size_; k++) {
                var_sum = var_sum + pow((input_scan.ranges[k] - mean), 2);
            }

            double std_dev = sqrt(var_sum / (window_size_ - 1));
            double percent_max_dist = (current_max_range/input_scan.range_max) * 100;
            double deviation_threshold = percent_max_dist * basic_std_dev_threshold_;

            //Filtering according to a threshold based on the max distance of the scan
            if(std_dev > deviation_threshold) {
                for(unsigned int h=i; h < i+window_size_; h++) {
                    filtered_scan.ranges[h] = input_scan.range_max + 1;
                }
            }
        }

        return true;
    }

};
}

#endif // SCAN_STD_DEVIATION_FILTER_H
