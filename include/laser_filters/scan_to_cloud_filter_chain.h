#ifndef SCAN_TO_CLOUD_FILTER_CHAIN_H
#define SCAN_TO_CLOUD_FILTER_CHAIN_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <float.h>

// Laser projection
#include <laser_geometry/laser_geometry.h>

// TF
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//Filters
#include "filters/filter_chain.h"

//Nodelet
#include <nodelet/nodelet.h>

namespace laser_filters {
class ScanToCloudFilterChain : public nodelet::Nodelet
{
public:
    virtual void onInit();
    ScanToCloudFilterChain();
    ~ScanToCloudFilterChain();

    void deprecation_warn(const ros::TimerEvent& e);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr&scan_msg);

private:
    // ROS related
    laser_geometry::LaserProjection projector_; // Used to project laser scans

    double laser_max_range_;           // Used in laser scan projection
    int window_;

    bool high_fidelity_;                    // High fidelity (interpolating time across scan)
    std::string target_frame_;                   // Target frame for high fidelity result
    std::string scan_topic_, cloud_topic_;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    // TF
    tf::TransformListener tf_;

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> *filter_;

    double tf_tolerance_;
    filters::FilterChain<sensor_msgs::PointCloud2> *cloud_filter_chain_;
    filters::FilterChain<sensor_msgs::LaserScan> *scan_filter_chain_;
    ros::Publisher cloud_pub_;

    // Timer for displaying deprecation warnings
    ros::Timer deprecation_timer_;
    bool  using_scan_topic_deprecated_;
    bool  using_cloud_topic_deprecated_;
    bool  using_default_target_frame_deprecated_;
    bool  using_laser_max_range_deprecated_;
    bool  using_filter_window_deprecated_;
    bool  using_scan_filters_deprecated_;
    bool  using_cloud_filters_deprecated_;
    bool  using_scan_filters_wrong_deprecated_;
    bool  using_cloud_filters_wrong_deprecated_;
    bool  incident_angle_correction_;

};
}

//

#endif // SCAN_TO_CLOUD_FILTER_CHAIN_H
