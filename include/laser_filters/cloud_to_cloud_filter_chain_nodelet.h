#ifndef CLOUD_TO_CLOUD_FILTER_CHAIN_H
#define CLOUD_TO_CLOUD_FILTER_CHAIN_H


#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"
#include <nodelet/nodelet.h>

namespace laser_filters {

class CloudToCloudFilterChain : public nodelet::Nodelet
{


    public:
        virtual void onInit();
        CloudToCloudFilterChain();
        ~CloudToCloudFilterChain();

        void deprecation_warn(const ros::TimerEvent& e);
        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in);

    private:
        // Our NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // Components for tf::MessageFilter
        tf::TransformListener *tf_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub_;
        tf::MessageFilter<sensor_msgs::PointCloud2> *tf_filter_;
        double tf_filter_tolerance_;

        // Filter Chain
        filters::FilterChain<sensor_msgs::PointCloud2> *filter_chain_;

        // Components for publishing
        sensor_msgs::PointCloud2 msg_;
        ros::Publisher output_pub_;

        // Deprecation helpers
        ros::Timer deprecation_timer_;
        bool  using_filter_chain_deprecated_;

};
}

#endif // SCAN_TO_SCAN_FILTER_CHAIN_H
