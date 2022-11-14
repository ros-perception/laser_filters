/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  box_filter.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "laser_filters/box_filter.h"
#include <ros/ros.h>

laser_filters::LaserScanBoxFilter::LaserScanBoxFilter(){

}


bool laser_filters::LaserScanBoxFilter::configure(){
  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<BoxFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<BoxFilterConfig>::CallbackType f;
  f = [this](auto& config, auto level){ reconfigureCB(config, level); };
  dyn_server_->setCallback(f);

  up_and_running_ = true;

  getParam("box_frame", config_.box_frame);
  getParam("max_x", config_.max_x);
  getParam("max_y", config_.max_y);
  getParam("max_z", config_.max_z);
  getParam("min_x", config_.min_x);
  getParam("min_y", config_.min_y);
  getParam("min_z", config_.min_z);
  getParam("invert", config_.invert);
  dyn_server_->updateConfig(config_);
  
  ROS_INFO("BOX filter started");
  ROS_INFO("Box frame is: %s", config_.box_frame.c_str());
  ROS_INFO("Box: x_min %f, x_max, %f, y_min, %f, y_max, %f, min_z %f, max_z %f", config_.min_x,
   config_.max_x, config_.min_y, config_.max_y, config_.min_z, config_.max_z);
  ROS_INFO("Box filter invert: %d", config_.invert);
  return true;
}

bool laser_filters::LaserScanBoxFilter::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  output_scan = input_scan;
  sensor_msgs::PointCloud2 laser_cloud;
  
  std::string error_msg;

  bool success = tf_.waitForTransform(
    config_.box_frame,
    input_scan.header.frame_id,
    input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size()*input_scan.time_increment),
    ros::Duration(1.0),
    ros::Duration(0.01),
    &error_msg
  );
  if(!success){
    ROS_WARN("Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
    return false;
  }

  try{
    projector_.transformLaserScanToPointCloud(config_.box_frame, input_scan, laser_cloud, tf_);
  }
  catch(tf::TransformException& ex){
    if(up_and_running_){
      ROS_WARN_THROTTLE(1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    }
    else
    {
      ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if(i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1){
      ROS_INFO_THROTTLE(.3, "x, y, z and index fields are required, skipping scan");

  }


  const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
  const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
  const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
  const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

  const int pstep = laser_cloud.point_step;
  const long int pcount = laser_cloud.width * laser_cloud.height;
  const long int limit = pstep * pcount;

  int i_idx, x_idx, y_idx, z_idx;  
  for(
    i_idx = i_idx_offset,
    x_idx = x_idx_offset,
    y_idx = y_idx_offset,
    z_idx = z_idx_offset;

    x_idx < limit;

    i_idx += pstep,
    x_idx += pstep,
    y_idx += pstep,
    z_idx += pstep)
  {

    // TODO works only for float data types and with an index field
    // I'm working on it, see https://github.com/ros/common_msgs/pull/78 
    float x = *((float*)(&laser_cloud.data[x_idx]));
    float y = *((float*)(&laser_cloud.data[y_idx]));
    float z = *((float*)(&laser_cloud.data[z_idx]));
    int index = *((int*)(&laser_cloud.data[i_idx]));

    tf::Point point(x, y, z);

    if(!config_.invert){
      if(inBox(point)){
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    else{
      if(!inBox(point)){
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }

  }
  up_and_running_ = true;
  return true;
}

bool laser_filters::LaserScanBoxFilter::inBox(tf::Point &point){
  return
    point.x() < config_.max_x && point.x() > config_.min_x &&
    point.y() < config_.max_y && point.y() > config_.min_y &&
    point.z() < config_.max_z && point.z() > config_.min_z;
}

void laser_filters::LaserScanBoxFilter::reconfigureCB(BoxFilterConfig& config, uint32_t level)
{
  config_ = config;
}

