/*
 * Copyright (c) 2013 Kei Okada
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/*
  \author Kei OKada


*/

#ifndef LASER_SCAN_BLOB_FILTER_H
#define LASER_SCAN_BLOB_FILTER_H

#include <set>

#include "filters/filter_base.h"
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"

namespace laser_filters{

/** @b ScanBlobFilter is a simple filter that filters shadow points in a laser scan line 
 */

class ScanBlobFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  double max_radius_;          // Filter angle threshold
  int min_points_;
    

  ////////////////////////////////////////////////////////////////////////////////
  ScanBlobFilter () 
  {


  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    max_radius_ = 0.1;//default value
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("max_radius"), max_radius_))
    {
      ROS_ERROR("Error: BlobFilter was not given min_radius.\n");
      return false;
    }

    min_points_ = 5;//default value
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("min_points"), min_points_))
    {
      ROS_INFO("Error: BlobFilter was not given min_points.\n");
      return false;
    }
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  virtual ~ScanBlobFilter () { }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief 
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
  {
    //copy across all data first
    scan_out = scan_in;

    std::set<int> indices_to_publish;
    // assume that all points is pass thorugh shadow filter, so each blob is separeted by invalide scan data
    std::vector<std::vector<int> > range_blobs;
    std::vector<int> range_blob;
    for (unsigned int i = 0; i < scan_in.ranges.size (); i++)
    {
      scan_out.ranges[i] = -1.0 * fabs(scan_in.ranges[i]); // set all ranges to invalid (*)
      if ( scan_in.ranges[i] < 0 || std::isnan(scan_in.ranges[i])) {
          if ( range_blob.size() > min_points_ ) {
              range_blobs.push_back(range_blob);
          }
          range_blob.clear();
      }else{
          range_blob.push_back(i);
      }
    }
    if ( range_blob.size() > min_points_ ) {
        range_blobs.push_back(range_blob);
    }
    // for each blob calculate center and radius
    for (unsigned int i = 0; i < range_blobs.size(); i++) {
        int size = range_blobs[i].size();
        // check center of blob
        double center_x = 0, center_y = 0;
        for (unsigned int j = 0; j < size; j++) {
            double x = scan_in.ranges[range_blobs[i][j]];
            double y = scan_in.ranges[range_blobs[i][j]] * scan_in.angle_increment;
            center_x += x;
            center_y += y;
        }
        center_x /= size;
        center_y /= size;

        // check range of blob
        double radius = 0;
        for (unsigned int j = 0; j < size; j++) {
            double x = scan_in.ranges[range_blobs[i][j]];
            double y = scan_in.ranges[range_blobs[i][j]] * scan_in.angle_increment;
            if ( radius < fabs(center_x - x) ) radius = fabs(center_x - x) ;
            if ( radius < fabs(center_y - y) ) radius = fabs(center_y - y) ;
        }

        ROS_DEBUG_STREAM("blob center " << center_x << " " << center_y << ", radius " << radius << ", num of ponits " << size);
        if ( radius < max_radius_ ) {
            indices_to_publish.insert(range_blobs[i][0] + size/2);
        }
    }
    ROS_DEBUG("ScanBlobFilter  %d Points from scan with min radius: %.2f, num of pints: %d", (int)indices_to_publish.size(), max_radius_, min_points_);
    for ( std::set<int>::iterator it = indices_to_publish.begin(); it != indices_to_publish.end(); ++it)
      {
	scan_out.ranges[*it] = fabs(scan_in.ranges[*it]); // valid only the ranges that passwd the test (*)
      }
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////

} ;
}

#endif //LASER_SCAN_BLOB_FILTER_H
