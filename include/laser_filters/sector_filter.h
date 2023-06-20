/*********************************************************************
* BSD 2-Clause License
*
* Copyright (c) 2021, Jimmy F. Klarke
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author: Jimmy F. Klarke
*********************************************************************/

#ifndef LASER_SCAN_SECTOR_FILTER_IN_PLACE_H
#define LASER_SCAN_SECTOR_FILTER_IN_PLACE_H

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <laser_filters/SectorFilterConfig.h>

#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> 

namespace laser_filters
{

class LaserScanSectorFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  LaserScanSectorFilter();
  bool configure();
  bool isClearInside();
  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& output_scan);

  virtual ~LaserScanSectorFilter(){}

private:
  ddynamic_reconfigure::DDynamicReconfigure dyn_server_;
  void reconfigureCB();
  boost::recursive_mutex own_mutex_;

  SectorFilterConfig config_ = SectorFilterConfig::__getDefault__();
};

} // end namespace laser_filters

#endif // LASER_SCAN_SECTOR_FILTER_IN_PLACE_H
