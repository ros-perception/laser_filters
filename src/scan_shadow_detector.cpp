/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2021, laser_filters authors
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
\author Atsushi Watanabe (SEQSENSE, Inc.)
*/

#include <laser_filters/scan_shadow_detector.h>
#include <math.h>
#include <ros/ros.h>

namespace laser_filters
{
  void ScanShadowDetector::configure(const float min_angle, const float max_angle)
  {
    min_angle_tan_ = tanf(min_angle);
    max_angle_tan_ = tanf(max_angle);

    // Correct sign of tan around singularity points
    if (min_angle_tan_ < 0.0)
      min_angle_tan_ = -min_angle_tan_;
    if (max_angle_tan_ > 0.0)
      max_angle_tan_ = -max_angle_tan_;
  }

  bool ScanShadowDetector::isShadow(const float r1, const float r2, const float included_angle)
  {
    float included_angle_sin = sinf(included_angle);
    float included_angle_cos = cosf(included_angle);
    return isShadow(r1, r2, included_angle_sin, included_angle_cos);
  }

  bool ScanShadowDetector::isShadow(float r1, float r2, float included_angle_sin, float included_angle_cos)
  {
    const float perpendicular_y_ = r2 * included_angle_sin;
    const float perpendicular_x_ = r1 - r2 * included_angle_cos;
    const float perpendicular_tan_ = fabs(perpendicular_y_) / perpendicular_x_;

    if (perpendicular_tan_ > 0) {
      if (perpendicular_tan_ < min_angle_tan_)
        return true;
    }
    else {
      if (perpendicular_tan_ > max_angle_tan_)
        return true;
    }

    return false;
  }
}
