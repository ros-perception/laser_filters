/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017, laser_filters authors
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

#ifndef SCAN_SHADOW_DETECTOR_H
#define SCAN_SHADOW_DETECTOR_H

#include <vector>

namespace laser_filters
{
class ScanShadowDetector
{
public:
  float min_angle_tan_, max_angle_tan_;  // Filter angle thresholds

  void configure(const float min_angle, const float max_angle);

  /** \brief Check if the point is a shadow of another point within one laser scan.
   * \param r1 the distance to the first point
   * \param r2 the distance to the second point
   * \param included_angle the angle between laser scans for these two points
   */
  bool isShadow(const float r1, const float r2, const float included_angle);

  /** \brief Check if the point is a shadow of another point within one laser scan.
   * Use this method instead of the version without the extra parameters to avoid
   * computing sin and cos of the angle on every execution.
   * \param r1 the distance to the first point
   * \param r2 the distance to the second point
   * \param included_angle_sin the sine of an angle between laser scans for these two points
   * \param included_angle_cos the cosine of an angle between laser scans for these two points
   */
  bool isShadow(float r1, float r2, float included_angle_sin, float included_angle_cos);
};
}

#endif  //SCAN_SHADOW_DETECTOR_H
