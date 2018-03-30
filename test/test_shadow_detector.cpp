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

#include <gtest/gtest.h>
#include <angles/angles.h>

#include "laser_filters/scan_shadow_detector.h"

double getAngleWithViewpoint(const float r1, const float r2, const float included_angle)
{
  return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
}

bool isShadowPureImpl(const float r1, const float r2, const float included_angle, const double min_angle, const double max_angle)
{
  const double angle = fabs(angles::to_degrees(
      getAngleWithViewpoint(r1, r2, included_angle)));
  if (angle < min_angle || angle > max_angle)
    return true;
  return false;
}

TEST(ScanShadowDetector, ShadowDetectionGeometry)
{
  for (float min_angle = 90.0; min_angle >= 0.0; min_angle -= 5.0)
  {
    for (float max_angle = 90.0; max_angle <= 180; max_angle += 5.0)
    {
      laser_filters::ScanShadowDetector detector;
      detector.configure(angles::from_degrees(min_angle), angles::from_degrees(max_angle));

      for (float r1 = 0.1; r1 < 1.0; r1 += 0.1)
      {
        for (float r2 = 0.1; r2 < 1.0; r2 += 0.1)
        {
          for (float inc = 0.01; inc < 0.1; inc += 0.02)
          {
            // Compare with original ScanShadowsFilter implementation
            EXPECT_EQ(
                detector.isShadow(r1, r2, inc),
                isShadowPureImpl(r1, r2, inc, min_angle, max_angle));
          }
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
