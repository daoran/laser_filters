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
\author Yannic Bachmann
*/

#ifndef LASER_SCAN_MEDIAN_SPATIAL_FILTER_H
#define LASER_SCAN_MEDIAN_SPATIAL_FILTER_H

#include "filters/filter_base.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <vector>
#include <stdexcept>

namespace laser_filters
{

class LaserScanMedianSpatialFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  int window_size_;

  bool configure()
  {
    // Default window size
    window_size_ = 3;
    getParam("window_size", window_size_);

    // Ensure window size is positive
    if (window_size_ <= 0)
    {
      throw std::runtime_error("Window size must be positive");
    }

    // Ensure window size is odd
    if (window_size_ % 2 == 0)
    {
      window_size_ += 1;
    }

    return true;
  }

  virtual ~LaserScanMedianSpatialFilter() {}

  bool update(const sensor_msgs::msg::LaserScan &input_scan, sensor_msgs::msg::LaserScan &filtered_scan)
  {
    filtered_scan = input_scan;

    int half_window = window_size_ / 2;
    std::vector<float> window;

    for (size_t i = 0; i < input_scan.ranges.size(); ++i)
    {
      window.clear();

      // Collect points within the window
      for (int j = -half_window; j <= half_window; ++j)
      {
        int index = i + j;

        if (index >= 0 && index < input_scan.ranges.size())
        {
          if (!std::isnan(input_scan.ranges[index]))
          {
            window.push_back(input_scan.ranges[index]);
          }
        }
      }

      if (!window.empty())
      {
        // Calculate median
        std::sort(window.begin(), window.end());
        filtered_scan.ranges[i] = window[window.size() / 2];
      }
      else
      {
        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    return true;
  }
};

} // namespace laser_filters

#endif // LASER_SCAN_MEDIAN_SPATIAL_FILTER_H
