//
// Created by felix
//

#ifndef TUW_LASER_SCAN_CONTOUR_H
#define TUW_LASER_SCAN_CONTOUR_H

#include <sensor_msgs/LaserScan.h>
#include <opencv2/core.hpp>

namespace tuw
{
  class LaserScanContour
  {
  public:
    using Ptr = std::shared_ptr<LaserScanContour>;

    LaserScanContour(const sensor_msgs::LaserScan &scan, double scale_factor, double add_factor);

    std::vector<cv::Point2f> &getContourRepresentation()
    {
      return contour_;
    };

  private:
    void compute(const sensor_msgs::LaserScan &scan);

    std::vector<cv::Point2f> contour_ = {};
    double scale_factor_ = 100.0; //float scale_factor = 100.0; Values from mrpt_object_converter
    int add_factor_ = 500; //int add_factor = 500.0;
  };
}

#endif //TUW_LASERSCANCONTOUR_H
