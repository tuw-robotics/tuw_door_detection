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

    double scale_factor()
    {
      return scale_factor_;
    }

    double contour_offset()
    {
      return contour_offset_;
    }

    int add_factor()
    {
      return add_factor_;
    }

    //For visualizing/debugging
    cv::Mat showAsImage();

    void print();

  private:
    void compute(const sensor_msgs::LaserScan &scan);

    std::vector<cv::Point2f> contour_ = {};
    //For internal computation and visualization
    double scale_factor_ = 100.0; //float scale_factor = 100.0; Values from mrpt_object_converter
    int add_factor_ = 500; //int add_factor = 500.0;

    //Tolerance offset of the convex hull endpoints
    double contour_offset_ = 0.25;
  };
}

#endif //TUW_LASERSCANCONTOUR_H
