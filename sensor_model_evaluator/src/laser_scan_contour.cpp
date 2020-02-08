//
// Created by felix
//

#include "laser_scan_contour.h"
#include <opencv2/imgproc.hpp>

using namespace tuw;

LaserScanContour::LaserScanContour(const sensor_msgs::LaserScan &laser_msg, double scale_factor, double add_factor) : scale_factor_(scale_factor), add_factor_(add_factor)
{
 compute(laser_msg);
}

void LaserScanContour::compute(const sensor_msgs::LaserScan &laser_msg)
{
  size_t nr = laser_msg.ranges.size();
  contour_.clear();
  contour_.resize( nr + 1 );
  size_t i;
  unsigned int contour_cnt;

  for ( i = 0, contour_cnt = 0; i < nr; i++ )
  {
    double length = laser_msg.ranges[i];

    if ((length < laser_msg.range_max) && isfinite(length ))
    {
      double angle = laser_msg.angle_min + (laser_msg.angle_increment * i);
      cv::Point2f p;
      p.x = cos( angle ) * length;
      p.y = sin( angle ) * length;
      contour_[contour_cnt] = p * scale_factor_ + cv::Point2f( add_factor_, add_factor_ );
      contour_cnt++;
    }

  }
  if ( contour_cnt < contour_.size())
  {
    contour_.resize( contour_cnt );
  }

  std::vector<cv::Point2f> hull;
  cv::convexHull( contour_, hull, false );
  contour_ = hull;
}
