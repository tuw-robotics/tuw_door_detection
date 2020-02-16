//
// Created by felix
//

#include "laser_scan_contour.h"
#include <opencv2/imgproc.hpp>

using namespace tuw;

LaserScanContour::LaserScanContour(const sensor_msgs::LaserScan &laser_msg, double scale_factor, double add_factor)
  : scale_factor_(scale_factor), add_factor_(add_factor)
{
  compute(laser_msg);
}

void LaserScanContour::compute(const sensor_msgs::LaserScan &laser_msg)
{
  size_t nr = laser_msg.ranges.size();
  contour_.clear();
  contour_.resize(nr + 1);
  size_t i;
  unsigned int contour_cnt;

  for (i = 0, contour_cnt = 0; i < nr; i++)
  {
    double length = laser_msg.ranges[i];

    if ((length < laser_msg.range_max) && isfinite(length))
    {
      double angle = laser_msg.angle_min + (laser_msg.angle_increment * i);
      cv::Point2f p;
      p.x = cos(angle) * length;
      p.y = sin(angle) * length;
      contour_[contour_cnt] = p * scale_factor_ + cv::Point2f(add_factor_, add_factor_);
      contour_cnt++;
    }

  }
  if (contour_cnt < contour_.size())
  {
    contour_.resize(contour_cnt);
  }

  std::vector<cv::Point2f> hull;
  cv::convexHull(contour_, hull, false);
  contour_ = hull;
}

void LaserScanContour::print()
{
  std::cout << "print contour " << std::endl;
  for (auto c : contour_)
  {
    std::cout << "( " << c.x << ", " << c.y << ")" << std::endl;
  }
}

cv::Mat LaserScanContour::showAsImage()
{
  auto min_max_x = std::make_pair(std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
  auto min_max_y = std::make_pair(std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
  for (auto c : contour_)
  {
    min_max_x.first = std::min(min_max_x.first,c.x);
    min_max_x.second = std::max(min_max_x.second,c.x);
    min_max_y.first = std::min(min_max_y.first,c.y);
    min_max_y.second = std::max(min_max_y.second,c.y);
  }

  cv::Point2f min_cv = cv::Point2f(min_max_x.first, min_max_y.first);

  float width = std::ceil(min_max_x.second - min_max_x.first);
  float height = std::ceil(min_max_y.second - min_max_y.first);

  std::cout << "showAsImage()" << std::endl;
  std::cout << "Dims (w,h):" << width  << ", " << height << std::endl;
  std::cout << "Contour size: " << contour_.size() << std::endl;

  cv::Mat image = cv::Mat::zeros(height,width,CV_8UC3);
  for (int i=0; i < (contour_.size()-1); ++i)
  {
    cv::Point2f augmented_p0 = contour_[i] - min_cv;
    cv::Point2f augmented_p1 = contour_[i+1] - min_cv;
    cv::line(image, augmented_p0, augmented_p1, cv::Scalar(255,0,0));
  }
  std::cout << "img rc " << image.rows << ", " << image.cols << std::endl;
  return image;
}
