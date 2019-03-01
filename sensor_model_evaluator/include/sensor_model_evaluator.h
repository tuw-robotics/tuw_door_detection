//
// Created by felix on 28.02.19.
//

#ifndef TUW_SENSORMODELEVALUATOR_H
#define TUW_SENSORMODELEVALUATOR_H

#include <ros/ros.h>
#include <memory>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_measurement_utils/contour.h>
#include <tuw_measurement_utils/laser_measurement.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

namespace tuw
{
  
  class SensorModelEvaluator
  {
  public:
    SensorModelEvaluator( const nav_msgs::OccupancyGridConstPtr &map );
    
    void evaluate( LaserMeasurementPtr &scan );
    
    bool convert( const nav_msgs::OccupancyGridConstPtr &src, cv::Mat &des );
  
  private:
    using measurement_table = std::map<unsigned int, Point2D>;
    using map_info_type = nav_msgs::OccupancyGrid_<std::allocator<void>>::_info_type;
    
    Point2DPtr rayTrace( const double scale, const Beam &b, const Eigen::Matrix4d &tf_ML );
    
    void updateExpectedMeasurementTable( const Beam &b, const Point2DPtr &intersection );
    
    void updateObservedMeasurementTable( const Point2DPtr &obs );
    
    void downscaleImshow( const cv::Mat &des );
    
    void clear();
    
    measurement_table expected_meas_;
    measurement_table observed_meas_;
    map_info_type map_info_;
    
    cv::Mat map_;
  };
  
  using SensorModelEvaluatorPtr = std::shared_ptr<SensorModelEvaluator>;
  
}

#endif //PROJECT_SENSORMODELEVALUATOR_H
