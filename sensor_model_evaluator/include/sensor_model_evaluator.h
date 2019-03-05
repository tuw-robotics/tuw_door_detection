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
#include <string>

namespace tuw
{
  
  class SensorModelEvaluator
  {
  public:
    
    SensorModelEvaluator( const nav_msgs::OccupancyGridConstPtr &map );
    
    void evaluate( LaserMeasurementPtr &scan );
  
  private:
    using measurement_table = std::map<unsigned int, Point2D>;
    using map_info_type = nav_msgs::OccupancyGrid_<std::allocator<void>>::_info_type;
    
    struct InternalMap
    {
    public:
      InternalMap() = default;
      
      ~InternalMap() = default;
      
      map_info_type map_info_;
      int size_x;
      int size_y;
      double scale;
      double origin_x;
      double origin_y;
      cv::Mat cv_ui8;
      
      // Convert from world coords to map coords
      double get_mx_from_wx( double x )
      {
        return (floor((x - origin_x) / scale + 0.5 ) + size_x / 2);
      }
      
      double get_my_from_wy( double y )
      {
        return (floor((y - origin_y) / scale + 0.5 ) + size_y / 2);
      }
      
      // Convert from map index to world coords
      double get_wx_from_mx( double x )
      {
        return (origin_x + ((x) - size_x / 2) * scale);
      }
      
      double get_wy_from_my( double y )
      {
        return (origin_y + ((y) - size_y / 2) * scale);
      }
      
      std::string to_string()
      {
        std::stringstream sstr( "" );
        sstr << "(sx, sy): \t" << ("( " + std::to_string( size_x ) + ", " + std::to_string( size_y ) + ")\n" +
                                   "(ox, oy): \t" + "(" + std::to_string( origin_x ) + ", " +
                                   std::to_string( origin_y ) +
                                   ")\n" +
                                   "scale: \t" + std::to_string( scale ) + "\n");
        return sstr.str();
      }
    };
    
    bool convert( const nav_msgs::OccupancyGridConstPtr &src, std::unique_ptr<InternalMap> &map );
    
    Point2DPtr rayTrace( const double scale, const Beam &b, const Eigen::Matrix4d &tf_ML );
    
    void updateExpectedMeasurementTable( const Beam &b, const Point2DPtr &intersection );
    
    void updateObservedMeasurementTable( const Point2DPtr &obs );
    
    void downscaleImshow( const std::unique_ptr<InternalMap> &map, LaserMeasurementPtr meas = nullptr );
    
    void clear();
    
    measurement_table expected_meas_;
    measurement_table observed_meas_;
    
    std::unique_ptr<InternalMap> map_;
    nav_msgs::OccupancyGrid map_msg_;
  };
  
  using SensorModelEvaluatorPtr = std::shared_ptr<SensorModelEvaluator>;
  
}

#endif //PROJECT_SENSORMODELEVALUATOR_H
