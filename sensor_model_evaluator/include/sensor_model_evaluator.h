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
#include <map>
#include <grid_map_msgs/GridMap.h>
#include <boost/filesystem.hpp>

#include <sensor_model_evaluator/SensorModelEvaluatorNodeConfig.h>


namespace tuw
{

  class SensorModelEvaluator
  {
  public:
    using measurement_table = std::map<double, Point2D>;
    using map_info_type = nav_msgs::OccupancyGrid_<std::allocator<void>>::_info_type;

    SensorModelEvaluator( const nav_msgs::OccupancyGridConstPtr &map, bool render = true );

    void evaluate( LaserMeasurementPtr &scan );

    void publish();

    template<typename M_DES>
    bool getMap( M_DES &des )
    {
      if ( has_result_ )
      {
        return convert(map_, des);
      }
      return false;
    }

    bool hasResult()
    {
      return has_result_;
    }

    void serializeResult( const std::string &filepath );

    void configure( const sensor_model_evaluator::SensorModelEvaluatorNodeConfig &cfg );

  private:

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
      cv::Mat cv_uc8;
      std::shared_ptr<InternalMap> parent;
      //For rendermap
      cv::Mat cv_untouched_initial;

      void clear()
      {
        cv_untouched_initial.copyTo(cv_uc8);
      }

      // Convert from world coords to map coords
      double get_mx_from_wx( double x )
      {
        return ( floor(( x - origin_x ) / scale + 0.5) + size_x / 2 );
      }

      double get_my_from_wy( double y )
      {
        return ( floor(( y - origin_y ) / scale + 0.5) + size_y / 2 );
      }

      // Convert from map index to world coords
      double get_wx_from_mx( double x )
      {
        return ( origin_x + (( x ) - size_x / 2 ) * scale );
      }

      double get_wy_from_my( double y )
      {
        return ( origin_y + (( y ) - size_y / 2 ) * scale );
      }

      std::string to_string()
      {
        std::stringstream sstr("");
        sstr << "(sx, sy): \t" << ( "( " + std::to_string(size_x) + ", " + std::to_string(size_y) + ")\n" +
                                    "(ox, oy): \t" + "(" + std::to_string(origin_x) + ", " +
                                    std::to_string(origin_y) +
                                    ")\n" +
                                    "scale: \t" + std::to_string(scale) + "\n" );
        return sstr.str();
      }
    };

    bool convert( const nav_msgs::OccupancyGridConstPtr &src, std::shared_ptr<InternalMap> &map );

    bool convert( const std::shared_ptr<InternalMap> &src, nav_msgs::OccupancyGrid &des );

    bool convert( const std::shared_ptr<InternalMap> &src, grid_map_msgs::GridMap &des );

    bool convert( const std::shared_ptr<InternalMap> &src, cv::Mat &mat );

    Point2DPtr rayTrace( const double scale, const Beam &b, const Eigen::Matrix4d &tf_ML );

    void updateExpectedMeasurementTable( double idx, const Point2D &expect );

    void updateObservedMeasurementTable( double idx, const Point2D &obs );

    void downscaleImshow( LaserMeasurementPtr meas = nullptr );

    std::shared_ptr<InternalMap>
    constructDownscaled( const std::shared_ptr<InternalMap> &source, const double scale_factor );

    void internalSerialize( boost::filesystem::ofstream &of );

    void clear();

    measurement_table expected_meas_;
    measurement_table observed_meas_;

    std::shared_ptr<InternalMap> map_;
    std::shared_ptr<InternalMap> render_map_;
    LaserMeasurementPtr laser_meas_;
    bool render_;
    nav_msgs::OccupancyGrid map_msg_;
    cv::Mat raytrace_image_dbg_;
    bool has_result_;
    bool filesys_force_override_;
    bool continuous_outstream_;

    geometry_msgs::Pose laser_pose_;
  };

  using SensorModelEvaluatorPtr = std::shared_ptr<SensorModelEvaluator>;

}

#endif //PROJECT_SENSORMODELEVALUATOR_H
