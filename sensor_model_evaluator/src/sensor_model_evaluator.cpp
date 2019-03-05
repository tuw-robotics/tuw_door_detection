//
// Created by felix on 28.02.19.
//

#include <sensor_model_evaluator.h>
#include <opencv2/highgui.hpp>

using namespace tuw;

SensorModelEvaluator::SensorModelEvaluator( const nav_msgs::OccupancyGridConstPtr &map )
{
  map_msg_ = nav_msgs::OccupancyGrid( *map );
  if ( !convert( map, map_ ))
  {
    ROS_ERROR( "ERROR CONVERTING MAP TO CV" );
  }
}

bool SensorModelEvaluator::convert( const nav_msgs::OccupancyGridConstPtr &src, std::unique_ptr<InternalMap> &des )
{
  des = std::make_unique<InternalMap>();
  ROS_INFO( "convert" );
  ROS_INFO( "map dim %d, %d\norigin: (x=%lf,y=%lf,z=%lf)",
            (int) src->info.width, (int) src->info.height,
            src->info.origin.position.x,
            src->info.origin.position.y,
            src->info.origin.position.z );
  
  if ((src->info.origin.orientation.x != 0) ||
      (src->info.origin.orientation.y != 0) ||
      (src->info.origin.orientation.z != 0) ||
      (src->info.origin.orientation.w != 1))
  {
    ROS_WARN( "UNSUPPORTED CONVERSION: Rotated map given in SensorModelEvaluator::convert()" );
    ROS_WARN( "(x,y,z,w) = (%lf,%lf,%lf,%lf)", src->info.origin.orientation.x,
              src->info.origin.orientation.y,
              src->info.origin.orientation.z,
              src->info.origin.orientation.w );
    return false;
  }
  
  des->cv_ui8 = cv::Mat::zeros( cv::Size( src->info.width, src->info.height ), CV_8U );
  des->size_x = src->info.width;
  des->size_y = src->info.height;
  des->scale = src->info.resolution;
  des->origin_x = src->info.origin.position.x + ((des->size_x / 2.0) * des->scale);
  des->origin_y = src->info.origin.position.y + ((des->size_y / 2.0) * des->scale);
  des->map_info_ = src->info;
  
  // Convert to player format
  memcpy( des->cv_ui8.data, src->data.data(), static_cast<std::size_t>(des->size_x * des->size_y));
  
  std::cout << "map init " << std::endl;
  std::cout << des->to_string() << std::endl;
  std::cout << "origin in image space " << std::endl;
  std::cout << des->get_mx_from_wx( des->origin_x ) << std::endl;
  std::cout << des->get_my_from_wy( des->origin_y ) << std::endl;
  
  //for ( unsigned int h = 0; h < src->info.height; h++ )
  //{
  //  cv::Mat row_des = des.row( h );
  //  const int8_t *pSrc = &src->data[h * src->info.width];
  //  for ( unsigned int w = 0; w < src->info.width; w++ )
  //  {
  //    row_des.at<int8_t>( w ) = *pSrc++;
  //  }
  //}
  
  return true;
}

void SensorModelEvaluator::clear()
{
  expected_meas_.clear();
  observed_meas_.clear();
}

void SensorModelEvaluator::downscaleImshow( const std::unique_ptr<InternalMap> &map, LaserMeasurementPtr meas )
{
  float scale_factor = 4.0;
  
  std::cout << "map init " << std::endl;
  std::cout << map->to_string() << std::endl;
  std::cout << "origin in image space " << std::endl;
  std::cout << map->get_mx_from_wx( map->origin_x ) << std::endl;
  std::cout << map->get_my_from_wy( map->origin_y ) << std::endl;
  
  InternalMap downscaled;
  map->cv_ui8.copyTo( downscaled.cv_ui8 );
  downscaled.scale = map->scale * scale_factor;
  downscaled.size_x = map->size_x / scale_factor;
  downscaled.size_y = map->size_y / scale_factor;
  downscaled.map_info_ = map->map_info_;
  
  downscaled.origin_x = downscaled.map_info_.origin.position.x + (downscaled.size_x / 2) * downscaled.scale;
  downscaled.origin_y = downscaled.map_info_.origin.position.y + (downscaled.size_y / 2) * downscaled.scale;
  
  cv::resize( downscaled.cv_ui8, downscaled.cv_ui8, cv::Size( downscaled.size_x, downscaled.size_y ));
  cv::cvtColor( downscaled.cv_ui8, downscaled.cv_ui8, CV_GRAY2BGR );
  
  cv::Point2d origin_img( downscaled.get_mx_from_wx( downscaled.origin_x ),
                          downscaled.get_my_from_wy( downscaled.origin_y ));
  
  std::cout << "o img: (" << origin_img.x << ", " << origin_img.y << ")" << std::endl;
  cv::circle( downscaled.cv_ui8, origin_img, 5, cv::Scalar( 255, 255, 0 ), 3 );
  
  std::cout << "downscaled version" << std::endl;
  std::cout << downscaled.to_string() << std::endl;
  
  if ( meas )
  {
    Eigen::Matrix4d tf = meas->getTfWorldSensor();
    for ( auto b_it = meas->begin();
          b_it != meas->end();
          ++b_it )
    {
      auto p_cv = b_it->end_point.cv();
      Eigen::Vector4d ws_p_b = tf * Eigen::Vector4d( b_it->end_point.x(), b_it->end_point.y(), 0, 1 );
      ws_p_b = ws_p_b / ws_p_b[3];
      
      cv::Point2d i_p_b( downscaled.get_mx_from_wx( ws_p_b( 0 )),
                         downscaled.get_my_from_wy( ws_p_b( 1 )));
      
      cv::circle( downscaled.cv_ui8, i_p_b, 2, cv::Scalar( 255, 0, 0 ), 2 );
    }
  }
  
  cv::imshow( "map", downscaled.cv_ui8 );
  cv::waitKey( 5 );
}

void SensorModelEvaluator::evaluate( LaserMeasurementPtr &scan )
{
  clear();
  
  Eigen::Matrix4d tf_ML = scan->getTfWorldSensor();
  uint32_t intersect = 0;
  
  for ( std::vector<Beam>::const_iterator beam_it = scan->begin();
        beam_it != scan->end();
        ++beam_it )
  {
    Beam beam = *beam_it;
    Point2DPtr intersection = rayTrace( scan->getLaser().range_max, beam, tf_ML );
    if ( intersection )
    {
      cv::circle( map_->cv_ui8, intersection->cv(), 2.0, cv::Scalar( 0, 0, 255 ), 2 );
    } else
    {
      intersect++;
    }
  }
  
  std::cout << "no intersections " << intersect << "/" << scan->size() << std::endl;
  downscaleImshow( map_, scan );
}

Point2DPtr SensorModelEvaluator::rayTrace( const double scale, const Beam &b, const Eigen::Matrix4d &tf_ML )
{
  cv::Point2d dir_beam = b.get_direction_vector<cv::Point2d>( tf_ML );
  auto origin_view = tf_ML.topRightCorner<2, 1>( 0, 3 );
  cv::Point2d origin = cv::Point2d( origin_view( 0 ), origin_view( 1 ));
  cv::LineIterator ray_tracer( map_->cv_ui8, origin + b.end_point.cv(), dir_beam * scale );
  
  for ( int i = 0; i < ray_tracer.count; ++i, ++ray_tracer )
  {
    if ( map_->cv_ui8.at<uint8_t>( ray_tracer.pos()) == 0 )
    {
      return std::make_shared<Point2D>( ray_tracer.pos());
    }
  }
  return nullptr;
}
