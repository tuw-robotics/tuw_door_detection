//
// Created by felix on 28.02.19.
//

#include <sensor_model_evaluator.h>
#include <opencv2/highgui.hpp>

using namespace tuw;

SensorModelEvaluator::SensorModelEvaluator( const nav_msgs::OccupancyGridConstPtr &map )
{
  if ( !convert( map, map_ ))
  {
    ROS_ERROR( "ERROR CONVERTING MAP TO CV" );
  }
  map_info_ = map->info;
  ROS_INFO( "constructor passed" );
}

bool SensorModelEvaluator::convert( const nav_msgs::OccupancyGridConstPtr &src, cv::Mat &des )
{
  ROS_INFO( "convert" );
  ROS_INFO( "map dim %d, %d\n", (int) src->info.width, (int) src->info.height );
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
  float xmin = src->info.origin.position.x;
  float ymin = src->info.origin.position.y;
  float xmax = xmin + src->info.width * src->info.resolution;
  float ymax = ymin + src->info.height * src->info.resolution;
  
  ROS_INFO( "ymax %lf, ymin %lf, xmax %lf xmin %lf", ymax, ymin, xmax, xmin );
  
  des = cv::Mat::zeros( cv::Size( src->info.width, src->info.height ), CV_8U );
  // printf("--------convert:  %i x %i, %4.3f, %4.3f, %4.3f, %4.3f,
  // r:%4.3f\n",des.getSizeX(), des.getSizeY(), des.getXMin(), des.getXMax(),
  // des.getYMin(), des.getYMax(), des.getResolution());
  
  /// I hope the data is allways aligned
  for ( unsigned int h = 0; h < src->info.height; h++ )
  {
    cv::Mat row_des = des.row( h );
    const int8_t *pSrc = &src->data[h * src->info.width];
    for ( unsigned int w = 0; w < src->info.width; w++ )
    {
      row_des.at<int8_t>( w ) = *pSrc++;
    }
  }
  
  //cv::Mat downscaled;
  //cv::resize( des, downscaled, cv::Size( src->info.width / 4.0, src->info.height / 4.0 ));
  //cv::imshow( "map", downscaled );
  //cv::waitKey( 0 );
  
  return true;
}

void SensorModelEvaluator::clear()
{
  expected_meas_.clear();
  observed_meas_.clear();
}

void SensorModelEvaluator::downscaleImshow( const cv::Mat &des )
{
  cv::Mat downscaled;
  cv::resize( des, downscaled, cv::Size( map_info_.width / 4.0, map_info_.height / 4.0 ));
  cv::imshow( "map", downscaled );
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
      cv::circle( map_, intersection->cv(), 2.0, cv::Scalar( 0, 0, 255 ), 2 );
    } else
    {
      intersect++;
    }
  }
  
  std::cout << "no intersections " << intersect << "/" << scan->size() << std::endl;
  
  downscaleImshow( map_ );
}

Point2DPtr SensorModelEvaluator::rayTrace( const double scale, const Beam &b, const Eigen::Matrix4d &tf_ML )
{
  cv::Point2d dir_beam = b.get_direction_vector<cv::Point2d>( tf_ML );
  auto origin_view = tf_ML.topRightCorner<2, 1>( 0, 3 );
  cv::Point2d origin = cv::Point2d( origin_view( 0 ), origin_view( 1 ));
  cv::LineIterator ray_tracer( map_, origin + b.end_point.cv(), dir_beam * scale );
  
  for ( int i = 0; i < ray_tracer.count; ++i, ++ray_tracer )
  {
    if ( map_.at<uint8_t>( ray_tracer.pos()) == 0 )
    {
      return std::make_shared<Point2D>( ray_tracer.pos());
    }
  }
  return nullptr;
}
