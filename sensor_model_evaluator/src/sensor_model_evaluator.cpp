//
// Created by felix on 28.02.19.
//

#include <sensor_model_evaluator.h>

using namespace tuw;

SensorModelEvaluator::SensorModelEvaluator( const nav_msgs::OccupancyGrid &map )
{
  if ( !convert( map, map_ ))
  {
    ROS_ERROR( "ERROR CONVERTING MAP TO CV" );
  }
}

bool SensorModelEvaluator::convert( const nav_msgs::OccupancyGrid &src, cv::Mat &des )
{
  
  if ((src.info.origin.orientation.x != 0) ||
      (src.info.origin.orientation.y != 0) ||
      (src.info.origin.orientation.z != 0) ||
      (src.info.origin.orientation.w != 1))
  {
    ROS_ERROR( "UNSUPPORTED CONVERSION: Rotated map given in SensorModelEvaluator::convert()" );
    return false;
  }
  float xmin = src.info.origin.position.x;
  float ymin = src.info.origin.position.y;
  float xmax = xmin + src.info.width * src.info.resolution;
  float ymax = ymin + src.info.height * src.info.resolution;
  
  des = cv::Mat::zeros( cv::Size( ymax - ymin, xmax - xmin ), CV_8UC3 );
  // printf("--------convert:  %i x %i, %4.3f, %4.3f, %4.3f, %4.3f,
  // r:%4.3f\n",des.getSizeX(), des.getSizeY(), des.getXMin(), des.getXMax(),
  // des.getYMin(), des.getYMax(), des.getResolution());
  
  /// I hope the data is allways aligned
  for ( unsigned int h = 0; h < src.info.height; h++ )
  {
    cv::Mat row_des = des.row( h );
    const int8_t *pSrc = &src.data[h * src.info.width];
    for ( unsigned int w = 0; w < src.info.width; w++ )
    {
      row_des.at<int8_t>( w ) = *pSrc++;
    }
  }
  return true;
}

void SensorModelEvaluator::evaluate( LaserMeasurementPtr &scan )
{
  Eigen::Matrix4d tf_ML = scan->getTfWorldSensor();
  
  for ( std::vector<Beam>::const_iterator beam_it = scan->begin();
        beam_it != scan->end();
        ++beam_it )
  {
    Beam beam = *beam_it;
    rayTrace( scan->getLaser().range_max, beam, tf_ML );
  }
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
      ROS_INFO( "HIT" );
      return std::make_shared<Point2D>( ray_tracer.pos());
    }
  }
  return nullptr;
}
