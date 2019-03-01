//
// Created by felix on 01.03.19.
//
#include <sensor_model_evaluator_node.h>

using namespace tuw;

SensorModelEvaluatorNode::SensorModelEvaluatorNode( ros::NodeHandle &nh ) : evaluator_( nullptr ),
                                                                            tf_listener_( tf_buffer_ )
{

}

void SensorModelEvaluatorNode::callbackLaser( const sensor_msgs::LaserScan &_laser )
{
  
  geometry_msgs::TransformStampedPtr tf;
  tf->transform.translation.x = 0;
  tf->transform.translation.y = 0;
  tf->transform.translation.z = 0;
  
  tf->transform.rotation.x = 0;
  tf->transform.rotation.y = 0;
  tf->transform.rotation.z = 0;
  tf->transform.rotation.w = 1;
  
  laser_measurement_.reset( new LaserMeasurement( tf ));
  laser_measurement_->initFromScan( _laser );
  
  evaluator_.reset( new SensorModelEvaluator( resp_map_.map ));
  evaluator_->evaluate( laser_measurement_ );
  
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "door_2d_detector_node" );
  
  ros::NodeHandle nh( "" );
  SensorModelEvaluatorNode eval_node( nh );
  
  while ( ros::ok())
  {
    ros::spin();
  }
  return 0;
}
