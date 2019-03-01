//
// Created by felix on 01.03.19.
//
#include <sensor_model_evaluator_node.h>

using namespace tuw;

SensorModelEvaluatorNode::SensorModelEvaluatorNode( ros::NodeHandle &nh ) : evaluator_( nullptr ),
                                                                            tf_listener_( tf_buffer_ )
{
  nh_ = nh;
  sub_laser_ = nh.subscribe( "/r0/laser0/scan/raw", 1000, &SensorModelEvaluatorNode::callbackLaser, this );
  sub_map_ = nh.subscribe( "/map", 1, &SensorModelEvaluatorNode::callbackMap, this );
}

void SensorModelEvaluatorNode::callbackMap( const nav_msgs::OccupancyGridConstPtr &map )
{
  if ( !evaluator_ )
  {
    evaluator_.reset( new SensorModelEvaluator( map ));
  }
}

void SensorModelEvaluatorNode::callbackLaser( const sensor_msgs::LaserScan &_laser )
{
  
  if ( evaluator_ )
  {
    
    geometry_msgs::TransformStampedPtr tf = boost::make_shared<geometry_msgs::TransformStamped>();
    tf->transform.translation.x = 0;
    tf->transform.translation.y = 0;
    tf->transform.translation.z = 0;
    
    tf->transform.rotation.x = 0;
    tf->transform.rotation.y = 0;
    tf->transform.rotation.z = 0;
    tf->transform.rotation.w = 1;
    
    laser_measurement_.reset( new LaserMeasurement( tf ));
    laser_measurement_->initFromScan( _laser );
    
    evaluator_->evaluate( laser_measurement_ );
    
  }
  
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "sensor_model_evaluator_node" );
  
  ros::NodeHandle nh( "" );
  SensorModelEvaluatorNode eval_node( nh );
  
  while ( ros::ok())
  {
    ros::spin();
  }
  return 0;
}
