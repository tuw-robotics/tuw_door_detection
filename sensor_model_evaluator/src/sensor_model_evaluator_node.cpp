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
    
    try
    {
      
      std::cout << "header " << _laser.header.frame_id.c_str() << std::endl;
      
      geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
          "map", "r0/laser0", ros::Time( 0 ));
      
      
      geometry_msgs::TransformStampedPtr tf = boost::make_shared<geometry_msgs::TransformStamped>( stamped_tf );
      laser_measurement_.reset( new LaserMeasurement( tf ));
      laser_measurement_->initFromScan( _laser );
      
      evaluator_->evaluate( laser_measurement_ );
      
    } catch (tf2::TransformException &ex)
    {
      ROS_WARN( "transform lookup failed SensorModelEvaluatorNode::callbackLaser\n" );
      ROS_WARN( "what(): %s\n", ex.what());
    }
    
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
